#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <csignal>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "messages/msg/userinput.hpp"
#include "messages/srv/ctrlsrv.hpp"

// using namespace std::chrono_literals;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

void signalHandler(int signum){
  // RCLCPP_INFO("Interrupt signal has been received. Now shutting down MuJoCo Node.")
  rclcpp::shutdown();
}

class MuJoCoControl : public rclcpp::Node
{
  public:
    MuJoCoControl()
    : Node("mujoco")
    {

      start_flag = 0;
      model_filename = "/home/mhingwe/codebases/tennis_trajectory_extraction-main/mujoco/robots/omni.xml"; // This is just the default world for testing purposes.
      model = nullptr;
      data = nullptr;
      threadFlag = false;

      // Create user and control nodes.
      publish_file = this->create_publisher<std_msgs::msg::String>("control_file", 10); // TODO: Need to change the message type of the control.
      subscribe_user = this->create_subscription<::messages::msg::Userinput>(
            "user_input", 10, std::bind(&MuJoCoControl::user_callback, this, std::placeholders::_1)); // binds the usercallback function defined below to the subscriber.

      // Client to get ctrl from the ctrl service 
      ctrl_client = this->create_client<::messages::srv::Ctrlsrv>("get_ctrl");
    
    }

    // Destructor
    ~MuJoCoControl()
    { 
      if(model != nullptr){
        mj_deleteModel(model); 
      }
      if(data != nullptr){
        mj_deleteData(data);
      }
    }

  private:

    // ROS2 Publish and Subscriber init
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_file; // TODO: Need to change the message type based on the info sent to the control node.
    rclcpp::Subscription<::messages::msg::Userinput>::SharedPtr subscribe_user;
    rclcpp::Client<::messages::srv::Ctrlsrv>::SharedPtr ctrl_client;

    // MuJoCo world and state pointers
    bool start_flag;
    string model_filename;
    mjModel* model;
    mjData* data;

    // User input goal state for simulation
    vector<double> goal_state;

    // Variables to control threat for simulation window
    thread simulation_thread;
    atomic<bool> threadFlag;

    // Simulates the mujoco environment as a seperate thread
    void simulate(){

      // Initialize the mujoco data structures
      mjvCamera cam;                      // abstract camera
      mjvOption opt;                      // visualization options
      mjvScene scn;                       // abstract scene
      mjrContext con;                     // custom GPU context
      mjvPerturb pert;

      // init GLFW, create window, make OpenGL context current, request v-sync
      glfwInit();
      GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
      glfwMakeContextCurrent(window);
      glfwSwapInterval(1);

      // initialize visualization data structures
      mjv_defaultCamera(&cam);
      mjv_defaultPerturb(&pert);
      mjv_defaultOption(&opt);
      mjr_defaultContext(&con);

      // create scene and context
      mjv_makeScene(this->model, &scn, 1000);
      mjr_makeContext(this->model, &con, mjFONTSCALE_100);



      // auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      // request->a = atoll(argv[1]);
      // request->b = atoll(argv[2]);

      // while (!client->wait_for_service(1s)) {
      //   if (!rclcpp::ok()) {
      //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      //     return 0;
      //   }
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      // }

      // auto result = client->async_send_request(request);
      // // Wait for the result.
      // if (rclcpp::spin_until_future_complete(node, result) ==
      //   rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      // } else {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      // }
      // this->ctrl_client


      while( !glfwWindowShouldClose(window) ) {
      // advance interactive simulation for 1/60 sec
      //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
      //  this loop will finish on time for the next frame to be rendered at 60 fps.
      //  Otherwise add a cpu timer and exit this loop when it is time to render.
          mjtNum simstart = this->data->time;

          //ctrl is double*
          auto request = std::make_shared<::messages::srv::Ctrlsrv::Request>();
          request->type = "MPC";
          request->time = 10.0;
          request->samples = 1.0;
          request->goal = this->goal_state;
          //caclualte current state 
          // Eigen::VectorXd init(this->model->njnt*2);
          // init = Eigen::VectorXd::Zero(this->model->njnt*2);
          vector<double> curr_state(this->model->njnt,0);
          for (int i =0; i < this->model->njnt; i++){
              curr_state[i] = this->data->qpos[i];
              curr_state[this->model->njnt+i] = this->data->qvel[i];
          }
          request->state = curr_state;

          while (!this->ctrl_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
          }

          auto result = ctrl_client->async_send_request(request);
          // Wait for the result.
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got it");
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          }

          // FIXME: THIS CTRL ONLY WORKS FOR THE TWO JOINT SYSTEM.
          vector<double> out = result.get()->ctrl;

          this->data->ctrl[0] = out[0];
          this->data->ctrl[1] = out[1];

          while( this->data->time - simstart < 1.0/60.0 )
              mj_step(this->model, this->data);

          // get framebuffer viewport
          mjrRect viewport = {0, 0, 0, 0};
          glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
          mjv_updateScene(this->model, this->data, &opt, NULL, &cam, mjCAT_ALL, &scn);
          mjr_render(viewport, &scn, &con);

          // swap OpenGL buffers (blocking call due to v-sync)
          glfwSwapBuffers(window);

          // process pending GUI events, call GLFW callbacks
          glfwPollEvents();

          if (this->threadFlag == false){
            break;
          }

      }

      // close GLFW, free visualization storage
      glfwTerminate();
      mjv_freeScene(&scn);
      mjr_freeContext(&con);


    }


    // Handles the users feelback from the user_input node
    void user_callback(const ::messages::msg::Userinput & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received: '%li'", msg.command);
      RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.xml_file.c_str());
      
  
      if ((msg.command == 1) and (!threadFlag)){

        start_flag = 1;
        char error[1000] = "Could not load model";
        model = mj_loadXML(msg.xml_file.c_str(), NULL, error, 1000);
        if (!model) {
            mju_error_s("Loading model error: %s", error);
        }
        data = mj_makeData(model);

        cout << "Model created Successfully" << endl;
        
        // Start simulation in another thread
        threadFlag = true;
        this->simulation_thread = thread(&MuJoCoControl::simulate, this);
        cout << "thread started" << endl;

        //Pass the file to the control node
        auto pub_msg = std_msgs::msg::String();
        pub_msg.data = msg.xml_file;
        this->publish_file->publish(pub_msg);

      }
      else if (msg.command == 0){
        if (start_flag == 1){
          
          // Destroying simulation thread
          this->threadFlag = false;
          if (this->simulation_thread.joinable()){
            this->simulation_thread.join();
          }
          cout << "Simulation thread stopped" << endl;
          
          start_flag = 0;
          mj_deleteModel(model); 
          model = nullptr;
          mj_deleteData(data);
          data = nullptr;
          cout << "Simulation aborted" << endl;
        }
      }
      else if (msg.command == 2){

        vector<double> data = msg.data;
        cout << "We must move to:  (";
        
        for (int i = 0; i < data.size(); i++){
          cout << data[i] << " ";
        }
        cout << ")" << endl;

        this->goal_state = msg.data;

      }
      else{
        cout << "Received unfamiliar command." << endl;
      }

    }


};

int main(int argc, char * argv[])
{
  std::signal(SIGINT, signalHandler); //Handle ctrl + c
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuJoCoControl>());
  rclcpp::shutdown();
  return 0;
}