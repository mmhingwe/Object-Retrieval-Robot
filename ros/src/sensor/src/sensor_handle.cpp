#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <csignal>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "messages/msg/userinput.hpp"
#include "messages/srv/ctrlsrv.hpp"
#include "messages/msg/sensormsg.hpp"

using namespace std;

class Sensor : public rclcpp::Node
{
  public:
    Sensor()
    : Node("sensor")
    {

      // publish_file = this->create_publisher<std_msgs::msg::String>("control_file", 10); // TODO: Need to change the message type of the control.
      // subscribe_user = this->create_subscription<::messages::msg::Userinput>(
      //       "user_input", 10, std::bind(&MuJoCoControl::user_callback, this, std::placeholders::_1)); // binds the usercallback function defined bel

      this->subscribe_mujoco_file = this->create_subscription<std_msgs::msg::String>(
            "control_file", 10, std::bind(&Sensor::callback, this, std::placeholders::_1));

      this->publish_constraints = this->create_publisher<::messages::msg::Sensormsg>("detected_constraints", 10);


    }

    // Destructor
    ~Sensor()
    { 
     

    }

  private:

    // Publisher and subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe_mujoco_file;
    rclcpp::Publisher<::messages::msg::Sensormsg>::SharedPtr publish_constraints;

    // Mjmodel
    mjModel* model;
    mjData* data;

    void callback(const std_msgs::msg::String & filename){

      char error[1000] = "Could not load model";
      model = mj_loadXML(filename.data.c_str(), NULL, error, 1000);
      if (!model) {
          mju_error_s("Loading model error: %s", error);
      }
      data = mj_makeData(model);

      cout << "Model created successfully" << endl;

      // string obs_name = "box1";
      // // Find the obtacles and send data to the controller
      // int box_id = mj_name2id(this->model,mjOBJ_BODY,obs_name.c_str());
      
      // Eigen::VectorXd(3) box_pos;
      // box_pos(0) = this->model->body_pos[box_id*3];
      // box_pos(1) = this->model->body_pos[box_id*3+1];
      // box_pos(2) = this->model->body_pos[box_id*3+2];

      // Eigen::VectorXd(4) box_quat;
      // box_quat(0) = this->model->body_quat[box_id*3]; 
      // box_quat(1) = this->model->body_quat[box_id*3+1]; 
      // box_quat(2) = this->model->body_quat[box_id*3+2]; 
      // box_quat(3) = this->model->body_quat[box_id*3+3];

      // //TODO: right now the size must be manually set. Change this
      // double size = 0.2;

      // Eigen::MatrixXd constraints(6,3) = Eigen::MatrixXd::Zero(6,3);

      


    }


};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Sensor>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // rclcpp::spin(std::make_shared<MuJoCoControl>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}