
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <controllers.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "messages/msg/userinput.hpp"
#include "messages/msg/ctrlmsg.hpp"
#include "messages/srv/ctrlsrv.hpp"

using namespace std;

class control_node : public rclcpp::Node
{
  public:
    control_node()
    : Node("ctrl"){
        // publish_user = this->create_publisher<::messages::msg::Ctrlmsg>("ctrl_output", 10); 
        subscribe_mujoco = this->create_subscription<std_msgs::msg::String>(
            "control_file", 10, std::bind(&control_node::create_model, this, std::placeholders::_1));

        // Service to return the ctrl to requesting node;
        ctrl_service = this->create_service<::messages::srv::Ctrlsrv>("get_ctrl", std::bind(&control_node::calculate_ctrl,this,std::placeholders::_1, std::placeholders::_2));

        // Initialize controllers to null
        this->pid_ctrl = nullptr;
        this->mpc_ctrl = nullptr;

    }

    // Destructor
    ~control_node(){

    }

  private:

    // rclcpp::Publisher<::messages::msg::Ctrlmsg>::SharedPtr publish_user;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe_mujoco;
    rclcpp::Service<::messages::srv::Ctrlsrv>::SharedPtr ctrl_service;  

    // Mujoco data and model data
    mjModel* model; 
    mjData* data;
    PID* pid_ctrl;
    MPC* mpc_ctrl; //TODO: For now, everything will be calculated using the MPC controller, In the future allow the user to chosse what tpye of controler

    // Sensor data
    vector<Eigen::MatrixXd> constraints;


    void calculate_ctrl(const std::shared_ptr<::messages::srv::Ctrlsrv::Request> request,
        std::shared_ptr<::messages::srv::Ctrlsrv::Response>   response){

        if (request->type == "PID"){

            if (this->pid_ctrl == nullptr){
                pid_ctrl = new PID(this->model,this->data,"rrt_base");
            }
            this->pid_ctrl->set_pid(request->kp,request->kd,request->ki);
            
            //convert the state and goal into eigen vectors
            vector<double> init = request->state;
            vector<double> goal = request->goal;

            Eigen::VectorXd init_input = Eigen::VectorXd::Zero(init.size());
            Eigen::VectorXd goal_input = Eigen::VectorXd::Zero(goal.size());

            for (int i = 0; i < init.size(); i++){
                init_input(i) = init[i];
                goal_input(i) = goal[i];
            }


            Eigen::VectorXd ctrl = this->pid_ctrl->get_control(init_input,goal_input);

            vector<double> out;

            for (int i = 0; i<ctrl.size();i++){
                out.push_back(ctrl(i));
            }


            response->ctrl = out;

        }
        else{

            if (this->mpc_ctrl == nullptr){
                mpc_ctrl = new MPC(this->model,this->data,"rrt_base");
            }
        
            cout << "MODEL CREATED" << endl;
            // cout << init.size() << " = " << goal.size() << endl;

            //convert the state and goal into eigen vectors
            vector<double> init = request->state;
            vector<double> goal = request->goal;

            Eigen::VectorXd init_input = Eigen::VectorXd::Zero(init.size());
            Eigen::VectorXd goal_input = Eigen::VectorXd::Zero(goal.size());
            cout << init.size() << " = " << goal.size() << endl;

            for (int i = 0; i < init.size(); i++){
                init_input(i) = init[i];
                goal_input(i) = goal[i];
            }

            cout << "calculating ctrl" << endl;

            cout << "Init: ( " ;
            for (int i = 0; i < init.size(); i++){

                cout << init[i] << " ";

            }
            cout << " )" << endl;

            cout << "Goal: ( " ;
            for (int i = 0; i < init.size(); i++){

                cout << goal[i] << " ";

            }
            cout << " )" << endl;

            //FIXME: input aquired constraints so it is recognized when the RRT planner is computing.
            Eigen::VectorXd ctrl = this->mpc_ctrl->get_control(init_input,goal_input,request->time, request->samples);
            cout << "ctrl calculated" << endl;


            vector<double> out;

            for (int i = 0; i<ctrl.size();i++){
                out.push_back(ctrl(i));
            }

            cout << "sending response " << endl;
            response->ctrl = out;

        }

    }

    void create_model(const std_msgs::msg::String & msg){
        cout << "File received: " << msg.data << endl;
        char error[1000] = "Could not load model";
        this->model = mj_loadXML(msg.data.c_str(), NULL, error, 1000);
        if (!this->model) {
            mju_error_s("Loading model error: %s", error);
        }
        this->data = mj_makeData(this->model);
        cout << "Model created Successfully" << endl;
    }


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
//   auto node = std::make_shared<control_node>();
  // node_instance = node.get();
  // std::signal(SIGINT, signalHandler); 
  rclcpp::spin(std::make_shared<control_node>());
  rclcpp::shutdown();
  // node_instance = nullptr;
  return 0;
}