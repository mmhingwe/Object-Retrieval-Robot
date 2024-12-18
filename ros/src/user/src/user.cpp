#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <csignal>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "messages/msg/userinput.hpp"


class user_input : public rclcpp::Node
{
  public:
    user_input()
    : Node("user"){
        publish_user = this->create_publisher<::messages::msg::Userinput>("user_input", 10); 
        user_input_thread = std::make_unique<std::thread>(&user_input::take_user_input, this);
    }

    // Destructor
    ~user_input(){
        exit_input_loop = true;
        user_input_thread->~thread();
    }



  private:

    // initialize pointers
    rclcpp::Publisher<::messages::msg::Userinput>::SharedPtr publish_user;

    //Flags for user input thread
    bool exit_input_loop = false;
    std::unique_ptr<std::thread> user_input_thread;

    void take_user_input(){
        while (exit_input_loop == false){
            std::cout << "Please Enter Command" << std::endl;
            int command_int;
            std::vector<double> input_buf;
            std::string input;
            std::string xml_file_path = "/home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/alg_lib/universal_robots_ur10e/scene.xml";
            std::getline(std::cin,input);
            std::cout << "Command entered: " << input << std::endl;

            if (input == "start"){
              command_int = 1;
            }
            else if(input == "stop"){
              command_int = 0;
            }
            else if(input == "move"){
              command_int = 2;
              std::cout << "Move where?" << std::endl;
              std::string location;
              std::getline(std::cin,location);
              std::stringstream ss(location);
              while(ss >> location){
                input_buf.push_back(std::stod(location));
              }
            }
            else{
              std::cout << "Unfamiliar command, please try again." << input << std::endl;
              command_int = -1;
            }

            // Send user message to mujoco node
            auto msg = messages::msg::Userinput();
            msg.command = command_int;
            msg.xml_file = xml_file_path;
            if (command_int == 2){
              msg.data = input_buf;
            }
            publish_user->publish(msg);

        }

    }


};

// user_input* node_instance = nullptr;

// void signalHandler(int signum){
//   // RCLCPP_INFO("Interrupt signal has been received. Now shutting down MuJoCo Node.")
//   // std::cout << "Signal handler called" << std::endl;
//   // if (node_instance != nullptr){
//   //   node_instance->~user_input();
//   // }
//   rclcpp::shutdown();
// }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // auto node = std::make_shared<user_input>();
  // node_instance = node.get();
  // std::signal(SIGINT, signalHandler); 
  rclcpp::spin(std::make_shared<user_input>());
  rclcpp::shutdown();
  // node_instance = nullptr;
  return 0;
}