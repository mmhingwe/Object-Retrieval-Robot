cmake_minimum_required(VERSION 3.8)
project(messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#Packages used by the messages and srvs
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Userinput.msg"
  "msg/Ctrlmsg.msg"
  "msg/Sensormsg.msg"
  "srv/Ctrlsrv.srv"
  #DEPENDENCIES std_msgs 
)


# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




ament_package()
