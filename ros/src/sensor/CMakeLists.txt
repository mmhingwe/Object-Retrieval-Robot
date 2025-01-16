cmake_minimum_required(VERSION 3.8)
project(sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Set C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

#include MuJoCo Libraries
# find_package(MuJoCo REQUIRED)
set(MUJOCO_PATH "~/codebases/mujoco-3.1.6")
include_directories(include ${MUJOCO_PATH}/include)

# Add MujoCo dependencies
add_executable(sensor_handle src/sensor_handle.cpp)
ament_target_dependencies(sensor_handle rclcpp std_msgs messages)
target_link_libraries(
  sensor_handle
  Eigen3::Eigen
  ${MUJOCO_PATH}/bin/mujoco_plugin/libactuator.so
  ${MUJOCO_PATH}/bin/mujoco_plugin/libelasticity.so
  ${MUJOCO_PATH}/bin/mujoco_plugin/libsdf.so
  ${MUJOCO_PATH}/bin/mujoco_plugin/libsensor.so
  ${MUJOCO_PATH}/lib/libmujoco.so
  # ${MUJOCO_PATH}/lib/libmujoco.so.3.1.6
  python3.10
  glfw
)

# set_target_properties(start_mujoco PROPERTIES
#     INSTALL_RPATH_USE_LINK_PATH TRUE
#     INSTALL_RPATH ${MUJOCO_PATH}/lib/libmujoco.so.3.1.6
# )


install(TARGETS
  sensor_handle
  DESTINATION lib/${PROJECT_NAME})





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
