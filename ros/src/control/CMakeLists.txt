cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set path variables
set(EIGEN_PATH "~/codebases/eigen")
set(MATPLOTLIB_PATH "~/codebases/matplotlib-cpp-master")
set(NLOPT_PATH "/usr/local/include")
set(CTRL_PATH "/home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/alg_lib/headers")
set(MUJOCO_PATH "~/codebases/mujoco_build")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(messages REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(ctrl src/control.cpp)
include_directories(include ${MATPLOTLIB_PATH})
include_directories(include ${MUJOCO_PATH}/include)
include_directories(include ${EIGEN_PATH})
include_directories(include ${NLOPT_PATH})
include_directories(include ${CTRL_PATH})
target_include_directories(ctrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers) # Edit to allow use of libraries 
ament_target_dependencies(ctrl rclcpp std_msgs messages)
# ament_target_dependencies(ctrl /usr/local/lib/libnlopt.so)

set_target_properties(ctrl 
  PROPERTIES
  INSTALL_RPATH "~/codebases/mujoco_build/lib"
)

target_link_libraries(
  ctrl
  Eigen3::Eigen
  python3.10 # Link python 3.10 for matplotlib
  /home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/alg_lib/build/libCtrlLibrary.a
  /usr/local/lib/libnlopt.so
  /usr/local/lib/libnlopt.so.0
  ${MUJOCO_PATH}/lib/libmujoco.so.3.2.0
)

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

install(TARGETS
  ctrl
  DESTINATION lib/${PROJECT_NAME})