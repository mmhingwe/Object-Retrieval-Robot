cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Object-Retrieval-Robot VERSION 1.0)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

# Add external dependency paths
set(MUJOCO_PATH "~/codebases/mujoco_build")
set(EIGEN_PATH "~/codebases/eigen")
set(MATPLOTLIB_PATH "~/codebases/matplotlib-cpp-master")
# set(MUJOCO_PATH_LIB "~/codebases/mujoco/build/lib")
# find_package(mujoco REQUIRED PATHS ${MUJOCO_PATH})

file(GLOB_RECURSE SRC_FILES src/*.cpp)

# Create main executable
add_executable(simulate simulate.cpp ${SRC_FILES})
include_directories(include ${MUJOCO_PATH}/include)
include_directories(include ${EIGEN_PATH})
target_include_directories(simulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers)

target_link_libraries(
simulate
${MUJOCO_PATH}/lib/libmujoco.so
${MUJOCO_PATH}/lib/libmujoco.so.3.2.0
glfw
Eigen3::Eigen
)

# Pathplan class test executable
add_executable(test_path_plan test_pathplan.cpp ${SRC_FILES})
include_directories(include ${MATPLOTLIB_PATH})
include_directories(include /usr/include/python3.10)
target_include_directories(test_path_plan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers)
target_link_libraries(
test_path_plan
Eigen3::Eigen
python3.10 # Link python 3.10 for matplotlib
)