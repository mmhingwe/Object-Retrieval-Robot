cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Object-Retrieval-Robot VERSION 1.0)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
# find_package (Eigen3 3.4 REQUIRED NO_MODULE)
# find_package(adloc REQUIRED)
    
# Add external dependency paths
set(MUJOCO_PATH "~/codebases/mujoco_build")
set(EIGEN_PATH "~/codebases/eigen")
set(MATPLOTLIB_PATH "~/codebases/matplotlib-cpp-master")
set(NLOPT_PATH "/usr/local/include")
# set(MUJOCO_PATH_LIB "~/codebases/mujoco/build/lib")
# find_package(mujoco REQUIRED PATHS ${MUJOCO_PATH})

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE H_FILES headers/*.h)

# Create library to be used by the ros package
add_library(CtrlLibrary
    ${SRC_FILES}
    # ${H_FILES}
)

set_target_properties(CtrlLibrary
    PROPERTIES
    CMAKE_CXX_STANDARD 17
    CMAKE_CXX_STANDARD_REQUIRED ON
    CMAKE_CXX_EXTENSIONS OFF
)

include(GNUInstallDirs)

target_include_directories(CtrlLibrary
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/headers>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(
    CtrlLibrary 
    /usr/local/lib/libnlopt.so
    /usr/local/lib/libnlopt.so.0
)

install(TARGETS CtrlLibrary
    EXPORT CtrlLibrary
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${H_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT CtrlLibrary
    FILE CtrlLibrary.cmake 
    NAMESPACE ctrl::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CtrlLibrary
)

# Create simulation test executable
add_executable(simulate simulate.cpp ${SRC_FILES})
include_directories(include ${MATPLOTLIB_PATH})
include_directories(include ${MUJOCO_PATH}/include)
include_directories(include ${EIGEN_PATH})
include_directories(include ${NLOPT_PATH})
include_directories(include "universal_robots_ur10e/")
target_include_directories(simulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers)

target_link_libraries(
simulate
${MUJOCO_PATH}/lib/libmujoco.so
# ${MUJOCO_PATH}/lib/libmujoco.so.3.2.0
glfw
Eigen3::Eigen
python3.10 # Link python 3.10 for matplotlib
/usr/local/lib/libnlopt.so
/usr/local/lib/libnlopt.so.0
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
/usr/local/lib/libnlopt.so
/usr/local/lib/libnlopt.so.0
)

# Robot model class test executable
add_executable(test_robot_model test_robot_tree.cpp ${SRC_FILES})
include_directories(include ${MATPLOTLIB_PATH})
include_directories(include /usr/include/python3.10)
target_include_directories(test_robot_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers)
target_link_libraries(
test_robot_model
${MUJOCO_PATH}/lib/libmujoco.so
# ${MUJOCO_PATH}/lib/libmujoco.so.3.2.0
glfw
Eigen3::Eigen
/usr/local/lib/libnlopt.so
/usr/local/lib/libnlopt.so.0
)