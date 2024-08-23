#pragma once 

#include <iostream>
#include <string>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include "node.h"
#include "robotmodel.h"
#include "pathplan.h"

// Takes the robot model and some initial options (such as the type of path planner) as the initializer input
// Calculate trajectory take the initial and final positions in task space and converts it to joint space and finds the path using the planner.
// Converts initial and final points into joint space and uses path planning algorithm to generate a path.
// Creates a robot model using the input xml file.
// The get_ctrl(); function then takes the current position of the robot, converts it to joint space, finds it on the rrt tree, then uses unified pid to output the necessary torque which is "u"
class PID {

private:
    
    //Essentially a n dimentional binary search of the path of the planner to find the closest point in the path matrix.
    int find_nearest_point();

    pathplan* planner; 
    robotModel model;

public:

    PID();
    ~PID();
    void find_path(Eigen::MatrixXd init, Eigen::MatrixXd goal);


};
