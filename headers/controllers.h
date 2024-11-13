#pragma once 

#include <iostream>
#include <string>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include "datastructures/node.h"
#include "datastructures/kdtree.h"
#include "robotmodel.h"
#include "pathplan.h"

// Takes the robot model and some initial options (such as the type of path planner) as the initializer input
// Calculate trajectory take the initial and final positions in joint space.
// Use path planning algorithm to generate a path.
// TODO: Add option to input the initial and goal positions in task space and use IK to convert the points.
// Creates a robot model using the input xml file.
// The get_ctrl(); function then takes the current position of the robot, converts it to joint space, finds it on the rrt tree, then uses unified pid to output the necessary torque which is "u"
// TODO: Add torque limits.
class PID {

private:
    
    // Search kd Tree of the calculated path for the nearest vector.
    std::pair<Eigen::VectorXd,int> find_next_point(Eigen::VectorXd init);


    // TODO: Implement these functions to handle contact forces.
    void update_constraints();
    void update_path();

    // Expects input to be the current positions and velocities of the robot. When not using mujoco, the input data will be taken from sensors.
    void find_path(Eigen::VectorXd init, Eigen::VectorXd goal);

    Eigen::MatrixXd constraints;
    Eigen::MatrixXd space_bounds;
    int config_space;

    pathplan* planner; 
    robotModel* model;
    kdTree kdtree;

    // PID Control variables
    double kp;
    double kd;
    double ki;
    Eigen::VectorXd cum_sum;

    Eigen::MatrixXd saved_path;
    bool path_found;
    std::vector<int> associated_joint_bodies;

public:

    // Constructor for mujoco simulation (the sensor data is received from the model and data.)
    PID(mjModel* m, mjData* d,std::string planner_type);

    //TODO: Add a constructor to take in real sensor data to get the state of the robot. This will come later. First just take in mujoco data.
    PID(std::string xml_filename,std::string planner_type);
    
    ~PID();
    void reset();

    void set_pid(double kp,double kd,double ki);

    // Get control for mujoco environment
    // If no path is created, take the current position from mjdata as initial, if path created, perform search, if contact forces detected, calculated new trajectory.
    Eigen::VectorXd get_control(mjModel* m, mjData* d, Eigen::VectorXd goal);


};
