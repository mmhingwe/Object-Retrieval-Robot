#pragma once 

#include <iostream>
#include <string>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include "datastructures/node.h"
#include "datastructures/kdtree.h"
#include "robotmodel.h"
#include "pathplan.h"
#include <nlopt.h>
// #include <psopt.h>
// #include <adolc/adolc_sparse.h>
// #include <adolc/adouble.h>
#include "datastructures/eigen_extentions.h"

class global_RRT_control {

    protected:

        // Search kd Tree of the calculated path for the nearest vector.
        std::pair<Eigen::VectorXd,int> find_next_point(Eigen::VectorXd init);

        // TODO: Implement these functions to handle contact forces.
        void update_constraints();
        void update_path();

        // Expects input to be the current positions and velocities of the robot. When not using mujoco, the input data will be taken from sensors.
        void find_path(Eigen::VectorXd init, Eigen::VectorXd goal);

        // Constrain specification matricies
        Eigen::MatrixXd constraints;
        Eigen::MatrixXd space_bounds;
        int config_space;

        // Global planner objects and robot model
        pathplan* planner; 
        robotModel* model;
        kdTree kdtree;

        // Saved path
        Eigen::MatrixXd saved_path;

        // Flags and joint identification
        bool path_found;
        std::vector<int> associated_joint_bodies;

        // Variables to help speed up point search
        int deleted_kdnodes;

    public:

        global_RRT_control(mjModel* m, mjData* d,std::string planner_type);
        void reset();

};



// Takes the robot model and some initial options (such as the type of path planner) as the initializer input
// Calculate trajectory take the initial and final positions in joint space.
class PID : private global_RRT_control {

private:
    
    // PID Control variables
    double kp;
    double kd;
    double ki;
    int num_jnt;
    Eigen::VectorXd cum_sum;

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
    Eigen::VectorXd get_control(Eigen::VectorXd init, Eigen::VectorXd goal);


};

struct x_dot_struct{

    int timesteps;
    int jnts;
    int pos;
    int index;

    Eigen::MatrixXd data; 

    void reset(){
        this->data = Eigen::MatrixXd::Zero(timesteps, jnts);
        this->pos = 0;
        this->index = 0;
    }

    x_dot_struct(int timesteps, int jnts){
        this->timesteps = timesteps;
        this->jnts = jnts;
        this->pos = 0;
        this->index = 0;
    }

};

struct init_struct{
    int pos;
    Eigen::VectorXd data;

    init_struct(Eigen::VectorXd init){

        this->data = init;
        this->pos = 0;

    }

};

// Take in the robot model and create a robot model, take current robot position and goal position and create RRT plan like the PID.
// To calculate the next step, solve a non-linear optimization and output the first step, move to next point in the rrt when its close enough. 
class MPC : global_RRT_control{

    private:

        // Buffer variables used by PSOPT functions
        Eigen::VectorXd final_goal;

        // NLOPT Specific optimization functions
        static double cost_function(unsigned n, const double *x, double *grad, void *data);
        static double dynamic_constraint(unsigned n, const double *x, double *grad, void *data);
        static double set_init_constraint(unsigned n, const double *x, double *grad, void *data);
        static double set_final_constraint(unsigned n, const double *x, double *grad, void *data);

        // run optimization function using PSOPT
        Eigen::VectorXd optimize(Eigen::VectorXd init, Eigen::VectorXd goal, double T, double N);

        Eigen::VectorXd curr_goal;

        // Other variables
        int num_jnt;

    public:

        // Initialize controller for MuJoCo environment
        MPC(mjModel* m, mjData* d,std::string planner_type);
        void reset();
        Eigen::VectorXd get_control(mjModel* m, mjData* d, Eigen::VectorXd goal, double T, double N);
        Eigen::VectorXd get_control(Eigen::VectorXd init, Eigen::VectorXd goal, double T, double N);

};
