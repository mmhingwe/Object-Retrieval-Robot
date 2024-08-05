#pragma once 
#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Core>
// #include "datastructures/kdtree.h"
#include "kdtree.h"


// Note: The cartesian initial point and goal is converted to config space in the robot model class.
//Input: intitial_config, goal_config, state dimension, constraints, options
//Output: Matrix of path in configuration space.
//        Matrix of the acceleration and velocities of config space path.


// // Make this a parent class, defne all the basics a path algo needs
class pathplan{

    protected:
        int config_space_dim;
        std::vector<Eigen::MatrixXd> constraints; // A (2n+2)xk sized matrix, where n is the dimensions of the configuration space and k is the number of constraints.
        Eigen::MatrixXd space_bounds;
        Eigen::MatrixXd path;
        Eigen::VectorXd init;
        Eigen::VectorXd goal;
        virtual void check_inputs();
    public:
        pathplan(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints, Eigen::MatrixXd space_bounds);
        virtual Eigen::MatrixXd return_path() = 0;
        virtual Eigen::MatrixXd return_path_vec() = 0;
        virtual Eigen::MatrixXd return_path_accel() = 0;

};


// Node for the RRT class 
struct rrtnode : public node<Eigen::VectorXd>{

    double dist;
    rrtnode(Eigen::VectorXd point, double dist);

};


// RRT Implementation here. Child class of the pathplan class.
class RRT : private pathplan{

    private:
        
        // Initial and Goal nodes
        rrtnode* init_node;
        rrtnode* goal_node;
        
        // Private functions
        void check_inputs();
        void delete_intermediate_nodes();
        Eigen::VectorXd sample_space();
        bool check_collision(Eigen::VectorXd point);
        rrtnode* nearest_node(Eigen::VectorXd sample);

    public:
        
        RRT(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints,Eigen::MatrixXd space_bounds);
        ~RRT();
        int run(double step_size, double goal_radius, int max_itt);
        Eigen::MatrixXd return_path();
        Eigen::MatrixXd return_path_vec();
        Eigen::MatrixXd return_path_accel();
        rrtnode* return_root(); // For graphing purposes.


};


// // RRT* Implementation using KD Trees
// class RRTstar : private pathplan{

//     private:
        
//         int k = 5; // TODO make this a constructor input.
//         int count;
//         int max_itt = 10;
//         double step_size;
        
//         double max_dist;
//         double max_radius = 5;
        
//         // RRT Graph (KD-Tree Datastructure so finding nearest neighbor will be O(log(N)) time)
//         kdnode* init_node;
//         kdnode* goal_node;
//         // kdTree* graph;
        
//         float dist(kdnode* input1, kdnode* input2);
//         void check_inputs();
//         // Eigen::VectorXf sample_space();
//         bool check_collision(Eigen::VectorXd point);
//         std::vector<kdnode*> nearest_node(Eigen::VectorXd sample);

//     public:
//         RRTstar(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim,std::vector<Eigen::MatrixXd> constraints,Eigen::MatrixXd space_bounds, double stepsize, double max_dist, double max_radius);
//         ~RRTstar();
//         void run();
//         Eigen::MatrixXd return_path();
//         Eigen::MatrixXd return_path_vec();
//         Eigen::MatrixXd return_path_accel();
//         Eigen::VectorXd sample_space();


// };