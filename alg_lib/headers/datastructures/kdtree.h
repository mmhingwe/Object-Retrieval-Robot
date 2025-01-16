#pragma once 
#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Core>
#include "node.h"

// Node for kdtree
struct kdnode{

    kdnode* parent;
    kdnode* left;
    kdnode* right;
    int dim;
    int idx;
    int count; //Keep track of repeated points.
    Eigen::VectorXd point;

    kdnode();
    kdnode(Eigen::VectorXd init,kdnode* parent);

};

class kdTree{

    kdnode* root;
    int dim;
    int size;
    std::pair<kdnode*,double> dfs(kdnode* curr_ptr, Eigen::VectorXd point, int depth);
    kdnode* recursive_del(kdnode* subroot, Eigen::VectorXd point, int depth);
    kdnode* return_min(kdnode* subroot,int dimension, int depth);

    public:
        kdTree();
        kdTree(Eigen::VectorXd init);
        ~kdTree();
        int return_size();
        void insert_node(Eigen::VectorXd point,int idx);
        void populate(Eigen::MatrixXd mat);
        void delete_node(Eigen::VectorXd point);
        std::pair<Eigen::VectorXd,int> find_nearest(Eigen::VectorXd point);
        std::vector<Eigen::VectorXd> find_k_nearest(Eigen::VectorXd point, int k, int radius); //If radius is true, if more than k points found within radius, return the closest k.
        void reset();

        // Debugging functions:
        void print_debug_info();
        

};
