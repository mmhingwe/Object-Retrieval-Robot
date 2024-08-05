#pragma once 
#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Core>
#include "node.h"

// left child will be children.at(0) and right child wil be children.at(1) in the node.h
struct kdnode : public node<Eigen::VectorXd>{


    int dim;
    int axis;

    kdnode(Eigen::VectorXd point, kdnode* parent,int axis);
    kdnode(Eigen::VectorXd point, kdnode* parent, kdnode* lchild, kdnode* rchild, int axis);

};

class kdTree{

    kdnode* root;
    int dim;
    // bool geq();

    public:
        kdTree();
        kdTree(Eigen::VectorXd init);
        ~kdTree();
        kdnode* insert_node(node<Eigen::VectorXd>* point);
        void delete_node(Eigen::VectorXd point);
        std::vector<kdnode*> find_k_nearest(Eigen::VectorXd point, int k, bool radius);
        

};
