#include <iostream>
#include <Eigen/Core>
#include <vector>
// #include "datastructures/kdtree.h" TODO: Move kd tree into seperate datastructures folder.
#include "kdtree.h"
#include "pathplan.h"
using namespace std;

// kdnode initializer 
kdnode::kdnode(Eigen::VectorXd point, kdnode* parent, int axis): axis(axis){

    this->data = point;
    this->parents.push_back(parent);
    this->children.push_back(nullptr);
    this->children.push_back(nullptr);
    dim = sizeof(point);

}

kdnode::kdnode(Eigen::VectorXd point, kdnode* parent, kdnode* lchild, kdnode* rchild, int axis):  axis(axis){

    this->data = point;
    this->parents.push_back(parent);
    this->children.push_back(lchild);
    this->children.push_back(rchild);
    dim = sizeof(point);

}



// Function definitions for kd tree datastructure.
kdTree::kdTree():root(nullptr),dim(0){}


kdTree::kdTree(Eigen::VectorXd init){
    this->root = new kdnode(init,nullptr,0);
    this->dim = sizeof(init);
}

kdTree::~kdTree(){

}

//TODO: Check if the size of the VectorXD is causing ant problems. There could verywell be as mismatch there. 
//NOTE: since this is a binary tree structure, the left node is index 0 in the children node vector, while the right is index 1
// Using the node class to make things more general. 
// Returns the pointer to the newly created node
kdnode* kdTree::insert_node(node<Eigen::VectorXd>* point){

    int current_axis = 0;
    kdnode* curr_kdnode = this->root;

    while(true){

        double diff = (curr_kdnode->data - point->data)[current_axis];

        if (diff >= 0){  //TODO: This shouldnt be a problem for rrt but if its equal to a node already in the tree is is necessary adding a new one?

            if(curr_kdnode->children.at(0) == nullptr){
                curr_kdnode->children.at(0) = new kdnode(point->data,curr_kdnode,(current_axis + 1) % this->dim); 
                curr_kdnode = static_cast<kdnode*>(curr_kdnode->children.at(0));
                return curr_kdnode;
            }
            else{
                curr_kdnode = static_cast<kdnode*>(curr_kdnode->children.at(0));
            }

        }
        else{

            if(curr_kdnode->children.at(1) == nullptr){
                curr_kdnode->children.at(1) = new kdnode(point->data,curr_kdnode,(current_axis + 1) % this->dim); 
                curr_kdnode = static_cast<kdnode*>(curr_kdnode->children.at(1));
                return curr_kdnode;
            }
            else{
                curr_kdnode = static_cast<kdnode*>(curr_kdnode->children.at(1));
            }

        }

        if (curr_kdnode == nullptr){
            break;
        }

        current_axis  = (current_axis + 1) % this->dim;

    }

}

//TODO: Implement node deletion on the kdtree for RRTstar.
// void kdTree::delete_node(Eigen::VectorXd point){

// }


//TODO: Implement search to find the nearest neighbors
// vector<kdnode*> kdTree::find_k_nearest(Eigen::VectorXd point, int k, bool radius){

// }