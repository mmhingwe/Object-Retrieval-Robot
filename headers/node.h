#pragma once 
#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Core>
// #include "datastructures/kdtree.h"


// This class acts as the parent class of all nodes in the project. This defines the basics of a node. All children classes define their own interneal data.
// The node also allows for reference to other node types. This is usefull in RRTstar where a node is stored in both the kkt graph and kdtree graph.
template<class T>
struct node{

    public:
        T data;
        node* reference; //NOTE: Not deleted in destructor as its just a reference to another node. The data the pointer is pointing to is deleted elsewhere.
        std::vector<node<T>*> parents;
        std::vector<node<T>*> children;


        node(){}
        node(std::vector<node*> parents, std::vector<node*> children):parents(parents),children(children){}
        
        // Leave the garbage collection to the graph and path planning classes.
        // ~node(){  
            // for(int i =0; i < parents.size(); i++){
            //     delete parents.at(i);
            //     parents.at(i) = nullptr;
            // }
            // for(int i =0; i < children.size(); i++){
            //     delete children.at(i);
            //     children.at(i) = nullptr;
            // }
        // }

        void set_reference(node* ref){
            this->reference = ref;
        }
        node* return_reference(){
            return reference;
        }

};