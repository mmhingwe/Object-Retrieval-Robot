#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <limits>
#include <queue>
#include <stack>
#include "datastructures/octtree.h"

using namespace std;


// Octnode Definitions

octnode::octnode(){
    for (int i=0; i < 8; i++){
        this->children[i] = nullptr;
    }
}

octnode::octnode(Eigen::MatrixXd bounding_box, octnode* parent, int max_depth, int depth, bool obstacle) : bounding_box(bounding_box), parent(parent) ,max_depth(max_depth), depth(depth), obstacle(obstacle){
    for (int i=0; i < 8; i++){
        this->children[i] = nullptr;
    }
}

int octnode::num_children(){
    int out = 0;
    for (int i = 0; i < 8; i++){
        if (this->children[i] != nullptr){
            out += 1;
        }
    }
    return out;
}


// Octtree function definitions

octtree::octtree():head(nullptr),subdivisions(-1){}

octtree::octtree(Eigen::MatrixXd space_bounds, int subdivisions) : subdivisions(subdivisions){
    this->head = new octnode(space_bounds,nullptr,subdivisions,0,false);
}

octtree::~octtree(){

    stack<octnode*> st;
    st.push(this->head);

    while (!st.empty()){

        octnode* ptr = st.top();
        st.pop();

        for (int i = 0; i < 8; i++){
            if (ptr->children[i] != nullptr){
                st.push(ptr->children[i]);
            }
        }

        delete ptr;

    }



}

void octtree::add(Eigen::Vector3d input){

    // using stack for the cases where the point is on the edge of two boundries
    // stack<octnode*> st;
    // st.push(this->head);
    octnode* ptr = this->head;

    while(ptr != nullptr){
        // octnode* ptr = st.top();
        // st.pop();
        if (ptr->obstacle == true){
            break;
        }

        // if leaf node, set the obstacle to 1, else create the next node and add it to the stack
        if (ptr->depth == (this->subdivisions-1)){
            ptr->obstacle = true;
            break;
        }
        else{
            Eigen::MatrixXd curr_bounding_box = Eigen::MatrixXd::Zero(3,2);
            curr_bounding_box = ptr->bounding_box;
            double mid_x = curr_bounding_box(0,0) + (curr_bounding_box(0,1) - curr_bounding_box(0,0))/2;
            double mid_y = curr_bounding_box(1,0) + (curr_bounding_box(1,1) - curr_bounding_box(1,0))/2;
            double mid_z = curr_bounding_box(2,0) + (curr_bounding_box(2,1) - curr_bounding_box(2,0))/2;

            // 0 indicates that it is from midpoint to upperbound, and 1 indicates the value is between the minimum and midpoint
            int octant = 0;
            if (input(0) < mid_x){
                octant += 1;
            }
            if (input(1) < mid_y){
                octant += 2;
            }
            if (input(2) < mid_z){
                octant += 4;
            }

            switch(octant){

                case 0: 
                    
                    if (ptr->children[0] != nullptr){
                        ptr = ptr->children[0];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << mid_x, curr_bounding_box(0,1), mid_y, curr_bounding_box(1,1), mid_z, curr_bounding_box(2,1);
                        ptr->children[0] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[0];
                    }

                    break;
                case 1: // octant 2
                    
                    if (ptr->children[1] != nullptr){
                        ptr = ptr->children[1];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << curr_bounding_box(0,0), mid_x, mid_y, curr_bounding_box(1,1), mid_z, curr_bounding_box(2,1);
                        ptr->children[1] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[1];
                    }

                    break;
                case 2: // octant 4
                    
                    if (ptr->children[2] != nullptr){
                        ptr = ptr->children[2];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << mid_x, curr_bounding_box(0,1), curr_bounding_box(1,0), mid_y, mid_z, curr_bounding_box(2,1);
                        ptr->children[2] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[2];
                    }

                    break;
                case 3: // octant 3
                    
                    if (ptr->children[3] != nullptr){
                        ptr = ptr->children[3];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << curr_bounding_box(0,0), mid_x, curr_bounding_box(1,0), mid_y, mid_z, curr_bounding_box(2,1);
                        ptr->children[3] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[3];
                    }

                    break;
                case 4: // octant 5
                    
                    if (ptr->children[4] != nullptr){
                        ptr = ptr->children[4];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << mid_x, curr_bounding_box(0,1), mid_y, curr_bounding_box(1,1), curr_bounding_box(2,0),mid_z;
                        ptr->children[4] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[4];
                    }

                    break;
                case 5:  // octant 6
                    
                    if (ptr->children[5] != nullptr){
                        ptr = ptr->children[5];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << curr_bounding_box(0,0), mid_x, mid_y, curr_bounding_box(1,1),curr_bounding_box(2,0),mid_z;
                        ptr->children[5] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[5];
                    }

                    break;
                case 6:  // octant 6
                    
                    if (ptr->children[6] != nullptr){
                        ptr = ptr->children[6];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << mid_x, curr_bounding_box(0,1), curr_bounding_box(1,0), mid_y, curr_bounding_box(2,0),mid_z;
                        ptr->children[6] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[6];
                    }

                    break;
                case 7: // octant 8
                    
                    if (ptr->children[7] != nullptr){
                        ptr = ptr->children[7];
                    }
                    else{
                        Eigen::MatrixXd subdivision_bounding_box(3,2);
                        subdivision_bounding_box << curr_bounding_box(0,0), mid_x, curr_bounding_box(1,0), mid_y,curr_bounding_box(2,0),mid_z;
                        ptr->children[7] = new octnode(subdivision_bounding_box,ptr,this->subdivisions,ptr->depth+1,false);
                        ptr = ptr->children[7];
                    }

                    break;

            }

        }

    }

    //work backwards to the top and check to see if any node should be made to have obstacle equal to true higher up the tree.
    while (ptr != nullptr){
        if (ptr->num_children() == 8){
            ptr->obstacle = true;
            for (int i = 0; i < 8; i++){
                if (ptr->children[i]->obstacle == false){
                    ptr->obstacle = false;
                    break;
                }
            }
        }
        ptr = ptr->parent;
    }

}

// delete empty nodes except the head at depth 0
void octtree::del(Eigen::Vector3d input){

    octnode* ptr = this->head;

    while(ptr != nullptr){

        if (ptr->depth == (this->subdivisions-1)){
            ptr->obstacle = false;

            // go upstream and delete all unnecessary nodes
            // while ptr->parent != nullptr makes sure the head node is not deleted
            while (ptr->parent != nullptr){
                if (ptr->num_children() == 0){
                    octnode* del_ptr = ptr;
                    ptr = ptr->parent;
                    delete del_ptr;
                }
            }

            break;
        }
        else{

            Eigen::MatrixXd curr_bounding_box = Eigen::MatrixXd::Zero(3,2);
            curr_bounding_box = ptr->bounding_box;
            double mid_x = curr_bounding_box(0,0) + (curr_bounding_box(0,1) - curr_bounding_box(0,0))/2;
            double mid_y = curr_bounding_box(1,0) + (curr_bounding_box(1,1) - curr_bounding_box(1,0))/2;
            double mid_z = curr_bounding_box(2,0) + (curr_bounding_box(2,1) - curr_bounding_box(2,0))/2;

            // 0 indicates that it is from midpoint to upperbound, and 1 indicates the value is between the minimum and midpoint
            int octant = 0;
            if (input(0) < mid_x){
                octant += 1;
            }
            if (input(1) < mid_y){
                octant += 2;
            }
            if (input(2) < mid_z){
                octant += 4;
            }

            ptr = ptr->children[octant];

        }


    }

}

bool octtree::is_collision(Eigen::Vector3d x){
    
    octnode* ptr = this->head;

    while(ptr != nullptr){

        if (ptr->depth == (this->subdivisions-1)){
            if (ptr->obstacle == true){
                return true;
            }
            else{
                return false;
            }
        }
        else{

            if (ptr->obstacle == true){
                return true;
            }

            Eigen::MatrixXd curr_bounding_box = Eigen::MatrixXd::Zero(3,2);
            curr_bounding_box = ptr->bounding_box;
            double mid_x = curr_bounding_box(0,0) + (curr_bounding_box(0,1) - curr_bounding_box(0,0))/2;
            double mid_y = curr_bounding_box(1,0) + (curr_bounding_box(1,1) - curr_bounding_box(1,0))/2;
            double mid_z = curr_bounding_box(2,0) + (curr_bounding_box(2,1) - curr_bounding_box(2,0))/2;

            // 0 indicates that it is from midpoint to upperbound, and 1 indicates the value is between the minimum and midpoint
            int octant = 0;
            if (x(0) < mid_x){
                octant += 1;
            }
            if (x(1) < mid_y){
                octant += 2;
            }
            if (x(2) < mid_z){
                octant += 4;
            }

            ptr = ptr->children[octant];

        }

    }

    return false;

}

// takes in 3x2 matrix outlining the bounding box in 3d space
vector<Eigen::MatrixXd> octtree::return_overlap_area(Eigen::MatrixXd bounds){

    vector<Eigen::MatrixXd> out;
    stack<octnode*> st;
    st.push(this->head);

    while(!st.empty()){

        octnode* ptr = st.top();
        st.pop();

        if (ptr->obstacle == true){
            out.push_back(ptr->bounding_box);
        }
        else{

            Eigen::MatrixXd curr_bounding_box = ptr->bounding_box;
            double mid_x = curr_bounding_box(0,0) + (curr_bounding_box(0,1) - curr_bounding_box(0,0))/2;
            double mid_y = curr_bounding_box(1,0) + (curr_bounding_box(1,1) - curr_bounding_box(1,0))/2;
            double mid_z = curr_bounding_box(2,0) + (curr_bounding_box(2,1) - curr_bounding_box(2,0))/2;

            vector<int> octants;

            //check x octants
            if (mid_x <= bounds(0,0)){
                octants.push_back(0);
            }
            else if (mid_x > bounds(0,1)){
                octants.push_back(1);
            }
            else{
                octants.push_back(0);
                octants.push_back(1);
            }

            // check y octants
            if (mid_y <= bounds(1,0)){
                // no change
            }
            else if (mid_y > bounds(1,1)){
                int oct_size = octants.size();
                for (int i = 0; i < oct_size; i++){
                    octants[i] += 2;
                }
            }
            else{
                int oct_size = octants.size();
                for (int i = 0; i < oct_size; i++){
                    octants.push_back(octants[i] + 2);
                }
            }

            // check z octants
            if (mid_z <= bounds(2,0)){
                // no change
            }
            else if (mid_z > bounds(2,1)){
                int oct_size = octants.size();
                for (int i = 0; i < oct_size; i++){
                    octants[i] += 4;
                }
            }
            else{
                int oct_size = octants.size();
                for (int i = 0; i < oct_size; i++){
                    octants.push_back(octants[i] + 4);
                }
            }

            // push all existing nodes which intersect with the input bounding box.
            for (int i = 0; i < octants.size(); i++){
                if (ptr->children[octants[i]] != nullptr){
                    st.push(ptr->children[octants[i]]);
                }
            }

        }

    }

    return out;

}
