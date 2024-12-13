#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <limits>
#include <queue>
#include <stack>
#include "datastructures/kdtree.h"
#include "pathplan.h"
using namespace std;

kdnode::kdnode(Eigen::VectorXd init,kdnode* parent){
    this->point = init;
    this->parent = parent;
    this->left = nullptr;
    this->right = nullptr;
    this->count = 1;
    this->dim = sizeof(init);
}

// Function definitions for kd tree datastructure.
kdTree::kdTree():root(nullptr),dim(0){}


kdTree::kdTree(Eigen::VectorXd init){
    this->root = new kdnode(init, nullptr);
    this->dim = init.size();
}

kdTree::~kdTree(){
    this->reset();
    this->root = nullptr;
}

void kdTree::insert_node(Eigen::VectorXd point,int idx){

    if (this->root == nullptr){  
        this->root = new kdnode(point,nullptr);
        this->dim = point.size();
        root->idx = idx;
    }
    else{
        int itt = 0;
        kdnode* ptr = this->root;
        while(true){
            int index = itt % this->dim;
            Eigen::VectorXd cur_point = ptr->point;
            if (cur_point(index) >= point(index)){
                if (ptr->left == nullptr){
                    kdnode* left_node = new kdnode(point,ptr);
                    ptr->left = left_node;
                    left_node->idx = idx;
                    left_node->dim = (index + 1) % this->dim;
                    break;
                }
                else{
                    ptr = ptr->left;
                }
            }
            else{
                if (ptr->right == nullptr){
                    kdnode* right_node = new kdnode(point,ptr);
                    right_node->idx = idx;
                    ptr->right = right_node;
                    right_node->dim = (index + 1) % this->dim;
                    break;
                }
                else{
                    ptr = ptr->right;
                }
            }
            itt += 1;
        }
    }
}

void kdTree::populate(Eigen::MatrixXd mat){
    Eigen::ArrayXi col_arr = Eigen::ArrayXi::LinSpaced(mat.cols(),0,mat.cols()-1);
    for (int i=0; i<mat.rows();i++){
        this->insert_node(mat(i,col_arr),i);
    }
}


pair<kdnode*,double> kdTree::dfs(kdnode* curr_ptr, Eigen::VectorXd point, int depth){

    if (curr_ptr == nullptr){
        return make_pair(nullptr,std::numeric_limits<double>::max());
    }

    kdnode* nextnode;
    kdnode* othernode;

    if (point(depth%point.size()) < curr_ptr->point(depth%point.size())){
        nextnode = curr_ptr->left;
        othernode = curr_ptr->right;
    }
    else{
        nextnode = curr_ptr->right;
        othernode = curr_ptr->left;
    }

    pair<kdnode*, double> out;
    pair<kdnode*,double> downstream_min = this->dfs(nextnode,point,depth+1);
    
    double current_dist = (point - curr_ptr->point).squaredNorm();
    if (current_dist <= downstream_min.second){
        out.first = curr_ptr;
        out.second = current_dist;
    }
    else{
        out = downstream_min;
    }

    double section_error = point(depth%point.size()) - curr_ptr->point(depth%point.size());

    if ((out.second * out.second) >= (section_error * section_error)){
        pair<kdnode*,double> downstream_min = this->dfs(othernode,point,depth+1);
        if (current_dist <= downstream_min.second){
            out.first = curr_ptr;
            out.second = current_dist;
        }
        else{
            out = downstream_min;
        }
    }

    
    return out;

}

// Use .squaredNorm() on a vector to get distance.
//use sizeof point to see if min is found yet.
pair<Eigen::VectorXd,int> kdTree::find_nearest(Eigen::VectorXd point){
    kdnode* ptr = this->root;
    pair<kdnode*,double> out = this->dfs(ptr,point,0);
    return make_pair(out.first->point,out.first->idx);
}

// vector<Eigen::VectorXd> kdTree::find_k_nearest(Eigen::VectorXd point, int k, int radius){}


// kdnode* find(Eigen::VectorXd point){

//     // stack<kdnode*> st;
//     // st.push(this->root);
//     kdnode* ptr = this->root;
//     int itt = 0;

//     while(ptr != nullptr){

//         if (point == ptr->point){
//             return ptr;
//         }

//         int at_dim = itt / this->dim;

//         if (point(at_dim) >= ptr->point(at_dim)){
//             ptr = ptr->right;
//         }
//         else{
//             ptr->ptr->left;
//         }

//     }

//     return nullptr;

// }

kdnode* kdTree::return_min(kdnode* subroot,int dimension, int depth){

    if (subroot == nullptr){
        return nullptr;
    }

    int cd = depth % this->dim;

    if (dimension == cd){
        if (subroot->left == nullptr){
            return subroot;
        }
        return this->return_min(subroot->left,dimension,depth +1);
    }
    
    // find minimum

    kdnode* min = subroot;
    kdnode* min_left = this->return_min(subroot->left, dimension, depth+1);
    kdnode* min_right = this->return_min(subroot->right, dimension, depth+1);

    if ((min_left != nullptr) && (min_left->point(dimension) < min->point(dimension))){
        min = min_left;
    }
    if ((min_right != nullptr) && (min_right->point(dimension) < min->point(dimension))){
        min = min_right;
    }

    return min;

}

kdnode* kdTree::recursive_del(kdnode* subroot, Eigen::VectorXd point, int depth){

    if (subroot == nullptr){
        return nullptr;
    }

    int current_dim = depth % this->dim;

    if (subroot->point == point){

        // If the node is a leaf node, just delete it and pass nullptr to its parent
        if ((subroot->left == nullptr) && (subroot->right == nullptr)){
            delete subroot;
            return nullptr;
        }
        else if (subroot->right != nullptr){
            kdnode* min_kdnode = this->return_min(subroot->right,current_dim,depth+1);
            subroot->point = min_kdnode->point;
            subroot->dim = current_dim;
            subroot->idx = min_kdnode->idx;
            subroot->right = this->recursive_del(subroot->right,min_kdnode->point,depth + 1);
        }
        else{
            kdnode* min_kdnode = this->return_min(subroot->left,current_dim,depth+1);
            subroot->point = min_kdnode->point;
            subroot->dim = current_dim;
            subroot->idx = min_kdnode->idx;
            subroot->right = this->recursive_del(subroot->left,min_kdnode->point,depth + 1);
            subroot->left = nullptr;
        }
    }
    else{
        if (point(current_dim) >= subroot->point(current_dim)){
            subroot->right = this->recursive_del(subroot->right, point, depth +1);
        }
        else{
            subroot->left = this->recursive_del(subroot->left,point,depth +1);
        }

    }

    return subroot;

}

void kdTree::delete_node(Eigen::VectorXd point){
    this->recursive_del(this->root,point,0);
}

void kdTree::reset(){
    queue<kdnode*> q;
    q.push(this->root);
    while (!q.empty()){
        kdnode* ptr = q.front();
        q.pop();
        if (ptr != nullptr){
            q.push(ptr->left);
            q.push(ptr->right);
            delete ptr;
        }
    }
    this->root = nullptr;
}
        

void kdTree::print_debug_info(){
    stack<kdnode*> st;
    st.push(this->root);
    while (!st.empty()){
        kdnode* ptr = st.top();
        st.pop();
        if (ptr->left != nullptr){
            st.push(ptr->left);
        }
        if (ptr->right != nullptr){
            st.push(ptr->right);
        }
    }
}