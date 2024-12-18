#include <iostream>
#include <Eigen/Core>
#include <time.h>
#include <vector>
#include <stack>
#include <numeric>
#include "pathplan.h"
// #include "datastructures/kdtree.h" TODO: Move kd tree into seperate datastructures folder.
#include "datastructures/kdtree.h"
using namespace std;


// pathplan class function definitions

pathplan::pathplan(){}

pathplan::pathplan(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints,Eigen::MatrixXd space_bounds): init(init), goal(goal), config_space_dim(config_space_dim),constraints(constraints),space_bounds(space_bounds){
    check_inputs();
}

// This function ensures that the dimensions and tpyes of the the constructor inputs are correct. If not, an error or warning message is sent.
void pathplan::check_inputs(){
    cout << "pathplan check" << endl;
}


//RRTNode class definitions here.
rrtnode::rrtnode(Eigen::VectorXd point, double dist){
    this->data = point;
    this->dist = dist;
    //TODO: Initialized the node class as well.
}


// RRT Class function definitions:
RRT::RRT() : pathplan(){

    // Iniialize random number generator
    srand((unsigned)time(NULL));

}

RRT::RRT(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints, Eigen::MatrixXd space_bounds):  pathplan(init,goal,config_space_dim,constraints,space_bounds){
    check_inputs();
     
    // Iniialize random number generator
    srand((unsigned)time(NULL));

    // Create nodes for the initial and goal state
    this->init_node = new rrtnode(init,0);
    this->goal_node = new rrtnode(goal,0);

}

// Destructor for RRT class
// O(V) space taken since it finds all node pointers and then deletes them induvidually
RRT::~RRT(){
   delete_intermediate_nodes();
   delete this->init_node;
   delete this->goal_node;
}

void RRT::set_perameters(Eigen::VectorXd init, Eigen::VectorXd goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints,Eigen::MatrixXd space_bounds){

    //Create Initial nodes
    this->init_node = new rrtnode(init,0);
    this->goal_node = new rrtnode(goal,0);

    // setup constraints
    this->config_space_dim = config_space_dim;
    this->constraints = constraints;
    this->space_bounds = space_bounds;

}

// O(2V+E) time and space to free all intermediate rrtnodes from the heap.
void RRT::delete_intermediate_nodes(){

    stack<rrtnode*> dfs_stack;
    vector<rrtnode*> nodes;
    rrtnode* current = this->init_node;

    // Find all the nodes in the RRT graph and store it in nodes.
    while(current != nullptr){  

        if  ((current != this->init_node) && (current != this->goal_node)){
            nodes.push_back(current);
        }
        for(int i =0; i < current->children.size(); i++){
           dfs_stack.push(static_cast<rrtnode*>(current->children.at(i)));
        }
        if (dfs_stack.empty() == false){
            current = dfs_stack.top();
            dfs_stack.pop();
        }
        else{
            current = nullptr;
        }
    }

    //Delete all nodes in the heap.
    for (int i = 0; i < nodes.size(); i++){
        if (nodes.at(i) != nullptr){
            delete nodes.at(i);
            nodes.at(i) = nullptr;
        }
    }

}

void RRT::check_inputs(){
    cout << "RRTstar input check" << endl;
}



void RRT::save_path(){

    vector<Eigen::VectorXd> buffer_path_mat;

    rrtnode* ptr = this->goal_node;

    while(ptr->parents.size() >= 1){
        // cout << "going to parent" << endl;
        buffer_path_mat.push_back(ptr->data);

        ptr = static_cast<rrtnode*>(ptr->parents[0]);

    }
    buffer_path_mat.push_back(this->init_node->data);

    Eigen::MatrixXd path_matrix = Eigen::MatrixXd::Zero(this->config_space_dim,buffer_path_mat.size());
    
    Eigen::ArrayXi dim_idx_array = Eigen::ArrayXi::LinSpaced(this->config_space_dim,0,this->config_space_dim-1);

    for (int i = 0; i < buffer_path_mat.size(); i++){
        path_matrix(dim_idx_array,i) = buffer_path_mat[buffer_path_mat.size() - 1 -i];
    }

    // cout << "PATH MATRIX" << endl;
    // cout << path_matrix << endl;

    this->path = path_matrix;

}

// Sample the configuration space.
Eigen::VectorXd RRT::sample_space(){
    
    vector<double> sample;
    cout << this->space_bounds << endl;

    for(int i = 0; i < config_space_dim; i++){
        int ub = space_bounds((2*i)+1,2);
        int lb = space_bounds(2*i,2);

        cout << lb << "   " << ub << endl;

        double gen_num = (double)rand()/RAND_MAX;
        
        // if(lb <= 0){
        //     sample.push_back(((ub + abs(lb)) * gen_num) - abs(lb));

        // }
        // else{
        //     sample.push_back(((ub - lb) * gen_num) + lb);
        // }

        sample.push_back(((ub - lb) * gen_num) + lb);

    }

    cout << "sample vector" << endl;
    for (int i = 0; i < sample.size(); i++){
        cout << sample[i] << "  ";
    }
    cout << endl;

    Eigen::VectorXd out = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(sample.data(),sample.size());
    cout << out << endl;

    return out;
}

// NOTE: May move this to the parent pathplan class or at least make it virtual as all methods should check for collisions with the constraints matrix.
bool RRT::check_collision(Eigen::VectorXd point){

    // check the type of the constraint (0:=, 1:>=, 2: <=, 3: >, 4: <)
    for (int i =0; i < this->constraints.size(); i++){

        int count = 0;
        for (int j = 0; j< this->constraints.at(i).rows(); j++){

            switch((int)this->constraints.at(i)(j,1)){

                case 0: // "= perameter"
                    if (point[this->constraints.at(i)(j,0)] == this->constraints.at(i)(j,2)){
                        count += 1;
                    }
                    break;

                case 1:
                    if (point[this->constraints.at(i)(j,0)] >= this->constraints.at(i)(j,2)){
                        count += 1;
                    }
                    break;

                case 2:
                    if (point[this->constraints.at(i)(j,0)] <= this->constraints.at(i)(j,2)){
                        count += 1;
                    }
                    break;

                case 3:
                    if (point[this->constraints.at(i)(j,0)] > this->constraints.at(i)(j,2)){
                        count += 1;
                    }
                    break;

                case 4:
                    if (point[this->constraints.at(i)(j,0)] < this->constraints.at(i)(j,2)){
                        count += 1;
                    }
                    break;

            }

        }
        if(count == this->constraints.at(i).rows()){
            return true; //There is a collision!
        }

    }

    return false;
    
}

// performs dfs to find the nearest node to the sample.
// O(V+E) time complexity. 
rrtnode* RRT::nearest_node(Eigen::VectorXd sample){

    stack<rrtnode*> dfs_stack;
    rrtnode* current = this->init_node;
    rrtnode* min;
    double min_dist = -1; //keeps track of the distance of the nodes

    while(current != nullptr){

        double dist = (current->data - sample).norm(); // Eigen library normalize to get distance.
       
        if (min_dist == -1){
            min = current;
            min_dist = dist;
        }

        if (dist<=min_dist){
            min = current;
            min_dist = dist;
        }

        for(int i =0; i < current->children.size(); i++){
           dfs_stack.push(static_cast<rrtnode*>(current->children.at(i)));
        }

        if (dfs_stack.empty() == false){
            current = dfs_stack.top();
            dfs_stack.pop();
        }
        else{
            current = nullptr;
        }


    }

    return min;
    

}

// Returns the total number of itterations it took to get to the goal node
int RRT::run(double step_size, double goal_radius, int max_itt){
    
    // Reset the graph
    delete_intermediate_nodes();

    int count = 0;

    cout << max_itt << endl;

    double init_goal_dist = (this->init_node->data - this->goal_node->data).norm();
    if (init_goal_dist < goal_radius){ 
    
        // Add the goal node to the graph.
        this->init_node->children.push_back(this->goal_node);
        this->goal_node->parents.push_back(this->init_node);
        
        //TODO: Add function which saves path from goal to the root in the pathplan::path matrix format.
        this->save_path();

        return count;

    }


    while (count < max_itt){

        // Uncomment for debug messages
        cout << "Itteration: " << count << endl;
        
        
        // sample a point from the bounds of the configuration space.
        Eigen::VectorXd sample = sample_space();

        // Find the nearest node to the sample point step in the direction of the sampeld point
        rrtnode* nearest_neighbor = nearest_node(sample);
        Eigen::VectorXd line = sample - nearest_neighbor->data;
        Eigen::VectorXd new_point = nearest_neighbor->data + ((sample - nearest_neighbor->data) * step_size);

        
        // Check if the new point is within the infeasable region of the space
        if (!check_collision(new_point)){
            cout << new_point << endl;
            // Create new node for the new point and add it to the graph.
            rrtnode* new_node = new rrtnode(new_point,sizeof(new_point));
            new_node->parents.push_back(nearest_neighbor); 
            nearest_neighbor->children.push_back(new_node);

            // Check if the new node is within the radius around the goal node defined when calling this function.
            double dist = (new_node->data - this->goal_node->data).norm();
            if (dist < goal_radius){ 
            
                // Add the goal node to the graph.
                new_node->children.push_back(this->goal_node);
                this->goal_node->parents.push_back(new_node);
                
                //TODO: Add function which saves path from goal to the root in the pathplan::path matrix format.
                this->save_path();

                return count;

            }

            count += 1;

        }

        // Uncomment for debug messages
        cout << "------------------------------------" << endl;


    }

    return -1;

}

// Return the initial node so that it can be graphed.
rrtnode* RRT::return_root(){
    return this->init_node;
}

// Rows: Points of the path
// Columns: coordinates of the point in the configuration space. 
Eigen::MatrixXd RRT::return_path(){
    return this->path;
}

Eigen::MatrixXd RRT::return_path_vec(){
    Eigen::MatrixXd buf;
    return buf;
}

Eigen::MatrixXd RRT::return_path_accel(){
    Eigen::MatrixXd buf;
    return buf;
}








//TODO: Continue working on this after implementing basic RRT
// // RRTStar class function definitions

// RRTstar::RRTstar(Eigen::VectorXf init, Eigen::VectorXf goal,int config_space_dim, std::vector<Eigen::MatrixXd> constraints, Eigen::MatrixXf space_bounds, double stepsize, double max_dist, double max_radius): step_size(stepsize), max_dist(max_dist), max_radius(max_radius), pathplan(init,goal,config_space_dim,constraint,space_bounds){
//     check_inputs();
     
//     // Iniialize random number generator
//     srand((unsigned)time(NULL));

//     // Create nodes for the initial and goal state
//     // this->init_node = new kdnode(init, nullptr, nullptr,0);
//     // this->goal_node = new kdnode(goal, nullptr, nullptr,0);

//     // TODO 
//     count = 0;
//     max_itt = 10; 
// }

// // Destructor for RRTstar class
// RRTstar::~RRTstar(){

// }

// // Samples the space defined by the bounds and returns a sample point in the configuration space.
// Eigen::VectorXf RRTstar::sample_space(){
    
//     vector<float> sample;

//     for(int i = 0; i < config_space_dim; i++){
//         int ub = space_bounds(i,1);
//         int lb = space_bounds(i,0);
//         float gen_num = (float)rand()/RAND_MAX;
        
//         if(lb <= 0){
//             sample.push_back(((ub + abs(lb)) * gen_num) - abs(lb));

//         }
//         else{
//             sample.push_back(((ub - lb) * gen_num) + lb);
//         }

//     }

//     // Convert vector to eigen::vector
//     // Map works with the memory of the sample vector. Create a copy of this as Eigen::VectorXf out so a segmentation fault does not occour when 
//     // vector<float> sample goes out of bounds!
//     Eigen::VectorXf out = Eigen::Map<Eigen::VectorXf,Eigen::Unaligned>(sample.data(),sample.size());

//     return out;

// }

// void RRTstar::check_inputs(){
//     cout << "RRTstar input check" << endl;
// }

// // bool RRTstar::check_collision(Eigen::VectorXf point){}


// // float RRTstar::dist(kdnode* input1, kdnode* input2){
// //     return (input1 - input2).norm();
// // }

// // The first index of the output array is the node which has the shortest path.
// // vector<kdnode*> RRTstar::nearest_node(Eigen::VectorXf sample){

// //     // Search for nearest neighbors using kd tree datastructure

    
    

    
// // }

// void RRTstar::run(){
    
//     while (this->count < this->max_itt){

//         // sample a point from the bounds of the configuration space.
//         Eigen::VectorXf sample = sample_space();

//         //TODO: Include code to check if sample is in the obstacle region.

//         // vector<kdnode*> nearest_nodes = nearest_node(sample);
        
//         // if (nearest_nodes.size() == 1){ //If the return size is one, no nearest neighbors found in radius. Do normal RRT, else do RRT*

//         // }
//         // else{


//         // }

        
//         this->count += 1;

//     }
    

// }

// Eigen::MatrixXf RRTstar::return_path(){
//     Eigen::MatrixXf buf;
//     return buf;
// }

// Eigen::MatrixXf RRTstar::return_path_vec(){
//     Eigen::MatrixXf buf;
//     return buf;
// }

// Eigen::MatrixXf RRTstar::return_path_accel(){
//     Eigen::MatrixXf buf;
//     return buf;
// }