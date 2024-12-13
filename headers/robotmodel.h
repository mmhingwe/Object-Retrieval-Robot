#pragma once 

#include <iostream>
#include <string>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include "datastructures/node.h"
#include "datastructures/eigen_extentions.h"



// This class acts as a wrapper around the mjmodel class. Specifications of each
// Robot body in relation to children and parent joints are stored 
// The robot body holds some specifications such as the orientation, weight, inertial matrix, etc and allows for transformation calculations in the robot Model class
// when doing forwards or backwards kinematics. The name of the body is stored in the "data" variable of the node parent class defined in node.h
// Contains information on the bodies of the robot
struct bodynode : public node<int>{
    
    // The id of the body and the node which it is attached to
    int bodyid;
    int joint;
    // Eigen::VectorXd screw_axis;

    // transform from the previous body
    Eigen::VectorXd rel_pos;
    Eigen::MatrixXd rel_transform;
    Eigen::MatrixXd rel_rot;

    // Mass information
    double mass;
    Eigen::VectorXd rel_inertia_com_pos;
    Eigen::MatrixXd rel_inertia_rot; //If identity, special case.
    Eigen::MatrixXd diagonal_inertia;

    bodynode(int bodyid):bodyid(bodyid){}
    bodynode(Eigen::VectorXd rel_pos, Eigen::MatrixXd rel_rot,Eigen::MatrixXd rel_transform,double mass,Eigen::VectorXd rel_inertia_com_pos,Eigen::MatrixXd rel_inertia_rot, Eigen::MatrixXd diagonal_inertia,int bodyid);
    void print_info();

};




// Comparison struct for the priority queue when creating joint tree.
struct bodynode_comp{

    bool operator()(const bodynode* input1, const bodynode* input2) const {
        return input1->joint > input2->joint;
    }

};

class jointnode : public node<int> {

    private:

        //Id of the joint
        int jointid;

        std::vector<bodynode*> attached_bodies;
        
        // Dimension of world space (This is usually 3d, but I thought I should make this general just incase I want to test some things with a 2d bot.)
        int dim_world_space;

        // Local Rotation (P(i)_R_i, rotation between parent and child) (NOTE: Initially inputing rotation matrix, maybe try quaternion to avoid gimble lock?)
        Eigen::MatrixXd R_p_i;
        Eigen::MatrixXd T_p_i;

        Eigen::MatrixXd R_w_i;  // Or the M matrix in the math
        Eigen::MatrixXd T_w_i;
        
        // Screw axis (local)
        Eigen::VectorXd screw_axis;

        // Spatial inertial matrix in the frame of the joint body
        Eigen::MatrixXd spatial_inertia;

        
    public:

        jointnode(int jointid):jointid(jointid){}
        jointnode(int jointid,Eigen::VectorXd screw_axis);
        jointnode(std::vector<bodynode*> bodies);
        jointnode(Eigen::VectorXd position, Eigen::VectorXd velocity, Eigen::VectorXd accel, jointnode parent);
        void set_attached_bodies(std::vector<bodynode*> input);
        void set_screw_axis(Eigen::VectorXd axis);
        void pass_info_to_child();
        int getid();
        Eigen::MatrixXd get_spatial_inertia();
        Eigen::MatrixXd get_world_transform();
        Eigen::MatrixXd get_local_transformation();
        Eigen::VectorXd get_screw_axis();
        void set_rotation_variables(Eigen::MatrixXd R_p_i, Eigen::MatrixXd T_p_i, Eigen::MatrixXd R_w_i, Eigen::MatrixXd T_w_i);
        bodynode* get_attached_body();
        void print_info();
        

}; 




// This class stores the body of the robot as a tree structure and uses DFS to go through the robot model and calculate all relative and global orientations of the the points.
// Algorithms to calculate inverse and forwards dynamics are also included to provide the controler information.
// NOTE: This class will be initialized using the mjmodel from mujoco. Eventually the physical robot perameters should be able to be passed in. 
class robotModel{

    private:
        std::vector<Eigen::MatrixXd> joint_transforms;
        int joint_tree_size;
        jointnode* root_joint;
        bodynode* root_body;
        std::vector<Eigen::VectorXd> joint_screw_axis;

        Eigen::MatrixXd joint_relation;

        // dfs through the joint three to calculate inertia of all bodies and the relative and global base position transformation matricies.
        void setup_joint_tree();

    //TODO: Include the "M" matrix which is the transformation from the base to the end effector for the are at base positions (all arm angles at 0)

    // Include backwards kinematics calculator to store the theta values between steps.
    public:

        robotModel(mjModel* model, mjData* data); //upload the mj model, might be different when moving to irl robot. 
        
        // ~robotModel(); //FIXME: Implement the destructor.

        // Matrix x is a 4xn matrix where n is the number of joints, 1: position, 2: velocity, 3:acceleration, 4: external forces, outputs the torque
        Eigen::VectorXd RNEA(Eigen::MatrixXd x);
        Eigen::VectorXd FNEA(Eigen::MatrixXd x);
        // Eigen::VectorXad FNEA_PSOPT(Eigen::MatrixXad x);

        // Forwards and Inverse kinematics calculators (used for task space to joint space conversion and vise versa.)
        std::vector<Eigen::MatrixXd> FK(Eigen::MatrixXd q);
        Eigen::VectorXd IK(int end_effector_body_id); 

        // Other functions 
        std::vector<int> return_joint_bodies();
        int return_joint_tree_size();

        // Debugging functions
        void print_joint_tree();
        void print_body_tree();

};



