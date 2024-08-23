#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <util.h>
#include <map>
#include <stack>
#include <queue>
#include <mujoco/mujoco.h>
#include "robotmodel.h"
using namespace std;


// bodynode class function definitions
bodynode::bodynode(Eigen::VectorXd rel_pos, Eigen::MatrixXd rel_rot,Eigen::MatrixXd rel_transform,double mass,Eigen::VectorXd rel_inertia_com_pos,Eigen::MatrixXd rel_inertia_rot, Eigen::MatrixXd diagonal_inertia, int bodyid){
    this->rel_pos = rel_pos;
    this->rel_transform = rel_transform;
    this->rel_rot = rel_rot;
    this->mass = mass;
    this->rel_inertia_com_pos = rel_inertia_com_pos;
    this->rel_inertia_rot = rel_inertia_rot;
    this->diagonal_inertia = diagonal_inertia;
    this->bodyid = bodyid;
}

void bodynode::print_info(){
    cout << endl << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

    cout << "Body id" << endl;
    cout << this->bodyid << endl << endl;

    cout << "Associated Joint id" << endl;
    cout << this->joint << endl << endl;

    cout << "Relative position" << endl;
    cout << this->rel_pos << endl << endl;

    cout << "Relative rotation matrix" << endl;
    cout << this->rel_rot << endl << endl;

    cout << "Relative transformation matrix" << endl;
    cout << this->rel_transform << endl << endl;

    cout << "Mass of body" << endl;
    cout << this->mass << endl << endl;

    cout << "Relative inertia center of mass position" << endl;
    cout << this->rel_inertia_com_pos << endl << endl;

    cout << "Relative inertia rotation matrix" << endl;
    cout << this->rel_inertia_rot << endl << endl;

    cout << "Diagonal Inertia" << endl;
    cout << this->diagonal_inertia << endl << endl;

    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl << endl;
}

// robot node class function definitions

//TODO: Store body information in the bodynode structure (maybe create initializer for it)
// 2) Save as vector<bodynode> until next joint is reached, initialize the joint node using the array of vectors
// Node: The root node of the joint tree is the world and the associated fixed joints to it.
jointnode::jointnode(int jointid,Eigen::VectorXd screw_axis):jointid(jointid),screw_axis(screw_axis){}

jointnode::jointnode(std::vector<bodynode*> bodies){
    this->attached_bodies = bodies;
    // this->compute_roatation();
}

// Compute inertial information.
void jointnode::set_attached_bodies(std::vector<bodynode*> input){
    this->attached_bodies = input;

    // Calculate spatial inertia for the newly attached bodies.
    // The first body of the attached bodies matrix is assumed to be body where the joint is attached.
    // Save to spatial inertia matrix

    Eigen::MatrixXd body_transform = Eigen::MatrixXd::Identity(4,4);
    this->spatial_inertia = Eigen::MatrixXd::Zero(6,6);

    if (this->attached_bodies[0]->bodyid == 0){
        for (int i=1; i < this->attached_bodies.size(); i++){

            if (i != 1){
                body_transform = body_transform * this->attached_bodies[i]->rel_transform;
            }

            // Calculate transform from the current body to the com of the body
            Eigen::MatrixXd com_transform(4,4);

            // cout << this->attached_bodies[i]->rel_inertia_rot.size() << endl;
            // cout << this->attached_bodies[i]->rel_inertia_com_pos << endl;
            // cout << Eigen::MatrixXd::Zero(1,3) << endl;

            com_transform << this->attached_bodies[i]->rel_inertia_rot, this->attached_bodies[i]->rel_inertia_com_pos, Eigen::MatrixXd::Zero(1,3), 1;

            com_transform = com_transform * body_transform;

   

            Eigen::MatrixXd spatial_inertia_com(6,6);
            spatial_inertia_com << this->attached_bodies[i]->diagonal_inertia, Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Zero(3,3), this->attached_bodies[i]->mass * Eigen::MatrixXd::Identity(3,3);

 

            if(com_transform == Eigen::MatrixXd::Identity(4,4)){
                this->spatial_inertia = this->spatial_inertia + spatial_inertia_com;
            }
            // else if (this->attached_bodies[i]->rel_inertia_rot == Eigen::Matrix::Identity(3,3)){

            //     Eigen::MatrixXd simplified_inertia (6,6);
            //     simplified_inertia << (this->attached_bodies[i])

            // } //TODO: Implement special case
            else{

                Eigen::MatrixXd X_star = transform_adjoint(com_transform,true);
                Eigen::MatrixXd X = transform_adjoint(com_transform.inverse());

                this->spatial_inertia += X_star * spatial_inertia_com * X;

            }



        }
    }
    else{

        for (int i=0; i < this->attached_bodies.size(); i++){

            if (i != 0){
                body_transform = body_transform * this->attached_bodies[i]->rel_transform;
            }

            // Calculate transform from the current body to the com of the body
            Eigen::MatrixXd com_transform(4,4);

            // cout << this->attached_bodies[i]->rel_inertia_rot.size() << endl;
            // cout << this->attached_bodies[i]->rel_inertia_com_pos << endl;
            // cout << Eigen::MatrixXd::Zero(1,3) << endl;

            com_transform << this->attached_bodies[i]->rel_inertia_rot, this->attached_bodies[i]->rel_inertia_com_pos, Eigen::MatrixXd::Zero(1,3), 1;

            com_transform = com_transform * body_transform;


            Eigen::MatrixXd spatial_inertia_com(6,6);
            spatial_inertia_com << this->attached_bodies[i]->diagonal_inertia, Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Zero(3,3), this->attached_bodies[i]->mass * Eigen::MatrixXd::Identity(3,3);

            if(com_transform == Eigen::MatrixXd::Identity(4,4)){
                this->spatial_inertia = this->spatial_inertia + spatial_inertia_com;
            }
            // else if (this->attached_bodies[i]->rel_inertia_rot == Eigen::Matrix::Identity(3,3)){

            //     Eigen::MatrixXd simplified_inertia (6,6);
            //     simplified_inertia << (this->attached_bodies[i])

            // } //TODO: Implement special case
            else{

                Eigen::MatrixXd X_star = transform_adjoint(com_transform,true);
                Eigen::MatrixXd X = transform_adjoint(com_transform.inverse());

                this->spatial_inertia += X_star * spatial_inertia_com * X;

            }

        }

    }

}

bodynode* jointnode::get_attached_body(){
    return this->attached_bodies[0];
}

// This function assumes that all the bodies are 
void jointnode::set_rotation_variables(Eigen::MatrixXd R_p_i, Eigen::MatrixXd T_p_i, Eigen::MatrixXd R_w_i, Eigen::MatrixXd T_w_i){
    this->R_p_i = R_p_i * this->attached_bodies[0]->rel_rot;
    this->T_p_i = T_p_i * this->attached_bodies[0]->rel_transform;
    this->R_w_i = R_w_i * this->attached_bodies[0]->rel_rot;
    this->T_w_i = T_w_i * this->attached_bodies[0]->rel_transform;
}

// The parent node is the currnet node. The child node of the parent is being updated based on the stored information.
void jointnode::pass_info_to_child(){
    // cout << "passing info on" << endl;
    // Calculate global and local transformation matrix. Only difference is with the global one as it take into acount the global rot and trans of the parent.
    if (this->parents.size() != 0){

        jointnode* p = static_cast<jointnode*>(parents[0]);
        Eigen::MatrixXd body_rotation_rel = Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd body_transform_rel = Eigen::MatrixXd::Identity(4,4);
        Eigen::MatrixXd body_rotation_global = this->R_w_i;
        Eigen::MatrixXd body_transform_global = this->T_w_i;
        // Relative transformation and rotation first
        for (int i =1; i < this->attached_bodies.size(); i++){ //starts at i+1 since the current body the joint is attached to is already considered.
            body_rotation_rel = body_rotation_rel * this->attached_bodies[i]->rel_rot;
            body_transform_rel = body_transform_rel * this->attached_bodies[i]->rel_transform;
            body_rotation_global = body_rotation_global * this->attached_bodies[i]->rel_rot;
            body_transform_global = body_transform_global * this->attached_bodies[i]->rel_transform;
        }

        for (int i=0; i < this->children.size(); i++){
            static_cast<jointnode*>(this->children[i])->set_rotation_variables(body_rotation_rel,body_transform_rel,body_rotation_global,body_transform_global);
        }

    }
    else{ //world joint node
        // cout << "beginning first info pass" << endl;
        this->R_w_i = Eigen::MatrixXd::Identity(3,3);
        this->T_w_i = Eigen::MatrixXd::Identity(4,4);

        Eigen::MatrixXd body_rotation = Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd body_transform = Eigen::MatrixXd::Identity(4,4);
        for (int i =1; i <this->attached_bodies.size(); i++){

            cout << this->attached_bodies[i]->rel_rot.size() << endl;
            body_rotation = body_rotation * this->attached_bodies[i]->rel_rot;
            body_transform = body_transform * this->attached_bodies[i]->rel_transform;
        }

        for (int i=0; i < this->children.size(); i++){
            static_cast<jointnode*>(this->children[i])->set_rotation_variables(body_rotation,body_transform,body_rotation,body_transform);
        }
        // cout << "Initial global node calculating correctly" << endl;
    }

}

void jointnode::print_info(){

    cout << endl << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

    cout << "Relative Rotation matrix from parent joint" << endl;
    cout << this->R_p_i << endl << endl;

    cout << "Relative transformation matrix from parent joint" << endl;
    cout << this->T_p_i << endl << endl;

    cout << "Rotation matrix from world joint" << endl;
    cout << this->R_w_i << endl << endl;

    cout << "Rotation matrix from world joint" << endl;
    cout << this->T_w_i << endl << endl;

    cout << "Rotation matrix from joint to com" << endl;
    cout << this->attached_bodies[0]->rel_inertia_rot << endl << endl;

    cout << "Position from joint to com" << endl;
    cout << this->attached_bodies[0]->rel_inertia_com_pos << endl << endl;

    cout << "Inertial Matrix" << endl;
    cout << this->spatial_inertia << endl;

    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

}


void robotModel::setup_joint_tree(){

    // cout << endl << endl << endl << "DEBUGGING THE JOINT SETUP" << endl << endl;
    queue<jointnode*> bfsq;

    jointnode* ptr = this->root_joint;
    ptr->pass_info_to_child();


    // cout << "children size: " << ptr->children.size() << endl;

    for (int i = 0; i<  ptr->children.size(); i++){
        // cout << "pushing child" << endl;
        bfsq.push(static_cast<jointnode*>(ptr->children[i]));
    }
    


    while(!bfsq.empty()){
        // cout << "Looping" << endl;
        ptr = bfsq.front();
        bfsq.pop();
        ptr->pass_info_to_child();
        for (int i = 0; i < ptr->children.size(); i++){
            bfsq.push(static_cast<jointnode*>(ptr->children[i]));
        }
    }

    // cout << endl << "----------------------------------" << endl;
}


// TODO: This does not handle robot arms with bodies not following a strict parent child order,
// Meaning if a particular 
robotModel::robotModel(mjModel* model, mjData* data){

    cout << "Robot model constructor called." << endl;
    
    return_info(model);  

    // Fill map of body and joint ids
    map<int,int> body_joint_map;
    for (int i =0; i < model->nbody; i++){
        body_joint_map[i] = -1;
    }
    for (int i =0; i < model->njnt; i++){
        body_joint_map[model->jnt_bodyid[i]] = i+1;
    }
    map<int,int> body_to_joint_map;
    body_to_joint_map[0] = 0;

    // Declare body struct vector
    vector<bodynode*> bodynode_vec;
    bodynode* world = new bodynode(0);
    bodynode_vec.push_back(world);
    this->root_body = bodynode_vec[0];

    // Begin Creating nodes
    // Start at body 1 since body 0 is the world
    // body_parentid to get the current bodies parent id. 
    int curr_joint_idx = 0;
    for (int i=1; i < model->nbody; i++){
        
        // Calculate relative position, rotation and transformation (when the robot is in the base position)
        Eigen::VectorXd rel_pos = mjtNum_to_eigenvec(model->body_pos,3,(3*i));
        Eigen::MatrixXd rel_rot = quat_to_rmat(mjtNum_to_eigenvec(model->body_quat,4,(4*i)));
        Eigen::MatrixXd rel_transform (4,4);
        rel_transform << rel_rot, rel_pos, Eigen::MatrixXd::Zero(1,3), 1;

        // Calculate the relative, COM position, inertia fram orientation and diagonal inertia (in the base position)
        double mass = model->body_mass[i];
        Eigen::VectorXd rel_inertia_pos = mjtNum_to_eigenvec(model->body_ipos,3,(3*i));
        Eigen::MatrixXd rel__inertia_rot = quat_to_rmat(mjtNum_to_eigenvec(model->body_iquat,4,(4*i)));
        Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(3,3);
        inertia(0,0) = model->body_inertia[(i*3)];
        inertia(1,1) = model->body_inertia[(i*3)+1];
        inertia(2,2) = model->body_inertia[(i*3)+2];

        // Create body structure
        bodynode* body = new bodynode(rel_pos,rel_rot,rel_transform, mass,rel_inertia_pos,rel__inertia_rot,inertia, i);
        bodynode_vec.push_back(body);

        // Create connection between parent and new node in the body tree. 
        int parent_body_id = model->body_parentid[i];
        bodynode* parent = bodynode_vec[parent_body_id];
        parent->children.push_back(body);
        body->parents.push_back(parent);


        // Check if there is a joint at the current body
        if (body_joint_map[i] != -1){
            body_to_joint_map[i] = body_joint_map[i];
            body->joint = body_joint_map[i];
        }
        else{ //set the joint of the current body as the joint of the parent
            body_to_joint_map[i] = body_to_joint_map[parent_body_id];
            body->joint = body_to_joint_map[parent_body_id];
        }
        
        
    }

    for (int j = 0; j < body_to_joint_map.size(); j++){
        cout << "Body id: " << j << "   Joint id: " << body_to_joint_map[j] << endl;
    }


    // Modified dfs to create joint nodes used for the kinematic and dynamic solvers.
    // Do dfs through body, if joint is detected, go through queue in order of joint (so if at joint 0 and one child has joint one, the other has joint 0, )
    // Prioritize child 2 (so will probably need priority queue for this)
    // This should create the joint tree with all joints having pointers to its respective bodies.
    int current_joint_idx = 0;
    vector<bodynode*> associated_bodies;

    priority_queue<bodynode*, vector<bodynode*>, bodynode_comp> pq;

    bodynode* ptr = this->root_body;

    this->root_joint = new jointnode();
    jointnode* jptr = this->root_joint;

    associated_bodies.push_back(ptr);
    for (int i =0; i < ptr->children.size(); i++){
        pq.push(static_cast<bodynode*>(ptr->children[0]));
    }

    while(!pq.empty()){
        ptr = pq.top();
        pq.pop();
        
        if (ptr->joint != curr_joint_idx){
            current_joint_idx = ptr->joint;
            jptr->set_attached_bodies(associated_bodies);
            jointnode* joint_buf = new jointnode();
            jptr->children.push_back(joint_buf);
            joint_buf->parents.push_back(jptr);
            jptr = joint_buf;
            associated_bodies.clear();
        }

        associated_bodies.push_back(ptr);

        for (int i =0; i < ptr->children.size(); i++){
            pq.push(static_cast<bodynode*>(ptr->children[0]));
        }

    }

    if (associated_bodies.size() != 0){
        jptr->set_attached_bodies(associated_bodies);
        associated_bodies.clear();
    }


    //print body tree (USed for debugging)
    queue<bodynode*> qb;
    bodynode* bptr = this->root_body;
    cout << "bodyid: " << bptr->bodyid << endl;
    cout << "Associated joint id: " << bptr->joint << endl;
    for (int i=0; i<bptr->children.size(); i++){
        cout << "child" << to_string(i) << ": " << static_cast<bodynode*>(bptr->children[i])->bodyid << endl;
        cout << bptr->rel_pos << endl;
        qb.push(static_cast<bodynode*>(bptr->children[i]));
    }
    while(!qb.empty()){

        bptr = qb.front();
        qb.pop();
        cout << "bodyid: " << bptr->bodyid << endl;
        cout << "Associated joint id: " << bptr->joint << endl;
        for (int i=0; i<bptr->children.size(); i++){
            cout << "child" << to_string(i) << ": " << static_cast<bodynode*>(bptr->children[i])->bodyid << endl;
            qb.push(static_cast<bodynode*>(bptr->children[i]));
        }
    }
    
    // The joint and body trees have been created. The inertia calculations should be handled in the joint node class.
  
    

    // Calculate the inertia and transform for all the joints relative to eachother.
    this->setup_joint_tree();
    
}

// Prints out the joint tree information stored in the robot model
void robotModel::print_joint_tree(){

    // DFS
    stack<jointnode*> dfs;
    
    jointnode* ptr = this->root_joint;
    ptr->print_info();
    for (int i =0; i < ptr->children.size(); i++){
        dfs.push(static_cast<jointnode*>(ptr->children[i]));
    }

    while(!dfs.empty()){

        ptr = dfs.top();
        dfs.pop();

        ptr->print_info();

        for (int i =0; i < ptr->children.size(); i++){
            dfs.push(static_cast<jointnode*>(ptr->children[i]));
        }

    }


}

void robotModel::print_body_tree(){

    // DFS
    stack<bodynode*> dfs;
    
    bodynode* ptr = this->root_body;
    ptr->print_info();
    for (int i =0; i < ptr->children.size(); i++){
        dfs.push(static_cast<bodynode*>(ptr->children[i]));
    }

    while(!dfs.empty()){

        ptr = dfs.top();
        dfs.pop();

        ptr->print_info();

        for (int i =0; i < ptr->children.size(); i++){
            dfs.push(static_cast<bodynode*>(ptr->children[i]));
        }

    }


}