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
    // this->screw_axis = screw_axis;
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
        this->screw_axis = Eigen::VectorXd::Zero(3); //Set world joint screw axis = 0
        for (int i=1; i < this->attached_bodies.size(); i++){
            if (i != 1){
                body_transform = body_transform * this->attached_bodies[i]->rel_transform;
            }

            // Calculate transform from the current body to the com of the body
            Eigen::MatrixXd com_transform(4,4);
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

int jointnode::getid(){
    return this->jointid;
}

Eigen::VectorXd jointnode::get_screw_axis(){
    return this->screw_axis;
}

Eigen::MatrixXd jointnode::get_world_transform(){
    return this->T_w_i;
}

Eigen::MatrixXd jointnode::get_spatial_inertia(){
    return this->spatial_inertia;
}

Eigen::MatrixXd jointnode::get_local_transformation(){
    return this->T_p_i;
}

void jointnode::set_screw_axis(Eigen::VectorXd axis){
    this->screw_axis = axis;
}

// The parent node is the currnet node. The child node of the parent is being updated based on the stored information.
void jointnode::pass_info_to_child(){

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
    }

}

void jointnode::print_info(){

    cout << endl << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

    cout << "Joint id: " << this->jointid << endl << endl;

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

    cout << "Screw axis" << endl;
    cout << this->screw_axis << endl;

    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

}


void robotModel::setup_joint_tree(){

    // Get size of the joint tree
    int tree_size = 1;
    queue<jointnode*> bfsq;

    jointnode* ptr = this->root_joint;
    ptr->pass_info_to_child();

    for (int i = 0; i<  ptr->children.size(); i++){
        bfsq.push(static_cast<jointnode*>(ptr->children[i]));
    }
    
    while(!bfsq.empty()){
        tree_size += 1;
        ptr = bfsq.front();
        bfsq.pop();
        ptr->pass_info_to_child();
        ptr->set_screw_axis(this->joint_screw_axis.at(ptr->getid()));
        for (int i = 0; i < ptr->children.size(); i++){
            bfsq.push(static_cast<jointnode*>(ptr->children[i]));
        }
    }

    this->joint_tree_size = tree_size;

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

    this->root_joint = new jointnode(0);
    jointnode* jptr = this->root_joint;

    associated_bodies.push_back(ptr);
    for (int i =0; i < ptr->children.size(); i++){
        pq.push(static_cast<bodynode*>(ptr->children[i]));
    }

    while(!pq.empty()){
        ptr = pq.top();
        pq.pop();
        
        if (ptr->joint != curr_joint_idx){
            current_joint_idx = ptr->joint;
            jptr->set_attached_bodies(associated_bodies);
            jointnode* joint_buf = new jointnode(ptr->joint);
            jptr->children.push_back(joint_buf);
            joint_buf->parents.push_back(jptr);
            jptr = joint_buf;
            curr_joint_idx = ptr->joint;
            associated_bodies.clear();
        }

        associated_bodies.push_back(ptr);

        for (int i =0; i < ptr->children.size(); i++){
            pq.push(static_cast<bodynode*>(ptr->children[i]));
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
    
    // Store the screw axis of the joints in this class
    //TODO: For now assume all joints have a pitch of 0 since its robotic arm, add some code to ensure
    //FIXME: In the real robotic hand, this joint data will be provided by sensors.
    //different types of joints (like the piston joints) get a appropriate linear velocity value.
    // Right now all the linear velocity is set to 0
    this->joint_screw_axis.push_back(Eigen::VectorXd::Zero(6)); //screw axis for the "world joint"
    for (int i =0; i < model->njnt; i++){

        Eigen::VectorXd buf_vec(6);
        buf_vec << model->jnt_axis[(i*3)], model->jnt_axis[(i*3)+1], model->jnt_axis[(i*3)+2], 0,0,0;
        this->joint_screw_axis.push_back(buf_vec);
        
        
    }
    

    // Calculate the inertia and transform for all the joints relative to eachother.
    this->setup_joint_tree();
    
}

// This calculates the torque which needs to be applied to each joint.
Eigen::VectorXd robotModel::RNEA(Eigen::MatrixXd x){

    // Seperate input to make it a bit easier.
    vector<int> joint_indicies(this->joint_tree_size,0);
    for (int i =0; i < joint_indicies.size(); i++){
        joint_indicies.at(i) = i;
    }
   
    Eigen::VectorXd j = x(0,joint_indicies);
    Eigen::VectorXd j_dot = x(1,joint_indicies);
    Eigen::VectorXd j_ddot = x(2,joint_indicies);
    Eigen::VectorXd ext_force = x(3, joint_indicies);
    // Note, the force described in the world joint index is the gravity calculation which is to be used for all the force and torque calculations.
    
    // Step 1: Use BFS to calculate the transformation matricies using the joint tree (recall the joint tree gives rotation and inertia at base position)
    // Step 2: Using reclaculated transformation matricies, calculate the local acceleration and velocities of each joint using forward dynamics
    vector<Eigen::VectorXd> loc_vec(this->joint_tree_size);
    vector<Eigen::VectorXd> loc_acc(this->joint_tree_size);
    vector<Eigen::VectorXd> loc_force(this->joint_tree_size);
    vector<double> loc_torque(this->joint_tree_size);

    // Keep track of updated global rotation matricies.
    vector<Eigen::MatrixXd> updated_world_transforms(this->joint_tree_size,Eigen::MatrixXd::Zero(4,4));
    updated_world_transforms.at(0) = Eigen::MatrixXd::Identity(4,4);

    // Get transformation matrix from world coordinates to each specific joint coordinates 
    // Will need to calculate the updated matrix transformation for each joint. Use Rodrigues' formula to calculate the matrix exponential easier.
    vector<Eigen::MatrixXd> global_trans(this->joint_tree_size);

    // world accel and velocity will always be zero
    loc_vec.at(0) = (Eigen::VectorXd::Zero(6)); 
    loc_acc.at(0) = (Eigen::VectorXd::Zero(6));
    loc_force.at(0) = (Eigen::VectorXd::Zero(6));
    loc_torque.at(0) = (0);

    // cout << "All Vectors initialized" << endl;
    queue<jointnode*> q; 
    queue<jointnode*> backward_q;
    jointnode* ptr = this->root_joint;

    for (int i =0; i < ptr->children.size(); i++){
        q.push(static_cast<jointnode*>(ptr->children[i]));
    }

    // Calculate spatial acceleration and velocity and updated global transformation matricies.
    while(!q.empty()){
        ptr = q.front();
        q.pop();

        int curr_jnt_id = ptr->getid();

        // Get adjoint transformation
        int parentid = static_cast<jointnode*>(ptr->parents[0])->getid();
        Eigen::MatrixXd transform_local = ptr->get_local_transformation() * exponential_rotation_spatial(ptr->get_screw_axis(),j(ptr->getid()));
        Eigen::MatrixXd adj_local_trans = transform_adjoint(transform_local.inverse());
        updated_world_transforms.at(ptr->getid()) = updated_world_transforms.at(static_cast<jointnode*>(ptr->parents.at(0))->getid()) * transform_local;

        // Calculate local velocity and store it in loc_vec
        loc_vec.at(curr_jnt_id) = adj_local_trans * loc_vec.at(parentid) + this->joint_screw_axis[curr_jnt_id] * j_dot(curr_jnt_id);

        // Calculate local acceleration and store it in loc_acc
        loc_acc.at(curr_jnt_id) = adj_local_trans * loc_acc.at(parentid) + spatial_cross_product(loc_vec.at(curr_jnt_id), (this->joint_screw_axis[curr_jnt_id] * j_dot(curr_jnt_id)), false) + spatial_cross_product(loc_vec.at(parentid), (this->joint_screw_axis[curr_jnt_id]), false)* j_dot(curr_jnt_id) + this->joint_screw_axis[curr_jnt_id] * j_ddot(curr_jnt_id);

        // Put the "leaf" nodes of the joints into the backwards pass to calculate the torque.
        if (ptr->children.size() == 0){
            backward_q.push(ptr);
        }
        else{
            for (int i =0; i < ptr->children.size(); i++){
                q.push(static_cast<jointnode*>(ptr->children[i]));
            }
        }

    }

    // Step 3 backwards pass to calculate force and torque.
    //FIXME: for now, the external forces calculations are being ignored. Add this in the future, for now only consider gravity.
    Eigen::VectorXd spatial_gravity(6);
    spatial_gravity << 0,0,0, 0,0,-9.81; // This should be replaced by the x input. But for now, this will be the gravity in the world frame. 

    //Calculate the force and torque at each point.
    while(!backward_q.empty()){

        ptr = backward_q.front();
        backward_q.pop();

        if (ptr->getid() != 0){

            // Eigen::VectorXd spatial_F_g = ptr->get_spatial_inertia() * transform_adjoint((ptr->get_world_transform() *exponential_rotation_spatial(ptr->get_screw_axis(),j(ptr->getid()))  ).inverse()) * spatial_gravity;
            Eigen::VectorXd spatial_F_g = ptr->get_spatial_inertia() * transform_adjoint(updated_world_transforms.at(ptr->getid()),true) * spatial_gravity;
            Eigen::VectorXd spatial_F_children(6);
            spatial_F_children = Eigen::VectorXd::Zero(6);
            for (int i = 0; i < ptr->children.size(); i++){

                // TODO: Taking the adjoint of the transformation of the local transformation works... not too sure why.
                jointnode* child = static_cast<jointnode*>(ptr->children[i]);
                cout << child->get_local_transformation().rows() << ", " << child->get_local_transformation().cols() << endl;
                Eigen::MatrixXd p_to_i = child->get_local_transformation() * exponential_rotation_spatial(child->get_screw_axis(),j(child->getid())) ;
                spatial_F_children += transform_adjoint(p_to_i,true) * loc_force.at(static_cast<jointnode*>(ptr->children[i])->getid());
            }

            // Calculate the force at each joint 
            Eigen::VectorXd spatial_F = ptr->get_spatial_inertia() * loc_acc.at(ptr->getid()) - spatial_cross_product(loc_vec.at(ptr->getid()),(ptr->get_spatial_inertia()* loc_vec.at(ptr->getid()))  ,true) - spatial_F_g + spatial_F_children; 

            loc_force.at(ptr->getid()) = spatial_F;

            // loc_torque.at(ptr->getid()) = ptr->get_screw_axis().transpose() * spatial_F;
            loc_torque.at(ptr->getid()) = spatial_F.transpose() * ptr->get_screw_axis();

            // Add parent classes to the queue (Since this is backwards pass)
            for (int i =0; i < ptr->parents.size(); i++){
                backward_q.push(static_cast<jointnode*>(ptr->parents[i]));
            }

        }

    }

    Eigen::VectorXd out(loc_torque.size());
    out = Eigen::VectorXd::Zero(loc_torque.size());
    for (int i = 0; i < loc_torque.size(); i++){
        out(i) = loc_torque.at(i);
    }

    return out;

}


// The input x matrix contains joint positions, joint velcoity, torque (u) and external forces
// This function calculates the acceleration of the joints using Recursive Forward Dynamics algorithm. 
// NOTE: This assumes an open chain system.
Eigen::VectorXd robotModel::FNEA(Eigen::MatrixXd x){

    // Split input matrix into specific vectors
    Eigen::MatrixXd q = x.block(0,0,1,this->joint_tree_size);
    Eigen::MatrixXd q_dot = x.block(1,0,1,this->joint_tree_size);
    Eigen::MatrixXd torque = x.block(2,0,1,this->joint_tree_size);
    Eigen::MatrixXd ext_force = x.block(3,0,1,this->joint_tree_size);

    // Forward Pass variables
    vector<Eigen::MatrixXd> loc_vec(this->joint_tree_size,Eigen::MatrixXd::Zero(6,1));
    vector<Eigen::MatrixXd> n_i(this->joint_tree_size,Eigen::MatrixXd::Zero(6,1));

    // Backwards pass variables
    vector<Eigen::MatrixXd> augmented_inertia(this->joint_tree_size,Eigen::MatrixXd::Zero(6,6));
    vector<Eigen::MatrixXd> augmented_bias(this->joint_tree_size,Eigen::MatrixXd::Zero(6,1));
    vector<double> gamma_term (this->joint_tree_size,0);
    vector<Eigen::MatrixXd> beta_term (this->joint_tree_size,Eigen::MatrixXd::Zero(6,1));
    vector<Eigen::MatrixXd> pi_term (this->joint_tree_size,Eigen::MatrixXd::Zero(6,6));

    // output pass variables
    vector<Eigen::VectorXd> spatial_F(this->joint_tree_size,Eigen::VectorXd::Zero(6));
    vector<double> q_ddot(this->joint_tree_size,0);
    vector<Eigen::VectorXd> spatial_acc(this->joint_tree_size,Eigen::VectorXd::Zero(6));


    // Keep track of updated local rotation matricies 
    vector<Eigen::MatrixXd> loc_rot_map (this->joint_tree_size,Eigen::MatrixXd::Zero(4,4));

    // Keep track of updated global rotation matricies.
    vector<Eigen::MatrixXd> updated_world_transforms(this->joint_tree_size,Eigen::MatrixXd::Zero(4,4));
    updated_world_transforms.at(0) = Eigen::MatrixXd::Identity(4,4);

    queue<jointnode*> q_for; 
    queue<jointnode*> backward_q;
    jointnode* ptr = this->root_joint;

    for (int i =0; i < ptr->children.size(); i++){
        q_for.push(static_cast<jointnode*>(ptr->children[i]));
    }

    // Forward pass 1: Calculate spatial velcity, transformatin matricies and n_i term.
    while(!q_for.empty()){
        
        ptr = q_for.front();
        q_for.pop();
        int curr_jnt_id = ptr->getid();

        // Get adjoint transformation from all parents 
        int parentid = static_cast<jointnode*>(ptr->parents[0])->getid();
        Eigen::MatrixXd transform_local = ptr->get_local_transformation() * exponential_rotation_spatial(ptr->get_screw_axis(),q(ptr->getid()));
        Eigen::MatrixXd adj_local_trans = transform_adjoint(transform_local.inverse());

        // Calculate updated joint transformation matricies
        loc_rot_map[curr_jnt_id] = transform_local;
        updated_world_transforms.at(ptr->getid()) = updated_world_transforms.at(static_cast<jointnode*>(ptr->parents.at(0))->getid()) * transform_local;

        // Calculate forward pass spatial velocity and n_i term
        loc_vec.at(curr_jnt_id) = adj_local_trans * loc_vec.at(parentid) + this->joint_screw_axis[curr_jnt_id] * q_dot(curr_jnt_id);
        n_i.at(curr_jnt_id) = spatial_cross_product(loc_vec.at(curr_jnt_id), (this->joint_screw_axis[curr_jnt_id] * q_dot(curr_jnt_id)), false) + spatial_cross_product(loc_vec.at(parentid), (this->joint_screw_axis[curr_jnt_id]), false)* q_dot(curr_jnt_id);

        // Put the "leaf" nodes of the joints into the backwards pass to calculate the torque.
        if (ptr->children.size() == 0){
            backward_q.push(ptr);
        }
        else{
            for (int i =0; i < ptr->children.size(); i++){
                q_for.push(static_cast<jointnode*>(ptr->children[i]));
            }
        }
    }


    // Backwards pass: Calculate augmented Inertia term, bias force, and intermediate terms, sigma,alpha and beta for each joint variable.
    while(!backward_q.empty()){

        ptr = backward_q.front();
        backward_q.pop();

        if (ptr->getid() != 0){

            // common variables
            int curr_id = ptr->getid();
            Eigen::MatrixXd S_i = ptr->get_screw_axis();
            Eigen::MatrixXd I_i = ptr->get_spatial_inertia();

            // TODO: Right now only external force being considered is gravity. Will need to change this in the future.
            Eigen::VectorXd spatial_gravity(6);
            spatial_gravity << 0,0,0, 0,0,-9.81;
            Eigen::VectorXd external_F = I_i * transform_adjoint(updated_world_transforms.at(curr_id),true) * spatial_gravity;

            augmented_inertia[curr_id] = ptr->get_spatial_inertia();
            augmented_bias[curr_id] = - spatial_cross_product(loc_vec[curr_id], (ptr->get_spatial_inertia() * loc_vec[curr_id] ), true) - external_F; 

            for (int i = 0; i < ptr->children.size(); i++){
                int child_id = static_cast<jointnode*>(ptr->parents[i])->getid();
                augmented_inertia[curr_id] = augmented_inertia[curr_id] + (transform_adjoint(loc_rot_map[child_id].inverse(),true) * pi_term[child_id] * transform_adjoint(loc_rot_map[child_id].inverse(),false) ); 
                augmented_bias[curr_id] = augmented_bias[curr_id] + (transform_adjoint(loc_rot_map[child_id].inverse(),true) * beta_term[child_id]);
            }

            gamma_term[curr_id] = (S_i.transpose() * augmented_inertia[curr_id] * S_i).inverse()(0,0);
            pi_term[curr_id] = augmented_inertia[curr_id] - (augmented_inertia[ptr->getid()] * ptr->get_screw_axis() * gamma_term[ptr->getid()] * ptr->get_screw_axis().transpose() * augmented_inertia[ptr->getid()]);
            beta_term[curr_id] = augmented_bias[curr_id] + augmented_inertia[curr_id] * (n_i[curr_id] + (S_i * gamma_term[curr_id]) * (torque(0,curr_id) - (S_i.transpose() * (augmented_inertia[curr_id] * n_i[curr_id] + augmented_bias[curr_id]))(0,0)  )  );

            // Add parent classes to the queue (Since this is backwards pass)
            for (int i =0; i < ptr->parents.size(); i++){
                backward_q.push(static_cast<jointnode*>(ptr->parents[i]));
            }
        }
    }


    // Second forward pass. Calculate acceleration, spatial acceleration, and Force.
    ptr = this->root_joint;

    for (int i =0; i < ptr->children.size(); i++){
        q_for.push(static_cast<jointnode*>(ptr->children[i]));
    }
    
    while(!q_for.empty()){
        ptr = q_for.front();
        q_for.pop();

        // Common variables
        int curr_id = ptr->getid();
        int parent_id = static_cast<jointnode*>(ptr->parents[0])->getid();
        Eigen::VectorXd S_i = ptr->get_screw_axis();

        q_ddot[curr_id] = gamma_term[curr_id]* (torque(0,curr_id) - ((S_i.transpose() * augmented_inertia[curr_id])*(transform_adjoint(loc_rot_map[curr_id].inverse(),false) * spatial_acc[parent_id] + n_i[curr_id] ))(0,0) - (S_i.transpose() * augmented_bias[curr_id])(0,0));
        spatial_acc[curr_id] = transform_adjoint(loc_rot_map[curr_id].inverse(),false) * spatial_acc[parent_id] + S_i * q_ddot[curr_id] + n_i[curr_id];
        spatial_F[curr_id] = augmented_inertia[curr_id] * spatial_acc[curr_id] + augmented_bias[curr_id];

    }

    Eigen::VectorXd out_acc (this->joint_tree_size);
    for (int i = 0; i < this->joint_tree_size; i++){
        out_acc(i) = q_ddot[i];
    }
    return out_acc;

}


// // Same as FNEA function except the matricies handle adouble scalar type used by psopt
// Eigen::VectorXad robotModel::FNEA_PSOPT(Eigen::MatrixXad x){

// // Split input matrix into specific vectors
//     Eigen::VectorXad q = x.block(0,0,1,this->joint_tree_size);
//     Eigen::VectorXad q_dot = x.block(1,0,1,this->joint_tree_size);
//     Eigen::VectorXad torque = x.block(2,0,1,this->joint_tree_size);
//     Eigen::VectorXad ext_force = x.block(3,0,1,this->joint_tree_size);


//     // Forward Pass variables
//     vector<Eigen::MatrixXad> loc_vec(this->joint_tree_size,Eigen::MatrixXad::Zero(6,1));
//     vector<Eigen::MatrixXad> n_i(this->joint_tree_size,Eigen::MatrixXad::Zero(6,1));

//     // Backwards pass variables
//     vector<Eigen::MatrixXad> augmented_inertia(this->joint_tree_size,Eigen::MatrixXad::Zero(6,6));
//     vector<Eigen::MatrixXad> augmented_bias(this->joint_tree_size,Eigen::MatrixXad::Zero(6,1));
//     vector<double> gamma_term (this->joint_tree_size,0);
//     vector<Eigen::MatrixXad> beta_term (this->joint_tree_size,Eigen::MatrixXad::Zero(6,1));
//     vector<Eigen::MatrixXad> pi_term (this->joint_tree_size,Eigen::MatrixXad::Zero(6,6));

//     // output pass variables
//     vector<Eigen::VectorXad> spatial_F(this->joint_tree_size,Eigen::VectorXad::Zero(6));
//     vector<double> q_ddot(this->joint_tree_size,0);
//     vector<Eigen::VectorXad> spatial_acc(this->joint_tree_size,Eigen::VectorXad::Zero(6));


//     // Keep track of updated local rotation matricies 
//     vector<Eigen::MatrixXad> loc_rot_map (this->joint_tree_size,Eigen::MatrixXad::Zero(4,4));

//     // Keep track of updated global rotation matricies.
//     vector<Eigen::MatrixXad> updated_world_transforms(this->joint_tree_size,Eigen::MatrixXad::Zero(4,4));
//     updated_world_transforms.at(0) = Eigen::MatrixXad::Identity(4,4);

//     queue<jointnode*> q_for; 
//     queue<jointnode*> backward_q;
//     jointnode* ptr = this->root_joint;

//     for (int i =0; i < ptr->children.size(); i++){
//         q_for.push(static_cast<jointnode*>(ptr->children[i]));
//     }

//     // Forward pass 1: Calculate spatial velcity, transformatin matricies and n_i term.
//     while(!q_for.empty()){
        
//         ptr = q_for.front();
//         q_for.pop();
//         int curr_jnt_id = ptr->getid();

//         // Get adjoint transformation from all parents 
//         int parentid = static_cast<jointnode*>(ptr->parents[0])->getid();
//         Eigen::MatrixXad transform_local = Matrix_dtad(ptr->get_local_transformation()) * exponential_rotation_spatial_adouble(Vector_dtad(ptr->get_screw_axis()),q(ptr->getid()));
//         Eigen::MatrixXad adj_local_trans = transform_adjoint(transform_local.inverse());

//         // Calculate updated joint transformation matricies
//         loc_rot_map[curr_jnt_id] = transform_local;
//         updated_world_transforms.at(ptr->getid()) = updated_world_transforms.at(static_cast<jointnode*>(ptr->parents.at(0))->getid()) * transform_local;

//         // Calculate forward pass spatial velocity and n_i term
//         loc_vec.at(curr_jnt_id) = adj_local_trans * loc_vec.at(parentid) + this->joint_screw_axis[curr_jnt_id] * q_dot(curr_jnt_id);
//         n_i.at(curr_jnt_id) = spatial_cross_product(loc_vec.at(curr_jnt_id), (this->joint_screw_axis[curr_jnt_id] * q_dot(curr_jnt_id)), false) + spatial_cross_product(loc_vec.at(parentid), (this->joint_screw_axis[curr_jnt_id]), false)* q_dot(curr_jnt_id);

//         // Put the "leaf" nodes of the joints into the backwards pass to calculate the torque.
//         if (ptr->children.size() == 0){
//             backward_q.push(ptr);
//         }
//         else{
//             for (int i =0; i < ptr->children.size(); i++){
//                 q_for.push(static_cast<jointnode*>(ptr->children[i]));
//             }
//         }


//     }

//     // Backwards pass: Calculate augmented Inertia term, bias force, and intermediate terms, sigma,alpha and beta for each joint variable.
//     while(!backward_q.empty()){

//         ptr = backward_q.front();
//         backward_q.pop();

//         if (ptr->getid() != 0){

//             // common variables
//             int curr_id = ptr->getid();
//             Eigen::MatrixXad S_i = ptr->get_screw_axis().transpose();
//             Eigen::MatrixXad I_i = ptr->get_spatial_inertia();

//             // TODO: Right now only external force being considered is gravity. Will need to change this in the future.
//             Eigen::VectorXad spatial_gravity(6);
//             spatial_gravity << 0,0,0, 0,0,-9.81;
//             Eigen::VectorXad external_F = I_i * transform_adjoint(updated_world_transforms.at(curr_id),true) * spatial_gravity;

//             augmented_inertia[curr_id] = ptr->get_spatial_inertia();
//             augmented_bias[curr_id] = - spatial_cross_product(loc_vec[curr_id], (ptr->get_spatial_inertia() * loc_vec[curr_id] ), true) - external_F; 

//             for (int i = 0; i < ptr->children.size(); i++){

//                 int child_id = static_cast<jointnode*>(ptr->parents[i])->getid();

//                 augmented_inertia[curr_id] = augmented_inertia[curr_id] + (transform_adjoint(loc_rot_map[child_id].inverse(),true) * pi_term[child_id] * transform_adjoint(loc_rot_map[child_id].inverse(),false) ); 
//                 augmented_bias[curr_id] = augmented_bias[curr_id] + (transform_adjoint(loc_rot_map[child_id].inverse(),true) * beta_term[child_id]);

//             }

//             gamma_term[curr_id] = (S_i.transpose() * augmented_inertia[curr_id] * S_i).inverse()(0,0);
//             pi_term[curr_id] = augmented_inertia[curr_id] - (augmented_inertia[ptr->getid()] * ptr->get_screw_axis() * gamma_term[ptr->getid()] * ptr->get_screw_axis().transpose() * augmented_inertia[ptr->getid()]);
//             beta_term[curr_id] = augmented_bias[curr_id] + augmented_inertia[curr_id] * (n_i[curr_id] + (S_i * gamma_term[curr_id]) * (torque(0,curr_id) - (S_i.transpose() * (augmented_inertia[curr_id] * n_i[curr_id] + augmented_bias[curr_id]))(0,0)  )  );

//             // Add parent classes to the queue (Since this is backwards pass)
//             for (int i =0; i < ptr->parents.size(); i++){
//                 backward_q.push(static_cast<jointnode*>(ptr->parents[i]));
//             }

//         }

//     }

//     // Second forward pass. Calculate acceleration, spatial acceleration, and Force.
//     ptr = this->root_joint;

//     for (int i =0; i < ptr->children.size(); i++){
//         q_for.push(static_cast<jointnode*>(ptr->children[i]));
//     }
    
//     while(!q_for.empty()){

//         ptr = q_for.front();
//         q_for.pop();

//         // Common variables
//         int curr_id = ptr->getid();
//         int parent_id = static_cast<jointnode*>(ptr->parents[0])->getid();
//         Eigen::VectorXad S_i = ptr->get_screw_axis();


//         q_ddot[curr_id] = gamma_term[curr_id]* (torque(0,curr_id) - ((S_i.transpose() * augmented_inertia[curr_id])*(transform_adjoint(loc_rot_map[curr_id].inverse(),false) * spatial_acc[parent_id] + n_i[curr_id] ))(0,0) - (S_i.transpose() * augmented_bias[curr_id])(0,0));
//         spatial_acc[curr_id] = transform_adjoint(loc_rot_map[curr_id].inverse(),false) * spatial_acc[parent_id] + S_i * q_ddot[curr_id] + n_i[curr_id];
//         spatial_F[curr_id] = augmented_inertia[curr_id] * spatial_acc[curr_id] + augmented_bias[curr_id];


//     }

//     Eigen::VectorXad out_acc (this->joint_tree_size);

//     for (int i = 0; i < this->joint_tree_size; i++){
//         out_acc(i) = q_ddot[i];
//     }

//     return out_acc;

// }


// Returns the bodies associated with the joints in the joint tree.
vector<int> robotModel::return_joint_bodies(){
    vector<int> out (this->joint_tree_size,0);
    stack<jointnode*> dfs;
    jointnode* ptr = this->root_joint;
    ptr->print_info();
    for (int i =0; i < ptr->children.size(); i++){
        dfs.push(static_cast<jointnode*>(ptr->children[i]));
    }
    while(!dfs.empty()){
        ptr = dfs.top();
        dfs.pop();
        out.at(ptr->getid()) = ptr->get_attached_body()->bodyid;
        for (int i =0; i < ptr->children.size(); i++){
            dfs.push(static_cast<jointnode*>(ptr->children[i]));
        }
    }
    return out;
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

int robotModel::return_joint_tree_size(){
    return this->joint_tree_size;
}