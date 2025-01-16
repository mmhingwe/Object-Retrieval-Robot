#include <iostream>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <math.h>
#include <nlopt.h>
#include "controllers.h"
// #include <adolc/adolc_sparse.h>
// #include <adolc/adouble.h>
#include "pathplan.h"
// #include <psopt.h>

using namespace std;


global_RRT_control::global_RRT_control(mjModel* m, mjData* d,std::string planner_type){

    this->model = new robotModel(m,d);
    this->associated_joint_bodies = this->model->return_joint_bodies();
    this->saved_path = Eigen::MatrixXd::Zero(1,1);
    this->path_found = false;
    this->deleted_kdnodes = 0;

    //find constraints for the model to be used for path planning


    // Create the planner type
    if(planner_type == "rrt_base"){
        this->planner = static_cast<pathplan*>(new RRT());
    }
    else{
        this->planner = static_cast<pathplan*>(new RRT());
    }


    // TODO: SPecifiy the space of the environment
    // Space bounds (position limits), if joint is revolute, the space bound should be 0 to 360
    Eigen::MatrixXd space_const_new (4*m->njnt,3);
    space_const_new = Eigen::MatrixXd::Zero(4*m->njnt,3); //FIXME: should generalize this some more 
    for (int i = 0; i < 2*m->njnt; i++){
        if (i < m->njnt){
            Eigen::Vector3d buffer_l;
            Eigen::Vector3d buffer_u;
            buffer_l << i, 1, -M_PI;
            buffer_u << i, 2, M_PI;
            space_const_new(2*i,{0,1,2}) = buffer_l;
            space_const_new(2*i + 1, {0,1,2}) = buffer_u; 
        }
        else{
            // TODO: Right now the velocity constraints are just below 10. CHange this in the future to look more into the specifications of the robot
            Eigen::Vector3d buffer_l;
            Eigen::Vector3d buffer_u;
            buffer_l << i, 1, -2;
            buffer_u << i, 2, 2;
            space_const_new(2*i,{0,1,2}) = buffer_l;
            space_const_new(2*i + 1, {0,1,2}) = buffer_u; 
        }
    }
    this->space_bounds = space_const_new;



    // Determine constraints of the input robot model. (note, the way this controller is setup, the state of the robot is the position and velocity.)
    vector<Eigen::Vector3d> constraints_vec;
    
    // Joint Constraints


    //TODO: Take torque constraints into account.
    
    Eigen::MatrixXd new_const(constraints_vec.size(),3);
    new_const = Eigen::MatrixXd::Zero(constraints_vec.size(),3);

    for(int i = 0; i < constraints_vec.size(); i++){
        new_const(i,{0,1,2}) = constraints_vec.at(i);
    }

    cout << "constraints: " << endl;
    cout << new_const << endl;

    this->constraints = new_const;

    this->config_space = 2 * m->njnt; //joint positions, then velocities

}

// FIXME: Implement way to change the rrt learning rate and step size
// Find the path using the specified path planning algorithm. The space bounds and constraints should be specified earlier.
void global_RRT_control::find_path(Eigen::VectorXd init, Eigen::VectorXd goal){ //,vector<Eigen::MatrixXd> input_constraints == NULL){
    
    this->kdtree.reset();

    vector<Eigen::MatrixXd> buffer_constraints; //FIXME: Temporary no constraints, implement null constraints of this case in the constructor

    // if (input_constraints != NULL ){
    //     buffer_constraints = input_constraints;
    // }

    this->planner->set_perameters(init,goal,this->config_space, buffer_constraints, this->space_bounds);
    int success = this->planner->run(0.5,0.5,100000);
    this->saved_path = this->planner->return_path().transpose();
    kdtree.populate(this->saved_path);

}


// Find the next nearest point in the saved path. 
pair<Eigen::VectorXd,int> global_RRT_control::find_next_point(Eigen::VectorXd init){

    pair<Eigen::VectorXd,int> nearest_point = this->kdtree.find_nearest(init);
    cout << "FOUND NEAREST" << endl;
    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(nearest_point.first.size(),0,(nearest_point.first.size())-1);

    // nearest found point
    cout << "NEAREST FOUND POINT" << endl;
    for (int i = 0; i < nearest_point.first.size(); i++){
        cout << nearest_point.first(i) << " ";
    }
    cout << endl;
    Eigen::VectorXd buf = this->saved_path(this->saved_path.rows()-1,allidx);
    cout << "Goal POINT" << endl;
    for (int i = 0; i < buf.size(); i++){
        cout << buf(i) << " ";
    }
    cout << endl;


    // TODO: HAndle case where you are at the goal position. For now, just output a message.
    if (nearest_point.first == buf){
        cout << "goal point reached: " << nearest_point.second << endl;
        // return make_pair(this->saved_path(nearest_point.second,allidx),nearest_point.second);
        return make_pair(this->saved_path(nearest_point.second,allidx),nearest_point.second);
    }
    else{
        cout << "getting next point: " << endl;
        return make_pair(this->saved_path(nearest_point.second + 1,allidx),nearest_point.second+1);
    }

}

// TODO: Implement reset function
// void global_RRT_control::reset(){}



// This is only in joint space right now, it may be advantages to also implement a task space version of this.
// Constructor for mujoco simulation control
PID::PID(mjModel* m, mjData* d, std::string planner_type) :global_RRT_control(m,d,planner_type){
    
    // Set default pid variables to 0
    this->kp = 0;
    this->kd = 0;
    this->ki = 0;
    this->cum_sum = Eigen::VectorXd::Zero(m->njnt);
    this->num_jnt = m->njnt;

}

// Constructor for actual control using sensors.
// PID::PID(std::string xml_filename,std::string planner_type){}


// Destructor
PID::~PID(){}

// Set PID values
void PID::set_pid(double kp,double kd,double ki){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}


// NOTE: THIS IS USING JOINT SPACE
Eigen::VectorXd PID::get_control(mjModel* m, mjData* d, Eigen::VectorXd goal){

    // STEP 1: Calculate the current state of the robot in Eigen::VectorXd format (size should be #joints *2)
    // Format the current joint positions and velocities as init:
    Eigen::VectorXd init(m->njnt*2);
    init = Eigen::VectorXd::Zero(m->njnt*2);
    for (int i =0; i < m->njnt; i++){
        init(i) = d->qpos[i];
        init(m->njnt+i) = d->qvel[i];
    }


    // STEP 2: If no path has been created, generate a new path using the path planning algorithm. Else find the closest point on path.

    pair<Eigen::VectorXd, int> nextpt_pair;

    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(m->njnt*2,0,(m->njnt*2)-1);
    Eigen::ArrayXi pos_idx = Eigen::ArrayXi::LinSpaced(m->njnt,0,(m->njnt - 1));
    Eigen::ArrayXi vel_idx = Eigen::ArrayXi::LinSpaced(m->njnt,m->njnt,(m->njnt*2)-1);
     
    // Uncomment to follow trajectory
    if (!this->path_found){ 
        this->find_path(init,goal);
        nextpt_pair = make_pair(this->saved_path(1,allidx),1);
        this->path_found = true;
        this->kdtree.print_debug_info();
    }
    else{
        this->kdtree.print_debug_info();
        nextpt_pair = this->find_next_point(init);
    }
    
    for (int i = 0; i < nextpt_pair.second; i++){ 
        this->kdtree.delete_node(this->saved_path(i,allidx));
    }
    Eigen::VectorXd nextpt = nextpt_pair.first;

    // STEP 3: Calculate the required acceleration to get to the next desired point.
    Eigen::VectorXd acc(m->njnt);
    this->cum_sum += (nextpt(pos_idx) - init(pos_idx)) * (1/60);
    acc = ((nextpt(vel_idx) - init(vel_idx))*(1/60)) + this->kp* ((nextpt(pos_idx) - init(pos_idx))) + this->kd * ((nextpt(vel_idx) - init(vel_idx))) + this->ki * this->cum_sum;
    cout << ((nextpt(vel_idx) - init(vel_idx))) << endl << endl;
 

    // STEP 4: Take into account any contact forces being experienced.
    // TODO: Implement Force handling

    // STEP 5: Create a state x containing, the position, velocity and acceleration of all joints + the external forces.
    Eigen::MatrixXd x (4,m->njnt);
    x = Eigen::MatrixXd::Zero(4,m->njnt+1);
    Eigen::ArrayXi x_idx = Eigen::ArrayXi::LinSpaced(m->njnt,1,(m->njnt));
    x(0,x_idx) = init(pos_idx);
    x(0,0) = 0;
    x(1,x_idx) = init(vel_idx);
    x(1,0) = 0;
    x(2,x_idx) = acc;
    x(2,0) = 0;

    // STEP 6: Use RNEA to calculate the torque required to take the next step.
    Eigen::VectorXd torque = this->model->RNEA(x);

    // STEP 7: Return the torque.
    return torque;

}

Eigen::VectorXd PID::get_control(Eigen::VectorXd init, Eigen::VectorXd goal){

    // STEP 2: If no path has been created, generate a new path using the path planning algorithm. Else find the closest point on path.

    pair<Eigen::VectorXd, int> nextpt_pair;

    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(this->num_jnt*2,0,(this->num_jnt*2)-1);
    Eigen::ArrayXi pos_idx = Eigen::ArrayXi::LinSpaced(this->num_jnt,0,(this->num_jnt - 1));
    Eigen::ArrayXi vel_idx = Eigen::ArrayXi::LinSpaced(this->num_jnt,this->num_jnt,(this->num_jnt*2)-1);
     
    // Uncomment to follow trajectory
    if (!this->path_found){ 
        this->find_path(init,goal);
        nextpt_pair = make_pair(this->saved_path(1,allidx),1);
        this->path_found = true;
        this->kdtree.print_debug_info();
    }
    else{
        this->kdtree.print_debug_info();
        nextpt_pair = this->find_next_point(init);
    }
    
    for (int i = 0; i < nextpt_pair.second; i++){ 
        this->kdtree.delete_node(this->saved_path(i,allidx));
    }
    Eigen::VectorXd nextpt = nextpt_pair.first;

    // STEP 3: Calculate the required acceleration to get to the next desired point.
    Eigen::VectorXd acc(this->num_jnt);
    this->cum_sum += (nextpt(pos_idx) - init(pos_idx)) * (1/60);
    acc = ((nextpt(vel_idx) - init(vel_idx))*(1/60)) + this->kp* ((nextpt(pos_idx) - init(pos_idx))) + this->kd * ((nextpt(vel_idx) - init(vel_idx))) + this->ki * this->cum_sum;
    cout << ((nextpt(vel_idx) - init(vel_idx))) << endl << endl;
 

    // STEP 4: Take into account any contact forces being experienced.
    // TODO: Implement Force handling

    // STEP 5: Create a state x containing, the position, velocity and acceleration of all joints + the external forces.
    Eigen::MatrixXd x (4,this->num_jnt);
    x = Eigen::MatrixXd::Zero(4,this->num_jnt+1);
    Eigen::ArrayXi x_idx = Eigen::ArrayXi::LinSpaced(this->num_jnt,1,(this->num_jnt));
    x(0,x_idx) = init(pos_idx);
    x(0,0) = 0;
    x(1,x_idx) = init(vel_idx);
    x(1,0) = 0;
    x(2,x_idx) = acc;
    x(2,0) = 0;

    // STEP 6: Use RNEA to calculate the torque required to take the next step.
    Eigen::VectorXd torque = this->model->RNEA(x);

    // STEP 7: Return the torque.
    return torque;

}



struct dynamic_struct{

    robotModel* m;
    int state_size;
    int timesteps;
    int pos; //pos 0 = x[0] - x[4], pos 1 = x[5] - x[9], etc
    int index;
    int dt;
    int num_jnt;
    Eigen::MatrixXd data;
    
    void reset(){
        this->pos = 0;
        this->index = 0;
        // delete this->data;
        this->data = Eigen::MatrixXd::Zero(timesteps,num_jnt);
    }

    void assign(int index,Eigen::VectorXd input){
        this->data.block(index,0,1,this->num_jnt) = input.transpose();

    }
    dynamic_struct(int timesteps,int state_size,int num_jnt,double dt, robotModel* m){
        this->state_size = state_size;
        this->timesteps = timesteps;
        this->pos = 0;
        this->num_jnt = num_jnt;
        this->data = Eigen::MatrixXd::Zero(timesteps,num_jnt);
        this->index =0;
        this->dt = dt;
        this->m = m;
    }


};


MPC::MPC(mjModel* m, mjData* d,std::string planner_type):global_RRT_control(m,d,planner_type){
    this->num_jnt = this->model->return_joint_tree_size() - 1;
}


double MPC::cost_function(unsigned n, const double *x, double *grad, void *data){
    return 1.0;
}
double MPC::dynamic_constraint(unsigned n, const double *x, double *grad, void *data){

    dynamic_struct* d = (dynamic_struct*) data;

    if (d->pos == 0){
        // calculate the FNEA here and store it in the dynamic struct
        // 4xnum joints, 0: pos, 1: vel, 2: torque ctrl, 3:ext force
        // Fill the input matrix for FNEA for the previous point
        Eigen::MatrixXd input_x = Eigen::MatrixXd::Zero(4,d->num_jnt+1);
        for (int i = 0; i < (3*d->num_jnt); i++){
            if (i < d->num_jnt){
                input_x(0,i+1) = x[(d->index*3*d->num_jnt) + i];
            }
            else if (i < (2*d->num_jnt)){
                input_x(1,(i-d->num_jnt+1)) = x[(d->index*3*d->num_jnt) + i];
            }
            else{
                input_x(2,(i-(2*d->num_jnt)+1)) = x[(d->index*3*d->num_jnt) + i];
            }
        }
        //Calculate and store the result of FNEA
        Eigen::VectorXd out = d->m->FNEA(input_x);
        d->assign(d->index,out.segment(1,out.size()-1));
    }

    // calculate the constraints needed depending on the joint.
    double out;
    if (d->pos < d->num_jnt){
        out = (x[(d->index * 3 * d->num_jnt) + d->pos] + (x[(d->index * 3 * d->num_jnt) + d->pos + d->num_jnt] * d->dt) ) - x[((d->index+1) * 3 * d->num_jnt) + d->pos];
    }
    else{
        out = (x[(d->index * 3 * d->num_jnt) + d->pos] + (d->data(d->index,(d->pos - d->num_jnt)) * d->dt) ) - x[((d->index+1) * 3 * d->num_jnt) + d->pos];
    }

    d->pos += 1;
    if (d->pos == (2*d->num_jnt)){
        d->index += 1;
        d->pos = 0;
    }
    if (d->index == (d->timesteps)){
        d->reset();
    }

    return out;

}

double MPC::set_init_constraint(unsigned n, const double *x, double *grad, void *data){
    init_struct d_struct = *(init_struct*) data;
    return d_struct.data(d_struct.pos) - x[d_struct.pos];
}

double MPC::set_final_constraint(unsigned n, const double *x, double *grad, void *data){
    init_struct d_struct = *(init_struct*) data;
    return d_struct.data((2*6 - 1 - d_struct.pos)) - x[(-1 - d_struct.pos)];
}


Eigen::VectorXd MPC::optimize(Eigen::VectorXd init, Eigen::VectorXd goal,double T, double N){
    
    // Calculate State dimension
    int state_dim = (T/N) * 3 * this->num_jnt;

    // specify joint limits (these are in radians )
    double lb[state_dim];
    double up[state_dim];

    for (int i = 0; i < state_dim; i++){
        if ((i % (this->num_jnt*3)) < this->num_jnt){ //position indicies
            lb[i] = -3.1415;
            up[i] = 3.1415;
        }
        else if ((i % (this->num_jnt*3)) < (this->num_jnt*2)){
            lb[i] = -2;
            up[i] = 2;
        }
        else{
            lb[i] = -100.0;
            up[i] = 100.0;
        }
    }

    // manually set the initial and end constraints using the upper and lower bounds
    // for now, the vector constraint is applied as well.
    for (int i = 0; i < (this->num_jnt*2); i++){
        lb[i] = init(i);
        up[i] = init(i);
        lb[state_dim -(3*this->num_jnt) + i] = goal(i);
        up[state_dim -(3*this->num_jnt) + i] = goal(i);
    }
   

    // Create nlopt struct
    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA,state_dim);
    nlopt_set_lower_bounds(opt,lb);
    nlopt_set_upper_bounds(opt,up);

    // //set initial point constraints
    // init_struct is(init);
    // for (int i = 0; i < (2*this->num_jnt); i++){
    //     nlopt_add_equality_constraint(opt,this->set_init_constraint,&is,1e-4);
    //     is.pos += 1;
    // }

    // //goal point constraint setup
    // init_struct fs(goal);
    // for (int i = 0; i < (2*this->num_jnt); i++){
    //     nlopt_add_equality_constraint(opt,this->set_final_constraint,&fs,1e-4);
    //     fs.pos += 1;
    // }

    // Create dynamic constraints
    dynamic_struct* ds = new dynamic_struct((T/N),state_dim,this->num_jnt,N, this->model);
    for (int i = 0; i < ((T/N - 1) * (2*this->num_jnt)); i++){
        nlopt_add_equality_constraint(opt,this->dynamic_constraint,ds,1e-4);
    }

    nlopt_set_min_objective(opt,this->cost_function,NULL);
    nlopt_set_xtol_rel(opt,1e-2);

    //set initial conditions
    double x[state_dim];
    for (int i =0; i < state_dim; i++){
        x[i] = 0.0;
    }
    for (int i = 0; i < (this->num_jnt*2); i++){
        x[i] = init(i);
    }

    double minf;
    nlopt_result res = nlopt_optimize(opt,x,&minf);

    if (res < 0){
        cout << "LOPT FAILED" << endl;
        Eigen::VectorXd out = Eigen::VectorXd::Zero(this->num_jnt);
        return out;
    }
    else{
        Eigen::VectorXd out = Eigen::VectorXd::Zero(this->num_jnt);
        for (int i =0; i < (this->num_jnt); i++){
            out(i) = x[(3*this->num_jnt) + i];
        }
        return out;
    }

}


Eigen::VectorXd MPC::get_control(mjModel* m, mjData* d, Eigen::VectorXd goal, double T, double N){

    // STEP 1: Calculate the current state of the robot in Eigen::VectorXd format (size should be #joints *2)
    // Format the current joint positions and velocities as init:
    Eigen::VectorXd init(m->njnt*2);
    init = Eigen::VectorXd::Zero(m->njnt*2);
    for (int i =0; i < m->njnt; i++){
        init(i) = d->qpos[i];
        init(m->njnt+i) = d->qvel[i];
    }


    // STEP 2: If no path has been created, generate a new path using the path planning algorithm. Else find the closest point on path.
    pair<Eigen::VectorXd, int> nextpt_pair;

    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(m->njnt*2,0,(m->njnt*2)-1);
    Eigen::ArrayXi pos_idx = Eigen::ArrayXi::LinSpaced(m->njnt,0,(m->njnt - 1));
    Eigen::ArrayXi vel_idx = Eigen::ArrayXi::LinSpaced(m->njnt,m->njnt,(m->njnt*2)-1);
    
    // Uncomment to follow trajectory
    if (!this->path_found){ 
        this->find_path(init,goal);
        nextpt_pair = make_pair(this->saved_path(1,allidx),1);
        this->path_found = true;
        this->kdtree.print_debug_info();
    }
    else{
        this->kdtree.print_debug_info();
        nextpt_pair = this->find_next_point(init);
    }
    
    // FIXME: Possibly getting caught up trying to match one point. Make it so that it chooses a point somewhere outside the vicinity of this one.
    // RIght now its just osilating between two points... Maybe remove a point from the kd tree once its close enough?!?!
    //FIXME: Use the find in the kdtree to get the index of that point, then delete all indicies below that from the kd tree so if the index is 14, remove 
    // points at index 0-14 from the kd tree;
    for (int i = 0; i < nextpt_pair.second; i++){ //FIXME: FOr some reason this is causing a segmentation fault. Firgure out why.
        this->kdtree.delete_node(this->saved_path(i,allidx));
    }
    Eigen::VectorXd nextpt = nextpt_pair.first;


    // STEP 3: Begin optimization to find the trajectory for the next N seconds and resturn one step in that direction
    Eigen::VectorXd ctrl = this->optimize(init,nextpt,T,N);
    return ctrl;

}

Eigen::VectorXd MPC::get_control(Eigen::VectorXd init, Eigen::VectorXd goal, double T, double N){

    // STEP 2: If no path has been created, generate a new path using the path planning algorithm. Else find the closest point on path.
    pair<Eigen::VectorXd, int> nextpt_pair;

    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(this->num_jnt*2,0,(this->num_jnt*2)-1);
    Eigen::ArrayXi pos_idx = Eigen::ArrayXi::LinSpaced(this->num_jnt,0,(this->num_jnt - 1));
    Eigen::ArrayXi vel_idx = Eigen::ArrayXi::LinSpaced(this->num_jnt,this->num_jnt,(this->num_jnt*2)-1);

    // Uncomment to follow trajectory
    if ((!this->path_found) || (this->curr_goal != goal)){ 
        cout << "path is generating" << endl;
        this->find_path(init,goal); //FIXME: Will need to possible include constraints here. Controller should keep track of data taken aquired from the sensor node and update the constraints. 
        nextpt_pair = make_pair(this->saved_path(1,allidx),1);
        this->path_found = true;
        this->kdtree.print_debug_info();
        this->curr_goal = goal;
        this->deleted_kdnodes = 0;
    }
    else{
        cout << "getting point from tree" << endl;
        this->kdtree.print_debug_info();
        nextpt_pair = this->find_next_point(init);
    }
    
    for (int i = this->deleted_kdnodes; i < nextpt_pair.second; i++){ 
        cout << "deleting node: " << i << endl;
        this->kdtree.delete_node(this->saved_path(i,allidx));
    }
    this->deleted_kdnodes += nextpt_pair.second - this->deleted_kdnodes;
    Eigen::VectorXd nextpt = nextpt_pair.first;
    cout << "( ";
    for (int i = 0; i < 4; i++){
        cout << nextpt(i) << " ";
    }
    cout << ")" << endl;


    // STEP 3: Begin optimization to find the trajectory for the next N seconds and resturn one step in that direction
    Eigen::VectorXd ctrl = this->optimize(init,nextpt,T,N);
    return ctrl;

}