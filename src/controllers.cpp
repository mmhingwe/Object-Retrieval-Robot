#include <iostream>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <math.h>
#include "controllers.h"
#include "pathplan.h"

using namespace std;

// This is only in joint space right now, it may be advantages to also implement a task space version of this.
// Constructor for mujoco simulation control
PID::PID(mjModel* m, mjData* d, std::string planner_type){
    
    // Set default pid variables to 0
    this->kp = 0;
    this->kd = 0;
    this->ki = 0;
    this->cum_sum = Eigen::VectorXd::Zero(m->njnt);


    this->model = new robotModel(m,d);
    this->associated_joint_bodies = this->model->return_joint_bodies();
    this->saved_path = Eigen::MatrixXd::Zero(1,1);
    this->path_found = false;

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

    cout << "Space bounds: " << endl;
    cout << space_const_new << endl << endl;

    this->space_bounds = space_const_new;



    // Determine constraints of the input robot model. (note, the way this controller is setup, the state of the robot is the position and velocity.)
    vector<Eigen::Vector3d> constraints_vec;
    
    // Joint Constraints
    //FIXME: Will need to add constraints in the configuration space, not workspace. Will need to use robot model to convert.

    // for (int i = 0; i < m->njnt; i++){
    //     Eigen::Vector3d buffer_l;
    //     Eigen::Vector3d buffer_u;
    //     buffer_l << i, 1, m->jnt_range[2*i];
    //     buffer_u << i, 2, m->jnt_range[2*i+1];
    //     constraints_vec.push_back(buffer_l);
    //     constraints_vec.push_back(buffer_u);
    // }

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

// Constructor for actual control using sensors.
PID::PID(std::string xml_filename,std::string planner_type){}

// Destructor
PID::~PID(){}

// Set PID values
void PID::set_pid(double kp,double kd,double ki){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}

// Find the path using the specified path planning algorithm. The space bounds and constraints should be specified earlier.
void PID::find_path(Eigen::VectorXd init, Eigen::VectorXd goal){
    
    // vector<Eigen::MatrixXd> buffer_constraints = {this->constraints};
    vector<Eigen::MatrixXd> buffer_constraints; //FIXME: Temporary no constraints, implement null constraints of this case in the constructor

    this->planner->set_perameters(init,goal,this->config_space, buffer_constraints, this->space_bounds);
    int success = this->planner->run(0.5,0.5,100000);
    cout << "--------------------->" << success << "<------------------" << endl;
    this->saved_path = this->planner->return_path().transpose();
    cout << "path saved" << endl;
    kdtree.populate(this->saved_path);
    cout << "populate passed" << endl;
    cout << this->saved_path << endl;
}

// Find the next nearest point in the saved path. 
pair<Eigen::VectorXd,int> PID::find_next_point(Eigen::VectorXd init){

    cout << "atempting to find nearest point" << endl;
    pair<Eigen::VectorXd,int> nearest_point = this->kdtree.find_nearest(init);
    cout << "nearest point found.  Id: " << nearest_point.second <<endl;
    cout << "max index: " << this->saved_path.cols() -1 << endl;
    // vector<int> allidx;
    // for (int i =0; i < nearest_point.first.cols(); i++){
    //     allidx.push_back(i);
    // }
    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(nearest_point.first.size(),0,(nearest_point.first.size())-1);

    //TODO: HAndle case where you are at the goal position. For now, just output a message.
    if (nearest_point.second == (this->saved_path.rows()-1)){
        cout << "GOAL POINT REACHED" << endl;
        return make_pair(this->saved_path(nearest_point.second,allidx),nearest_point.second);
    }
    else{
        cout << "GOAL POINT NOT REACHED " << endl;
        return make_pair(this->saved_path(nearest_point.second + 1,allidx),nearest_point.second+1);
    }

}

// NOTE: THIS IS USING JOINT SPACE
Eigen::VectorXd PID::get_control(mjModel* m, mjData* d, Eigen::VectorXd goal){

    cout << "Get control called" <<endl;

    // STEP 1: Calculate the current state of the robot in Eigen::VectorXd format (size should be #joints *2)
    // Format the current joint positions and velocities as init:
    Eigen::VectorXd init(m->njnt*2);
    init = Eigen::VectorXd::Zero(m->njnt*2);
    // cout << "Printing the q pos and q vel values." << endl;
    for (int i =0; i < m->njnt; i++){
        // cout << "Index: " << i << endl;
        // cout << "qpos: " << d->qpos[i] << endl;
        // cout << "qvel" << d->qvel[i] << endl;
        // cout << endl << endl;
        init(i) = d->qpos[i];
        init(m->njnt+i) = d->qvel[i];
    }
    cout << "current position" << endl;
    cout << init << endl << endl;
    cout << "goal position" << endl;
    cout << goal << endl << endl;

    // STEP 2: If no path has been created, generate a new path using the path planning algorithm. Else find the closest point on path.

    // Eigen::VectorXd nextpt(m->njnt*2);

    pair<Eigen::VectorXd, int> nextpt_pair;

    // vector<int> allidx;
    Eigen::ArrayXi allidx = Eigen::ArrayXi::LinSpaced(m->njnt*2,0,(m->njnt*2)-1);
    Eigen::ArrayXi pos_idx = Eigen::ArrayXi::LinSpaced(m->njnt,0,(m->njnt - 1));
    Eigen::ArrayXi vel_idx = Eigen::ArrayXi::LinSpaced(m->njnt,m->njnt,(m->njnt*2)-1);
    
  






    // Uncomment to follow trajectory
    if (!this->path_found){ //} == Eigen::MatrixXd::Zero(1,1)){
        // cout << "Finding path" << endl;
        this->find_path(init,goal);
        // cout << this->saved_path << endl;
        cout <<"PATH SAVED" << endl;
        nextpt_pair = make_pair(this->saved_path(1,allidx),1);
        this->path_found = true;
        this->kdtree.print_debug_info();
    }
    else{
        cout << "finding point" << endl;
        this->kdtree.print_debug_info();

        cout << "calculating next point " << endl;
        nextpt_pair = this->find_next_point(init);

        cout << nextpt_pair.first << endl << endl;

    }
    
    // FIXME: Possibly getting caught up trying to match one point. Make it so that it chooses a point somewhere outside the vicinity of this one.
    // RIght now its just osilating between two points... Maybe remove a point from the kd tree once its close enough?!?!
    //FIXME: Use the find in the kdtree to get the index of that point, then delete all indicies below that from the kd tree so if the index is 14, remove 
    // points at index 0-14 from the kd tree;
    cout << "Next point:" << endl;
    for (int i = 0; i < nextpt_pair.second; i++){ //FIXME: FOr some reason this is causing a segmentation fault. Firgure out why.
        this->kdtree.delete_node(this->saved_path(i,allidx));
    }
    cout << nextpt_pair.first << "   " << nextpt_pair.second << endl << endl;
    Eigen::VectorXd nextpt = nextpt_pair.first;



    // Eigen::VectorXd nextpt = goal;

    // STEP 3: Calculate the required acceleration to get to the next desired point.
    // TODO: Right now an arbritrary time interval is chosen
    //FIXME: Add k1 and k2 to the constructor and private variables, basically feedback linearization.
    Eigen::VectorXd acc(m->njnt);
    int k1 = 40;
    int k2 = 200;
    //PD Control
    this->cum_sum += (nextpt(pos_idx) - init(pos_idx)) * (1/60);
    acc = ((nextpt(vel_idx) - init(vel_idx))*(1/60)) + this->kp* ((nextpt(pos_idx) - init(pos_idx))) + this->kd * ((nextpt(vel_idx) - init(vel_idx))) + this->ki * this->cum_sum;
    cout << "Goal acceleration: " << endl;
    cout << ((nextpt(vel_idx) - init(vel_idx))) << endl << endl;
    //((nextpt(vel_idx) - init(vel_idx)) * (1/60))

    cout << "acceleration calculated" << endl;
    cout << endl << acc << endl << endl;

    // STEP 4: Take into account any contact forces being experienced.

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
    // Add force calculation here x(3,vel_idx);

    cout << "input into rnea: " << endl << x << endl <<  endl << endl;


    // STEP 6: Use RNEA to calculate the torque required to take the next step.
    Eigen::VectorXd torque = this->model->RNEA(x);
    cout << "Final Torque: " << endl;
    cout << torque << endl << endl;

    // STEP 7: Return the torque.
    return torque;

}