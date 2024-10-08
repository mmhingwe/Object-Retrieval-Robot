// This file is used to test the implemented path planning algorithms.
// NOTE: This file uses matplotlib-cpp which is a wrapper around the python library.
// Therefore, matplotlib python must be installed on your computer or you must be using a conda environment which contains it.

#include <iostream>
#include <util.h>
#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <queue>
#include <string>
#include "robotmodel.h"
#include <numeric>
using namespace std;

int main(){

    string model_filename = "/home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/universal_robots_ur10e/scene.xml";
    mjModel* m;
    mjData* d;
    char error[1000] = "Could not load model";
    m = mj_loadXML(model_filename.c_str(), NULL, error, 1000);
    d = mj_makeData(m);
    cout << "Model created Successfully" << endl;

    robotModel model_tree(m,d);
    
    // Eigen::MatrixXd test = Eigen::MatrixXd::Random(4,4);
    // cout << test << endl << endl;

    // Eigen::MatrixXd res = transform_to_spatial_transform(test);
    // cout << res << endl;
    // cout << endl << endl << endl << transform_to_spatial_transform(test,true);


    // cout << "body tree information" << endl;
    // model_tree.print_body_tree();

    // cout << "Testing Spatial cross product" << endl;
    // Eigen::VectorXd v11(6);
    // Eigen::VectorXd v12(6);
    // Eigen::VectorXd res(6);
    // v11 << 1,0,0,0,0,0;
    // v12 << 0,0,1,0,0,0;
    // res = spatial_cross_product(v11,v12);
    // cout << res << endl;
    // return -1;


    cout << "joint tree information" << endl;
    model_tree.print_joint_tree();

    cout << endl << endl << endl << endl << endl; 


    Eigen::MatrixXd test_state(4,3);
    test_state << 0,0,0,0,0,0,0,0,0,0,0,0;
    cout << test_state << endl;

    Eigen::VectorXd torque_calc(3);
    torque_calc = model_tree.RNEA(test_state);
    cout << torque_calc << endl;
}