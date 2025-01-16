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
#include "datastructures/kdtree.h"
using namespace std;

int main(){

    // string model_filename = "/home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/alg_lib/universal_robots_ur10e/scene.xml";
    string model_filename = "/home/mhingwe/Documents/Robot_Projects/Object-Retrieval-Robot/alg_lib/universal_robots_ur10e/ur10e.xml";
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


    // TODO: Make seperate file to testcase this.
    // Testing KD tree
    cout << endl << endl << endl;

    Eigen::VectorXd p1(2);
    p1 << 2,6;
    Eigen::VectorXd p2(2);
    p2 << 5,4;
    Eigen::VectorXd p3(2);
    p3 << 8,7;
    Eigen::VectorXd p4(2);
    p4 << 10,2;
    Eigen::VectorXd p5(2);
    p5 << 13,3;
    Eigen::VectorXd p6(2);
    p6 << 9,4;

    kdTree test_tree;

    Eigen::MatrixXd test_mat(5,2);
    test_mat << 2,6,5,4,8,7,10,2,13,3;
    test_tree.populate(test_mat);

    test_tree.delete_node(p2);

    // test_tree.insert_node(p1);
    // cout << "first point inserted" << endl;
    // test_tree.insert_node(p2);
    // cout << "second point inserted" << endl;
    // test_tree.insert_node(p3);
    // cout << "third point inserted" << endl;
    // test_tree.insert_node(p4);
    // cout << "fourth point inserted" << endl;
    // test_tree.insert_node(p5);

    cout << "points inserted" << endl;
    
    pair<Eigen::VectorXd,int> out = test_tree.find_nearest(p6);
    cout << out.first << endl;


    cout << "Testing forward kinematics function: " << endl;
    Eigen::VectorXd fkt(3);
    fkt << 3.1415/2,0,3.1415/2;
    vector<Eigen::MatrixXd> test_out = model_tree.FK(fkt);

    cout << endl << endl;
    for (int i = 0; i < test_out.size(); i++){
        // cout << test_out[i](0,3) << " " << test_out[i](1,3) << " " << test_out[2](2,3) << endl;

        for (int j = 0; j < test_out[i].rows(); j++){

            for (int k = 0; k < test_out[i].cols(); k++ ){

                cout << test_out[i](j,k) << " ";

            }

            cout << endl;

        }

        cout << endl; 

    }

    mj_forward(m, d);

    cout << endl << "mujoco joint coordinates: " << endl;
    // cout << d->qpos[0] << endl;
    for(int i = 0; i < 21; i++){
        if (i % 3 == 0){
            cout << endl;
        }
        cout << d->xpos[i] << " ";

    }
    cout << endl;

    cout << "TESTS END" << endl;

}