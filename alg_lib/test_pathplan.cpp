// This file is used to test the implemented path planning algorithms.
// NOTE: This file uses matplotlib-cpp which is a wrapper around the python library.
// Therefore, matplotlib python must be installed on your computer or you must be using a conda environment which contains it.

#include <iostream>
#include <pathplan.h>
#include <datastructures/octtree.h>
#include <util.h>
#include <Eigen/Core>
#include <queue>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;

int main(){


    // Octtree tests:
    Eigen::MatrixXd space_bounds(3,2);
    space_bounds << -10,10,-10,10,-10,10;
    octtree oct_test (space_bounds,3);
    Eigen::Vector3d pt1; 
    pt1 << 0,0,0;

    Eigen::Vector3d pt2; 
    pt2 << 7.5,2.5,2.5;

    Eigen::Vector3d pt3; 
    pt3 << 7.5,7.5,2.5;

    Eigen::Vector3d pt4; 
    pt4 << 2.5,7.5,2.5;

    Eigen::Vector3d pt5; 
    pt5 << 0,0,7.5;

    Eigen::Vector3d pt6; 
    pt6 << 7.5,2.5,7.5;

    Eigen::Vector3d pt7; 
    pt7 << 7.5,7.5,7.5;

    Eigen::Vector3d pt8; 
    pt8 << 2.5,7.5,6.5;

    oct_test.add(pt1);
    oct_test.add(pt2);
    oct_test.add(pt3);
    oct_test.add(pt4);
    oct_test.add(pt5);
    oct_test.add(pt6);
    oct_test.add(pt7);
    oct_test.add(pt8);

    Eigen::Vector3d pt_exist; 
    pt_exist << 0,2.5,2.5;
    if (oct_test.is_collision(pt_exist)){
        cout << "COLLISION" << endl;
    }
    else{
        cout << "NO COLLISION" << endl;
    }

    Eigen::MatrixXd test_bounds(3,2);
    space_bounds << 0,10,-10,7, -7,2;

    vector<Eigen::MatrixXd> bounding_boxes = oct_test.return_overlap_area(test_bounds);

    cout << "Overlap bounds print: " << endl;
    for (int i = 0; i < bounding_boxes.size(); i++){

        cout << bounding_boxes[i](0,0) << "  " << bounding_boxes[i](0,1) << endl;
        cout << bounding_boxes[i](1,0) << "  " << bounding_boxes[i](1,1) << endl;
        cout << bounding_boxes[i](2,0) << "  " << bounding_boxes[i](2,1) << endl;
        cout << endl << endl; 

    } 
    


    return 1;



    // Define initial position
    Eigen::VectorXd init(2);
    init << 1,2; 

    // Define goal position
    Eigen::VectorXd goal(2);
    goal << 49,49; 

    // Define constraints 
    Eigen::MatrixXd constraint1(4,3);
    constraint1 << 0,1,10,
              0,2,40,
              1,1,10,
              1,2,40;
    vector<Eigen::MatrixXd> constraints = {constraint1};

    // Define bounds of the space (x= [0,50], y = [0,50])
    Eigen::MatrixXd bound(2,2);
    bound << 0, 50,             
            0,50;

    // Initialize the RRT class and run it.
    RRT test(init,goal,2, constraints , bound);
    int count = test.run(0.1,0.3,1000);

    //TODO: Move the graphing to the RRT class.
    // Use DFS to go through the graph and plot it using matplot lib.
    rrtnode* root = test.return_root();
    queue<rrtnode*> gq;
    vector<double> x;
    vector<double> y;

    while(true){
        
        //Convert from eigen3 vector to std vector
        Eigen::VectorXd buffer = root->data;
        vector<double> graph_point(&buffer[0],buffer.data()+buffer.cols()*buffer.rows());
        x.push_back(graph_point.at(0));
        y.push_back(graph_point.at(1));

        if (root->parents.size() != 0){
            Eigen::VectorXd buffer2 = root->parents.at(0)->data;
            vector<double> graph_point_parent(&buffer2[0],buffer2.data()+buffer2.cols()*buffer2.rows());
            vector<double> xbuf = {graph_point.at(0),graph_point_parent.at(0)};
            vector<double> ybuf = {graph_point.at(1),graph_point_parent.at(1)};
            plt::plot(xbuf,ybuf,"b");
        }

        for (int i = 0; i < root->children.size(); i++){

            gq.push(static_cast<rrtnode*>(root->children.at(i)));

        }

        if (gq.empty() == true){
            break;
        }
        else{
            root = gq.front();
            gq.pop();
        }

    }

    plt::plot(x,y,"og");
    plt::show();


    


}