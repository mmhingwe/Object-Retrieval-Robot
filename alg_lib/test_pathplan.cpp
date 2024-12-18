// This file is used to test the implemented path planning algorithms.
// NOTE: This file uses matplotlib-cpp which is a wrapper around the python library.
// Therefore, matplotlib python must be installed on your computer or you must be using a conda environment which contains it.

#include <iostream>
#include <pathplan.h>
#include <util.h>
#include <Eigen/Core>
#include <queue>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;

int main(){

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