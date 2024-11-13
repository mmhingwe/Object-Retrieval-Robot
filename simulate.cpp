#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "pathplan.h"
#include "controllers.h"
#include "matplotlibcpp.h"
#include "util.h"

namespace plt = matplotlibcpp;
using namespace std;

int main(){

    // Load the model of the robot and the mjdata
    // string model_filename = "/home/mhingwe/codebases/tennis_trajectory_extraction-main/mujoco/robots/omni.xml";
    string model_filename = "/home/mhingwe/Documents/Robot_Projects/mujoco_test/universal_robots_ur10e/scene.xml";
    mjModel* m;
    mjData* d;
    char error[1000] = "Could not load model";
    m = mj_loadXML(model_filename.c_str(), NULL, error, 1000);
    d = mj_makeData(m);
    cout << "Model created Successfully" << endl;


    // Initialize the mujoco data structures
    // mjModel* m = NULL;                  // MuJoCo model
    // mjData* d = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    mjvPerturb pert;

    // init GLFW, create window, make OpenGL context current, request v-sync
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultPerturb(&pert);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_100);

    // cout << m->opt.gravity[2] << endl;
    // return 1;

    // Setup Robot Perameters
    m->opt.gravity[0] = 0;
    m->opt.gravity[1] = 0;
    m->opt.gravity[2] = -9.81;

    // Setup robot control
    PID rob_ctrl(m,d,"rrt_base");
    rob_ctrl.set_pid(200.0,50.0,0.0);
    Eigen::VectorXd goal(4);
    // goal << 0,0,0,0;
    goal << 1.0,-0.5,0,0;
    // rob_ctrl.get_control(m,d, goal);

    // Setup graphing tools.
    vector<double> pos_x;
    vector<double> pos_y;
    vector<double> vel_x;
    vector<double> vel_y;

    vector<double> goal_pos_x;
    vector<double> goal_pos_y;
    vector<double> goal_vel_x;
    vector<double> goal_vel_y;

    vector<double> time_axis;

    // Set initial information
    // pos_x.push_back(d->qpos[0]);
    // pos_y.push_back(d->qpos[1]);
    // vec_x.push_back(d->qvel[0]);
    // vec_y.push_back(d->qvel[1]);
    // goal_pos_x.push_back(0.0);
    // goal_pos_y.push_back(0.0);
    // goal_vec_x.push_back(0.0);
    // goal_vec_y.push_back(0.0);
    // time_axis.push_back(0.0);




    //TODO: setup mouse callbacks
    int count = 0;
    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) ) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;

        pos_x.push_back(d->qpos[0]);
        pos_y.push_back(d->qpos[1]);
        vel_x.push_back(d->qvel[0]);
        vel_y.push_back(d->qvel[1]);
        goal_pos_x.push_back(0.0);
        goal_pos_y.push_back(0.0);
        goal_vel_x.push_back(0.0);
        goal_vel_y.push_back(0.0);
        time_axis.push_back(d->time);

        Eigen::VectorXd torque = rob_ctrl.get_control(m,d, goal);
        d->ctrl[0] = torque(1);
        d->ctrl[1] = torque(2);

        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);



        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

        cout << "---------------------------------------------------------------------------------------------------------------------" << endl;
        
        if (count == 500){
            break;
        }

        count += 1;

        // return 1;

    }

    plt::subplot(2,2,1);
    plt::plot(time_axis,pos_x);
    plt::plot(time_axis,goal_pos_x);

    plt::subplot(2,2,2);
    plt::plot(time_axis,pos_y);
    plt::plot(time_axis,goal_pos_y);

    plt::subplot(2,2,3);
    plt::plot(time_axis,vel_x);
    plt::plot(time_axis,goal_vel_x);

    plt::subplot(2,2,4);
    plt::plot(time_axis,vel_y);
    plt::plot(time_axis,goal_vel_y);

    plt::show();

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);



}