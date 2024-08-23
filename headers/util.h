#pragma once

// #include <mujoco/mujoco.h>
#include <string>
#include <Eigen/Core>
#include <mujoco/mujoco.h>

//Convert the mjt data type into a Eigen::VectorXd type
Eigen::VectorXd mjtNum_to_eigenvec(mjtNum* input, int size, int offset);

// Hamilton Product for the product of quaternions
Eigen::VectorXd hamilton_product(Eigen::VectorXd q1, Eigen::VectorXd q2);

// Convert quaternions into 3d rotation matrix
Eigen::MatrixXd quat_to_rmat(Eigen::VectorXd quat);

// Returns information from the robot model such as names and associated id's etc
void return_info(mjModel* model);

// Convert a 3d vector into a skew symmetric matrix
Eigen::MatrixXd vector_to_skew3d(Eigen::VectorXd input);

Eigen::MatrixXd transform_adjoint(Eigen::MatrixXd input, bool star = false);



