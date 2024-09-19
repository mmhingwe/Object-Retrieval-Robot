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

// Takes the adjoint of a transformation matrix
Eigen::MatrixXd transform_adjoint(Eigen::MatrixXd input, bool star = false);

// Returns the adjoint matrix of a spatrial vector
Eigen::MatrixXd spatial_adjoint(Eigen::VectorXd input);

// Returns the spatial cross product of two spatial vectors
Eigen::VectorXd spatial_cross_product(Eigen::VectorXd input1, Eigen::VectorXd input2, bool star);

Eigen::MatrixXd exponential_rotation_spatial(Eigen::VectorXd screw, double theta);



