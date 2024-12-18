#include "util.h"
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <math.h>
using namespace std;


//Convert the mjt data type into a Eigen::VectorXd type
Eigen::VectorXd mjtNum_to_eigenvec(mjtNum* input, int size,int offset){
    Eigen::VectorXd out(size);
    out = Eigen::VectorXd::Zero(size);
    for(int i=0; i < size; i++){
        out(i) = input[offset + i];
    }
    return out;
}

// Hamilton Product for the product of quaternions
Eigen::VectorXd hamilton_product(Eigen::VectorXd q1, Eigen::VectorXd q2){
    double w = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    double x = q1(0) * q2(1) + q1(1) * q2(0) + q1(2)*q2(3)  - q1(3) * q2(2);
    double y = q1(0) * q2(2) - q1(1) * q2(3) + q1(2)*q2(0)  + q1(3) * q2(1);
    double z = q1(0) * q2(3) + q1(1) * q2(2) - q1(2)*q2(1)  + q1(3) * q2(0);
    Eigen::VectorXd out(4);
    out << w, x, y, z;
    return out;
}

// Convert quaternions into 3d rotation matrix
// NOTE: The output is sometimes off by a very small number, probably due to using doubles. 
Eigen::MatrixXd quat_to_rmat(Eigen::VectorXd q){
    Eigen::MatrixXd out(3,3);
    double r1 = 1 - 2 * (pow(q(2),2) + pow(q(3),2));
    double r2 = 2 * ((q(1)*q(2)) - (q(3)*q(0)));
    double r3 = 2 * ((q(1)*q(3)) + (q(2)*q(0)));
    double r4 = 2 * ((q(1)*q(2)) + (q(3)*q(0)));
    double r5 = 1 - 2 * (pow(q(1),2) + pow(q(3),2));
    double r6 = 2 * ((q(2)*q(3)) - (q(1)*q(0)));
    double r7 = 2 * ((q(1)*q(3)) - (q(2)*q(0)));
    double r8 = 2 * ((q(2)*q(3)) + (q(1)*q(0)));
    double r9 = 1 - 2 * (pow(q(1),2) + pow(q(2),2));
    out << r1,r2,r3,r4,r5,r6,r7,r8,r9;
    return out;
}


void return_info(mjModel* model){

    cout << "-----------Body name and id----------" << endl;

    for (int i =0; i < model->nbody; i++){
        cout << model->names + model->name_bodyadr[i]<< "           bodyid:" << i   << endl;
    }

    cout << endl << "-----------Body Base Quaternions--------------" << endl;

    for (int i =0; i < model->nbody; i++){
        cout << "id: " << i << "       ";
        for (int j=0; j < 4; j++){
            cout << model->body_quat[(i * 4) + j] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "-----------Body associated with joints and their twist axis--------------" << endl;
    for (int i =0; i < model->njnt; i++){
        cout << "Joint id: " << i << "   Body id: " << model->jnt_bodyid[i] << "      twist axis: ";
        for (int j=0; j < 3; j++){
            cout << model->jnt_axis[(i * 3) + j] << " ";
        }
        cout << endl;
    }
}


// Turns a vector into a skew symetric matrix
Eigen::MatrixXd vector_to_skew(Eigen::VectorXd input){
    
    // cout << input.size() << endl;
    
    Eigen::MatrixXd out(3,3);
    out << 0, -input(2), input(1),
           input(2), 0, -input(0),
           -input(1), input(0), 0;
    return out;
}

//Takes the adjoint of a transformation matrix
// If star == true, then tkaing the transformation for wrench force
Eigen::MatrixXd transform_adjoint(Eigen::MatrixXd input, bool star){

    Eigen::MatrixXd out(6,6);
    Eigen::MatrixXd R;
    Eigen::VectorXd P;
    Eigen::MatrixXd P_skew;

    // Get the rotation portion from the transformation matrix
    R = input({0,1,2},{0,1,2});
    P = input({0,1,2},3);
    P_skew = vector_to_skew(P);

    if (!star){
        out << R, Eigen::MatrixXd::Zero(3,3), P_skew*R, R;
        return out;
    }
    else{
        out << R, P_skew*R, Eigen::MatrixXd::Zero(3,3),R;
        return out;
    }

}

// Returns the adjoint matrix of a spatrial vector
Eigen::MatrixXd spatial_adjoint(Eigen::VectorXd input){
    Eigen::MatrixXd out (6,6);
    out << vector_to_skew(input({0,1,2})), Eigen::MatrixXd::Zero(3,3), vector_to_skew(input({3,4,5})), vector_to_skew(input({0,1,2}));
    return out;
}

// Returns the spatial cross product of two spatial vectors
Eigen::VectorXd spatial_cross_product(Eigen::VectorXd input1, Eigen::VectorXd input2, bool star){
    //spatial cross product
    if (!star){
        Eigen::VectorXd out;
        out = spatial_adjoint(input1) * input2;
        return out;
    }
    // Wrench cross product
    else{

        //TODO: Make this spatial less expensive.
        Eigen::VectorXd out(6);
        Eigen::VectorXd w(3);
        Eigen::VectorXd v(3);
        Eigen::VectorXd t(3);
        Eigen::VectorXd f(3);
        w = input1({0,1,2});
        v = input1({3,4,5});
        t = input2({0,1,2});
        f = input2({3,4,5});

        out({0,1,2}) = (vector_to_skew(w) * t )+ (vector_to_skew(v)*f);
        out({3,4,5}) = vector_to_skew(w) * f;

        return out;

    }
}

Eigen::MatrixXd exponential_rotation_spatial(Eigen::VectorXd screw, double theta){

    Eigen::MatrixXd out (4,4);
    out = Eigen::MatrixXd::Identity(4,4);
    
    Eigen::VectorXd ang_vel = screw({0,1,2});
    Eigen::VectorXd lin_vel = screw({3,4,5});

    Eigen::MatrixXd skew_ang = vector_to_skew(ang_vel);
    Eigen::MatrixXd mat_exp = Eigen::MatrixXd::Identity(3,3)+ (skew_ang * sin(theta)) + ((skew_ang * skew_ang) * (1-cos(theta)));
    Eigen::MatrixXd p = (Eigen::MatrixXd::Identity(3,3) - mat_exp) * lin_vel + (mat_exp*mat_exp.transpose()) * lin_vel*theta;
 
    return out;

}


// // The following functions convert double matrix into adouble
// // Converts MatrixXd into an MatrixXad (adouble for automatic differentiation)
// Eigen::MatrixXad Matrix_dtad(Eigen::MatrixXd x){
//     Eigen::MatrixXad out(x.rows(),x.cols());
//     for (int i =0; i < x.rows(); i++){
//         for (int j = 0; j < x.cols(); j++){
//             out(i,j) = x(i,j);
//         }
//     }
//     return out;
// }

// Eigen::VectorXad Vector_dtad(Eigen::VectorXd x){
//     Eigen::VectorXad out(x.size());
//     for (int i =0; i < x.size(); i++){
//        out(i) = x(i);
//     }
//     return out;
// }


// // The following functions are the same as the above matrix manipulation functions except they deal with the adouble scalar type;

// // Convert a 3d vector into a skew symmetric matrix for the adouble matrix type
// Eigen::MatrixXad vector_to_skew_adouble(Eigen::VectorXad input){
//     Eigen::MatrixXad out(3,3);
//     out << 0, -input(2), input(1),
//            input(2), 0, -input(0),
//            -input(1), input(0), 0;
//     return out;
// }

// // Takes the adjoint of a transformation matrix for the adouble matrix type
// Eigen::MatrixXad transform_adjoint_adouble(Eigen::MatrixXad input, bool star){
//     Eigen::MatrixXad out(6,6);
//     Eigen::MatrixXad R;
//     Eigen::VectorXad P;
//     Eigen::MatrixXad P_skew;

//     // Get the rotation portion from the transformation matrix
//     R = input({0,1,2},{0,1,2});
//     P = input({0,1,2},3);
//     P_skew = vector_to_skew_adouble(P);

//     if (!star){
//         out << R, Eigen::MatrixXad::Zero(3,3), P_skew*R, R;
//         return out;
//     }
//     else{
//         out << R, P_skew*R, Eigen::MatrixXad::Zero(3,3),R;
//         return out;
//     }
// }

// // Returns the adjoint matrix of a spatrial vector for the adouble matrix type
// Eigen::MatrixXad spatial_adjoint_adouble(Eigen::VectorXad input){
//     Eigen::MatrixXad out (6,6);
//     out << vector_to_skew_adouble(input({0,1,2})), Eigen::MatrixXad::Zero(3,3), vector_to_skew_adouble(input({3,4,5})), vector_to_skew_adouble(input({0,1,2}));
//     return out;
// }

// // Returns the spatial cross product of two spatial vectors for the adouble matrix type
// Eigen::VectorXad spatial_cross_product_adouble(Eigen::VectorXad input1, Eigen::VectorXad input2, bool star){
//         //spatial cross product
//     if (!star){
//         Eigen::VectorXad out;
//         out = spatial_adjoint_adouble(input1) * input2;
//         return out;
//     }
//     // Wrench cross product
//     else{

//         //TODO: Make this spatial less expensive.
//         Eigen::VectorXad out(6);
//         Eigen::VectorXad w(3);
//         Eigen::VectorXad v(3);
//         Eigen::VectorXad t(3);
//         Eigen::VectorXad f(3);
//         w = input1({0,1,2});
//         v = input1({3,4,5});
//         t = input2({0,1,2});
//         f = input2({3,4,5});

//         out({0,1,2}) = (vector_to_skew_adouble(w) * t )+ (vector_to_skew_adouble(v)*f);
//         out({3,4,5}) = vector_to_skew_adouble(w) * f;

//         return out;

//     }
// }

// Eigen::MatrixXad exponential_rotation_spatial_adouble(Eigen::VectorXad screw, adouble theta){

//     Eigen::MatrixXad out (4,4);
//     out = Eigen::MatrixXad::Identity(4,4);
    
//     Eigen::VectorXad ang_vel = screw({0,1,2});
//     Eigen::VectorXad lin_vel = screw({3,4,5});

//     Eigen::MatrixXad skew_ang = vector_to_skew_adouble(ang_vel);
//     Eigen::MatrixXad mat_exp = Eigen::MatrixXad::Identity(3,3)+ (skew_ang * sin(theta)) + ((skew_ang * skew_ang) * (1-cos(theta)));
//     Eigen::MatrixXad p = (Eigen::MatrixXad::Identity(3,3) - mat_exp) * lin_vel + (mat_exp*mat_exp.transpose()) * lin_vel*theta;
 
//     return out;

// }