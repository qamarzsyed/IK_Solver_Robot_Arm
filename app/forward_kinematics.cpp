
#include <cmath>
#include <iostream>
#include <vector>
#include "../include/forward_kinematics.hpp"
#include "../include/Eigen/Dense"

using Eigen::Matrix;
using namespace std;

Matrix<double, 4, 4> ForwardKinematics::calculate_TF(int j){
    ForwardKinematics k;
    _dh_matrix = k.get_dh_parameters();
    int i = j;
    Matrix<double, 4, 4> Tf{
        {cos(_dh_matrix.coeff(i, 0)), -sin(_dh_matrix.coeff(i, 0)) * cos(_dh_matrix.coeff(i, 1)), sin(_dh_matrix.coeff(i,0)) * sin(_dh_matrix.coeff(i,1)), _dh_matrix.coeff(i, 2) * cos(_dh_matrix.coeff(i, 0))},
        {sin(_dh_matrix.coeff(i, 0)), cos(_dh_matrix.coeff(i, 0)) * cos(_dh_matrix.coeff(i, 1)), -cos(_dh_matrix.coeff(i,0)) * sin(_dh_matrix.coeff(i,1)), _dh_matrix.coeff(i, 2) * sin(_dh_matrix.coeff(i, 0))},
        {0, sin(_dh_matrix.coeff(i, 1)), cos(_dh_matrix.coeff(i, 1)), _dh_matrix.coeff(i,3)},
        {0, 0 , 0, 1},
    };
    return Tf;
}

Matrix<double, 4, 4> ForwardKinematics::solve_fk(){

    Matrix<double, 4, 4>Tf1 = Matrix<double, 4, 4>::Identity();
    for(int i = 0; i < 6; i++)
    {   
        Matrix<double, 4, 4> Tf_i = calculate_TF(i);
        Tf1 = Tf1 * Tf_i;  
    };
    Matrix<double, 3, 3>Rotation = Tf1.block<3,3>(0,0);
    Matrix<double, 4, 1>Position1 = Tf1.col(3);
    vector<double>Position(3);
    cout << "The position is vector " << endl;
    cout << "[";
    for(int i = 0; i < 3; i++){
        Position[i] = Position1.coeff(i,0);
        cout << Position[i] << ',';
    }
        cout << "]" << endl;
    cout << "The Rotation matrix is " << endl << Rotation<< endl;
//For the final implementation, we might need to instead pass the 3x3 matrix for rotation and 3 element vector for position instead of 4x4 matrix.
    return Tf1;
}
