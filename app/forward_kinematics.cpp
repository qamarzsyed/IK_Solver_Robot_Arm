
#include <cmath>
#include <iostream>
#include <vector>
#include "../include/forward_kinematics.hpp"
#include "../include/Eigen/Dense"

using Eigen::Matrix;
using namespace std;

Matrix<double, 4, 4> ForwardKinematics::calculate_TF(int j){
    cout << j << endl;
    ForwardKinematics k;
    _dh_matrix = k.get_dh_parameters();
    Matrix<double, 4, 4>Tf = Matrix<double, 4, 4>::Identity();   
    return Tf;
}

Matrix<double, 4, 4> ForwardKinematics::solve_fk(){

    Matrix<double, 4, 4>Tf1 = Matrix<double, 4, 4>::Identity();
    return Tf1;
}
