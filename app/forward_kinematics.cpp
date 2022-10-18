
#include <cmath>
#include <iostream>
#include <vector>
#include "../include/forward_kinematics.hpp"
#include "../include/Eigen/Dense"

using Eigen::MatrixXd;
using namespace std;

Eigen::Matrix<double, 4, 4> ForwardKinematics::calculate_TF(int j){
    
    int i = j;
    Eigen::Matrix<double, 4, 4> Tf{
        {cos(_dh_matrix(i, 0)), -sin(_dh_matrix(i, 0)) * cos(_dh_matrix(i, 1)), sin(_dh_matrix(i,0)) * sin(_dh_matrix(i,1)), _dh_matrix(i, 2) * cos(_dh_matrix(i, 0))},
        {sin(_dh_matrix(i, 0)), cos(_dh_matrix(i, 0)) * cos(_dh_matrix(i, 1)), -cos(_dh_matrix(i,0)) * sin(_dh_matrix(i,1)), _dh_matrix(i, 2) * sin(_dh_matrix(i, 0))},
        {0, sin(_dh_matrix(i, 1)), cos(_dh_matrix(i, 1)), _dh_matrix(i,3)},
        {0, 0 , 0, 1},
    };
    return Tf;
}

Eigen::Matrix<double, 4, 4> ForwardKinematics::solve_fk(){

    _dh_matrix = RobotParameters::get_dh_parameters();

    Eigen::Matrix<double, 4, 4>Tf1 = Eigen::Matrix<double, 4, 4>::Identity();



    for(int i = 0; i < 6; i++)
    {   
        Eigen::Matrix<double, 4, 4> Tf_i = calculate_TF(i);
        Tf1 = Tf1 * Tf_i;  
    }

    return Tf1;
}
