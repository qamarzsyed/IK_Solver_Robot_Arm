/**
 * @file robot_parameters.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-10-13
 * @copyright MIT License (c)
 *
 */
// Class to calculate forward kinematics

#ifndef FORWARD_KINEMATICS_HPP_  
#define FORWARD_KINEMATICS_HPP_

#include <iostream>
#include <vector>
#include "./robot_parameters.hpp"
#include "Eigen/Dense"

class ForwardKinematics: public RobotParameters {
 private:
    Eigen::MatrixXd _dh_matrix;
 public:
    
    Eigen::Matrix<double, 4, 4> calculate_TF( int i);
    Eigen::Matrix<double, 4, 4> solve_fk();
};

#endif // FORWARD_KINEMATICS_HPP_