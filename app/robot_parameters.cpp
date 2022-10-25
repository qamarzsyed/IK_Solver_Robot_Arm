/**
 * @file robot_parameters.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Program to define the Methods of Robot Parameters Class
 * @version 0.11
 * @date 2022-10-13
 * @copyright MIT License (c)
 *
 */

#include "../include/robot_parameters.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;
using std::vector;

RobotParameters::RobotParameters() {
  robot_name = "Kuka KR5";
  _robot_angles = {M_PI_2, -M_PI_2, 0, 0, 0, 0};
  _dh_alpha = {-M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, M_PI};
  _dh_a = {0.075, 0.270, 0.090, 0, 0, 0};
  _dh_d = {0.335, 0, 0, -0.295, 0, -0.080};
  
}

MatrixXd RobotParameters::get_dh_parameters() {
  MatrixXd DH(6, 4);
  DH.col(0) = Map<VectorXd>(get_robot_angles().data(),
                                          _robot_angles.size());
  DH.col(1) = Map<VectorXd>(_dh_alpha.data(), _dh_alpha.size());
  DH.col(2) = Map<VectorXd>(_dh_a.data(), _dh_a.size());
  DH.col(3) = Map<VectorXd>(_dh_d.data(), _dh_d.size());
  return DH;
}

vector<double> RobotParameters::get_robot_angles() { return _robot_angles; }

void RobotParameters::set_robot_angles(vector<double> robot_angles) {
  _robot_angles = robot_angles;
}
