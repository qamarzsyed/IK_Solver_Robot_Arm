/**
 * @file robot_parameters.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Program to define the Methods of Robot Parameters Class
 * @version 0.12
 * @date 2022-10-28
 * @copyright MIT License (c)
 *
 */

#include "../include/robot_parameters.hpp"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

RobotParameters::RobotParameters() {
  robot_name = "Stanford Manipulator";
  _robot_angles = {0, 0, 0, 0, 0, 0};
  _dh_alpha = {-1.5708, 1.5708, 0, -1.5708, 1.5708, 0};
  _dh_a = {0, 0, 0, 0, 0, 0};
  _dh_d = {0.25, 0.15, 0.2, 0, 0, 0.1};
  DH = MatrixXd::Identity(6, 4);
}

void RobotParameters::set_dh_parameters() {
  DH.col(0) = Map<VectorXd>(get_robot_angles().data(), _robot_angles.size());
  DH.col(1) = Map<VectorXd>(_dh_alpha.data(), _dh_alpha.size());
  DH.col(2) = Map<VectorXd>(_dh_a.data(), _dh_a.size());
  DH.col(3) = Map<VectorXd>(_dh_d.data(), _dh_d.size());
}

MatrixXd RobotParameters::get_dh_parameters() {
  set_dh_parameters();
  return DH;
}

vector<double> RobotParameters::get_robot_angles() { return _robot_angles; }

void RobotParameters::set_robot_angles(vector<double> robot_angles) {
  _robot_angles = robot_angles;
}
