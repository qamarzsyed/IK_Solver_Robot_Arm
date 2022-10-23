/**
 * @file inverse_kinematics.hpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu) , Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-21
 * @copyright MIT License (c)
 *
 */
#include "../include/inverse_kinematics.hpp"

#include <math.h>

#include <cmath>
#include <iostream>
#include <ostream>
#include <vector>

#include "../include/forward_kinematics.hpp"
#include "../include/robot_parameters.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

InverseKinematics::InverseKinematics() {
  _eff_angles = {0, 0, 0};
  _eff_position = {0, 0, 0};
}

vector<double> InverseKinematics::get_eff_angles() { return _eff_angles; }

vector<double> InverseKinematics::get_eff_position() { return _eff_position; }

void InverseKinematics::set_eff_angles(vector<double> eff_angles) {
  _eff_angles = eff_angles;
}

void InverseKinematics::set_eff_position(vector<double> eff_position) {
  _eff_position = eff_position;
}

vector<double> InverseKinematics::solve_ik() {
  ForwardKinematics f;
  std::vector<double> robot_angles = {0, 0, 0, 0, 0, 0};

  return robot_angles;
}
