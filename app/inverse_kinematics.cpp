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
using std::cout;
using std::endl;
using std::vector;

InverseKinematics::InverseKinematics() {
  _eff_angles = {-M_PI_2, -M_PI_2, 0};
  _eff_position = {0, 370, 695};
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
  VectorXd dh_a = get_dh_parameters().col(2);

  double z_03 = 0.695;
  double x_03 = 0.00;
  double y_03 = 0.370;
  double r2 = z_03 - dh_a[0];
  double r3 = sqrt(x_03 * x_03 + y_03 * y_03 + r2 * r2);

  double theta_3 = M_PI - acos(((r3 * r3) - (dh_a[1] * dh_a[1]) - (dh_a[2] * dh_a[2]))/(-2 * dh_a[1] * dh_a[2]));
  cout << "Theta 3 is " << theta_3 << endl;

  double theta_1 = atan2(y_03, x_03);

  double r1 = sqrt(x_03 * x_03 + y_03 * y_03);

  double phi_1 =
      atan(dh_a[2] * sin(theta_3)/(dh_a[1] + dh_a[2] * cos(theta_3)));

  double theta_2 = atan(r2 / r1) - phi_1;

  cout<< "t3" << phi_1 << endl;
  cout << ((r3 * r3) - (dh_a[1] * dh_a[1]) - (dh_a[2] * dh_a[2])) << " " << (-2 * dh_a[1] * dh_a[2]) << " " << (((r3 * r3) - (dh_a[1] * dh_a[1]) - (dh_a[2] * dh_a[2]))/(-2 * dh_a[1] * dh_a[2])) << endl;

  MatrixXd Tf_i(4, 4);
  MatrixXd Tf1(4, 4);
  Tf1 = MatrixXd::Identity(4, 4);
  for (int i = 0; i < 3; i++) {
    Tf_i = f.calculate_TF(i);
    Tf1 = Tf1 * Tf_i;
  }
  MatrixXd Rotation = Tf1.block<3, 3>(0, 0);

  MatrixXd R06(3, 3);
  R06(0, 0) = cos(_eff_angles[2]) * cos(_eff_angles[1]);
  R06(0, 1) = -sin(_eff_angles[2]) * cos(_eff_angles[0]) +
              cos(_eff_angles[2]) * sin(_eff_angles[1]) * sin(_eff_angles[0]);
  R06(0, 2) = sin(_eff_angles[2]) * sin(_eff_angles[0]) +
              cos(_eff_angles[2]) * sin(_eff_angles[1]) * cos(_eff_angles[0]);
  R06(1, 0) = sin(_eff_angles[2]) * cos(_eff_angles[1]);
  R06(1, 1) = cos(_eff_angles[2]) * cos(_eff_angles[0]) +
              sin(_eff_angles[2]) * sin(_eff_angles[1]) * sin(_eff_angles[0]);
  R06(1, 2) = -cos(_eff_angles[2]) * sin(_eff_angles[0]) +
              sin(_eff_angles[2]) * sin(_eff_angles[1]) * cos(_eff_angles[0]);
  R06(2, 0) = -sin(_eff_angles[1]);
  R06(2, 1) = cos(_eff_angles[1]) * sin(_eff_angles[0]);
  R06(2, 2) = cos(_eff_angles[1]) * cos(_eff_angles[0]);

  MatrixXd R36(3, 3);
  R36 = Rotation.inverse() * R06;

  cout << "R06\n" << R06 << endl;

  double theta_5 = acos(R06.coeff(2, 2));
  double theta_6 = acos(-R06.coeff(2, 0) / sin(theta_5));
  double theta_4 = acos(R06.coeff(1, 2) / sin(theta_5));

  vector<double> robot_angles = {theta_1, theta_2, theta_3,
                                 theta_4, theta_5, theta_6};

  cout << "Angles I" << endl;

  for (int i = 0; i < 6; i++) {
    cout << robot_angles[i] << endl;
  }

  cout << "Theta 1 is " << theta_1 << endl;
  cout << "Theta 2 is " << theta_2 << endl;

  return robot_angles;
}
