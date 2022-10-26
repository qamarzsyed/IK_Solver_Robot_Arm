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
#include "robot_parameters.hpp"
#include <cmath>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::sqrt;
using std::vector;

InverseKinematics::InverseKinematics() {
  _eff_angles = {-M_PI_2, 0, -M_PI_2};
  _eff_position = {0, 0.45, 0.695};
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
  RobotParameters r;
  VectorXd dh_a = get_dh_parameters().col(2);
  VectorXd dh_d = get_dh_parameters().col(3);

  double x_03 = 0.00;
  double y_03 = 0.08;
  double z_03 = 0.00;

  double x_0e = _eff_position[0];
  double y_0e = _eff_position[1];
  double z_0e = _eff_position[2];

  double x_3e = x_0e - x_03;
  double y_3e = y_0e - y_03;
  double z_3e = z_0e - z_03;

  double theta_1 = atan2(y_3e, x_3e);
  double r1 = sqrt(pow(x_3e, 2) + pow(y_3e, 2));
  double r2 = z_3e - dh_d[0];
  double phi_2 = atan2(r2, r1);
  double r3 = sqrt(pow(r1, 2) + pow(r2, 2));
  double phi_3 = (pow(r3, 2) - pow(dh_a[1], 2) - pow(dh_d[3], 2)) /
                      (2 * dh_a[1] * dh_d[3]);
  double theta_3 = atan2(phi_3,sqrt(1-pow(phi_3,2)));
  double theta_2 = phi_2 - atan2(dh_a[1]+dh_d[2]*cos(theta_3),dh_d[2]*sin(theta_3));


  MatrixXd Tf_i(4, 4);
  MatrixXd Tf1(4, 4);
  Tf1 = MatrixXd::Identity(4, 4);
  vector<double> robot_angles = {theta_1, theta_2, theta_3, 0, 0, 0};
  r.set_robot_angles(robot_angles);

  for (int i = 0; i < 3; i++) {
    Tf_i = f.calculate_TF(i);
    Tf1 = Tf1 * Tf_i;
  }
  MatrixXd Rotation = Tf1.block<3, 3>(0, 0);

  MatrixXd R06(3, 3);
  R06(0, 0) = cos(_eff_angles[2]) * cos(_eff_angles[0]) -
              sin(_eff_angles[2]) * sin(_eff_angles[1]) * sin(_eff_angles[0]);
  R06(0, 1) = -cos(_eff_angles[1]) * sin(_eff_angles[0]);
  R06(0, 2) = cos(_eff_angles[2]) * sin(_eff_angles[0]) +
              sin(_eff_angles[2]) * sin(_eff_angles[1]) * cos(_eff_angles[0]);

  R06(2, 0) = -sin(_eff_angles[2]) * cos(_eff_angles[1]);
  R06(2, 1) = sin(_eff_angles[1]);
  R06(2, 2) = cos(_eff_angles[2]) * cos(_eff_angles[1]);

  R06(1, 2) = sin(_eff_angles[2]) * sin(_eff_angles[0]) -
              cos(_eff_angles[2]) * sin(_eff_angles[1]) * cos(_eff_angles[0]);
  R06(1, 1) = cos(_eff_angles[1]) * cos(_eff_angles[0]);
  R06(1, 0) = sin(_eff_angles[2]) * cos(_eff_angles[0]) +
              cos(_eff_angles[2]) * sin(_eff_angles[1]) * sin(_eff_angles[0]);

  MatrixXd R36(3, 3);
  R36 = Rotation.transpose() * R06;

  double theta_5 =
      atan2(sin(theta_1) * R06.coeff(0, 2) - cos(theta_1) * R06.coeff(1, 2),
            sqrt(1 - pow(sin(theta_1) * R06.coeff(0, 2) -
                             cos(theta_1) * R06.coeff(1, 2),
                         2)));
  double theta_6 =
      atan2(-sin(theta_1) * R06.coeff(0, 0) + cos(theta_1) * R06.coeff(1, 0),
            sin(theta_1) * R06.coeff(0, 1) - cos(theta_1) * R06.coeff(1, 1));
  double theta_4 =
      atan2(cos(theta_1) * cos(theta_2 + theta_3) * R06.coeff(0, 2) +
                sin(theta_1) * cos(theta_2 + theta_3) * R06.coeff(1, 2) +
                sin(theta_2 + theta_3) * R06.coeff(2, 2),
            -cos(theta_1) * sin(theta_2 + theta_3) * R06.coeff(0, 2) -
                sin(theta_1) * sin(theta_2 + theta_3) * R06.coeff(1, 2) +
                cos(theta_2 + theta_3) * R06.coeff(2, 2));

  robot_angles = {theta_1, theta_2, theta_3, theta_4, theta_5, theta_6};

  cout << "Theta 1 is " << theta_1 << endl;
  cout << "Theta 2 is " << theta_2 << endl;
  cout << "Theta 3 is " << theta_3 << endl;
  cout << "Theta 4 is " << theta_4 << endl;
  cout << "Theta 5 is " << theta_5 << endl;
  cout << "Theta 6 is " << theta_6 << endl;

  return robot_angles;