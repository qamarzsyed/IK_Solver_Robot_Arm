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

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::sqrt;
using std::vector;

InverseKinematics::InverseKinematics() {
  Matrix3d E{
      {-0.1268, -0.9427, -0.3085},
      {0.9268, -0.0017, -0.3756},
      {0.3535, -0.3336, 0.8739},
  };
  R_E = E;
  _eff_position = {-0.2283, 0.0216, 0.4788};
}

void InverseKinematics::set_eff_rotation(Eigen::Matrix3d _R_E) { R_E = _R_E; }

Eigen::Matrix3d InverseKinematics::get_eff_rotation() { return R_E; }

Eigen::Vector3d InverseKinematics::get_eff_position() { return _eff_position; }

void InverseKinematics::set_eff_position(Eigen::Vector3d eff_position) {
  _eff_position = eff_position;
}

vector<double> InverseKinematics::solve_ik() {
  ForwardKinematics f;
  RobotParameters r;

  VectorXd dh_a = get_dh_parameters().col(2);
  VectorXd dh_d = get_dh_parameters().col(3);

  VectorXd c_p = _eff_position - dh_d[5] * R_E.col(2);

  double d3 = sqrt(pow(c_p[0], 2) + pow(c_p[1], 2) +
                   pow((c_p[2] - dh_d[0]), 2) - pow(dh_d[1], 2));

  double D = (c_p[2] - dh_d[0]) / d3;
  double theta_2 = atan2(D, sqrt(1 - pow(D, 2)));

  Matrix2d M{
      {-dh_d[1], -dh_d[2] * cos(theta_2)},
      {-dh_d[2] * cos(theta_2), dh_d[1]},
  };
  Eigen::Vector2d R{c_p[0], c_p[1]};
  VectorXd sol = M.inverse() * R;
  double theta_1 = atan2(sol[0], sol[1]);

  double theta_3 = -1.5708;

  vector<double> I_Theta = {theta_1, theta_2 - 1.5708, theta_3, 0, 0, 0};

  set_robot_angles(I_Theta);
  set_dh_parameters();

  Matrix4d H0_3 = Matrix4d::Identity();
  for (int i = 0; i < 3; i++) {
    Matrix4d Tf_ = f.calculate_TF(i, get_dh_parameters());
    H0_3 = H0_3 * Tf_;
  }
  Matrix3d R0_3 = H0_3.block<3, 3>(0, 0);

  Matrix3d R3_6 = R0_3.transpose() * R_E;

  double theta_4 = atan2(R3_6(1, 2), R3_6(0, 2));

  double theta_5 =
      atan2(sqrt(pow(R3_6(1, 2), 2) + pow(R3_6(0, 2), 2)), R3_6(2, 2));

  double theta_6 = atan2(R3_6(2, 1), -R3_6(2, 0));

  vector<double> F_Theta = {theta_1, theta_2 - 1.5708, theta_3,
                            theta_4, theta_5, theta_6};

  return F_Theta;
}
