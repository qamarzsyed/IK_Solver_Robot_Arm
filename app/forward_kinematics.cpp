/**
 * @file forward_kinematics.cpp
 * @author Driver : Tanmay Haldankar (tanmayh@umd.edu), Navigator: Sanchit Kedia
 * (sanchit@terpmail.umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Program to define the Methods of Forward Kinematics Class.
 * @version 0.12
 * @date 2022-10-18
 * @copyright MIT License (c)
 *
 */
#include "../include/forward_kinematics.hpp"
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::VectorXd;

Matrix4d ForwardKinematics::calculate_TF(int i, Eigen::MatrixXd _dh_matrix) {
  Matrix4d Tf{
      {cos(_dh_matrix.coeff(i, 0)),
       -sin(_dh_matrix.coeff(i, 0)) * cos(_dh_matrix.coeff(i, 1)),
       sin(_dh_matrix.coeff(i, 0)) * sin(_dh_matrix.coeff(i, 1)),
       _dh_matrix.coeff(i, 2) * cos(_dh_matrix.coeff(i, 0))},
      {sin(_dh_matrix.coeff(i, 0)),
       cos(_dh_matrix.coeff(i, 0)) * cos(_dh_matrix.coeff(i, 1)),
       -cos(_dh_matrix.coeff(i, 0)) * sin(_dh_matrix.coeff(i, 1)),
       _dh_matrix.coeff(i, 2) * sin(_dh_matrix.coeff(i, 0))},
      {0, sin(_dh_matrix.coeff(i, 1)), cos(_dh_matrix.coeff(i, 1)),
       _dh_matrix.coeff(i, 3)},
      {0, 0, 0, 1},
  };
  return Tf;
}

Matrix4d ForwardKinematics::solve_fk(Eigen::MatrixXd _dh_matrix) {
  Matrix4d Tf1 = Matrix4d::Identity();
  for (int i = 0; i < 6; i++) {
    Matrix4d Tf_ = calculate_TF(i, _dh_matrix);
    Tf1 = Tf1 * Tf_;
  }
  Eigen::Matrix3d Rotation = Tf1.block(0, 0, 3, 3);

  euler_x = atan(Rotation.coeff(2, 1) / (Rotation.coeff(2, 2)));
  euler_y =
      atan(-Rotation.coeff(2, 0) /
           (sqrt(pow(Rotation.coeff(2, 1), 2) + pow(Rotation.coeff(2, 2), 2))));
  euler_z = atan(Rotation.coeff(1, 0) / (Rotation.coeff(0, 0)));

  return Tf1;
}
