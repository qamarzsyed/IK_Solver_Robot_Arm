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

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using std::vector;

Matrix4d ForwardKinematics::calculate_TF(int j) {
  _dh_matrix = get_dh_parameters();
  int i = j;
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

Matrix4d ForwardKinematics::solve_fk() {
  Matrix4d Tf1 = Matrix4d::Identity();
  for (int i = 0; i < 6; i++) {
    Matrix4d Tf_i = calculate_TF(i);
    Tf1 = Tf1 * Tf_i;
  }
  Matrix3d Rotation = Tf1.block<3, 3>(0, 0);
  VectorXd Position1 = Tf1.col(3);
  vector<double> Position(3);
  //cout << "The position is vector " << endl;
  //cout << "[";
  //for (int i = 0; i < 3; i++) {
    //cout << Position1.coeff(i, 0) <<"," ;
  //}
  //cout << "]" << endl;
  //cout << "The Rotation matrix is \n" << Rotation << endl;
  // For the final implementatio n, we might need to instead pass the 3x3 matrix
  // for rotation and 3 element vector for position instead of 4x4 matrix.
  return Tf1;
}
