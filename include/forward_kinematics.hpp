/**
 * @file forward_kinematics.hpp
 * @author Driver : Tanmay Haldankar (tanmayh@umd.edu), Navigator: Sanchit Kedia
 * (sanchit@terpmail.umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Definition of Forward Kinematics class and Declaration of its Methods
 * @version 0.3
 * @date 2022-10-28
 * @copyright MIT License (c)
 *
 */

#ifndef INCLUDE_FORWARD_KINEMATICS_HPP_
#define INCLUDE_FORWARD_KINEMATICS_HPP_

#include "../include/robot_parameters.hpp"

/**
 * @brief Definition of the Forward Kinematics Class
 *
 */
class ForwardKinematics : public RobotParameters {
 public:
  double euler_x;
  double euler_y;
  double euler_z;
  /**
   * @brief Calculate the DH transfromation matrix for each joint pair
   *
   * @param i integer value denoting the row of _dh_matrix to be considered
   * @return Eigen::Matrix<double, 4, 4> Returns the transformation matrix
   */
  Eigen::Matrix4d calculate_TF(int i, Eigen::MatrixXd _dh_matrix);
  /**
   * @brief Solve the forward kinematics for manipulator
   *
   * @return Eigen::Matrix<double, 4, 4> Returns the final Homogeneous
   * transformation matrix
   */
  Eigen::Matrix4d solve_fk(Eigen::MatrixXd _dh_matrix);
};

#endif  // INCLUDE_FORWARD_KINEMATICS_HPP_
