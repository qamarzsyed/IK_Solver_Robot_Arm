/**
 * @file forward_kinematics.hpp
 * @author Driver : Tanmay Haldankar (tanmayh@umd.edu), Navigator: Sanchit Kedia
 * (sanchit@terpmail.umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Definition of Forward Kinematics class and Declaration of its Methods
 * @version 0.1
 * @date 2022-10-18
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
 private:
  Eigen::MatrixXd _dh_matrix;

 public:
  /**
   * @brief Calculate the DH transfromation matrix for each joint pair
   *
   * @param i integer value denoting the row of _dh_matrix to be considered
   * @return Eigen::Matrix<double, 4, 4> Returns the transformation matrix
   */
  Eigen::Matrix<double, 4, 4> calculate_TF(int i);
  /**
   * @brief Solve the forward kinematics for manipulator
   *
   * @return Eigen::Matrix<double, 4, 4> Returns the final Homogeneous
   * transformation matrix
   */
  Eigen::Matrix<double, 4, 4> solve_fk();
};

#endif  // INCLUDE_FORWARD_KINEMATICS_HPP_
