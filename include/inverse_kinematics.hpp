/**
 * @file inverse_kinematics.hpp
 * @author Driver : Tanmay Haldankar (tanmayh@umd.edu), Navigator: Sanchit Kedia
 * (sanchit@terpmail.umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief
 * @version 0.3
 * @date 2022-10-28
 * @copyright MIT License (c)
 *
 */
// class to calculate inverse kinematics

#ifndef INVERSE_KINEMATICS_HPP_
#define INVERSE_KINEMATICS_HPP_

#include "../include/forward_kinematics.hpp"


/**
 * @brief Definition of Inverse Kinematics Class
 *
 */
class InverseKinematics : public RobotParameters {
 private:
  Eigen::Vector3d _eff_position;
  Eigen::Matrix3d R_E;

 public:
  /**
   * @brief Construct a new Inverse Kinematics object to assign default values
   *
   */
  InverseKinematics();
  /**
   * @brief Sets the end effector rotation matrix
   *
   * @param _R_E Rotation matrix for end effector with respect to base frame
   */
  void set_eff_rotation(Eigen::Matrix3d _R_E);
  /**
   * @brief Gets the end effector rotation matrix
   *
   * @return Eigen::Matrix3d Returns End effector rotation matrix
   */
  Eigen::Matrix3d get_eff_rotation();
  /**
   * @brief Gets the end effector position
   *
   * @return std::vector<double>  Returns a vector containing end effector
   * postion
   */
  Eigen::Vector3d get_eff_position();
  /**
   * @brief Sets the end effector position
   *
   * @param eff_position  Vector containing end effector position
   */
  void set_eff_position(Eigen::Vector3d eff_position);
  /**
   * @brief Method to solve the inverse kinematics of the Manipulator
   *
   * @return std::vector<double>
   */
  std::vector<double> solve_ik();
};

#endif  // INVERSE_KINEMATICS_HPP_
