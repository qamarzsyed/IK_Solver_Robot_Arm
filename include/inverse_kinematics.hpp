/**
 * @file inverse_kinematics.hpp
 * @author Driver : Tanmay Haldankar (tanmayh@umd.edu), Navigator: Sanchit Kedia
 * (sanchit@terpmail.umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-21
 * @copyright MIT License (c)
 *
 */
// class to calculate inverse kinematics

#ifndef INVERSE_KINEMATICS_HPP_
#define INVERSE_KINEMATICS_HPP_

#include <iostream>
#include <vector>

#include "../include/robot_parameters.hpp"
#include "Eigen/Dense"

/**
 * @brief Definition of Inverse Kinematics Class
 *
 */
class InverseKinematics : public RobotParameters {
 private:
  std::vector<double> _eff_angles;
  std::vector<double> _eff_position;

 public:
  /**
   * @brief Construct a new Inverse Kinematics object to assign default values
   *
   */
  InverseKinematics();
  /**
   * @brief Gets the end effector angles
   * 
   * @return std::vector<double>  Returns a vector containing end effector angles
   */
  std::vector<double> get_eff_angles();
  /**
   * @brief Gets the end effector position
   * 
   * @return std::vector<double>  Returns a vector containing end effector postion
   */
  std::vector<double> get_eff_position();
  /**
   * @brief Sets the end effector angles
   * 
   * @param eff_angles  Vector containing end effector angles
   */
  void set_eff_angles(std::vector<double> eff_angles);
  /**
   * @brief Sets the end effector position
   * 
   * @param eff_position  Vector containing end effector position
   */
  void set_eff_position(std::vector<double> eff_position);
  /**
   * @brief Method to solve the inverse kinematics of the Manipulator
   * 
   * @return std::vector<double> 
   */
  std::vector<double> solve_ik();
};

#endif  // INVERSE_KINEMATICS_HPP_