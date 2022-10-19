/**
 * @file robot_parameters.hpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Definition of Robot Parameters class and Declaration of its Methods
 * @version 0.1
 * @date 2022-10-15
 * @copyright MIT License (c)
 *
 */

#ifndef INCLUDE_ROBOT_PARAMETERS_HPP_
#define INCLUDE_ROBOT_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "../include/Eigen/Dense"
/**
 * @brief Definition of the Robot Parameter Class
 *
 */
class RobotParameters {
 private:
  std::vector<double> _dh_alpha;
  std::vector<double> _dh_a;
  std::vector<double> _dh_d;
  std::vector<double> _robot_angles;

 public:
  /**
   * @brief Construct a new Robot Parameters object to assign default values
   *
   */
  RobotParameters();
  std::string robot_name;
  /**
   * @brief Compute the dh parameters matrix
   *
   * @return Eigen::MatrixXd  Returns DH matrix
   */
  Eigen::MatrixXd get_dh_parameters();
  /**
   * @brief Gets the robot angles
   *
   * @return std::vector<double>  Returns the robot angles
   */
  std::vector<double> get_robot_angles();
  /**
   * @brief Sets the robot angles
   *
   * @param robot_angles Sets the robot angles from the ik solver output
   */
  void set_robot_angles(std::vector<double> robot_angles);
};

#endif  // INCLUDE_ROBOT_PARAMETERS_HPP_
