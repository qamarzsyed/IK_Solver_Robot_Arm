/**
 * @file robot_parameters.hpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-10-15
 * @copyright MIT License (c)
 *
 */

#ifndef INCLUDE_ROBOT_PARAMETERS_HPP_
#define INCLUDE_ROBOT_PARAMETERS_HPP_

#include <vector>
#include <string>
#include "Eigen/Dense"

class RobotParameters {
 private:
    std::vector<double> _dh_alpha;
    std::vector<double> _dh_a;
    std::vector<double> _dh_d;
    std::vector<double> _robot_angles;
 public:
    RobotParameters();
    std::string robot_name;
    Eigen::MatrixXd get_dh_parameters();
    std::vector<double> get_robot_angles();
    void set_robot_angles(std::vector<double> robot_angles);
};

#endif  // INCLUDE_ROBOT_PARAMETERS_HPP_
