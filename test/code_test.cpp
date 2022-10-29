/**
 * @file code_test.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Program to perform unit testing
 * @version 0.3
 * @date 2022-10-15
 * @copyright MIT License (c)
 *
 */
#include <gtest/gtest.h>

#include "../include/forward_kinematics.hpp"

/**
 * @brief Construct a new TEST to check if the robot angles vector is empty
 *
 */
TEST(Robot_Parameters, CheckAngles) {
  RobotParameters r1;
  std::vector<double> angles = r1.get_robot_angles();
  bool empty = angles.empty();
  EXPECT_FALSE(empty);
}
/**
 * @brief Construct a new TEST to check the size of the DH Parameters matrix
 *
 */
TEST(Robot_Parameters, CheckDH) {
  RobotParameters r1;
  Eigen::MatrixXd DH = r1.get_dh_parameters();
  EXPECT_EQ(DH.size(), 24);
}
/**
 * @brief Construct a new TEST to check if the robot angles are being set
 * correctly
 *
 */
TEST(Robot_Parameters, CheckSetAngles) {
  RobotParameters r1;
  std::vector<double> angles = {0, 0, 0, 0, 0, 0};
  r1.set_robot_angles(angles);
  std::vector<double> angles_o = r1.get_robot_angles();
  EXPECT_EQ(angles, angles_o);
}
/**
 * @brief Construct a new TEST to check the size of the DH transformation matrix
 *
 */
TEST(Forward_Kinematics, check_calculateTF) {
  ForwardKinematics k1;
  Eigen::Matrix<double, 4, 4> TF = k1.calculate_TF(1, k1.get_dh_parameters());
  EXPECT_EQ(TF.size(), 16);
}
/**
 * @brief Construct a new TEST to check the size of final homogeneous
 * transformation matrix
 *
 */
TEST(Forward_Kinematics, check_solvefk) {
  ForwardKinematics k1;
  Eigen::Matrix<double, 4, 4> HTF = k1.solve_fk(k1.get_dh_parameters());
  EXPECT_EQ(HTF.size(), 16);
}
