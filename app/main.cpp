/**
 * @file main.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay
 * Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief Program to execute the inverse kinematics and forward kinematics
 * @version 0.2
 * @date 2022-10-18
 * @copyright MIT License (c)
 *
 */
#include <iostream>

#include "../include/forward_kinematics.hpp"

using std::cout;
using std::endl;

/**
 * @brief Main Function
 *
 * @return int 0
 */
int main() {
  RobotParameters r1;
  ForwardKinematics k1;
  cout << "The Robot for the project is " << r1.robot_name << endl;
  cout << "The DH parameters are : \n " << r1.get_dh_parameters() << endl;
  cout << "The Transformation Matrix \u2070T\u2081 is \n"
       << k1.calculate_TF(1) << endl;
  cout << "The Transformation Matrix \u2070T\u2086 is \n"
       << k1.solve_fk() << endl;
  return 0;
}
