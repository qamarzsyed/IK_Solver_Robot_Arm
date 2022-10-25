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

#include "../include/inverse_kinematics.hpp"

using std::cout;
using std::endl;

/**
 * @brief Main Function
 *
 * @return int 0
 */
int main() {
  RobotParameters r1;
  InverseKinematics i1;
  ForwardKinematics f1;

  //cout << "Angles RP"<<endl;
  
  //for (int i = 0; i < 6; i++) {
  //  cout << r1.get_robot_angles()[i]<<endl;
  //}
  i1.solve_ik();
  r1.set_robot_angles({M_PI_2, -M_PI_2, 0, 0, 0, 0});

 // cout << "Angles RPI"<<endl;
  
  //for (int i = 0; i < 6; i++) {
  //  cout << r1.get_robot_angles()[i]<<endl;
 // }

  cout << "\nThe transformation matrix is " << endl;
  cout << f1.solve_fk() << endl;

  return 0;
} 
