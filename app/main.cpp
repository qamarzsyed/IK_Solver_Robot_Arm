/**
 * @file main.cpp
 * @author Driver : Sanchit Kedia (sanchit@terpmail.umd.edu), Navigator: Tanmay Haldankar (tanmayh@umd.edu), Design Keeper: Qamar Syed (qsyed@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-10-13
 * @copyright MIT License (c)
 *
 */
#include <iostream>

#include "../include/robot_parameters.hpp"

using std::cout;
using std::endl;

/**
 * @brief Main Function 
 * 
 * @return int 0
 */
int main() {
    RobotParameters r1;
    cout << "The Robot for the project is " << r1.robot_name << endl;
    cout << "The DH parameters are : \n " << r1.get_dh_parameters() << endl;
    return 0;
}
