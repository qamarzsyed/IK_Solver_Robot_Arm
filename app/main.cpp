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
#include "../include/inverse_kinematics.hpp"


using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * @brief Main Function
 *
 * @return int 0
 */
int main() {
  RobotParameters r1;
  InverseKinematics i1;
  ForwardKinematics f1;

  vector<double> position_1 = {-0.2283, 0.0216, 0.4788};
  vector<double> position_2 = {-0.3, 0.027, 0.3};
  vector<double> position_3 = {-0.3, 0.06, 0.25};
  vector<double> position_4 = {-0.25, 0.1, 0.35};

  MatrixXd position = MatrixXd::Identity(3, 4);
  position.col(0) = Map<VectorXd>(position_1.data(), position_1.size());
  position.col(1) = Map<VectorXd>(position_2.data(), position_2.size());
  position.col(2) = Map<VectorXd>(position_3.data(), position_3.size());
  position.col(3) = Map<VectorXd>(position_4.data(), position_4.size());

  vector<double> rotation_1 = {
      0.5870, -0.7946, -0.1549, 0.7779, 0.5007, 0.3793, -0.2238, -0.3432, 0.9121,
  };
  vector<double> rotation_2 = {
      0.7489, -0.6335, -0.1939, 0.5887, 0.5020, 0.6335, -0.3039, -0.5887, 0.7489,
  };
  vector<double> rotation_3 = {
      0.7882, -0.6056, -0.1087, 0.4903, 0.5115, 0.7056, -0.3717, -0.6095, 0.7001,
  };
  vector<double> rotation_4 = {
      0.6602, -0.7505, -0.0282, 0.6223, 0.5256, 0.5800, -0.4204, -0.4005, 0.8141,
  };

  MatrixXd rotation = MatrixXd::Identity(9, 4);
  rotation.col(0) = Map<VectorXd>(rotation_1.data(), rotation_1.size());
  rotation.col(1) = Map<VectorXd>(rotation_2.data(), rotation_2.size());
  rotation.col(2) = Map<VectorXd>(rotation_3.data(), rotation_3.size());
  rotation.col(3) = Map<VectorXd>(rotation_4.data(), rotation_4.size());

  for (int i = 0; i < 4; i++) {
    Matrix3d Rotation{
        {
            rotation.coeff(0, i),
            rotation.coeff(1, i),
            rotation.coeff(2, i),
        },
        {
            rotation.coeff(3, i),
            rotation.coeff(4, i),
            rotation.coeff(5, i),
        },
        {
            rotation.coeff(6, i),
            rotation.coeff(7, i),
            rotation.coeff(8, i),
        },
    };

    Vector3d position_v = position.col(i);
    i1.set_eff_position(position_v);
    i1.set_eff_rotation(Rotation);

    r1.set_robot_angles(i1.solve_ik());
    r1.set_dh_parameters();
    cout << "\nThe FK solution for position and rotation " << i + 1 << " is:\n"
         << endl;
    cout << f1.solve_fk(r1.get_dh_parameters()) << endl;
  }
  return 0;
}
