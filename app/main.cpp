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

  vector<double> position_1 = {-0.30, 0.027, 0.30};
  vector<double> position_2 = {-0.40, 0.30, 0.40};
  vector<double> position_3 = {-0.15, 0.35, 0.60};
  vector<double> position_4 = {-0.05, 0.50, 0.20};
  vector<double> position_5 = {-0.22, 0.021, 0.48};
  vector<double> position_6 = {-0.30, 0.027, 0.30};

  MatrixXd position = MatrixXd::Identity(3, 6);
  position.col(0) = Map<VectorXd>(position_1.data(), position_1.size());
  position.col(1) = Map<VectorXd>(position_2.data(), position_2.size());
  position.col(2) = Map<VectorXd>(position_3.data(), position_3.size());
  position.col(3) = Map<VectorXd>(position_4.data(), position_4.size());
  position.col(4) = Map<VectorXd>(position_5.data(), position_5.size());
  position.col(5) = Map<VectorXd>(position_6.data(), position_6.size());

  vector<double> rotation_1 = {
      0.7490811, -0.6335077, -0.1937665, 0.5885996, 0.5022137,
      0.6335077, -0.3040198, -0.5885996, 0.7490811,
  };
  vector<double> rotation_2 = {
      0.6951739, -0.6872963, -0.2106111, 0.6052662, 0.4015926,
      0.6872963, -0.3877964, -0.6052662, 0.6951739,
  };
  vector<double> rotation_3 = {
      0.5223351, -0.7831739, 0.3373496, 0.6792093, 0.6212983,
      0.3907215, -0.5155975, 0.0250434, 0.8564648,
  };
  vector<double> rotation_4 = {
      0.5043118, -0.3630103, 0.7835134, 0.2775711, 0.9273402,
      0.2509869, -0.8176942, 0.0909050, 0.5684298,
  };
  vector<double> rotation_5 = {
      0.5870, -0.7946, -0.1549, 0.7779, 0.5007,
      0.3793, -0.2238, -0.3432, 0.9121,
  };
  vector<double> rotation_6 = {
      0.7489, -0.6335, -0.1939, 0.5887, 0.5020,
      0.6335, -0.3039, -0.5887, 0.7489,
  };

  MatrixXd rotation = MatrixXd::Identity(9, 6);
  rotation.col(0) = Map<VectorXd>(rotation_1.data(), rotation_1.size());
  rotation.col(1) = Map<VectorXd>(rotation_2.data(), rotation_2.size());
  rotation.col(2) = Map<VectorXd>(rotation_3.data(), rotation_3.size());
  rotation.col(3) = Map<VectorXd>(rotation_4.data(), rotation_4.size());
  rotation.col(4) = Map<VectorXd>(rotation_5.data(), rotation_5.size());
  rotation.col(5) = Map<VectorXd>(rotation_6.data(), rotation_6.size());

  for (int i = 0; i < 6; i++) {
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
    std::cout << "Euler X, Y and Z angles are " << f1.euler_x << " "
            << f1.euler_y << " " << f1.euler_z << std::endl;
  }
  return 0;
}
