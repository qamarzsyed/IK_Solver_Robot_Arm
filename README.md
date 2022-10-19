# 6 DOF Manipulator - Inverse Kinematics Solver

[![Build Status](https://github.com/qamarzsyed/IK_Solver_Robot_Arm/actions/workflows/cmake.yml/badge.svg)](https://github.com/qamarzsyed/IK_Solver_Robot_Arm/actions)
[![Coverage Status](https://coveralls.io/repos/github/qamarzsyed/IK_Solver_Robot_Arm/badge.svg?branch=main)](https://coveralls.io/github/qamarzsyed/IK_Solver_Robot_Arm?branch=main)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project Description

## Overview

- The aim of this project is to design and develop an inverse kinematics solver for a 6-axis Manipulator at ACME Robotics.
- This implementaion is based on the Kuka-KR 5 Manipulator, however it can provide a solution for any 6 axis             manipulator with a sperical wrist configuration. 
- Our software will compute and simulate a trajectory based on the path coordinates provided and will be integrated as a module into a future project over at ACME Robotics.

## Method
- Our implementation includes three methods, two for solving inverse and forwards kinematics for the manipulator, and one method to store the attributes of the specified robot arm.
- The solution for inverse kinematics will be calculated and stored as joint angles for each path coordinate provided.
- The forward kinematics solver will calculate end effector position based on the joint values and robot parameters provided. Denavit Hartenberg representation will be employed for both the inverse and forward kinematics solvers.
- A two sprint Agile Iterative Process approach and test driven development style is utilized in the making of this project.

## Contributors
- **Qamar Syed**  UID: 119128824 
- **Sanchit Kedia** UID: 119159620
- **Tanmay Haldankar** UID: 119175460

## Agile Iterative Process
- Product Backlog Sheet: https://docs.google.com/spreadsheets/d/1m7ZkmKvhe5lrAsWQkt8Hllt3Yv4Hs6iVAIpfvJ87dw4/edit?usp=sharing
- Sprint planning notes and review: https://docs.google.com/document/d/1zYB-wOUc6sJvRMuZ350FZNAi0NLVUb29pffhljllGPY/edit?usp=sharing

## Submission
- Project Proposal Video : https://drive.google.com/file/d/1DN1yFC1ftZCqTR0bdwy7rqwPNF0FwN4z/view?usp=sharing
- Project Phase 1 Video :

## Dependencies
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) :The project includes the Eigen library for linear algebra operations.


## Compilation
```
git clone "https://github.com/qamarzsyed/IK_Solver_Robot_Arm.git"
cd IK_Solver_Robot_Arm
mkdir build && cd build
cmake .. 
make
```

## Build for Code Coverage
```
cd build
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make 
make code_coverage
```

## Run

1. Once inside the `/build` directory after compilation, run: `./app/ik_solver`
2. To perform Google Test, run:  `./test/code_test`

## Cpplint

Change to the `/app` directory, and run:
```
cpplint main.cpp > ../results/cpplint_main.txt
cpplint forward_kinematics.cpp > ../results/cpplint_forward_kinematics.txt
cpplint robot_parameters.cpp > ../results/cpplint_robot_parameters.txt
cpplint ../test/code_test.cpp > ../results/cpplint_code_test.txt
cpplint ../include/forward_kinematics.hpp > ../results/cpplint_forward_kinematics_hpp.txt
cpplint ../include/robot_parameters.hpp > ../results/cpplint_robot_parameters_hpp.txt
```

## Cppcheck

Change to the `root` directory, and run:
```
cppcheck --enable=all --std=c++11 --force -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./include/Eigen/") > results/cppcheck.txt'
```

# Valgrind
```
cd build
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes ./app/ik_solver args  > ../results/valgrind.txt 2>&1
```

## License

MIT License

Copyright (c) 2022 Qamar Syed, Sanchit Kedia, Tanmay Haldankar

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.