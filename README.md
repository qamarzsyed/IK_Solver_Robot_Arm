# 6 DOF Manipulator - Inverse Kinematics Solver

[![Build Status](https://github.com/qamarzsyed/IK_Solver_Robot_Arm/actions/workflows/cmake.yml/badge.svg)
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
