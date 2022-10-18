add_executable(ik_solver main.cpp robot_parameters.cpp forward_kinematics.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
