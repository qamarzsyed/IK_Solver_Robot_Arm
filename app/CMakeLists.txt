add_executable(ik_solver main.cpp robot_parameters.cpp forward_kinematics.cpp inverse_kinematics.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(ik_solver ${PYTHON_LIBRARIES})