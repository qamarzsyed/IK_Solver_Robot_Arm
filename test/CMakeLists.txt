set(GTEST_SHUFFLE 1)

add_executable(
    code_test
    main.cpp
    code_test.cpp
    ../app/robot_parameters.cpp
    ../app/forward_kinematics.cpp
    ../app/inverse_kinematics.cpp
)

target_include_directories(code_test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(code_test PUBLIC gtest)
