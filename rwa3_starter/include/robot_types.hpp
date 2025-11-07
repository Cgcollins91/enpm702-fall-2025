/**
 * @file robot_types.hpp
 * @author Chris Collins (ccollin5@umd.edu)
 * @brief  Defines Robot State Structs(Joint State, End Effector Pose) and consts for Robot Geometry and movement interpolation
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include <iostream>


/**
 * @brief Define 2-D Robot Joint State for 2 DoF Robot Arm
 * 
 */
struct JointState {
    double theta1;        // Joint 1 angle [rad]
    double theta2;        // Joint 2 angle [rad]
    
    // Initialize Default Velocities to zero
    double dtheta1 = 0.0; // Joint 1 velocity [rad/s]
    double dtheta2 = 0.0; // Joint 2 velocity [rad/s]
};

/**
 * @brief Define 2-D Robot End Effector Pose for 2 DoF Robot Arm
 * 
 */
struct EndEffectorPose {
    double x; // [m]
    double y; // [m]
};

// Print function Definitions (implemented in main.cpp)
void print_joint_state(const JointState& js);
void print_pose(const EndEffectorPose& ps);

inline constexpr double k_link1{0.5};        // Arm Length of Robot Link 1 [m]
inline constexpr double k_link2{0.3};        // Arm Length of Robot Link 2 [m]
inline constexpr double k_vel_limit{1.0};    // Robot Arm Angular Velocity Limit [rad/s]
inline constexpr int    k_num_samples{21};   // Number of Trajectory Points, includes endpoints
inline constexpr double k_alpha_step{1.0 / (k_num_samples - 1)}; // Step size for interpolation 
