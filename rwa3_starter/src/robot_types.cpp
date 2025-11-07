/**
 * @file robot_types.cpp
 * @author Chris Collins (ccollin5@umd.edu)
 * @brief  Print Joint State and Pose State Function Definitions
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_types.hpp"
#include <iomanip>

// ==== Print Functions ====//
// Print JointState to console
void print_joint_state(const JointState& js) {
    std::cout << std::fixed << std::setprecision(4)
              << "θ1 = " << js.theta1 << " rad | "
              << "θ2 = " << js.theta2 << " rad | "
              << "dθ1 = " << js.dtheta1 << " rad/s | "
              << "dθ2 = " << js.dtheta2 << " rad/s\n";
}
// Print EndEffectorPose to console
void print_pose(const EndEffectorPose& ps) {
    std::cout << std::fixed << std::setprecision(4)
              << "x = " << ps.x << " m,  "
              << "y = " << ps.y << " m \n";
}