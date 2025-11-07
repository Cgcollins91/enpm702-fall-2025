/**
 * @file robot_kinematics.hpp
 * @author Chris Collins (ccollin5@umd.edu)
 * @brief Calculate Forward Kinematics of 2 DoF Robot Arm
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include "robot_types.hpp"
#include <cmath>


/**
 * @brief Calculate Forward Kinematics of 2 DoF Robot Arm
 * 
 * @tparam State 
 * @param s 
 * @param L1 
 * @param L2 
 * @return EndEffectorPose 
 */
template <typename State>
EndEffectorPose forward_kinematics(const State& s,
                                   double L1 = k_link1,
                                   double L2 = k_link2)
{
    EndEffectorPose pose{};
    pose.x = L1*std::cos(s.theta1) + L2*std::cos(s.theta1 + s.theta2);
    pose.y = L1*std::sin(s.theta1) + L2*std::sin(s.theta1 + s.theta2);
    return pose;
}