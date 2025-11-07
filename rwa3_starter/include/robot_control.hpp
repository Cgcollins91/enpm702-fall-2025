/**
 * @file    robot_control.hpp
 * @author  Chris Collins (ccollin5@umd.edu)
 * @brief   Simple 2 DoF Robot Arm Control Header: Define Linear Interpolation Template and Define Trajectory Filter Function
 * @version 0.1
 * @date    2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include "robot_types.hpp"
#include <vector>
#include <functional>


/**
 * @brief Creates linear interpolation between given start & goal state for 2-D, 2 DoF Robot Arm and Finite-Difference Arm Velocities
 * 
 * @tparam State 
 * @param start 
 * @param goal 
 * @param alpha 
 * @return State 
 */
template <typename State>
State interpolate_linear(const State& start, const State& goal, double alpha, int k_num_samples)
{
    // Clamp Alpha to [0, 1] just in case
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    State out{};

    // Interpolate theta1, theta2 linearly
    // Set dtheta1, dtheta2 using finite-difference with (goal - start)/(num_samples - 1)
    out.theta1  = start.theta1 + alpha * (goal.theta1 - start.theta1);
    out.theta2  = start.theta2 + alpha * (goal.theta2 - start.theta2);
    out.dtheta1 = (goal.theta1 - start.theta1) / (k_num_samples-1); // Finite-difference
    out.dtheta2 = (goal.theta2 - start.theta2) / (k_num_samples-1); // Finite-difference
    return out;
}


/**
 * @brief Takes Trajectory and applies given filter function to each JointState in Trajectory
 * 
 *
 * @param traj 
 * @param filter 
 **/
void apply_filter(std::vector<JointState>& traj,
                  std::function<JointState(const JointState&)> filter);
