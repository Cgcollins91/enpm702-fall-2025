/**
 * @file robot_control.cpp
 * @author Chris Collins (ccollin5@umd.edu)
 * @brief  Filter function used to limit arm velocity definition 
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_control.hpp"

void apply_filter(std::vector<JointState> &traj,
                  std::function<JointState(const JointState &)> filter)
{   // Loop through each JointState in traj and apply filter
    for (auto &s : traj)
    {
        s = filter(s);
    }
}
