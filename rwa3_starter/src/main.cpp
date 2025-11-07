/**
 * @file main.cpp
 * @author Chris Collins
 * @brief 
 * @version 0.1
 * @date 2025-10-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_types.hpp"
#include "robot_kinematics.hpp"
#include "robot_control.hpp"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>

int main() {
    std::cout << "=== Robot Kinematics & Control (Starter Skeleton) ===\n\n";

    // 1) Start/Goal 
    const JointState start{0.0, 0.0};                // θ1=0, θ2=0
    const JointState goal{M_PI / 4.0, -M_PI };       // θ1=45°, θ2=-180°
    int filtered_count{0};                           // Count Number of Trajectory points clamped to velocity limits
    int print_step{5};                               // Only Print every 5th trajectory point to console
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Start   ->   θ1 = " << start.theta1 << " rad, θ2 = " << start.theta2 << " rad \n";
    std::cout << "Goal    ->   θ1 = "  << goal.theta1 << " rad, θ2 = "<<  goal.theta2  << " rad \n\n";
 

    // 2) Trajectory container (with unique ownership requirement)
    auto traj = std::make_unique<std::vector<JointState>>();
    traj->reserve(k_num_samples);

    // Create Trajectory points 
    for (int i {0}; i < k_num_samples; ++i) {
        // Force i to double for alpha calculation
        const double alpha = static_cast<double>(i) * k_alpha_step; 
        traj->push_back(interpolate_linear(start, goal, alpha, k_num_samples));
    }

    // Print Count of Unfiltered Trajectory Points
    std::cout << "Trajectory Points: " << traj->size()  << "\n\n";
    std::cout << "Unfiltered Trajectory (Every 5th Point Shown): \n";

    // Print every print_stepth JointState in the trajectory
    for (size_t i{0}; i < traj->size(); i += print_step) {
        std::cout << "[" << i << "]  ";
        print_joint_state( (*traj)[i]);
    }

    std::cout << '\n' << "Applying velocity-limit filter: |dθ| ≤ 1.0 rad/s" << '\n';

    // 3) Define & apply velocity-limit filter (lambda)
    auto velocity_limit_lambda = [&filtered_count](const JointState& s) -> JointState {
        JointState out = s; // Make copy before applying velocity limit in place

        // Clamp dtheta1/dtheta2 to be within [-k_vel_limit, +k_vel_limit]
        out.dtheta1 = std::clamp(out.dtheta1, -k_vel_limit, k_vel_limit);
        out.dtheta2 = std::clamp(out.dtheta2, -k_vel_limit, k_vel_limit);

        // Count how many times velocity filter is applied 
        if( out.dtheta1 != s.dtheta1 || out.dtheta2 != s.dtheta2){
            filtered_count++;
        }
        return out; // Return modified copy
    };

    // Apply velocity limit filter limiting dtheta to +/- k_vel_limit
    apply_filter(*traj, velocity_limit_lambda);

    // Report on filtering results
    if ( filtered_count == 0 ){
        std::cout << "  -> Filter applied successfully, all values within limits.\n\n";
    } else{
        std::cout << "  -> Filter applied successfully, " << (filtered_count)
                  << " points clamped to dθ limits \n\n";
    }

    // Print End Effector Pose at every print_step step 
    for (size_t i{0}; i < traj->size(); i += print_step) {
        std::cout << "[" << i << "]  ";
        print_joint_state( (*traj)[i]);
    }
    // 4) End-effector poses (shared ownership)
    auto ee_poses = std::make_shared<std::vector<EndEffectorPose>>();
    ee_poses->reserve(traj->size());

    // Compute end-effector poses for each JointState in the filtered trajectory
    std::cout << '\n' << "Computing end-effector poses for filtered trajectory... \n";
    std::cout << "Link Lengths: L1 = " << k_link1 << " m, L2 = " << k_link2 << " m \n";
    for (const auto& state : *traj){
        EndEffectorPose pose = forward_kinematics(state);
        ee_poses->push_back(pose);
    }

    std::cout << '\n';
    // Print every print_stepth EndEffectorPose in the trajectory
    for (size_t  i = 0; (i <  ee_poses->size()); i += print_step) {
        std::cout << "[" << i << "]  ";
        print_pose( (*ee_poses)[i]);
    }

    std::cout << "\n Summary: \n" << "-------------\n";
    std::cout << "- Total Joint States : " << ee_poses->size()  << "\n\n";
    std::cout << "- Velocity filter: active (|dθ| ≤ 1.0000) \n";

}
