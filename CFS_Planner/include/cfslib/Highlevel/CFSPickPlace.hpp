#pragma once
#include "cfslib/cfslib.hpp"

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, bool> CFSPickPlace(
    const Eigen::MatrixXd& current_pose_joint,
    const Eigen::Matrix4d& mid_pose_cartesian,
    const Eigen::Matrix4d& target_pose_cartesian,
    const std::string& robot_model,
    const int& movement_steps,
    const double& joint_limit,
    const std::string& robot_cap_path,
    const std::string& obs_path
);
