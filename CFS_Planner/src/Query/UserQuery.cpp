#include "cfslib/Query/UserQuery.hpp"

namespace cfslib
{
namespace query
{

UserQuery::UserQuery(
    const cfslib::math::Vector6d& x0,
    const cfslib::math::VectorJd& q0,
    const Eigen::MatrixXd& DH,
    const Eigen::MatrixXd& cart_traj_ref,
    const Eigen::VectorXd& joint_ref,
    const Eigen::MatrixXd& joint_limits,
    const Eigen::MatrixXd& joint_vel_limits,
    const Eigen::MatrixXd& joint_acc_limits,
    const Eigen::MatrixXd& joint_displacement_limits, 
    const Eigen::MatrixXd& base_frame,
    const Eigen::MatrixXi& collision_avoidance_links,
    const double& cart_vel_max,
    const double& cart_acc_max,
    const double& execution_time,
    const Mode& mode, 
    const std::string& config_fname,
    const std::string& robo_cap_fname,
    const std::string& obstacles_fname):
    x_               {x0},
    q_               {q0},
    DH_              {DH},
    cart_traj_ref_   {cart_traj_ref},
    ref_frame_       {Eigen::Matrix4d::Identity()},
    ref_frame_vel_   {Eigen::Vector3d::Zero()},
    joint_ref_       {joint_ref},
    obstacles_fname_ {obstacles_fname},
    robo_cap_fname_  {robo_cap_fname},
    config_fname_     {config_fname},
    joint_limits_     {joint_limits},
    joint_vel_limits_ {joint_vel_limits},
    joint_acc_limits_ {joint_acc_limits},
    joint_displacement_limits_ {joint_displacement_limits},
    base_frame_       {base_frame},
    execution_time_   {execution_time},
    mode_             {mode},
    collision_avoidance_links_ {collision_avoidance_links},
    cart_vel_max_     {cart_vel_max},
    cart_acc_max_     {cart_acc_max}
{
    std::cout << "\n"
        << "--------------------------------------------------------------------------\n"
        << "|                         User Query established!                        |\n"
        << "--------------------------------------------------------------------------\n";
}

UserQuery::UserQuery(
    const cfslib::math::Vector6d& x0,
    const cfslib::math::VectorJd& q0,
    const Eigen::MatrixXd& DH,
    const Eigen::MatrixXd& cart_traj_ref,
    const Eigen::MatrixXd& ref_frame,
    const Eigen::MatrixXd& ref_frame_vel,
    const Eigen::VectorXd& joint_ref,
    const Eigen::MatrixXd& joint_limits,
    const Eigen::MatrixXd& joint_vel_limits,
    const Eigen::MatrixXd& joint_acc_limits, 
    const Eigen::MatrixXd& joint_displacement_limits,
    const Eigen::MatrixXd& base_frame,
    const Eigen::MatrixXi& collision_avoidance_links,
    const double& cart_vel_max,
    const double& cart_acc_max,
    const double& execution_time,
    const Mode& mode, 
    const std::string& config_fname,
    const std::string& robo_cap_fname,
    const std::string& obstacles_fname):
    x_               {x0},
    q_               {q0},
    DH_              {DH},
    cart_traj_ref_   {cart_traj_ref},
    ref_frame_       {ref_frame},
    ref_frame_vel_   {ref_frame_vel},
    joint_ref_       {joint_ref},
    obstacles_fname_ {obstacles_fname},
    robo_cap_fname_  {robo_cap_fname},
    config_fname_     {config_fname},
    joint_limits_     {joint_limits},
    joint_vel_limits_ {joint_vel_limits},
    joint_acc_limits_ {joint_acc_limits},
    joint_displacement_limits_ {joint_displacement_limits},
    base_frame_       {base_frame},
    execution_time_   {execution_time},
    mode_             {mode},
    collision_avoidance_links_ {collision_avoidance_links},
    cart_vel_max_     {cart_vel_max},
    cart_acc_max_     {cart_acc_max}
{
    std::cout << "\n"
        << "--------------------------------------------------------------------------\n"
        << "|                         User Query established!                        |\n"
        << "--------------------------------------------------------------------------\n";
}


}
}