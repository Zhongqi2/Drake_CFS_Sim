#pragma once
#include "cfslib/Utils/Common.hpp"
#include "cfslib/Robot/Robot.hpp"
#include "cfslib/Query/ProcessedQuery.hpp"

namespace cfslib
{
namespace trajectory
{
class Trajectory
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
    
        typedef std::shared_ptr<Trajectory> Ptr;
        typedef std::shared_ptr<Trajectory const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        bool CFS_valid_;
        bool vel_valid_;
        bool dense_valid_;
        bool collision_valid_;

        // CFS trajectory
        Eigen::MatrixXd joint_path_waypoints_;
        Eigen::MatrixXd joint_path_waypoints_critical_;
        Eigen::MatrixXd cart_path_waypoints_;
        
        // Input reference trajectory
        Eigen::MatrixXd cart_init_reference_;
        Eigen::MatrixXd joint_init_reference_;

        // Distance to obstacles
        Eigen::MatrixXd dist_profile_;
        Eigen::MatrixXd dist_init_profile_;
        
        Eigen::MatrixXd time_profile_;
        
        // Cartesian velocity & acceleration
        Eigen::MatrixXd cart_vel_profile_;
        Eigen::MatrixXd cart_vel_profile_reference_;
        Eigen::MatrixXd cart_acc_profile_;
        Eigen::MatrixXd cart_acc_profile_reference_;

        // Joint velocity & acceleration
        Eigen::MatrixXd joint_vel_profile_;
        Eigen::MatrixXd joint_vel_profile_reference_;
        Eigen::MatrixXd joint_acc_profile_;
        Eigen::MatrixXd joint_acc_profile_reference_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        Eigen::MatrixXd DistanceProfile(Eigen::MatrixXd q, double dt,
                                        const robot::Robot::ConstPtr& robot_ptr,
                                        const query::ProcessedQuery::ConstPtr& processed_query_ptr);
        Eigen::MatrixXd Derivative(Eigen::MatrixXd q, double dt);

    public:
        Trajectory();
        ~Trajectory(){}

        // setter
        void GeneratePath(Eigen::MatrixXd q, 
                          const robot::Robot::ConstPtr& robot_ptr,
                          const query::ProcessedQuery::ConstPtr& processed_query_ptr);
        void set_status(bool flag) {CFS_valid_ = flag;};
        void set_vel_flag(bool flag) {vel_valid_ = flag;};
        void set_dense_flag(bool flag) {dense_valid_ = flag;};
        
        // getter
        bool status() {return CFS_valid_;};
        bool vel_flag() {return vel_valid_;};
        bool dense_flag() {return dense_valid_;};
        bool collision_flag() {return collision_valid_;};
        Eigen::MatrixXd joint_path(){return joint_path_waypoints_;};
        Eigen::MatrixXd joint_path_critical(){return joint_path_waypoints_critical_;};
        Eigen::MatrixXd cart_path(){return cart_path_waypoints_;};
        Eigen::MatrixXd dist_profile(){return dist_profile_;};
        Eigen::MatrixXd dist_init_profile(){return dist_init_profile_;};
        Eigen::MatrixXd joint_init_ref(){return joint_init_reference_;};
        Eigen::MatrixXd cart_init_ref(){return cart_init_reference_;};
        Eigen::MatrixXd time_profile(){return time_profile_;};
        Eigen::MatrixXd velocity_cart(){return cart_vel_profile_;};
        Eigen::MatrixXd velocity_cart_init(){return cart_vel_profile_reference_;};
        Eigen::MatrixXd acceleration_cart(){return cart_acc_profile_;};
        Eigen::MatrixXd acceleration_cart_init(){return cart_acc_profile_reference_;};
        Eigen::MatrixXd velocity_joint(){return joint_vel_profile_;};
        Eigen::MatrixXd velocity_joint_init(){return joint_vel_profile_reference_;};
        Eigen::MatrixXd acceleration_joint(){return joint_acc_profile_;};
        Eigen::MatrixXd acceleration_joint_init(){return joint_acc_profile_reference_;};

        // Operations
        static bool CollisionValidStatic(Eigen::MatrixXd q, double dt,
                                        const robot::Robot::ConstPtr& robot_ptr,
                                        const query::ProcessedQuery::ConstPtr& processed_query_ptr);
};
}
}

