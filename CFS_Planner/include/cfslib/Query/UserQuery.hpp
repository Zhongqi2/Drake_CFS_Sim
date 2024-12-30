#pragma once
#include "cfslib/Utils/Math.hpp"

#define EXEC_TIME_DEFAULT 3.0

namespace cfslib
{
namespace query
{

typedef enum : int {
    CartPoint,
    CartPath,
    CartWaypoints,
    JointPoint,
    GoalTypeSize
} GoalType;

const std::vector<std::string> GoalTypeStr{
    "CartPoint", "CartPath", "CartWaypoints", "JointPoint", "Size"
};

typedef enum : int {
    TransRotX,
    TransRotY,
    TransRotZ,
    TransOnly,
    None,
    CartesianModeSize
} CartesianMode;

const std::vector<std::string> CartesianModeStr{
    "TransRotX",
    "TransRotY",
    "TransRotZ",
    "TransOnly",
    "None",
    "Size"
};

typedef struct Mode_t{
    
    GoalType goal_type;
    bool use_timestamp;
    bool enable_tempopt;
    bool dynamic_obstacle;
    CartesianMode enforce_cartesian;

    Mode_t()
    {
        goal_type = GoalType::CartPoint;
        use_timestamp = false;
        enable_tempopt = false;
        dynamic_obstacle = false;
        enforce_cartesian = CartesianMode::None;
    }

    ~Mode_t(){}

} Mode;

class UserQuery
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<UserQuery> Ptr;
        typedef std::shared_ptr<UserQuery const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        // ?
        double robot_dt_;

        // kinematics
        const Eigen::MatrixXd DH_;

        const std::string robo_cap_fname_;
        const std::string obstacles_fname_;
        const std::string config_fname_;
        const Eigen::MatrixXd base_frame_;
        const Eigen::MatrixXi collision_avoidance_links_;

        // planning
        const double execution_time_;

        // dyn limits
        const Eigen::MatrixXd joint_limits_;
        const Eigen::MatrixXd joint_vel_limits_;
        const Eigen::MatrixXd joint_acc_limits_;
        const Eigen::MatrixXd joint_displacement_limits_;
        const double cart_vel_max_;
        const double cart_acc_max_;

        // goal
        const Eigen::VectorXd joint_ref_;
        Eigen::MatrixXd cart_traj_ref_;
        const Eigen::Matrix4d ref_frame_;
        const Eigen::Vector3d ref_frame_vel_; // velocity of ref frame in base frame

        // current status
        const cfslib::math::Vector6d x_;
        const cfslib::math::VectorJd q_;

        // query type
        const Mode mode_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:


    public:

        // cart ref in world frame
        UserQuery(
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
            const std::string& config_fname = "",
            const std::string& robo_cap_fname = "",
            const std::string& obstacles_fname = "");

        // cart ref in work piece frame
        UserQuery(
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
            const std::string& config_fname = "",
            const std::string& robo_cap_fname = "",
            const std::string& obstacles_fname = "");
        ~UserQuery(){}

        // setter
        void set_robot_dt(const double& dt) {robot_dt_ = dt;};
        void set_cart_traj_ref(Eigen::MatrixXd ref) {cart_traj_ref_ = ref;};
        
        // getter
        double          robot_dt()          const {return robot_dt_;};
        Eigen::MatrixXd joint_limits()      const {return joint_limits_;};
        Eigen::MatrixXd joint_vel_limits()  const {return joint_vel_limits_;};
        Eigen::MatrixXd joint_acc_limits()  const {return joint_acc_limits_;};
        Eigen::MatrixXd joint_displacement_limits() const {return joint_displacement_limits_;};
        Eigen::MatrixXd DH()                const {return DH_;};
        Eigen::MatrixXd base_frame()        const {return base_frame_;};
        math::Vector6d  x()                 const {return x_;};
        math::VectorJd  q()                 const {return q_;};
        Eigen::MatrixXd cart_traj_ref()     const {return cart_traj_ref_;};
        Eigen::Matrix4d ref_frame()         const {return ref_frame_;};
        Eigen::Vector3d ref_frame_vel()     const {return ref_frame_vel_;};
        Eigen::VectorXd joint_ref()         const {return joint_ref_;};
        std::string     robo_cap_fname()    const {return robo_cap_fname_;};
        std::string     obstacles_fname()   const {return obstacles_fname_;};
        std::string     param_fname()       const {return config_fname_;};
        double          execution_time()    const {return execution_time_;};
        Mode            mode()              const {return mode_;};
        double          velocity_max()      const {return cart_vel_max_;};
        double          acceleration_max()  const {return cart_acc_max_;};
        Eigen::MatrixXi collision_avoidance_links() const {return collision_avoidance_links_;};
        
        //Operations
};
}
}