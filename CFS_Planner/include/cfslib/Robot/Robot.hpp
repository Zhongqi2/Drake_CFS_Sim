#pragma once
#include "cfslib/Utils/Math.hpp"
#include "cfslib/Utils/FileIO.hpp"
#include "cfslib/Query/UserQuery.hpp"

namespace cfslib
{
namespace robot
{
class Robot
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<Robot> Ptr;
        typedef std::shared_ptr<Robot const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        // Robot Properties
        int n_joint_;
        Eigen::MatrixXd q_limits_;
        Eigen::MatrixXd qd_limits_;
        Eigen::MatrixXd qdd_limits_;
        Eigen::MatrixXd q_displacement_limits_;
        Eigen::MatrixXd DH_; // size = [1 + n_joint + n_ee, 4]
        double dt_; // ? what is this?

        // link transformations 1 (base) + n_joint + n_ee
        std::vector<std::function<Eigen::Matrix4d(double q)>> link_transforms_;

        // Capsules size = 1 (base) + n_joint + 1 (ee)
        std::vector<cfslib::math::Capsule> capsules_;

        // Base frame
        Eigen::MatrixXd base_frame_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:

        void SetupLinkTransforms();

    public:
        Robot();
        ~Robot(){}

        // setter
        void set_base_frame(const Eigen::MatrixXd& base) {base_frame_ = base;}
        
        // getter
        Eigen::MatrixXd joint_limits() const {return q_limits_;};
        Eigen::MatrixXd joint_vel_limits() const {return qd_limits_;};
        Eigen::MatrixXd joint_acc_limits() const {return qdd_limits_;};
        Eigen::MatrixXd joint_displacement_limits() const {return q_displacement_limits_;};
        Eigen::MatrixXd DH() const {return DH_;};
        double          dt() const {return dt_;};
        Eigen::MatrixXd base_frame() const {return base_frame_;};
        int robot_DOF() const {return n_joint_;};
        // std::vector<math::Capsule> capsules() const {return capsules_;};
        /**
         * @brief 
         * 
         * @param q 
         * @return std::vector<math::Capsule> size = 1 (base) + n_joint + 1 (ee)
         */
        std::vector<math::Capsule> capsules_now(const Eigen::VectorXd& q) const;

        // Operations
        void Setup(const query::UserQuery::ConstPtr& user_query_ptr);

};
}
}