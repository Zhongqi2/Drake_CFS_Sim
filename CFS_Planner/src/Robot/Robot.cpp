#include "cfslib/Robot/Robot.hpp"

namespace cfslib
{
namespace robot
{

Robot::Robot()
{
    std::cout << "Robot established!" << std::endl;
}

void Robot::Setup(const query::UserQuery::ConstPtr& user_query_ptr)
{
    try
    {
        q_limits_   = user_query_ptr->joint_limits();
        qd_limits_  = user_query_ptr->joint_vel_limits();
        qdd_limits_ = user_query_ptr->joint_acc_limits();
        q_displacement_limits_ = user_query_ptr->joint_displacement_limits();
        DH_         = user_query_ptr->DH();
        base_frame_ = user_query_ptr->base_frame();
        n_joint_    = user_query_ptr->q().rows();
        capsules_   = cfslib::io::LoadCapsulesFromFile(user_query_ptr->robo_cap_fname(), 
                                                       io::CapType::Robot);
        dt_         = user_query_ptr->robot_dt();
        this->SetupLinkTransforms();
        std::cout << "Robot Setup Done!\n" << std:: endl;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void Robot::SetupLinkTransforms()
{
    try
    {
        link_transforms_.clear();
        // generate transform matrix including ee links
        for (uint i = 0; i < DH_.rows(); ++i)
        {
            link_transforms_.emplace_back(
                [this, i] (double q) {
                    Eigen::Matrix4d trans_mat;
                    trans_mat <<
                        cos(q+DH_(i, 0)), -sin(q+DH_(i, 0))*cos(DH_(i, 3)),  sin(q+DH_(i, 0))*sin(DH_(i, 3)), DH_(i, 2)*cos(q+DH_(i, 0)),
                        sin(q+DH_(i, 0)),  cos(q+DH_(i, 0))*cos(DH_(i, 3)), -cos(q+DH_(i, 0))*sin(DH_(i, 3)), DH_(i, 2)*sin(q+DH_(i, 0)),
                                    0,                  sin(DH_(i, 3)),                  cos(DH_(i, 3)),                 DH_(i, 1),
                                    0,                              0,                              0,                        1;
                    return trans_mat;
                }
            );
        }
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

std::vector<math::Capsule> Robot::capsules_now(const Eigen::VectorXd& q) const
{
    try
    {
        int n_joint{q.rows()};

        /* --------------------------- get all trans mats --------------------------- */
        std::vector<Eigen::Matrix4d> trans_mats;

        // base link
        trans_mats.emplace_back(link_transforms_.front()(0.0f));

        // joint links
        for (uint i=0; i<n_joint; ++i)
        {
            const auto& f = link_transforms_.at(i+1);
            trans_mats.emplace_back( trans_mats.back() * f( q(i) ) );
        }

        // ee links
        for (uint i=n_joint+1; i<link_transforms_.size(); ++i)
        {
            const auto& f = link_transforms_.at(i);
            trans_mats.emplace_back( trans_mats.back() * f( 0.0f ) );
        }

        assert(trans_mats.size() == DH_.rows());

        /* ------------------------------- get cap pos ------------------------------ */

        // ! remove
        Eigen::MatrixXd base(3, 2);
        base << 0, 0, 0, 0, 0, 0; // ! for normal testing
        // base << 0, 0, 0, 0, 1.035, 1.035; // ! for testing static goal
        // ! remove end

        std::vector<math::Capsule> caps_now;
        math::Capsule cap;
        Eigen::MatrixXd ph = Eigen::MatrixXd::Ones(4, 2);
        // base and joint capsules
        for (uint i=0; i<capsules_.size()-1; ++i)
        {
            cap.r = capsules_.at(i).r;
            ph.topRows(3) = capsules_.at(i).p;
            cap.p = (trans_mats.at(i) * ph).topRows(3) + base;
            caps_now.emplace_back(cap);
        }
        
        // ee capsule (last one)
        cap.r = capsules_.back().r;
        ph.topRows(3) = capsules_.back().p;
        cap.p = (trans_mats.back() * ph).topRows(3) + base;
        caps_now.emplace_back(cap);

        return caps_now;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}
}
