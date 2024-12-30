#include "cfslib/Query/ProcessedQuery.hpp"

namespace cfslib
{
namespace query
{

ProcessedQuery::ProcessedQuery()
{
    critical_ids_.clear();
    std::cout << "Processed Query established!" << std::endl;
}

void ProcessedQuery::ConfigLQR()
{
    try
    {
        if (std::isnan(cart_vel_max_) || std::isnan(cart_acc_max_))
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "invalid cart_vel_max [" << cart_vel_max_
                << "], cart_acc_max [" << cart_acc_max_ << "].";
            throw std::runtime_error(ss.str());
        }
        lqr_bxdot_   = Eigen::MatrixXd::Ones(n_joint_, 1) * cart_vel_max_;
        lqr_bu_      = Eigen::MatrixXd::Ones(n_joint_, 1) * cart_acc_max_;
        lqr_Q_       = math::ToEigen(Qdiag_).asDiagonal();
        lqr_S_       = math::ToEigen(Sdiag_).asDiagonal();
        lqr_R_       = math::ToEigen(Rdiag_).asDiagonal();
        lqr_horizon_ = horizon_;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void ProcessedQuery::Print()
{
    std::ostringstream ss;

    ss << "---- User query processed ---\n";
    ss << ">> Mode - Goal type:         [" << GoalTypeStr.at(mode_.goal_type) << "]\n"
       << "          Use timestamp      [" << mode_.use_timestamp << "]\n"
       << "          Enable tempopt     [" << mode_.enable_tempopt << "]\n"
       << "          Dynamic obstacle   [" << mode_.dynamic_obstacle << "]\n"
       << "          Enforce cartesian  [" << CartesianModeStr.at(mode_.enforce_cartesian) << "]\n";
    ss << ">> x0: [" << x0_.transpose() << "]\n";
    ss << ">> q0: [" << q0_.transpose() << "]\n";
    // ss << ">> Processed cart traj ref: \n" << cart_traj_ref_.transpose() << "\n";
    // ss << ">> Processed joint traj ref: \n" << joint_traj_ref_.transpose() << "\n";
    ss << ">> horizon:          [" << horizon_ << "]\n";
    ss << ">> execution time:   [" << execution_time_ << "]\n";
    ss << ">> cart vel max:     [" << cart_vel_max_ << "]\n";
    ss << ">> cart acc max:     [" << cart_acc_max_ << "]\n";
    // ss << ">> n_joint:          [" << n_joint_ << "]\n";
    // ss << ">> DH: \n" << DH_ << "\n";
    // ss << ">> trajectory frequency:    [" << traj_hz_ << "]\n";
    ss << ">> safety margin:           [" << safety_margin_ << "]\n";
    // ss << ">> weight ref:              [" << io::vtos(weight_ref_) << "]\n";
    // ss << ">> weight self:             [" << io::vtos(weight_self_) << "]\n";
    // ss << ">> CFS max iteration:       [" << cfs_max_iter_ << "]\n";
    // ss << ">> CFS convergence thres:   [" << cfs_convergence_thres_ << "]\n";
    // ss << ">> CFS resample n insert:   [" << cfs_resample_n_insert_ << "]\n";
    // ss << ">> CFS max resample:        [" << cfs_max_resample_ << "]\n";
    // ss << ">> CFS eq max iteration:    [" << cfs_eq_max_iter_ << "]\n";
    // ss << ">> CFS eq thres:            [" << cfs_eq_thres_ << "]\n";
    // ss << ">> ICOP threshold:          [" << icop_thresh_ << "]\n";
    // ss << ">> ICOP max iteration:      [" << icop_maxiter_ << "]\n";
    if (mode_.goal_type == GoalType::CartPoint)
    {
        ss << ">> LQR Q slack: [" << lqr_Qslack_ << "]\n";
        ss << ">> LQR Q: \n" << lqr_Q_ << "\n";
        ss << ">> LQR S: \n" << lqr_S_ << "\n";
        ss << ">> LQR R: \n" << lqr_R_ << "\n";
        ss << ">> LQR acc/action bound:    [" << lqr_bu_.transpose() << "]\n";
        ss << ">> LQR velocity bound:      [" << lqr_bxdot_.transpose() << "]\n";
        ss << ">> LQR horizon:             [" << lqr_horizon_ << "]\n";
    }

    std::cout << ss.str() << "\n";
}

void ProcessedQuery::ProcessGoal(const query::UserQuery::ConstPtr& user_query_ptr, const double& traj_hz)
{
    try
    {
        std::cout << "Processing goal...\n";

        /* ------------- compute horizon, traj freq, and execution time ------------- */
        if (mode_.use_timestamp)
        {
            execution_time_ = user_query_ptr->execution_time();
        }
        else
        {
            execution_time_ = EXEC_TIME_DEFAULT;
        }
        if(traj_hz > 0){
            traj_hz_ = traj_hz;
        }

        if (mode_.goal_type == GoalType::CartPoint || mode_.goal_type == GoalType::JointPoint)
        {
            // given traj freq, compute horizon
            horizon_ = (int) ceil(execution_time_ * traj_hz_);
            traj_hz_ = horizon_ / execution_time_;
        }
        else if (mode_.goal_type == GoalType::CartPath)
        {
            // given horizon, compute traj freq
            horizon_ = cart_traj_ref_.cols()-1;
            traj_hz_ = horizon_ / execution_time_;
        }
        else if (mode_.goal_type == GoalType::CartWaypoints)
        {
            traj_hz_ = 250.0;
            horizon_ = (int) ceil(execution_time_ * traj_hz_);
            traj_hz_ = horizon_ / execution_time_;
        }
        else
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "unknown goal type: " << mode_.goal_type;
            throw std::runtime_error(ss.str());
        }

        if (horizon_ <= 0)
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "Horizon: " << horizon_ << ", check input.";
            throw std::runtime_error(ss.str());
        }

        /* ----------------------- convert to joint trajectory ---------------------- */
        if (mode_.goal_type == GoalType::CartPath ||
            mode_.goal_type == GoalType::CartPoint ||
            mode_.goal_type == GoalType::CartWaypoints)
        {
            /* ------------------------ transform into base frame ----------------------- */
            double t{0.0};
            Eigen::Matrix4d f( ref_frame_ );
            Eigen::Matrix4d ref_mat;
            math::Vector6d p;
            if(mode_.goal_type == GoalType::CartPath){
                for (int i=0; i<cart_traj_ref_.cols(); ++i)
                {
                    t = i * ( 1.0f / traj_hz_ );
                    // only translation
                    f.block(0, 3, 3, 1) = ref_frame_.block(0, 3, 3, 1) + ref_frame_vel_ * t;
                    math::PoseAngRadToTransMat( cart_traj_ref_.col(i), ref_mat );
                    math::TransMatToPoseAngRad( f * ref_mat,  p );
                    cart_traj_ref_.col(i) = p;
                }
            }
            else if (mode_.goal_type == GoalType::CartWaypoints)
            {
                for (int i=0; i<cart_traj_ref_.cols(); ++i)
                {
                    f.block(0, 3, 3, 1) = ref_frame_.block(0, 3, 3, 1);
                    math::PoseAngRadToTransMat( cart_traj_ref_.col(i), ref_mat );
                    math::TransMatToPoseAngRad( f * ref_mat,  p );
                    cart_traj_ref_.col(i) = p;
                }
            }
            else{
                t = execution_time_;
                f.block(0, 3, 3, 1) = ref_frame_.block(0, 3, 3, 1) + ref_frame_vel_ * t;
                math::PoseAngRadToTransMat( cart_traj_ref_.col(0), ref_mat );
                math::TransMatToPoseAngRad( f * ref_mat,  p );
                cart_traj_ref_.col(0) = p;
            }
        }

        /* ---------------------- if goal reaching, interpolate --------------------- */
        if (mode_.goal_type == GoalType::CartPoint)
        {
            this->ConfigLQR();
            std::unique_ptr<math::CartesianLQR2ndOrder> lqr_ptr =
                std::make_unique<math::CartesianLQR2ndOrder>(
                    lqr_bu_, lqr_bxdot_, lqr_Q_, lqr_S_, lqr_Qslack_, lqr_R_,
                    1.0/traj_hz_, lqr_horizon_);

            math::Vector6d x = math::GetTransInBase(x0_, cart_traj_ref_.col(0));
            Eigen::MatrixXd xacc_arr, xx; // xacc_arr: length N, xx: length N+1
            std::tie(xacc_arr, xx) = lqr_ptr->Solve(
                math::EigenVcat(x, Eigen::MatrixXd::Zero(CARTESIAN_DIMS, 1)));
            
            Eigen::MatrixXd x_arr;
            x_arr = xx.block(0, 0, CARTESIAN_DIMS, xx.cols());
            math::BatchApplyTrans(x_arr, cart_traj_ref_.col(0));

            cart_traj_ref_ = x_arr;

            // todo return failure if qp cannot solve
        }

        // cart_traj_ref is a path with consistent horizon, dt, exec time

        /* ----------------------- convert to joint trajectory ---------------------- */
        std::cout << "Retrieving joint trajectory reference...\n";
        if (mode_.goal_type == GoalType::CartPath ||
            mode_.goal_type == GoalType::CartPoint)
        {
            /* --------------------------------- ICOP IK -------------------------------- */
            joint_traj_ref_ = math::icop_cartesian2joint(
                cart_traj_ref_, x0_, q0_, DH_, icop_thresh_, icop_maxiter_);
        }
        else if (mode_.goal_type == GoalType::CartWaypoints)
        {
            
            // call IK to convert cart_traj_ref_ to joint_traj_ref_
            joint_traj_ref_.resize(n_joint_, cart_traj_ref_.cols());
            joint_traj_ref_.col(0) = q0_;
            for (int i=1; i<cart_traj_ref_.cols(); ++i)
            {
                joint_traj_ref_.col(i) = math::IK(
                    cart_traj_ref_.col(i), cart_traj_ref_.col(i-1), joint_traj_ref_.col(i-1),
                    DH_, 1e-2, 1e-2, 100);
            }

            std::cout << "*******************************************************\n";
            std::cout << ">>> goal_type: " << GoalTypeStr.at(mode_.goal_type) << "\n";
            std::cout << ">>> current joint: " << math::print_eigen_mat(q0_.transpose()) << "\n";
            std::cout << ">>> cart_traj_ref_:\n" << math::print_eigen_mat(cart_traj_ref_.transpose()) << "\n";
            std::cout << ">>> joint_traj_ref_:\n" << math::print_eigen_mat(joint_traj_ref_.transpose()) << "\n";
            std::cout << "*******************************************************\n";

            // interploate joint trajectory and distribute steps proportional to step sizes in Cartesian space
            std::vector<double> cart_dists;
            double total_dist{0.0};
            for (int i=0; i<cart_traj_ref_.cols()-1; ++i)
            {
                cart_dists.push_back((cart_traj_ref_.col(i+1) - cart_traj_ref_.col(i)).head(3).norm());
                total_dist += cart_dists.back();
            }

            std::vector<int> steps;
            for (int i=0; i<cart_dists.size()-1; ++i)
            {
                steps.push_back(static_cast<int>(horizon_ * cart_dists[i] / total_dist));
            }
            steps.push_back(horizon_ - std::accumulate(steps.begin(), steps.end(), 0));
            
            Eigen::MatrixXd joint_traj_ref_dense;
            joint_traj_ref_dense.resize(n_joint_, horizon_+1);
            joint_traj_ref_dense.col(0) = joint_traj_ref_.col(0);

            critical_ids_.clear();
            int idx{0};
            for (int i=0; i<joint_traj_ref_.cols()-1; ++i)
            {
                critical_ids_.push_back(idx);

                Eigen::MatrixXd q0 = joint_traj_ref_.col(i);
                Eigen::MatrixXd q1 = joint_traj_ref_.col(i+1);
                Eigen::MatrixXd dq = (q1 - q0) / steps[i];

                for (int j=0; j<steps[i]; ++j)
                {
                    joint_traj_ref_dense.col(idx++) = q0 + dq * j;
                }
            }
            critical_ids_.push_back(idx);
            joint_traj_ref_dense.col(idx) = joint_traj_ref_.col(joint_traj_ref_.cols()-1);

            assert(idx == horizon_);

            joint_traj_ref_ = joint_traj_ref_dense;
        }
        else
        {
            joint_traj_ref_ = math::LinInterp(q0_, user_query_ptr->joint_ref(), horizon_);
        }
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void ProcessedQuery::LoadConfig(std::string fname)
{
    try
    {
        std::cout << "Loading config...\n";
        // Default values
        if(fname.compare("") == 0){
            std::cout << "Using default parameters!" << std::endl;
        }
        else{
            
            io::ConfigMap dict = io::ParseConfigFile(fname);

            io::ParseVariable("trajectory_frequency",   traj_hz_,       dict);
            io::ParseVariable("safety_margin",          safety_margin_, dict);
            io::ParseVariable("weight_ref",             weight_ref_,    dict);
            io::ParseVariable("weight_self",            weight_self_,   dict);
            io::ParseVariable("cfs_max_iter",           cfs_max_iter_,          dict);
            io::ParseVariable("cfs_convergence_thres",  cfs_convergence_thres_, dict);
            io::ParseVariable("cfs_resample_hz_insert",  cfs_resample_n_insert_, dict);
            io::ParseVariable("cfs_max_resample",       cfs_max_resample_,      dict);
            io::ParseVariable("cfs_eq_max_iter",        cfs_eq_max_iter_,       dict);
            io::ParseVariable("cfs_eq_thres",           cfs_eq_thres_,          dict);
            io::ParseVariable("icop_thresh",            icop_thresh_,   dict);
            io::ParseVariable("icop_max_iter",          icop_maxiter_,  dict);
            io::ParseVariable("Qslack",                 lqr_Qslack_,    dict);
            io::ParseVariable("Qdiag",                  Qdiag_,         dict);
            io::ParseVariable("Sdiag",                  Sdiag_,         dict);
            io::ParseVariable("Rdiag",                  Rdiag_,         dict);
        }
    }
    catch(const std::exception& e)
    {
        throw e;
    }
}

void ProcessedQuery::Process(const query::UserQuery::ConstPtr& user_query_ptr, const double& traj_hz)
{
    try
    {   
        // task
        mode_           = user_query_ptr->mode();
        x0_             = user_query_ptr->x();
        q0_             = user_query_ptr->q();
        cart_traj_ref_  = user_query_ptr->cart_traj_ref();
        ref_frame_      = user_query_ptr->ref_frame();
        ref_frame_vel_  = user_query_ptr->ref_frame_vel();
        cart_vel_max_   = user_query_ptr->velocity_max();
        cart_acc_max_   = user_query_ptr->acceleration_max();
        collision_avoidance_links_ = user_query_ptr->collision_avoidance_links();

        obstacle_capsules_ = cfslib::io::LoadCapsulesFromFile(user_query_ptr->obstacles_fname(), io::CapType::Obstacle);
        obstacle_bounding_boxes_ = cfslib::io::LoadBoundingBoxFromFile(user_query_ptr->obstacles_fname());

        if (!mode_.dynamic_obstacle)
        {
            for (auto& cap : obstacle_capsules_)
            {
                cap.vel.setZero();
            }
            for (auto& box : obstacle_bounding_boxes_)
            {
                box.vel.setZero();
            }
        }

        // robot
        n_joint_   = user_query_ptr->joint_limits().rows();
        DH_        = user_query_ptr->DH();
        
        // hyper params
        this->LoadConfig(user_query_ptr->param_fname());
        
        // handle different goal modes
        this->ProcessGoal(user_query_ptr, traj_hz);

        // print all variables
        this->Print();

        std::cout<<"Process Query Done!\n"<<std::endl;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}
}

