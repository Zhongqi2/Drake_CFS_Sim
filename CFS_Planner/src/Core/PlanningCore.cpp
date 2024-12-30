#include "cfslib/Core/PlanningCore.hpp"

namespace cfslib
{

namespace core
{

PlanningCore::PlanningCore()
{   

}

void PlanningCore::CoreParse(const robot::Robot::ConstPtr& robot_ptr,
                    const query::ProcessedQuery::ConstPtr& processed_query_ptr,
                    Eigen::MatrixXd& xref)
{
    try
    {
        horizon_           = processed_query_ptr->horizon();
        n_pt_              = horizon_ + 1;
        exec_time_         = processed_query_ptr->execution_time();
        dt_tempopt_        = robot_ptr->dt();
        safety_margin_     = processed_query_ptr->safety_margin();
        weight_ref_        = processed_query_ptr->weight_ref();
        weight_self_       = processed_query_ptr->weight_self();
        max_iter_          = processed_query_ptr->cfs_max_iter();
        convergence_thres_ = processed_query_ptr->cfs_convergence_thres();
        eq_max_iter_       = processed_query_ptr->cfs_eq_max_iter();
        eq_thres_          = processed_query_ptr->cfs_eq_thres();
        enforce_eq_        = (processed_query_ptr->mode().enforce_cartesian < query::CartesianMode::None);
        enable_tempopt_    = (!processed_query_ptr->mode().dynamic_obstacle) &&
                            (processed_query_ptr->mode().enable_tempopt);
        xdim_              = robot_ptr->robot_DOF();
        xref               = processed_query_ptr->joint_traj_ref();

        // ! tmp
        dynamic_obstacle_ = processed_query_ptr->mode().dynamic_obstacle;

        std::cout << ">> jvel limit " << robot_ptr->joint_vel_limits().transpose() << "\n";
        std::cout << ">> traj hz " << processed_query_ptr->trajectory_frequency() << "\n";
        
        /* -------------------------------- dist func ------------------------------- */
        /**
         * @brief cap/box pos is at pos_init + t*vel
         * 
         */
        // this->DistCapFn = [robot_ptr, processed_query_ptr] (const Eigen::VectorXd& x, const double& t,
        //                                                     const math::Capsule& cap, const math::Capsule& obs_cap)
        // {
        //     const auto& obstacle_capsules = processed_query_ptr->obstacles_cap();
        //     const auto& obstacle_bounding_boxes = processed_query_ptr->obstacles_box();

        //     auto robot_cap_now = robot_ptr->capsules_now(x); // get cap at joint state x

        //     // calculate the closest distance
        //     double min_d = std::numeric_limits<double>::infinity();
        //     double dis;
        //     for (int i=0; i<robot_cap_now.size(); ++i){
        //         for(int j=0; j<obstacle_capsules.size(); ++j){
        //             dis = math::DistCap2Cap(robot_cap_now[i], obstacle_capsules[j].at(t));
        //             if (dis < min_d){
        //                 min_d = dis;
        //             }
        //         }
        //         for(int j=0; j<obstacle_bounding_boxes.size(); ++j){
        //             dis = math::DistCap2BB(robot_cap_now[i], obstacle_bounding_boxes[j].at(t));
        //             if (dis < min_d){
        //                 min_d = dis;
        //             }
        //         }
        //     }
         
        //     // Robot self-collision
        //     // for (int i=0; i<robot_cap_now.size(); ++i){
        //     //     for(int j=i+2; j<robot_cap_now.size(); ++j){
        //     //         dis = math::DistCap2Cap(robot_cap_now[i], robot_cap_now[j]);
        //     //         if (dis < min_d){
        //     //             min_d = dis;
        //     //         }
        //     //     }
        //     // }
            
        //     return min_d;
        // };

        /* ------------------------------ central diff ------------------------------ */
        this->CentralDiffCapFn = [this, robot_ptr, processed_query_ptr] (const Eigen::VectorXd& x, const double& t, 
                                                                         const int robot_link_idx, const int obs_idx)
        {
                        Eigen::MatrixXd grad;
            grad.resize(1, x.rows());
            double eps = 1e-5;
            const auto& obstacle_capsules = processed_query_ptr->obstacles_cap();

            Eigen::MatrixXd x_tmp;
            std::vector<cfslib::math::Capsule> robot_cap_now;
            double dist_h, dist_l;
            for (int i=0; i<x.rows(); ++i){
                x_tmp = x;
                x_tmp(i,0) = x(i,0) + eps/2;
                robot_cap_now = robot_ptr->capsules_now(x_tmp); // get cap at joint state x
                dist_h = math::DistCap2Cap(robot_cap_now[robot_link_idx], obstacle_capsules[obs_idx].at(t));

                x_tmp(i,0) = x(i,0) - eps/2;
                robot_cap_now = robot_ptr->capsules_now(x_tmp); // get cap at joint state x
                dist_l = math::DistCap2Cap(robot_cap_now[robot_link_idx], obstacle_capsules[obs_idx].at(t));

                grad(0,i) = (dist_h - dist_l) / eps;
            }
            return grad;
        };

        this->CentralDiffBBFn = [this, robot_ptr, processed_query_ptr] (const Eigen::VectorXd& x, const double& t, 
                                                                         const int& robot_link_idx, const int& obs_idx)
        {
            Eigen::MatrixXd grad;
            grad.resize(1, x.rows());
            double eps = 1e-5;
            const auto& obstacle_bounding_boxes = processed_query_ptr->obstacles_box();

            Eigen::MatrixXd x_tmp;
            std::vector<cfslib::math::Capsule> robot_cap_now;
            double dist_h, dist_l;
            for (int i=0; i<x.rows(); ++i){
                x_tmp = x;
                x_tmp(i,0) = x(i,0) + eps/2;
                robot_cap_now = robot_ptr->capsules_now(x_tmp);
                dist_h = math::DistCap2BB(robot_cap_now[robot_link_idx], obstacle_bounding_boxes[obs_idx].at(t));

                x_tmp(i,0) = x(i,0) - eps/2;
                robot_cap_now = robot_ptr->capsules_now(x_tmp);
                dist_l = math::DistCap2BB(robot_cap_now[robot_link_idx], obstacle_bounding_boxes[obs_idx].at(t));

                grad(0,i) = (dist_h - dist_l) / eps;
            }
            return grad;
        };

        /* ------------------------------ LinConstraint ----------------------------- */
        this->LinConstraintFn = [this, robot_ptr, processed_query_ptr] (Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final, const Eigen::VectorXd& xx)
        {
            // create new variable space 
            Eigen::MatrixXd x;
            x = xx.block(1*xdim_, 0, xdim_, 1);

            std::string ttttt;

            const auto& obstacle_capsules = processed_query_ptr->obstacles_cap();
            const auto& obstacle_bounding_boxes = processed_query_ptr->obstacles_box();
            int ncap = obstacle_capsules.size();
            int nbb = obstacle_bounding_boxes.size();
            int nobs = ncap + nbb;
            int nlink = robot_ptr->capsules_now(x).size(); // get cap at joint state x
            Eigen::MatrixXi consider_links = processed_query_ptr->collision_avoidance_links();
            int tmp = 0;
            for(int i=0; i<consider_links.rows(); i++){
                if(consider_links(i, 0) == 1){
                    tmp ++;
                }
            }
            int nconstraints = tmp * nobs;

            // std::cout << ">> consider_links: \n" << consider_links << "\n";
            // std::cout << ">> nlink: " << nlink << "\n";
            // std::cout << ">> ncap: " << ncap << "\n";
            // std::cout << ">> nbb: " << nbb << "\n";

            // std::cin >> ttttt;
            
            Eigen::MatrixXd Lfull, S;
            Lfull.resize(n_pt_ * nconstraints, n_pt_ * xdim_); // no need for the start and end 
            S.resize(n_pt_ * nconstraints, 1); // no need for the start and end
            S = Eigen::MatrixXd::Zero(n_pt_ * nconstraints, 1);
            
            // start constraint specification
            Eigen::MatrixXd grad, s, l, d, Diff;
            double distance;
            double dt{exec_time_/horizon_}, t;
            for (int i=1; i<n_pt_-1; ++i)
            {
                x = xx.block(i*xdim_, 0, xdim_, 1);
                t = dt * i;
                auto robot_cap_now = robot_ptr->capsules_now(x);
      
                // std::cout << ">> lin, robot pos: " << math::print_eigen_mat(x.transpose()) << "\n";

                int offset = 0;
                for (int ri=0; ri<nlink; ++ri){
                    if(consider_links(ri, 0) == 0){
                        
                        // std::cout << ">> skip link: " << ri << "\n";

                        continue;
                    }
                    for (int j=0; j<ncap; ++j){

                        distance = math::DistCap2Cap(robot_cap_now[ri], obstacle_capsules[j].at(t));
                        d.resize(1,1);
                        d(0,0) = distance - safety_margin_;
                        
                        // numerical gradient
                        grad.resize(1, x.rows());
                        // grad = helper::central_diff(x, DH_use, base_, obs_static, cap);
                        grad = this->CentralDiffCapFn(x, t, ri, j);

                        Diff.resize(1, n_pt_*xdim_); // construct gradient of xref in terms of every time step
                        Diff = Eigen::MatrixXd::Zero(1, n_pt_*xdim_); // the rest is zero
                        Diff.block(0, i*xdim_, 1, xdim_) = grad;
                        s = d - Diff*xx;
                        l = -1*Diff;

                        // concatnate s, l to construct S, Lfull
                        assert(l.cols() == n_pt_*xdim_);
                        assert(s.cols() == 1 && s.rows() == 1);
                        S(i * nconstraints + offset, 0) = s(0, 0);
                        Lfull.block((i)*nconstraints + offset, 0, 1, l.cols()) = l;
                        offset += 1;
                    }
                    for(int j=0; j<nbb; ++j){
                        distance = math::DistCap2BB(robot_cap_now[ri], obstacle_bounding_boxes[j].at(t));

                        // std::cout << ">> robot cap id: " << ri << "\n";
                        // std::cout << ">> obs id: " << j << "\n";
                        // std::cout << ">> robot ee cap: \n" << math::print_eigen_mat(robot_cap_now[ri].p.transpose()) << "\n";
                        // std::cout << ">> bbox p1_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p1_1) << "\n";
                        // std::cout << ">> bbox p1_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p1_2) << "\n";
                        // std::cout << ">> bbox p2_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p2_1) << "\n";
                        // std::cout << ">> bbox p2_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p2_2) << "\n";
                        // std::cout << ">> bbox p3_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p3_1) << "\n";
                        // std::cout << ">> bbox p3_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p3_2) << "\n";
                        // std::cout << ">> distance: " << distance << "\n";

                        // std::cin >> ttttt;

                        d.resize(1,1);
                        d(0,0) = distance - safety_margin_;
                        
                        // numerical gradient
                        grad.resize(1, x.rows());
                        // grad = helper::central_diff(x, DH_use, base_, obs_static, cap);
                        grad = this->CentralDiffBBFn(x, t, ri, j);

                        Diff.resize(1, n_pt_*xdim_); // construct gradient of xref in terms of every time step
                        Diff = Eigen::MatrixXd::Zero(1, n_pt_*xdim_); // the rest is zero
                        Diff.block(0, i*xdim_, 1, xdim_) = grad;

                        // reset linear constriants 
                        s = d - Diff*xx;
                        l = -1*Diff;

                        // concatnate s, l to construct S, Lfull
                        assert(l.cols() == n_pt_*xdim_);
                        assert(s.cols() == 1 && s.rows() == 1);
                        S(i * nconstraints + offset, 0) = s(0, 0);
                        Lfull.block((i)*nconstraints + offset, 0, 1, l.cols()) = l;
                        offset += 1;
                    }
                }
            }
            // get rid of the first and end point constraints 
            Lfull_final = Lfull.block(nconstraints ,0,(n_pt_-2) * nconstraints,Lfull.cols());
            S_final = S.block(nconstraints , 0, (n_pt_-2) * nconstraints , S.cols());

            /* --------------------- append joint limit constraints --------------------- */
            const Eigen::MatrixXd& ub = robot_ptr->joint_limits().col(1);
            const Eigen::MatrixXd& lb = robot_ptr->joint_limits().col(0);

            Lfull_final = math::EigenVcat(Lfull_final, Eigen::MatrixXd::Identity(n_pt_*xdim_, n_pt_*xdim_));
            for (uint i=0; i<n_pt_; ++i)
            {
                S_final = math::EigenVcat(S_final, ub);
            }

            Lfull_final = math::EigenVcat(Lfull_final, -Eigen::MatrixXd::Identity(n_pt_*xdim_, n_pt_*xdim_));
            for (uint i=0; i<n_pt_; ++i)
            {
                S_final = math::EigenVcat(S_final, -lb);
            }
        };

        /* ----------------------------- SetCostMatrix3d ---------------------------- */
        this->SetCostMatrix3dFn = [this] (Eigen::MatrixXd& H, Eigen::MatrixXd& f, const Eigen::VectorXd& xx)
        {
            Eigen::MatrixXd qd;
            qd.resize(xdim_,xdim_);
            qd <<   1, 0, 0, 0, 0, 0,
                    0, 0.01, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 4, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 4;
            // assign Q1
            Eigen::MatrixXd Q1 = Eigen::MatrixXd::Zero(n_pt_*xdim_,n_pt_*xdim_);
            for (int i=0; i < n_pt_; i++)
            {
                if(i == n_pt_ - 1)
                {
                    Q1.block(i*xdim_, i*xdim_, xdim_, xdim_) = qd;
                }
                else
                {
                    Q1.block(i*xdim_, i*xdim_, xdim_, xdim_) = qd*0.1;
                }
            }

            // assign Q2
            // n_pt_ = 4;
            Eigen::MatrixXd diag(n_pt_*xdim_,n_pt_*xdim_);
            Eigen::MatrixXd diag11, diag12, diag21, diag22;
            diag11 = Eigen::MatrixXd::Zero((n_pt_-1)*xdim_, xdim_);
            diag12 = Eigen::MatrixXd::Identity((n_pt_-1)*xdim_, (n_pt_-1)*xdim_);
            diag21 = Eigen::MatrixXd::Zero(xdim_, xdim_);
            diag22 = Eigen::MatrixXd::Zero(xdim_, (n_pt_-1)*xdim_);
            diag << diag11, diag12,
                    diag21, diag22;
            
            Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(n_pt_*xdim_,n_pt_*xdim_);
            Eigen::MatrixXd Vdiff(n_pt_*xdim_,n_pt_*xdim_);
            Vdiff = Id - diag;
            Eigen::MatrixXd Q2 = Vdiff.block(0, 0, (n_pt_-1)*xdim_, n_pt_*xdim_).transpose() * Vdiff.block(0, 0, (n_pt_-1)*xdim_, n_pt_*xdim_);

            // assign Q3
            Eigen::MatrixXd diag2(n_pt_*xdim_,n_pt_*xdim_);
            Eigen::MatrixXd diag211, diag212, diag221, diag222;
            diag211 = Eigen::MatrixXd::Zero((n_pt_-2)*xdim_, xdim_*2);
            diag212 = Eigen::MatrixXd::Identity((n_pt_-2)*xdim_, (n_pt_-2)*xdim_);
            diag221 = Eigen::MatrixXd::Zero(xdim_*2, xdim_*2);
            diag222 = Eigen::MatrixXd::Zero(xdim_*2, (n_pt_-2)*xdim_);
            diag2 << diag211, diag212,
                    diag221, diag222;
            Eigen::MatrixXd Adiff = Vdiff - diag + diag2;
            Eigen::MatrixXd Q3 = Adiff.block(0, 0, (n_pt_-2)*xdim_, n_pt_*xdim_).transpose()  * Adiff.block(0, 0, (n_pt_-2)*xdim_, n_pt_*xdim_);
            
            // Add new Q_diff to minimize squared difference between consecutive vectors
            Eigen::MatrixXd Q_diff = Eigen::MatrixXd::Zero(n_pt_ * xdim_, n_pt_ * xdim_);
            for (int i = 0; i<n_pt_; ++i)
            {
                if (i == 0 || i == n_pt_-1)
                {
                    Q_diff.block(i*xdim_, i*xdim_, xdim_, xdim_) = Eigen::MatrixXd::Identity(xdim_, xdim_);
                }
                else
                {
                    Q_diff.block(i*xdim_, i*xdim_, xdim_, xdim_) = 2 * Eigen::MatrixXd::Identity(xdim_, xdim_);
                }
            }
            for (int i = 0; i<n_pt_-1; ++i)
            {
                int j = i+1;
                Q_diff.block(i*xdim_, j*xdim_, xdim_, xdim_) = -1 * Eigen::MatrixXd::Identity(xdim_, xdim_);
                Q_diff.block(j*xdim_, i*xdim_, xdim_, xdim_) = -1 * Eigen::MatrixXd::Identity(xdim_, xdim_);
            }
            Q_diff *= 1.0; // Scaling factor to adjust influence

            // set Qref and Qself
            Eigen::MatrixXd Qref = weight_ref_[0] * Q1 + weight_ref_[1] * Q2 +  weight_ref_[2] * Q3;
            Eigen::MatrixXd Qself = weight_self_[0] * Q1 + weight_self_[1] * Q2 +  weight_self_[2] * Q3;

            H = Qref + Qself + Q_diff;
            f = -1 * Qref * xx;
        };

        /* ------------------------------ EqConstraint ------------------------------ */
        this->EqConstraintFn = [this, robot_ptr, processed_query_ptr] (Eigen::MatrixXd& Aeq, Eigen::MatrixXd& beq, const Eigen::VectorXd& xx)
        {
            try
            {
                Eigen::MatrixXd DH = robot_ptr->DH();

                // !debug
                // DH.resize(7, 4);
                // DH << 0, 0, 0, 0,
                //         0,    0.1273,         0,    1.5708,
                //         0,         0,   -0.6120,         0,
                //         0,         0,   -0.5723,         0,
                //         0,    0.1639,         0,    1.5708,
                //         0,    0.1157,         0,   -1.5708,
                //         0,    0.0922,         0,         0;

                Eigen::MatrixXd cart_traj_ref = processed_query_ptr->cart_traj_ref();

                // !debug
                // cart_traj_ref.resize(6, 5);
                // cart_traj_ref << 0.7755, 0.5327, 0.2721, 0.0298, -0.1639,
                //                 -0.7661, -0.8762, -0.8951, -0.8264, -0.6880,
                //                 1.0885, 1.2422, 1.3716, 1.4729, 1.5421,
                //                 0, 0, 0, 0, 0, 
                //                 0, 0, 0, 0, 0, 
                //                 0, 0, 0, 0, 0;
                
                if (enforce_eq_) // intermediate Cartesian constraints
                {
                    // !debug
                    // Eigen::MatrixXd cart_traj = math::BatchFKine(math::ColWiseUnflatten(xx, robot_ptr->robot_DOF()), DH);
                    // std::cout << "current cart traj:\n" << cart_traj << "\n";
                    // std::cout << "desired cart traj:\n" << cart_traj_ref << "\n";
                    // while (true);

                    // handle enforce cartesian
                    const query::CartesianMode cart_mode = processed_query_ptr->mode().enforce_cartesian;
                    int n_eq{0};
                    if (cart_mode == query::CartesianMode::TransOnly)
                    {
                        n_eq = TRANS_DIMS;
                    }
                    else if (cart_mode >= query::CartesianMode::TransRotX && cart_mode <= query::CartesianMode::TransRotZ)
                    {
                        n_eq = TRANS_DIMS + AXIS_DIMS;
                    }
                    else
                    {
                        std::ostringstream ss;
                        ss << ERR_HEADER << "Unknown cartesian mode [" << cart_mode << "]";
                        throw std::runtime_error(ss.str());
                    }

                    // first and last unchanged
                    Aeq.resize( ( n_pt_ - 2 ) * n_eq + 2 * xdim_, n_pt_ * xdim_);
                    Aeq.setZero();
                    beq.resize( ( n_pt_ - 2 ) * n_eq + 2 * xdim_, 1);
                    beq.setZero();

                    Aeq.topLeftCorner(xdim_, xdim_) = Eigen::MatrixXd::Identity(xdim_, xdim_);
                    Aeq.bottomRightCorner(xdim_, xdim_) = Eigen::MatrixXd::Identity(xdim_, xdim_);
                    beq.block(0, 0, xdim_, 1) = xx.head(xdim_);
                    beq.block( ( n_pt_ - 2 ) * n_eq + xdim_, 0, xdim_, 1) = xx.tail(xdim_);

                    // intermediate points
                    Eigen::MatrixXd J; // ee jacobian
                    Eigen::VectorXd x; // joint state
                    Eigen::Matrix4d pose_mat, pose_mat_ref; // transformation matrices
                    Eigen::Vector3d axis, axis_ref; // axis to be tracked, can be x/y/z unit vec
                    Eigen::MatrixXd joint_axes; // joint rotation axes
                    Eigen::MatrixXd Jaxis; // axis jacobian (d axis / d q)
                    math::Vector6d cart, cart_ref; // current cart, cart ref
                    for (uint i=1; i<n_pt_-1; ++i)
                    {
                        // current cart
                        x = xx.block(i*xdim_, 0, xdim_, 1);
                        pose_mat = math::FKine(x, DH);
                        math::TransMatToPoseAngRad( pose_mat, cart );

                        // cart ref
                        cart_ref = cart_traj_ref.col(i);
                        math::PoseAngRadToTransMat(cart_ref, pose_mat_ref );

                        // !debug
                        // cart(2) += 1.035;

                        J = math::Jacobi(x, cart, DH).topRows(TRANS_DIMS);

                        /* ------------------------- translation constriant ------------------------- */
                        Aeq.block( xdim_ + ( i - 1 ) * n_eq, xdim_ + ( i - 1 ) * xdim_, TRANS_DIMS, xdim_ ) = J;
                        beq.block( xdim_ + ( i - 1 ) * n_eq, 0, TRANS_DIMS, 1) = (cart_ref - cart).head(TRANS_DIMS) + J * x;

                        /* --------------------------- rotation constraint -------------------------- */
                        if (cart_mode >= query::CartesianMode::TransRotX && cart_mode <= query::CartesianMode::TransRotZ)
                        {
                            int rot_i = (int) cart_mode;
                            axis        = pose_mat.block<3, 1>(0, rot_i);
                            axis_ref    = pose_mat_ref.block<3, 1>(0, rot_i);
                            joint_axes  = math::JointAxes(x, DH);
                            Jaxis       = math::BatchCross(joint_axes, axis);

                            Aeq.block( xdim_ + ( i - 1 ) * n_eq + TRANS_DIMS, xdim_ + ( i - 1 ) * xdim_, AXIS_DIMS, xdim_ ) = Jaxis;
                            beq.block( xdim_ + ( i - 1 ) * n_eq + TRANS_DIMS, 0, AXIS_DIMS, 1) = axis_ref - axis + Jaxis * x;
                        }

                        // !debug
                        // std::cout << "cart_ref:\n" << cart_ref << "\n";
                        // std::cout << "cart:\n" << cart << "\n";
                        // while (true);
                    }

                    // !debug
                    // std::cout << "Aeq:\n" << Aeq << "\n";
                    // std::cout << "beq:\n" << beq << "\n";
                    // while (true);
                    // std::cout << "Aeq_:\n" << Aeq << "\nbeq_:\n" << beq << "\n";

                    /* ---------------------------- enforce full rank --------------------------- */
                    Eigen::MatrixXd Aeq_fullrank(0, 0);
                    Eigen::MatrixXd beq_fullrank(0, 0);
                    Eigen::MatrixXd Aeq_new(0, 0);
                    int rank{0};
                    for (uint i=0; i<Aeq.rows(); ++i)
                    {
                        Aeq_new = math::EigenVcat(Aeq_fullrank, Aeq.row(i));
                        Eigen::FullPivLU<Eigen::MatrixXd> lu(Aeq_new);
                        if (lu.rank() > rank) // appending new line increases rank
                        {
                            rank = lu.rank();
                            Aeq_fullrank = Aeq_new;
                            beq_fullrank = math::EigenVcat(beq_fullrank, beq.row(i));
                        }
                    }

                    Aeq = Aeq_fullrank;
                    beq = beq_fullrank;
                }
                else // only head and tail
                {
                    Aeq.resize(xdim_*2, xdim_*n_pt_);
                    beq.resize(xdim_*2,1);

                    Aeq << Eigen::MatrixXd::Identity(xdim_,xdim_), Eigen::MatrixXd::Zero(xdim_, (n_pt_-1)*xdim_),
                            Eigen::MatrixXd::Zero(xdim_, (n_pt_-1)*xdim_), Eigen::MatrixXd::Identity(xdim_,xdim_);

                    beq << xx.block(0, 0, xdim_, 1),
                        xx.block((n_pt_-1)*xdim_, 0, xdim_, 1);
                }
            }
            catch(const std::exception& e)
            {
                throw;
            }
        };

        this->EqSatisfied = [this, robot_ptr, processed_query_ptr] (const Eigen::VectorXd& xx)
        {
            const Eigen::MatrixXd DH = robot_ptr->DH();
            const Eigen::MatrixXd cart_traj_ref = processed_query_ptr->cart_traj_ref();
            const query::CartesianMode cart_mode = processed_query_ptr->mode().enforce_cartesian;

            Eigen::MatrixXd cart_traj = math::BatchFKine(math::ColWiseUnflatten(xx, robot_ptr->robot_DOF()), DH);
            Eigen::MatrixXd err = cart_traj - cart_traj_ref;
            Eigen::VectorXd trans_err_norm = err.topRows(TRANS_DIMS).colwise().norm();

            assert(cart_traj.cols() == n_pt_);
            Eigen::Matrix4d pose_mat, pose_mat_ref;
            Eigen::Vector3d axis_err = Eigen::Vector3d::Zero();
            for (uint i=0; i<n_pt_; ++i)
            {
                math::PoseAngRadToTransMat(cart_traj.col(i), pose_mat);
                math::PoseAngRadToTransMat(cart_traj_ref.col(i), pose_mat_ref);

                axis_err += ( pose_mat.topLeftCorner(AXIS_DIMS, AXIS_DIMS) -
                            pose_mat_ref.topLeftCorner(AXIS_DIMS, AXIS_DIMS) )
                            .colwise().norm().transpose();
            }
            axis_err /= n_pt_;

            // print cart err
            std::cout << "Trans err: [" << trans_err_norm.mean() << "] ";
            std::cout << "Axis err: [" << axis_err.transpose() << "] ";

            bool trans_ok{ trans_err_norm.mean() < eq_thres_ };
            bool rot_ok{ false };
            if (cart_mode >= query::CartesianMode::TransRotX && cart_mode <= query::CartesianMode::TransRotZ)
            {
                rot_ok = (bool) ( axis_err( (int) cart_mode ) < eq_thres_ );
            }
            else
            {
                rot_ok = true;
            }

            return (trans_ok && rot_ok);
        };

        /* ------------------------ SetCostMatrix3dTempoptFn ------------------------ */
        this->SetCostMatrix3dTempoptFn = [this] (Eigen::MatrixXd& H, Eigen::MatrixXd& f)
        {
            // std::cout << "start cost setting" << "\n";
            H = Eigen::MatrixXd::Identity(n_pt_, n_pt_);
            f = Eigen::MatrixXd::Zero(n_pt_, 1);
            // std::cout << "finished objective setting" << "\n";
        };

        /* -------------------------- LinConstraintTempopt -------------------------- */
        this->LinConstraintTempoptFn = [this] (Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final,
                                    const Eigen::VectorXd& tref, const Eigen::MatrixXd& xref_mat)
        {
            Eigen::MatrixXd Lfull, S;
            Eigen::MatrixXd xk0, xk1, xk2;
            double tk0, tk1;
            Eigen::MatrixXd lt0, lt1;
            Eigen::MatrixXd G, gG;
            Eigen::MatrixXd s, l;
            Eigen::MatrixXd Sstack, Lstack;

            // start inequality setting 
            for (int i=0; i<n_pt_; ++i){
                if (i >= 1 && i <=n_pt_-2){
                    // xk2 = xref_mat.block((i+1)*xdim_,0,xdim_,1);
                    // xk1 = xref_mat.block(i*xdim_,0,xdim_,1);
                    // xk0 = xref_mat.block((i-1)*xdim_,0,xdim_,1);
                    xk2 = xref_mat.col(i+1);
                    xk1 = xref_mat.col(i);
                    xk0 = xref_mat.col(i-1);
                    tk1 = tref(i,0); //.block(i,0,1,1);
                    tk0 = tref(i-1,0); //.block(i-1,0,1,1);

                    // cout << "x and z is good" << endl;
                    // G1 >= 0
                    lt0 = -2*(xk2 - xk1) + Eigen::MatrixXd::Ones(xdim_,1)*(amax_*tk1*tk1 + 2*amax_*tk1*tk0);
                    lt1 = 2*(xk1 - xk0) + Eigen::MatrixXd::Ones(xdim_,1)*(amax_*tk0*tk0 + 2*amax_*tk1*tk0);
                    // cout << "lt is good" << endl;

                    G = -2*tk0*(xk2 - xk1) + 2*tk1*(xk1 - xk0) + Eigen::MatrixXd::Ones(xdim_,1)*amax_*tk1*tk0*(tk1+tk0);
                    gG.resize(xdim_,n_pt_);
                    gG << Eigen::MatrixXd::Zero(xdim_,i-1), lt0, lt1, Eigen::MatrixXd::Zero(xdim_,n_pt_-i-1);

                    // cout << "gG is good" << endl;

                    s = -1*(gG * tref - G);
                    l.resize(xdim_,n_pt_);
                    l << Eigen::MatrixXd::Zero(xdim_,i-1), -1*lt0, -1*lt1, Eigen::MatrixXd::Zero(xdim_,n_pt_-i-1);

                    Sstack = cfslib::math::EigenVcat(Sstack, s);
                    Lstack = cfslib::math::EigenVcat(Lstack, l);
                    // cout << "G1 is good" << endl;

                    // G2 >= 0
                    lt0 = 2*(xk2 - xk1) + Eigen::MatrixXd::Ones(xdim_,1)*(amax_*tk1*tk1 + 2*amax_*tk1*tk0);
                    lt1 = -2*(xk1 - xk0) + Eigen::MatrixXd::Ones(xdim_,1)*(amax_*tk0*tk0 + 2*amax_*tk1*tk0);
                    
                    G = 2*tk0*(xk2 - xk1) - 2*tk1*(xk1 - xk0) + Eigen::MatrixXd::Ones(xdim_,1)*amax_*tk1*tk0*(tk1+tk0);
                    gG.resize(xdim_,n_pt_);
                    gG << Eigen::MatrixXd::Zero(xdim_,i-1), lt0, lt1, Eigen::MatrixXd::Zero(xdim_,n_pt_-i-1);
                    
                    s = -(gG * tref - G);
                    l.resize(xdim_,n_pt_);
                    l << Eigen::MatrixXd::Zero(xdim_,i-1), -1*lt0, -1*lt1, Eigen::MatrixXd::Zero(xdim_,n_pt_-i-1);
                    Sstack = cfslib::math::EigenVcat(Sstack, s);
                    Lstack = cfslib::math::EigenVcat(Lstack, l);
                    // cout << "G2 is good " << endl;
                    // cout << "middle is good " << endl;
                }

                if (i == 0){
                    // v0 - v1 + a1 * t1 = 0
                    // xk2 = xref_mat.block((i+1)*xdim_,0,xdim_,1);
                    // xk1 = xref_mat.block(i*xdim_,0,xdim_,1);
                    xk2 = xref_mat.col(i+1);
                    xk1 = xref_mat.col(i);
                    l.resize(xdim_,n_pt_);
                    l << -1*Eigen::MatrixXd::Ones(xdim_,1), Eigen::MatrixXd::Zero(xdim_,n_pt_-1);
                    s = -((xk1-xk2).cwiseAbs() / amax_).cwiseSqrt();
                    Sstack = cfslib::math::EigenVcat(Sstack, s);
                    Lstack = cfslib::math::EigenVcat(Lstack, l);
                    // cout << "1 is good" << endl;
                }

                if (i == n_pt_-1){
                    // v0 - v1 + a1 * t1 = 0
                    // xk1 = xref_mat.block(i*xdim_,0,xdim_,1);
                    // xk0 = xref_mat.block((i-1)*xdim_,0,xdim_,1);
                    xk1 = xref_mat.col(i);
                    xk0 = xref_mat.col(i-1);
                    l.resize(xdim_,n_pt_);
                    l << Eigen::MatrixXd::Zero(xdim_,n_pt_-2), -1*Eigen::MatrixXd::Ones(xdim_,1), Eigen::MatrixXd::Zero(xdim_,1);
                    s = -((xk0-xk1).cwiseAbs() / amax_).cwiseSqrt();
                    Sstack = cfslib::math::EigenVcat(Sstack, s);
                    Lstack = cfslib::math::EigenVcat(Lstack, l);
                    // cout << "final is good" << endl;
                }
            }

            Lfull_final = Lstack;
            S_final = Sstack;
        };

    }
    catch(const std::exception& e)
    {
        throw;
    }
}

Status PlanningCore::CFS(Eigen::MatrixXd& xref)
{
    try
    {
        Status stat = Status::None;

        // The iteration start

        // optimization vars
        Eigen::VectorXd x_vec, x_vec_prev;

        std::cout << ">> CFS start" << "\n";
        std::cout << ">> xref shape: " << xref.rows() << " " << xref.cols() << "\n";

        // init optim var with current x reference
        x_vec = math::ColWiseFlatten(xref);

        std::cout << ">> x_vec shape: " << x_vec.rows() << " " << x_vec.cols() << "\n";

        // !debug
        // x_vec.resize(30, 1);
        // x_vec << -0.6283,
        //             -2.5133,
        //             -0.9425,
        //             -1.5708,
        //             1.5708,
        //             0,
        //             -0.8639,
        //             -2.2777,
        //             -1.0996,
        //             -1.5708,
        //             1.5708,
        //             0,
        //             -1.0996,
        //             -2.0421,
        //             -1.2567,
        //             -1.5708,
        //             1.5708,
        //             0,
        //             -1.3352,
        //             -1.8064,
        //             -1.4137,
        //             -1.5708,
        //             1.5708,
        //             0,
        //             -1.5708,
        //             -1.5708,
        //             -1.5708,
        //             -1.5708,
        //             1.5708,
        //             0;

        // set Objective from planning problem
        Eigen::MatrixXd Hfull_, f_;
        Eigen::MatrixXd Lfull_, S_; // inequality constraints
        Eigen::MatrixXd Aeq_, beq_; // equality constraints
        Eigen::MatrixXd nLT_, nAeqT_; // negative transpose matrix
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0, x;

        // initialize reference input
        math::SetQuadVecFromEigen(x, x_vec);

        std::cout << ">> SetQuadVecFromEigen\n";

        // set objecive of QP solver 
        auto start_obj = std::chrono::high_resolution_clock::now();

        this->SetCostMatrix3dFn(Hfull_,f_, x_vec); // objective always using initial xref

        std::cout << ">> SetCostMatrix3dFn\n";

        // !debug [OK]
        // std::cout << "H:\n" << Hfull_ << "\nf:\n" << f_ << "\n";

        std::cout << "plnning problem finished the objective setting" << "\n";

        auto stop_obj = std::chrono::high_resolution_clock::now(); 
        auto duration_obj = std::chrono::duration_cast<std::chrono::microseconds>(stop_obj - start_obj); 

        std::cout << "the obj time is: " << double(duration_obj.count())/1000000.0 << " s" << "\n";

        math::SetQuadMatFromEigen(G, Hfull_);
        math::SetQuadVecFromEigen(g0, f_);

        std::cout << ">> begin cfs iteration\n";

        /* -------------------------------------------------------------------------- */
        /*                                cfs iteration                               */
        /* -------------------------------------------------------------------------- */
        x_vec_prev = x_vec;

        for (int k=0; k<max_iter_; k++)
        {
            std::cout << "\n>> Iteration " << k << " " << " ";

            // Processing
            // if (resample_i > 0)
            // {
            //     std::cout << "x_vec: \n" << x_vec << "\n";
            // }

            /* -------------------- set linear inequality constraint -------------------- */
            this->LinConstraintFn(Lfull_, S_, x_vec);
            nLT_ = -1*Lfull_.transpose(); // negative transpose
            math::SetQuadMatFromEigen(CI, nLT_);
            math::SetQuadVecFromEigen(ci0, S_);

            l_ = nLT_;
            s_ = S_;
            // !debug
            // CI.resize(nLT_.rows(), 0);
            // ci0.resize(0);
            // this->EqSatisfied(x_vec);
            
            int eq_T = enforce_eq_ ? eq_max_iter_ : 1;

            for (int eq_iter=0; eq_iter<eq_T; ++eq_iter)
            {
                /* --------------------- set linear equality constraint --------------------- */
                if (enforce_eq_)
                {
                    std::cout << "\n>>> [Eq " << eq_iter << "] ";
                    // !debug
                    // std::cout << "x_vec: " << x_vec.transpose() << "\n";
                }

                this->EqConstraintFn(Aeq_, beq_, x_vec);

                nAeqT_ = -1*Aeq_.transpose();
                math::SetQuadMatFromEigen(CE, nAeqT_);
                math::SetQuadVecFromEigen(ce0, beq_);

                // !debug
                // std::cout << "Aeq_:\n" << Aeq_ << "\nbeq_:\n" << beq_ << "\n";

                // Eigen::FullPivLU<Eigen::MatrixXd> luAeq(Aeq_);
                // int r{luAeq.rank()};
                // std::cout << "Aeq_ rank: " << r << "\n";
                // while (enforce_eq_);

                // Eigen::MatrixXd Abeq = math::EigenHcat(Aeq_, beq_);
                // Eigen::FullPivLU<Eigen::MatrixXd> luAbeq(Abeq);
                // r = luAbeq.rank();
                // std::cout << "Abeq rank: " << r << "\n";

                // Eigen::MatrixXd Abeq0 = Abeq.block(6, 0, 6, Abeq.cols());
                // // Eigen::MatrixXd Abeq0_no_zero = math::EigenVcat(Abeq0.topRows(4), Abeq0.bottomRows(1));

                // std::cout << "Abeq top left:\n" << Abeq0 << "\n";

                // Eigen::FullPivLU<Eigen::MatrixXd> luAbeq0(Abeq0);
                // r = luAbeq0.rank();
                // std::cout << "Abeq0 rank: " << r << "\n";

                // std::cout << "5th row is almost zero: " << Abeq0.row(4).isApprox(Eigen::MatrixXd::Zero(1, Abeq0.cols())) << "\n";

                // Eigen::MatrixXd tmp = Aeq_.block(0, 0, 12, Aeq_.cols());
                // Eigen::FullPivLU<Eigen::MatrixXd> luAeq0(tmp);
                // r = luAeq0.rank();

                // std::cout << "Aeq0:\n" << tmp << "\n";
                // std::cout << "Aeq0 rank: " << r << "\n";

                // std::cout << "Lfull_:\n" << Lfull_ << "\nS_:\n" << S_ << "\n";
                // while (true);

                // Solve the subproblem
                quadprogpp::Matrix<double> G_copy = G;
                quadprogpp::Vector<double> g0_copy = g0;
                quadprogpp::Matrix<double> CI_copy = CI;
                quadprogpp::Vector<double> ci0_copy = ci0;

                // if (resample_i > 0)
                // {
                //     std::cout << "G: \n" << G_copy << "\n";
                //     std::cout << "g0: \n" << g0_copy << "\n";
                //     std::cout << "CE: \n" << CE << "\n";
                //     std::cout << "ce0: \n" << ce0 << "\n";
                //     std::cout << "CI: \n" << CI << "\n";
                //     std::cout << "ci0: \n" << ci0 << "\n";

                //     while (true);
                // }

                std::cout << "solving QP... ";
                double tmp_cost = solve_quadprog(G_copy, g0_copy, CE, ce0, CI_copy, ci0_copy, x);
                std::cout << "cost : " << tmp_cost << " ";

                math::SetEigenMatFromQuad(x_vec, x);

                if (!math::ConstraintsSatisfied(x_vec, Lfull_, S_, Aeq_, beq_))
                {
                    std::ostringstream ss;
                    ss << ERR_HEADER << "quadprog++ infeasible solution.";
                    throw std::runtime_error(ss.str());
                }

                // !debug
                // std::cout << "x_vec: " << x_vec << "\n";
                // while (true);

                if (enforce_eq_)
                {
                    if (this->EqSatisfied(x_vec))
                    {
                        std::cout << "\n>>> CFSLin converged at step " << eq_iter << "\n";
                        break;
                    }
                    else if (eq_iter == eq_T-1)
                    {
                        std::cout << "\n>>> CFSLin not converged.\n";
                        return Status::CFSLinNotConverged;
                    }

                    // !debug
                    // while (true);
                }
            }

            // Convergence check
            // compare new xreference
            if (math::ApproxEq(x_vec, x_vec_prev, convergence_thres_))
            {
                std::cout << "\n>> CFS converged at step " << k << "\n";
                xref = math::ColWiseUnflatten(x_vec, xdim_);
                return Status::OK;
            }
            else if (k == max_iter_-1)
            {
                std::cout << "\n>> CFS not converged.\n";
                xref = math::ColWiseUnflatten(x_vec, xdim_);
                return Status::CFSNotConverged;
            }
            x_vec_prev = x_vec;

        } // end cfs iteration

        return Status::LogicError;
    }
    catch(const std::exception& e)
    {
        throw;
    }   
}

Status PlanningCore::CFSTempOpt(Eigen::VectorXd& tref, const Eigen::MatrixXd& xref)
{
    try
    {
        // set Objective from planning problem
        Eigen::MatrixXd Hfull_, f_;
        Eigen::MatrixXd Lfull_, S_; // inequality constraints
        Eigen::MatrixXd Aeq_, beq_; // equality constraints
        Eigen::MatrixXd nLT_, nAeqT_; // negative transpose matrix
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0, x;

        // initilize the time profile reference 
        tref = Eigen::MatrixXd::Ones(n_pt_,1) * dt_tempopt_;
        // amax_ = robot.umax;

        // initialize reference input
        math::SetQuadVecFromEigen(x, tref); // set initial value of u;
        Eigen::MatrixXd tref_prev = tref;

        // set objecive of QP solver 
        auto start_obj = std::chrono::high_resolution_clock::now();
        // pp_->setObjective3d(Hfull_,f_,robot);

        // SetCostMatrix3dTempopt(Hfull_,f_);
        this->SetCostMatrix3dTempoptFn(Hfull_, f_);

        // std::cout << "temporal optimization problem finished the objective setting" << "\n";
        auto stop_obj = std::chrono::high_resolution_clock::now(); 
        auto duration_obj = std::chrono::duration_cast<std::chrono::microseconds>(stop_obj - start_obj); 
        // std::cout << "the obj time is: " << double(duration_obj.count())/1000000.0 << "seconds" << "\n"; 
        math::SetQuadMatFromEigen(G, Hfull_);
        math::SetQuadVecFromEigen(g0, f_);

        // The iteration start

        for (int k=0; k<max_iter_; k++){
            // for (int k=0; k<1; k++){
            std::cout << "\n>> Iteration " << k << " ";

            // set linear inequality constraint
            // LinConstraintTempopt(Lfull_, S_, tref, xref);
            this->LinConstraintTempoptFn(Lfull_, S_, tref, xref);
            nLT_ = -1*Lfull_.transpose(); // negative transpose
            math::SetQuadMatFromEigen(CI, nLT_);
            math::SetQuadVecFromEigen(ci0, S_);

            // set linear equality constraint 
            // no equality constraint
            Aeq_ = Eigen::MatrixXd::Zero(1, tref.rows());
            beq_ = Eigen::MatrixXd::Zero(1, 1);
            CE.resize(n_pt_,0);
            ce0.resize(0);

            // Solve the subproblem
            std::cout << "solving QP...";
            quadprogpp::Matrix<double> G_copy = G;
            quadprogpp::Vector<double> g0_copy = g0;

            double tmp_cost = solve_quadprog(G_copy, g0_copy, CE, ce0, CI, ci0, x);
            math::SetEigenMatFromQuad(tref, x); // update uref_ in planning problem;

            if (!math::ConstraintsSatisfied(tref, Lfull_, S_, Aeq_, beq_))
            {
                std::ostringstream ss;
                ss << ERR_HEADER << "quadprog++ infeasible solution.";
                throw std::runtime_error(ss.str());
            }
            
            std::cout << "cost: " << tmp_cost << "\n";

            //Convergence check
            // compare new xreference
            if (math::ApproxEq(tref, tref_prev, convergence_thres_))
            {
                std::cout << ">> Converged at step " << k+1 << "\n";
                break;
            }
            tref_prev = tref;
        }

        return Status::OK;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

Status PlanningCore::Solve(const robot::Robot::ConstPtr& robot_ptr,
                    const query::ProcessedQuery::ConstPtr& processed_query_ptr,
                    Eigen::MatrixXd& xref, Eigen::VectorXd& tref)
{
    try
    {
        this->CoreParse(robot_ptr, processed_query_ptr, xref);
        
        std::cout << "\n_______________________________ CFS _______________________________\n";
        Status cfs_stat = this->CFS(xref);

        if (cfs_stat != Status::OK)
        {
            return cfs_stat;
        }

        if (enable_tempopt_)
        {
            std::cout << "\n____________________________ CFS TempOpt ____________________________\n";
            Status cfs_tempopt_stat = this->CFSTempOpt(tref, xref);
            if (cfs_tempopt_stat != Status::OK)
            {
                return cfs_tempopt_stat;
            }
        }

        return Status::OK;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}
}