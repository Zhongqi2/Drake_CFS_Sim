#include "cfslib/cfslib.hpp"

namespace cfslib
{

CFSLib::CFSLib()
{

}

trajectory::Trajectory::ConstPtr CFSLib::Solve(const query::UserQuery::ConstPtr& query_ptr)
{
    try
    {
        robot::Robot::Ptr robot_ptr{std::make_shared<robot::Robot>()};
        robot_ptr->Setup(query_ptr);

        query::ProcessedQuery::Ptr processed_query_ptr{std::make_shared<query::ProcessedQuery>()};
        processed_query_ptr->Process(query_ptr);

        // ! tmp
        Eigen::VectorXd joint_traj_ref(30);
        joint_traj_ref << -0.6283, -2.5133, -0.9425, -1.5708, 1.5708, 0,
                        -0.86392, -2.2777, -1.0996, -1.5708, 1.5708, 0,
                        -1.0996, -2.0421, -1.2567, -1.5708, 1.5708, 0,
                        -1.3352, -1.8064, -1.4137, -1.5708, 1.5708, 0,
                        -1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 0;
        processed_query_ptr->set_joint_traj_ref(math::ColWiseUnflatten(joint_traj_ref, 6));
        processed_query_ptr->set_trajectory_frequency(10.0);
        // ! tmp end

        core::PlanningCore::Ptr core_ptr{std::make_shared<core::PlanningCore>()};
        
        Eigen::MatrixXd xref;
        Eigen::VectorXd tref;
        core_ptr->Solve(robot_ptr, processed_query_ptr, xref, tref);

        std::cout << "xref sol: \n" << xref << "\n";
        std::cout << "time sol: \n" << tref << "\n";

        return std::make_shared<trajectory::Trajectory>();
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

bool CFSLib::JointVelOK(const Eigen::MatrixXd& x, const Eigen::MatrixXd& dx_limit)
{
    int horizon{ x.cols() - 1 };
    Eigen::MatrixXd dx((x.leftCols(horizon) - x.rightCols(horizon)).cwiseAbs());
    Eigen::VectorXd dx_max( dx.rowwise().maxCoeff() );
    #ifdef DEBUG_PRINT
        // std::cout << ">> x:\n" << x << "\n";
    #endif

    bool OK { (dx_max - dx_limit).maxCoeff() < 0 };

    std::cout << ">> dvel_max: " << dx_max.transpose() << "\n";
    std::cout << ">> dvel_limit: " << dx_limit.transpose() << "\n";

    return OK;
}

bool CFSLib::JointDispOK(const Eigen::MatrixXd& x, const Eigen::MatrixXd& dx_limit)
{
    Eigen::MatrixXd dx_max(6, 1);
    dx_max << 0, 0, 0, 0, 0, 0;
    bool ret = true;
    for(int r=0; r<x.rows(); r++)
    {
        for(int c=1; c<x.cols(); c++)
        {
            if(abs(x(r, c) - x(r, c-1)) > dx_max(r, 0))
            {
                dx_max(r, 0) = abs(x(r, c) - x(r, c-1));
            }
        }
        if(dx_max(r, 0) > dx_limit(r, 0))
        {
            ret = false;
        }
    }
    std::cout << ">> ddisp_max: " << dx_max.transpose() << "\n";
    std::cout << ">> ddisp_limit: " << dx_limit.transpose() << "\n";
    return ret;
}

// Eigen::MatrixXd ResamplePath(const Eigen::MatrixXd& x)
// {
//     Eigen::MatrixXd new_x(x.rows(), x.cols()*2-1);
//     for(int i=0; i<new_x.cols(); i++)
//     {
//         if(i % 2 == 0){
//             new_x.col(i) = x.col((int)(i/2));
//         }
//         else{
//             int pre_i = (i-1)/2;
//             int post_i = (i+1)/2;
//             Eigen::MatrixXd diff = (x.col(post_i) - x.col(pre_i));
//             for(int j=0; j<diff.rows(); j++){
//                 diff(j, 0) = diff(j, 0) / 2;
//             }
//             new_x.col(i) = x.col(pre_i) + diff;
//         }
//     }
//     return new_x;
// }

trajectory::Trajectory::Ptr CFSLib::DemoSolve(const query::UserQuery::ConstPtr& query_ptr)
{
    try
    {
        robot::Robot::Ptr robot_ptr{std::make_shared<robot::Robot>()};
        robot_ptr->Setup(query_ptr);
        query::ProcessedQuery::Ptr processed_query_ptr;
        Eigen::MatrixXd xref;
        Eigen::VectorXd tref;
        core::PlanningCore::Ptr core_ptr;
        double traj_freq{-1};
        // processed_query_ptr = std::make_shared<query::ProcessedQuery>();
        // processed_query_ptr->Process(query_ptr, traj_freq);

        int resample_cnt{0}, resample_max, resample_n_insert;
        resample_max = 3;
        resample_n_insert = 10;
        core::Status ret;
        bool CFS_valid = false;
        bool is_dense = false;
        bool vel_valid = false;

        while (resample_cnt++ < resample_max)
        {
            processed_query_ptr = std::make_shared<query::ProcessedQuery>();
            processed_query_ptr->Process(query_ptr, traj_freq);

            // check collision
            if (trajectory::Trajectory::CollisionValidStatic(
                processed_query_ptr->joint_traj_ref(),
                processed_query_ptr->execution_time() / processed_query_ptr->horizon(),
                robot_ptr, processed_query_ptr
            ))
            {
                // xref = processed_query_ptr->joint_traj_ref();
                // ret = core::Status::OK;

                std::cout << ">> No collision.\n";
            }
            else
            {
                std::cout << "Initial trajectory is in collision.\n";
                CFS_valid = false;
                xref = processed_query_ptr->joint_traj_ref();
                break;
            }
            
            std::cout << ">> Solve CFS.\n";

            core_ptr = std::make_shared<core::PlanningCore>();
            ret = core_ptr->Solve(robot_ptr, processed_query_ptr, xref, tref);
            
            Eigen::MatrixXd dx_limit = robot_ptr->joint_vel_limits() / processed_query_ptr->trajectory_frequency();

            // std::cout << "checking joint vel limits\n";
            // std::cout << robot_ptr->joint_vel_limits()<< std::endl; 
            // std::cout << processed_query_ptr->trajectory_frequency() << std::endl;

            Eigen::MatrixXd joint_disp_limit = robot_ptr->joint_displacement_limits(); 
            bool joint_vel_OK, joint_disp_OK;           
            
            // Point Goal
            if(query_ptr->mode().goal_type == query::GoalType::JointPoint ||
               query_ptr->mode().goal_type == query::GoalType::CartPoint)
            {
                
                joint_vel_OK = JointVelOK(xref, dx_limit);
                if(!joint_vel_OK){
                    std::cout << ">> Joint velocity limit exceeded.\n";
                    std::cout << "> CFS failed to find trajectory within joint velocity limit. Increase initial execution time.\n";
                    CFS_valid = (bool) (ret == core::Status::OK);
                    break;
                }
                joint_disp_OK = JointDispOK(xref, joint_disp_limit);
                if (!joint_disp_OK)
                {
                    if (resample_cnt < resample_max)
                    {
                        double traj_freq_old{ processed_query_ptr->trajectory_frequency() };
                        traj_freq = traj_freq_old + resample_n_insert;
                        std::cout << "> [" << resample_cnt << "] CFS resample increasing traj freq from [" << traj_freq_old
                                << "] to [" << traj_freq << "].\n";
                    }
                    else
                    {
                        std::cout << "> CFS failed to find trajectory within joint displacement limit. \n";
                        CFS_valid = (bool) (ret == core::Status::OK);
                        vel_valid = joint_vel_OK;
                        is_dense = joint_disp_OK;
                        break;
                    }
                }
                else
                {
                    std::cout << "> CFS finished.\n";
                    CFS_valid = (bool) (ret == core::Status::OK);
                    is_dense = true;
                    vel_valid = true;
                    break;
                }

            }
            else
            {
                joint_vel_OK = JointVelOK(xref, dx_limit);
                joint_disp_OK = JointDispOK(xref, joint_disp_limit);
                if(!joint_vel_OK){
                    std::cout << ">> Joint velocity limit exceeded.\n";
                    std::cout << "> CFS failed to find trajectory within joint velocity limit. Increase initial execution time.\n";
                }
                if(!joint_disp_OK)
                {
                    std::cout << ">> Joint displacement limit exceeded.\n";
                    std::cout << "> CFS failed to find trajectory within joint displacement limit. \n";
                }
                CFS_valid = (bool) (ret == core::Status::OK);
                vel_valid = joint_vel_OK;
                is_dense = joint_disp_OK;
                break;
            }
        }

        // cfslib::io::SaveMatToFile(core_ptr->get_l(), "results/L.txt");
        // cfslib::io::SaveMatToFile(core_ptr->get_s(), "results/S.txt");
        trajectory::Trajectory::Ptr traj_ptr = std::make_shared<trajectory::Trajectory>();
        traj_ptr->GeneratePath(xref, robot_ptr, processed_query_ptr);
        traj_ptr->set_dense_flag(is_dense);
        traj_ptr->set_status(CFS_valid);
        traj_ptr->set_vel_flag(vel_valid);
        return traj_ptr;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

// solve only up to pq
query::ProcessedQuery::Ptr CFSLib::TmpSolve(const query::UserQuery::ConstPtr& query_ptr)
{
    try
    {
        robot::Robot::Ptr robot_ptr{std::make_shared<robot::Robot>()};
        robot_ptr->Setup(query_ptr);

        query::ProcessedQuery::Ptr processed_query_ptr{std::make_shared<query::ProcessedQuery>()};
        processed_query_ptr->Process(query_ptr);

        core::PlanningCore::Ptr core_ptr{std::make_shared<core::PlanningCore>()};

        return processed_query_ptr;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}