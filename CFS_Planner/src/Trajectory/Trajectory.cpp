#include "cfslib/Trajectory/Trajectory.hpp"

namespace cfslib
{

namespace trajectory
{

Trajectory::Trajectory()
{
    
}

void Trajectory::GeneratePath(Eigen::MatrixXd q, 
                              const robot::Robot::ConstPtr& robot_ptr,
                              const query::ProcessedQuery::ConstPtr& processed_query_ptr)
{
    Eigen::MatrixXd x(6, q.cols());
    Eigen::MatrixXd q0(q.rows(), 1);
    Eigen::MatrixXd DH = robot_ptr->DH();
    math::Vector6d x0;

    joint_path_waypoints_ = q;
    joint_init_reference_ = processed_query_ptr->joint_traj_ref();
    cart_init_reference_ = processed_query_ptr->cart_traj_ref();
    assert(joint_init_reference_.cols() == joint_path_waypoints_.cols());
    double exec_time = processed_query_ptr->execution_time();
    double horizon = cart_init_reference_.cols()-1;
    double dt = exec_time / horizon;

    std::vector<int> critical_ids = processed_query_ptr->critical_ids();
    joint_path_waypoints_critical_.resize(joint_path_waypoints_.rows(), critical_ids.size());
    for(int i=0; i<critical_ids.size(); i++){
        joint_path_waypoints_critical_.col(i) = joint_path_waypoints_.col(critical_ids.at(i));
    }
    
    for(int i=0; i<q.cols(); i++){
        q0 = q.col(i);
        math::TransMatToPoseAngRad(math::FKine(q0, DH), x0);
        x.col(i) << x0;
    }
    cart_path_waypoints_ = x;
    dist_init_profile_ = DistanceProfile(joint_init_reference_, dt, robot_ptr, processed_query_ptr);
    dist_profile_ = DistanceProfile(q, dt, robot_ptr, processed_query_ptr);
    
    cart_vel_profile_ = Derivative(cart_path_waypoints_, dt);
    cart_vel_profile_reference_ = Derivative(cart_init_reference_, dt);
    cart_acc_profile_ = Derivative(cart_vel_profile_, dt);
    cart_acc_profile_reference_ = Derivative(cart_vel_profile_reference_, dt);

    joint_vel_profile_ = Derivative(q, dt);
    joint_vel_profile_reference_ = Derivative(joint_init_reference_, dt);
    joint_acc_profile_ = Derivative(joint_vel_profile_, dt);
    joint_acc_profile_reference_ = Derivative(joint_vel_profile_reference_, dt);
} 

Eigen::MatrixXd Trajectory::DistanceProfile(Eigen::MatrixXd q, double dt,
                                            const robot::Robot::ConstPtr& robot_ptr,
                                            const query::ProcessedQuery::ConstPtr& processed_query_ptr)
{
    Eigen::MatrixXd dist_list(1, q.cols());
    Eigen::MatrixXd q0(q.rows(), 1);
    double min_d, dis, t;
    const auto& obstacle_capsules = processed_query_ptr->obstacles_cap();
    const auto& obstacle_bounding_boxes = processed_query_ptr->obstacles_box();
    double collision_thresh = processed_query_ptr->safety_margin();
    collision_valid_ = true;
    for(int i=0; i<q.cols(); i++){
        q0 = q.col(i);
        t = i * dt;

        // calculate the closest distance
        auto robot_cap_now = robot_ptr->capsules_now(q0); // get cap at joint state q0
        min_d = std::numeric_limits<double>::infinity();
        
        for (int nlink=0; nlink<robot_cap_now.size(); ++nlink){
            for(int j=0; j<obstacle_capsules.size(); ++j){
                dis = math::DistCap2Cap(robot_cap_now[nlink], obstacle_capsules[j].at(t));
                if (dis < min_d){
                    min_d = dis;
                }
            }
            for(int j=0; j<obstacle_bounding_boxes.size(); ++j){
                dis = math::DistCap2BB(robot_cap_now[nlink], obstacle_bounding_boxes[j].at(t));
                if (dis < min_d){
                    min_d = dis;
                }
            }
        }
        dist_list.col(i) << min_d;
        if(min_d < collision_thresh - 0.001){
            collision_valid_ = false;
        }
    }
    return dist_list;
}

bool Trajectory::CollisionValidStatic(Eigen::MatrixXd q, double dt,
                                        const robot::Robot::ConstPtr& robot_ptr,
                                        const query::ProcessedQuery::ConstPtr& processed_query_ptr)
{
    Eigen::MatrixXd dist_list(1, q.cols());
    Eigen::MatrixXd q0(q.rows(), 1);
    Eigen::MatrixXd q_min_dist;
    double min_d, dis, t;
    const auto& obstacle_capsules = processed_query_ptr->obstacles_cap();
    const auto& obstacle_bounding_boxes = processed_query_ptr->obstacles_box();
    double collision_thresh = processed_query_ptr->safety_margin();
    bool collision_valid = true;
    for(int i=0; i<q.cols(); i++){
        q0 = q.col(i);
        t = i * dt;

        // calculate the closest distance
        auto robot_cap_now = robot_ptr->capsules_now(q0); // get cap at joint state q0
        min_d = std::numeric_limits<double>::infinity();
        
        for (int nlink=0; nlink<robot_cap_now.size(); ++nlink){
            for(int j=0; j<obstacle_capsules.size(); ++j){
                dis = math::DistCap2Cap(robot_cap_now[nlink], obstacle_capsules[j].at(t));
                if (dis < min_d){
                    min_d = dis;
                    q_min_dist = q0;
                }
            }
            for(int j=0; j<obstacle_bounding_boxes.size(); ++j){
                dis = math::DistCap2BB(robot_cap_now[nlink], obstacle_bounding_boxes[j].at(t));
                if (dis < min_d){
                    min_d = dis;
                    q_min_dist = q0;

                    // std::cout << ">> ---------------- new min d in pre check ----------------\n";
                    // std::cout << ">> robot pose: \n" << math::print_eigen_mat(q0.transpose()) << "\n";
                    // std::cout << ">> robot cap id: " << nlink << "\n";
                    // std::cout << ">> obs id: " << j << "\n";
                    // std::cout << ">> robot cap: \n" << math::print_eigen_mat(robot_cap_now[nlink].p.transpose()) << "\n";
                    // std::cout << ">> robot cap radius: " << robot_cap_now[nlink].r << "\n";
                    // std::cout << ">> bbox p1_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p1_1) << "\n";
                    // std::cout << ">> bbox p1_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p1_2) << "\n";
                    // std::cout << ">> bbox p2_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p2_1) << "\n";
                    // std::cout << ">> bbox p2_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p2_2) << "\n";
                    // std::cout << ">> bbox p3_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p3_1) << "\n";
                    // std::cout << ">> bbox p3_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes[j].at(t).p3_2) << "\n";
                    // std::cout << ">> distance: " << min_d << "\n";
                }
            }
        }
        dist_list.col(i) << min_d;

        if(min_d < collision_thresh - 0.001){
            collision_valid = false;
            std::cout << "collison" << std::endl;
            for (int h = 0; h < 6; h++){
                std::cout << q0 << std::endl;
            }
        }
    }

    // std::cout << "min_d: " << min_d << "\n";
    // std::cout << "q_min_dist: \n" << math::print_eigen_mat(q_min_dist.transpose()) << "\n";
    // auto robot_cap_min_d = robot_ptr->capsules_now(q_min_dist);
    // std::cout << ">> robot cap: \n" << math::print_eigen_mat(robot_cap_min_d.back().p.transpose()) << "\n";
    // std::cout << ">> robot cap radius: " << robot_cap_min_d.back().r << "\n";
    // std::cout << ">> bbox p1_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p1_1) << "\n";
    // std::cout << ">> bbox p1_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p1_2) << "\n";
    // std::cout << ">> bbox p2_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p2_1) << "\n";
    // std::cout << ">> bbox p2_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p2_2) << "\n";
    // std::cout << ">> bbox p3_1: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p3_1) << "\n";
    // std::cout << ">> bbox p3_2: \n" << math::print_eigen_mat(obstacle_bounding_boxes.front().at(t).p3_2) << "\n";
    // std::cout << "collision_thresh: " << collision_thresh << "\n";

    return collision_valid;
}

Eigen::MatrixXd Trajectory::Derivative(Eigen::MatrixXd q, double dt)
{
    Eigen::MatrixXd vel_prof(q.rows(), q.cols());
    Eigen::MatrixXd v(q.rows(), 1);
    for(int i=0; i<q.cols()-1; i++)
    {
        v = (q.col(i+1) - q.col(i)) / dt;
        vel_prof.col(i) << v;
    }
    v = v * 0;
    vel_prof.col(q.cols()-1) << v;
    return vel_prof;

}

}

}