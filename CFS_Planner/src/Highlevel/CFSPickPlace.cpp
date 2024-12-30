#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "cfslib/Highlevel/CFSPickPlace.hpp"

Eigen::MatrixXd get_DH(const std::string &robot_model)
{
    if (robot_model == "ur5")
    {
        Eigen::MatrixXd DH(8, 4);
        DH << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.089159, 0.0, 1.5707963267948966,
            -1.5707963267948966, 0.0, -0.425, 0.0,
            0.0, 0.0, -0.39225, 0.0,
            -1.5707963267948966, 0.10915, 0.0, 1.5707963267948966,
            0.0, 0.09465, 0.0, -1.5707963267948966,
            0.0, 0.0823, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0;
        return DH;
    }
    else if (robot_model == "gp7")
    {
        Eigen::MatrixXd DH(8, 4);
        DH << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.330, 0.040, -1.5707963267948966,
            1.5707963267948966, 0.0, -0.445, 3.14159265359,
            0.0, 0.0, -0.040, 1.5707963267948966,
            0.0, -0.44, 0.0, -1.5707963267948966,
            0.0, 0.0, 0.0, 1.5707963267948966,
            0.0, -0.080, 0.0, 0.0,
            0, 0, 0, 0;
        return DH;
    }
    else if (robot_model == "gp12")
    {
        Eigen::MatrixXd DH(8, 4);
        DH << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.155, -1.5707963267948966,
            1.5707963267948966, 0.0, -0.614, 3.14159265359,
            0.0, 0.0, -0.200, 1.5707963267948966,
            0.0, -0.64, 0.0, -1.5707963267948966,
            0.0, 0.0, 0.0, 1.5707963267948966,
            0.0, -0.100, 0.0, 0.0,
            0, 0, 0, 0;
        return DH;
    }
    else
    {
        throw std::invalid_argument("Invalid robot model");
        return Eigen::MatrixXd::Zero(8, 4);
    }
}

Eigen::Matrix4d FKine(const Eigen::VectorXd& q, const std::string& robot_model)
{
    return cfslib::math::FKine(q, robot_model);
}

// Eigen::Matrix4d FKine(const Eigen::VectorXd& q, const std::string& robot_model)
// {
//     // DH row: [ theta, d, a, alpha ]
//     // row 0:                   base to joint 1
//     // row [1, n_joint]:        joint i to i+1
//     // row [n_joint+1, end]:    additional trans for until ee

//     Eigen::MatrixXd DH = get_DH(robot_model);

//     std::vector<std::function<Eigen::Matrix4d(double q)>> link_transform;
//     int n_joint{q.rows()};

//     link_transform.clear();
//     // generate transform matrix including ee links
//     for (uint i = 0; i < DH.rows(); ++i)
//     {
//         link_transform.emplace_back(
//             [DH, i] (double q) {
//                 Eigen::Matrix4d trans_mat;
//                 trans_mat <<
//                     cos(q+DH(i, 0)), -sin(q+DH(i, 0))*cos(DH(i, 3)),  sin(q+DH(i, 0))*sin(DH(i, 3)), DH(i, 2)*cos(q+DH(i, 0)),
//                     sin(q+DH(i, 0)),  cos(q+DH(i, 0))*cos(DH(i, 3)), -cos(q+DH(i, 0))*sin(DH(i, 3)), DH(i, 2)*sin(q+DH(i, 0)),
//                                   0,                  sin(DH(i, 3)),                  cos(DH(i, 3)),                 DH(i, 1),
//                                   0,                              0,                              0,                        1;
//                 return trans_mat;
//             }
//         );
//     }

//     // base to joint 1
//     auto f_transform = link_transform.at(0);
//     auto trans_mat = f_transform(0.0f);

//     // joint 1 to link n_joint
//     for (uint i = 0; i < n_joint; ++i)
//     {
//         // i -> i+1, result is pose of joint i+1
//         f_transform = link_transform.at(i + 1);
//         trans_mat = trans_mat * f_transform(q(i));
//     }

//     // link n_joint to ee
//     for (uint i = n_joint + 1; i < link_transform.size(); ++i)
//     {
//         f_transform = link_transform.at(i);
//         trans_mat = trans_mat * f_transform(0.0f);
//     }

//     return trans_mat;
// }

cfslib::math::Vector6d _IK_single_pt(
    const cfslib::math::Vector6d &c_ref, const cfslib::math::Vector6d &c0,
    const cfslib::math::VectorJd &theta0, const Eigen::MatrixXd &DH,
    const double &Cartesian_thres, const int &max_iter)
{
    return cfslib::math::IK(
        c_ref, c0, theta0, DH, Cartesian_thres, 1.0e-3, max_iter
    );
}

// cfslib::math::Vector6d _IK_single_pt(
//     const cfslib::math::Vector6d &c_ref, const cfslib::math::Vector6d &c0,
//     const cfslib::math::VectorJd &theta0, const Eigen::MatrixXd &DH,
//     const double &Cartesian_thres, const int &max_iter)
// {
//     double kp = 0.2;
//     double kd = 0.01;
//     int cnt = 0;
//     cfslib::math::Vector6d c_var, c_err, c_err_prev, c_delta;
//     cfslib::math::VectorJd theta_var{theta0};
//     c_var = c0;
//     c_err = c_ref - c_var;
//     c_err_prev = c_err;

//     double max_delta_theta = -1;

//     while (c_err.norm() > Cartesian_thres)
//     {
//         // std::cout << ">> _IK_single_pt iter: " << cnt << "\n";
//         // check Jacobian singularity
//         Eigen::MatrixXd J = cfslib::math::Jacobi(theta_var, c_var, DH);

//         // Perform SVD on the Jacobian
//         Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
//         Eigen::VectorXd singularValues = svd.singularValues();
//         Eigen::MatrixXd U = svd.matrixU();
//         Eigen::MatrixXd V = svd.matrixV();

//         // perturb joint until out of singularity
//         int cnt_singular = 0;
//         while (singularValues.minCoeff() < 0.01)
//         {
//             theta_var = theta_var + V.col(5) * 0.1;
//             cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_var, DH), c_var);

//             J = cfslib::math::Jacobi(theta_var, c_var, DH);
//             svd.compute(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
//             singularValues = svd.singularValues();
//             U = svd.matrixU();
//             V = svd.matrixV();

//             // std::cout << "\n >>>>>>>> perturb \n";
//             // std::cout << ">> theta_var: " << theta_var.transpose() << "\n";
//             // std::cout << ">> c_var: " << c_var.transpose() << "\n";
//             // std::cout << ">> singularValues: " << singularValues.transpose() << "\n";

//             if (++cnt_singular > 100)
//             {
//                 break;
//             }
//         }

//         c_err = c_ref - c_var;
//         c_delta = kp * c_err + kd * (c_err - c_err_prev);

//         // apply jacob and update c
//         cfslib::math::VectorJd theta_delta = cfslib::math::PInv(J) * c_delta;
//         theta_var = theta_var + theta_delta;
//         cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_var, DH), c_var);

//         c_err_prev = c_err;
//         c_err = c_ref - c_var;

//         /* ---------------------------------- debug --------------------------------- */

//         // std::cout << "\n\n\n>> +++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
//         // std::cout << ">> cnt: " << cnt << "\n";
//         // std::cout << ">> c0:            " << cfslib::math::print_eigen_mat(c0.transpose()) << "\n";
//         // std::cout << ">> c_var:         " << cfslib::math::print_eigen_mat(c_var.transpose()) << "\n";
//         // std::cout << ">> c_ref:         " << cfslib::math::print_eigen_mat(c_ref.transpose()) << "\n";
//         // std::cout << ">> c_delta:       " << cfslib::math::print_eigen_mat(c_delta.transpose()) << "\n";
//         // std::cout << ">> theta0:        " << cfslib::math::print_eigen_mat(theta0.transpose()) << "\n";
//         // std::cout << ">> theta_delta:   " << cfslib::math::print_eigen_mat(theta_delta.transpose()) << "\n";
//         // std::cout << ">> theta_var:     " << cfslib::math::print_eigen_mat(theta_var.transpose()) << "\n";
//         // std::cout << ">> c_err:         " << cfslib::math::print_eigen_mat(c_err.transpose()) << "\n";
//         // std::cout << ">> c_err norm:    " << c_err.norm() << "\n";
//         // std::cout << ">> Jacobi: \n" << J << "\n";
//         // std::cout << ">> Jacobi SVD: \n" << math::print_eigen_mat(singularValues.transpose()) << "\n";
//         // std::cout << ">> Jacobi U: \n" << U << "\n";
//         // std::cout << ">> Jacobi_inv: \n" << cfslib::math::PInv(J) << "\n";

//         // get out of singularity

//         /* ---------------------------------- debug --------------------------------- */

//         // if (theta_delta.norm() > max_delta_theta)
//         // {
//         //     max_delta_theta = theta_delta.norm();
//         // }

//         // if (max_delta_theta > 10)
//         // {
//         //     std::ostringstream ss;
//         //     ss << ERR_HEADER << "IK: Exceeded max delta theta " << 10;
//         //     throw std::runtime_error(ss.str());
//         // }

//         if (++cnt > max_iter)
//         {
//             break;
//         }
//     }

//     // std::cout << ">> converged in: " << cnt << " iterations\n";
//     // std::cout << ">> max_delta_theta: " << max_delta_theta << "\n";

//     return theta_var;
// }

cfslib::math::Vector6d IKine(
    const cfslib::math::VectorJd &theta0, const Eigen::Matrix4d &mat_ref,
    const std::string &robot_model,
    const double &Cartesian_thres, const double &convergence_thres,
    const int &max_initializations)
{
    return cfslib::math::IKine(
        theta0, mat_ref, robot_model, Cartesian_thres, convergence_thres, max_initializations
    );
}

// cfslib::math::Vector6d _IK_single_pt(
//     const cfslib::math::VectorJd &theta0, const Eigen::Matrix4d &mat_ref,
//     const std::string &robot_model,
//     const double &Cartesian_thres, const double &convergence_thres,
//     const int &max_initializations)
// {
//     Eigen::MatrixXd DH = get_DH(robot_model);

//     cfslib::math::Vector6d c_ref;
//     cfslib::math::TransMatToPoseAngRad(mat_ref, c_ref);

//     cfslib::math::Vector6d theta_guess, c_guess, theta_sol, c_sol;
//     cfslib::math::Vector6d c_err;

//     cfslib::math::Vector6d theta_optimal;
//     cfslib::math::Vector6d theta_optimal_prev;
//     for (int i = 0; i < 6; ++i)
//     {
//         theta_optimal(i) = 1e6;
//         theta_optimal_prev(i) = 1e6;
//     }
//     bool found_feasible = false;

//     int guess_max = max_initializations;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dist(-M_PI / 2.0, M_PI / 2.0);

//     for (int guess_i = 0; guess_i < guess_max; ++guess_i)
//     {
//         // std::cout << ">> ------------------------------ \n";
//         // std::cout << ">> Guess " << guess_i << "\n";

//         // randomize theta_guess
//         for (int i = 0; i < theta_guess.rows(); ++i)
//         {
//             theta_guess(i) = theta0(i) + dist(gen);
//         }
//         cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_guess, DH), c_guess);

//         // solve IK
//         theta_sol = _IK_single_pt(c_ref, c_guess, theta_guess, DH, Cartesian_thres, 50);

//         // check solution
//         cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_sol, DH), c_sol);
//         c_err = c_ref - c_sol;

//         if (c_err.norm() < Cartesian_thres && theta_sol(2) < M_PI_2 && (theta_sol - theta0).cwiseAbs().maxCoeff() < M_PI * 3.0 / 4.0)
//         {
//             // Cartesian accurate
//             // std::cout << "-------------------------------------------\n";
//             // std::cout << ">> Found feasible solution at guess " << guess_i << ".\n";
//             // std::cout << ">> c_ref: " << math::print_eigen_mat(c_ref.transpose()) << "\n";
//             // std::cout << ">> c_sol: " << math::print_eigen_mat(c_sol.transpose()) << "\n";
//             // std::cout << ">> c_err: " << math::print_eigen_mat(c_err.transpose()) << "\n";

//             if ((theta_sol - theta0).norm() < (theta_optimal - theta0).norm())
//             {

//                 // better solution
//                 found_feasible = true;

//                 // backup
//                 theta_optimal_prev = theta_optimal;

//                 // update
//                 theta_optimal = theta_sol;

//                 // std::cout << ">> Found better solution at guess " << guess_i << "\n";
//                 // std::cout << ">> theta_optimal:         " << cfslib::math::print_eigen_mat(theta_optimal.transpose()) << "\n";
//                 // std::cout << ">> theta_optimal_prev:    " << cfslib::math::print_eigen_mat(theta_optimal_prev.transpose()) << "\n";
//                 // std::cout << ">> optimal diff:          " << (theta_optimal - theta_optimal_prev).norm() << "\n";
//                 // std::cout << ">> diff with initial:     " << (theta_optimal - theta0).norm() << "\n";
//                 // std::cout << ">> max joint diff:        " << (theta_optimal - theta0).cwiseAbs().maxCoeff() << "\n";

//                 // check termination
//                 if ((theta_optimal - theta_optimal_prev).norm() < convergence_thres)
//                 {
//                     // std::cout << ">> Converged\n";
//                     break;
//                 }
//             }
//         }
//     }

//     std::cout << ">> IK error: " << c_err.norm() << ", convergence: " << (theta_optimal - theta_optimal_prev).norm() << "\n";

//     return theta_optimal;
// }

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, bool> _CFSPickPlace(
    const Eigen::MatrixXd &current_pose_joint,
    const Eigen::Matrix4d &mid_pose_cartesian,
    const Eigen::Matrix4d &target_pose_cartesian,
    const std::string &robot_model,
    const int &movement_steps,
    const std::string &robot_cap_path,
    const std::string &obs_path)
{

    int T_ms{4}; // robot pos control loop time ms
    double execution_time{0};

    // DH
    Eigen::MatrixXd DH = get_DH(robot_model);

    std::string config_path = XSTRING(SOURCE_ROOT) + "/config/user_config/user_config_default.txt";

    // base frame (same as world)
    Eigen::MatrixXd base_frame = Eigen::MatrixXd::Identity(4, 4);

    int n_joint{6};
    Eigen::MatrixXd joint_limits(n_joint, 2);
    joint_limits << -PI, PI,
        -PI, PI,
        -PI, PI,
        -PI, PI,
        -PI, PI,
        -2 * PI, 2 * PI;

    Eigen::MatrixXd joint_vel_limits(n_joint, 1);
    // joint_vel_limits << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    joint_vel_limits << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;

    Eigen::MatrixXd joint_acc_limits(n_joint, 1);
    joint_acc_limits << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    Eigen::MatrixXd joint_disp_limits(n_joint, 1);
    joint_disp_limits << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;

    double cart_vel_max = 1;
    double cart_acc_max = 0.5;

    Eigen::MatrixXi collision_avoidance_links(8, 1);
    collision_avoidance_links << 0, 0, 0, 0, 0, 0, 0, 1;

    /* --------------------------- set ref trajectory --------------------------- */
    Eigen::Matrix4d current_pose_cartesian = cfslib::math::FKine(current_pose_joint, DH);
    Eigen::Quaterniond x0_quat{current_pose_cartesian.block<3, 3>(0, 0)};

    cfslib::math::Vector6d x_0;
    cfslib::math::TransMatToPoseAngRad(current_pose_cartesian, x_0);
    cfslib::math::Vector6d x_mid;
    cfslib::math::TransMatToPoseAngRad(mid_pose_cartesian, x_mid);
    cfslib::math::Vector6d x_1;
    cfslib::math::TransMatToPoseAngRad(target_pose_cartesian, x_1);

    /* --------------------------- compose trajectory --------------------------- */
    Eigen::MatrixXd cart_waypoint_ref;
    Eigen::VectorXd joint_ref(0);

    execution_time = ((double)(movement_steps * T_ms)) / 1000.0;
    cart_waypoint_ref.resize(6, 3);
    cart_waypoint_ref.col(0) = x_0;
    cart_waypoint_ref.col(1) = x_mid;
    cart_waypoint_ref.col(2) = x_1;

    /* ------------------------------ call cfs core ----------------------------- */

    Eigen::MatrixXd CFS_traj;
    cfslib::query::UserQuery::Ptr query;
    cfslib::trajectory::Trajectory::Ptr traj_ptr;
    cfslib::query::Mode mode;
    mode.goal_type = cfslib::query::GoalType::CartWaypoints;
    mode.use_timestamp = true;
    mode.enable_tempopt = false;
    mode.dynamic_obstacle = false;
    mode.enforce_cartesian = cfslib::query::CartesianMode::None;

    cfslib::CFSLib solver{};

    query = std::make_shared<cfslib::query::UserQuery>(
        x_0, current_pose_joint, DH, cart_waypoint_ref, joint_ref,
        joint_limits, joint_vel_limits, joint_acc_limits, joint_disp_limits,
        base_frame, collision_avoidance_links, cart_vel_max, cart_acc_max,
        execution_time, mode, config_path, robot_cap_path, obs_path);
    auto start_time = std::chrono::high_resolution_clock::now();
    traj_ptr = solver.DemoSolve(query);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Computation time: " << duration.count() << " milliseconds\n";

    CFS_traj = traj_ptr->joint_path();
    bool success = traj_ptr->status() && traj_ptr->vel_flag() && traj_ptr->dense_flag();
    Eigen::MatrixXd critical_traj = traj_ptr->joint_path_critical();

    return std::make_tuple(CFS_traj, critical_traj, success);
}

bool joint_in_limits(const Eigen::MatrixXd &joint_traj, const double &joint_limit)
{
    // traj [6, N]
    for (int i = 0; i < joint_traj.rows(); ++i)
    {
        if (joint_traj.row(i).maxCoeff() > joint_limit || joint_traj.row(i).minCoeff() < -joint_limit)
        {
            return false;
        }
    }
    return true;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, bool> CFSPickPlace(
    const Eigen::MatrixXd &current_pose_joint,
    const Eigen::Matrix4d &mid_pose_cartesian,
    const Eigen::Matrix4d &target_pose_cartesian,
    const std::string &robot_model,
    const int &movement_steps,
    const double &joint_limit,
    const std::string &robot_cap_path,
    const std::string &obs_path)
{
    bool CFS_success{false};
    Eigen::MatrixXd CFS_traj, critical_traj;
    bool solved{false};

    // try to solve cfs
    int t = movement_steps;

    // for (int i=0; i<20; ++i)
    // {

    // CFS_success = false;

    std::tie(CFS_traj, critical_traj, CFS_success) = _CFSPickPlace(
        current_pose_joint, mid_pose_cartesian, target_pose_cartesian, robot_model, t, robot_cap_path, obs_path);

    // if (CFS_success && joint_in_limits(CFS_traj, joint_limit))
    // {
    //     break;
    // }
    // else
    // {
    //     std::cout << ">> CFS failed.\n";
    //     std::cout << ">> Within joint limit (fwd): " << joint_in_limits(CFS_traj, joint_limit) << "\n";
    //     std::cout << ">> critical: \n" << critical_traj << "\n";
    // }

    // t = floor(((double)t) * 1.5);

    // if (t > 1250) // no more than 5 seconds execution time
    // {
    //     break;
    // }

    // }

    solved = CFS_success && joint_in_limits(CFS_traj, joint_limit);

    return std::make_tuple(CFS_traj, critical_traj, solved);
}

PYBIND11_MODULE(cfspy, m)
{
    m.doc() = "CFS Library";
    m.def("CFSPickPlace", &CFSPickPlace, "CFS Pick and Place");
    m.def("FKine", &FKine, "Forward Kinematics");
    m.def("IKine", &IKine, "Inverse Kinematics");
}
