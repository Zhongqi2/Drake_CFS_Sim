#include "cfslib/cfslib.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <unistd.h>
#include "hardware_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "realsense/realsense.h"
#include "detector/chess_board_detector.h"
#include "detector/pnp_problem.h"

#undef inverse

extern "C" {
    #include "extApi.h"
}

void track_pos(
    const Eigen::Matrix4d& goal_mat,
    HardwareInterface& robot_hw,
    const int& n_joint,
    const Eigen::MatrixXd& DH,
    const int& clientID,
    const simxInt joint_handles[],
    const int& movement_steps,
    const Eigen::MatrixXd& base_frame,
    const int& execution_time,
    const std::string& config_fname,
    const std::string& robot_cap_fname,
    const std::string& obs_fname,
    const bool& enable_robot,
    const bool& real_robot,
    const bool& robot_execute
)
{
    Eigen::MatrixXd joint_limits(n_joint, 2);
    joint_limits << -PI/2, PI/2, 
                    -PI/2, PI/2, 
                    -PI/2, PI/2, 
                    -PI, PI, 
                    -PI, PI, 
                    -PI, PI;

    Eigen::MatrixXd joint_vel_limits(n_joint, 1);
    joint_vel_limits << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    
    Eigen::MatrixXd joint_acc_limits(n_joint, 1);
    joint_acc_limits << 0.6, 0.6, 0.6, 0.6, 0.6, 0.6;

    Eigen::MatrixXd joint_disp_limits(n_joint, 1);
    joint_disp_limits << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
    
    double cart_vel_max = 1;
    double cart_acc_max = 0.5;

    Eigen::MatrixXi collision_avoidance_links(8, 1);
    collision_avoidance_links << 1, 1, 1, 1, 1, 1, 1, 1;

    /* ----------------------------- get robot state ---------------------------- */
    double joint_pos_fbk[n_joint] = {0};
    if (enable_robot)
    {
        if (real_robot)
        {
            robot_hw.getJointPos(joint_pos_fbk);
        }
        else
        {
            simxFloat J_pos[6]={0.0};
            for (int i=0;i<n_joint; i++)
            {
                simxGetJointPosition(clientID, joint_handles[i], &J_pos[i], simx_opmode_buffer); 
                joint_pos_fbk[i] = J_pos[i];
            }
        }
    }

    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    if (enable_robot)
    {
        q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
        std::cout << ">> q0:\n" << q0 << "\n";
    }
    else
    {
        q0 << -2.6, 31.3, 8.7, -2.2, 19.0, 2.3;
        q0 = q0 / 180.0 * M_PI;
    }
    // q0 << 0.185927, 0.520814, 0.142393, -0.432303, 0.468512, 0.915695;
    // q0 << -2.6, 31.3, 8.7, -2.2, 19.0, 2.3;
    // q0 = q0 / 180.0 * M_PI;
    cfslib::math::Vector6d x0;
    Eigen::Matrix4d x0_mat = cfslib::math::FKine(q0, DH);
    cfslib::math::TransMatToPoseAngRad(x0_mat, x0);
    cfslib::math::Vector6d x_init = x0;
    Eigen::Quaterniond x0_quat{ x0_mat.block<3, 3>(0, 0) };

    /* --------------------------- set ref trajectory --------------------------- */
    cfslib::math::Vector6d x1;
    cfslib::math::TransMatToPoseAngRad(goal_mat, x1);

    cfslib::math::Vector6d dx = (x1-x0)/(movement_steps-1);

    Eigen::MatrixXd cart_waypoint_ref;
    Eigen::VectorXd joint_ref(0);
    cart_waypoint_ref.resize(6, movement_steps);
    cart_waypoint_ref.col(0) = x0;
    for (int traj_i=1; traj_i < movement_steps; ++traj_i)
    {
        cart_waypoint_ref.col(traj_i) = x0 + traj_i * dx;
    }

    #ifdef DEBUG_PRINT
    std::cout << ">> input cart waypoint:\n" << cart_waypoint_ref.transpose() << "\n";

    std::cout << ">> q0:\n" << q0 << "\n";
    std::cout << ">> x0:\n" << x0 << "\n";
    std::cout << ">> x0_mat:\n" << x0_mat << "\n";
    std::cout << ">> x0_quat:\n" << x0_quat.w() << ", " << x0_quat.x() << ", "
                                << x0_quat.y() << ", " << x0_quat.z() << "\n";
    #endif

    // Eigen::MatrixXd cart_traj(0,0), joint_traj(0,0), dist_profile(0,0), dist_init_profile(0,0), cart_input_ref(0, 0), joint_input_ref(0, 0);
    // Eigen::MatrixXd cart_vel(0, 0), cart_vel_ref(0, 0), cart_acc(0, 0), cart_acc_ref(0, 0);
    // Eigen::MatrixXd joint_vel(0, 0), joint_vel_ref(0, 0), joint_acc(0, 0), joint_acc_ref(0, 0);
    Eigen::MatrixXd CFS_traj;
    // std::vector<double> traj_len;

    cfslib::query::UserQuery::Ptr query;
    cfslib::trajectory::Trajectory::Ptr traj_ptr;
    cfslib::query::Mode mode;
    mode.goal_type         = cfslib::query::GoalType::CartPath;
    mode.use_timestamp     = true;
    mode.enable_tempopt    = false;
    mode.dynamic_obstacle  = false;
    mode.enforce_cartesian = cfslib::query::CartesianMode::None;

    cfslib::CFSLib solver{};

    query = std::make_shared<cfslib::query::UserQuery>(
        x0, q0, DH, cart_waypoint_ref, joint_ref,
        joint_limits, joint_vel_limits, joint_acc_limits, joint_disp_limits,
        base_frame, collision_avoidance_links, cart_vel_max, cart_acc_max,
        execution_time, mode, config_fname, robot_cap_fname, obs_fname
    );
    traj_ptr = solver.DemoSolve(query);
    CFS_traj = traj_ptr->joint_path();

    #ifdef DEBUG_PRINT
    // q0 = CFS_traj.rightCols(1);
    // x0 = cart_waypoint_ref.col(0);
    std::cout << "CFS_traj:\n" << CFS_traj.transpose() << "\n";
    std::cout<<"Cart reference:\n"<<traj_ptr->cart_init_ref().transpose()<<"\n";
    std::cout<<"Cart path:\n"<<traj_ptr->cart_path().transpose()<<"\n";
    #endif

    /* --------------------------- Send for execution --------------------------- */
    if (traj_ptr->status() && traj_ptr->vel_flag() && traj_ptr->dense_flag())
    {
        if (enable_robot && robot_execute)
        {
            double joint_pos_des[n_joint] = {0};
            for (int t=0; t<movement_steps; ++t)
            {
                // send cmd
                for (int joint_i=0; joint_i<6; ++joint_i)
                {
                    joint_pos_des[joint_i] = CFS_traj(joint_i, t);
                }

                if (real_robot)
                {
                    robot_hw.getJointPos(joint_pos_fbk);
                    robot_hw.setJointPosCmd(joint_pos_des);
                }

                if (clientID != -1)
                {
                    for (int i=0;i<n_joint; i++)
                    {
                        simxSetJointTargetPosition(clientID, joint_handles[i], joint_pos_des[i], simx_opmode_oneshot);
                    }
                    simxSynchronousTrigger(clientID);
                }

                // send to vrep
                if (t % 10 == 0)
                {
                    // send
                }

                usleep(4000);
            }

            // extra cmd to ensure reaching
            for (int i=0; i<100; ++i)
            {
                if (real_robot)
                {
                    robot_hw.getJointPos(joint_pos_fbk);
                    robot_hw.setJointPosCmd(joint_pos_des);
                }
                usleep(4000);
            }
        }
    }
    else
    {
        std::cerr << "Planning flags failed...\n";
        std::cerr << "CFS       OK: "  << traj_ptr->status() << "\n";
        std::cerr << "Velocity  OK: "  << traj_ptr->vel_flag() << "\n";
        std::cerr << "Density   OK: " << traj_ptr->dense_flag() << "\n";
    }
}

int main(int argc, char **argv)
{
    try
    {
        /* ------------------------------ global config ----------------------------- */
        int T_ms{40}; // robot pos control loop time ms
        // bool enable_perception{false};
        bool enable_robot{true};
        bool real_robot{true};
        bool robot_execute{true};
        
        float movement_calib{-0.05};
        float movement_recover{0.1};
        int movement_steps{1001};
        int execution_time = (int)((movement_steps-1)*T_ms/1000);

        float force_thres{10.0};

        /* ------------------------------- vrep setup ------------------------------- */
        int clientID=simxStart((simxChar*)"127.0.0.1",18274,true,true,2000,5);
        simxInt joint_handles[6] = {0,0,0,0,0,0};
        if (clientID!=-1)
        {
            printf("Connected to remote API server\n");

            // Now try to retrieve data in a blocking fashion (i.e. a service call):
            int objectCount;
            int* objectHandles;
            simxInt ret=simxGetObjects(clientID,sim_handle_all,&objectCount,&objectHandles,simx_opmode_blocking);
            if (ret==simx_return_ok)
                printf("Number of objects in the scene: %d\n",objectCount);
            else
                printf("Remote API function call returned with error code: %d\n",ret);


            // 1-simulaiton parameters setting
            double dt=0.050;
            simxSetFloatingParameter(clientID,sim_floatparam_simulation_time_step,dt,simx_opmode_oneshot_wait);
            simxSynchronous(clientID,true);
            // start the simulation:
            simxStartSimulation(clientID,simx_opmode_oneshot_wait);

            // 2-initialization
            simxSynchronousTrigger(clientID);
            
            int JOINTS_NUM=6;
            simxFloat J_pos[6]={0.0};
            simxFloat J_vel[6]={0.0};

            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_11",&joint_handles[0],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_21",&joint_handles[1],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_31",&joint_handles[2],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_41",&joint_handles[3],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_51",&joint_handles[4],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_61",&joint_handles[5],simx_opmode_blocking);

            for (int i=0;i<JOINTS_NUM; i++)
            {
            simxGetJointPosition(clientID, joint_handles[i], &J_pos[i], simx_opmode_buffer); 
            simxGetObjectFloatParameter(clientID, joint_handles[i], 2012, &J_vel[i], simx_opmode_buffer);
            }
            simxInt inittime=simxGetLastCmdTime(clientID);

            // 3-precomputed trajectory and torques

            // 4- pd controller parameters
            int amplify_factor=1;
            simxFloat weights[7] = {0.3, 0.8, 0.6, 0.6, 0.2, 0.1, 0.1};
            simxFloat kp[7],kd[7];
            simxFloat q_des[6] = {0};

            for (int i=0;i<JOINTS_NUM; i++)
            {
                kp[i]=100 * weights[i] * amplify_factor;
                kd[i] = 2 * weights[i] * amplify_factor;
            }
        }
        else
        {
            printf("Cannot connect to Vrep Simulator. Skipping digital twin.\n");
        }

        /* ------------------------------- robot setup ------------------------------ */
        int n_joint{6};
        HardwareInterface robot_hw;
        double joint_pos_fbk[n_joint] = {0};
        int32_t joint_trq_fbk[n_joint] = {0};

        // sync sim with real
        if (clientID!=-1)
        {
            robot_hw.getJointPos(joint_pos_fbk);
            std::cout << joint_pos_fbk[0] << std::endl;
            std::cout << joint_pos_fbk[1] << std::endl;
            std::cout << joint_pos_fbk[2] << std::endl;
            std::cout << joint_pos_fbk[3] << std::endl;
            std::cout << joint_pos_fbk[4] << std::endl;
            std::cout << joint_pos_fbk[5] << std::endl;
            usleep(10000);
            for (int i=0;i<JOINT_NUM; i++)
            {
                simxSetJointTargetPosition(clientID, joint_handles[i], joint_pos_fbk[i], simx_opmode_oneshot);
            }
            simxSynchronousTrigger(clientID);
        }

        // DH
        std::string dh_fname{"config/robot_properties/robot_DH_gp50.txt"};
        Eigen::MatrixXd DH = Eigen::MatrixXd::Zero(1, 4); // base to joint 1
        DH = cfslib::math::EigenVcat(DH, cfslib::io::LoadMatFromFile(dh_fname)); // base to ee
        std::cout << std::setprecision(10);

        // robot capsule
        std::string robot_cap_fname = "config/geometry_wrappers/robot_capsules_gp50.txt";

        // base frame (same as world)
        Eigen::MatrixXd base_frame = Eigen::MatrixXd::Identity(4, 4);

        Eigen::Matrix4d mat_flange2tool;
        mat_flange2tool << 0, -1, 0, 0,
                           -1, 0, 0, 0,
                            0, 0, -1, -0.10,
                            0, 0, 0, 1;
        std::cout << ">> mat_flange2tool:\n" << mat_flange2tool << "\n";

        /* ------------------------------ obstacle info ----------------------------- */
        // ! update
        std::string obs_fname = "config/geometry_wrappers/CFSLin_test/obs_capsules.txt";

        /* ------------------------------ solver config ----------------------------- */
        std::string config_fname = "config/user_config/user_config_default.txt";

        /* ------------------------------ detect marker ----------------------------- */
        RealSense cam;
        cam.getIntrinsics();
        // std::thread t1(display_camera_stream);
        cout << "camera connected..." << endl;
        std::string intrinsic_param_file = "/home/icl/Documents/MASCEI_root/MASCEI/Auto_calibration/config/cam_calibration/845112072047/cv_calib_realsense.yaml";
        PnPProblem pnp_solver(intrinsic_param_file.c_str()); 
        ChessBoardDetector checker_board_detector(pnp_solver);

        bool detected{false};

        cv::Mat rgb_img, gray_img, depth_img, viz_img;
        cv::Mat pattern_rotation, pattern_translation;
        cam.capture(rgb_img, depth_img);
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
        detected = checker_board_detector.poseEstimationFromCoplanarPoints(gray_img, cv::Size(7, 8), 10.0,
                                                                intrinsic_param_file.c_str(), 
                                                                pattern_rotation,
                                                                pattern_translation);
        if (!detected)
        {
            std::cout << "Failed to detect marker, exiting.\n";
            std::cout << "\n---------------------------------------------\n\n\n";
            return 0;
        }
        pattern_translation /= 1000.0;

        Eigen::Matrix3d R_CT;
        Eigen::Vector3d p_CT;

        cv::cv2eigen(pattern_rotation, R_CT);
        cv::cv2eigen(pattern_translation, p_CT);

        Eigen::Matrix4d mat_cam2marker;
        mat_cam2marker << R_CT, p_CT, 0, 0, 0, 1;

        std::cout << ">> mat_cam2marker:\n" << mat_cam2marker << "\n";

        Eigen::Matrix4d mat_base2cam;
        mat_base2cam << -0.03509235188393173, 0.2731988260037135, -0.9613172880529338, 1.851151415928018,
                        0.9791951849675118, -0.1829695848586838, -0.0877434940783734, -1.427842147272113,
                        -0.1998632446841963, -0.9443963852571831, -0.2610941419056957, 1.107960096378761,
                        0, 0, 0, 1;

        Eigen::Matrix4d pose_test_toch = mat_base2cam * mat_cam2marker * mat_flange2tool.inverse();

        std::cout << ">> pose_test_toch:\n" << pose_test_toch << "\n";

        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>> Moving robot... <<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        track_pos(pose_test_toch,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);

        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

