#include "cfslib/cfslib.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include "hardware_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "epick_driver/default_driver.hpp"
#include "epick_driver/default_serial.hpp"
#include "epick_driver/default_driver_utils.hpp"
#include "epick_driver/epick_gripper.hpp"

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
        int movement_steps{701};
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

        Eigen::Matrix4d mat_flange2tool = cfslib::io::ReadMat4d("config/tool_calibration/tool_cal_mat.txt");
        std::cout << ">> mat_flange2tool:\n" << mat_flange2tool << "\n";

        /* -------------------------------- goal info ------------------------------- */
        // ! use scene calibration result
        Eigen::Matrix4d mat_base2table;
        mat_base2table << 1, 0, 0, 0,
                          0, 1, 0, -1.2,
                          0, 0, 1, 0.7,
                          0, 0, 0, 1;
        
        Eigen::Matrix4d mat_table2pick;
        mat_table2pick << 0, 1, 0, 0.3,
                          1, 0, 0, 0,
                          0, 0, -1, 0.11,
                          0, 0, 0, 1;

        Eigen::Matrix4d mat_table2prepick = mat_table2pick;
        mat_table2prepick(2, 3) += 0.3;

        Eigen::Matrix4d mat_table2place;
        mat_table2place << 0, 1, 0, -0.3,
                           1, 0, 0, 0,
                           0, 0, -1, 0.11,
                           0, 0, 0, 1;
        
        Eigen::Matrix4d mat_table2preplace = mat_table2place;
        mat_table2preplace(2, 3) += 0.3;

        /* ------------------------------ obstacle info ----------------------------- */
        // ! update
        std::string obs_fname = "config/geometry_wrappers/CFSLin_test/obs_capsules.txt";

        /* ------------------------------ solver config ----------------------------- */
        std::string config_fname = "config/user_config/user_config_default.txt";

        Eigen::Matrix4d pose_prepick = mat_base2table * mat_table2prepick * mat_flange2tool.inverse();
        Eigen::Matrix4d pose_pick = mat_base2table * mat_table2pick * mat_flange2tool.inverse();
        Eigen::Matrix4d pose_preplace = mat_base2table * mat_table2preplace * mat_flange2tool.inverse();
        Eigen::Matrix4d pose_place = mat_base2table * mat_table2place * mat_flange2tool.inverse();

        gripper_activate();

        // prepick
        track_pos(pose_prepick,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);

        // pick
        track_pos(pose_pick,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);

        gripper_grip1();

        // prepick
        track_pos(pose_prepick,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);


        // preplace
        track_pos(pose_preplace,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);

        // place
        track_pos(pose_place,
            robot_hw, n_joint, DH, clientID, joint_handles,
            movement_steps, base_frame, execution_time, config_fname, robot_cap_fname,
            obs_fname, enable_robot, real_robot, robot_execute);
        
        gripper_release();
        
        // preplace
        track_pos(pose_preplace,
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