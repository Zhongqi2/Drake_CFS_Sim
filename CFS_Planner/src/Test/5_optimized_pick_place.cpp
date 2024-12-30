#include "cfslib/cfslib.hpp"
#include "cfslib/Highlevel/CFSPickPlace.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include "hardware_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <fstream>

#undef inverse

extern "C" {
    #include "extApi.h"
#include <simConst.h>
}

void track_pos(
    double joint_pos_fbk[],
    const Eigen::Matrix4d& goal_mat,
    const std::string& subroutine_mode,
    const int& movement_steps,
    const bool& use_offset,
    const Eigen::Vector3d& offset_vec, // base frame
    const Eigen::Vector3d& offset_tilt_axis, // flange frame
    const double& offset_tilt_angle, // rad
    HardwareInterface& robot_hw,
    const int& n_joint,
    const int& clientID,
    const simxInt joint_handles[],
    const bool& enable_robot,
    const bool& real_robot,
    const bool& robot_execute
)
{
    /* ----------------------------- get robot state ---------------------------- */
    // if (enable_robot)
    // {
    //     if (real_robot)
    //     {
    //         robot_hw.getJointPos(joint_pos_fbk);
    //     }
    //     else
    //     {
    //         // ! temporary hack
    //         // simxFloat J_pos[6]={1.0, 1.0};
    //         // for (int i=0;i<n_joint; i++)
    //         // {
    //         //     simxGetJointPosition(clientID, joint_handles[i], &J_pos[i], simx_opmode_buffer); 
    //         //     joint_pos_fbk[i] = J_pos[i];
    //         // }
    //     }
    // }

    // Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    // if (enable_robot)
    // {
    //     q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
    //     std::cout << ">> q0:\n" << q0 << "\n";
    // }
    // else
    // {
    //     q0 << -2.6, 31.3, 8.7, -2.2, 19.0, 2.3;
    //     q0 = q0 / 180.0 * M_PI;
    // }

    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];

    /* -------------------------------- plan CFS -------------------------------- */
    bool CFS_success{false};
    Eigen::MatrixXd CFS_traj, critical_traj;
    std::string robot_model = "ur5";
    std::tie(CFS_traj, critical_traj, CFS_success) = CFSPickPlace(
        q0, goal_mat, goal_mat, robot_model, movement_steps, 0.5, "", ""
    );
    std::cout << ">> critical_traj:\n" << critical_traj << "\n";

    /* ------------------------------ write to file ----------------------------- */
    // std::ofstream file("optimized_traj.txt");
    // if (file.is_open())
    // {
    //     for (int i=0; i<CFS_traj.rows(); ++i)
    //     {
    //         for (int j=0; j<CFS_traj.cols(); ++j)
    //         {
    //             file << CFS_traj(i, j) << " ";
    //         }
    //         file << "\n";
    //     }
    //     file.close();
    // }
    // else
    // {
    //     std::cerr << "Unable to open file\n";
    // }

    // std::ofstream file_critical("critical_traj.txt");
    // if (file_critical.is_open())
    // {
    //     for (int i=0; i<critical_traj.rows(); ++i)
    //     {
    //         for (int j=0; j<critical_traj.cols(); ++j)
    //         {
    //             file_critical << critical_traj(i, j) << " ";
    //         }
    //         file_critical << "\n";
    //     }
    //     file_critical.close();
    // }
    // else
    // {
    //     std::cerr << "Unable to open file\n";
    // }

    /* --------------------------- Send for execution --------------------------- */
    if (CFS_success)
    {
        if (enable_robot && robot_execute)
        {
            double joint_pos_des[n_joint] = {0};
            for (int t=0; t<CFS_traj.cols(); ++t)
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
    }

    return;
}

int main(int argc, char **argv)
{
    try
    {
        /* ------------------------------ global config ----------------------------- */
        bool enable_robot{true};
        bool real_robot{false};
        bool robot_execute{true};
        
        float movement_calib{-0.05};
        float movement_recover{0.1};

        float force_thres{10.0};

        /* ------------------------------- vrep setup ------------------------------- */
        int clientID=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
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

            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_1",&joint_handles[0],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_2",&joint_handles[1],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_3",&joint_handles[2],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_4",&joint_handles[3],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_5",&joint_handles[4],simx_opmode_blocking);
            ret = simxGetObjectHandle(clientID,(simxChar*)"joint_6",&joint_handles[5],simx_opmode_blocking);

            for (int i=0;i<JOINTS_NUM; i++)
            {
                simxGetJointPosition(clientID, joint_handles[i], &J_pos[i], simx_opmode_buffer); 
                simxGetObjectFloatParameter(clientID, joint_handles[i], 2012, &J_vel[i], simx_opmode_buffer);
                std::cout << J_pos[i] << std::endl;
            }
            simxInt inittime=simxGetLastCmdTime(clientID);

            // set to default position
            // double joint_pos_des[JOINTS_NUM] = {0, 0, M_PI_2, 0, -M_PI_2, 0};
            // double joint_pos_des[JOINTS_NUM] = {0.63, 0.35, M_PI_2, -0.35, -M_PI_2, 0.52};
            // double joint_pos_des[JOINTS_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // for (int t=0; t<100; ++t)
            // {
            //     // 4ms loop in sim
            //     for (int i=0;i<JOINTS_NUM; i++)
            //     {
            //         simxSetJointTargetPosition(clientID, joint_handles[i], joint_pos_des[i], simx_opmode_oneshot);
            //     }
            //     simxSynchronousTrigger(clientID);
            // }

            // usleep(5000000);

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
        // if (clientID!=-1)
        // {
        //     robot_hw.getJointPos(joint_pos_fbk);
        //     std::cout << joint_pos_fbk[0] << std::endl;
        //     std::cout << joint_pos_fbk[1] << std::endl;
        //     std::cout << joint_pos_fbk[2] << std::endl;
        //     std::cout << joint_pos_fbk[3] << std::endl;
        //     std::cout << joint_pos_fbk[4] << std::endl;
        //     std::cout << joint_pos_fbk[5] << std::endl;
        //     usleep(10000);
        //     for (int i=0;i<JOINT_NUM; i++)
        //     {
        //         simxSetJointTargetPosition(clientID, joint_handles[i], joint_pos_fbk[i], simx_opmode_oneshot);
        //     }
        //     simxSynchronousTrigger(clientID);
        // }

        
        std::cout << std::setprecision(10);

        Eigen::Matrix4d mat_flange2tool = cfslib::io::ReadMat4d("config/tool_calibration/tool_cal_mat_ur5.txt");
        std::cout << ">> mat_flange2tool:\n" << mat_flange2tool << "\n";

        /* ------------------------------ home to pick ------------------------------ */
        double joint_pos_home[n_joint] = {0.0, 0.0, M_PI_2, 0.0, -M_PI_2, 0.};
        Eigen::Matrix4d pose_pick;
        pose_pick << -0.1097783008, 0.993956098, 4.210844825e-17, -0.4276628141,
                        0.993956098, 0.1097783008, -8.555212719e-17, -0.4468972917,
                    -1.091199431e-16, 5.588530973e-17, -1, 0.271590738,
                                0, 0, 0, 1;

        bool use_offset{false};
        Eigen::Vector3d offset_vec = Eigen::Vector3d::Zero(); // base frame
        Eigen::Vector3d offset_tilt_axis = Eigen::Vector3d::Zero(); // flange frame
        double offset_tilt_angle{0.0}; // rad

        track_pos(joint_pos_home, pose_pick, "home_to_pick", 200, use_offset, offset_vec, offset_tilt_axis, offset_tilt_angle,
                    robot_hw, n_joint, clientID, joint_handles, enable_robot, real_robot, robot_execute);

        /* ------------------------------ pick to place ----------------------------- */
        double joint_pos_pick[n_joint] = {0.63, 0.35, M_PI_2, -0.35, -M_PI_2, 0.52};
        Eigen::Matrix4d mat_base2place;
        mat_base2place << 1.0, 0.0, 0.0, -0.1,
                         0.0, -1.0, 0.0, 0.8,
                         0.0, 0.0, -1.0, 0.001,
                         0.0, 0.0, 0.0, 1.0;

        use_offset = true;
        offset_vec << 0.1, 0.0, 0.1; // base frame
        offset_tilt_axis << 0.0, 1.0, 0.0; // flange frame
        offset_tilt_angle = 1.0; // rad

        Eigen::Matrix4d pose_place = mat_base2place * mat_flange2tool.inverse();
        track_pos(joint_pos_pick, pose_place, "pick_to_place", 1000, use_offset, offset_vec, offset_tilt_axis, offset_tilt_angle,
                    robot_hw, n_joint, clientID, joint_handles, enable_robot, real_robot, robot_execute);

        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}