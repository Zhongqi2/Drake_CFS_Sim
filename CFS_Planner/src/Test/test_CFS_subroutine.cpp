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

int main(int argc, char **argv)
{
    try
    {
        
        int n_joint{6};
        bool CFS_success{false};
        Eigen::MatrixXd CFS_traj, critical_traj;
        
        /* -------------------------------------------------------------------------- */
        /*      to be replaced by robot API (e.g., Yaskawa read/write functions)      */
        /* -------------------------------------------------------------------------- */

        // replace with actual code to get pick pose
        Eigen::Matrix4d pose_pick;
        pose_pick << -0.1097783008, 0.993956098, 4.210844825e-17, -0.4276628141,
                        0.993956098, 0.1097783008, -8.555212719e-17, -0.4468972917,
                    -1.091199431e-16, 5.588530973e-17, -1, 0.271590738,
                                0, 0, 0, 1;
        
        // replace with actual code to get home joint state
        Eigen::MatrixXd joint_pos_home_Eigen = Eigen::MatrixXd::Zero(n_joint, 1);
        joint_pos_home_Eigen << 0.0, 0.0, M_PI_2, 0.0, -M_PI_2, 0.;

        // replace with actual code to get place pose
        Eigen::Matrix4d pose_place;
        pose_place << 1,  0,  0,  -0.1,
                      0, -1,  0,   0.5,
                      0,  0, -1, 0.301,
                      0,  0,  0,     1;


        // replace with actual code to get pick joint state
        Eigen::MatrixXd joint_pos_pick_Eigen = Eigen::MatrixXd::Zero(n_joint, 1);
        joint_pos_pick_Eigen << 0.63, 0.35, M_PI_2, -0.35, -M_PI_2, 0.52;

        /* -------------------------------------------------------------------------- */
        /*                          replace with actual robot                         */
        /* -------------------------------------------------------------------------- */
        std::string robot_model = "ur5";

        /* -------------------------------------------------------------------------- */
        /*                            CFS Subroutine Usage                            */
        /* -------------------------------------------------------------------------- */

        // home to pick
        std::tie(CFS_traj, critical_traj, CFS_success) = CFSPickPlace(
            joint_pos_home_Eigen, pose_pick, pose_pick, robot_model, 200, M_PI, "", ""
        );
        std::cout << ">> CFS success: " << CFS_success << "\n";
        std::cout << ">> Three point trajectory:\n" << critical_traj << "\n";

        // pick to place
        Eigen::Vector3d offset_vec = Eigen::Vector3d::Zero(); // base frame
        Eigen::Vector3d offset_tilt_axis = Eigen::Vector3d::Zero(); // flange frame
        double offset_tilt_angle{0.0}; // rad
        offset_vec << 0.1, 0.0, 0.1; // base frame
        offset_tilt_axis << 0.0, 1.0, 0.0; // flange frame
        offset_tilt_angle = 1.0; // rad
        std::tie(CFS_traj, critical_traj, CFS_success) = CFSPickPlace(
            joint_pos_pick_Eigen, pose_place, pose_place, robot_model, 1000, M_PI, "", ""
        );
        std::cout << ">> CFS success: " << CFS_success << "\n";
        std::cout << ">> Five point trajectory:\n" << critical_traj << "\n";

        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}