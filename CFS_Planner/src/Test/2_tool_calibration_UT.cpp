#include "cfslib/cfslib.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <unistd.h>
#include <cstdlib>
#include "hardware_interface.h"
#include "realsense/realsense.h"
#include "detector/chess_board_detector.h"
#include "detector/pnp_problem.h"
#include <thread>
#include <mutex>
#include <signal.h>
#undef inverse
constexpr double kEpsilon = 1e-3;
bool CompareMatrices(const Eigen::Matrix4d& mat1, const Eigen::Matrix4d& mat2, double epsilon = kEpsilon) {
    for (int i = 0; i < mat1.rows(); ++i) {
        for (int j = 0; j < mat1.cols(); ++j) {
            if (std::fabs(mat1(i, j) - mat2(i, j)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}
int main(int argc, char **argv)
{

    /* ----------------------------- Find robot pose ---------------------------- */
    int n_joint{6};
    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    std::string dh_fname{"../config/robot_properties/robot_DH_gp7.txt"};
    Eigen::MatrixXd DH = Eigen::MatrixXd::Zero(1, 4); // base to joint 1
    DH = cfslib::math::EigenVcat(DH, cfslib::io::LoadMatFromFile(dh_fname)); // base to ee
    double joint_pos_fbk[n_joint] = {0};    
    q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
    Eigen::Matrix4d mat_base2flange = cfslib::math::FKine(q0, DH);
    std::cout << ">> mat_base2flange:\n" << mat_base2flange << "\n";

    /* ----------------------- Find marker in camera frame ---------------------- */
    cv::Mat pattern_rotation, pattern_translation;
    cv::Mat desired_pattern_rotation, desired_pattern_translation;
    cv::Mat rgb_img, gray_img;
    Eigen::Matrix3d R_CT;
    Eigen::Vector3d p_CT;
    std::string image_path = "../data/pnp_cali/rgb_image.png";
    std::string intrinsic_param_file = "../config/cam_calibration/845112072047/cv_calib_realsense.yaml";
    PnPProblem pnp_solver(intrinsic_param_file.c_str()); 
    ChessBoardDetector checker_board_detector(pnp_solver);
    rgb_img = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    checker_board_detector.poseEstimationFromCoplanarPoints(gray_img, cv::Size(7, 8), 10.0,
                                                            intrinsic_param_file.c_str(), 
                                                            pattern_rotation,
                                                            pattern_translation);
    cv::cv2eigen(pattern_rotation, R_CT);
    cv::cv2eigen(pattern_translation, p_CT);
    Eigen::Matrix4d mat_cam2marker;
    mat_cam2marker << R_CT, p_CT, 0, 0, 0, 1;
    std::cout << ">> mat_cam2marker:\n" << mat_cam2marker << "\n";

    /* -------------------- Read hand eye calibration result -------------------- */
    Eigen::Matrix4d mat_base2cam = cfslib::io::ReadMat4d("../config/cam_calibration/845112072047/hand_to_eye_calib_mat.txt");
    std::cout << ">> mat_base2cam:\n" << mat_base2cam << "\n";

    /* -------------------- Calculate tool calibration result ------------------- */

    // use this for two-finger gripper
    Eigen::Matrix4d mat_marker2tool_2finger;
    mat_marker2tool_2finger << 1, 0, 0, 0.133,
                                0, 1, 0, 0.0,
                                0, 0, 1, 0.0,
                                0, 0, 0, 1.0;

    // use this for suction gripper
    Eigen::Matrix4d mat_marker2tool_suction;
    mat_marker2tool_suction << cos(45.0/180.0*M_PI), -sin(45.0/180.0*M_PI), 0, 0.074,
                                sin(45.0/180.0*M_PI), cos(45.0/180.0*M_PI), 0, 0.0114,
                                0, 0, 1, 0.003,
                                0, 0, 0, 1.0;

    Eigen::Matrix4d mat_marker2tool;
    mat_marker2tool_2finger = mat_marker2tool_2finger;
    mat_marker2tool_suction = mat_marker2tool_suction;

    std::cout << ">> mat_marker2tool_2finger:\n" << mat_marker2tool_2finger << "\n";
    std::cout << ">> mat_marker2tool_suction:\n" << mat_marker2tool_suction << "\n";
        
    Eigen::Matrix4d mat_flange2tool_2finger = mat_base2flange.inverse() * mat_base2cam * mat_cam2marker * mat_marker2tool_2finger;
    Eigen::Matrix4d mat_flange2tool_suction = mat_base2flange.inverse() * mat_base2cam * mat_cam2marker * mat_marker2tool_suction;

    std::cout << ">> mat_flange2tool_2finger:\n" << mat_flange2tool_2finger << "\n";
    std::cout << ">> mat_flange2tool_suction:\n" << mat_flange2tool_suction << "\n";

    Eigen::Matrix4d desired_2finger_result = cfslib::io::ReadMat4d("../data/tool_cali/tool_cali_2finger_test.txt");
    Eigen::Matrix4d desired_suction_result = cfslib::io::ReadMat4d("../data/tool_cali/tool_cali_suction_test.txt");

    if (CompareMatrices(mat_flange2tool_2finger, desired_2finger_result)&&CompareMatrices(mat_flange2tool_suction, desired_suction_result)) {
        std::cout << "Unit test passed" << std::endl;
    } else {
        std::cout << "Unit test failed" << std::endl;
    }
    return 0;
}