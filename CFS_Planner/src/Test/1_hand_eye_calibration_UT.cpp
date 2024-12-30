#include "cfslib/cfslib.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <cstdlib>
#include "hardware_interface.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include "realsense/realsense.h"
#include "detector/chess_board_detector.h"
#include "detector/pnp_problem.h"

std::vector<cv::Mat> pattern_rot_list;
std::vector<cv::Mat> pattern_trans_list;
std::vector<cv::Mat> eef_rot_list;
std::vector<cv::Mat> eef_trans_list;

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
    std::string path = "/home/MASCEI/Auto_calibration/DecData/robot2calibration/";

    cv::Mat pattern_rotation, pattern_translation;
    cv::Mat rgb_img, gray_img;
    // loop over .png files in path 
    // for(int i=0; i<25; ++i)
    // {
    //     std::string image_path = path + "/" + std::to_string(i) + ".png";
    //     std::string intrinsic_param_file = "../config/cam_calibration/henderson/cv_calib_realsense.yaml";
    //     PnPProblem pnp_solver(intrinsic_param_file.c_str()); 
    //     ChessBoardDetector checker_board_detector(pnp_solver);
    //     rgb_img = cv::imread(image_path, cv::IMREAD_COLOR);
    //     cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    //     checker_board_detector.poseEstimationFromCoplanarPoints(gray_img, cv::Size(7, 8), 10.0,
    //                                                         intrinsic_param_file.c_str(), 
    //                                                         pattern_rotation,
    //                                                         pattern_translation);
    //     // combine pattern_rotation and pattern_translation
    //     Eigen::Matrix3f rot;
    //     Eigen::Vector3f trans;
    //     cv::cv2eigen(pattern_rotation, rot);
    //     cv::cv2eigen(pattern_translation, trans);
    //     Eigen::Matrix4d T_CT = Eigen::Matrix4d::Identity();
    //     T_CT.topLeftCorner<3,3>() = rot.cast<double>();
    //     T_CT.topRightCorner<3,1>() = trans.cast<double>() * 1.0e-3;
    //     // write to file
    //     cfslib::io::WriteMat4d(path + "c2t_" + std::to_string(i) + ".txt", T_CT);
    // }
                                         
    for(int i=0; i<25; ++i)
    {
        Eigen::Matrix4d T_CT = cfslib::io::ReadMat4d(path+"c2t_"+std::to_string(i) + ".txt");
        Eigen::Matrix4d T_GB = cfslib::io::ReadMat4d(path+"g2b_"+std::to_string(i) + ".txt");
        
        // Eigen::Matrix4d T_BG = T_GB.inverse();

        Eigen::Matrix3d R_CT = T_CT.block<3,3>(0,0);
        Eigen::Vector3d p_CT = T_CT.block<3,1>(0,3);
        
        Eigen::Matrix3d R_GB = T_GB.block<3,3>(0,0);
        Eigen::Vector3d p_GB = T_GB.block<3,1>(0,3);

        std::cout << T_CT << std::endl;
        std::cout << R_CT << std::endl;
        std::cout << p_CT << std::endl;

        std::cout << T_GB << std::endl;
        std::cout << R_GB << std::endl;
        std::cout << p_GB << std::endl;

        cv::Mat curr_CT_rot_mat, curr_CT_trans_mat;

        cv::eigen2cv(R_CT, curr_CT_rot_mat);
        cv::eigen2cv(p_CT, curr_CT_trans_mat);

        cv::Mat curr_GB_rot_mat, curr_GB_trans_mat;

        cv::eigen2cv(R_GB, curr_GB_rot_mat);
        cv::eigen2cv(p_GB, curr_GB_trans_mat);

        eef_rot_list.push_back(curr_GB_rot_mat);
        eef_trans_list.push_back(curr_GB_trans_mat);

        pattern_rot_list.push_back(curr_CT_rot_mat);
        pattern_trans_list.push_back(curr_CT_trans_mat);
    }
    
    cv::Mat calib_rot, calib_trans;
    // std::cout << "eef_trans_list" << std::endl;
    // for(int i=0; i<25; ++i)
    // {
    //     std::cout << eef_trans_list[i] << std::endl;
    // }
    // std::cout << "pattern_trans_list" << std::endl;
    // for(int i=0; i<25; ++i)
    // {
    //     std::cout << pattern_trans_list[i] << std::endl;
    // }
    cv::calibrateHandEye(eef_rot_list,
                         eef_trans_list,
                         pattern_rot_list,
                         pattern_trans_list,
                         calib_rot,
                         calib_trans,
                         cv::CALIB_HAND_EYE_DANIILIDIS);
    cv::Mat calibration_result = cv::Mat::eye(4, 4, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            calibration_result.at<double>(i, j) = calib_rot.at<double>(i, j);
        }
    }
    for (int i = 0; i < 3; i++) {
        calibration_result.at<double>(i, 3) = calib_trans.at<double>(i, 0);
    }

    std::cout << calibration_result << std::endl;
    // Eigen::Matrix4d calib_results_eigen;
    // Eigen::Matrix4d desirede_HEC_result;
    // cv::cv2eigen(calibration_result, calib_results_eigen);
    // desirede_HEC_result = cfslib::io::ReadMat4d(
    //     "../config/cam_calibration/henderson/hand_to_eye_calib_mat.txt");
    // std::cout << desirede_HEC_result << std::endl;
    // if (CompareMatrices(calib_results_eigen, desirede_HEC_result)) {
    //     std::cout << "Unit test passed" << std::endl;
    // } else {
    //     std::cout << "Unit test failed" << std::endl;
    // }
    
    return 1;
}