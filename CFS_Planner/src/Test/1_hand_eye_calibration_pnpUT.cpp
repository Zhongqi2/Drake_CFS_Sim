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

bool areMatricesEqual(const cv::Mat& mat1, const cv::Mat& mat2) {
    if (mat1.size() != mat2.size() || mat1.type() != mat2.type()) {
        return false;
    }
    return cv::norm(mat1, mat2) < 1e-3;
}
void loadMatrices(cv::Mat& rotation, cv::Mat& translation, const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file to read." << std::endl;
        return;
    }
    fs["pattern_rotation"] >> rotation;
    fs["pattern_translation"] >> translation;
    fs.release();
}
int main(int argc, char **argv)
{
    cv::Mat pattern_rotation, pattern_translation;
    cv::Mat desired_pattern_rotation, desired_pattern_translation;
    cv::Mat rgb_img, gray_img;
    std::string filename = "../data/pnp_cali/test_matrices.yml";
    loadMatrices(desired_pattern_rotation, desired_pattern_translation, filename);
    std::string image_path = "../data/pnp_cali/rgb_image.png";
    std::string intrinsic_param_file = "../config/cam_calibration/henderson/cv_calib_realsense.yaml";
    PnPProblem pnp_solver(intrinsic_param_file.c_str()); 
    ChessBoardDetector checker_board_detector(pnp_solver);
    rgb_img = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    checker_board_detector.poseEstimationFromCoplanarPoints(gray_img, cv::Size(7, 8), 10.0,
                                                            intrinsic_param_file.c_str(), 
                                                            pattern_rotation,
                                                            pattern_translation);
    if (areMatricesEqual(pattern_rotation, desired_pattern_rotation)&&areMatricesEqual(pattern_translation, desired_pattern_translation)) {
        std::cout << "Unit test passed" << std::endl;
    } else {
        std::cout << "Unit test failed" << std::endl;
    }

    
    return 1;
}