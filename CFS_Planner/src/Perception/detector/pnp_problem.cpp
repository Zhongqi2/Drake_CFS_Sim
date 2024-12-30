#include <iostream>
#include <sstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "detector/pnp_problem.h"
using namespace cv;
using namespace std;



PnPProblem::PnPProblem(const std::string &intrinsic_params_path)
{
    cout << intrinsic_params_path<<endl;
    // YAML::Node config = YAML::LoadFile(intrinsic_params_path);
    // cout << "intrinsics_c:" << config["intrinsics_c"] << endl;

    cv::FileStorage fs(intrinsic_params_path, cv::FileStorage::READ);
    fs["intrinsics_c"] >> A_matrix_;
    fs["dist_coeff_c"] >> distCoeffs_;
    cout << "a mat = " << A_matrix_.at<double>(0,2) <<endl;
    fs.release();
    //need to change the generation method

    Five_points_.push_back(cv::Point3f(36.8,0,0));
    Five_points_.push_back(cv::Point3f(11.371825,  34.99888 ,   0));
    Five_points_.push_back(cv::Point3f(-29.771824,  21.630497,   0));
    Five_points_.push_back(cv::Point3f(-29.771824, -21.630497,   0));
    Five_points_.push_back(cv::Point3f(11.371825, -34.99888 ,   0));
    R_matrix_ = cv::Mat::zeros(3,3,CV_64FC1);
    t_matrix_ = cv::Mat::zeros(3,1,CV_64FC1);
    P_matrix_ = cv::Mat::zeros(3,4,CV_64FC1);

    cout << "here\n";
}

PnPProblem::~PnPProblem()
{

}

void PnPProblem::setTransformation(const cv::Mat &rotation, const cv::Mat &translation)
{
    // Rotation-Translation Matrix Definition
    for (int i = 0; i < 3; i++)  {
        for (int j = 0; j < 3; j++) {
            P_matrix_.at<double>(i, j) = rotation.at<double>(i, j);
        }
        P_matrix_.at<double>(i, 3) = translation.at<double>(i);
    }    
}

bool PnPProblem::estimatePose(const std::vector<cv::Point3f> &list_points3d,
                              const std::vector<cv::Point2f> &list_points2d,
                              PatternProblem flags)
{
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

    
    if (list_points2d.size() != 5 && flags == FIVEPOINTS)
    {
        return false;
    }
    // std::cout << "calling pnp function..." << std::endl;

    // std::cout << "list_points3d : " << list_points3d << std::endl;
    // std::cout << "list_points2d : " << list_points2d << std::endl;
    // std::cout << "A_matrix_ : " << A_matrix_ << std::endl;
    // std::cout << "distCoeffs_ : " << distCoeffs_ << std::endl;
    
    // Pose estimation
    bool correspondence = cv::solvePnP(list_points3d, list_points2d, A_matrix_, distCoeffs_, rvec, tvec);
    
    // std::cout << "estimated rotation and translation" << std::endl;

    // Transforms Rotation Vector to Matrix
    cv::Rodrigues(rvec, R_matrix_);
    t_matrix_ = tvec;

    // Set projection matrix
    this->setTransformation(R_matrix_, t_matrix_);
    
    // std::cout << "end setTransformation(R_matrix_, t_matrix_);" << std::endl;
    return correspondence;
}

void PnPProblem::drawFrameAxes(cv::Mat &image, const cv::Mat &R_matrix, const cv::Mat &t_matrix, float length, int thickness)
{

    int type = image.type();
    int cn = CV_MAT_CN(type);
    // CV_CheckType(type, cn == 1 || cn == 3 || cn == 4,
    //              "Number of channels must be 1, 3 or 4" );

    // project axes points
    std::vector<cv::Point3f> axes_points;
    axes_points.push_back(cv::Point3f(0, 0, 0));
    axes_points.push_back(cv::Point3f(length, 0, 0));
    axes_points.push_back(cv::Point3f(0, length, 0));
    axes_points.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axes_points, R_matrix, t_matrix, A_matrix_, distCoeffs_, image_points);

    // draw axes lines
    cv::line(image, image_points[0], image_points[1], Scalar(0, 0, 255), thickness);
    cv::line(image, image_points[0], image_points[2], Scalar(0, 255, 0), thickness);
    cv::line(image, image_points[0], image_points[3], Scalar(255, 0, 0), thickness);
}

