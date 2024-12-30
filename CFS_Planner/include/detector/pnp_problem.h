#ifndef PNP_PROBLEM_H_
#define PNP_PROBLEM_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

enum PatternProblem { CHESSPOINTS, FIVEPOINTS };

class PnPProblem
{
public:
    explicit PnPProblem(const std::string &intrinsic_params_path);  // custom constructor
    virtual ~PnPProblem();

    // cv::Point2f backproject3DPoint(const cv::Point3f &point3d);
    bool estimatePose(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, PatternProblem flags);

    cv::Mat getCameraMatrix() const { return A_matrix_; }
    cv::Mat getRotation() const { return R_matrix_; }
    cv::Mat getTranslation() const { return t_matrix_; }
    cv::Mat getTransformation() const { return P_matrix_; }
    std::vector<cv::Point3f> getFivePoints() const { return Five_points_; }
    cv::Mat getDistCoeff() const { return distCoeffs_; }

    void setTransformation( const cv::Mat &rotation, const cv::Mat &translation);
    void drawFrameAxes(cv::Mat &image, const cv::Mat &rotation, const cv::Mat &translation, float length, int thickness);

private:
    /** The camera intrinsic parameter calibration matrix */
    cv::Mat A_matrix_;
    /** The computed rotation matrix */
    cv::Mat R_matrix_;
    /** The computed translation matrix */
    cv::Mat t_matrix_;
    /** The computed projection matrix */
    cv::Mat P_matrix_;

    std::vector<cv::Point3f> Five_points_;

    cv::Mat distCoeffs_;
};

#endif /* PNPPROBLEM_H_ */