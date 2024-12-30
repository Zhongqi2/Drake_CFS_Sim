#ifndef CHESS_BOARD_DETECTOR_H
#define CHESS_BOARD_DETECTOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "detector/pnp_problem.h"

enum Pattern {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};

class ChessBoardDetector {
public:
    ChessBoardDetector(PnPProblem &pnp_solver);
    void calcChessboardCorners(const cv::Size &boardSize, 
                               const float squareSize, 
                               std::vector<cv::Point3f>& corners, 
                               Pattern patternType);
    bool poseEstimationFromCoplanarPoints(cv::Mat &frame, 
                                          cv::Size patternSize, 
                                          const float squareSize, 
                                          const std::string &intrinsicsPath, 
                                          cv::Mat &rvec, 
                                          cv::Mat &tvec);

private: 
    PnPProblem pnp_solver_;
};



#endif