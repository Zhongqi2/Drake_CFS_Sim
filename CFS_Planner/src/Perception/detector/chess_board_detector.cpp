#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "detector/pnp_problem.h"
#include "detector/chess_board_detector.h"
// #include "headers.h"
using namespace std;
using namespace cv;
ChessBoardDetector::ChessBoardDetector(PnPProblem &pnp_solver) : pnp_solver_(pnp_solver) {
    
}

void ChessBoardDetector::calcChessboardCorners(const cv::Size &board_size, 
                                               const float square_size, 
                                               std::vector<cv::Point3f>& corners, 
                                               Pattern pattern_type)
{
    corners.resize(0);

    switch (pattern_type) 
    {
        case CHESSBOARD:
        case CIRCLES_GRID:
            //! [compute-chessboard-object-points]
            for (int i = 0; i < board_size.height; i++ ) 
            {
                for (int j = 0; j < board_size.width; j++ ) 
                {
                    corners.push_back(cv::Point3f(float(j * square_size),
                                                  float(i * square_size), 
                                                  0));
                }
            }
            //! [compute-chessboard-object-points]
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < board_size.height; i++ ) 
            {
                for (int j = 0; j < board_size.width; j++ ) 
                {
                    corners.push_back(cv::Point3f(float((2 * j + i % 2) * square_size),
                                                  float(i * square_size), 
                                                  0));
                }
            }
            break;

        default:
            CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
    }
}

bool ChessBoardDetector::poseEstimationFromCoplanarPoints(cv::Mat &frame, 
                                                          cv::Size pattern_size, 
                                                          const float square_size, 
                                                          const std::string &intrinsic_params_path, 
                                                          cv::Mat &rvec, 
                                                          cv::Mat &tvec)
{
	std::vector<cv::Point2f> corners;
    

    // crc_handeye::getTimePrint();
    cout << "findChessboardCorners begin"<<endl;
	bool is_pattern_found = cv::findChessboardCorners(frame, pattern_size, corners);//, chessBoardFlags);

	if (!is_pattern_found) {
        cout << "can not find ChessboardCorners!"<<endl;
		return false;
	}
    // crc_handeye::getTimePrint();
    cout << "findChessboardCorners finish corners size = " << corners.size() <<endl;
	cv::cornerSubPix(frame, 
                     corners, 
                     cv::Size(5,5),
                     cv::Size(-1,-1), 
                     cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
    cout << "cornerSubPix finish" <<endl;
	std::vector<cv::Point3f> object_points;
	this->calcChessboardCorners(pattern_size, square_size, object_points, CHESSBOARD);    
    //cout << "calcChessboardCorners finish" <<endl;
	pnp_solver_.estimatePose(object_points, corners, CHESSPOINTS);
    //cout << "estimatePose finish" <<endl;

	std::cout << pnp_solver_.getRotation() << std::endl;
	std::cout << pnp_solver_.getTranslation() << std::endl;

    // cv::Mat image(1000,1000,CV_8UC3,Scalar(255,255,255));
    // pnp_solver_.drawFrameAxes(image,pnp_solver_.getRotation(),pnp_solver_.getTranslation(),200,2);
    // cv::namedWindow("detection", cv::WINDOW_NORMAL);
    // cv::imshow("detection", image);
    // cv::waitKey(400);   
	rvec = pnp_solver_.getRotation();
	tvec = pnp_solver_.getTranslation();
    // printf("end poseEstimationFromCoplanarPoints");
	return true;
}
