#ifndef REALSENSE_H
#define REALSENSE_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include "rs.hpp"
#include "h/rs_sensor.h"
#include <fstream>
#include <iostream>
#include <sstream>

class RealSense {
public:
    RealSense();
    ~RealSense();
    rs2::device getDevice();
    void getIntrinsics();
    // void alignDepth();
    // cv::Mat align_Depth2Color(cv::Mat depth, cv::Mat color, rs2::pipeline_profile profile);
    // void measure_distance(cv::Mat &color, cv::Mat depth, cv::Size range, rs2::pipeline_profile profile);
    // void align_test();
    // void captureColorImage();
    // void captureVideoStream();

    // void savePointImage(const cv::Mat &point_img);
    void capture(cv::Mat& rgb_img, cv::Mat& depth_img);
    // void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_ptr);
    void getPointImage(cv::Mat& point_img);

private:
    float getDepthScale(rs2::device realsense_device);
    // void convert_to_pcl_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_ptr);
    rs2::colorizer colorizer_;          
    rs2::pipeline realsense_pipeline_;
    rs2::config realsense_config_;
    rs2::align* rgb_align_ptr_;
    float depth_scale_;
    rs2::pointcloud realsense_point_cloud_;
    rs2::points realsense_points_;
    int h_;
    int w_;
};

#endif
