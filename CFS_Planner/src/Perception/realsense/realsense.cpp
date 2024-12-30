#include "realsense/realsense.h"
#include <assert.h>

RealSense::RealSense(){
    // realsense_config_.enable_stream(RS2_STREAM_DEPTH, 320, 240, RS2_FORMAT_Z16, 30);
    // realsense_config_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    realsense_config_.enable_stream(RS2_STREAM_DEPTH);
    realsense_config_.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    realsense_pipeline_.start(realsense_config_);
    rgb_align_ptr_ = new rs2::align(RS2_STREAM_COLOR);
    // initialization of realsense stream
    for (auto i = 0; i < 30; ++i)
        realsense_pipeline_.wait_for_frames();
    depth_scale_ = getDepthScale(getDevice());    
    h_ = 480;
    w_ = 640;
}

RealSense::~RealSense(){
    delete rgb_align_ptr_;
}

rs2::device RealSense::getDevice(){
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0) 
        throw std::runtime_error("No device detected. Is it plugged in?");
    return list.front();
}

float RealSense::getDepthScale(rs2::device realsense_device) {
    for (rs2::sensor& sensor : realsense_device.query_sensors()) {
        if (rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>()) {
            return depth_sensor.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

void RealSense::capture(cv::Mat& rgb_img, cv::Mat& depth_img){
    rs2::frameset frameset = realsense_pipeline_.wait_for_frames();
    frameset = rgb_align_ptr_->process(frameset);
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();
    realsense_points_ = realsense_point_cloud_.calculate(depth);
    realsense_point_cloud_.map_to(color);
    // convert_to_pcl_point_cloud(point_cloud_ptr);
    // convert_to_pcl_point_cloud(point_img);
    cv::Mat curr_depth_img(cv::Size(depth.get_width(), depth.get_height()), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat curr_color_img(cv::Size(color.get_width(),color.get_height()), CV_8UC3, (void*)color.get_data(),cv::Mat::AUTO_STEP);
    rgb_img = curr_color_img;
    curr_depth_img.convertTo(depth_img, CV_32FC1);
    depth_img *= depth_scale_;
}

// void RealSense::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_ptr) {
//     int n_points = realsense_points_.size();
//     auto vertices = realsense_points_.get_vertices();   
//     int points_count = 0;
//     for (int pt_idx = 0; pt_idx < n_points; pt_idx++) {
//         if (vertices[pt_idx].z) {
//             pcl::PointXYZ pt;
//             pt.x = vertices[pt_idx].x;
//             pt.y = vertices[pt_idx].y;
//             pt.z = vertices[pt_idx].z;
//             point_cloud_ptr->points.push_back(pt);
//             points_count++;
//         }
//     }
//     point_cloud_ptr->is_dense = false;
//     point_cloud_ptr->height = 1;
//     point_cloud_ptr->width = points_count;
// }

void RealSense::getPointImage(cv::Mat& point_img) {
    point_img = cv::Mat::zeros(h_, w_, CV_32FC3);
    int n_points = realsense_points_.size();
    auto vertices = realsense_points_.get_vertices();   
    for (int pt_idx = 0; pt_idx < n_points; pt_idx++) {
        int y = pt_idx / w_;
        int x = pt_idx % w_;
        if (vertices[pt_idx].z) {
            point_img.at<cv::Vec3f>(y, x)[0] = vertices[pt_idx].x;
            point_img.at<cv::Vec3f>(y, x)[1] = vertices[pt_idx].y;
            point_img.at<cv::Vec3f>(y, x)[2] = vertices[pt_idx].z;
        }
    }
}

void RealSense::getIntrinsics(){
    rs2::frameset frameset = realsense_pipeline_.wait_for_frames();
    auto depth_frame = frameset.get_depth_frame();       
    auto color_frame = frameset.get_color_frame();       
    auto depth_profile = depth_frame.get_profile();    
    auto color_profile = color_frame.get_profile();  
    auto depth_intrin = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
    auto color_intrin = color_profile.as<rs2::video_stream_profile>().get_intrinsics();

    std::cout << "---depth_intrinsics---" << std::endl;
    std::cout << "  width : " << depth_intrin.width << std::endl;
    std::cout << "  height: " << depth_intrin.height << std::endl;
    std::cout << "  coeffs 1: " << depth_intrin.coeffs[0] << std::endl;   
    std::cout << "  coeffs 2: " << depth_intrin.coeffs[1] << std::endl;
    std::cout << "  coeffs 3: " << depth_intrin.coeffs[2] << std::endl;
    std::cout << "  coeffs 4: " << depth_intrin.coeffs[3] << std::endl;
    std::cout << "  coeffs 5: " << depth_intrin.coeffs[4] << std::endl;    

    std::cout << "---color_intrinsics---" << std::endl;
    std::cout << "  width : " << color_intrin.width << std::endl;
    std::cout << "  height: " << color_intrin.height << std::endl;
    std::cout << "  ppx : " << color_intrin.ppx << std::endl;             
    std::cout << "  ppy : " << color_intrin.ppy << std::endl;
    std::cout << "  fx: " << color_intrin.fx << std::endl;                
    std::cout << "  fy: " << color_intrin.fy << std::endl;    
    std::cout << "  model: " << color_intrin.model << std::endl;          
    std::cout << "  coeffs 1: " << color_intrin.coeffs[0] << std::endl;   
    std::cout << "  coeffs 2: " << color_intrin.coeffs[1] << std::endl;
    std::cout << "  coeffs 3: " << color_intrin.coeffs[2] << std::endl;
    std::cout << "  coeffs 4: " << color_intrin.coeffs[3] << std::endl;
    std::cout << "  coeffs 5: " << color_intrin.coeffs[4] << std::endl;    
}
