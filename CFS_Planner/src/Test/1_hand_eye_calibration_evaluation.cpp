#include "cfslib/cfslib.hpp"
#include <aruco/aruco.h>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <cstdlib>
#include "hardware_interface.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#undef inverse

Eigen::Matrix4d get_robot_flange_pose(
    HardwareInterface& robot_hw,
    const int& n_joint,
    const Eigen::MatrixXd& DH)
{
    double joint_pos_fbk[n_joint] = {0};
    robot_hw.getJointPos(joint_pos_fbk);
    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
    Eigen::Matrix4d x0_mat = cfslib::math::FKine(q0, DH);

    return x0_mat;
}

void track_pos(
    const Eigen::Matrix4d& goal_mat,
    HardwareInterface& robot_hw,
    const int& n_joint,
    const Eigen::MatrixXd& DH,
    const int& movement_steps,
    const Eigen::MatrixXd& base_frame,
    const int& execution_time,
    const std::string& config_fname,
    const std::string& robot_cap_fname,
    const std::string& obs_fname,
    const bool& enable_robot,
    const bool& real_robot,
    const bool& robot_execute
)
{
    Eigen::MatrixXd joint_limits(n_joint, 2);
    joint_limits << -PI/2, PI/2, 
                    -PI/2, PI/2, 
                    -PI/2, PI/2, 
                    -PI, PI, 
                    -PI, PI, 
                    -PI, PI;

    Eigen::MatrixXd joint_vel_limits(n_joint, 1);
    joint_vel_limits << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    
    Eigen::MatrixXd joint_acc_limits(n_joint, 1);
    joint_acc_limits << 0.6, 0.6, 0.6, 0.6, 0.6, 0.6;

    Eigen::MatrixXd joint_disp_limits(n_joint, 1);
    joint_disp_limits << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    
    double cart_vel_max = 1;
    double cart_acc_max = 0.5;

    Eigen::MatrixXi collision_avoidance_links(8, 1);
    collision_avoidance_links << 1, 1, 1, 1, 1, 1, 1, 1;

    /* ----------------------------- get robot state ---------------------------- */
    double joint_pos_fbk[n_joint] = {0};
    if (enable_robot)
    {
        if (real_robot)
        {
            robot_hw.getJointPos(joint_pos_fbk);
        }
    }

    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    if (enable_robot)
    {
        q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
        std::cout << ">> q0:\n" << q0 << "\n";
    }
    else
    {
        q0 << -2.6, 31.3, 8.7, -2.2, 19.0, 2.3;
        q0 = q0 / 180.0 * M_PI;
    }
    // q0 << 0.185927, 0.520814, 0.142393, -0.432303, 0.468512, 0.915695;
    // q0 << -2.6, 31.3, 8.7, -2.2, 19.0, 2.3;
    // q0 = q0 / 180.0 * M_PI;
    cfslib::math::Vector6d x0;
    Eigen::Matrix4d x0_mat = cfslib::math::FKine(q0, DH);
    cfslib::math::TransMatToPoseAngRad(x0_mat, x0);
    cfslib::math::Vector6d x_init = x0;
    Eigen::Quaterniond x0_quat{ x0_mat.block<3, 3>(0, 0) };

    /* --------------------------- set ref trajectory --------------------------- */
    cfslib::math::Vector6d x1;
    cfslib::math::TransMatToPoseAngRad(goal_mat, x1);

    cfslib::math::Vector6d dx = (x1-x0)/(movement_steps-1);

    Eigen::MatrixXd cart_waypoint_ref;
    Eigen::VectorXd joint_ref(0);
    cart_waypoint_ref.resize(6, movement_steps);
    cart_waypoint_ref.col(0) = x0;
    for (int traj_i=1; traj_i < movement_steps; ++traj_i)
    {
        cart_waypoint_ref.col(traj_i) = x0 + traj_i * dx;
    }

    #ifdef DEBUG_PRINT
    std::cout << ">> input cart waypoint:\n" << cart_waypoint_ref.transpose() << "\n";

    std::cout << ">> q0:\n" << q0 << "\n";
    std::cout << ">> x0:\n" << x0 << "\n";
    std::cout << ">> x0_mat:\n" << x0_mat << "\n";
    std::cout << ">> x0_quat:\n" << x0_quat.w() << ", " << x0_quat.x() << ", "
                                << x0_quat.y() << ", " << x0_quat.z() << "\n";
    #endif

    // Eigen::MatrixXd cart_traj(0,0), joint_traj(0,0), dist_profile(0,0), dist_init_profile(0,0), cart_input_ref(0, 0), joint_input_ref(0, 0);
    // Eigen::MatrixXd cart_vel(0, 0), cart_vel_ref(0, 0), cart_acc(0, 0), cart_acc_ref(0, 0);
    // Eigen::MatrixXd joint_vel(0, 0), joint_vel_ref(0, 0), joint_acc(0, 0), joint_acc_ref(0, 0);
    Eigen::MatrixXd CFS_traj;
    // std::vector<double> traj_len;

    cfslib::query::UserQuery::Ptr query;
    cfslib::trajectory::Trajectory::Ptr traj_ptr;
    cfslib::query::Mode mode;
    mode.goal_type         = cfslib::query::GoalType::CartPath;
    mode.use_timestamp     = true;
    mode.enable_tempopt    = false;
    mode.dynamic_obstacle  = false;
    mode.enforce_cartesian = cfslib::query::CartesianMode::None;

    cfslib::CFSLib solver{};

    query = std::make_shared<cfslib::query::UserQuery>(
        x0, q0, DH, cart_waypoint_ref, joint_ref,
        joint_limits, joint_vel_limits, joint_acc_limits, joint_disp_limits,
        base_frame, collision_avoidance_links, cart_vel_max, cart_acc_max,
        execution_time, mode, config_fname, robot_cap_fname, obs_fname
    );
    traj_ptr = solver.DemoSolve(query);
    CFS_traj = traj_ptr->joint_path();

    std::cout << "CFS_traj pt  0:\n" << CFS_traj.col(0).transpose() << "\n";
    std::cout << "CFS_traj pt  1:\n" << CFS_traj.col(1).transpose() << "\n";
    std::cout << "CFS_traj pt  2:\n" << CFS_traj.col(2).transpose() << "\n";
    std::cout << "CFS_traj pt -1:\n" << CFS_traj.rightCols(1).transpose() << "\n";
    std::cout << "CFS_traj pt -1:\n" << CFS_traj.col(CFS_traj.cols()-1).transpose() << "\n";

    #ifdef DEBUG_PRINT
    // q0 = CFS_traj.rightCols(1);
    // x0 = cart_waypoint_ref.col(0);
    std::cout << "CFS_traj:\n" << CFS_traj.transpose() << "\n";
    std::cout<<"Cart reference:\n"<<traj_ptr->cart_init_ref().transpose()<<"\n";
    std::cout<<"Cart path:\n"<<traj_ptr->cart_path().transpose()<<"\n";
    #endif

    /* --------------------------- Send for execution --------------------------- */
    if (traj_ptr->status() && traj_ptr->vel_flag() && traj_ptr->dense_flag())
    {
        if (enable_robot && robot_execute)
        {
            double joint_pos_des[n_joint] = {0};
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>> Sending commands... <<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
            for (int t=0; t<CFS_traj.cols(); ++t)
            {
                // send cmd
                for (int joint_i=0; joint_i<6; ++joint_i)
                {
                    joint_pos_des[joint_i] = CFS_traj(joint_i, t);
                }

                if (real_robot)
                {
                    robot_hw.getJointPos(joint_pos_fbk);
                    robot_hw.setJointPosCmd(joint_pos_des);
                }

                // send to vrep
                if (t % 10 == 0)
                {
                    // send
                }

                usleep(4000);
            }

            // extra cmd to ensure reaching
            for (int i=0; i<500; ++i)
            {
                if (real_robot)
                {
                    robot_hw.getJointPos(joint_pos_fbk);
                    robot_hw.setJointPosCmd(joint_pos_des);
                }
                usleep(4000);
            }

            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>> Done <<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
        }
    }
    else
    {
        std::cerr << "Planning flags failed...\n";
        std::cerr << "CFS       OK: "  << traj_ptr->status() << "\n";
        std::cerr << "Velocity  OK: "  << traj_ptr->vel_flag() << "\n";
        std::cerr << "Density   OK: " << traj_ptr->dense_flag() << "\n";
    }
}

std::vector<cv::Mat> DetectMarkerByID(cv::Mat& image, cv::VideoCapture& cam, const aruco::CameraParameters& param, const float& marker_size,
                            aruco::MarkerDetector& detector, const std::vector<int>& marker_ids, const std::string& name, const bool& show=true)
{
    try
    {
        // ! Object Pose Stamp
        cam >> image;
        if (image.empty())
        {
            std::cerr << "Failed to read camera.\n";
            return cv::Mat();
        }

        std::vector<cv::Mat> mats;
        bool marker_found{false};
        auto markers = detector.detect(image, param, marker_size);
        for (auto marker_id:marker_ids)
        {
            marker_found = false;
            for (auto marker:markers)
            {
                if (marker.id == marker_id)
                {
                    aruco::CvDrawingUtils::draw3dAxis(image, marker, param);
                    aruco::CvDrawingUtils::draw3dCube(image, marker, param);
                    mats.emplace_back(marker.getTransformMatrix());
                    marker_found = true;
                    break;
                }
            }

            if (!marker_found)
            {
                std::cerr << "Marker id " << marker_id << " is not found.\n";
            }
        }

        if (show)
        {
            cv::imshow(name, image);
        }

        return mats;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw e;
    }
}

std::pair<Eigen::Translation3d, Eigen::Quaterniond> to_eigen_tans_q(const cv::Mat& trans_cvmat)
{
    Eigen::Matrix<double, 3, 3> rot_mat;
    rot_mat << (double)trans_cvmat.at<float>(0, 0), (double)trans_cvmat.at<float>(0, 1), (double)trans_cvmat.at<float>(0, 2),
                (double)trans_cvmat.at<float>(1, 0), (double)trans_cvmat.at<float>(1, 1), (double)trans_cvmat.at<float>(1, 2),
                (double)trans_cvmat.at<float>(2, 0), (double)trans_cvmat.at<float>(2, 1), (double)trans_cvmat.at<float>(2, 2);
    Eigen::Quaterniond q{ rot_mat };

    Eigen::Vector3d tvec;
    tvec << (double)trans_cvmat.at<float>(0, 3), (double)trans_cvmat.at<float>(1, 3), (double)trans_cvmat.at<float>(2, 3);

    return std::make_pair(Eigen::Translation3d{tvec}, q);
}

double get_pose_distance(const cv::Mat &pose_1, const cv::Mat &pose_2) {
    cv::Vec3d trans_1;

    trans_1[0] = pose_1.at<double>(0, 3);
    trans_1[1] = pose_1.at<double>(1, 3);
    trans_1[2] = pose_1.at<double>(2, 3);

    // std::cout << "pose1(x,y,z)" << trans_1 << std::endl;
    cv::Vec3d trans_2;
    trans_2[0] = pose_2.at<double>(0, 3);
    trans_2[1] = pose_2.at<double>(1, 3);
    trans_2[2] = pose_2.at<double>(2, 3);
    // std::cout << "pose2(x,y,z)" << trans_2 << std::endl;
    cv::Vec3d trans_diff = trans_1 - trans_2;
    // std::cout << "diff(x,y,z)" << trans_diff << std::endl;
    double dist = cv::norm(trans_diff);
    // std::cout << "dist = " << dist << std::endl;
    return dist;
}

double evaluate_daniilidis(std::vector<cv::Mat> eef_rot_list, std::vector<cv::Mat> eef_trans_list,
                           std::vector<cv::Mat> pattern_rot_list, std::vector<cv::Mat> pattern_trans_list,
                           cv::Mat res) {
    int nSize = eef_rot_list.size();
    if (nSize != eef_trans_list.size() || nSize != pattern_rot_list.size() || nSize != pattern_trans_list.size()) {
        cout << "eef & pattern size error" << endl;
        return 0.0f;
    }
    std::vector<cv::Mat> pattern_poses;
    std::vector<cv::Mat> eef_poses;

    for (int n = 0; n < nSize; ++n) {
        cv::Mat pattern4f = cv::Mat::eye(4, 4, CV_64FC1);
        cv::Mat eef4f = cv::Mat::eye(4, 4, CV_64FC1);
        // Eigen::Affine3d pattern_eigen,eef_eigen;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pattern4f.at<double>(i, j) = pattern_rot_list[n].at<double>(i, j);
                eef4f.at<double>(i, j) = eef_rot_list[n].at<double>(i, j);
            }
        }

        for (int i = 0; i < 3; ++i) {
            pattern4f.at<double>(i, 3) = pattern_trans_list[n].at<double>(i, 0);
            eef4f.at<double>(i, 3) = eef_trans_list[n].at<double>(i, 0);
        }

        pattern_poses.push_back(pattern4f);
        eef_poses.push_back(eef4f);

        // cout << n << ",pattern4*4 : " << endl
        //  << pattern4f << endl;
        // cout <<n << ",eef4*4 : " << endl
        //  << eef4f << endl;
    }

    for (int j = 0; j < 15; ++j)
    {
        cv::Mat a_1 = pattern_poses[j];
        cv::Mat b_1 = eef_poses[j];
        cv::Mat res_inv;
        cv::invert(res, res_inv);
        double sum_distance = 0;
        // nSize = pattern_poses.size();
        // cout << "nSize = " << nSize << ",pattern_poses size = " << pattern_poses.size()<<endl;
        for (int i = 1; i < nSize; ++i) {
            cv::Mat a_i = pattern_poses[i];
            cv::Mat b_i = eef_poses[i];
            cv::Mat b_i_inv;
            cv::invert(b_i, b_i_inv);

            cv::Mat a_i_hat = res_inv * b_i_inv * b_1 * res * a_1;

            double distance = get_pose_distance(a_i, a_i_hat);
            sum_distance += distance;
        }
        std::cout << "sum_distance = " << sum_distance << std::endl;
        double avg_distance = sum_distance / (nSize - 1);
        std::cout << "the avg_distance = " << avg_distance << std::endl;
    }
    return 1;
}

std::vector<cv::Mat> pattern_rot_list;
std::vector<cv::Mat> pattern_trans_list;
std::vector<cv::Mat> eef_rot_list;
std::vector<cv::Mat> eef_trans_list;

int main(int argc, char **argv)
{
    std::string path = "/home/MASCEI/Auto_calibration/config/cam_calibration/845112072047/hand_to_eye_poses/";

    for(int i=0; i<15; ++i)
    {
        Eigen::Matrix4d T_CT = cfslib::io::ReadMat4d(path+"c2t_"+std::to_string(i) + ".txt");
        Eigen::Matrix4d T_GB = cfslib::io::ReadMat4d(path+"g2b_"+std::to_string(i) + ".txt");


        // Eigen::Matrix4d T_BG = T_GB.inverse();

        Eigen::Matrix3d R_CT = T_CT.block<3,3>(0,0);
        Eigen::Vector3d p_CT = T_CT.block<3,1>(0,3);
        
        Eigen::Matrix3d R_GB = T_GB.block<3,3>(0,0);
        Eigen::Vector3d p_GB = T_GB.block<3,1>(0,3);
        // Eigen::Matrix3d R_GB = T_BG.block<3,3>(0,0);
        // Eigen::Vector3d p_GB = T_BG.block<3,1>(0,3);

        cv::Mat curr_CT_rot_mat, curr_CT_trans_mat;

        cv::eigen2cv(R_CT, curr_CT_rot_mat);
        cv::eigen2cv(p_CT, curr_CT_trans_mat);

        // std::cout << "converted eef pose: " << std::endl;
        // std::cout << curr_eef_rot_mat << std::endl;
        // std::cout << curr_eef_trans_mat << std::endl;

        cv::Mat curr_GB_rot_mat, curr_GB_trans_mat;

        cv::eigen2cv(R_GB, curr_GB_rot_mat);
        cv::eigen2cv(p_GB, curr_GB_trans_mat);

        eef_rot_list.push_back(curr_GB_rot_mat);
        eef_trans_list.push_back(curr_GB_trans_mat);

        pattern_rot_list.push_back(curr_CT_rot_mat);
        pattern_trans_list.push_back(curr_CT_trans_mat);
    }
    
    cv::Mat calib_rot, calib_trans;
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
    Eigen::Matrix4d calib_results_eigen;
    cv::cv2eigen(calibration_result, calib_results_eigen);
    cout<<"calib_results_eigen" <<calib_results_eigen.inverse() << endl;
    cfslib::io::WriteMat4d(
        "/mnt/storage/hand_to_eye_calib_mat.txt",
        calib_results_eigen);

    evaluate_daniilidis(eef_rot_list,
                        eef_trans_list,
                        pattern_rot_list,
                        pattern_trans_list, calibration_result);
    
    return 1;
}