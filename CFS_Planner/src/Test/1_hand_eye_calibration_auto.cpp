#include "cfslib/cfslib.hpp"
#include "cfslib/Utils/Math.hpp"
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
#include "robot_model_gp50.h"
#undef inverse

RealSense cam;
bool is_display = true;
std::mutex mu;

Eigen::Matrix4d get_robot_flange_pose(
    HardwareInterface& robot_hw,
    const int& n_joint)
{
    double joint_pos_fbk[n_joint] = {0};
    for (int i=0; i<1000; ++i)
    {
        robot_hw.getJointPos(joint_pos_fbk);
    }
    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
    q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
    
    RobotModel_GP50 GP50;
    Eigen::MatrixXd T_BF = GP50.calFK(q0,'F');

    // Eigen::Matrix4d x0_mat = cfslib::math::FKine(q0, DH);
    // std::cout << "T_BF: \n" << T_BF<< std::endl;

    // std::cout << "robot pose read: \n" << q0/M_PI*180.0 << std::endl;

    return T_BF;
}

void display_camera_stream() {
    cv::Mat rgb_img, depth_img, vis_img;
    bool is_display_flag = true;
    while (is_display_flag) {
        mu.lock();
        cam.capture(rgb_img, depth_img);
        is_display_flag = is_display;
        mu.unlock();
        // cv::resize(rgb_img, vis_img, cv::Size(960, 540));
        cv::namedWindow("detection", cv::WINDOW_NORMAL);
        cv::imshow("detection", rgb_img);
        cv::waitKey(40);   
    }
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
            for (int i=0; i<250; ++i)
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
std::vector<cv::Mat> pattern_rot_list;
std::vector<cv::Mat> pattern_trans_list;
std::vector<cv::Mat> eef_rot_list;
std::vector<cv::Mat> eef_trans_list;

int main(int argc, char **argv)
{
    try
    {

        /* -------------------------------------------------------------------------- */
        /*                                Camera Setup                                */
        /* -------------------------------------------------------------------------- */

        // int id_rs1{10};
        // int marker_id{15};
        // std::vector<int> marker_ids{marker_id};
        // float marker_size{0.187};
        std::string serial_rs1{"845112072047"};
        std::string cam_param_path{"config/cam_calibration/"};

        cam.getIntrinsics();
        // std::thread t1(display_camera_stream);
        cout << "camera connected..." << endl;
        std::string intrinsic_param_file = "../config/cam_calibration/henderson/cv_calib_realsense.yaml";
        PnPProblem pnp_solver(intrinsic_param_file.c_str()); 
        ChessBoardDetector checker_board_detector(pnp_solver);

        /* -------------------------------------------------------------------------- */
        /*                                 Setup Robot                                */
        /* -------------------------------------------------------------------------- */

        // mode
        bool manual_move{true};

        // robot switch
        bool enable_robot{true}; // if true, use robot (real or simulated)
        bool real_robot{true}; // if true, use real robot
        bool robot_execute{true}; // if true, command robot to move

        // movement settings
        int T_ms{40}; // robot pos control loop time ms
        int movement_steps{301};
        int execution_time = (int)((movement_steps-1)*T_ms/1000);

        // robot property
        int n_joint{6};
        HardwareInterface robot_hw;
        double joint_pos_fbk[n_joint] = {0};
        int32_t joint_trq_fbk[n_joint] = {0};
        std::string robot_cap_fname = "../config/geometry_wrappers/robot_capsules_gp7.txt";
        Eigen::MatrixXd base_frame = Eigen::MatrixXd::Identity(4, 4);
        // std::string dh_fname{"../config/robot_properties/robot_DH_gp7.txt"};
        // Eigen::MatrixXd DH = Eigen::MatrixXd::Zero(1, 4); // base to joint 1
        // DH = cfslib::math::EigenVcat(DH, cfslib::io::LoadMatFromFile(dh_fname)); // base to ee
        std::cout << std::setprecision(10);

        // initial calibration pose, hardcoded accordingto CAD model
        Eigen::Matrix4d mat_base2calib_init = get_robot_flange_pose(robot_hw, n_joint);
        Eigen::Matrix4d mat_base2calib{mat_base2calib_init};

        // obstacle position (none in handeye calibration)
        std::string obs_fname = "config/geometry_wrappers/CFSLin_test/obs_capsules.txt";

        // solver config
        std::string config_fname = "config/user_config/user_config_default.txt";

        /* -------------------------------------------------------------------------- */
        /*                           Hand-to-eye Calibration                          */
        /* -------------------------------------------------------------------------- */

        /* ---------------------------------- Setup --------------------------------- */

        Eigen::Matrix4d mat_base2cam;
        int N_CALIB_POSES_MAX{15};
        int n_calib_poses{0};
        float T_DETECTION_MAX_MS{2000}; // max detection time per pose (ms)
        float DETECTION_HZ{10}; // detection frequency

        double perturb_trans_max{0.2}; // max translation distance along each dimension
        double perturb_ang_max{10.0f/180.0f*M_PI}; // max rotation angle in rad

        std::vector<Eigen::Matrix4d> mat_cam2marker_list;
        std::vector<Eigen::Matrix4d> mat_flange2base_list;

        /* ------------------------------ Collect Poses ----------------------------- */

        while (mat_cam2marker_list.size() < N_CALIB_POSES_MAX)
        {

            /* ------------------------------- Move robot ------------------------------- */
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>> Moving robot... <<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
            if (!manual_move)
            {
                // track_pos(mat_base2calib,
                //     robot_hw, n_joint, DH, movement_steps, base_frame, execution_time,
                //     config_fname, robot_cap_fname, obs_fname, enable_robot, real_robot, 
                //     robot_execute);
                
                // wait for robot to stablize
                usleep(1000*1000);
            }
            else
            {
                std::cout << "Manually move robot to next pose...\n";
                std::cin.get();
            }

            /* ------------------------- Update calibration pose ------------------------ */
            double dx = double(std::rand() % 1000) / 1000.0 * perturb_trans_max;
            double dy = double(std::rand() % 1000) / 1000.0 * perturb_trans_max;
            double dz = double(std::rand() % 1000) / 1000.0 * perturb_trans_max;
            double ax = double(std::rand() % 1000) / 1000.0;
            double ay = double(std::rand() % 1000) / 1000.0;
            double az = double(std::rand() % 1000) / 1000.0;
            double ang = double(std::rand() % 1000) / 1000.0 * perturb_ang_max;
            Eigen::Vector3d rot_axis{ax, ay, az};
            Eigen::AngleAxisd drot{ang, rot_axis.normalized()};
            Eigen::Affine3d trans_perturb{Eigen::Translation3d{dx, dy, dz}};
            trans_perturb.rotate(drot);
            Eigen::Matrix4d mat_perturb = trans_perturb.matrix();

            mat_base2calib = mat_perturb * mat_base2calib_init;

            std::cout << "Updated calibration pose.\n";
            std::cout << ">> mat_perturb:\n" << mat_perturb << "\n";
            std::cout << ">> mat_base2calib:\n" << mat_base2calib << "\n";

            /* ----------------------- Find marker in camera frame ---------------------- */
            cv::Mat img_rs1;
            std::vector<cv::Mat> trans_rs1;
            bool detected{false};

            cv::Mat rgb_img, gray_img, depth_img, viz_img;
            cv::Mat pattern_rotation, pattern_translation;
            cam.capture(rgb_img, depth_img);
            cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
            detected = checker_board_detector.poseEstimationFromCoplanarPoints(gray_img, cv::Size(7, 8), 10.0,
                                                                    intrinsic_param_file.c_str(), 
                                                                    pattern_rotation,
                                                                    pattern_translation);
            if (!detected)
            {
                std::cout << "Failed to detect marker, skipping current pose.\n";
                std::cout << "\n---------------------------------------------\n\n\n";
                continue;
            }
            pattern_translation /= 1000.0;

            Eigen::Matrix3d R_CT;
            Eigen::Vector3d p_CT;

            cv::cv2eigen(pattern_rotation, R_CT);
            cv::cv2eigen(pattern_translation, p_CT);

            Eigen::Matrix4d mat_cam2marker;
            mat_cam2marker << R_CT, p_CT, 0, 0, 0, 1;

            mat_cam2marker_list.push_back(mat_cam2marker);

            std::cout << ">> R_CT:\n" << R_CT << "\n";
            std::cout << ">> p_CT:\n" << p_CT << "\n";
            std::cout << ">> mat_cam2marker:\n" << mat_cam2marker << "\n";
            cfslib::io::WriteMat4d("../" + cam_param_path + serial_rs1 +
                                    "/hand_to_eye_poses/c2t_" + std::to_string(mat_cam2marker_list.size()-1) + ".txt", mat_cam2marker);

            /* ----------------------------- Find robot pose ---------------------------- */
            Eigen::Matrix4d mat_base2flange = get_robot_flange_pose(robot_hw, n_joint);
            mat_flange2base_list.push_back(mat_base2flange.inverse());

            std::cout << ">> mat_flange2base:\n" << mat_flange2base_list.back() << "\n";
            cfslib::io::WriteMat4d("../" + cam_param_path + serial_rs1 +
                                    "/hand_to_eye_poses/g2b_" + std::to_string(mat_flange2base_list.size()-1) + ".txt", mat_flange2base_list.back());

            std::cout << "Collected " << mat_flange2base_list.size() << "/" << N_CALIB_POSES_MAX << " poses.\n";

            double joint_pos_fbk[n_joint] = {0};
            robot_hw.getJointPos(joint_pos_fbk)
            Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(n_joint, 1);
            q0 << joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5];
            cfslib::io::WriteMat4d("../" + cam_param_path + serial_rs1 +
                        "/hand_to_eye_poses/jointpos_" + std::to_string(mat_flange2base_list.size()-1) + ".txt", joint_pos_fbk);
            std::cout << "\n---------------------------------------------\n\n\n";

        }
        std::string path = "/home/MASCEI/Auto_calibration/config/cam_calibration/845112072047/hand_to_eye_poses/";

        for(int i=0; i<N_CALIB_POSES_MAX; ++i)
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

        std::cout << "hand_to_eye_calib_mat" << std::endl;
        std::cout << calibration_result << std::endl;
        Eigen::Matrix4d calib_results_eigen;
        cv::cv2eigen(calibration_result, calib_results_eigen);
        cout<<"calib_results_eigen" <<calib_results_eigen.inverse() << endl;
        cfslib::io::WriteMat4d(
            "/mnt/storage/hand_to_eye_calib_mat.txt",
            calib_results_eigen);

        // mat_flange2marker = inv(mat_base2flange) * mat_base2marker = inv(mat_base2flange) * base2camera * camera2marker
        // take average of mat_flange2marker
        cfslib::math::Vector6d f2t_sum;
        for(int i=0; i<N_CALIB_POSES_MAX; ++i)
        {
            Eigen::Matrix4d T_CT = cfslib::io::ReadMat4d(path+"c2t_"+std::to_string(i) + ".txt");
            Eigen::Matrix4d T_GB = cfslib::io::ReadMat4d(path+"g2b_"+std::to_string(i) + ".txt");
            Eigen::Matrix4d T_FT = T_GB * calib_results_eigen * T_CT;
            cfslib::math::Vector6d f2t;
            cfslib::math::TransMatToPoseAngRad(T_FT, f2t);
            f2t_sum += f2t;
        }
        cfslib::math::Vector6d f2t_avg = f2t_sum / N_CALIB_POSES_MAX;
        Eigen::Matrix4d mat_flange2marker;
        cfslib::math::PoseAngRadToTransMat(f2t_avg, mat_flange2marker);

        std::cout << "flang_to_tool_calib_mat" << std::endl;
        std::cout << mat_flange2marker << std::endl;
        cfslib::io::WriteMat4d(
            "/mnt/storage/hand_to_tool_calib_mat.txt",
            mat_flange2marker);

        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}