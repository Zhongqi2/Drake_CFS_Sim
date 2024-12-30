import open3d as o3d
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BaseField,
    Box,
    DepthImageToPointCloud,
    DiagramBuilder,
    Fields,
    MeshcatPointCloudVisualizer,
    MeshcatVisualizer,
    Parser,
    PixelType,
    PointCloud,
    Rgba,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    SpatialInertia,
    StartMeshcat,
    LeafSystem,
    AbstractValue,
    Concatenate,
)
from scipy.spatial import KDTree
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    Cylinder,
    DiagramBuilder,
    InverseKinematics,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    RigidTransform,
    Role,
    RollPitchYaw,
    RotationMatrix,
    Parser,
    Simulator,
    InverseDynamicsController,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    CameraInfo,
    RgbdSensor,
    ClippingRange,
    RenderCameraCore,
    ColorRenderCamera,
    DepthRenderCamera,
    DepthRange,
    FixedOffsetFrame,
    AutoDiffXd,
    InitializeAutoDiff,
    Solve,
    StartMeshcat,
)
from IPython.display import HTML, SVG, display, clear_output
import cv2 as cv
import open3d as o3d
import time
import teaserpp_python
import copy
import math
import open3d as o3d
import teaserpp_python
import numpy as np 
import copy
from teaser_helpers import *
from pcd_registration import *
from os import path as osp
import os
from transforms3d import affines as aff
from transforms3d import euler as eul
import ipdb

# First import the library
import pyrealsense2 as rs
import numpy as np
from IPython.display import HTML, SVG, display, clear_output
import cv2 as cv
import open3d as o3d

ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    print("resetting device: ", dev.get_info(rs.camera_info.name))
    dev.hardware_reset()

def get_scene_pcd():
    intrinsic_inv = np.linalg.inv(np.matrix([[917.5706787109375, 0.0, 645.0847778320312],
                                             [0.0, 917.9429931640625, 372.301513671875], 
                                             [0.0, 0.0, 1.0]]))
    
    with open('config/cam_calibration/henderson/hand_to_eye_calib_mat.txt', 'r') as infile:
        lines = infile.readlines()
        extrinsic_mat = np.array([[float(val) for val in line[:-1].split(',')] for line in lines[:3]])
        extrinsic_mat = np.concatenate([extrinsic_mat, np.array([[0, 0, 0, 1]])], axis=0)

    # print('Hand eye calib mat:')
    # print(extrinsic_mat)

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        print("1")
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # rs.config.enable_device_from_file(config, "20240930_160037.bag")

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1.5 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    decimation = rs.decimation_filter()
    hole_filling = rs.hole_filling_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)
    # Streaming loop
    try:
        while True:
            depth_frames = []
            color_frames = []

            for i in range(10):
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = spatial.process(aligned_frames.get_depth_frame())
                depth_frames.append(depth_frame)
                color_frames.append(aligned_frames.get_color_frame())
            # # Get frameset of color and depth
            # frames = pipeline.wait_for_frames()
            # # frames.get_depth_frame() is a 640x360 depth image

            # # Align the depth frame to color frame
            # aligned_frames = align.process(frames)

            # # Get aligned frames
            # aligned_depth_frame = aligned_frames.get_depth_frame()
            # color_frame = aligned_frames.get_color_frame()

            color_frame = color_frames[0]


            temporal = rs.temporal_filter()
            for x in range(10):
                aligned_depth_frame = temporal.process(depth_frames[x])
                        
            # # Filter the depth frame
            # aligned_depth_frame = decimation.process(aligned_depth_frame)
            aligned_depth_frame = hole_filling.process(aligned_depth_frame)
            # colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Remove background - Set pixels further than clipping_distance to grey
            # grey_color = 153
            # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # Render images:
            #   depth align to color on left
            #   depth on right
            # depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

            xrange = [0, 1280]
            yrange = [0, 720]

            # bg_removed[yrange[0], xrange[0]:xrange[1]] = [0, 0, 255]
            # bg_removed[yrange[1], xrange[0]:xrange[1]] = [0, 0, 255]
            # bg_removed[yrange[0]:yrange[1], xrange[0]] = [0, 0, 255]
            # bg_removed[yrange[0]:yrange[1], xrange[1]] = [0, 0, 255]

            # depth_colormap[yrange[0], xrange[0]:xrange[1]] = [0, 0, 255]
            # depth_colormap[yrange[1], xrange[0]:xrange[1]] = [0, 0, 255]
            # depth_colormap[yrange[0]:yrange[1], xrange[0]] = [0, 0, 255]
            # depth_colormap[yrange[0]:yrange[1], xrange[1]] = [0, 0, 255]

            # images = np.hstack((bg_removed, depth_colormap))

            # cv.namedWindow('Align Example', cv.WINDOW_NORMAL)
            # cv.imshow('Align Example', images)
            # key = cv.waitKey(1)
            # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:

            # compute 3d pcd
            xlin = np.linspace(xrange[0], xrange[1], xrange[1]-xrange[0], dtype=int, endpoint=False)
            ylin = np.linspace(yrange[0], yrange[1], yrange[1]-yrange[0], dtype=int, endpoint=False)
            x, y = np.meshgrid(xlin, ylin, indexing='xy')
            x = np.reshape(x, -1)
            y = np.reshape(y, -1)

            pt = np.stack([x, y, np.ones_like(y)], axis=0)
            depth_val = depth_image[y, x]

            pt_3d = np.multiply(np.matmul(intrinsic_inv, pt), np.stack([depth_val for _ in range(3)], axis=0)) * depth_scale
            pt_3d = np.concatenate([pt_3d, np.ones_like(y).reshape(1,-1)], axis=0)
            pt_world = np.matmul(extrinsic_mat, pt_3d)

            # pt_world 
            # cv.destroyAllWindows()
            return pt_3d[:3, :]
        
    finally:
        pipeline.stop()

import numpy as np

def check_transform_error(matrix1, matrix2, threshold=0.01):
    # Extract rotation and translation components
    R1, t1 = matrix1[:3, :3], matrix1[:3, 3]
    R2, t2 = matrix2[:3, :3], matrix2[:3, 3]
    
    # Compute distance error
    distance_error = np.linalg.norm(t1 - t2)
    
    # Compute rotation error
    rotation_diff = np.dot(R1.T, R2)
    rotation_error = np.arccos((np.trace(rotation_diff) - 1) / 2)
    
    # Check if both errors are within the threshold
    return distance_error <= threshold and rotation_error <= threshold
        
TARGET = 'pallet' #'container2_robot2' # 'container' 'bagger' 'tool' 'conveyor'
TARGET_PCD = 'pallet'#'container2_robot2' # 'container' 'Bagging Machine' 'EPick' 'conveyor'
N_POS = 1 # num of camera positions
N_ANG = 1 # num of camera view angles
N_NOISE_REPEAT = 1 # num of noisy observations

theta = -60 * np.pi / 180
c = np.cos(theta)
s = np.sin(theta)
# T_guess = np.array([[ 1 , 0 , 0 , -0.5],[ 0 , c , -s ,0],[ 0 ,-s , c , 2.5],[ 0,  0 , 0 , 1]])
# print
# T_guess = np.load('T_guess.npy')
# ipdb.set_trace()
VIZ = True
from scipy.spatial.transform import Rotation as R

def rotation_matrix_to_euler_xyz_no_scipy(rotation_matrix):
    # Ensure it's a valid 3x3 rotation matrix
    assert rotation_matrix.shape == (3, 3), "Input must be a 3x3 matrix."

    # Extract angles
    if abs(rotation_matrix[2, 0]) != 1:  # Not at a singularity
        ry = -np.arcsin(rotation_matrix[2, 0])
        cos_ry = np.cos(ry)
        rx = np.arctan2(rotation_matrix[2, 1] / cos_ry, rotation_matrix[2, 2] / cos_ry)
        rz = np.arctan2(rotation_matrix[1, 0] / cos_ry, rotation_matrix[0, 0] / cos_ry)
    else:  # Gimbal lock
        rz = 0  # Or any arbitrary value
        if rotation_matrix[2, 0] == -1:
            ry = np.pi / 2
            rx = rz + np.arctan2(rotation_matrix[0, 1], rotation_matrix[0, 2])
        else:
            ry = -np.pi / 2
            rx = -rz + np.arctan2(-rotation_matrix[0, 1], -rotation_matrix[0, 2])

    # Convert to degrees
    rx, ry, rz = np.degrees([rx, ry, rz])
    return rx, ry, rz

def extract_pose(matrix):
    # Ensure the input is a 4x4 matrix
    assert matrix.shape == (4, 4), "Input must be a 4x4 matrix."
    
    # Extract translation (x, y, z)
    x, y, z = matrix[:3, 3]
    
    # Extract rotation matrix (top-left 3x3 part)
    rotation_matrix = matrix[:3, :3]
    
    # Convert the rotation matrix to Euler angles (in radians)
    rx, ry, rz = rotation_matrix_to_euler_xyz_no_scipy(rotation_matrix)
    return x, y, z, rx, ry, rz

meshcat = StartMeshcat()
while 1:
    # scene_val = get_scene_pcd() # [3, M]
    # scene_val = np.array(scene_val)
    
    # # Define the path where you want to save the file
    file_path = f"data/{TARGET}_test/{TARGET}.npy" #'c2r2.npy'

    # Save the array
    # np.save(file_path, scene_val)
    np.load(file_path)
    # if VIZ:
    #     meshcat = StartMeshcat()

    cam_xs = [] # camera x coordinates
    cam_as = [] # camera x rotations
    sigmas = [0.0] # std of gaussian noise

    # model pcd model, model pcd values (6, N)
    model_drake, model_val,T_model = load_model_pointcloud_xyz(f"data/{TARGET}_test/{TARGET}_model_fixed.xyz")
    meshcat.SetObject(f"{TARGET}_model_detected", model_drake, point_size=0.005, rgba=Rgba(1, 1, 0, 0.5))
    # ipdb.set_trace()
    # load ground truth target pose
    T_gt = np.loadtxt(f'data/container_test/container_pose.txt')

    # results
    rot_err_all     = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    trans_err_all   = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    time_teaser_all = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    time_icp_all    = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))

    for pos_i in range(N_POS):

        for ang_i in range(N_ANG):
            
            data_id = pos_i*N_POS + ang_i

            # parse camera pos
            mat_w2c = np.loadtxt(osp.join(f'data/container_test/camera_pose0.txt'))
            T, R, _, _ = aff.decompose44(mat_w2c)
            rz, ry, rx = eul.mat2euler(R, axes='rzyx')
            
            # collect all view angles
            if pos_i == 0:
                cam_as.append(rx/np.pi*180.0)

            # collect all x pos values
            if ang_i == 0:
                cam_xs.append(T[0])

            # for each level of camera uncertainty
            for (sigma_i, sigma) in enumerate(sigmas):

                # run a batch of exp
                for noise_i in range(N_NOISE_REPEAT):
                    
                    # print(f'\n------- {pos_i} th pos, {ang_i} th ang, {sigma_i} th sigma, {noise_i} th exp -------\n')

                    # scene pcd model, scene pcd values (6, M)
                    scene_path = f"data/{TARGET}_test/{TARGET}.npy" #'c2r2.npy'
                    scene_drake_original = save_scene_pl(scene_path, TARGET)
                    ipdb.set_trace()
                    scene_drake, scene_val = load_scene_pointcloud(f"data/{TARGET}_test/{TARGET}_scene{0}.txt", sigma=sigma)

                    # display
                    if VIZ:
                        # meshcat.SetObject(f"{TARGET}_scene", scene_drake_original, point_size=0.005, rgba=Rgba(0, 0, 0, 1))
                        meshcat.SetObject(f"{TARGET}_scene", scene_drake, point_size=0.02, rgba=Rgba(0, 0, 0, 1))

                    # # save as ply
                    save_cloud(f"{TARGET}_model.ply", model_val)
                    save_cloud(f"{TARGET}_scene.ply", scene_val)
                    # ipdb.set_trace()
                    # global detection via teaser++
                    start = time.time()
                    T_teaser = teaserpp_registration(f'{TARGET}_model.ply', f'{TARGET}_scene.ply', viz=False)
                    time_teaser_all[pos_i, ang_i, sigma_i, noise_i] = time.time() - start
                    # T_guess = np.array([[ 1 , 0 , 0 , -0.5],[ 0 , c , -s ,0],[ 0 ,-s , c , 2.5],[ 0,  0 , 0 , 1]])
                    # T_guess = np.array([[ 1 , 0 , 0 , 0],[ 0 , c , -s ,0],[ 0 ,-s , c , 0],[ 0,  0 , 0 , 1]])
                    
                    theta_x = -130  # 90 degrees in radians
                    theta_y = 0
                    # Create a RollPitchYaw object
                    rotation_matrix = RollPitchYaw(-130 * np.pi/180,0,0)
                    X_7G = RigidTransform(RollPitchYaw(130 * np.pi/180, 0, 0), [-0.7,0.8,1.8])
                    T_guess = X_7G.GetAsMatrix4()
                    # if VIZ:
                    #     meshcat.SetObject(f"{TARGET}_model_detected", model_drake, point_size=0.02, rgba=Rgba(1, 1, 0, 0.5))
                    #     meshcat.SetTransform(f"{TARGET}_model_detected", X_7G)
                    
                    # T_guess = RigidTransform(rotation_matrix,[-0.7,0.8,1.6]).GetAsMatrix4()
                    is_same = check_transform_error(T_guess,T_teaser)
                    print(is_same)
                    if is_same ==  True:
                        T_guess = T_teaser
                    # ipdb.set_trace()
                    # fine tuning via icp
                    start = time.time()
                    T_icp = icp(T_guess, model_val.T, scene_val.T)
                    time_icp_all[pos_i, ang_i, sigma_i, noise_i] = time.time() - start
                    if VIZ:
                        meshcat.SetObject(f"{TARGET}_model_ICP_refined", model_drake, point_size=0.006, rgba=Rgba(0, 1, 0, 1))
                        meshcat.SetTransform(f"{TARGET}_model_ICP_refined", RigidTransform(T_icp))
                        meshcat.SetObject(f"{TARGET}_123", model_drake, point_size=0.005, rgba=Rgba(0, 1, 0, 1))
                        meshcat.SetTransform(f"{TARGET}_123", RigidTransform())


    print("camera in robot base:")
    T_b_c = np.loadtxt('/mnt/storage/hand_to_eye_calib_mat.txt', delimiter=',')
    print(T_b_c)

    print("target pose in camera transformation:")
    T_s_m = T_icp
    T_c_m = T_model.GetAsMatrix4()
    T_c_t = T_c_m @ T_s_m
    print(T_c_t)
    
    T_b_t = T_b_c * T_c_t
    # Extract translation and Euler angles
    x, y, z, rx, ry, rz = extract_pose(T_b_t)
    print("x:",x,"y:",y,"rz:",rz)
    T_guess = T_icp
    ipdb.set_trace()


ipdb.set_trace()
