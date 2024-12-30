## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import numpy as np
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

def get_scene_pcd():
    intrinsic_inv = np.linalg.inv(np.matrix([[917.5706787109375, 0.0, 645.0847778320312],
                                             [0.0, 917.9429931640625, 372.301513671875], 
                                             [0.0, 0.0, 1.0]]))
    
    with open('config/cam_calibration/845112072047/hand_to_eye_calib_mat.txt', 'r') as infile:
        lines = infile.readlines()
        extrinsic_mat = np.array([[float(val) for val in line[:-1].split(',')] for line in lines[:3]])
        extrinsic_mat = np.concatenate([extrinsic_mat, np.array([[0, 0, 0, 1]])], axis=0)

    print('Hand eye calib mat:')
    print(extrinsic_mat)

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
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

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

    # Streaming loop
    try:
        while True:

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

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
            return pt_world[:3, :]
        
    finally:
        pipeline.stop()

TARGET = 'bagger' # 'bagger' 'tool'
TARGET_PCD = 'Bagging Machine' # 'Bagging Machine' 'EPick'
VIZ = True

def run_environment_calibration():
    # Start the visualizer.
    meshcat = StartMeshcat()

    meshcat.Delete()
    
    # model pcd model, model pcd values (6, N)
    model_drake, model_val = load_model_pointcloud(f"config/geometry_wrappers/{TARGET_PCD}.xyz")
    print(f'model size: {model_val.shape}')
    
    seq = 0
    # while True:

    seq = seq + 1

    scene_val = get_scene_pcd() # [3, M]
    scene_val = np.array(scene_val)
    scene_drake = PointCloud(scene_val.shape[1])
    scene_drake.mutable_xyzs()[:] = scene_val
    scene_drake = scene_drake.VoxelizedDownSample(voxel_size=0.04)
    scene_drake.EstimateNormals(0.1, 6)
    scene_val = np.concatenate([scene_drake.xyzs(), scene_drake.normals()], axis=0) # [6, M]

    if VIZ:
        meshcat.SetObject(f"{TARGET}_scene", scene_drake, point_size=0.02, rgba=Rgba(0, 0, 0, 1))
    
    # load scene model
    save_cloud(f"{TARGET}_model.ply", model_val)
    save_cloud(f"{TARGET}_scene.ply", scene_val)
    
    # global detection via teaser++
    T_teaser = teaserpp_registration(f'{TARGET}_model.ply', f'{TARGET}_scene.ply', viz=VIZ)

    if VIZ:
        meshcat.SetObject(f"{TARGET}_model_detected", model_drake, point_size=0.02, rgba=Rgba(1, 1, 0, 0.5))
        meshcat.SetTransform(f"{TARGET}_model_detected", RigidTransform(T_teaser))

    # fine tuning via icp
    T_icp = icp(T_teaser, model_val.T, scene_val.T)
    print(T_icp)
    
    if VIZ:
        meshcat.SetObject(f"{TARGET}_model_ICP_refined", model_drake, point_size=0.02, rgba=Rgba(0, 1, 0, 1))
        meshcat.SetTransform(f"{TARGET}_model_ICP_refined", RigidTransform(T_icp))
        
    return T_icp

    
if __name__ == '__main__':
    result = run_environment_calibration()