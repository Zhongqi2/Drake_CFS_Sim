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

TARGET = 'container' # 'bagger' 'tool' 'conveyor'
TARGET_PCD = 'container' # 'Bagging Machine' 'EPick' 'conveyor'
N_POS = 1 # num of camera positions
N_ANG = 1 # num of camera view angles
N_NOISE_REPEAT = 3 # num of noisy observations

VIZ = True

if __name__ == '__main__':

    if VIZ:
        meshcat = StartMeshcat()

    cam_xs = [] # camera x coordinates
    cam_as = [] # camera x rotations
    sigmas = [0.0, 0.001, 0.002, 0.003, 0.005] # std of gaussian noise
    
    # model pcd model, model pcd values (6, N)
    model_drake, model_val = load_model_pointcloud(f"config/geometry_wrappers/{TARGET_PCD}.xyz")
    print(f'model size: {model_val.shape}')

    # load ground truth target pose
    T_gt = np.loadtxt(f'data/{TARGET}_test/{TARGET}_pose.txt')

    # results
    rot_err_all     = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    trans_err_all   = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    time_teaser_all = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))
    time_icp_all    = np.zeros(shape=(N_POS, N_ANG, len(sigmas), N_NOISE_REPEAT))

    for pos_i in range(N_POS):

        for ang_i in range(N_ANG):
            
            data_id = pos_i*N_POS + ang_i

            # parse camera pos
            mat_w2c = np.loadtxt(osp.join(f'data/{TARGET}_test/camera_pose{data_id}.txt'))
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
                    
                    print(f'\n------- {pos_i} th pos, {ang_i} th ang, {sigma_i} th sigma, {noise_i} th exp -------\n')

                    # scene pcd model, scene pcd values (6, M)
                    scene_drake, scene_val = load_scene_pointcloud(f"data/{TARGET}_test/{TARGET}_scene{data_id}.txt", sigma=sigma)

                    # display
                    if VIZ:
                        meshcat.SetObject(f"{TARGET}_scene", scene_drake, point_size=0.02, rgba=Rgba(0, 0, 0, 1))

                    # save as ply
                    save_cloud(f"{TARGET}_model.ply", model_val)
                    save_cloud(f"{TARGET}_scene.ply", scene_val)

                    # global detection via teaser++
                    start = time.time()
                    T_teaser = teaserpp_registration(f'{TARGET}_model.ply', f'{TARGET}_scene.ply', viz=VIZ)
                    time_teaser_all[pos_i, ang_i, sigma_i, noise_i] = time.time() - start
                    if VIZ:
                        meshcat.SetObject(f"{TARGET}_model_detected", model_drake, point_size=0.02, rgba=Rgba(1, 1, 0, 0.5))
                        meshcat.SetTransform(f"{TARGET}_model_detected", RigidTransform(T_teaser))

                    # fine tuning via icp
                    start = time.time()
                    T_icp = icp(T_teaser, model_val.T, scene_val.T)
                    time_icp_all[pos_i, ang_i, sigma_i, noise_i] = time.time() - start
                    if VIZ:
                        meshcat.SetObject(f"{TARGET}_model_ICP_refined", model_drake, point_size=0.02, rgba=Rgba(0, 1, 0, 1))
                        meshcat.SetTransform(f"{TARGET}_model_ICP_refined", RigidTransform(T_icp))

                    # calculate registration error
                    rot_err, trans_err = compute_transformation_diff(T_icp, T_gt)
                    print(f'rot err: {rot_err}, trans err: {trans_err}')

                    # log errors
                    rot_err_all[pos_i, ang_i, sigma_i, noise_i]   = rot_err
                    trans_err_all[pos_i, ang_i, sigma_i, noise_i] = trans_err

    np.save(f'data/{TARGET}_test/rot_err_all.npy', rot_err_all)
    np.save(f'data/{TARGET}_test/trans_err_all.npy', trans_err_all)
    np.save(f'data/{TARGET}_test/camera_x_coords.npy', cam_xs)
    np.save(f'data/{TARGET}_test/camera_x_angles.npy', cam_as)
    np.save(f'data/{TARGET}_test/noise_sigmas.npy', sigmas)

    while 1:
        print('done')
        pass