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

def get_angular_error(R_gt, R_est):
    """
    Get angular error
    """
    try:
        A = (np.trace(np.dot(R_gt.T, R_est))-1) / 2.0
        if A < -1:
            A = -1
        if A > 1:
            A = 1
        rotError = math.fabs(math.acos(A));
        return math.degrees(rotError)
    except ValueError:
        import pdb; pdb.set_trace()
        return 99999

def compute_transformation_diff(est_mat, gt_mat):
    """
    Compute difference between two 4-by-4 SE3 transformation matrix
    """
    R_gt = gt_mat[:3,:3]
    R_est = est_mat[:3,:3]
    rot_error = get_angular_error(R_gt, R_est)

    t_gt = gt_mat[:,-1]
    t_est = est_mat[:,-1]
    trans_error = np.linalg.norm(t_gt - t_est)

    return rot_error, trans_error

# read stl-generated model pcd
def load_model_pointcloud(fname):

    f = open(fname)
    data_lists = f.readline()

    dataset= []
    while data_lists:
        num = list(map(float,data_lists.split()))
        dataset.append(num)
        data_lists = f.readline()
    f.close()
    data_array = np.array(dataset, dtype=np.float32) # [N, 6]
    model_val = data_array.T # [6, N]
    # model_val[[0,1,2],:] = model_val[[0,1,2],:] * 0.001 # stl-generated in mm unit
    model_val[[0,1,2],:] = model_val[[0,1,2],:] 

    model_drake = PointCloud(model_val.shape[1])
    model_drake.mutable_xyzs()[:] = model_val[[0,1,2],:]
    model_drake.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    assert(model_drake.has_normals())
    model_drake.mutable_normals()[:] = model_val[[3,4,5],:]

    return model_drake, model_val

def load_model_pointcloud_xyz(fname):

    f = open(fname)
    data_lists = f.readline()

    dataset= []
    while data_lists:
        num = list(map(float,data_lists.split()))
        dataset.append(num)
        data_lists = f.readline()
    f.close()
    data_array = np.array(dataset, dtype=np.float32) # [N, 6]
    model_val = data_array.T # [6, N]
    model_val[[0,1,2],:] = model_val[[0,1,2],:] * 0.0091 # stl-generated in mm unit
    
    # model_val = model_val[:, model_val[2, :] <= 0.1] 
    p_4 = [0,0,0] #put model in random position

    X_O = RigidTransform(RotationMatrix(), p_4)
    p_s = X_O.multiply(model_val[[0,1,2],:] )

    # model_val[[0,1,2],:] = p_s 
    # model_val = model_val[:, model_val[2, :] <= 0.1]  # only get some part of model c1r2
    # model_val = model_val[:, model_val[2, :] <= 2.5]  # only get some part of model c1r2
    # model_val = model_val[:, model_val[2, :] <= 0.1]  # only get some part of model c1r1
    # ipdb.set_trace()
    model_drake = PointCloud(model_val.shape[1])
    model_drake.mutable_xyzs()[:] = model_val[[0,1,2],:]
    # model_drake = model_drake.VoxelizedDownSample(0.02)
    model_drake.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    assert(model_drake.has_normals())
    model_drake.mutable_normals()[:] = model_val[[3,4,5],:]
    model_drake = model_drake.VoxelizedDownSample(0.01)

    return model_drake, model_val,X_O
# read simulated scene pcd
def load_scene_pointcloud(fname, sigma=None):
    scene_val = np.loadtxt(fname, dtype=np.float32) # (6, M)
    if sigma is not None:
        scene_val = scene_val + np.random.normal(scale=sigma, size=scene_val.shape).astype(np.float32)
    scene_drake = PointCloud(scene_val.shape[1])
    scene_drake.mutable_xyzs()[:] = scene_val[[0,1,2],:] # assign xyz
    scene_drake.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    assert(scene_drake.has_normals())
    scene_drake.mutable_normals()[:] = scene_val[[3,4,5],:] # assign normals

    return scene_drake, scene_val

def save_cloud(fname, model_cloud):
    '''
        cloud_np: [6, N] np array
    '''
    # save 6d pcd as ply

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(model_cloud[[0,1,2],:].T)
    pcd.normals = o3d.utility.Vector3dVector(model_cloud[[3,4,5],:].T)
    o3d.io.write_point_cloud(fname, pcd)

def teaserpp_registration(model_ply, scene_ply, viz=False):

    VOXEL_SIZE = 0.03

    # Load and visualize two point clouds from 3DMatch dataset
    A_pcd_raw = o3d.io.read_point_cloud(model_ply)
    B_pcd_raw = o3d.io.read_point_cloud(scene_ply)
    A_pcd_raw.paint_uniform_color([0.0, 0.0, 1.0]) # show A_pcd in blue
    B_pcd_raw.paint_uniform_color([1.0, 0.0, 0.0]) # show B_pcd in red
    if viz:
        o3d.visualization.draw_geometries([A_pcd_raw,B_pcd_raw]) # plot A and B 

    # voxel downsample both clouds
    A_pcd = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    B_pcd = B_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    if viz:
        o3d.visualization.draw_geometries([A_pcd,B_pcd]) # plot downsampled A and B 

    A_xyz = pcd2xyz(A_pcd) # np array of size 3 by N
    B_xyz = pcd2xyz(B_pcd) # np array of size 3 by M

    # extract FPFH features
    A_feats = extract_fpfh(A_pcd,VOXEL_SIZE)
    B_feats = extract_fpfh(B_pcd,VOXEL_SIZE)

    # establish correspondences by nearest neighbour search in feature space
    corrs_A, corrs_B = find_correspondences(
        A_feats, B_feats, mutual_filter=True)
    A_corr = A_xyz[:,corrs_A] # np array of size 3 by num_corrs
    B_corr = B_xyz[:,corrs_B] # np array of size 3 by num_corrs

    num_corrs = A_corr.shape[1]
    print(f'FPFH generates {num_corrs} putative correspondences.')

    # visualize the point clouds together with feature correspondences
    points = np.concatenate((A_corr.T,B_corr.T),axis=0)
    lines = []
    for i in range(num_corrs):
        lines.append([i,i+num_corrs])
    colors = [[0, 1, 0] for i in range(len(lines))] # lines are shown in green
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    if viz:
        o3d.visualization.draw_geometries([A_pcd,B_pcd,line_set])

    # robust global registration using TEASER++
    NOISE_BOUND = VOXEL_SIZE
    teaser_solver = get_teaser_solver(NOISE_BOUND)
    teaser_solver.solve(A_corr,B_corr)
    solution = teaser_solver.getSolution()
    R_teaser = solution.rotation
    t_teaser = solution.translation
    T_teaser = Rt2T(R_teaser,t_teaser)

    # Visualize the registration results
    A_pcd_T_teaser = copy.deepcopy(A_pcd).transform(T_teaser)
    if viz:
        o3d.visualization.draw_geometries([A_pcd_T_teaser,B_pcd])

    return T_teaser

def icp(T_init, model_val, scene_val):
    '''
        model_val [M, 6]
        scene_val [M, 6]
    '''
    print('Performing ICP...')
    icp = cv.ppf_match_3d_ICP(100, 0.01, 2.5, 6)
    results_ppf = [cv.ppf_match_3d_Pose3D()]
    results_ppf[0].updatePose(T_init)
    _, results_icp = icp.registerModelToScene(model_val, scene_val, results_ppf)
    T_icp = results_icp[0].pose

    return T_icp