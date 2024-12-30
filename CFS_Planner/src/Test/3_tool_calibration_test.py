# %%
import open3d as o3d
import ipdb
import numpy as np
from pydrake.all import (
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
# import open3d as o3d
import time
# import teaserpp_python
import copy
import math

# %%
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
def generate_model_pointcloud():

    f = open("EPick.xyz")
    data_lists = f.readline()

    dataset= []
    while data_lists:
        num = list(map(float,data_lists.split()))
        dataset.append(num)
        data_lists = f.readline()
    f.close()
    data_array = np.array(dataset) # [N, 6]
    cloud_val_raw = data_array.T # [6, N]
    cloud_val_raw[[0,1,2],:] = cloud_val_raw[[0,1,2],:] * 0.001 # stl-generated in mm unit

    # convert to point cloud
    cloud_drake_raw = PointCloud(cloud_val_raw.shape[1])
    cloud_drake_raw.mutable_xyzs()[:] = cloud_val_raw[[0,1,2],:]
    
    # assign normals
    cloud_drake_raw.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    assert(cloud_drake_raw.has_normals())
    cloud_drake_raw.mutable_normals()[:] = cloud_val_raw[[3,4,5],:]

    cloud_drake = cloud_drake_raw.VoxelizedDownSample(voxel_size=0.01)

    cloud_val = np.concatenate([cloud_drake.xyzs(), cloud_drake.normals()], axis=0)

    return cloud_drake, cloud_val

def save_cloud(fname, cloud_np):
    '''
        cloud_np: [6, N] np array
    '''
    # save 6d pcd as ply
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np[[0,1,2],:].T)
    pcd.normals = o3d.utility.Vector3dVector(cloud_np[[3,4,5],:].T)
    o3d.io.write_point_cloud(fname, pcd)
    
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

for i in range(1):
    meshcat = StartMeshcat()

    # --------------------- load the bagger model point cloud -------------------- #
    # model pcd model, model pcd values (6, N)
    bagger_model_cloud_drake, bagger_model_cloud_val = generate_model_pointcloud()
    # initial pose guess
    # X_model_default = RigidTransform(RollPitchYaw(np.pi /2, 0, np.pi/6), [0,0,0.64])

    # display
    # meshcat.SetObject("bagger_model", bagger_model_cloud_drake, point_size=0.02, rgba=Rgba(0, 0, 1, 1))
    # meshcat.SetTransform("bagger_model", X_model_default)

    print(f'bagger model size: {bagger_model_cloud_val.shape}')

    # --------------------- load the bagger scene point cloud -------------------- #
    # bagger_scene_cloud_val_raw = np.loadtxt("bagger_scene.txt") # (6, M)
    bagger_scene_cloud_val_raw = np.loadtxt(f"tool_position_test/tool_scene{i}.txt") # (6, M)

    bagger_scene_cloud_drake_raw = PointCloud(bagger_scene_cloud_val_raw.shape[1])
    bagger_scene_cloud_drake_raw.mutable_xyzs()[:] = bagger_scene_cloud_val_raw[[0,1,2],:] # assign xyz
    bagger_scene_cloud_drake_raw.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    assert(bagger_scene_cloud_drake_raw.has_normals())
    bagger_scene_cloud_drake_raw.mutable_normals()[:] = bagger_scene_cloud_val_raw[[3,4,5],:] # assign normals
    # bagger_scene_cloud_drake = bagger_scene_cloud_drake_raw.VoxelizedDownSample(voxel_size=0.01) # downsample
    bagger_scene_cloud_drake = bagger_scene_cloud_drake_raw
    bagger_scene_cloud_val = np.concatenate([bagger_scene_cloud_drake.xyzs(), bagger_scene_cloud_drake.normals()], axis=0)

    # display
    meshcat.SetObject("bagger_scene", bagger_scene_cloud_drake, point_size=0.02, rgba=Rgba(0, 0, 0, 1))

    print(f'bagger scene size: {bagger_scene_cloud_val.shape}')

    # %%
    save_cloud("bagger_model.ply", bagger_model_cloud_val)
    save_cloud("bagger_scene.ply", bagger_scene_cloud_val)
    print(bagger_model_cloud_val.shape)
    print(bagger_scene_cloud_val.shape)

    import open3d as o3d
    import teaserpp_python
    import numpy as np 
    import copy
    from helpers import *

    VOXEL_SIZE = 0.01
    VISUALIZE = False

    # Load and visualize two point clouds from 3DMatch dataset
    A_pcd_raw = o3d.io.read_point_cloud('bagger_model.ply')
    B_pcd_raw = o3d.io.read_point_cloud('bagger_scene.ply')
    A_pcd_raw.paint_uniform_color([0.0, 0.0, 1.0]) # show A_pcd in blue
    B_pcd_raw.paint_uniform_color([1.0, 0.0, 0.0]) # show B_pcd in red
    if VISUALIZE:
        o3d.visualization.draw_geometries([A_pcd_raw,B_pcd_raw]) # plot A and B 

    # voxel downsample both clouds
    A_pcd = A_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    B_pcd = B_pcd_raw.voxel_down_sample(voxel_size=VOXEL_SIZE)
    if VISUALIZE:
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
    # o3d.visualization.draw_geometries([A_pcd,B_pcd,line_set])

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
    # o3d.visualization.draw_geometries([A_pcd_T_teaser,B_pcd])

    meshcat.SetObject("bagger_model_detected", bagger_model_cloud_drake, point_size=0.02, rgba=Rgba(1, 1, 0, 0.5))
    meshcat.SetTransform("bagger_model_detected", RigidTransform(T_teaser))

    T_teaser

    print('Performing ICP...')
    # icp = cv.ppf_match_3d_ICP(1000, 0.01, 2.5, 4)
    # results_ppf = [cv.ppf_match_3d_Pose3D()]
    # results_ppf[0].updatePose(T_teaser)
    
    # # ipdb.set_trace()
    # _, results_icp = icp.registerModelToScene(bagger_model_cloud_val.T, bagger_scene_cloud_val.T, results_ppf)

    # meshcat.SetObject("bagger_model_ICP_refined", bagger_model_cloud_drake, point_size=0.02, rgba=Rgba(0, 1, 0, 1))
    # meshcat.SetTransform("bagger_model_ICP_refined", RigidTransform(results_icp[0].pose))
    
    # meshcat.SetObject("original_model", bagger_model_cloud_drake, point_size=0.02, rgba=Rgba(1, 0, 1, 1))
    # meshcat.SetTransform("original_model", RigidTransform())
    # print(T_teaser)
    # print(results_icp[0].pose)
    
    
    source = A_pcd
    target = B_pcd
    threshold = 0.02
    trans_init = T_teaser
    print("Apply point-to-point ICP")
    # draw_registration_result(source, target, trans_init)

    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    
    print(reg_p2p)
    print("Transformation is:")
    print(T_teaser)
    print(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)
        
    meshcat.SetObject("bagger_model_ICP_refined", bagger_model_cloud_drake, point_size=0.02, rgba=Rgba(0, 1, 0, 1))
    meshcat.SetTransform("bagger_model_ICP_refined", RigidTransform(reg_p2p.transformation))
while 1:
    a = 1


