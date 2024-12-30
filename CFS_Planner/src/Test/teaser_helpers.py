import open3d as o3d
import numpy as np 
from scipy.spatial import cKDTree
import teaserpp_python
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

def save_scene_pl(path, TARGET):
    data = np.load(path)
    np.savetxt(f"data/{TARGET}_test/{TARGET}_scene{0}.txt", data)
    fname = f"data/{TARGET}_test/{TARGET}_scene{0}.txt"
    f = open(fname)
    data_lists = f.readline()

    dataset= []
    while data_lists:
        num = list(map(float,data_lists.split()))
        dataset.append(num)
        data_lists = f.readline()
    f.close()
    data_array = np.array(dataset, dtype=np.float32) # [N, 6]
    scene_val_original = data_array # [6, N]
    scene_drake_original = PointCloud(scene_val_original.shape[1])
    scene_drake_original.mutable_xyzs()[:] = scene_val_original[[0,1,2],:]
    scene_drake_original = scene_drake_original.VoxelizedDownSample(0.02)
    scene_drake_original.EstimateNormals(0.1, 6) # arbitrary, results are overridden

    # just remove the background
    Pallet = True
    if Pallet == True:
        scene_val = scene_val_original[:, scene_val_original[2, :] <= 2.8]
        scene_val = scene_val[:, scene_val[2, :] >= 0.7]
        scene_val = scene_val[:, scene_val[1, :] >= 0.0]
        scene_val = scene_val[:, scene_val[0, :] <= 0.1]
        scene_val = scene_val[:, scene_val[0, :] >= -1.2]
    else:
        scene_val = scene_val_original[:, scene_val_original[2, :] <= 0.8]
        scene_val = scene_val[:, scene_val[1, :] <= 0.25]
        scene_val = scene_val[:, scene_val[1, :] >= -0.25]
        scene_val = scene_val[:, scene_val[0, :] <= 0.25]
        scene_val = scene_val[:, scene_val[0, :] >= -0.25]
    # scene_val = scene_val_original
    scene_drake = PointCloud(scene_val.shape[1])
    scene_drake.mutable_xyzs()[:] = scene_val[[0,1,2],:]
    scene_drake = scene_drake.VoxelizedDownSample(0.01)
    scene_drake.EstimateNormals(0.1, 6) # arbitrary, results are overridden
    data_array = np.zeros([6,scene_drake.normals().shape[1]])
    data_array[[0,1,2],:] = scene_drake.xyzs()
    data_array[[3,4,5],:] = scene_drake.normals()
    np.savetxt(f"data/{TARGET}_test/{TARGET}_scene{0}.txt", data_array)
    
    return scene_drake_original
  
def pcd2xyz(pcd):
    return np.asarray(pcd.points).T

def extract_fpfh(pcd, voxel_size):
  radius_normal = voxel_size * 2
  pcd.estimate_normals(
      o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

  radius_feature = voxel_size * 5
  fpfh = o3d.pipelines.registration.compute_fpfh_feature(
      pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
  return np.array(fpfh.data).T

def find_knn_cpu(feat0, feat1, knn=1, return_distance=False):
  feat1tree = cKDTree(feat1)
  dists, nn_inds = feat1tree.query(feat0, k=knn, workers=-1)
  if return_distance:
    return nn_inds, dists
  else:
    return nn_inds

def find_correspondences(feats0, feats1, mutual_filter=True):
  nns01 = find_knn_cpu(feats0, feats1, knn=1, return_distance=False)
  corres01_idx0 = np.arange(len(nns01))
  corres01_idx1 = nns01

  if not mutual_filter:
    return corres01_idx0, corres01_idx1

  nns10 = find_knn_cpu(feats1, feats0, knn=1, return_distance=False)
  corres10_idx1 = np.arange(len(nns10))
  corres10_idx0 = nns10

  mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
  corres_idx0 = corres01_idx0[mutual_filter]
  corres_idx1 = corres01_idx1[mutual_filter]

  return corres_idx0, corres_idx1

def get_teaser_solver(noise_bound):
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1.0
    solver_params.noise_bound = noise_bound
    solver_params.estimate_scaling = False
    solver_params.inlier_selection_mode = \
        teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
    solver_params.rotation_tim_graph = \
        teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
    solver_params.rotation_estimation_algorithm = \
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 10000
    solver_params.rotation_cost_threshold = 1e-16
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    return solver

def Rt2T(R,t):
    T = np.identity(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T 