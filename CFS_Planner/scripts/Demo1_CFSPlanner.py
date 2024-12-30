import cfspy
import numpy as np

if __name__ == '__main__':
    
    # ------------------------- replace with actual path ------------------------- #
    robot_model = "gp7"    # "gp7", "gp12"   
    
    # ------------- move from q_home (joint) to pose_pick (cartesian) ------------ #
    q_home = np.array([0.0, 0.0, 0.0, 0.0, -np.pi/2, 0.0], dtype=np.float64).reshape(6, 1)
    pose_pick = np.array([
            [-1, 0, 0, 0.48],
            [0, -1, 0, 0],
            [0, 0, 1, 0.335],
            [0, 0, 0, 1]
    ], dtype=np.float64)
    CFS_traj, critical_traj, CFS_success = cfspy.CFSPickPlace(
        q_home, pose_pick, "home_to_pick", robot_model, 0.05, 0.05, 0.05, 200, False, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 0.0, 0.0]).T, 0)
    print(f"CFS success: {CFS_success}")
    print(f"Sparse trajectory: \n{critical_traj}")