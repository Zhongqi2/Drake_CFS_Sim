# %%
import cfspy
import numpy as np
np.set_printoptions(precision=3, suppress=True)

trajj = np.load("realdata/pickplaceconveyor0928_manual/trajj.npy",
                allow_pickle=True)
trajt = np.load("realdata/pickplaceconveyor0928_manual/trajt.npy",
                allow_pickle=True)

robot_model = "gp7"

if robot_model == "gp7":
    q_home = np.deg2rad(
        np.array([43.014,  34.623,   2.181,   0.08, -57.485,  46.943]))
elif robot_model == "gp12":
    q_home = np.deg2rad(np.array([-0.62, 25.42, -36.10, -0.04, -28.56, -89.36]))


# %% testing
tn=0
home_q=trajj[tn][0]
home_T=trajt[tn][0]
pick_T=trajt[tn][2]
place_T=trajt[tn][6]

print("home_T: ", home_T)
# Home to Pick
CFS_traj, critical_qs, CFS_success=cfspy.CFSPickPlace(
    q_home, pick_T, "home_to_pick", robot_model, 0.05, 0.05, 0.05, 400, False, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 0.0, 0.0]).T, 0)
exit(0)

# %%
critical_trajs=[]
# for tn in range(num_trajectory):
for tn in (range(1)):
    critical_traj=[]
    home_T=trajt[tn][0]
    pick_T=trajt[tn][2]
    place_T=trajt[tn][6]

    # Home to Pick
    CFS_traj, critical_qs, CFS_success=cfspy.CFSPickPlace(
        q_home, pick_T, "home_to_pick", robot_model, 0.05, 0.05, 0.05, 200, False, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 0.0, 0.0]).T, 0)
    if not CFS_success:
        print("CFS Home to Pick failed at tn: ", tn)
        break
    critical_qs=critical_qs.T.tolist()
    critical_traj += critical_qs
    print(critical_qs[-1])


    # Pick to Place
    q_pick=np.array(critical_qs[-1], dtype=np.float64).reshape(6, 1)
    pose_place=np.array(place_T, dtype=np.float64)
    CFS_traj, critical_qs, CFS_success=cfspy.CFSPickPlace(
        q_pick, pose_place, "pick_to_place", robot_model, 0.05, 0.05, 0.30, 800, False, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 0.0, 0.0]).T, 0)
    if not CFS_success:
        print("CFS pick_to_place failed at tn: ", tn)
        break
    critical_qs=critical_qs.T.tolist()
    critical_qs=[i for j, i in enumerate(critical_qs) if j not in [5]]
    critical_traj += critical_qs
    print(critical_qs[-1])

    # Place to Home
    q_place=np.array(critical_qs[-1], dtype=np.float64).reshape(6, 1)
    CFS_traj, critical_qs, CFS_success=cfspy.CFSPickPlace(
        q_place, home_T, "place_to_home", robot_model, 0.10, 0.40, 0.10, 400, False, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 0.0, 0.0]).T, 0)
    if not CFS_success:
        print("CFS place_to_home failed at tn: ", tn)
        break
    critical_qs=critical_qs.T.tolist()
    critical_traj += critical_qs

    # Remove Duplicate
    remove_indices=[2, 6]
    critical_traj=[i for j, i in enumerate(
        critical_traj) if j not in remove_indices]

    critical_trajs.append(critical_traj)
