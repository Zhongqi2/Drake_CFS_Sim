# %%
from tqdm import tqdm
import cfspy
import numpy as np
import scipy.spatial.transform as st
import snap7
from snap7.util import *
import struct
import time
import ctypes
# from environment_calibration import *
np.set_printoptions(precision=3, suppress=True)

# %%
client = snap7.client.Client()

plc_ip = '192.168.1.12'
rack = 0
slot = 1
client.connect(plc_ip, rack, slot)

# %%
CFS_traj, critical_traj, CFS_success = cfspy.CFSPickPlace(
    np.deg2rad(UserData_arr[6]), np.eye(4), "pick_to_place", "gp7", 0.30, 0.30, 0.30, 200, True, np.array([0.0, 0.0, 0.0]).T, np.array([0.0, 1.0, 0.0]).T, np.deg2rad(15.0))

# %%
t_init = time.time()
t_arr = []
CommandedPosition_arr = []
UserData_arr = []
# %%
MLX_DB_number = 7003
MLX_SystemState_offset = 0
MLX_SystemState_size = 4  # 4(DInt)
MLX_RobotAxes_offset = 10334
MLX_RobotAxes_size = 228
MLX_CommandedPosition_offset = 10546 - 10334
MLX_CommandedPosition_size = 4  # 4(Real)
MLX_UserData_offset = 10550 - 10334
MLX_UserData_size = 4  # 4(Real)
MLX_num_axes = 6

while True:
    state_raw = client.db_read(
        MLX_DB_number, MLX_SystemState_offset, MLX_SystemState_size)
    state = struct.unpack('>I', state_raw)[0]
    if state != 4:  # "RUNNING"
        if len(t_arr) > 0:
            break   
            pass
        else:
            continue
    else:
        if len(t_arr) == 0:
            print("Start recording")

    CommandedPosition_q = np.zeros(MLX_num_axes)
    UserData_q = np.zeros(MLX_num_axes)
    for j in range(MLX_num_axes):
        CommandedPosition_raw = client.db_read(
            MLX_DB_number, MLX_RobotAxes_offset + j * MLX_RobotAxes_size + MLX_CommandedPosition_offset, MLX_CommandedPosition_size)
        CommandedPosition = np.frombuffer(CommandedPosition_raw, dtype='>f')
        CommandedPosition_q[j] = CommandedPosition[0]

        UserData_raw = client.db_read(
            MLX_DB_number, MLX_RobotAxes_offset + j * MLX_RobotAxes_size + MLX_UserData_offset, MLX_UserData_size)
        UserData = np.frombuffer(UserData_raw, dtype='>f')
        UserData_q[j] = UserData[0]

    t_arr.append(time.time() - t_init)
    CommandedPosition_arr.append(CommandedPosition_q)
    UserData_arr.append(UserData_q)

    # break


 # %%
np.save("/home/MASCEI/Auto_calibration/DecData/pickplaceconveyor1205_record/cfstrajj_t_arr", np.array(t_arr))
np.save("/home/MASCEI/Auto_calibration/DecData/pickplaceconveyor1205_record/cfstrajj_CommandedPosition_arr", np.array(CommandedPosition_arr))
np.save("/home/MASCEI/Auto_calibration/DecData/pickplaceconveyor1205_record/cfstrajj_UserData_arr", np.array(UserData_arr))


# %%

######### Container 1 config ##################
# %%
DB_number = 7004
TargetType_offset = 0
TargetType_size = 1
TargetType = struct.unpack('>?', client.db_read(
    DB_number, TargetType_offset, TargetType_size))
print(TargetType)

# %%
Comu_Trajectory_offset = 2
Comu_Trajectory_size = 1728
Trajectory_offset = 0
Trajectory_size = 1728
Points_offset = 0
Points_size = 192
TCPPosition_offset = 84
TCPPosition_size = 32  # 4(Real) * 8
AxisPosition_offset = 152
AxisPosition_size = 32  # 4(Real) * 8

num_trajectory = Comu_Trajectory_size // Trajectory_size
num_points = Trajectory_size // Points_size
# for i in range(num_trajectory):
print(f"num_trajectory: {num_trajectory}")
print(f"num_points: {num_points}")
######### Container 2 config ##################

######### Container 2 config ##################
# %%
DB_number = 5
TargetType_offset = 0
TargetType_size = 1
TargetType = struct.unpack('>?', client.db_read(
    DB_number, TargetType_offset, TargetType_size))
print(TargetType)

# %%
Comu_Trajectory_offset = 2
Comu_Trajectory_size = 34560
Trajectory_offset = 0
Trajectory_size = 1728
Points_offset = 0
Points_size = 192
TCPPosition_offset = 84
TCPPosition_size = 32  # 4(Real) * 8
AxisPosition_offset = 152
AxisPosition_size = 32  # 4(Real) * 8

num_trajectory = Comu_Trajectory_size // Trajectory_size
num_points = Trajectory_size // Points_size
# for i in range(num_trajectory):
print(f"num_trajectory: {num_trajectory}")
print(f"num_points: {num_points}")
######### Container 2 config ##################


# %% Read
TCPPosition_arr = []
AxisPosition_arr = []
for tn in range(num_trajectory):
    TCPPosition_pts = []
    AxisPosition_pts = []
    for pn in range(num_points):
        TCPPosition_raw = client.db_read(DB_number, Comu_Trajectory_offset + tn *
                                         Trajectory_size + pn * Points_size + TCPPosition_offset, TCPPosition_size)
        TCPPosition = np.frombuffer(TCPPosition_raw, dtype='>f')
        TCPPosition_pts.append(TCPPosition)
        # print(np.array(TCPPosition))

        AxisPosition_raw = client.db_read(DB_number, Comu_Trajectory_offset + tn *
                                          Trajectory_size + pn * Points_size + AxisPosition_offset, AxisPosition_size)
        AxisPosition = np.frombuffer(AxisPosition_raw, dtype='>f')
        AxisPosition_pts.append(AxisPosition)
        # print(np.array(AxisPosition))

    TCPPosition_arr.append(TCPPosition_pts)
    AxisPosition_arr.append(AxisPosition_pts)
# %%
np.save("/home/MASCEI/Auto_calibration/DecData/boxpacking1211_chengtaotuned/TCPPosition_arr.npy", TCPPosition_arr)
np.save("/home/MASCEI/Auto_calibration/DecData/boxpacking1211_chengtaotuned/AxisPosition_arr.npy", AxisPosition_arr)
# np.save("trajj.npy", trajj)
# np.save("trajt.npy", trajt)

# %%
client.db_write(DB_number, TargetType_offset, struct.pack('>?', True))
print(struct.unpack('>?', client.db_read(
    DB_number, TargetType_offset, TargetType_size)))

# %%
TCPPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/palletizing1204_afterteach/TCPPosition_arr.npy", allow_pickle=True)
AxisPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/palletizing1204_afterteach/AxisPosition_arr.npy", allow_pickle=True)
trajj = np.load("/home/MASCEI/Auto_calibration/DecData/palletizing1204_afterteach/trajj_optimized.npy", allow_pickle=True)
trajt = np.load("/home/MASCEI/Auto_calibration/DecData/palletizing1204_afterteach/trajt_optimized.npy", allow_pickle=True)
# trajj[8][5][:6]

# %%
target_to_hand_mat = np.loadtxt("/mnt/storage/target_to_hand_mat.txt")
target_to_hand_mat_nominal = np.loadtxt("/mnt/storage/target_to_hand_mat_nominal.txt")
np.rad2deg(np.norm(st.Rotation.from_matrix(target_to_hand_mat[:3, :3]).as_rotvec()))
# %%
## Robot 1: only modify point 7 (trajj index 5)
tn = 0
pn = 4
pn_plc = 6
critical_q = trajj[tn][pn]
critical_q = np.hstack([critical_q, np.zeros(2)])
AxisPosition = np.array(critical_q, dtype=np.float32)
AxisPosition_raw = struct.pack('>ffffffff', *AxisPosition)
# print(Comu_Trajectory_offset + tn *
#                 Trajectory_size + pn * Points_size + AxisPosition_offset)
client.db_write(DB_number, Comu_Trajectory_offset + tn *
                Trajectory_size + pn_plc * Points_size + AxisPosition_offset, AxisPosition_raw)
# %% Write to PLC
client.db_write(DB_number, TargetType_offset, struct.pack('>?', False))
for tn, critical_traj in enumerate(trajj[:, :, :]):
    for pn, critical_q in enumerate(critical_traj):
        critical_q = np.hstack([critical_q, np.zeros(2)])
        AxisPosition = np.array(critical_q, dtype=np.float32)
        AxisPosition_raw = struct.pack('>ffffffff', *AxisPosition)
        # print(Comu_Trajectory_offset + tn *
        #                 Trajectory_size + pn * Points_size + AxisPosition_offset)
        client.db_write(DB_number, Comu_Trajectory_offset + tn *
                        Trajectory_size + pn * Points_size + AxisPosition_offset, AxisPosition_raw)
# %% Write AxisPosition_arr to PLC
client.db_write(DB_number, TargetType_offset, struct.pack('>?', False))
for tn, critical_traj in enumerate(AxisPosition_arr):
    for pn, AxisPosition in enumerate(critical_traj):
        AxisPosition_raw = AxisPosition.astype('>f').tobytes()
        client.db_write(DB_number,
                        Comu_Trajectory_offset + tn * Trajectory_size + pn * Points_size + AxisPosition_offset, AxisPosition_raw)
# %%        
trajj[0][6]
 # %%
# %% Write TCPPosition_arr to PLC
client.db_write(DB_number, TargetType_offset, struct.pack('>?', True))
for tn, critical_traj in enumerate(TCPPosition_arr):
    for pn, TCPPosition in enumerate(critical_traj):
        # if (tn in [1, 3, 5, 7]) and (pn in [5, 6, 7]):
        #     TCPPosition[1] -= 130 
        #     print(TCPPosition[1])
        TCPPosition_raw = TCPPosition.astype('>f').tobytes()
        client.db_write(DB_number,
                        Comu_Trajectory_offset + tn * Trajectory_size + pn * Points_size + TCPPosition_offset, TCPPosition_raw)

 # %%
print(Comu_Trajectory_offset + 0 * Trajectory_size +
      6 * Points_size + TCPPosition_offset)

# # %%

client.disconnect()

# %%


# %%
julia_q.shape

# %%
# /home/MASCEI/Auto_calibration/scripts/realdata/boxpacking09271635_record/TCPPosition_arr_optimized_debug.npy: Interpolation Invalid @ 2, 9 nd place
# /home/MASCEI/Auto_calibration/scripts/realdata/boxpacking09271635_record/trajj_optimized.npy: stuck