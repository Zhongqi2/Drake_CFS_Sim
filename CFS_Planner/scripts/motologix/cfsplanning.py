# %%
import numpy as np
import cfspy
import modern_robotics as mr
import scipy.spatial.transform as st
np.set_printoptions(precision=3, suppress=True)
# load npy


def gp7_fk(thetalist):
    M = np.array([[0, 0,  -1, 0.56],
                 [ 0, -1,  0, 0.0],
                 [ -1, 0, 0, 0.485],
                 [ 0, 0,  0, 1]])

    w1 = np.array([0, 0,  1])
    w2 = np.array([0, 1,  0])
    w3 = np.array([0, -1,  0])
    w4 = np.array([-1, 0,  0])
    w5 = np.array([0, -1,  0])
    w6 = np.array([-1, 0,  0])

    p1 = np.array([0, 0,  0])
    p2 = np.array([0.04, 0,  0.0])
    p3 = np.array([0.04, 0,  0.485])
    p4 = np.array([0.04, 0,  0.485])
    p5 = np.array([0.480, 0,  0.485])
    p6 = np.array([0.56, 0,  0.485])

    Slist1 = np.concatenate([w1,-np.cross(w1, p1)])
    Slist2 = np.concatenate([w2,-np.cross(w2, p2)])
    Slist3 = np.concatenate([w3,-np.cross(w3, p3)])
    Slist4 = np.concatenate([w4,-np.cross(w4, p4)])
    Slist5 = np.concatenate([w5,-np.cross(w5, p5)])
    Slist6 = np.concatenate([w6,-np.cross(w6, p6)])
    Slist = np.vstack([Slist1, Slist2, Slist3, Slist4, Slist5, Slist6]).T

    T = mr.FKinSpace(M, Slist, thetalist)

    return T

def gp7_ik(T,q0):
    M = np.array([[0, 0,  -1, 0.56],
                 [ 0, -1,  0, 0.0],
                 [ -1, 0, 0, 0.485],
                 [ 0, 0,  0, 1]])

    w1 = np.array([0, 0,  1])
    w2 = np.array([0, 1,  0])
    w3 = np.array([0, -1,  0])
    w4 = np.array([-1, 0,  0])
    w5 = np.array([0, -1,  0])
    w6 = np.array([-1, 0,  0])

    p1 = np.array([0, 0,  0])
    p2 = np.array([0.04, 0,  0.0])
    p3 = np.array([0.04, 0,  0.485])
    p4 = np.array([0.04, 0,  0.485])
    p5 = np.array([0.480, 0,  0.485])
    p6 = np.array([0.56, 0,  0.485])

    Slist1 = np.concatenate([w1,-np.cross(w1, p1)])
    Slist2 = np.concatenate([w2,-np.cross(w2, p2)])
    Slist3 = np.concatenate([w3,-np.cross(w3, p3)])
    Slist4 = np.concatenate([w4,-np.cross(w4, p4)])
    Slist5 = np.concatenate([w5,-np.cross(w5, p5)])
    Slist6 = np.concatenate([w6,-np.cross(w6, p6)])
    Slist = np.vstack([Slist1, Slist2, Slist3, Slist4, Slist5, Slist6]).T

    eomg = 0.01
    ev = 0.001
    q = mr.IKinSpace(Slist, M, T, q0, eomg, ev)

    return q

def fix_gp7_pose(pose):
    # q2 = np.array([0,0,0,0,0,0])
    # t2 = np.array([0.560,0.0,0.485,180,-90,0])

    # cfs_T = cfspy.FKine(np.deg2rad(q2), "gp7")
    # yaskawa_rot = st.Rotation.from_euler(
    #     'xyz',  t2[3:6], degrees=True)
    # yaskawa_trans = t2[0:3]
    # correction_yaskawa2cfs_rot = cfs_T[:3, :3] @ yaskawa_rot.inv().as_matrix()
    # correction_yaskawa2cfs_trans = -yaskawa_trans + cfs_T[:3, 3]

    # pose[:3, :3] = correction_yaskawa2cfs_rot @ pose[:3, :3]
    # pose[:3, 3] = correction_yaskawa2cfs_trans + pose[:3, 3]
    # return pose

    # pose[:3, :3] = np.array([[-1,0,0],[0,1,0],[0,0,-1]]) @ pose[:3, :3]
    # pose[:3, 3] = np.array([0,0,0]) + pose[:3, 3]

    T_fixed = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
    T3 = T_fixed @ pose
    return T3
# %%
q0 = np.array([1.57,0,0,0,0,0])
print(q0)
t0 = cfspy.FKine(q0, "gp7")
print(t0)
q1 = cfspy.IKine(q0, t0, "gp7", 1e-2, 1e-2, 100)
print(q1)
t1 = cfspy.FKine(q1, "gp7")
print(t1)
# %%
q0 = np.array([0,-1.57,0,0,0,0])
print(q0)
t0 = cfspy.FKine(q0, "gp7")
print(t0)
q1 = cfspy.IKine(q0, t0, "gp7", 1e-2, 1e-2, 100)
print(q1)
t1 = cfspy.FKine(q1, "gp7")
print(t1)

# %%

np.linalg.inv(np.array([[ 0, 0, -1, -0.07],
          [0, -1, 0, 0],
          [-1, 0, 0, -0.4],
          [0, 0, 0, 1]]))


# %%
q0 = [0, 0, 0, 0, 0, 0]
print(gp7_fk(q0))
t0 = cfspy.FKine(q0, "gp7")
print(t0)
# correction = np.linalg.inv(np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))
# q1 = cfspy.IKine(q0, t0, "gp7", 1e-2, 1e-2, 100)
# print(q1)
# %%
print(gp7_fk(np.array([-0.002, -0.016, -0.027, 0.302, 0.012, -0.3])))
# %%
TCPPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/boxpacking1209/TCPPosition_arr.npy")
AxisPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/boxpacking1209/AxisPosition_arr.npy")
t2 = TCPPosition_arr[1][5][:6]
T2 = np.eye(4)
T2[:3, 3] = t2[0:3] * 1.0e-3
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(T2)
correction = np.linalg.inv(np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))
print(np.rad2deg(cfspy.IKine(np.deg2rad([9.25, 18.50, -0.2, -14.45, -15.49, -27.07]), T2 @ correction, "gp7", 1e-2, 1e-2, 100)))
# %%

# %%
# q0 = np.deg2rad(np.array([18.0815, 70.6080, -3.6887, 0.1312, -15.6088, 69.4935]))  
q0 = np.deg2rad(np.array([33.3577, 38.9660, -4.0192, 0.2050, -46.7385, 55.4162]))  
print(q0)
# q0 = np.deg2rad(np.array([0,0,0,0,0,0]))
# cfsfk = np.array([
#         [1,0,0,0],[0,1,0,0],[0,0,1,-0.33],[0,0,0,1]]) @ cfspy.FKine(q0, "gp7") @ np.array([
#         [0, 0, -1, -0.07],
#         [0, -1, 0, 0],
#         [-1, 0, 0, -0.4],
#         [0, 0, 0, 1]])
cfsfk = cfspy.FKine(q0, "gp7")
mrfk = gp7_fk(q0)
print(cfsfk)
# print(fix_gp7_pose(cfsfk))
# print(fix_gp7_pose(mrfk))
# cfsik = cfspy.IKine(q0, cfsfk, "gp7", 1e-2, 1e-2, 100)
# print(cfspy.FKine(cfsik, "gp7"))
# print(gp7_ik(cfsfk, q0))

# %%
# print(T2[:3, :3] @ np.linalg.inv(cfsfk[:3, :3]))
print(T2[:3, :3])
print(cfsfk[:3, :3])
# %%
q2 = np.array([33.3577, 38.9660, -4.0192, 0.2050, -46.7385, 55.4162])
t2 = np.array([0.558998, 0.367960, -0.004794, 179.68, 0, 88.966])
T2 = np.eye(4)
T2[:3, 3] = t2[0:3]
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(T2)
# %%
q2 = np.array([1.57, 0,0,0,0,0])
t2 = np.array([0.0038, 0.559992, 0.484826, 145.19, -89.98, 124.406])
T2 = np.eye(4)
T2[:3, 3] = t2[0:3]
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(T2)

# %%
correction = np.linalg.inv(np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))
# q0 = np.deg2rad(trajj[0][0])
q0 = np.deg2rad([90, 0,0,0,0,0])
print(q0)
t0 = cfspy.FKine(q0, "gp7") @ np.linalg.inv(correction)
print(t0)
q1 = cfspy.IKine(q0, t0 @ np.linalg.inv(correction), "gp7", 1e-2, 1e-2, 100)
print(q1)
t1 = cfspy.FKine(q1, "gp7") @ np.linalg.inv(correction)
print(t1)

# %%
q2 = np.array([18.0815, 70.6080, -3.6887, 0.1312, -15.6088, 69.4935])
t2 = np.array([0.587,0.1916,-0.344999,180,0,87.7])
T2 = np.eye(4)
T2[:3, 3] = t2[0:3]
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(T2)
# %%
q2 = [-11.3268, -9.7538, 12.6571, 0.9210, -50.4519, -27.4058]
t2 = np.array([0.418227, -0.084785, 0.605695, -140.0971, -52.2161, -58.7475])
T2 = np.eye(4)
T2[:3, 3] = t2[0:3]
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
t3 = fix_gp7_pose(T2)
print(t3)


q3 = gp7_ik(t3, np.deg2rad(q2))[0]
print(gp7_fk(q3))
print(gp7_fk(np.deg2rad(q2)))
print(q2)
print(np.rad2deg(q3))
# print(gp7_ik(t3, np.deg2rad(q2)))
# print(np.rad2deg(q3))


# print(gp7_ik(gp7_fk(np.deg2rad(q2)), np.deg2rad(q2)))
# print(np.deg2rad(q2))
# print(np.deg2rad(q2))

# %%
q2 = np.deg2rad([0.6606, 54.9558, 57.7072, -1.9828, -92.6573, -3.0223])
cfsfk = cfspy.FKine(q2, "gp7")
print(cfsfk)
# %%
# q2 = np.deg2rad([0.6606, 54.9558, 57.7072, -1.9828, -92.6573, -3.0223])
q2 = np.deg2rad([0.6, 54., 57., -1., -90., -3.])
# q2 = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print(cfspy.IKine(q2, gp7_fk(q2), "gp7", 1e-2, 1e-2, 100))
# print(gp7_ik(gp7_fk(q2), q2))

# %%
t2 = np.array([0.841940, 0.012474, 0.236646, -178.0, 0.0089, -2.268])
T2 = np.eye(4)
T2[:3, 3] = t2[0:3]
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(T2 @ correction)
# %%
q2 = np.deg2rad(AxisPosition_arr[0][1][:6])
t2 = TCPPosition_arr[0][6][:6]
T2 = np.eye(4)
T2[:3, 3] = t2[0:3] * 1.0e-3
T2[:3, :3] = st.Rotation.from_euler(
        'xyz',  t2[3:6], degrees=True).as_matrix()
print(q2)
print(T2)
# print(cfspy.FKine(q2, "gp7"))
# print(T2 @ np.linalg.inv(cfspy.FKine(q2, "gp7")))
# print(cfspy.FKine(q2, "gp7") @ gp7correction)
# gp7correction = np.linalg.inv(cfspy.FKine(q2, "gp7")) @ T2
# %%
AxisPosition_arr[0][0][:6]
# %%
import cfspy
flange2tool = np.array([[ 9.99998459e-01, -2.06950460e-07,  1.75541378e-03,  0],
                     [ 2.87386840e-06, -9.99998460e-01, -1.75503585e-03,  0],
                     [ 1.75541144e-03,  1.75503819e-03, -9.99996919e-01, -0.33],
                     [ 0, 0, 0, 1]])
print(cfspy.FKine(np.deg2rad([-0.3117, -1.3506, -0.2812, -55.2304, -90.4367, 83.9407]), "gp7"))
# %%
AxisPosition_arr[0][0][:6]

# %%
trajj = np.load(
    "/home/MASCEI/Auto_calibration/DecData/boxpacking1206/trajj_nominal.npy")
trajj[0][6][:6]
# %%
TCPPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/boxpacking1206/TCPPosition_arr.npy")
TCPPosition_arr[0][0][:6]


trajj[0][2]
# %%
print(trajt[0][2])
print(np.deg2rad([0.66, 54.95, 57.07, -1.98, -92.6573, -3.0223]))
print(gp7_fk(np.deg2rad([0.66, 54.95, 57.07, -1.98, -92.6573, -3.0223])))
T = gp7_fk(np.deg2rad([0.66, 54.95, 57.07, -1.98, -92.6573, -3.0223]))


print(gp7_ik(trajt[0][2], np.deg2rad([0.66, 54.95, 57.07, -1.98, -90.6573, -3.0223])))

# %%
# AxisPosition_arr = np.load(
#     "/home/MASCEI/Auto_calibration/DecData/boxpacking1206/trajj_nominal.npy")
AxisPosition_arr = np.load(
    "/home/MASCEI/Auto_calibration/DecData/boxpacking1206/trajj_nominal.npy")
# print(gp7_fk(np.deg2rad([33.3577, 38.9660, -4.0192, 0.2050, -46.7385, 55.4162])))
q0 = np.deg2rad([33.3577, 38.9660, -4.0192, 0.2050, -46.7385, 55.4162])
print(np.rad2deg(gp7_ik(gp7_fk(q0), q0)[0]))
# print(TCPPosition_arr[0][0][:6])
print(gp7_fk(q0))
cfsfk = cfspy.FKine(q0, "gp7")
print(cfsfk)
print(np.rad2deg(cfspy.IKine(q0, cfsfk, "gp7", 1e-2, 1e-2, 100).reshape(-1)))
# %%
import matplotlib.pyplot as plt
AxisPosition_arr[0][0][:6]
# %%
pts = TCPPosition_arr[:, :, :3].reshape(-1, 3)
# %%
pts[1, :]
# %%
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')

ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
plt.show()
# %%
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot()
ax.plot(pts[:, 0], label='x')
ax.plot(pts[:, 1], label='y')
ax.plot(pts[:, 2], label='z')
plt.show()
# %%
joints = AxisPosition_arr.reshape(-1, 6)
# %%
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot()
ax.plot(joints[:, 0], label='joint 1')
ax.plot(joints[:, 1], label='joint 2')
ax.plot(joints[:, 2], label='joint 3')
ax.plot(joints[:, 3], label='joint 4')
ax.plot(joints[:, 4], label='joint 5')
ax.plot(joints[:, 5], label='joint 6')
plt.show()
# %%
trajj = np.load("/home/MASCEI/Auto_calibration/DecData/pickplaceconveyor1205/AxisPosition_arr.npy")
# %%
trajj = trajj.reshape(-1, 6)
# %%
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot()
ax.plot(trajj[:, 0], label='joint 1')
ax.plot(trajj[:, 1], label='joint 2')
ax.plot(trajj[:, 2], label='joint 3')
ax.plot(trajj[:, 3], label='joint 4')
ax.plot(trajj[:, 4], label='joint 5')
ax.plot(trajj[:, 5], label='joint 6')
plt.show()
# %%
