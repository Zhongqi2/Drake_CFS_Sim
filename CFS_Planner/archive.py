
# %%
tcps = [
    [
        [0.908425,   0.401873, -0.115163,   0.258714],
        [-0.409308,   0.911053, -0.0494834,  0.0940818],
        [0.0850333,   0.092089,   0.992113,   0.453581],
        [0,   0,   0,      1]
    ], [
        [0.368491,   0.927555, -0.0620972,   0.413498],
        [-0.929578,   0.366933, -0.0352822,   0.0207283],
        [-0.00994064,   0.0707254,   0.997446,   0.33584],
        [0,   0,   0,      1]
    ], [
        [0.368483,   0.927558, -0.0620925,   0.413497],
        [-0.929581,   0.366925, -0.035281, -0.148169],
        [-0.00994193,   0.0707204,   0.997447,   0.328194],
        [0,   0,   0,      1]
    ], [
        [0.862135,   0.505945, -0.0272435,   0.565896],
        [-0.50658,   0.859662, -0.0660176, -0.0352221],
        [-0.00998115,   0.0707171,   0.997446,   0.440843],
        [0,   0,   0,     1]
    ], [
        [0.94074, -0.337483, 0.0333666,   0.710496],
        [0.33898,   0.938672, -0.0631442, -0.274169],
        [-0.0100102,  0.0707129,   0.997446,   0.388194],
        [0,   0,   0,   1]
    ], [
        [0.940744, -0.337473,   0.0333579,   0.721897],
        [0.33897,   0.938675, -0.0631564, -0.274165],
        [-0.00999869,   0.0707214,   0.997446,   0.35819],
        [0,   0,   0,     1]
    ], [
        [0.940744, -0.337475,   0.0333386,   0.608801],
        [0.338971,   0.938675, -0.063148, -0.274164],
        [-0.00998318,   0.0707069,   0.997447,   0.358188],
        [0,   0,   0,       1]
    ]
]
# %%
home = UserData_arr[0]
prepick = UserData_arr[1]
pick = UserData_arr[2]
postpick = UserData_arr[1]
intermediate = UserData_arr[3]
preplace = UserData_arr[4]
place = UserData_arr[5]
postplace = UserData_arr[6]
home = UserData_arr[0]

homeT = tcps[0]
prepickT = tcps[1]
pickT = tcps[2]
postpickT = tcps[1]
intermediateT = tcps[3]
preplaceT = tcps[4]
placeT = tcps[5]
postplaceT = tcps[6]
homeT = tcps[0]

wypts = [home, prepick, pick, postpick,
         intermediate, preplace, place, postplace, home]
wyptsT = [homeT, prepickT, pickT, postpickT,
          intermediateT, preplaceT, placeT, postplaceT, homeT]
for q in wypts:
    print(q.tolist())

trajj = [wypts]
trajt = [wyptsT]

np.save("/home/MASCEI/Auto_calibration/realdata/pickplaceconveyor0928_manual/trajj.npy", trajj)
np.save("/home/MASCEI/Auto_calibration/realdata/pickplaceconveyor0928_manual/trajt.npy", trajt)