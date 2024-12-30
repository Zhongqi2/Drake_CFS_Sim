# MASCEI
## V-rep simulator 
- V-rep simulation for RACER project

### V-rep simulator installation
- Download and install the V-REP simulator
https://www.coppeliarobotics.com/downloads
- The detail of v-rep installation and operation tutorial is on the document: https://github.com/intelligent-control-lab/MASCEI/tree/main/V-rep%20Simulator/3_doc

### Import the RACER scene
- Click “File” -> “Open scene”
- Import XXX.ttt file: https://github.com/intelligent-control-lab/MASCEI/tree/main/V-rep%20Simulator/1_V-rep%20Scene
- Then you will see RACER scene in the simulator

### Programming the robot path and start the simulation:
- Change the programming lua file: https://github.com/intelligent-control-lab/MASCEI/blob/main/V-rep%20Simulator/1_V-rep%20Scene/car_move%5C.txt
- click “start” and you will see robot is moving in the simulation

### V-rep interface
- The interface of V-rep is used to connect to V-REP scene and interact with the simulation.
- Here we provide two version of V-rep interace demo code in C++ and Matlab
- C++ interface: https://github.com/intelligent-control-lab/MASCEI/tree/main/V-rep%20Simulator/4_V-rep%20Interface/c%2B%2B%20interface
- Matlab interace: https://github.com/intelligent-control-lab/MASCEI/tree/main/V-rep%20Simulator/4_V-rep%20Interface/matlab%20interface

---
## Planner

### Dependencies

1. Install opencv: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
2. Install aruco: https://sourceforge.net/projects/aruco/files/

### Build

1. `cd /path/to/Planner && mkdir build && cd build`
2. If build for debug, `cmake .. -DCMAKE_BUILD_TYPE=debug && make -j8`. Debug level log will show.
3. If build for release, `cmake .. -DCMAKE_BUILD_TYPE=release && make -j8`. Debug level log will be omitted.

---
## Perception

The perception module is needed for hand-eye calibration below in auto-calibration.

### Setup
1. Install realsense ros wrapper: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
2. Clone virtual cam package: `cd ros_wksp/src && git clone git@github.com:jgoppert/ros-virtual-cam.git`
3. Install opencv 3.x: https://opencv.org/releases/
4. `catkin_make`

### Run
1. [Run once] `sudo modprobe v4l2loopback video_nr=10`
2. `roslaunch mascei_perception start_cam.launch`

---
## Auto Calibration

### Dependencies
1. Install opencv: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
2. Install aruco: https://sourceforge.net/projects/aruco/files/
3. Drake(only for environment calibration): https://drake.mit.edu/from_binary.html#stable-releases
### Build
1. `cd /path/to/Auto_calibration && mkdir build && cd build`
2. If build for debug, `cmake .. -DCMAKE_BUILD_TYPE=debug && make -j8`. Debug level log will show.
3. If build for release, `cmake .. -DCMAKE_BUILD_TYPE=release && make -j8`. Debug level log will be omitted.

### General Procedure

#### Step 1: Run hand-eye calibration example
`cd /path/to/Auto_calibration && build/cfslib_hand_eye_calibration`

This program detects the transformation from the world frame to the camera frame, and stores the result in `config/cam_calibration/<cam serial id>/extrinsics_mat.txt`.

#### Step 2: Run environment calibration example
`cd /path/to/Auto_calibration/src/Test && python3 1_environment_calibration.py`

This program detects the point cloud of the target object (e.g., table) and locates the object within the world frame based on CAD model.

#### Step 3: Run tool calibration example
`cd /path/to/Auto_calibration && build/cfslib_tool_calibration`

This program controls the robot to touch the calibrated object and calibrates the tool pose with respect to the robot.

## Docker environments

## General Procedure to create a docker container for our project

#### Step 1: Download our docker image from google drive:
https://drive.google.com/drive/folders/1g8pwfG5yKLgLcnOTi_BKV5_4ayI07WQx?usp=drive_link

The actual docker file name might vary, but will start with `mascei` (e.g., `mascei_cfspy` for docker with cfs subroutine python api setup.)

If needed, replace `mascei` in the following commands with actual docker file name.

#### Step 2: Load this docker image on the PC:
`docker load --input ./mascei.tar`

#### Step 3: Create the docker containner from this docker image:
`docker run -it mascei`

#### Step 4: When inside the docker containner, add python path to environment:
`export PYTHONPATH=/opt/drake/lib/python3.12/site-packages:${PYTHONPATH}`

#### Step 5: Run the example code, the project code is in the home directory.
To test cfs subroutine python api, run the following inside the docker:
```
cd /home/MASCEI/Auto_calibration
source /home/envs/mascei/bin/activate
python scripts/test_cfs_pick_place.py
```

---
## CFS Subroutine

### C++ API

The subroutine has C++ API
```
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, bool> CFSPickPlace(
    const Eigen::MatrixXd current_pose_joint,
    const Eigen::Matrix4d target_pose_cartesian,
    const int& num_waypoints,
    const std::string pick_place_mode,
    const std::string dh_path,
    const std::string robot_cap_path,
    const std::string obs_path,
    const std::string config_path)
```
The type and meaning of each argument follows
- `current_pose_joint`: `Eigen::MatrixXd`, current robot joint state
- `target_pose_cartesian`: `Eigen::Matrix4d`, target robot Cartesian pose
- `num_waypoints`: number of critical waypoints including start and end points (e.g., 3 for `home_to_pick`/`place_to_home`, 5 for `pick_to_place`)
- `pick_place_mode`: `std::string`, one of the following options:
    - `home_to_pick`
    - `place_to_home`
    - `pick_to_place`
- `dh_path`, `robot_cap_path`, `obs_path`, `config_path`: paths to parameter files. Files are already in the repo. Please replace the path string to actual value. See `Auto_calibration/src/Test/unit_test_CFS_subroutine.cpp` for example.

The returned variables are
- `CFS_traj`: `Eigen::MatrixXd`, dense robot joint trajectory containing `critical_traj`
- `critical_traj`: `Eigen::MatrixXd`, sparse robot joint trajectory
    - If pick_place_mode = `home_to_pick`, this contains three points `[home, pre_pick, pick]`
    - If pick_place_mode = `place_to_home`, this contains three points `[place, pre_place, home]`
    - If pick_place_mode = `pick_to_place`, this contains five points `[pick, pre_pick, mid_point, pre_place, place]`
- `CFS_success`:if CFS subroutine returns a valid solution.

See `Auto_calibration/src/Test/unit_test_CFS_subroutine.cpp` for an example of using CFS subroutine to compute three point trajectory for home-to-pick and five point trajectory for pick-to-place.

To run the example, run `build/cfslib_unit_test_CFS_subroutine`.

### Python API

The above CFS subroutine also provides a python API with exactly the same interface as C++ function.

To install the python interface, first activate any virtual python environment (if needed), then
```
cd /path/to/Auto_calibration
cd build && cmake .. && make -j12
cd ../
pip install .
```

See `Auto_calibration/scripts/test_cfs_pick_place.py` for an example of the usage.

To test the python api, run
```
python scripts/test_cfs_pick_place.py
```
The output should match the C++ API output (by running `cfslib_unit_test_CFS_subroutine`),
with potential different in precision of printing.