# Drake_CFS_SIM

### Dependencies

1. Install opencv: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
2. Install aruco: https://sourceforge.net/projects/aruco/files/
2. Install drake: https://drake.mit.edu/

### Build

1. `cd /path/to/Planner && mkdir build && cd build`
2. If build for debug, `cmake .. -DCMAKE_BUILD_TYPE=debug && make -j8`. Debug level log will show.
3. If build for release, `cmake .. -DCMAKE_BUILD_TYPE=release && make -j8`. Debug level log will be omitted.

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
- `dh_path`, `robot_cap_path`, `obs_path`, `config_path`: paths to parameter files. Files are already in the repo. Please replace the path string to actual value. See `CFS_Planner/src/Test/unit_test_CFS_subroutine.cpp` for example.

The returned variables are
- `CFS_traj`: `Eigen::MatrixXd`, dense robot joint trajectory containing `critical_traj`
- `critical_traj`: `Eigen::MatrixXd`, sparse robot joint trajectory
    - If pick_place_mode = `home_to_pick`, this contains three points `[home, pre_pick, pick]`
    - If pick_place_mode = `place_to_home`, this contains three points `[place, pre_place, home]`
    - If pick_place_mode = `pick_to_place`, this contains five points `[pick, pre_pick, mid_point, pre_place, place]`
- `CFS_success`:if CFS subroutine returns a valid solution.

See `CFS_Planner/src/Test/unit_test_CFS_subroutine.cpp` for an example of using CFS subroutine to compute three point trajectory for home-to-pick and five point trajectory for pick-to-place.

To run the example, run `build/cfslib_unit_test_CFS_subroutine`.

### Python API

The above CFS subroutine also provides a python API with exactly the same interface as C++ function.

To install the python interface, first activate any virtual python environment (if needed), then
```
cd /path/to/CFS_Planner
cd build && cmake .. && make -j12
cd ../
pip install .
```

See `CFS_Planner/scripts/Demo1_CFSPlanner.py` for an example of the usage.

To test the python api, run
```
python scripts/Demo1_CFSPlanner.py
```
The output should match the C++ API output (by running `cfslib_unit_test_CFS_subroutine`),
with potential different in precision of printing.

### Three CFS demo examples
```
python scripts/Demo1_CFSPlanner.py
```

```
python scripts/Demo2_RobotHandover.py
```

```
python scripts/Demo3_RobotPick.py
```