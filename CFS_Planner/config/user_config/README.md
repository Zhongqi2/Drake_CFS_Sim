# User Config Parameters Guide
## General Parameters
1. `trajectory_frequency`: Minimum frequency for planned trajectory. (Point goal)
2. `safety_margin`: Safety margin from the robot to the obstacles. Larger margin indicates further away to the obstacles, but might lead to infeasible solution.

## CFS Parameters
1. `weight_ref`: Three values correspond to the penalties to position (Q<sub>1</sub>), velocity (Q<sub>2</sub>), and acceleration (Q<sub>3</sub>) between the reference and the generated trajectories. Larger Qs lead to more similar properties between the generated trajectory and the reference trajectory.
2. `weight_self`: Three values correspond to the penalties to position (Q<sub>1</sub>), velocity (Q<sub>2</sub>), and acceleration (Q<sub>3</sub>) of the generated trajectory. Larger values of Qs lead to smoother properties of the generated trajectory.
3. `cfs_max_iter`: Maximum number of iterations for CFS optimization.
4. `cfs_convergence_thres`: Convergence threshold for CFS optimization. 
5. `cfs_resample_n_insert`: Number of new waypoints to be added during resampling between the far waypoints in the original trajectory.
6. `cfs_max_resample`: Maximum number of resamplings to perform.

## ICOP IK Parameters
1. `icop_thresh`: Convergence threshold for ICOP IK. Smaller threshold leads to more accurate IK solution, but longer computation time.
2. `icop_max_iter`: Maximum number of iterations for ICOP IK. More iterations usually lead to better feasibility, but longer computation time.

