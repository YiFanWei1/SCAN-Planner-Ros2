# scan_mppi_controller

Standalone CPU MPPI controller for SCAN-Planner. It follows the existing 3-D
`scan_planner_msgs/Bspline`, checks predicted swept motion against SCAN's
inflated 3-D occupancy cloud, and sends raw velocity commands through the
existing `cmd_vel_safety_gate`.

The package does not replace or modify `scan_planner`, `plan_env`, or
`bspline_opt`. Its top-level launches start the existing planning nodes without
the legacy closed-loop controller.

## Build and test

```bash
colcon build --packages-select scan_mppi_controller --symlink-install
source install/setup.bash
colcon test --packages-select scan_mppi_controller
colcon test-result --verbose
```

## Run

Simulation starts motion disabled by default:

```bash
ros2 launch scan_mppi_controller mppi_sim.launch.py rviz:=true enable_motion:=false
```

Real Go2/Livox:

```bash
ros2 launch scan_mppi_controller mppi_real_go2_livox.launch.py \
  enable_motion:=false
```

After validating the inflated cloud, predicted footprints, optimal path, and
velocity arrows in RViz, enable the unchanged safety gate:

```bash
ros2 service call /scan_planner/enable_motion std_srvs/srv/SetBool "{data: true}"
```

## Important limitation

Collision checking uses `/grid_map/occupancy_inflate`, which is the existing
visualization point cloud and is cropped by SCAN's `grid_map.vis_height`.
The controller reports map age and point count, but it cannot recover obstacle
voxels omitted by that publisher. Validate railing coverage at every relevant
height before enabling motion.
