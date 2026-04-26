# F1TENTH T3

ROS2 Humble + Gazebo Fortress workspace for the lidar-equipped F1TENTH test
car. This repo has been trimmed to the active ROS2 stack; the legacy ROS1
content from the upstream fork is no longer included.

## Packages
- `ros2_ws/src/drive_msgs`: `DriveParam.msg`
- `ros2_ws/src/car_control`: drive multiplexer, controller, UART bridge
- `ros2_ws/src/wallfollowing2`: autonomy and real-hardware test nodes
- `ros2_ws/src/racer_bringup`: launch files, dashboard, RViz config, Fortress
  world assets

## Prerequisites
- ROS2 Humble
- Gazebo Fortress with the `ign` CLI
- `ros_gz_bridge` / `ros_gz_sim`
- `urg_node`
- `python3-pyqt5`
- `python3-pyqtgraph`
- Python packages used by the lidar algorithms:

```bash
pip install circle-fit
```

## Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Fortress Simulation

Start the base sim stack:

```bash
ros2 launch racer_bringup fortress_sim.launch.py
```

Common sim bringup variants:

```bash
ros2 launch racer_bringup fortress_teleop_rviz.launch.py
ros2 launch racer_bringup fortress_wallfollowing.launch.py
ros2 launch racer_bringup fortress_wallbalancing.launch.py
ros2 launch racer_bringup fortress_follow_the_gap.launch.py
ros2 launch racer_bringup fortress_scan_stop_test.launch.py
```

The Fortress world now lives in
`ros2_ws/src/racer_bringup/worlds/racetrack_decorated_2_hokuyo.world`, with
its referenced meshes in `ros2_ws/src/racer_bringup/meshes/`. The sim launch
files prefer those source-tree assets so edits take effect on the next launch
without rebuilding, and they fall back to the installed package copy when the
source tree is not present.

## Real-Car Bringup

Autonomy launches:

```bash
ros2 launch racer_bringup real_follow_the_gap.launch.py
ros2 launch racer_bringup real_wallbalancing.launch.py
ros2 launch racer_bringup real_weighted_pairs_mmse.launch.py
```

Test / characterization launches:

```bash
ros2 launch racer_bringup real_scan_stop_test.launch.py
ros2 launch racer_bringup real_speed_test.launch.py
ros2 launch racer_bringup real_steering_test.launch.py
ros2 launch racer_bringup real_timed_arc_test.launch.py
```

The weighted-pairs MMSE launch loads parameters from
`ros2_ws/src/racer_bringup/config/weighted_pairs_mmse.yaml` by default. Edit
that file directly for persistent tuning, or override with:

```bash
ros2 launch racer_bringup real_weighted_pairs_mmse.launch.py \
  weighted_pairs_params_file:=/absolute/path/to/weighted_pairs_mmse.yaml
```

## Dashboard

Run the dashboard on its own:

```bash
ros2 launch racer_bringup dashboard.launch.py
```

The dashboard now mirrors the simulator-style UI: one `pyqtgraph` LiDAR window
with forward/lateral axes plus a separate command-history window for speed and
steering setpoints. The sim wallfollowing / wallbalancing launches and the real
follow-the-gap / wallbalancing / weighted-pairs launches start it by default.
Use `start_dashboard:=false` to suppress it, or `dashboard_start_rviz:=true`
when the including launch supports RViz alongside the dashboard.

Controller-based launches label the steering history as an angle command.
`real_weighted_pairs_mmse.launch.py` labels it as steering radius and hides
zero-valued straight commands so the plot matches that algorithm's raw UART
command convention.

## Notes
- Shells in this environment are expected to be `zsh`.
- Gazebo Fortress uses `ign`, not `gz`, on this machine.
- The launch files still use the install-space `car_control` executables, so
  rebuild `ros2_ws` after changing those C++ nodes.
