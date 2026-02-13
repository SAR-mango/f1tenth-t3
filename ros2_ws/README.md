# ROS2 workspace (Humble)
This workspace is for the ROS2 + Gazebo Fortress migration. The legacy ROS1
workspace remains in `ros_ws/`.

## Packages
- `drive_msgs`: ROS2 message definitions (note the ROS2 CamelCase message name
  `DriveParam.msg`).
- `wallfollowing2`: ROS2 port of the lidar wallfollowing algorithm.
- `car_control`: ROS2 control nodes (`drive_parameters_multiplexer`,
  `car_controller`) with optional `/cmd_vel` output.
- `racer_bringup`: Gazebo Fortress launch and ros_gz_bridge wiring.

## Build (ROS2)
```bash
cd ros2_ws
colcon build --packages-select drive_msgs wallfollowing2 car_control racer_bringup
source install/setup.bash
```

## Python deps
`wallfollowing2` relies on `circle-fit` and numpy. Install via pip if needed:
```bash
pip install circle-fit
```

## Run (ROS2 + Gazebo Fortress)
```bash
ros2 launch racer_bringup fortress_sim.launch.py
```
