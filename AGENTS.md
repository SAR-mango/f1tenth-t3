# AGENTS.md
This file is a continuity note for Codex-style agents working in this repo.
Update it whenever you make major changes (new features, architecture changes,
new dependencies, new workflows, or significant refactors).

## Project overview
- ROS2 Humble + Gazebo Fortress workspace for a lidar-equipped 1/10th-scale
  autonomous racing car.
- The legacy ROS1/catkin stack from the upstream fork has been removed from
  this repo; keep new work ROS2-only unless there is a deliberate reason to
  reintroduce archived material elsewhere.
- Supports both Fortress simulation and the current real-car Hokuyo + UART
  workflow.
- Core maintained autonomy paths are wallfollowing, wallbalancing,
  follow-the-gap, weighted-pairs MMSE, and the real-hardware test nodes.

## Repository layout
- ros2_ws/                    # Active ROS2 workspace
  - src/drive_msgs            # ROS2 interface package (DriveParam.msg)
  - src/wallfollowing2        # ROS2 autonomy + test nodes (rclpy)
  - src/car_control           # ROS2 control nodes (multiplexer + controller)
  - src/racer_bringup         # ROS2 bringup, dashboard, Fortress assets
    - launch/                 # Sim + real-car launch files
    - config/                 # RViz + weighted-pairs parameter files
    - worlds/                 # Fortress world files kept in-repo
    - meshes/                 # Meshes referenced by the Fortress world
- README.md                   # Top-level ROS2 usage doc
- AGENTS.md                   # Cross-session continuity log

## Key simulation assets
- ros2_ws/src/racer_bringup/worlds/racetrack_decorated_2_hokuyo.world
- ros2_ws/src/racer_bringup/meshes/
  - track_decorated_2.dae
  - walls.dae
  - decoration_colliders_2.dae

## Common workflows
- Build: `colcon build` from `ros2_ws/`
- ROS2 source: `source install/setup.bash`
- ROS2 sim: `ros2 launch racer_bringup fortress_sim.launch.py`
- ROS2 sim controllers:
  `fortress_wallfollowing.launch.py`,
  `fortress_wallbalancing.launch.py`,
  `fortress_follow_the_gap.launch.py`
- Real-car autonomy:
  `real_follow_the_gap.launch.py`,
  `real_wallbalancing.launch.py`,
  `real_weighted_pairs_mmse.launch.py`
- Real-car tests:
  `real_scan_stop_test.launch.py`,
  `real_speed_test.launch.py`,
  `real_steering_test.launch.py`,
  `real_timed_arc_test.launch.py`
- Dashboard: `ros2 launch racer_bringup dashboard.launch.py`
- Shell: zsh, ROS2 environment is pre-sourced on new terminals.
- Assumed installed: ROS2 Humble, Gazebo Fortress, ros_gz bridge.
- Use `ign` CLI for Gazebo Fortress (`gz` is not available).

## Continuity log (append newest on top)
- 2026-04-25: Removed the remaining legacy upstream repository content so the
  tree is now ROS2-only. Moved the active Fortress world and required meshes
  from `ros_ws/src/simulation/racer_world` into
  `ros2_ws/src/racer_bringup/{worlds,meshes}`, updated all Fortress launch
  files to prefer those source-tree assets with install-space fallback, taught
  `racer_bringup/setup.py` to install the world + mesh files, removed the dead
  `cmd_vel_to_drive_param_real_node` entrypoint, dropped legacy docs/scripts/
  ROS1 workspace files from the repo, and rewrote the README / `.gitignore`
  for the cleaned ROS2 layout.
- 2026-04-24: Replaced the `weighted_pairs_mmse` launch-argument tuning fanout
  with a YAML parameter file workflow. Added
  `racer_bringup/config/weighted_pairs_mmse.yaml`, changed
  `real_weighted_pairs_mmse.launch.py` to load that file via a new
  `weighted_pairs_params_file` launch arg, and made the default prefer the
  source-tree config path so retuning the file takes effect on the next launch
  without rebuilding. Updated `racer_bringup/setup.py` to install all config
  files and refreshed the README usage notes.
- 2026-04-24: Made the ROS2 real-car `weighted_pairs_mmse` stack fully
  launch-tunable. `wallfollowing2/weighted_pairs_mmse_algorithm.py` now keeps
  its algorithm constants in a validated parameter table, and
  `wallfollowing2/weighted_pairs_mmse_node.py` declares/applies those as ROS
  parameters so the next scan uses updated values without a package rebuild.
  `racer_bringup/real_weighted_pairs_mmse.launch.py` now exposes every
  weighted-pairs algorithm parameter plus the straight-radius override as
  launch arguments, and `README.md` documents CLI tuning plus `--show-args`.
- 2026-04-24: Added a scan-based hard stop override to the direct ROS2 real-car
  `weighted_pairs_mmse` path. `wallfollowing2/weighted_pairs_mmse_node.py`
  now forces `/cmd_vel` to zero whenever any finite lidar return inside a
  configurable forward cone is at or below the configured stop distance
  (`front_stop_distance_m`, default 0.2 m, and
  `front_stop_half_angle_deg`, default 15 deg), overriding the weighted-pairs
  algorithm until the front cone clears. Exposed both parameters in
  `racer_bringup/real_weighted_pairs_mmse.launch.py`.
- 2026-04-24: Fixed ROS2 autonomy bringup emergency-stop behavior so the
  `racer_bringup` weighted-pairs, follow-the-gap, wallfollowing, and
  wallbalancing launch files no longer pin `/commands/emergency_stop=false`
  continuously. They now publish a short startup clear burst and exit, which
  preserves the previous automatic estop-clear-on-launch behavior while
  allowing later manual estop assertions to latch.
- 2026-04-24: Added direct raw-command ROS2 real-car autonomy path for the new
  `weighted_pairs_mmse` lidar algorithm. New
  `wallfollowing2/weighted_pairs_mmse_node.py` wraps the uploaded algorithm
  implementation (`weighted_pairs_mmse_algorithm.py`), consumes `/scan`, and
  publishes `/cmd_vel` directly as speed in m/s plus signed turning radius in
  meters while preserving the current positive-left / negative-right radius
  convention and encoding straight driving as `0.0` radius for the UART path.
  The wrapper node now stops on algorithm fallback states by default even
  though the imported algorithm's own fallback result is `v_min` straight.
  Added `racer_bringup/real_weighted_pairs_mmse.launch.py` to run
  `urg_node_driver` -> `weighted_pairs_mmse_node` -> `uart_actuator_bridge`
  with the dashboard enabled by default. Extended
  `car_control/uart_actuator_bridge` with optional drive-mode and
  emergency-stop gating so direct raw `/cmd_vel` launches can preserve the
  existing stop behavior without routing through
  `drive_parameters_multiplexer` or `car_controller`.
- 2026-04-22: Made `racer_bringup/setup.py` install all
  `launch/*.launch.py` files automatically so `dashboard.launch.py` and future
  bringup launches are present in install space after a rebuild. Rebuilt
  `racer_bringup` so installed real launch files include the dashboard by
  default and `ros2 launch racer_bringup dashboard.launch.py` resolves.
- 2026-04-22: Kept the simplified dashboard UI unchanged but extended its data
  plumbing so the numeric tiles now prefer
  `/telemetry/uart_command` (`TwistStamped`) and fall back to `/cmd_vel`
  (`Twist`), allowing the same dashboard to work in both Fortress sim and
  real-car bringup. Updated `real_follow_the_gap.launch.py` and
  `real_wallbalancing.launch.py` to include the dashboard by default, matching
  the existing Fortress wallfollowing / wallbalancing behavior, with
  `start_dashboard:=false` and `dashboard_start_rviz:=true` control flags.
- 2026-04-21: Extended the simplified ROS2 dashboard to read numeric velocity
  and steering values from a configurable `Twist` topic (default `/cmd_vel`)
  while keeping the XY LiDAR view. Updated Fortress sim bringup so
  `fortress_wallfollowing.launch.py` and `fortress_wallbalancing.launch.py`
  now include the dashboard by default, with `start_dashboard:=false` to
  disable it and `dashboard_start_rviz:=true` to also open RViz2.
- 2026-04-21: Simplified the ROS2 live dashboard UI to match current real-car
  needs. `racer_bringup/dashboard_node` now renders `/scan` directly as a
  PyQt XY LiDAR view instead of the previous time-series/status panel layout,
  and the velocity / steering areas are reduced to placeholder numeric tiles
  until real telemetry topics are wired in. Updated
  `racer_bringup/dashboard.launch.py` so RViz2 is optional via
  `start_rviz:=true` rather than launching by default, and refreshed the
  README dashboard description to match.
- 2026-04-20: Ported the latest FTG-only tuning from
  `origin/varsha_branch` commit `95d70bb` onto the adapted ROS2
  `navjit_branch` implementation without merging the full branch history.
  Updated `wallfollowing2/follow_the_gap_node.py` with the newer adaptive
  bubble cap, gap-fallback steering/speed behavior, weighted target selection,
  steering-target-angle scaling, and steering-step helper while preserving the
  branch's dynamic parameter typing and compatible `DriveParam` pipeline.
  Retuned `racer_bringup/fortress_follow_the_gap.launch.py` to the newer FTG
  sim parameters and controller limits, while keeping this branch's current
  repo-root/path fallback and reduced launch log noise behavior.
- 2026-04-17: Added separate ROS2 live dashboard bringup
  `racer_bringup/dashboard.launch.py` for real-car runs. It launches RViz2 on
  the existing lidar config plus a new `racer_bringup/dashboard_node` PyQt
  UI that keeps a 60s sliding buffer of command/feedback plots, shows scan
  freshness, drive mode, emergency-stop state, and the latest UART CSV frame.
  Also extended `car_control/uart_actuator_bridge` to publish transmitted UART
  telemetry on `/telemetry/uart_command` (`TwistStamped`),
  `/telemetry/uart_frame` (`String`), and `/telemetry/uart_command_stale`
  (`Bool`) so the dashboard reflects the actual bridge output rather than only
  upstream `/cmd_vel`.
- 2026-04-13: Added ROS2 real-hardware autonomous launch
  `racer_bringup/real_follow_the_gap.launch.py` for the ported
  `wallfollowing2/follow_the_gap_node`, mirroring the existing physical-car
  autonomy stack: `urg_node_driver` -> `follow_the_gap_node` ->
  `drive_parameters_multiplexer` -> `car_controller` -> `/cmd_vel` ->
  `uart_actuator_bridge`, plus continuous autonomous drive-mode and
  emergency-stop-clear publishers. Registered the launch for install in
  `racer_bringup/setup.py` and documented the command in `README.md`.
- 2026-04-10: Selectively ported the ROS2 `follow_the_gap` autonomy path from
  `varsha_branch` onto the `robbie_branch`/`navjit_branch` infrastructure
  instead of merging the full branch histories. Added
  `wallfollowing2/follow_the_gap_node.py` plus dedicated Fortress sim launch
  `racer_bringup/fortress_follow_the_gap.launch.py`, registered both in the
  package setup files, preserved the newer real-car/UART/wallbalancing
  infrastructure, enabled dynamic parameter typing in the FTG node to match the
  current ROS2 controller style, and dropped stale launch parameters that were
  not implemented in the branch tip.
- 2026-04-09: Hardened ROS2 real-car shutdown behavior so all real launch paths
  using `car_control/uart_actuator_bridge` now transmit a final neutral UART
  frame (`0` speed, `0` steering, `0` brake when present) before closing the
  serial port. Also updated `car_control/car_controller::stop()` to publish a
  neutral steering command in addition to zero speed, preventing the last angle
  from remaining latched on drive-mode lock or emergency stop.
- 2026-04-09: Added ROS2 real-hardware autonomous launch
  `racer_bringup/real_wallbalancing.launch.py` for the `wallfollowing2`
  `wallbalancing_node` using live Hokuyo lidar and Jetson UART output. The new
  path mirrors the existing autonomy stack on the physical car:
  `urg_node_driver` -> `wallbalancing_node` ->
  `drive_parameters_multiplexer` -> `car_controller` -> `/cmd_vel` ->
  `uart_actuator_bridge`, plus continuous autonomous drive-mode and
  emergency-stop-clear publishers. Registered the launch for install in
  `racer_bringup/setup.py`.
- 2026-04-08: Revised the ROS2 real-hardware `speed_test_node` schedule from a
  uniform speed ladder to the requested stopped-dwell characterization run:
  after 3s startup delay it now commands 0.5 m/s for 12s, 0.6 for 10s, 0.75
  for 8s, 1.0 for 6s, 1.5 for 4s, 2.0 for 3s, and 3.0 for 2s, with 20s stopped
  waits between each speed command. Updated `real_speed_test.launch.py` to
  expose `inter_step_wait_sec` instead of the old uniform `step_duration_sec`.
- 2026-04-07: Added ROS2 real-hardware open-loop characterization tests on the
  raw `/cmd_vel` UART path. New `wallfollowing2/speed_test_node` waits 3s, then
  commands fixed 10s speed plateaus at 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, and
  10 m/s before stopping. New `wallfollowing2/steering_test_node` waits 3s,
  then commands fixed-radius turns for 20s each at 0.5, 0.6, 0.75, 1.0, 1.5,
  2.0, and 3.0 m with a `turn_right` boolean to mirror the sequence. Added
  `racer_bringup/real_speed_test.launch.py` and
  `racer_bringup/real_steering_test.launch.py` to run the nodes through the
  existing `car_control/uart_actuator_bridge` `cmd_vel` mode. Note: the full
  requested steering radius list yields a 143s timeline (3s + 7x20s), not 123s.
- 2026-03-16: Added a new ROS2 real-hardware timed arc test path for UART
  control without lidar dependencies. New `wallfollowing2/timed_arc_test_node`
  publishes raw `/cmd_vel` commands for a fixed sequence: startup delay, left
  arc at constant speed/radius, right arc at constant speed/radius, then stop.
  Added `racer_bringup/real_timed_arc_test.launch.py` to run the node through
  the existing `car_control/uart_actuator_bridge` raw UART path. Defaults
  currently follow the per-phase request exactly: 6s wait + 20s left + 20s
  right = 46s total timeline.
- 2026-03-12: Fixed `car_control/uart_actuator_bridge` never opening the UART
  on this Jetson: the reconnect gate initialized `last_open_attempt_` with
  `steady_clock::time_point::min()`, which prevented the first open attempt
  from reaching `open()`/`tcsetattr()` in practice. Switched the initial value
  to `steady_clock::time_point{}` so the bridge opens the port normally on the
  first timer tick.
- 2026-03-12: Switched the ROS2 `uart_actuator_bridge` default command path
  from post-`car_controller` actuator topics to raw `/cmd_vel` so the MCU can
  receive `linear.x` in m/s and `angular.z` unchanged as the steering/radius
  field. `real_scan_stop_test.launch.py` now uses the bridge directly on
  `/cmd_vel`, drops the unused `cmd_vel_to_drive_param_real_node` /
  `drive_parameters_multiplexer` / `car_controller` stack for this test path,
  and defaults `uart_send_rate_hz` to 40.0. The bridge still supports the old
  actuator-topic mode via `command_mode:=actuator_topics`, but `cmd_vel` is now
  the default.
- 2026-03-12: Added ROS2 `car_control` UART actuator bridge node
  `uart_actuator_bridge` that subscribes to existing actuator topics
  (`/commands/motor/speed`, `/commands/servo/position`,
  `commands/motor/brake`) and writes an ASCII CSV frame
  `speed,angle,brake\n` over Jetson UART using POSIX termios
  (`uart_device`, `baud_rate`, `send_rate_hz`, timeout/reconnect params).
  Updated `racer_bringup/real_scan_stop_test.launch.py` to optionally start
  the bridge with launch args for UART device and baud rate while keeping the
  existing autonomy/car_control pipeline intact.
- 2026-03-10: Fixed ROS2 launch/root-path drift that caused sim bringup to
  fail on this machine with stale `/home/erk/f1tenth-t3` references and
  non-expanded `~/f1tenth-t3` defaults. Updated `racer_bringup` Fortress launch
  files (`fortress_sim`, `fortress_teleop_rviz`, `fortress_scan_stop_test`,
  `fortress_wallfollowing`, `fortress_wallbalancing`) to use
  `os.path.expanduser("~/f1tenth-t3")` fallback for `F1TENTH_T3_ROOT`, then
  rebuilt `drive_msgs`, `car_control`, `wallfollowing2`, `racer_bringup` so
  installed launch/scripts regenerate with correct local paths.
- 2026-03-10: Added ROS2 real-car scan-stop test path while keeping
  `scan_stop_reverse_test_node.py` unchanged: new
  `cmd_vel_to_drive_param_real_node` in `wallfollowing2` converts
  `/cmd_vel` (m/s + turning-radius mode) to normalized
  `/input/drive_param/autonomous` for `car_control` using Ackermann defaults
  from `racetrack_decorated_2_hokuyo.world`
  (`wheel_base=0.300568`, `kingpin_width=0.2`, `steering_limit=0.7`) and
  legacy servo/ERPM mapping. Added `racer_bringup/real_scan_stop_test.launch.py`
  to run real Hokuyo (`urg_node_driver`), translator, scan-stop test node,
  multiplexer/controller, and continuous drive-mode/emergency-stop publishers.
  Registered new launch + console script in package setup files and added
  `urg_node` runtime dependency to `racer_bringup`.
- 2026-02-18: Fixed `wallfollowing` immediate right-wall crash without
  changing controller type: corrected ROS LaserScan left/right mapping in
  `wallfollowing_node.py` cartesian conversion (`x = sin(angle) * range`),
  switched wallfollowing steering sign to positive for this setup, eliminated
  startup derivative kick, and added a safe straight/slow fallback on failed
  wall fits. Retuned `fortress_wallfollowing.launch.py` to lower
  `kp/kd/steering_limit` and slower throttle/acceleration for stable testing.
- 2026-02-18: Retuned ROS2 `wallfollowing` for stability-first behavior after
  aggressive right-wall impacts: lowered PD/speed authority in
  `fortress_wallfollowing.launch.py` (`car_controller` speed/steer caps and
  wallfollowing throttle/accel/gains) and added lidar bump robustness in
  `wallfollowing_node.py` via left-wall line-fit outlier rejection plus
  low-pass smoothing of wall-distance and derivative terms.
- 2026-02-17: Fixed ROS2 parameter override type mismatch for wallfollowing
  and wallbalancing nodes (e.g., `kp:=1` as int) by enabling dynamic parameter
  typing at declaration and coercing loaded values to expected numeric types.
- 2026-02-17: Split autonomous controllers into two named setups:
  `wallfollowing` now uses a new simple left-wall tracker (target 0.25m,
  line-fit on subsampled left-forward lidar points, no percentile/notch
  filtering), while the previous advanced balanced-wall controller was renamed
  to `wallbalancing`. Added `wallbalancing_node` entry point and
  `fortress_wallbalancing.launch.py`; updated `fortress_wallfollowing.launch.py`
  to use snappier simple-controller + faster car controller limits.
- 2026-02-17: Applied aggressive wallfollowing speed retune for straight-line
  throughput: increased `car_controller` speed/steer limits in
  `fortress_wallfollowing.launch.py`, retuned wallfollowing params
  (`min/max_throttle`, accel, radius gating, barrier gating) and added
  `min_relative_speed` floor. Updated ROS2 wallfollowing logic to use average
  wall curvature and speed-floor scaling by front clearance to prevent
  near-stops on clear straights.
- 2026-02-17: Renamed misspelled wallfollowing parameter
  `barrier_size_realtive` -> `barrier_size_relative` in ROS2 and legacy ROS1
  wallfollowing sources/configs. Updated ROS2 wallfollowing speed gating with
  `barrier_speed_weight` and retuned `fortress_wallfollowing.launch.py`
  (`barrier_size_relative`, barrier limits, percentile) so straight-line speed
  is less constrained by front-barrier heuristics.
- 2026-02-12: Added wallfollowing balancing + notch robustness for faster
  laps: new `wall_balance_gain` shifts target away from the closer wall; new
  `barrier_percentile` uses percentile front distance for speed gating to avoid
  overreacting to small wall notches. Retuned `fortress_wallfollowing` for
  higher straight speed with safer sharp-turn behavior.
- 2026-02-12: Retuned `fortress_wallfollowing.launch.py` for faster but more
  stable driving: increased `car_controller` max linear speed / steering
  authority, raised wallfollowing throttle and acceleration limits, adjusted
  barrier speed gating (lower/upper/exponent), and tuned PID + lookahead
  (`controller_*`, `corner_cutting`, `straight_smoothing`) for sharper turn
  recovery.
- 2026-02-12: Improved wallfollowing stability: corrected front barrier metric
  to nearest obstacle (`min` front distance), added `steering_sign` parameter
  and hard front-stop guard, clamped steering/speed commands, and tuned
  `fortress_wallfollowing.launch.py` to more conservative speeds/acceleration.
- 2026-02-12: Fixed `wallfollowing_node` crash in
  `wallfollowing2/rviz_geometry.py` by replacing invalid positional
  `geometry_msgs/Point` construction with explicit field assignment helper.
  Set marker frame to `car/chassis/ust10lx`.
- 2026-02-12: Reduced terminal noise in
  `fortress_wallfollowing.launch.py` by sending helper drive-mode/emergency-stop
  `ros2 topic pub` process output to log files instead of screen.
- 2026-02-12: Patched `fortress_sim.launch.py` and
  `fortress_wallfollowing.launch.py` to start `car_control` executables via
  workspace install paths (`ros2_ws/install/car_control/lib/car_control/*`)
  instead of package lookup, working around intermittent `package 'car_control'
  not found` overlay-resolution issues.
- 2026-02-11: Added dedicated autonomous launch
  `fortress_wallfollowing.launch.py` (ign + bridge + car_control +
  wallfollowing2 + continuous `/commands/drive_mode=2` and
  `/commands/emergency_stop=false` publishers). Includes conservative
  wallfollowing speed parameters for initial stability.
- 2026-02-11: Removed external `circle_fit` dependency by implementing
  numpy least-squares circle fitting in `wallfollowing2/circle.py`. Hardened
  `wallfollowing_node.py` against sparse/invalid scans and fit failures.
- 2026-02-11: Tuned scan-stop-reverse test timing: startup delay increased to
  6.0s and pre-reverse hold increased to 3.0s (both node defaults and
  `fortress_scan_stop_test.launch.py` parameters).
- 2026-02-11: Fixed crash in `scan_stop_reverse_test_node` caused by passing
  printf-style args directly to `rclpy` logger. Startup info log now formats
  as a single string, allowing the test state machine to run.
- 2026-02-11: Added ROS2 lidar-behavior test node
  `scan_stop_reverse_test_node` in `wallfollowing2`:
  waits 3s, drives forward until front scan <= 0.5m, stops, reverses ~0.5m by
  timed command, then remains stopped. Added dedicated launch
  `fortress_scan_stop_test.launch.py` (ign + bridge + test node).
- 2026-02-11: Rebuilt `rviz2_lidar.rviz` from Humble's
  `/opt/ros/humble/share/rviz_common/default.rviz` baseline and overlaid
  `/scan` LaserScan + fixed frame `car/chassis/ust10lx` after stripped config
  caused RViz displays to appear but not create topic subscriptions.
- 2026-02-11: Replaced `rviz2_lidar.rviz` with a minimal deterministic config
  (fixed frame `car/chassis/ust10lx`, LaserScan display on `/scan`) to match
  the known-good manual RViz setup and avoid display/QoS config side effects.
- 2026-02-11: Re-added static TF publisher to
  `fortress_teleop_rviz.launch.py` (`base_link` ->
  `car/chassis/ust10lx`) so RViz fixed frame exists at startup and LaserScan
  subscribes immediately.
- 2026-02-11: Simplified `fortress_teleop_rviz.launch.py` to match manual RViz
  success path: no forced `use_sim_time`, removed static TF publisher, and load
  RViz config from source tree by default. Set `rviz2_lidar.rviz` fixed frame to
  `car/chassis/ust10lx`.
- 2026-02-11: Switched ros_gz_bridge message type names in Fortress launch
  files from `gz.msgs.*` to `ignition.msgs.*` (`/scan`, `/cmd_vel`, `/clock`)
  to match `ign topic` transport types and restore scan payload flow to ROS2.
- 2026-02-10: Synced Fortress setup with Gazebo sensor docs: switched
  racetrack_decorated_2_hokuyo.world system plugin filenames to
  `ignition-gazebo-*-system`, added UserCommands plugin, set lidar topic to
  `/scan`, and changed lidar tag to `<always_on>true</always_on>`.
- 2026-02-10: Updated ros_gz bridge mappings in fortress_sim.launch.py and
  fortress_teleop_rviz.launch.py to one-way directions (`GZ->ROS` for `/scan`
  and `/clock`, `ROS->GZ` for `/cmd_vel`) to avoid scan/clock feedback loops.
- 2026-02-10: Replaced rviz2_lidar.rviz with explicit Grid + TF + LaserScan
  display config and explicit `/scan` QoS settings for reliable lidar display.
- 2026-02-05: Reduced chassis mass to 1.0, lowered COM to z=-0.05, increased
  steering limits (0.7 rad) and max velocity/accel in racetrack_decorated_2*.world.
- 2026-02-05: Lowered chassis COM, increased chassis mass, increased wheel
  friction, and dropped lidar visual to roof level in racetrack_decorated_2*.world.
- 2026-02-05: Set wheel joint axes to use parent model frame in
  racetrack_decorated_2*.world to correct wheel spin direction with steering.
- 2026-02-05: Enabled Grid + TF displays in RViz config and added explicit
  gravity / physics step settings to racetrack_decorated_2_hokuyo.world.
- 2026-02-05: Added static TF publisher and configurable scan/base frames in
  fortress_teleop_rviz.launch.py; RViz fixed frame set to base_link.
- 2026-02-05: Added Physics + SceneBroadcaster system plugins and physics
  settings to racetrack_decorated_2_hokuyo.world to match the working
  visualize_lidar.sdf pattern.
- 2026-02-05: Updated racetrack_decorated_2_hokuyo.world to SDF 1.6 and
  gpu_lidar `<lidar>` block (matching Ignition visualize_lidar.sdf) so /scan
  can publish.
- 2026-02-05: Added minimal lidar-only test world
  `ros_ws/src/simulation/racer_world/worlds/lidar_test.world` for isolating
  Ignition /scan publishing issues.
- 2026-02-05: Reverted Hokuyo world lidar block to match
  racetrack_decorated_2.world (gpu_lidar + relative_to + topic scan).
- 2026-02-05: Replaced Fuel Hokuyo include in racetrack_decorated_2_hokuyo.world
  with the inline lidar sensor/visual from racetrack_decorated_2.world, using
  SDF 1.4-compatible ray sensor and explicit pose.
- 2026-02-05: Set Hokuyo lidar pose explicitly (no relative_to) to avoid SDF
  1.4 frame issues and get /scan publishing.
- 2026-02-05: Switched Hokuyo world lidar sensor to CPU `ray` for better
  compatibility after /scan still produced no messages.
- 2026-02-05: Changed Hokuyo world lidar sensor to `gpu_ray` for SDF 1.4
  compatibility so /scan publishes correctly in Ignition.
- 2026-02-05: Fixed ROS2 launch world path to use F1TENTH_T3_ROOT (default
  ~/f1tenth-t3) so Ignition can find the world from install space.
- 2026-02-04: Added ROS2 Fortress teleop+RViz launch and RViz lidar config;
  Hokuyo world now publishes /scan from an explicit lidar sensor.
- 2026-02-04: Fixed wheel joint axes (spin on Y) and tightened steering limits/
  Ackermann parameters in racetrack_decorated_2*.world for realistic turning.
- 2026-02-04: Noted `ign` CLI required for Gazebo Fortress (no `gz`).
- 2026-01-31: Switched ROS2 Fortress launch to use `ign gazebo -r` instead of
  `gz sim` for compatibility with the current setup.
- 2026-01-31: Noted zsh shell, pre-sourced ROS2 setup, and installed
  Humble + Fortress + ros_gz bridge assumptions.
- 2026-01-31: Added ROS2 wallfollowing2 (rclpy), car_control (rclcpp), and
  racer_bringup launch with ros_gz_bridge for /scan and /cmd_vel.
- 2026-01-31: Added ROS2 workspace scaffold with `drive_msgs` interface package
  and `DriveParam.msg` to start the migration.
- 2026-01-31: Clarified ROS2 Humble + Gazebo Fortress migration goal; noted
  repo is a legacy ROS1/Gazebo project with limited modifications so far.
- 2026-01-31: Created AGENTS.md for cross-session continuity.

## Update checklist
- If you change Gazebo worlds or models, note which files and why.
- If you add/modify ROS packages or launch files, summarize the intent.
- If you change dependencies or setup steps, update this file and README.md.
- Track migration steps to ROS2 Humble / Gazebo Fortress and any blockers.
