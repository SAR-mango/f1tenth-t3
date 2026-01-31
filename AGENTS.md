# AGENTS.md
This file is a continuity note for Codex-style agents working in this repo.
Update it whenever you make major changes (new features, architecture changes,
new dependencies, new workflows, or significant refactors).

## Project overview
- Legacy ROS1 (catkin) workspace for 1/10th-scale autonomous racing (F1/10),
  originally built around older Gazebo.
- Current goal: migrate to ROS2 Humble and Gazebo Fortress to simulate an
  autonomous driving algorithm for a lidar-equipped car.
- Supports simulation (Gazebo) and real hardware; only a few world files and
  some new folders have been modified so far.
- Core autonomy: wallfollowing, navigation stack/SLAM, reinforcement learning,
  and evolutionary neural network training (legacy ROS1 implementations).

## Repository layout
- ros_ws/                     # ROS workspace
  - launch/                   # Top-level launch files
  - src/                      # ROS packages
    - autonomous/             # Driving algorithms and learning
    - car_control/            # Drive parameters, control, and safety
    - car_tf/                 # TF transforms and laser scan handling
    - hardware/               # Real-car hardware configs
    - navigation_stack/       # SLAM/navigation stack integration
    - simulation/             # Gazebo worlds, models, and simulation tools
    - teleoperation/          # Keyboard/joystick control
- doc/                        # Documentation and assets
- scripts/                    # Helper scripts (formatting, cartographer, etc.)

## Key simulation assets
- ros_ws/src/simulation/racer_world/worlds/
  - racetrack_decorated_2_hokuyo.world (custom world with Hokuyo model)
  - racetrack_decorated_2.world
  - hokuyo_ust10lx/ (model config, SDF, mesh, textures)

## Common workflows
- Build: `catkin_make` from `ros_ws/`
- Source: `source devel/setup.bash` (or `setup.zsh`)
- Launch: see `ros_ws/launch/*.launch` (gazebo, nav stack, RL training)

## Continuity log (append newest on top)
- 2026-01-31: Clarified ROS2 Humble + Gazebo Fortress migration goal; noted
  repo is a legacy ROS1/Gazebo project with limited modifications so far.
- 2026-01-31: Created AGENTS.md for cross-session continuity.

## Update checklist
- If you change Gazebo worlds or models, note which files and why.
- If you add/modify ROS packages or launch files, summarize the intent.
- If you change dependencies or setup steps, update this file and README.md.
- Track migration steps to ROS2 Humble / Gazebo Fortress and any blockers.
