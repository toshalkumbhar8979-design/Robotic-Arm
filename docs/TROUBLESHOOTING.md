# Troubleshooting

## Setup / environment

- **`apt search ros-lyrical-*` finds nothing** — the mirror may not carry
  Lyrical yet or the ROS repo key is stale. Re-run
  `scripts/setup_ubuntu.sh`; verify with
  `apt-cache policy ros-lyrical-desktop`.
- **Package name differs** (e.g. `web-video-server`) — search the exact
  binary name: `apt search <partial name>` and edit the script.
- **Gazebo GUI won't open in WSL** — WSLg should forward it. If the window
  is blank: `wsl --update`, then
  `export LIBGL_ALWAYS_SOFTWARE=1` before launching (software GL fallback).
- **No /dev/input/js0** — attach the gamepad on Windows:
  `usbipd list` → `usbipd bind --busid <BUSID>` (elevated) →
  `usbipd attach --wsl --busid <BUSID>`, then relaunch the joy node.

## Launch / build

- **`colcon build` fails on a package** — install its missing system dep
  (see `package.xml`), then rebuild just that package:
  `colcon build --symlink-install --packages-select <pkg>`.
- **`ros2 launch ...` "package not found"** — the workspace is not sourced:
  `source ~/ros2_ws/install/setup.bash`, or re-run `sync_to_wsl.sh`.
- **Simulator hangs at startup** — first Gazebo start compiles shaders;
  wait. If it still hangs: `pkill -f "gz sim"` and relaunch.
- **`gz: command not found`** — install the `gz-sim10` deb or source
  `/opt/ros/lyrical/setup.bash` (ros_gz puts `gz` shims on PATH).

## Runtime

- **Arm never spawns in Gazebo** — check the gz server log. Common causes:
  spawn raced the world (the launch waits with `-timeout 60`; if the world
  isn't up in 60 s, restart), or `/robot_description` was never published
  (check `ros2 topic echo /robot_description`).
- **Controllers not active** — `ros2 control list_controllers`; expect
  `joint_trajectory_controller` (active), `finger_controller` (active),
  `joint_state_broadcaster` (active). If spawners died, the launch chain
  stopped: restart `arm_sim.launch.py`.
- **MoveIt can't find the robot description** — `move_group.launch.py`
  reads the URDF via `Command(["cat ", ...])`; run
  `ros2 param get /move_group robot_description` to verify.
- **Servo drifts / stops** — `/cmd_vel` must arrive continuously while the
  enable button is held (incoming_command_timeout=0.5 s). Also confirm
  `teleop_twist_joy` gets `/joy` (`ros2 topic echo /joy`).
- **Pick fails with "no plan"** — the arm cannot reach the target from the
  current pose, or blocks are out of the workspace. Watch
  `/arm_pick/status`; try `pre_pick` posture adjustments or move blocks
  closer to the arm in `world/gen_arm_world.py`.
- **Vision mode finds no blocks** — check camera stream
  (<http://localhost:8080/stream?topic=/camera_table/image_raw>) and tune
  HSV bounds in `src/arm_pick/config/pick_params.yaml`.
- **Gripper does not close** — `finger_controller` must be active and the
  topic is `/finger_controller/commands`; verify
  `ros2 topic echo /joint_states` shows `left_finger_joint` moving.

## Web dashboard

- **Page loads but model is missing** — rosbridge (9090) must be running
  and `/robot_description` latched; check browser console.
- **Cameras show a broken image** — `web_video_server` must run with
  `use_sim_time=true`; streams need the sim to be publishing image topics.
- **CDN blocked (offline)** — download three/urdf-loader/roslib into
  `web/js/vendor/` and update the import map in `index.html`.
