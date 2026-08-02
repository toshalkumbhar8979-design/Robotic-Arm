# ROS 2 — 6-DOF Robotic Arm Stack

Full ROS 2 (Lyrical Luth) / Gazebo (Jetty) stack for the BabyROS 6-DOF
manipulator on Windows via WSL 2: simulated arm with two cameras, MoveIt 2
motion planning, gamepad teleop (MoveIt Servo), autonomous pick-and-place,
and a browser-based digital twin.

```
C:\Users\tosha\Downloads\Roboticarm
├── scripts/                  setup + sync helpers (see below)
├── docs/
│   ├── ARCHITECTURE.md       node/graph overview
│   └── TROUBLESHOOTING.md    common issues
└── src/
    ├── arm_description/      URDF, world, controllers, Gazebo bridge
    ├── arm_moveit_config/    SRDF, kinematics, OMPL, servo, move_group
    ├── arm_teleop/           gamepad twist + joint jog + gripper action
    ├── arm_pick/             autonomous pick-and-place node
    ├── arm_bringup/          one-command full stack
    └── web_dashboard/        Three.js digital twin + camera streams
```

## 1. Setup

Windows host — run [scripts/setup_windows.ps1](scripts/setup_windows.ps1)
(updates WSL/WSLg; optional usbipd gamepad passthrough).

Inside WSL (Ubuntu 26.04) — run [scripts/setup_ubuntu.sh](scripts/setup_ubuntu.sh).
It installs ROS 2 Lyrical, MoveIt 2, `gz_ros2_control`, Gazebo Jetty,
rosbridge, web video server and OpenCV, and creates `~/ros2_ws`.

> If `ros-lyrical-*` packages are not found, run
> `apt search ros-lyrical- 2>/dev/null | grep <name>` to find the exact
> binary package name on your mirror and adjust the script.

## 2. Build

From WSL:

```bash
~/Downloads-folder: scripts/sync_to_wsl.sh   # rsync + colcon build
# or manually:
rsync -a /mnt/c/Users/tosha/Downloads/Roboticarm/src/ ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

## 3. Run

Full stack (sim + MoveIt + servo + teleop + pick + dashboard):

```bash
ros2 launch arm_bringup arm_full.launch.py
# vision-based picking instead:
ros2 launch arm_bringup arm_full.launch.py localization_mode:=vision
```

Individual pieces:

```bash
ros2 launch arm_description arm_sim.launch.py          # sim only
ros2 launch arm_moveit_config moveit_servo.launch.py   # move_group + rviz + servo
ros2 launch arm_teleop teleop_joy.launch.py            # gamepad -> /cmd_vel
ros2 launch arm_pick arm_pick.launch.py                # pick node (ground_truth)
ros2 launch arm_pick arm_pick.launch.py localization_mode:=vision
ros2 launch web_dashboard web.launch.py                # dashboard + rosbridge
```

## 4. Interfaces

| Topic / Service | Type | Description |
|---|---|---|
| `/cmd_vel` | Twist | servo input (teleop_twist_joy) |
| `/joint_states` | JointState | arm + finger states |
| `/joint_trajectory_controller/joint_trajectory` | JointTrajectory | arm motion |
| `/finger_controller/commands` | Float64MultiArray | finger position |
| `/gripper_action_controller/gripper_cmd` | GripperCommand action | MoveIt gripper |
| `/camera_table/image_raw`, `/camera_ee/image_raw` | Image | cameras |
| `/world/arm_world/pose/info` | PoseArray | ground-truth block poses |
| `/arm_pick/status` | String | pick status |
| `/arm_pick/trigger` | Trigger | start pick sequence |

Gamepad (Logitech F710): hold **LB** to drive (left stick = linear, right
stick = yaw); **A/B** jog joint_1, **X/Y** joint_2, **LB/RB** joint_3,
**Back** close gripper, **Start** open gripper.

## 5. Web dashboard

With the stack running, open <http://localhost:8000> for the digital twin
(3D model + joint states + pick control). Camera streams:
<http://localhost:8080/stream?topic=/camera_table/image_raw>.

## 6. Notes

- Blocks (red/green/blue) spawn on the table; `ground_truth` localization
  uses Gazebo poses, `vision` uses the table camera + OpenCV.
- The right finger is a mimic of the left (`multiplier -1`), so only
  `left_finger_joint` is commanded.
- If no gamepad is attached, move the arm with:
  `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
  or publish `/joint_trajectory_controller/joint_trajectory` goals.
