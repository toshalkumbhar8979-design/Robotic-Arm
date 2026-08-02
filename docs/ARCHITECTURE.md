# Architecture

```
                        ┌────────────────────────────── WSL 2 (Ubuntu 26.04) ──┐
 Windows                │                                                        │
 ┌──────────────┐       │  ┌──────────────┐   gz transport   ┌───────────────┐  │
 │ Browser      │ :8000 │  │ web_dashboard │ ─── websocket ──▶ │ rosbridge     │  │
 │ Three.js     │ :8080 │  │ serve (http)  │   (port 9090)   │ rosbridge_ws  │  │
 │ urdf-loader  │──────▶│  │               │ ◀── /stream ────│ web_video_    │  │
 └──────────────┘       │  └──────────────┘                  │ server (8080) │  │
  USB gamepad ─ usbipd ─┼─▶ ┌───────────────┐               └───────────────┘  │
                        │  │ joy_node      │ /joy                                │
                        │  │ teleop_twist_ │ /cmd_vel ──▶ moveit_servo          │
                        │  │ joy           │   (servo_server)                   │
                        │  └───────────────┘          │ delta joint cmds        │
                        │  ┌────────────────┐         ▼                         │
                        │  │ arm_pick       │─▶ MoveGroupCommander ─▶ move_group │
                        │  │ pick_node      │        (moveit_ros_move_group)    │
                        │  │ (ground_truth/ │         │                         │
                        │  │  vision)       │         ▼                         │
                        │  └────────────────┘  ┌───────────────┐                │
                        │   arm_teleop ───────▶│ controller_   │                │
                        │   gripper_action_    │ manager       │                │
                        │   server             │ (jtc, finger, │                │
                        │                      │  jsb)         │                │
                        │                      └──────┬────────┘                │
                        │  ┌─────────────────────────▼──────────┐               │
                        │  │ Gazebo Jetty (gz-sim 10)            │               │
                        │  │  arm_6dof (gz_ros2_control)         │               │
                        │  │  table + blocks + cameras + lights  │               │
                        │  └─────────────────────────────────────┘               │
                        └────────────────────────────────────────────────────────┘
```

## Data flow

1. **arm_description** — `arm_6dof.urdf` (source: `urdf/gen_arm_urdf.py`)
   declares the robot, two camera sensors and a `ros2_control` block using
   the `gz_ros2_control` GazeboSimSystem plugin. `arm_world.sdf` (source:
   `world/gen_arm_world.py`) is the physics world. `arm_controllers.yaml`
   defines three controllers:
   - `joint_state_broadcaster` → `/joint_states`
   - `joint_trajectory_controller` → arm joints 1–6
   - `finger_controller` (forward command) → `left_finger_joint`
   `arm_bridge.yaml` bridges clock, both cameras, and world pose info.
   The launch spawns the arm via `ros_gz_sim create -topic /robot_description`
   and chains controller spawners after the spawn completes.

2. **arm_moveit_config** — SRDF (source: `srdf/gen_arm_srdf.py`) defines
   groups `arm` (base_link→tool0), `gripper`, `arm_gripper`, end effector
   `gripper_eef` on `gripper_palm`, states `home`, `pre_pick`, `open`,
   `closed`, and adjacent-pair collision disables. KDL IK + OMPL
   (RRTConnect). `servo.yaml` runs at 200 Hz consuming `/cmd_vel`.
   `moveit_controllers.yaml` routes trajectories to the trajectory
   controller and gripper commands to the GripperCommand action server.

3. **arm_teleop** — `teleop_twist_joy` maps the gamepad to `/cmd_vel`;
   `teleop_buttons` jogs joints and drives the fingers;
   `gripper_action_server` implements
   `/gripper_action_controller/gripper_cmd`
   (`control_msgs/GripperCommand`) on top of the finger controller.

4. **arm_pick** — `pick_node` localizes blocks either from bridged Gazebo
   poses (`ground_truth`, default) or by HSV segmentation + ray-plane
   projection (`vision`), then runs: home → pre_pick → approach →
   cartesian descend → close gripper → lift → drop → open. Status on
   `/arm_pick/status`; triggered on startup or via `/arm_pick/trigger`.

5. **web_dashboard** — rosbridge websocket (9090), web video server (8080),
   static dashboard (8000). The browser loads `/robot_description`
   (latched) into urdf-loader, applies `/joint_states`, streams both
   cameras, and can trigger picking.

## Conventions

- All generated artifacts have Python sources (`gen_*.py`) as the
  single source of truth; regenerate with the urdf/sdf/srdf skill CLIs.
- Coordinates are meters/radians; joint limits live only in the URDF.
- `right_finger_joint` is a URDF mimic of `left_finger_joint` and is
  passive in MoveIt.
- Cameras: 30 Hz, 640×480, HFOV ≈65°; table camera looks at the table
  center from the side pole, EE camera looks forward from the palm.
