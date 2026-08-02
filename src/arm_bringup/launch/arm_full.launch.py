import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, file, **overrides):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg),
                         "launch", file)),
        launch_arguments=[(k, v if isinstance(v, Substitution) else str(v))
                          for k, v in overrides.items()],
    )


def generate_launch_description():
    declare_mode = DeclareLaunchArgument(
        "localization_mode", default_value="ground_truth")
    declare_use_gui = DeclareLaunchArgument(
        "use_gui", default_value="true")
    declare_stream_gui = DeclareLaunchArgument(
        "stream_gui", default_value="false",
        description="Stream the Gazebo GUI to the browser via noVNC "
                    "(scripts/stream_gazebo_gui.sh)")
    declare_run_rviz = DeclareLaunchArgument(
        "run_rviz", default_value="true")
    declare_joy_device = DeclareLaunchArgument(
        "joy_device", default_value="/dev/input/js0")
    declare_controller_type = DeclareLaunchArgument(
        "controller_type", default_value="auto",
        description="Gamepad layout: auto | ps5 | xbox | f710")
    declare_use_native_joy = DeclareLaunchArgument(
        "use_native_joy", default_value="true",
        description="Run the WSL joy_node; set false to use the browser "
                    "gamepad bridge (/joy via rosbridge) instead")

    mode = LaunchConfiguration("localization_mode")
    use_gui = LaunchConfiguration("use_gui")
    stream_gui = LaunchConfiguration("stream_gui")
    run_rviz = LaunchConfiguration("run_rviz")
    joy_device = LaunchConfiguration("joy_device")
    controller_type = LaunchConfiguration("controller_type")
    use_native_joy = LaunchConfiguration("use_native_joy")

    sim = _include("arm_description", "arm_sim.launch.py",
                   use_gui=use_gui, stream_gui=stream_gui)
    move_group = _include("arm_moveit_config", "move_group.launch.py")
    servo = _include("arm_moveit_config", "moveit_servo.launch.py",
                     run_move_group="false", run_rviz=run_rviz)
    teleop = _include("arm_teleop", "teleop_joy.launch.py",
                      joy_device=joy_device, controller_type=controller_type,
                      use_native_joy=use_native_joy)
    gripper = _include("arm_teleop", "gripper_action_server.launch.py")
    pick = _include("arm_pick", "arm_pick.launch.py",
                    localization_mode=mode)
    web = _include("web_dashboard", "web.launch.py")

    return LaunchDescription([
        declare_mode,
        declare_use_gui,
        declare_stream_gui,
        declare_run_rviz,
        declare_joy_device,
        declare_controller_type,
        declare_use_native_joy,
        sim,
        move_group,
        servo,
        teleop,
        gripper,
        pick,
        web,
    ])
