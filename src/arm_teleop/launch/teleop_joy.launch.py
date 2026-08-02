import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("arm_teleop")

    declare_joy_device = DeclareLaunchArgument(
        "joy_device", default_value="/dev/input/js0")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    declare_controller_type = DeclareLaunchArgument(
        "controller_type", default_value="auto",
        description="Gamepad layout: auto | ps5 | xbox | f710")
    declare_use_native_joy = DeclareLaunchArgument(
        "use_native_joy", default_value="true",
        description="Run the WSL joy_node (native USB/Bluetooth gamepad). "
                    "Set to false when the browser gamepad bridge publishes /joy")

    joy_device = LaunchConfiguration("joy_device")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_type = LaunchConfiguration("controller_type")
    use_native_joy = LaunchConfiguration("use_native_joy")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{
            "dev": joy_device,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
        condition=IfCondition(use_native_joy),
    )

    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[
            os.path.join(pkg_dir, "config", "teleop_twist_joy.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    teleop_buttons = Node(
        package="arm_teleop",
        executable="teleop_buttons",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_type": controller_type},
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_joy_device,
        declare_use_sim_time,
        declare_controller_type,
        declare_use_native_joy,
        joy_node,
        teleop_twist_joy,
        teleop_buttons,
    ])
