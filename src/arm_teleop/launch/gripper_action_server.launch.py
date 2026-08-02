from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gripper_action_server = Node(
        package="arm_teleop",
        executable="gripper_action_server",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        gripper_action_server,
    ])
