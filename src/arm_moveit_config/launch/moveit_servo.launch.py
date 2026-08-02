import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MOVEIT_CFG = "arm_moveit_config"


def _servo_actions(context, *args, **kwargs):
    cfg_dir = get_package_share_directory(MOVEIT_CFG)
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_semantic = ParameterValue(
        Command(["cat ", os.path.join(cfg_dir, "srdf", "arm_6dof.srdf")]),
        value_type=str)

    actions = [
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[
                os.path.join(cfg_dir, "config", "servo.yaml"),
                {"robot_description_semantic": robot_description_semantic},
                {"use_sim_time": use_sim_time},
                os.path.join(cfg_dir, "config", "kinematics.yaml"),
                os.path.join(cfg_dir, "config", "joint_limits.yaml"),
            ],
            output="screen",
        ),
    ]

    if LaunchConfiguration("run_move_group").perform(context) == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cfg_dir, "launch", "move_group.launch.py")),
            launch_arguments=[("use_sim_time", use_sim_time)],
        ))

    if LaunchConfiguration("run_rviz").perform(context) == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cfg_dir, "launch", "moveit_rviz.launch.py")),
            launch_arguments=[("use_sim_time", use_sim_time)],
        ))
    return actions


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    declare_run_move_group = DeclareLaunchArgument(
        "run_move_group", default_value="true")
    declare_run_rviz = DeclareLaunchArgument(
        "run_rviz", default_value="true")

    return LaunchDescription([
        declare_use_sim_time,
        declare_run_move_group,
        declare_run_rviz,
        OpaqueFunction(function=_servo_actions),
    ])
