import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MOVEIT_CFG = "arm_moveit_config"


def generate_launch_description():
    pkg_dir = get_package_share_directory("arm_pick")
    cfg_dir = get_package_share_directory(MOVEIT_CFG)
    desc_dir = get_package_share_directory("arm_description")

    declare_mode = DeclareLaunchArgument(
        "localization_mode", default_value="ground_truth")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    mode = LaunchConfiguration("localization_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["cat ", os.path.join(desc_dir, "urdf", "arm_6dof.urdf")]),
        value_type=str)
    robot_description_semantic = ParameterValue(
        Command(["cat ", os.path.join(cfg_dir, "srdf", "arm_6dof.srdf")]),
        value_type=str)

    pick_node = Node(
        package="arm_pick",
        executable="pick_node",
        parameters=[
            os.path.join(pkg_dir, "config", "pick_params.yaml"),
            {"localization_mode": mode},
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            os.path.join(cfg_dir, "config", "kinematics.yaml"),
            os.path.join(cfg_dir, "config", "joint_limits.yaml"),
            os.path.join(cfg_dir, "config", "ompl_planning.yaml"),
            os.path.join(cfg_dir, "config", "moveit_controllers.yaml"),
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_mode,
        declare_use_sim_time,
        pick_node,
    ])
