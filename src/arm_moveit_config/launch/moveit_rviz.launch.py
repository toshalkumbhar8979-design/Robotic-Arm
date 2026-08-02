import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION = "arm_description"
MOVEIT_CFG = "arm_moveit_config"


def generate_launch_description():
    desc_dir = get_package_share_directory(DESCRIPTION)
    cfg_dir = get_package_share_directory(MOVEIT_CFG)

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    declare_urdf = DeclareLaunchArgument(
        "urdf",
        default_value=os.path.join(desc_dir, "urdf", "arm_6dof.urdf"))
    declare_srdf = DeclareLaunchArgument(
        "srdf",
        default_value=os.path.join(cfg_dir, "srdf", "arm_6dof.srdf"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_path = LaunchConfiguration("urdf")
    srdf_path = LaunchConfiguration("srdf")

    robot_description = ParameterValue(
        Command(["cat ", urdf_path]), value_type=str)
    robot_description_semantic = ParameterValue(
        Command(["cat ", srdf_path]), value_type=str)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d",
                   os.path.join(cfg_dir, "launch", "moveit.rviz")],
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_urdf,
        declare_srdf,
        rviz,
    ])
