import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "arm_world"


def _gz_actions(context, *args, **kwargs):
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")
    world_path = os.path.join(
        get_package_share_directory("arm_description"),
        "world", "arm_world.sdf")
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")),
        launch_arguments=[
            ("gz_args", f"-s -r -v 2 {world_path}"),
            ("on_exit_shutdown", "true"),
        ],
    )
    actions = [server]
    if LaunchConfiguration("use_gui").perform(context) == "true":
        if LaunchConfiguration("stream_gui").perform(context) == "true":
            stream_script = LaunchConfiguration("stream_script").perform(context)
            actions.append(ExecuteProcess(
                cmd=["bash", stream_script, "start"],
                output="screen",
            ))
            # GUI rendered into Xvfb (:99) and streamed to the browser via noVNC
            actions.append(ExecuteProcess(
                cmd=["gz", "sim", "-v", "2", "-g", world_path],
                additional_env={
                    "DISPLAY": ":99",
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                },
                output="screen",
            ))
        else:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")),
                launch_arguments=[("gz_args", f"-v 2 -g {world_path}")],
            ))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory("arm_description")

    declare_use_gui = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Start the Gazebo GUI")
    declare_stream_gui = DeclareLaunchArgument(
        "stream_gui", default_value="false",
        description="Render the Gazebo GUI into Xvfb and stream it to the "
                    "browser via noVNC (needs scripts/stream_gazebo_gui.sh)")
    declare_stream_script = DeclareLaunchArgument(
        "stream_script",
        default_value="/project/scripts/stream_gazebo_gui.sh",
        description="Path to the noVNC stream bootstrap script")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) time")

    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_path = os.path.join(pkg_dir, "urdf", "arm_6dof.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    spawn_arm = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-world", WORLD_NAME,
            "-topic", "/robot_description",
            "-name", "arm_6dof",
            "-allow_renaming", "false",
            "-timeout", "60",
        ],
        output="screen",
    )

    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    spawn_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    spawn_finger_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "finger_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera_table@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_stick@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_ee@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_table/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_stick/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_ee/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/world/arm_world/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/camera_table/detected@sensor_msgs/msg/Image]gz.msgs.Image",
            "/camera_stick/detected@sensor_msgs/msg/Image]gz.msgs.Image",
            "/camera_ee/detected@sensor_msgs/msg/Image]gz.msgs.Image",
            "--ros-args",
            "-r", "/camera_table:=/camera_table/image_raw",
            "-r", "/camera_stick:=/camera_stick/image_raw",
            "-r", "/camera_ee:=/camera_ee/image_raw",
            "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_use_gui,
        declare_stream_gui,
        declare_stream_script,
        declare_use_sim_time,
        OpaqueFunction(function=_gz_actions),
        robot_state_publisher,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_arm,
                on_exit=[spawn_joint_state_broadcaster]),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_joint_trajectory_controller]),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_joint_trajectory_controller,
                on_exit=[spawn_finger_controller]),
        ),
        spawn_arm,
        bridge,
    ])
