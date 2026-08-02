from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    web_video = Node(
        package="web_video_server",
        executable="web_video_server",
        parameters=[{
            "port": 8080,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    serve = Node(
        package="web_dashboard",
        executable="serve_dashboard",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        rosbridge,
        web_video,
        serve,
    ])
