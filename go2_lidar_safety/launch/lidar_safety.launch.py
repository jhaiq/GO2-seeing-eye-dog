"""
lidar_safety.launch.py: start lidar_hazard_node wired to the arbiter topics.

    ros2 launch go2_lidar_safety lidar_safety.launch.py cloud_topic:=/go2/lidar/points

The node publishes /go2/safety_state and /go2/safety_alert, the same two
channels go2_safety_monitor uses. The arbiter resolves them most restrictive
wins, so running this alongside the depth monitor is safe.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/go2/lidar/points"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("go2_lidar_safety"), "config", "lidar_safety.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="go2_lidar_safety",
                executable="lidar_hazard_node",
                name="lidar_hazard_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                remappings=[
                    ("cloud", LaunchConfiguration("cloud_topic")),
                    ("safety_state", "/go2/safety_state"),
                    ("safety_alert", "/go2/safety_alert"),
                    ("lidar_safety/status", "/go2/lidar_safety/status"),
                ],
            ),
        ]
    )
