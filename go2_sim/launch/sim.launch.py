"""
sim.launch.py: the kinematic GO2 simulator on its own.

    ros2 launch go2_sim sim.launch.py [world_file:=...] [clock_skew_sec:=...]

Stands in for the robot only. Start the motion chain separately with
hardware_adapter:=unitree_sport; the sim consumes the Sport API requests that
adapter publishes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_world = PathJoinSubstitution(
        [FindPackageShare("go2_sim"), "worlds", "apartment.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument(
                "clock_skew_sec",
                default_value="-27605481.0",
                description="Robot clock minus wall clock, as measured on the real GO2.",
            ),
            Node(
                package="go2_sim",
                executable="go2_kinematic_sim_node",
                name="go2_kinematic_sim_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "world_file": LaunchConfiguration("world_file"),
                        "clock_skew_sec": LaunchConfiguration("clock_skew_sec"),
                    }
                ],
            ),
        ]
    )
