"""
system_dry_run.launch.py, the full decision stack with no hardware at all.

    ros2 launch go2_bringup system_dry_run.launch.py

Identical to ``system.launch.py`` except that hardware-dependent perception is
disabled and the actuation adapter is the recording dry-run bridge. Everything
between, caller confirmation, intent grounding, goal emission, the candidate
controller, and the safety arbiter, is the SAME CODE that runs on the robot.
That is the point: this variant exercises the real decision path, not a
simulation of it.

Perception inputs must be supplied externally (a bag, a test fixture, or
``ros2 topic pub``) on:

    /go2/detected_humans     go2_msgs/DetectedHumanArray
    /go2/audio/bearing_deg   std_msgs/Float32
    /go2/voice_command       std_msgs/String
    /go2/safety_state        std_msgs/String

Running this launch file does not validate anything about physical hardware,
and no artefact it produces may be described as hardware validation.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run_log_path", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("go2_bringup"),
                                "launch",
                                "system.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "perception": "none",
                    "planner": "staged",
                    "hardware_adapter": "dry_run",
                    "dry_run_log_path": LaunchConfiguration("dry_run_log_path"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ]
    )
