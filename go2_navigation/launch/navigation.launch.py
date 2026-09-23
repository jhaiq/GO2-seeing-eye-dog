"""
navigation.launch.py: the Nav2 servers for the GO2, owned by this repository.

Equivalent to Humble's nav2_bringup/navigation_launch.py (non-composed),
with our params file as the default and one addition: ``controller_prefix``
runs controller_server under a wrapper, e.g. gdb, to capture thread stacks
when the controller's map->odom stall occurs (docs/DEPLOYMENT.md, open
issue). The velocity output is /cmd_vel (smoothed), which system.launch.py
diverts into the safety arbiter's candidate inlet.

``nav_bt``  default | no_recovery   (default: default)
    ``default`` leaves bt_navigator exactly as before: Humble's stock
    navigate_to_pose_w_replanning_and_recovery tree. ``no_recovery`` sets
    default_nav_to_pose_bt_xml to behavior_trees/navigate_to_pose_no_recovery.xml:
    same 1 Hz replanning, no recovery behaviours, a failure aborts the goal.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]

NAV_BT_CHOICES = ("default", "no_recovery")


def no_recovery_bt_path() -> str:
    return os.path.join(get_package_share_directory("go2_navigation"), "behavior_trees",
                        "navigate_to_pose_no_recovery.xml")


def bt_navigator_extra_params(nav_bt: str) -> list:
    """Parameters added on top of the params file for bt_navigator. [] for the default."""
    if nav_bt not in NAV_BT_CHOICES:
        raise ValueError(f"nav_bt must be one of {NAV_BT_CHOICES}, got {nav_bt!r}")
    if nav_bt == "no_recovery":
        return [{"default_nav_to_pose_bt_xml": no_recovery_bt_path()}]
    return []


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time, "autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    args = ["--ros-args", "--log-level", log_level]

    def server(package, executable, extra_remaps=(), prefix=None, extra_params=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params] + list(extra_params),
            arguments=args,
            remappings=remaps + list(extra_remaps),
            prefix=prefix,
        )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "nav_bt",
                default_value="default",
                description="default (stock Humble tree) or no_recovery (goal aborts on failure).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("go2_navigation"), "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "controller_prefix",
                default_value="",
                description="Command prefix for controller_server (debugging only).",
            ),
            server("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")],
                   prefix=LaunchConfiguration("controller_prefix")),
            server("nav2_smoother", "smoother_server"),
            server("nav2_planner", "planner_server"),
            server("nav2_behaviors", "behavior_server"),
            OpaqueFunction(function=lambda context: [server(
                "nav2_bt_navigator", "bt_navigator",
                extra_params=bt_navigator_extra_params(
                    LaunchConfiguration("nav_bt").perform(context)))]),
            server("nav2_waypoint_follower", "waypoint_follower"),
            server("nav2_velocity_smoother", "velocity_smoother",
                   [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=args,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": LIFECYCLE_NODES},
                ],
            ),
        ]
    )
