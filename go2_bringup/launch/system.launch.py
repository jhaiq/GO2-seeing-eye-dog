"""
system.launch.py — the canonical entrypoint for the GO2 seeing-eye-dog stack.

    ros2 launch go2_bringup system.launch.py

This replaces the previous collection of launch files that had to be started
by hand in separate terminals and that, between them, never actually connected
perception to actuation.

Arguments
---------
``perception``      real | none        (default: real)
    ``real`` starts the microphone, camera, YOLO and depth-safety nodes, all
    of which require hardware. ``none`` starts none of them, for running the
    decision half against replayed or synthetic inputs.

``planner``         staged | nav2      (default: staged)
    ``staged``  go2_approach_controller: straight-line approach, no planning.
    ``nav2``    the real Nav2 stack, with its unstamped ``cmd_vel`` remapped
                into the candidate inlet so it can never reach the bridge.
                Requires a map, localization, odometry, TF and a laser scan —
                see docs/target_runtime_architecture.md before using it.

``hardware_adapter`` dry_run | unitree_sport   (default: dry_run)
    The default is dry_run. Selecting a physical adapter is an explicit,
    deliberate act.

The motion path is defined in exactly one place, ``motion_authority.launch.py``,
which every variant of this file includes unchanged.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from_motion_authority = PathJoinSubstitution(
    [FindPackageShare("go2_bringup"), "launch", "motion_authority.launch.py"]
)


def _config(name: str):
    return PathJoinSubstitution([FindPackageShare("go2_bringup"), "config", name])


def _perception_nodes(log_level):
    """Hardware-dependent perception. Every one of these needs a real device."""
    condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("perception"), "' == 'real'"])
    )
    args = ["--ros-args", "--log-level", log_level]
    return [
        Node(
            package="go2_audio_perception",
            executable="audio_perception_node",
            name="audio_perception_node",
            output="screen",
            arguments=args,
            condition=condition,
            remappings=[("/go2/audio/bearing_deg", "/go2/audio/bearing_deg")],
        ),
        Node(
            package="go2_voice_commander",
            executable="voice_commander_node",
            name="voice_commander_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
        Node(
            package="go2_perception",
            executable="perception_node",
            name="perception_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
        Node(
            package="go2_safety_monitor",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
    ]


def _grounding_node(log_level):
    return Node(
        package="go2_intent_grounding",
        executable="intent_grounding_node",
        name="intent_grounding_node",
        output="screen",
        emulate_tty=True,
        parameters=[_config("fusion.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("audio_bearing_deg", "/go2/audio/bearing_deg"),
            ("detected_humans", "/go2/detected_humans"),
            ("voice_command", "/go2/voice_command"),
            ("confirmed_target", "/go2/confirmed_target"),
            ("grounding_status", "/go2/grounding_status"),
            ("grounding_state", "/go2/grounding_state"),
            ("goal_pose", "/goal_pose"),
        ],
    )


def _staged_controller(log_level):
    return Node(
        package="go2_approach_controller",
        executable="approach_controller_node",
        name="approach_controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[_config("navigation.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("planner"), "' == 'staged'"])
        ),
        remappings=[
            ("goal_pose", "/goal_pose"),
            ("cancel_goal", "/go2/cancel_goal"),
            # Publishes the CANDIDATE topic. Not the safe topic. It has no
            # publisher of the safe topic's type at all.
            ("cmd_vel_candidate", "/cmd_vel_candidate"),
            ("controller/status", "/go2/controller/status"),
        ],
    )


def _nav2_group(log_level):
    """
    Real Nav2, with its velocity output diverted into the candidate inlet.

    The remapping ``cmd_vel -> /cmd_vel_candidate_unstamped`` is the entire
    integration. Nav2 believes it is driving the robot; it is driving the
    arbiter's inlet. Because the bridge consumes a different message type,
    even removing this remapping would not connect Nav2 to the hardware — it
    would connect Nav2 to nothing.
    """
    condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("planner"), "' == 'nav2'"])
    )
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nav2_bringup"),
                                "launch",
                                "navigation_launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": PathJoinSubstitution(
                        [
                            FindPackageShare("go2_navigation"),
                            "config",
                            "nav2_params.yaml",
                        ]
                    ),
                }.items(),
            ),
            # The stamper is the ONLY sanctioned producer on the unstamped
            # inlet, and the inlet is off unless this group is active.
            SetParameter(name="accept_unstamped_candidate", value=True),
            Node(
                package="go2_approach_controller",
                executable="candidate_stamper_node",
                name="candidate_stamper_node",
                output="screen",
                parameters=[_config("navigation.yaml")],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel_in", "/cmd_vel"),
                    ("cmd_vel_candidate", "/cmd_vel_candidate"),
                ],
            ),
        ],
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument(
            "perception",
            default_value="real",
            description="real (needs microphone, RealSense, YOLO weights) or none.",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="staged",
            description=(
                "staged (go2_approach_controller: straight-line, no planning) "
                "or nav2 (requires map, localization, odometry, TF, laser scan)."
            ),
        ),
        DeclareLaunchArgument(
            "hardware_adapter",
            default_value="dry_run",
            description="dry_run or unitree_sport.",
        ),
        DeclareLaunchArgument("dry_run_log_path", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    motion_authority = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([from_motion_authority]),
        launch_arguments={
            "hardware_adapter": LaunchConfiguration("hardware_adapter"),
            "dry_run_log_path": LaunchConfiguration("dry_run_log_path"),
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        declarations
        + _perception_nodes(log_level)
        + [
            _grounding_node(log_level),
            _staged_controller(log_level),
            _nav2_group(log_level),
            motion_authority,
        ]
    )
