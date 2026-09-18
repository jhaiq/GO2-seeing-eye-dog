"""
localization.launch.py: the frame tree Nav2 needs, built from the stock GO2.

    /utlidar/robot_odom + LiDAR --go2_state_relay_node--> /odom, TF odom->base_link,
                                                          /go2/lidar/points,
                                                          /go2/localization_valid
    /go2/lidar/points --pointcloud_to_laserscan--> /scan (base_link)
    /scan + TF --slam_toolbox--> /map, TF map->odom

Arguments
---------
``slam_mode``   mapping | localization   (default: mapping)
    ``mapping`` builds a map while navigating. ``localization`` loads a
    serialized pose graph (``map_file``, without extension) saved with
    ``ros2 service call /slam_toolbox/serialize_map ...``.
``map_file``    pose-graph path for localization mode.
``cloud_in_topic``  /utlidar/cloud_deskewed (default) or /utlidar/cloud.
``publish_lidar_extrinsic``  true only for the raw sensor-frame cloud.
``require_map_frame``  localization_valid also requires map->odom (default true).
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cfg(name: str):
    return PathJoinSubstitution([FindPackageShare("go2_localization"), "config", name])


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_mode = LaunchConfiguration("slam_mode")

    relay = Node(
        package="go2_localization",
        executable="go2_state_relay_node",
        name="go2_state_relay_node",
        output="screen",
        parameters=[
            _cfg("relay.yaml"),
            {
                "use_sim_time": use_sim_time,
                "cloud_in_topic": LaunchConfiguration("cloud_in_topic"),
                "publish_lidar_extrinsic": LaunchConfiguration("publish_lidar_extrinsic"),
                "require_map_frame": LaunchConfiguration("require_map_frame"),
                "restamp_mode": LaunchConfiguration("restamp_mode"),
            },
        ],
    )

    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[_cfg("pointcloud_to_laserscan.yaml"), {"use_sim_time": use_sim_time}],
        remappings=[("cloud_in", "/go2/lidar/points"), ("scan", "/scan")],
    )

    is_mapping = IfCondition(PythonExpression(["'", slam_mode, "' == 'mapping'"]))
    is_localization = IfCondition(PythonExpression(["'", slam_mode, "' == 'localization'"]))

    slam_mapping = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[_cfg("slam_toolbox.yaml"), {"use_sim_time": use_sim_time, "mode": "mapping"}],
        condition=is_mapping,
    )
    slam_localization = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            _cfg("slam_toolbox.yaml"),
            {
                "use_sim_time": use_sim_time,
                "mode": "localization",
                "map_file_name": LaunchConfiguration("map_file"),
                "map_start_at_dock": True,
            },
        ],
        condition=is_localization,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("cloud_in_topic", default_value="/utlidar/cloud_deskewed"),
            DeclareLaunchArgument("publish_lidar_extrinsic", default_value="false"),
            DeclareLaunchArgument("require_map_frame", default_value="true"),
            DeclareLaunchArgument("restamp_mode", default_value="offset"),
            relay,
            scan,
            slam_mapping,
            slam_localization,
        ]
    )
