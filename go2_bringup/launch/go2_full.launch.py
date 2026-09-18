"""
DEPRECATED, superseded by ``system.launch.py``.

This file used to be the main entrypoint. It is retained only to fail loudly,
because launching the graph it described was worse than launching nothing:

* It started Nav2 with a params file referencing ``nav2_reactive_fallback_bt_node``,
  a behaviour-tree plugin library that does not exist in ROS 2 Humble, so
  ``bt_navigator`` failed to configure and the whole Nav2 lifecycle never came up.
* It passed ``default_nav_to_pose_bt_xml`` to ``nav2_bringup/navigation_launch.py``,
  which does not declare that argument, so the repository's own behaviour tree
  was silently ignored while a preflight check verified the file existed.
* Nothing in the graph subscribed to ``/cmd_vel``. Even a working Nav2 would
  have terminated at an unconsumed topic: the launch file started no hardware
  bridge and no gait controller at all.
* Its ``recoveries_server`` parameter block targeted a node name from a
  previous Nav2 release (``behavior_server`` in Humble) and was never read.

Use instead::

    ros2 launch go2_bringup system.launch.py                 # real perception
    ros2 launch go2_bringup system_dry_run.launch.py         # no hardware

See ``docs/runtime_graph_audit.md`` for the full account of what this launch
file actually did, and ``docs/target_runtime_architecture.md`` for what
replaced it.
"""
from __future__ import annotations

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    raise RuntimeError(
        "go2_full.launch.py is deprecated and intentionally non-functional.\n"
        "It started a Nav2 stack that could not complete lifecycle bring-up, "
        "and no node in its graph consumed /cmd_vel, so no command could reach "
        "the robot.\n"
        "Use:  ros2 launch go2_bringup system.launch.py\n"
        "or:   ros2 launch go2_bringup system_dry_run.launch.py\n"
        "See docs/target_runtime_architecture.md."
    )
