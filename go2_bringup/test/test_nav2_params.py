"""
Static checks on go2_navigation/config/nav2_params.yaml.

The previous Nav2 config named a BT plugin library that does not exist in
Humble, so bt_navigator failed on_configure and navigation never started.
These tests make that class of mistake a test failure instead of a lab-session
failure, and pin the velocity budget inside the arbiter's envelope.
"""
from __future__ import annotations

import glob
import os
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PARAMS = yaml.safe_load((ROOT / "go2_navigation/config/nav2_params.yaml").read_text())
SAFETY = yaml.safe_load((ROOT / "go2_bringup/config/safety.yaml").read_text())
ROS_PREFIX = Path(os.environ.get("ROS_PREFIX", "/opt/ros/humble"))

requires_humble = pytest.mark.skipif(
    not (ROS_PREFIX / "share/nav2_bt_navigator").is_dir(), reason="Nav2 Humble not installed"
)


def _arbiter_limits():
    node = SAFETY["safety_arbiter_node"]["ros__parameters"]
    return node


def _registered_plugin_classes() -> set[str]:
    classes = set()
    for xml in glob.glob(str(ROS_PREFIX / "share/*/*.xml")):
        text = Path(xml).read_text(errors="ignore")
        if "<class" not in text:
            continue
        for chunk in text.split("<class")[1:]:
            for key in ("name=", "type="):
                if key in chunk.split(">")[0]:
                    val = chunk.split(key, 1)[1].split('"')[1]
                    classes.add(val)
    return classes


@requires_humble
def test_every_bt_plugin_library_exists():
    libs = PARAMS["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    missing = [lib for lib in libs if not (ROS_PREFIX / "lib" / f"lib{lib}.so").exists()]
    assert not missing, f"BT plugin libraries absent from Humble: {missing}"


@requires_humble
def test_every_server_plugin_is_registered():
    registered = _registered_plugin_classes()
    wanted = []
    ctrl = PARAMS["controller_server"]["ros__parameters"]
    wanted += [ctrl[name]["plugin"] for name in ctrl["controller_plugins"]]
    wanted += [ctrl[ctrl["progress_checker_plugin"]]["plugin"]]
    wanted += [ctrl[name]["plugin"] for name in ctrl["goal_checker_plugins"]]
    plan = PARAMS["planner_server"]["ros__parameters"]
    wanted += [plan[name]["plugin"] for name in plan["planner_plugins"]]
    beh = PARAMS["behavior_server"]["ros__parameters"]
    wanted += [beh[name]["plugin"] for name in beh["behavior_plugins"]]
    sm = PARAMS["smoother_server"]["ros__parameters"]
    wanted += [sm[name]["plugin"] for name in sm["smoother_plugins"]]
    for cm in ("local_costmap", "global_costmap"):
        p = PARAMS[cm][cm]["ros__parameters"]
        wanted += [p[layer]["plugin"] for layer in p["plugins"]]
    missing = [w for w in wanted if w not in registered]
    assert not missing, f"plugins not registered with pluginlib: {missing}"


def test_smoother_limits_sit_inside_the_arbiter_envelope():
    arb = _arbiter_limits()
    vs = PARAMS["velocity_smoother"]["ros__parameters"]
    vmax, vmin = vs["max_velocity"], vs["min_velocity"]
    assert vmax[0] < arb["max_vx"] and abs(vmin[0]) < arb["max_vx"]
    assert vmax[1] <= arb["max_vy"]
    assert vmax[2] < arb["max_wz"] and abs(vmin[2]) < arb["max_wz"]
    assert vs["max_accel"][0] < arb["max_accel_linear"]
    assert vs["max_accel"][2] < arb["max_accel_angular"]
    ctrl = PARAMS["controller_server"]["ros__parameters"]["FollowPath"]
    assert ctrl["desired_linear_vel"] <= vmax[0]
    assert ctrl["rotate_to_heading_angular_vel"] <= vmax[2]


def test_costmaps_publish_full_maps_for_semantic_freshness_gate():
    for cm in ("local_costmap", "global_costmap"):
        assert PARAMS[cm][cm]["ros__parameters"]["always_send_full_costmap"] is True


def test_no_sim_time_on_hardware_config():
    for name, block in PARAMS.items():
        params = block.get("ros__parameters") or block.get(name, {}).get("ros__parameters", {})
        assert params.get("use_sim_time", False) is False, name
