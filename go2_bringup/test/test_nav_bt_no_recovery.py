"""nav_bt:=no_recovery wiring, checked without starting anything.

The default must add nothing to bt_navigator (today's params file and Humble's
stock tree); no_recovery must point default_nav_to_pose_bt_xml at a tree that
has no recovery actions and no retry nodes.
"""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TREE = ROOT / "go2_navigation" / "behavior_trees" / "navigate_to_pose_no_recovery.xml"


def _nav_launch():
    pytest.importorskip("launch_ros")
    pytest.importorskip("nav2_common")
    spec = importlib.util.spec_from_file_location(
        "go2_navigation_launch", ROOT / "go2_navigation" / "launch" / "navigation.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_default_adds_no_bt_navigator_parameters():
    assert _nav_launch().bt_navigator_extra_params("default") == []


def test_no_recovery_selects_the_no_recovery_tree(monkeypatch):
    mod = _nav_launch()
    monkeypatch.setattr(mod, "no_recovery_bt_path", lambda: str(TREE))
    assert mod.bt_navigator_extra_params("no_recovery") == [
        {"default_nav_to_pose_bt_xml": str(TREE)}]


def test_unknown_choice_is_refused():
    with pytest.raises(ValueError):
        _nav_launch().bt_navigator_extra_params("no_recovry")


def test_tree_has_no_recovery_actions_or_retries():
    root = ET.parse(TREE).getroot()
    assert "BTCPP_format" not in root.attrib  # Humble is BT.CPP v3
    tags = {e.tag for e in root.iter()}
    forbidden = {"Spin", "BackUp", "Wait", "DriveOnHeading", "ClearEntireCostmap",
                 "ClearCostmapAroundRobot", "ClearCostmapExceptRegion", "RecoveryNode",
                 "RoundRobin", "ReactiveFallback", "Fallback", "AssistedTeleop"}
    assert not tags & forbidden, tags & forbidden
    assert {"ComputePathToPose", "FollowPath", "RateController"} <= tags
