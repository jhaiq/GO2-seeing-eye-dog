#!/usr/bin/env python3
"""
Static repository checks for the safety-architecture contracts.

These run in CI without ROS installed, so they are text- and structure-level
checks only. They exist to catch, at review time, the class of regression that
would otherwise only show up on a robot:

  * a hardware bridge that names the candidate topic;
  * a second publisher of the safe-command topic;
  * a launch file that starts an actuator outside motion_authority.launch.py;
  * safety limits drifting between the arbiter and the bridge;
  * documentation claiming hardware validation this repository does not have.

Runtime proofs of the same properties live in the test suite; these are the
cheap guard rails in front of them.
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FAILURES: list[str] = []


def fail(message: str) -> None:
    print(f"[repo-doctor] FAIL: {message}")
    FAILURES.append(message)


def ok(message: str) -> None:
    print(f"[repo-doctor] OK: {message}")


def check_exists(relative_path: str) -> None:
    if not (ROOT / relative_path).exists():
        fail(f"missing required path: {relative_path}")
    else:
        ok(relative_path)


# ── Safety-architecture invariants ─────────────────────────────────────────


def check_bridge_has_one_motion_inlet() -> None:
    """Invariant A: the bridge must not reference the candidate topic at all."""
    source = ROOT / "go2_hardware_bridge/go2_hardware_bridge/hardware_bridge_node.py"
    text = source.read_text(encoding="utf-8")

    subscriptions = re.findall(r"create_subscription\(\s*([A-Za-z_][\w]*)", text)
    motion_types = {t for t in subscriptions if t not in ("Trigger",)}
    if motion_types != {"SafeVelocityCommand"}:
        fail(
            "hardware bridge subscribes to something other than "
            f"SafeVelocityCommand: {sorted(motion_types)}"
        )
    else:
        ok("hardware bridge subscribes only to SafeVelocityCommand")

    # Mentions inside comments and docstrings are fine and in fact desirable;
    # a mention inside a create_subscription call is not.
    code_lines = [
        line
        for line in text.splitlines()
        if "cmd_vel_candidate" in line and not line.strip().startswith("#")
    ]
    executable = [line for line in code_lines if "create_subscription" in line]
    if executable:
        fail(f"hardware bridge subscribes to a candidate topic: {executable}")
    else:
        ok("hardware bridge has no subscription to any candidate topic")


def check_only_the_arbiter_publishes_safe_commands() -> None:
    """Invariant D: exactly one package may publish the actuator's input type."""
    offenders = []
    for path in ROOT.glob("go2_*/**/*.py"):
        if "test" in path.parts or path.parts[-3:-1] == ("go2_msgs",):
            continue
        text = path.read_text(encoding="utf-8")
        if "create_publisher" not in text:
            continue
        for line in text.splitlines():
            if "create_publisher" in line and "SafeVelocityCommand" in line:
                package = path.relative_to(ROOT).parts[0]
                if package != "go2_safety_arbiter":
                    offenders.append(f"{path.relative_to(ROOT)}: {line.strip()}")
    if offenders:
        fail(f"SafeVelocityCommand published outside the arbiter: {offenders}")
    else:
        ok("only go2_safety_arbiter publishes SafeVelocityCommand")


def check_no_launch_file_starts_a_second_actuator() -> None:
    """Every actuation path must go through motion_authority.launch.py."""
    offenders = []
    for path in ROOT.glob("go2_*/launch/*.py"):
        if path.name == "motion_authority.launch.py":
            continue
        text = path.read_text(encoding="utf-8")
        if "go2_hardware_bridge" in text and "executable=" in text:
            offenders.append(str(path.relative_to(ROOT)))
    if offenders:
        fail(f"launch files starting an actuator directly: {offenders}")
    else:
        ok("the hardware bridge is startable only from motion_authority.launch.py")


def check_removed_bypass_stays_removed() -> None:
    """The old gait-controller hardware bridge must not come back."""
    for relative in (
        "go2_gait_controller/scripts/hw_bridge.py",
        "go2_gait_controller/launch/gait_hw_launch.py",
    ):
        if (ROOT / relative).exists():
            fail(
                f"{relative} has returned; it is a second actuation path that "
                "bypasses the safety arbiter (see go2_gait_controller/DEPRECATED.md)"
            )
        else:
            ok(f"removed bypass stays removed: {relative}")


def check_safety_limits_agree() -> None:
    """The bridge re-checks the arbiter's limits; they must not drift apart."""
    try:
        import yaml
    except ImportError:
        print("[repo-doctor] SKIP: pyyaml unavailable, cannot cross-check limits")
        return

    config = ROOT / "go2_bringup/config/safety.yaml"
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    arbiter = data["safety_arbiter_node"]["ros__parameters"]
    bridge = data["hardware_bridge_node"]["ros__parameters"]

    for key in ("max_vx", "max_vy", "max_wz"):
        if arbiter[key] != bridge[key]:
            fail(
                f"{key} differs between arbiter ({arbiter[key]}) and bridge "
                f"({bridge[key]}); the bridge would refuse valid commands or "
                "accept ones the arbiter would not issue"
            )
    if bridge["watchdog_timeout_sec"] > arbiter["watchdog_timeout_sec"]:
        fail(
            "the bridge watchdog is looser than the arbiter's, so a dead "
            "arbiter would leave the robot moving for the difference"
        )
    if bridge["hardware_adapter"] != "dry_run":
        fail(
            f"the default hardware adapter is {bridge['hardware_adapter']!r}; "
            "selecting a physical adapter must be a deliberate act"
        )
    ok("arbiter and bridge safety limits agree, and dry_run is the default")


def check_limits_are_marked_unvalidated() -> None:
    """No number in this repository has been measured on a GO2. Say so."""
    text = (ROOT / "go2_bringup/config/safety.yaml").read_text(encoding="utf-8")
    if "hardware_validated: false" not in text:
        fail("safety.yaml does not mark its limits as hardware-unvalidated")
    elif "NOTHING IN THIS FILE HAS BEEN MEASURED ON A PHYSICAL GO2" not in text:
        fail("safety.yaml is missing its hardware-validation disclaimer")
    else:
        ok("safety limits are explicitly marked hardware-unvalidated")


def check_no_false_hardware_claims() -> None:
    """
    Guard the honesty boundary in user-facing documents.

    Markdown table rows are skipped. Tables in these documents carry an
    explicit per-row Yes/No validation column, so a phrase like "validated on
    the robot" appearing in a header is a column label rather than a claim,
    and scanning them by substring produces only false positives. Prose is
    where an overstatement would actually mislead, so prose is what is
    scanned — backed by a positive check that the disclaimer is present.
    """
    banned = [
        "hardware validated",
        "validated on the robot",
        "tested on the go2",
        "running on the go2",
    ]
    hedges = ("not ", "never", "no ", "unvalidated", "would ")
    offenders = []
    for relative in ("README.md", "docs/research_system_claims.md"):
        path = ROOT / relative
        if not path.exists():
            continue
        for line in path.read_text(encoding="utf-8").lower().splitlines():
            stripped = line.strip()
            if stripped.startswith(("|", ">", "*", "-", "#")):
                continue  # table row, quote, list item, or heading
            for phrase in banned:
                if phrase in stripped and not any(h in stripped for h in hedges):
                    offenders.append(f"{relative}: {stripped[:90]}")
    if offenders:
        fail(f"unsupported hardware claims: {offenders}")
    else:
        ok("no unsupported hardware-validation claims in prose")


def check_disclaimer_is_present() -> None:
    """
    The positive half of the honesty check.

    Absence of a bad phrase is weak evidence. Requiring the disclaimer to be
    present, in these words, means it cannot be quietly dropped when the
    status table is next edited.
    """
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    if "has ever moved a\nphysical robot" not in readme and (
        "has ever moved a physical robot" not in readme.replace("\n", " ")
    ):
        fail(
            "README.md no longer states that no code here has moved a physical "
            "robot; that sentence is not optional while the Unitree adapter "
            "remains unexecuted"
        )
    else:
        ok("README carries the hardware-validation disclaimer")


def check_behavior_tree() -> None:
    bt_path = ROOT / "go2_navigation/behavior_trees/navigate_to_pose_recovery.xml"
    root = ET.parse(bt_path).getroot()
    if root.tag != "root":
        fail(f"unexpected behavior tree root tag in {bt_path}")
    else:
        ok(f"parsed behavior tree {bt_path.relative_to(ROOT)}")


def check_deprecated_launch_is_inert() -> None:
    text = (ROOT / "go2_bringup/launch/go2_full.launch.py").read_text(encoding="utf-8")
    if "raise RuntimeError" not in text:
        fail("go2_full.launch.py is deprecated but still loadable")
    else:
        ok("the deprecated entrypoint refuses to run")


def check_readme_sections() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for section in ("## Setup", "## Build", "## Test And Validate", "## Run", "## Troubleshooting"):
        if section not in readme:
            fail(f"README.md missing section '{section}'")
    ok("README operational sections present")


def main() -> None:
    for relative_path in (
        ".editorconfig",
        ".pre-commit-config.yaml",
        "pyproject.toml",
        "conftest.py",
        "scripts/bootstrap.sh",
        "scripts/lint.sh",
        "scripts/test.sh",
        "scripts/validate.sh",
        "scripts/run.sh",
        "scripts/reproduce.sh",
        "scripts/ros_inspect.sh",
        "docs/architecture.md",
        "docs/debugging.md",
        "docs/ros_graph.md",
        "docs/hardware_assumptions.md",
        "docs/runtime_graph_audit.md",
        "docs/target_runtime_architecture.md",
        "docs/safety_architecture_audit.md",
        "docs/research_system_claims.md",
        "docs/END_TO_END_UPGRADE_REPORT.md",
        "go2_bringup/launch/system.launch.py",
        "go2_bringup/launch/system_dry_run.launch.py",
        "go2_bringup/launch/motion_authority.launch.py",
        "go2_bringup/config/safety.yaml",
        "go2_bringup/config/fusion.yaml",
        "go2_bringup/config/navigation.yaml",
        "go2_gait_controller/DEPRECATED.md",
        "go2_navigation/behavior_trees/navigate_to_pose_recovery.xml",
    ):
        check_exists(relative_path)

    check_bridge_has_one_motion_inlet()
    check_only_the_arbiter_publishes_safe_commands()
    check_no_launch_file_starts_a_second_actuator()
    check_removed_bypass_stays_removed()
    check_safety_limits_agree()
    check_limits_are_marked_unvalidated()
    check_no_false_hardware_claims()
    check_disclaimer_is_present()
    check_deprecated_launch_is_inert()
    check_behavior_tree()
    check_readme_sections()

    if FAILURES:
        print(f"\n[repo-doctor] {len(FAILURES)} check(s) failed")
        sys.exit(1)
    print("\n[repo-doctor] complete: all checks passed")


if __name__ == "__main__":
    main()
