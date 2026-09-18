"""
Launch-graph wiring tests.

These inspect the launch description WITHOUT starting any process, so they
catch a mis-wiring at review time rather than on a robot. They are the static
half of required Test 10; the dynamic half lives in
``go2_hardware_bridge/test/test_bridge_node.py``.

The property under test is the one the whole architecture rests on:

    the hardware bridge's only motion input is the safe topic, and the only
    publisher of that topic is the arbiter.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import ROS_AVAILABLE  # noqa: E402

pytestmark = pytest.mark.skipif(
    not ROS_AVAILABLE, reason="launch inspection requires the ROS launch packages"
)

LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def load_module(path: Path):
    import importlib.util

    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def motion_authority():
    return load_module(LAUNCH_DIR / "motion_authority.launch.py")


def literal(value):
    """
    Flatten a launch substitution (or tuple of them) into a plain string.

    ``launch_ros`` normalises parameter dicts and remappings into tuples of
    Substitution objects, so a test that reads them has to undo that to
    compare against literals.
    """
    if isinstance(value, (list, tuple)):
        return "".join(literal(part) for part in value)
    if hasattr(value, "text"):
        return value.text
    return str(value)


def parameter_dict(node_action):
    """
    Resolve a launch Node's inline parameter overrides into plain values.

    ``launch_ros`` normalises each override value into a YAML document (so
    that types survive the trip to the node), which is why the value is
    parsed rather than compared as a string.
    """
    import yaml

    resolved = {}
    for entry in node_action._Node__parameters or []:
        if not isinstance(entry, dict):
            continue  # a ParameterFile (config path), not an inline override
        for key, value in entry.items():
            try:
                parsed = yaml.safe_load(literal(value))
            except yaml.YAMLError:
                parsed = literal(value)
            resolved[literal(key)] = parsed
    return resolved


def remap_dict(node_action):
    """Resolve a launch Node's remappings into a plain dict of literals."""
    return {
        literal(source): literal(target)
        for source, target in (node_action._Node__remappings or [])
    }


class TestMotionAuthorityWiring:
    def test_it_starts_exactly_the_arbiter_and_the_bridge(self, motion_authority):
        nodes = motion_authority.get_motion_authority_nodes()
        packages = [literal(n._Node__package) for n in nodes]
        assert packages == ["go2_safety_arbiter", "go2_hardware_bridge"]

    def test_the_candidate_and_safe_topics_are_different(self, motion_authority):
        """
        Invariant A, checked as a literal string comparison.

        If these ever became the same topic, a controller's raw output would
        be what the hardware consumes.
        """
        assert motion_authority.CANDIDATE_TOPIC != motion_authority.SAFE_TOPIC
        assert motion_authority.CANDIDATE_UNSTAMPED_TOPIC != motion_authority.SAFE_TOPIC

    def test_the_bridge_never_names_the_candidate_topic(self, motion_authority):
        """
        Invariant A, checked against the actual launch wiring.

        The bridge's remappings must contain no route, under any name, to the
        candidate stream.
        """
        _arbiter, bridge = motion_authority.get_motion_authority_nodes()
        remaps = remap_dict(bridge)

        assert motion_authority.CANDIDATE_TOPIC not in remaps.values()
        assert motion_authority.CANDIDATE_UNSTAMPED_TOPIC not in remaps.values()
        assert not any("candidate" in key for key in remaps)
        assert not any("candidate" in value for value in remaps.values())

    def test_the_bridge_consumes_the_safe_topic(self, motion_authority):
        _arbiter, bridge = motion_authority.get_motion_authority_nodes()
        remaps = remap_dict(bridge)
        assert remaps.get("cmd_vel_safe") == motion_authority.SAFE_TOPIC

    def test_the_arbiter_is_the_only_producer_of_the_safe_topic(
        self, motion_authority
    ):
        """Invariant D: motion authority is mechanically identifiable."""
        arbiter, bridge = motion_authority.get_motion_authority_nodes()
        arbiter_remaps = remap_dict(arbiter)
        bridge_remaps = remap_dict(bridge)

        assert arbiter_remaps.get("cmd_vel_safe") == motion_authority.SAFE_TOPIC
        assert arbiter_remaps.get("cmd_vel_candidate") == motion_authority.CANDIDATE_TOPIC
        # The bridge names the safe topic only as an input.
        assert bridge_remaps.get("cmd_vel_safe") == motion_authority.SAFE_TOPIC

    def test_the_dry_run_adapter_is_the_default(self, motion_authority):
        """
        Selecting a physical adapter must be a deliberate act.

        A default that touched hardware would mean an incomplete command line
        could move a real robot.
        """
        _arbiter, bridge = motion_authority.get_motion_authority_nodes()
        assert parameter_dict(bridge).get("hardware_adapter") == "dry_run"


class TestSystemLaunch:
    def test_the_canonical_entrypoint_exists_and_loads(self):
        module = load_module(LAUNCH_DIR / "system.launch.py")
        assert hasattr(module, "generate_launch_description")

    def test_the_dry_run_entrypoint_exists_and_loads(self):
        module = load_module(LAUNCH_DIR / "system_dry_run.launch.py")
        assert hasattr(module, "generate_launch_description")

    def test_system_launch_builds_a_description(self):
        module = load_module(LAUNCH_DIR / "system.launch.py")
        description = module.generate_launch_description()
        assert description.entities

    def test_system_launch_includes_the_motion_authority_graph(self):
        """
        Every entrypoint must route actuation through the one place that
        defines it, rather than assembling its own bridge wiring.
        """
        from launch.actions import IncludeLaunchDescription

        module = load_module(LAUNCH_DIR / "system.launch.py")
        description = module.generate_launch_description()
        includes = [
            e for e in description.entities if isinstance(e, IncludeLaunchDescription)
        ]
        sources = " ".join(str(i.launch_description_source.location) for i in includes)
        assert "motion_authority" in sources

    def test_no_launch_file_starts_a_bridge_outside_motion_authority(self):
        """
        The bridge is startable from exactly one launch file.

        This is what stops a future launch file from quietly standing up a
        second actuation path.
        """
        offenders = []
        for path in LAUNCH_DIR.glob("*.py"):
            if path.name == "motion_authority.launch.py":
                continue
            text = path.read_text()
            if "go2_hardware_bridge" in text and "executable=" in text:
                offenders.append(path.name)
        assert offenders == [], f"launch files starting a bridge directly: {offenders}"


def _started_nodes(planner):
    """(package, executable, remappings) of every Node the planner starts."""
    from launch import LaunchContext
    from launch_ros.actions import Node

    module = load_module(LAUNCH_DIR / "system.launch.py")
    context = LaunchContext()
    context.launch_configurations.update(
        {"planner": planner, "perception": "none", "log_level": "info",
         "use_sim_time": "false", "hardware_adapter": "dry_run", "dry_run_log_path": ""}
    )
    started = []
    for entity in module.generate_launch_description().entities:
        if not isinstance(entity, Node):
            continue
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        remaps = {literal(a): literal(b) for a, b in (entity._Node__remappings or [])}
        started.append((entity._Node__package, entity._Node__node_executable, remaps))
    return started


class TestPlannerSelection:
    @pytest.mark.parametrize(
        "planner,expected",
        [
            ("staged", {"approach_controller_node"}),
            ("staged_nav", {"approach_controller_node", "nav_to_pose_adapter_node"}),
            ("nav2", set()),
        ],
    )
    def test_each_planner_starts_the_right_controllers(self, planner, expected):
        executables = {
            literal(exe) for pkg, exe, _ in _started_nodes(planner)
            if literal(pkg) == "go2_approach_controller"
        }
        assert executables == expected

    def test_the_adapter_serves_the_nav2_action_and_drives_the_controller(self):
        adapter = [
            remaps for pkg, exe, remaps in _started_nodes("staged_nav")
            if literal(exe) == "nav_to_pose_adapter_node"
        ]
        assert len(adapter) == 1
        remaps = adapter[0]
        assert remaps["navigate_to_pose"] == "/navigate_to_pose"
        assert remaps["goal_pose"] == "/goal_pose"
        assert remaps["cancel_goal"] == "/go2/cancel_goal"
        assert remaps["controller/status"] == "/go2/controller/status"

    @pytest.mark.parametrize("planner", ["staged", "staged_nav", "nav2"])
    def test_no_planner_node_touches_the_safe_topic(self, planner):
        for _pkg, exe, remaps in _started_nodes(planner):
            assert not any("cmd_vel_safe" in v for v in remaps.values()), literal(exe)


class TestDeprecatedLaunchIsInert:
    def test_the_old_entrypoint_refuses_to_run(self):
        """
        The previous main launch file started a Nav2 stack that could not
        complete lifecycle bring-up and connected nothing to actuation.
        Leaving it silently loadable would invite someone to use it.
        """
        module = load_module(LAUNCH_DIR / "go2_full.launch.py")
        with pytest.raises(RuntimeError) as excinfo:
            module.generate_launch_description()
        assert "deprecated" in str(excinfo.value).lower()
        assert "system.launch.py" in str(excinfo.value)


class TestConfigurationIsVersioned:
    CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

    def test_the_safety_config_exists_and_parses(self):
        import yaml

        data = yaml.safe_load((self.CONFIG_DIR / "safety.yaml").read_text())
        assert "safety_arbiter_node" in data
        assert "hardware_bridge_node" in data

    def test_arbiter_and_bridge_limits_agree(self):
        """
        The bridge re-checks the arbiter's limits. If the two disagree, either
        the bridge refuses valid commands or it accepts ones the arbiter would
        not have issued. Both are faults; catch them here.
        """
        import yaml

        data = yaml.safe_load((self.CONFIG_DIR / "safety.yaml").read_text())
        arbiter = data["safety_arbiter_node"]["ros__parameters"]
        bridge = data["hardware_bridge_node"]["ros__parameters"]
        for key in ("max_vx", "max_vy", "max_wz"):
            assert arbiter[key] == bridge[key], f"{key} differs between arbiter and bridge"

    def test_the_bridge_watchdog_is_tighter_than_the_arbiters(self):
        """
        The bridge must stop first. If it waited longer than the arbiter, a
        dead arbiter would leave the robot moving for the difference.
        """
        import yaml

        data = yaml.safe_load((self.CONFIG_DIR / "safety.yaml").read_text())
        arbiter = data["safety_arbiter_node"]["ros__parameters"]
        bridge = data["hardware_bridge_node"]["ros__parameters"]
        assert bridge["watchdog_timeout_sec"] <= arbiter["watchdog_timeout_sec"]

    def test_every_velocity_limit_is_marked_unvalidated(self):
        """
        No limit in this repository has been measured on a GO2. The config
        must say so, next to each number, so a reader cannot mistake a
        conservative guess for a measurement.
        """
        text = (self.CONFIG_DIR / "safety.yaml").read_text()
        assert "hardware_validated: false" in text
        assert "NOTHING IN THIS FILE HAS BEEN MEASURED ON A PHYSICAL GO2" in text

    def test_the_fusion_config_documents_its_derivation(self):
        text = (self.CONFIG_DIR / "fusion.yaml").read_text()
        assert "DERIVED, not tuned" in text
        for key in (
            "bearing_gate_deg",
            "audio_absent_factor",
            "min_confidence_threshold",
            "request_timeout_sec",
        ):
            assert key in text

    def test_fusion_config_matches_the_code_defaults(self):
        """Config drift is a silent way to change safety-relevant behaviour."""
        import yaml
        from go2_intent_grounding.fusion import FusionParams

        data = yaml.safe_load((self.CONFIG_DIR / "fusion.yaml").read_text())
        params = data["intent_grounding_node"]["ros__parameters"]
        defaults = FusionParams()

        assert params["bearing_gate_deg"] == defaults.bearing_gate_deg
        assert params["audio_absent_factor"] == defaults.audio_absent_factor
        assert params["min_confidence_threshold"] == defaults.min_confidence
        assert params["audio_weight"] == defaults.audio_weight
        assert params["visual_weight"] == defaults.visual_weight
