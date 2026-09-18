"""Pure-function tests for the kinematic GO2 simulator core."""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_sim.sim_core import (  # noqa: E402
    API_ID_DAMP,
    API_ID_MOVE,
    API_ID_STOP_MOVE,
    BASE_HEIGHT_M,
    DEFAULT_CLOCK_SKEW_SEC,
    Box,
    LidarExtrinsic,
    RobotState,
    World,
    beam_directions,
    footprint_collides,
    lag_velocity,
    load_world,
    parse_sport_request,
    raycast,
    rot_rpy,
    simulate_scan,
    skewed_stamp_ns,
    step_kinematics,
)

WORLD_FILE = Path(__file__).resolve().parents[1] / "worlds" / "apartment.yaml"


class TestKinematics:
    def test_lag_converges_to_command(self):
        v = 0.0
        for _ in range(200):
            v = lag_velocity(v, 0.4, 0.01, 0.15)
        assert v == pytest.approx(0.4, abs=1e-4)

    def test_lag_at_one_time_constant_is_63_percent(self):
        v = lag_velocity(0.0, 1.0, 0.15, 0.15)
        assert v == pytest.approx(1.0 - math.exp(-1.0))

    def test_straight_line_distance_accounts_for_lag(self):
        state = RobotState()
        for _ in range(200):  # 2 s at 100 Hz
            state = step_kinematics(state, (0.5, 0.0, 0.0), 0.01, 0.15)
        # Distance of a first-order lag: v * (T - tau * (1 - exp(-T/tau))).
        expected = 0.5 * (2.0 - 0.15 * (1.0 - math.exp(-2.0 / 0.15)))
        assert state.x == pytest.approx(expected, abs=5e-3)
        assert state.y == pytest.approx(0.0, abs=1e-9)

    def test_body_frame_velocity_follows_heading(self):
        state = RobotState(yaw=math.pi / 2)
        for _ in range(100):
            state = step_kinematics(state, (0.5, 0.0, 0.0), 0.01, 0.0)
        assert state.x == pytest.approx(0.0, abs=1e-6)
        assert state.y == pytest.approx(0.5, abs=1e-6)

    def test_lateral_is_rep103_left(self):
        state = RobotState()
        for _ in range(100):
            state = step_kinematics(state, (0.0, 0.2, 0.0), 0.01, 0.0)
        assert state.y == pytest.approx(0.2, abs=1e-6)

    def test_turning_integrates_yaw_ccw_positive(self):
        state = RobotState()
        for _ in range(100):
            state = step_kinematics(state, (0.0, 0.0, 0.5), 0.01, 0.0)
        assert state.yaw == pytest.approx(0.5, abs=1e-9)


class TestCollision:
    WALL = Box(x=2.0, y=0.0, yaw=0.0, size_x=0.1, size_y=4.0, z_min=0.0, z_max=1.0)

    def test_clear_footprint(self):
        assert footprint_collides(0.0, 0.0, 0.0, 0.70, 0.31, [self.WALL]) is None

    def test_footprint_touching_wall(self):
        # Front edge at 1.6 + 0.35 = 1.95, wall face at 1.95.
        assert footprint_collides(1.61, 0.0, 0.0, 0.70, 0.31, [self.WALL]) is self.WALL

    def test_rotated_footprint_uses_true_extent(self):
        # Sideways the robot is only 0.155 m deep: fits where it would not head-on.
        assert footprint_collides(1.7, 0.0, math.pi / 2, 0.70, 0.31, [self.WALL]) is None
        assert footprint_collides(1.7, 0.0, 0.0, 0.70, 0.31, [self.WALL]) is self.WALL

    def test_overhead_box_does_not_collide(self):
        tabletop = Box(0.0, 0.0, 0.0, 1.0, 1.0, z_min=0.7, z_max=0.75)
        assert footprint_collides(0.0, 0.0, 0.0, 0.70, 0.31, [tabletop]) is None


class TestRaycast:
    def test_floor_distance_from_known_height(self):
        origin = np.array([0.0, 0.0, 0.5])
        t = raycast(origin, np.array([[0.0, 0.0, -1.0]]), [], 10.0)
        assert t[0] == pytest.approx(0.5)

    def test_box_face_distance(self):
        box = Box(3.0, 0.0, 0.0, 1.0, 1.0, 0.0, 2.0)
        t = raycast(np.array([0.0, 0.0, 0.5]), np.array([[1.0, 0.0, 0.0]]), [box], 10.0)
        assert t[0] == pytest.approx(2.5)

    def test_rotated_box_face_distance(self):
        # Box rotated 45 deg: the ray meets a corner at 3 - sqrt(2)/2.
        box = Box(3.0, 0.0, math.pi / 4, 1.0, 1.0, 0.0, 2.0)
        t = raycast(np.array([0.0, 0.0, 0.5]), np.array([[1.0, 0.0, 0.0]]), [box], 10.0)
        assert t[0] == pytest.approx(3.0 - math.sqrt(2) / 2, abs=1e-9)

    def test_ray_over_short_box_misses(self):
        box = Box(3.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.3)
        t = raycast(np.array([0.0, 0.0, 0.5]), np.array([[1.0, 0.0, 0.0]]), [box], 10.0)
        assert np.isinf(t[0])

    def test_nearest_surface_wins_and_range_limits(self):
        near = Box(2.0, 0.0, 0.0, 0.2, 1.0, 0.0, 2.0)
        far = Box(5.0, 0.0, 0.0, 0.2, 1.0, 0.0, 2.0)
        dirs = np.array([[1.0, 0.0, 0.0]])
        origin = np.array([0.0, 0.0, 0.5])
        assert raycast(origin, dirs, [far, near], 10.0)[0] == pytest.approx(1.9)
        assert np.isinf(raycast(origin, dirs, [far], 4.0)[0])


class TestLidar:
    def test_urdf_extrinsic_points_sensor_z_down_and_forward(self):
        z_axis = LidarExtrinsic().rotation() @ np.array([0.0, 0.0, 1.0])
        assert z_axis[2] < -0.9
        assert z_axis[0] > 0.2

    def test_floor_appears_below_robot_in_odom(self):
        world = World()
        _, pts_odom = simulate_scan(
            RobotState(), world, LidarExtrinsic(), beam_directions(), noise_std=0.0
        )
        assert len(pts_odom) > 100
        assert np.allclose(pts_odom[:, 2], 0.0, atol=1e-4)

    def test_sensor_and_odom_clouds_are_the_same_points(self):
        state = RobotState(x=1.0, y=2.0, yaw=0.7)
        ext = LidarExtrinsic()
        pts_s, pts_o = simulate_scan(state, World(), ext, beam_directions(), noise_std=0.0)
        r_ob = rot_rpy(0, 0, state.yaw)
        p_ol = np.array([1.0, 2.0, BASE_HEIGHT_M]) + r_ob @ np.array(ext.xyz)
        rebuilt = pts_s @ (r_ob @ ext.rotation()).T + p_ol
        assert np.allclose(rebuilt, pts_o, atol=1e-4)

    def test_wall_ahead_is_seen_at_the_right_place(self):
        wall = Box(3.0, 0.0, 0.0, 0.1, 6.0, 0.0, 1.2)
        _, pts = simulate_scan(
            RobotState(), World(obstacles=[wall]), LidarExtrinsic(), beam_directions(),
            noise_std=0.0,
        )
        on_wall = pts[pts[:, 2] > 0.05]
        assert len(on_wall) > 10
        assert np.allclose(on_wall[:, 0], 2.95, atol=1e-3)

    def test_apartment_scan_is_fast_enough(self):
        world = load_world(WORLD_FILE)
        dirs = beam_directions()
        rng = np.random.default_rng(0)
        state = RobotState(world.start_x, world.start_y, world.start_yaw)
        simulate_scan(state, world, LidarExtrinsic(), dirs, rng=rng)  # warm up
        t0 = time.perf_counter()
        for _ in range(20):
            simulate_scan(state, world, LidarExtrinsic(), dirs, rng=rng)
        per_scan_ms = (time.perf_counter() - t0) / 20 * 1e3
        # Budget is 20 ms at 15 Hz; assert loosely so a loaded CI box does not flake.
        assert per_scan_ms < 60.0


class TestSportRequests:
    def test_move_parses_body_velocity(self):
        cmd = parse_sport_request(API_ID_MOVE, '{"x": 0.3, "y": -0.1, "z": 0.2}')
        assert cmd.velocity == (0.3, -0.1, 0.2) and not cmd.error

    def test_stop_and_damp_are_zero(self):
        assert parse_sport_request(API_ID_STOP_MOVE, "").velocity == (0.0, 0.0, 0.0)
        assert parse_sport_request(API_ID_DAMP, "").velocity == (0.0, 0.0, 0.0)

    @pytest.mark.parametrize("param", ["", "{}", '{"x": 1}', "not json", '{"x": "nan", "y": 0, "z": 0}'])
    def test_malformed_move_is_rejected(self, param):
        cmd = parse_sport_request(API_ID_MOVE, param)
        assert cmd.velocity is None and cmd.error

    def test_unknown_api_is_rejected(self):
        assert parse_sport_request(1016, "").error


class TestClockSkewAndWorld:
    def test_skew_matches_measured_robot_offset(self):
        now_ns = 1_788_291_312_981_572_224  # 2026-09-01, receipt time of the real bag
        stamp_s = skewed_stamp_ns(now_ns, DEFAULT_CLOCK_SKEW_SEC) / 1e9
        # The real robot's stamps read 2025-10-17.
        assert time.gmtime(stamp_s)[:3] == (2025, 10, 17)

    def test_apartment_world_loads_with_all_labels(self):
        world = load_world(WORLD_FILE)
        labels = {o.label for o in world.objects}
        assert {"chair", "couch", "table", "refrigerator", "door", "person"} <= labels
        start = footprint_collides(
            world.start_x, world.start_y, world.start_yaw, 0.70, 0.31, world.boxes
        )
        assert start is None, "robot starts inside an obstacle"

    def test_doorway_is_passable(self):
        world = load_world(WORLD_FILE)
        assert footprint_collides(8.0, 3.0, 0.0, 0.70, 0.31, world.boxes) is None
