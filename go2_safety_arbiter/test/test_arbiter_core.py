"""
Deterministic safety-arbiter core tests.

These are the proofs that the fail-closed property holds. They are pure
(no ROS) so they run everywhere and so a failure points at the decision logic
rather than at middleware.
"""

import pytest
from go2_safety_arbiter.core import (
    Candidate,
    SafetyArbiterCore,
    SafetyContext,
    Velocity,
)
from go2_safety_arbiter.limits import (
    LimitConfigError,
    RequirementPolicy,
    TimingPolicy,
    VelocityLimits,
)
from go2_safety_arbiter.reasons import Reason, SafetyState

PERIOD = 0.05
T0 = 1000.0


def make_core(limits=None, timing=None, requirements=None, period=PERIOD):
    return SafetyArbiterCore(
        limits or VelocityLimits(),
        timing or TimingPolicy(),
        requirements or RequirementPolicy(),
        period,
    )


def clear_context(now=T0, **kwargs):
    base = dict(hazard_type="CLEAR", hazard_stamp=now)
    base.update(kwargs)
    return SafetyContext(**base)


def settle(core, context_factory, now=T0, velocity=None, ticks=60):
    """
    Run the core to steady state on a constant candidate.

    The acceleration limiter means a single evaluation never reaches the
    commanded velocity; tests that care about the clamp rather than the slew
    need to run the loop.
    """
    velocity = velocity or Velocity(0.3, 0.0, 0.0)
    decision = None
    t = now
    for _ in range(ticks):
        decision = core.evaluate(
            Candidate(velocity, t), context_factory(t), t
        )
        t += PERIOD
    return decision


class TestNominalPassThrough:
    def test_in_limit_command_is_authorized(self):
        core = make_core()
        decision = settle(core, lambda t: clear_context(t))
        assert decision.state == SafetyState.SAFE_TO_MOVE
        assert decision.velocity.vx == pytest.approx(0.3)
        assert not decision.is_stop()

    def test_authorized_velocity_never_exceeds_the_candidate(self):
        """The arbiter may reduce a command. It must never amplify one."""
        core = make_core()
        t = T0
        for _ in range(80):
            decision = core.evaluate(
                Candidate(Velocity(0.25, 0.0, 0.1), t), clear_context(t), t
            )
            assert abs(decision.velocity.vx) <= 0.25 + 1e-12
            assert abs(decision.velocity.wz) <= 0.1 + 1e-12
            t += PERIOD


class TestCommandValidity:
    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize("axis", ["vx", "vy", "wz"])
    def test_non_finite_component_is_refused(self, bad, axis):
        core = make_core()
        velocity = Velocity(**{axis: bad})
        decision = core.evaluate(Candidate(velocity, T0), clear_context(), T0)
        assert decision.is_stop()
        assert Reason.INVALID_COMMAND in decision.reason_codes

    @pytest.mark.parametrize("stamp", [0.0, -1.0, float("nan"), float("inf")])
    def test_invalid_timestamp_is_refused(self, stamp):
        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), stamp), clear_context(), T0
        )
        assert decision.is_stop()
        assert Reason.INVALID_TIMESTAMP in decision.reason_codes

    def test_command_from_the_future_is_refused(self):
        """Broken clock sync must fail closed rather than be trusted."""
        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0 + 5.0), clear_context(), T0
        )
        assert decision.is_stop()
        assert Reason.INVALID_TIMESTAMP in decision.reason_codes

    def test_small_future_skew_is_tolerated(self):
        """Sub-tick skew between processes must not stop the robot."""
        timing = TimingPolicy(future_tolerance_sec=0.05)
        core = make_core(timing=timing)
        decision = core.evaluate(
            Candidate(Velocity(0.01, 0, 0), T0 + 0.02), clear_context(), T0
        )
        assert decision.state == SafetyState.SAFE_TO_MOVE

    def test_stale_command_is_refused(self):
        core = make_core(timing=TimingPolicy(candidate_max_age_sec=0.3))
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0 - 0.5), clear_context(), T0
        )
        assert decision.is_stop()
        assert Reason.STALE_COMMAND in decision.reason_codes

    def test_wrong_frame_is_refused(self):
        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0, frame_id="odom"),
            clear_context(),
            T0,
        )
        assert decision.is_stop()
        assert Reason.INVALID_COMMAND in decision.reason_codes


class TestWatchdog:
    def test_absent_candidate_stops(self):
        core = make_core()
        decision = core.evaluate(None, clear_context(), T0)
        assert decision.is_stop()
        assert Reason.WATCHDOG_TIMEOUT in decision.reason_codes

    def test_last_command_is_never_carried_forward(self):
        """
        The single most important property in this file.

        After a good command, an absent one must produce zero — not a repeat
        of the previous value. "Continue last command" is how a robot keeps
        walking after its planner dies.
        """
        core = make_core()
        moving = settle(core, lambda t: clear_context(t))
        assert moving.velocity.vx > 0.2

        decision = core.evaluate(None, clear_context(), T0 + 10.0)
        assert decision.velocity.as_tuple() == (0.0, 0.0, 0.0)


class TestLimits:
    def test_over_limit_is_clamped_to_the_configured_maximum(self):
        core = make_core(limits=VelocityLimits(max_vx=0.4))
        decision = settle(core, lambda t: clear_context(t), velocity=Velocity(5.0, 0, 0))
        assert decision.velocity.vx == pytest.approx(0.4)
        assert decision.state == SafetyState.SAFE_TO_MOVE

    def test_clamp_is_symmetric_for_reverse(self):
        core = make_core(limits=VelocityLimits(max_vx=0.4))
        decision = settle(core, lambda t: clear_context(t), velocity=Velocity(-5.0, 0, 0))
        assert decision.velocity.vx == pytest.approx(-0.4)

    def test_clamping_reports_speed_limit(self):
        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(9.0, 0, 0), T0), clear_context(), T0
        )
        assert Reason.SPEED_LIMIT in decision.reason_codes
        assert decision.intervened

    def test_acceleration_is_slew_limited(self):
        """A step command must not become a step in authorized velocity."""
        core = make_core(limits=VelocityLimits(max_vx=0.4, max_accel_linear=0.5))
        first = core.evaluate(Candidate(Velocity(0.4, 0, 0), T0), clear_context(), T0)
        assert first.velocity.vx == pytest.approx(0.5 * PERIOD)
        assert Reason.ACCEL_LIMIT in first.reason_codes

    def test_slew_limit_applies_to_deceleration_too(self):
        core = make_core()
        settle(core, lambda t: clear_context(t))
        after = core.evaluate(
            Candidate(Velocity(0.0, 0, 0), T0 + 5.0), clear_context(T0 + 5.0), T0 + 5.0
        )
        assert after.velocity.vx > 0.0
        assert after.velocity.vx < 0.3

    def test_yaw_limit_is_enforced(self):
        core = make_core(limits=VelocityLimits(max_wz=0.6))
        decision = settle(
            core, lambda t: clear_context(t), velocity=Velocity(0, 0, 3.0)
        )
        assert decision.velocity.wz == pytest.approx(0.6)


class TestHazardPolicy:
    def test_missing_hazard_context_stops(self):
        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0), SafetyContext(), T0
        )
        assert decision.is_stop()
        assert Reason.SAFETY_CONTEXT_STALE in decision.reason_codes

    def test_stale_hazard_context_stops(self):
        core = make_core(timing=TimingPolicy(hazard_max_age_sec=1.0))
        context = SafetyContext(hazard_type="CLEAR", hazard_stamp=T0 - 5.0)
        decision = core.evaluate(Candidate(Velocity(0.3, 0, 0), T0), context, T0)
        assert decision.is_stop()
        assert Reason.SAFETY_CONTEXT_STALE in decision.reason_codes

    @pytest.mark.parametrize(
        "hazard", ["EMERGENCY_STOP", "STAIRS_DETECTED", "DROP_DETECTED"]
    )
    def test_stop_class_hazard_stops(self, hazard):
        core = make_core()
        decision = settle(
            core, lambda t: clear_context(t, hazard_type=hazard)
        )
        assert decision.is_stop()
        assert Reason.HAZARD_STOP in decision.reason_codes

    @pytest.mark.parametrize("hazard", ["SLOWDOWN", "NARROW_PASSAGE"])
    def test_slowdown_class_hazard_derates(self, hazard):
        core = make_core(limits=VelocityLimits(max_vx=0.4, degraded_scale=0.35))
        decision = settle(
            core, lambda t: clear_context(t, hazard_type=hazard),
            velocity=Velocity(0.4, 0, 0),
        )
        assert decision.state == SafetyState.DEGRADED
        assert decision.velocity.vx == pytest.approx(0.4 * 0.35)
        assert Reason.HAZARD_SLOWDOWN in decision.reason_codes

    def test_unknown_hazard_type_stops(self):
        """An alert the policy does not recognise is not permission to move."""
        core = make_core()
        decision = settle(
            core, lambda t: clear_context(t, hazard_type="SOMETHING_NEW")
        )
        assert decision.is_stop()
        assert Reason.HAZARD_STOP in decision.reason_codes

    def test_hazard_requirement_can_be_relaxed_but_limits_still_apply(self):
        """
        Relaxing a requirement must not disable the arbiter.

        There is no parameter combination that turns the arbiter into a
        pass-through; this proves the most permissive one still clamps.
        """
        core = make_core(
            requirements=RequirementPolicy(require_hazard_context=False),
            limits=VelocityLimits(max_vx=0.4),
        )
        decision = settle(
            core, lambda t: SafetyContext(), velocity=Velocity(9.0, 0, 0)
        )
        assert decision.velocity.vx == pytest.approx(0.4)


class TestLocalizationPolicy:
    def test_missing_localization_stops_when_required(self):
        core = make_core(requirements=RequirementPolicy(require_localization=True))
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0), clear_context(), T0
        )
        assert decision.is_stop()
        assert Reason.NO_LOCALIZATION in decision.reason_codes

    def test_stale_localization_stops_when_required(self):
        core = make_core(
            requirements=RequirementPolicy(require_localization=True),
            timing=TimingPolicy(localization_max_age_sec=2.0),
        )
        context = clear_context(
            localization_ok=True, localization_stamp=T0 - 10.0
        )
        decision = core.evaluate(Candidate(Velocity(0.3, 0, 0), T0), context, T0)
        assert decision.is_stop()
        assert Reason.NO_LOCALIZATION in decision.reason_codes

    def test_fresh_localization_permits_motion(self):
        core = make_core(requirements=RequirementPolicy(require_localization=True))
        decision = settle(
            core,
            lambda t: clear_context(t, localization_ok=True, localization_stamp=t),
        )
        assert decision.state == SafetyState.SAFE_TO_MOVE
        assert decision.velocity.vx > 0.0


class TestEmergencyStop:
    def test_estop_latches_and_zeroes(self):
        core = make_core()
        settle(core, lambda t: clear_context(t))
        core.engage_estop()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0), clear_context(), T0
        )
        assert decision.state == SafetyState.EMERGENCY_STOP
        assert decision.is_stop()

    def test_estop_does_not_clear_when_the_cause_clears(self):
        """An e-stop that self-releases is not an e-stop."""
        core = make_core()
        core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0),
            clear_context(estop_engaged=True),
            T0,
        )
        for i in range(50):
            t = T0 + i * PERIOD
            decision = core.evaluate(
                Candidate(Velocity(0.3, 0, 0), t), clear_context(t), t
            )
            assert decision.state == SafetyState.EMERGENCY_STOP
            assert decision.is_stop()

    def test_explicit_release_restores_motion(self):
        core = make_core()
        core.engage_estop()
        assert core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0), clear_context(), T0
        ).is_stop()
        core.release_estop()
        decision = settle(core, lambda t: clear_context(t), now=T0 + 1.0)
        assert decision.state == SafetyState.SAFE_TO_MOVE
        assert decision.velocity.vx > 0.0

    def test_recovery_after_release_is_still_slew_limited(self):
        """Releasing an e-stop must not produce a lurch."""
        core = make_core()
        core.engage_estop()
        core.evaluate(Candidate(Velocity(0.4, 0, 0), T0), clear_context(), T0)
        core.release_estop()
        first = core.evaluate(
            Candidate(Velocity(0.4, 0, 0), T0 + PERIOD),
            clear_context(T0 + PERIOD),
            T0 + PERIOD,
        )
        assert first.velocity.vx == pytest.approx(0.5 * PERIOD)


class TestFailClosedUnderFaults:
    def test_a_raising_rule_produces_a_stop_not_an_exception(self):
        """
        A safety rule that throws must stop the robot.

        Simulated by handing the core a context whose attribute access raises,
        which is the closest analogue to a rule bug that a pure test can build.
        """
        class ExplodingContext:
            expected_frame_id = "base_link"
            estop_engaged = False

            def __getattr__(self, name):
                raise RuntimeError(f"rule blew up reading {name}")

        core = make_core()
        decision = core.evaluate(
            Candidate(Velocity(0.3, 0, 0), T0), ExplodingContext(), T0
        )
        assert decision.is_stop()
        assert Reason.RULE_EVALUATION_FAILED in decision.reason_codes

    def test_every_stopping_state_authorizes_exactly_zero(self):
        """
        Exhaustive sweep: no combination of adverse inputs may produce motion.

        This is the property the whole architecture exists to guarantee, so it
        is asserted over a grid rather than at a handful of points.
        """
        core = make_core()
        bad_velocities = [
            Velocity(float("nan"), 0, 0),
            Velocity(0, float("inf"), 0),
            Velocity(0, 0, float("-inf")),
        ]
        bad_stamps = [0.0, -1.0, T0 - 99.0, T0 + 99.0]
        bad_contexts = [
            SafetyContext(),
            SafetyContext(hazard_type="EMERGENCY_STOP", hazard_stamp=T0),
            SafetyContext(hazard_type="CLEAR", hazard_stamp=T0 - 99.0),
            SafetyContext(hazard_type="UNRECOGNISED", hazard_stamp=T0),
        ]
        for velocity in bad_velocities:
            for context in bad_contexts:
                decision = core.evaluate(Candidate(velocity, T0), context, T0)
                assert decision.velocity.as_tuple() == (0.0, 0.0, 0.0)
        for stamp in bad_stamps:
            for context in bad_contexts:
                decision = core.evaluate(
                    Candidate(Velocity(0.3, 0, 0), stamp), context, T0
                )
                assert decision.velocity.as_tuple() == (0.0, 0.0, 0.0)


class TestConfigValidation:
    @pytest.mark.parametrize(
        "kwargs",
        [
            {"max_vx": 0.0},
            {"max_vx": -1.0},
            {"max_vx": float("nan")},
            {"max_wz": float("inf")},
            {"max_accel_linear": 0.0},
            {"degraded_scale": 0.0},
            {"degraded_scale": 1.5},
        ],
    )
    def test_unusable_limits_are_rejected(self, kwargs):
        """A bad safety config must fail loudly, never be silently corrected."""
        with pytest.raises(LimitConfigError):
            VelocityLimits(**kwargs)

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"candidate_max_age_sec": 0.0},
            {"watchdog_timeout_sec": -1.0},
            {"hazard_max_age_sec": float("nan")},
            {"future_tolerance_sec": -0.1},
        ],
    )
    def test_unusable_timing_is_rejected(self, kwargs):
        with pytest.raises(LimitConfigError):
            TimingPolicy(**kwargs)

    def test_zero_control_period_is_rejected(self):
        with pytest.raises(ValueError):
            SafetyArbiterCore(
                VelocityLimits(), TimingPolicy(), RequirementPolicy(), 0.0
            )


class TestBookkeeping:
    def test_interventions_are_counted_and_attributed(self):
        core = make_core()
        core.evaluate(Candidate(Velocity(9.0, 0, 0), T0), clear_context(), T0)
        assert core.intervention_count >= 1
        assert Reason.SPEED_LIMIT in core.last_intervention_reasons

    def test_decision_count_tracks_every_evaluation(self):
        core = make_core()
        for i in range(7):
            core.evaluate(None, clear_context(), T0 + i)
        assert core.decision_count == 7


class TestDirectionalHazards:
    """RESTRICT:<F|B|W> forbids only the motion that would hit the obstacle.

    A total stop for "obstacle ahead" deadlocked the robot in closed-loop sim:
    it could not turn or back away from the furniture that stopped it.
    """

    def _after(self, hazard, velocity):
        core = make_core()
        settle(core, lambda t: clear_context(t), velocity=velocity)  # moving already
        t = T0 + 60 * PERIOD
        return core.evaluate(Candidate(velocity, t), clear_context(t, hazard_type=hazard), t)

    def test_forward_blocked_zeroes_forward_on_the_same_tick(self):
        decision = self._after("RESTRICT:F", Velocity(0.3, 0.0, 0.0))
        assert decision.velocity.vx == 0.0  # no slew ramp-down into the obstacle
        assert Reason.HAZARD_DIRECTIONAL in decision.reason_codes
        assert decision.state == SafetyState.DEGRADED

    def test_forward_blocked_permits_rotation_and_reverse_derated(self):
        core = make_core()
        d = settle(core, lambda t: clear_context(t, hazard_type="RESTRICT:F"),
                   velocity=Velocity(-0.3, 0.0, 0.5))
        assert d.velocity.vx < 0.0
        assert d.velocity.wz > 0.0
        scaled = VelocityLimits().scaled(VelocityLimits().degraded_scale)
        assert abs(d.velocity.vx) <= scaled.max_vx + 1e-12
        assert abs(d.velocity.wz) <= scaled.max_wz + 1e-12

    def test_backward_blocked_forbids_only_reverse(self):
        core = make_core()
        d = settle(core, lambda t: clear_context(t, hazard_type="RESTRICT:B"),
                   velocity=Velocity(-0.3, 0.0, 0.0))
        assert d.velocity.vx == 0.0
        d = settle(core, lambda t: clear_context(t, hazard_type="RESTRICT:B"),
                   velocity=Velocity(0.3, 0.0, 0.0))
        assert d.velocity.vx > 0.0

    def test_rotation_blocked_zeroes_yaw_and_lateral(self):
        decision = self._after("RESTRICT:W", Velocity(0.2, 0.1, 0.4))
        assert decision.velocity.wz == 0.0
        assert decision.velocity.vy == 0.0
        assert decision.velocity.vx > 0.0

    def test_every_direction_blocked_is_a_stop(self):
        decision = self._after("RESTRICT:FBW", Velocity(0.2, 0.0, 0.2))
        assert decision.is_stop()
        assert Reason.HAZARD_STOP in decision.reason_codes

    @pytest.mark.parametrize("bad", ["RESTRICT:", "RESTRICT:X", "RESTRICT:FZ", "RESTRICT: F"])
    def test_malformed_restriction_is_an_unknown_hazard_and_stops(self, bad):
        # A turning candidate discriminates: any valid F restriction would let
        # the rotation through; a stop does not.
        decision = self._after(bad, Velocity(0.2, 0.0, 0.3))
        assert decision.state == SafetyState.STOPPED
        assert decision.velocity.is_zero()
