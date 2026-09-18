"""
Deterministic safety arbiter core.

This module contains the entire motion-authorization decision and has NO ROS
dependency.  It is a pure function of (candidate command, safety context,
current time, previous authorized command).  Given identical inputs it
produces an identical decision, which is what makes the behaviour testable
and auditable.

Design rules enforced here:

* **Fail closed.**  Every path that cannot positively establish permission
  returns zero velocity.  The default return value of every branch is stop.
  There is no ``else: allow``.
* **No unbounded carry-over.**  The previous authorized command is used only
  to enforce acceleration limits, never as a fallback output.  If nothing
  fresh arrives the output is zero, not the last value.
* **Exceptions are stops.**  ``evaluate`` catches any exception raised by a
  rule and converts it into an EMERGENCY-free but zero-velocity STOPPED
  decision with ``RULE_EVALUATION_FAILED``.  A crashing rule must never leave
  the robot moving.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar, List, Optional, Sequence, Tuple

from go2_safety_arbiter.limits import (
    RequirementPolicy,
    TimingPolicy,
    VelocityLimits,
)
from go2_safety_arbiter.reasons import Reason, SafetyState

#: Hazard alert types (from go2_msgs/SafetyAlert.alert_type) that require a
#: full stop, versus those that only require derating.
HAZARD_STOP_TYPES = frozenset({"EMERGENCY_STOP", "STAIRS_DETECTED", "DROP_DETECTED"})
HAZARD_SLOWDOWN_TYPES = frozenset({"SLOWDOWN", "NARROW_PASSAGE"})
HAZARD_CLEAR_TYPES = frozenset({"CLEAR", ""})
#: Directional hazards: "RESTRICT:" followed by one or more of F (no forward
#: vx), B (no backward vx), W (no rotation). Lateral vy is always zeroed under
#: a restriction. Any other suffix is treated as an unknown hazard (stop).
HAZARD_RESTRICT_PREFIX = "RESTRICT:"
HAZARD_RESTRICT_FLAGS = frozenset("FBW")


def parse_restriction(hazard: str) -> Optional[frozenset]:
    """Flags of a well-formed RESTRICT hazard, else None."""
    if not hazard.startswith(HAZARD_RESTRICT_PREFIX):
        return None
    flags = hazard[len(HAZARD_RESTRICT_PREFIX):]
    if not flags or set(flags) - HAZARD_RESTRICT_FLAGS:
        return None
    return frozenset(flags)


@dataclass(frozen=True)
class Velocity:
    """A body velocity in base_link (REP-103)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    #: Populated below the class body. Declared here only for readability;
    #: it is a ClassVar, not a dataclass field.
    ZERO: ClassVar["Velocity"]

    def is_finite(self) -> bool:
        return all(
            isinstance(v, (int, float)) and math.isfinite(v)
            for v in (self.vx, self.vy, self.wz)
        )

    def is_zero(self, eps: float = 1e-9) -> bool:
        return abs(self.vx) <= eps and abs(self.vy) <= eps and abs(self.wz) <= eps

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.vx, self.vy, self.wz)


Velocity.ZERO = Velocity(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Candidate:
    """
    A candidate motion command from a planner/controller.

    ``stamp`` is the time the command was generated, in the same monotonic
    time base the arbiter uses.  For unstamped sources (e.g. Nav2 Humble's
    ``geometry_msgs/Twist``) the ROS node fills this with the *reception*
    time and records that reduced assurance; the core does not care which,
    it only enforces the age policy.
    """

    velocity: Velocity
    stamp: float
    frame_id: str = "base_link"


@dataclass(frozen=True)
class SafetyContext:
    """
    Everything the arbiter knows about the world besides the candidate.

    Ages are absolute timestamps (same time base as ``now``); ``None`` means
    "never observed", which is treated as failing the corresponding
    requirement.
    """

    #: Most recent hazard alert type and when it was received.
    hazard_type: Optional[str] = None
    hazard_stamp: Optional[float] = None
    #: Whether localization is currently considered valid, and when that was
    #: last asserted.
    localization_ok: bool = False
    localization_stamp: Optional[float] = None
    #: Latched emergency stop.
    estop_engaged: bool = False
    #: Expected frame for candidate commands. A mismatch is a hard stop.
    expected_frame_id: str = "base_link"


@dataclass(frozen=True)
class Decision:
    """The arbiter's output. ``velocity`` is what may be sent to hardware."""

    velocity: Velocity
    state: str
    reason_codes: Tuple[str, ...] = ()
    #: True when ``velocity`` differs from the candidate (or no candidate
    #: existed and a stop was issued).
    intervened: bool = False
    #: Age of the candidate at evaluation time; -1.0 when there was none.
    candidate_age_sec: float = -1.0

    def is_stop(self) -> bool:
        return self.velocity.is_zero()


def _clamp(value: float, limit: float) -> Tuple[float, bool]:
    """Clamp ``value`` to [-limit, +limit]. Returns (clamped, was_clamped)."""
    if value > limit:
        return limit, True
    if value < -limit:
        return -limit, True
    return value, False


def _slew(target: float, previous: float, max_delta: float) -> Tuple[float, bool]:
    """Limit the change from ``previous`` to ``target`` to ``max_delta``."""
    delta = target - previous
    if delta > max_delta:
        return previous + max_delta, True
    if delta < -max_delta:
        return previous - max_delta, True
    return target, False


class SafetyArbiterCore:
    """
    Deterministic, fail-closed motion authorization.

    Usage::

        core = SafetyArbiterCore(limits, timing, requirements)
        decision = core.evaluate(candidate, context, now)

    ``evaluate`` must be called on a fixed control period even when no
    candidate has arrived, that is how the watchdog fires.  Pass
    ``candidate=None`` in that case.
    """

    def __init__(
        self,
        limits: VelocityLimits,
        timing: TimingPolicy,
        requirements: RequirementPolicy,
        control_period_sec: float,
    ) -> None:
        if not math.isfinite(control_period_sec) or control_period_sec <= 0.0:
            raise ValueError("control_period_sec must be finite and > 0")
        self._limits = limits
        self._timing = timing
        self._requirements = requirements
        self._period = control_period_sec

        # Mutable state. Deliberately small and fully inspectable.
        self._last_authorized = Velocity.ZERO
        self._decision_count = 0
        self._intervention_count = 0
        self._last_intervention_reasons: Tuple[str, ...] = ()
        self._seen_hazard = False
        self._seen_localization = False
        self._estop_latched = False

    # ── Introspection ──────────────────────────────────────────────────

    @property
    def last_authorized(self) -> Velocity:
        return self._last_authorized

    @property
    def decision_count(self) -> int:
        return self._decision_count

    @property
    def intervention_count(self) -> int:
        return self._intervention_count

    @property
    def last_intervention_reasons(self) -> Tuple[str, ...]:
        return self._last_intervention_reasons

    @property
    def estop_latched(self) -> bool:
        return self._estop_latched

    # ── Emergency stop ─────────────────────────────────────────────────

    def engage_estop(self) -> None:
        """Latch emergency stop. Takes effect on the next evaluation."""
        self._estop_latched = True

    def release_estop(self) -> None:
        """
        Release the latch.

        Release is an explicit, separate action: an e-stop never clears
        because the triggering condition went away.  The node exposes this
        only via a dedicated service, never via a topic, so a stray message
        cannot re-enable motion.
        """
        self._estop_latched = False

    # ── Main decision ──────────────────────────────────────────────────

    def evaluate(
        self,
        candidate: Optional[Candidate],
        context: SafetyContext,
        now: float,
    ) -> Decision:
        """Authorize, modify or suppress ``candidate``. Never raises."""
        try:
            decision = self._evaluate_inner(candidate, context, now)
        except Exception:  # noqa: BLE001, a failing rule must stop the robot
            decision = self._stop(
                SafetyState.STOPPED,
                [Reason.RULE_EVALUATION_FAILED],
                candidate_age_sec=-1.0,
            )
        self._commit(decision, candidate)
        return decision

    # ── Internals ──────────────────────────────────────────────────────

    def _stop(
        self,
        state: str,
        reasons: Sequence[str],
        candidate_age_sec: float,
    ) -> Decision:
        return Decision(
            velocity=Velocity.ZERO,
            state=state,
            reason_codes=tuple(reasons),
            intervened=True,
            candidate_age_sec=candidate_age_sec,
        )

    def _commit(self, decision: Decision, candidate: Optional[Candidate]) -> None:
        self._decision_count += 1
        # The acceleration limiter must track what was actually authorized,
        # including forced stops, so that recovery from a stop is also
        # slew-limited rather than a step change.
        self._last_authorized = decision.velocity
        if decision.intervened:
            self._intervention_count += 1
            if decision.reason_codes:
                self._last_intervention_reasons = decision.reason_codes

    def _evaluate_inner(
        self,
        candidate: Optional[Candidate],
        context: SafetyContext,
        now: float,
    ) -> Decision:
        # Emergency stop is latched in the core, and can additionally be
        # asserted by the context. Either latches.
        if context.estop_engaged:
            self._estop_latched = True
        if self._estop_latched:
            return self._stop(
                SafetyState.EMERGENCY_STOP,
                [Reason.EMERGENCY_STOP],
                candidate_age_sec=self._age(candidate, now),
            )

        # ── Watchdog / presence ────────────────────────────────────────
        if candidate is None:
            return self._stop(
                SafetyState.STOPPED,
                [Reason.WATCHDOG_TIMEOUT],
                candidate_age_sec=-1.0,
            )

        # ── Timestamp validity ─────────────────────────────────────────
        if not isinstance(candidate.stamp, (int, float)) or not math.isfinite(candidate.stamp):
            return self._stop(SafetyState.STOPPED, [Reason.INVALID_TIMESTAMP], -1.0)
        if candidate.stamp <= 0.0:
            return self._stop(SafetyState.STOPPED, [Reason.INVALID_TIMESTAMP], -1.0)

        age = now - candidate.stamp
        if age < -self._timing.future_tolerance_sec:
            # A command from the future means broken clock sync. Fail closed
            # rather than trusting it.
            return self._stop(SafetyState.STOPPED, [Reason.INVALID_TIMESTAMP], age)
        if age > self._timing.candidate_max_age_sec:
            return self._stop(SafetyState.STOPPED, [Reason.STALE_COMMAND], age)

        # ── Value validity ─────────────────────────────────────────────
        if not candidate.velocity.is_finite():
            return self._stop(SafetyState.STOPPED, [Reason.INVALID_COMMAND], age)

        # ── Frame validity ─────────────────────────────────────────────
        if context.expected_frame_id and candidate.frame_id != context.expected_frame_id:
            return self._stop(SafetyState.STOPPED, [Reason.INVALID_COMMAND], age)

        # ── Safety context requirements ────────────────────────────────
        reasons: List[str] = []
        degraded = False
        restriction: Optional[frozenset] = None

        if self._requirements.require_hazard_context:
            if context.hazard_stamp is None or context.hazard_type is None:
                return self._stop(SafetyState.STOPPED, [Reason.SAFETY_CONTEXT_STALE], age)
            self._seen_hazard = True
            hazard_age = now - context.hazard_stamp
            if hazard_age > self._timing.hazard_max_age_sec or hazard_age < -self._timing.future_tolerance_sec:
                return self._stop(SafetyState.STOPPED, [Reason.SAFETY_CONTEXT_STALE], age)
            hazard = (context.hazard_type or "").strip().upper()
            if hazard in HAZARD_STOP_TYPES:
                return self._stop(SafetyState.STOPPED, [Reason.HAZARD_STOP], age)
            restriction = parse_restriction(hazard)
            if restriction is not None:
                if restriction >= HAZARD_RESTRICT_FLAGS:
                    return self._stop(SafetyState.STOPPED, [Reason.HAZARD_STOP], age)
                degraded = True
                reasons.append(Reason.HAZARD_DIRECTIONAL)
            elif hazard in HAZARD_SLOWDOWN_TYPES:
                degraded = True
                reasons.append(Reason.HAZARD_SLOWDOWN)
            elif hazard not in HAZARD_CLEAR_TYPES:
                # An alert type the policy does not recognise is not a licence
                # to move. Unknown hazard => stop.
                return self._stop(SafetyState.STOPPED, [Reason.HAZARD_STOP], age)

        if self._requirements.require_localization:
            if context.localization_stamp is None or not context.localization_ok:
                return self._stop(SafetyState.STOPPED, [Reason.NO_LOCALIZATION], age)
            self._seen_localization = True
            loc_age = now - context.localization_stamp
            if loc_age > self._timing.localization_max_age_sec:
                return self._stop(SafetyState.STOPPED, [Reason.NO_LOCALIZATION], age)

        # ── Limits ─────────────────────────────────────────────────────
        limits = self._limits.scaled(self._limits.degraded_scale) if degraded else self._limits

        vx, cx = _clamp(candidate.velocity.vx, limits.max_vx)
        vy, cy = _clamp(candidate.velocity.vy, limits.max_vy)
        wz, cw = _clamp(candidate.velocity.wz, limits.max_wz)
        if cx or cy or cw:
            reasons.append(Reason.SPEED_LIMIT)

        max_dv = limits.max_accel_linear * self._period
        max_dw = limits.max_accel_angular * self._period
        prev = self._last_authorized
        vx, sx = _slew(vx, prev.vx, max_dv)
        vy, sy = _slew(vy, prev.vy, max_dv)
        wz, sw = _slew(wz, prev.wz, max_dw)
        if sx or sy or sw:
            reasons.append(Reason.ACCEL_LIMIT)

        # Directional restriction is applied AFTER slew limiting, so a
        # forbidden component drops to zero on this tick instead of ramping
        # down (ramping would keep driving into the obstacle for seconds).
        if restriction is not None:
            vy = 0.0
            if "F" in restriction:
                vx = min(vx, 0.0)
            if "B" in restriction:
                vx = max(vx, 0.0)
            if "W" in restriction:
                wz = 0.0

        authorized = Velocity(vx, vy, wz)
        state = SafetyState.DEGRADED if degraded else SafetyState.SAFE_TO_MOVE
        intervened = bool(reasons) or authorized.as_tuple() != candidate.velocity.as_tuple()

        return Decision(
            velocity=authorized,
            state=state,
            reason_codes=tuple(reasons),
            intervened=intervened,
            candidate_age_sec=age,
        )

    @staticmethod
    def _age(candidate: Optional[Candidate], now: float) -> float:
        if candidate is None:
            return -1.0
        try:
            age = now - candidate.stamp
        except TypeError:
            return -1.0
        return age if math.isfinite(age) else -1.0
