"""
Pure geometry for the staged approach controller.

No ROS, no TF, no I/O — just "given the goal expressed in the robot's own
frame, what body velocity moves toward it?".  Kept separate so the control
law is testable without a ROS graph and so it can be deleted wholesale when
Nav2 takes over without touching anything else.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


class ApproachStatus:
    """Outcome of one control step."""

    ACTIVE = "ACTIVE"
    #: Within the goal tolerance: hold still, goal achieved.
    REACHED = "REACHED"
    #: No goal is currently set.
    IDLE = "IDLE"
    #: A goal exists but its pose could not be expressed in the robot frame.
    NO_TRANSFORM = "NO_TRANSFORM"
    #: Goal is older than the configured lifetime.
    GOAL_STALE = "GOAL_STALE"


@dataclass(frozen=True)
class ApproachGains:
    """
    Tuning for the staged controller.

    These are *shaping* gains only.  They do not bound the robot: the
    SafetyArbiter owns the velocity envelope and will clamp anything produced
    here.  Setting them badly produces a sloppy approach, not an unsafe one.
    """

    k_linear: float = 0.6         # (m/s) per metre of range error
    k_angular: float = 1.2        # (rad/s) per radian of heading error
    #: Stop translating when the heading error exceeds this: turn in place
    #: first, so the robot does not arc into furniture it cannot see.
    turn_in_place_rad: float = 0.6
    #: Distance at which the approach is considered complete. A guide dog
    #: stopping on top of its handler is a failure mode, so this is not zero.
    goal_tolerance_m: float = 0.8
    #: Heading tolerance once the position goal is met.
    yaw_tolerance_rad: float = 0.25
    #: Begin derating linear speed inside this range.
    slowdown_radius_m: float = 1.5

    def __post_init__(self) -> None:
        for name in ("k_linear", "k_angular", "turn_in_place_rad",
                     "goal_tolerance_m", "yaw_tolerance_rad", "slowdown_radius_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0, got {value}")
        if self.slowdown_radius_m <= self.goal_tolerance_m:
            raise ValueError("slowdown_radius_m must exceed goal_tolerance_m")


@dataclass(frozen=True)
class ApproachCommand:
    vx: float
    vy: float
    wz: float
    status: str
    range_m: float
    heading_error_rad: float


def normalize_angle(angle: float) -> float:
    """Wrap to (-pi, pi]. Correct for any finite input, unlike a min() trick."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_approach(
    goal_x: float,
    goal_y: float,
    gains: ApproachGains,
) -> ApproachCommand:
    """
    Compute a body velocity toward a goal expressed in ``base_link``.

    Args:
        goal_x: Goal position ahead of the robot, metres (REP-103 +x forward).
        goal_y: Goal position to the robot's left, metres (REP-103 +y left).

    Returns:
        An :class:`ApproachCommand`. Lateral velocity is always zero: the
        controller drives the GO2 like a differential-drive base, which is the
        conservative choice and matches what Nav2's default controllers emit.
    """
    if not (math.isfinite(goal_x) and math.isfinite(goal_y)):
        return ApproachCommand(0.0, 0.0, 0.0, ApproachStatus.NO_TRANSFORM, float("nan"), float("nan"))

    range_m = math.hypot(goal_x, goal_y)
    heading = normalize_angle(math.atan2(goal_y, goal_x))

    if range_m <= gains.goal_tolerance_m:
        # Position reached. Square up to the caller so the robot ends facing
        # the person, which matters for a handover.
        if abs(heading) > gains.yaw_tolerance_rad and range_m > 1e-3:
            wz = gains.k_angular * heading
            return ApproachCommand(0.0, 0.0, wz, ApproachStatus.ACTIVE, range_m, heading)
        return ApproachCommand(0.0, 0.0, 0.0, ApproachStatus.REACHED, range_m, heading)

    wz = gains.k_angular * heading

    if abs(heading) > gains.turn_in_place_rad:
        # Too far off-heading to translate safely: rotate first.
        return ApproachCommand(0.0, 0.0, wz, ApproachStatus.ACTIVE, range_m, heading)

    effective_range = range_m - gains.goal_tolerance_m
    vx = gains.k_linear * effective_range
    if range_m < gains.slowdown_radius_m:
        span = gains.slowdown_radius_m - gains.goal_tolerance_m
        vx *= max(0.0, min(1.0, effective_range / span))
    # Derate translation as heading error grows, so the path is an arc that
    # tightens rather than a wide sweep.
    vx *= math.cos(heading)
    vx = max(0.0, vx)

    return ApproachCommand(vx, 0.0, wz, ApproachStatus.ACTIVE, range_m, heading)
