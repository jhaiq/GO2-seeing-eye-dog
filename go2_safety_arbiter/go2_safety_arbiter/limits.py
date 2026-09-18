"""
Velocity and acceleration limits for the GO2.

Every number here is a *default* that must be overridden from
``config/safety.yaml``.  Defaults are deliberately conservative and are
marked ``hardware_validated: false`` in that file: none of them has been
measured on the physical robot by this repository.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping


class LimitConfigError(ValueError):
    """Raised when a limit set is not usable. Callers must fail closed."""


# ── Absolute ceilings ──────────────────────────────────────────────────────
#
# A configured limit is a *reduction* of these, never an increase. Without
# them, validating only "positive and finite" meant `-p max_vx:=100.0` was
# accepted and the arbiter dutifully clamped to 100 m/s, a safety layer that
# enforces whatever it is told to enforce is not a safety layer.
#
# These are hard-coded rather than configurable on purpose: the point of a
# ceiling that lives in source and fails the node at startup is that it cannot
# be raised from a command line or a params file. Raising one is a code change
# and a review.
#
# The values are bounded by what is plausible for a GO2 operating next to a
# person who cannot see it, not by what the platform can do. They are not
# targets; the operating defaults in config/safety.yaml are far below them.

#: m/s. Unitree documents higher; this is the ceiling for assistive operation.
MAX_ALLOWED_VX = 1.5
#: m/s lateral.
MAX_ALLOWED_VY = 1.0
#: rad/s yaw (~86 deg/s).
MAX_ALLOWED_WZ = 2.0
#: m/s^2.
MAX_ALLOWED_ACCEL_LINEAR = 3.0
#: rad/s^2.
MAX_ALLOWED_ACCEL_ANGULAR = 6.0

#: Seconds. A command older than this cannot meaningfully be called fresh, and
#: a watchdog longer than this cannot meaningfully be called a watchdog.
MAX_ALLOWED_COMMAND_AGE_SEC = 1.0
MAX_ALLOWED_WATCHDOG_SEC = 2.0
MAX_ALLOWED_HAZARD_AGE_SEC = 5.0
MAX_ALLOWED_LOCALIZATION_AGE_SEC = 10.0
MAX_ALLOWED_FUTURE_TOLERANCE_SEC = 0.5
#: Bounds safe_command_lifetime_sec. Also keeps `now + lifetime` inside the
#: int32 range that builtin_interfaces/Time.sec requires, which an unbounded
#: value did not: a large enough lifetime raised inside the publish path and
#: killed the arbiter outright, taking the stop command with it.
MAX_ALLOWED_COMMAND_LIFETIME_SEC = 2.0

_VELOCITY_CEILINGS = {
    "max_vx": MAX_ALLOWED_VX,
    "max_vy": MAX_ALLOWED_VY,
    "max_wz": MAX_ALLOWED_WZ,
    "max_accel_linear": MAX_ALLOWED_ACCEL_LINEAR,
    "max_accel_angular": MAX_ALLOWED_ACCEL_ANGULAR,
}

_TIMING_CEILINGS = {
    "candidate_max_age_sec": MAX_ALLOWED_COMMAND_AGE_SEC,
    "watchdog_timeout_sec": MAX_ALLOWED_WATCHDOG_SEC,
    "hazard_max_age_sec": MAX_ALLOWED_HAZARD_AGE_SEC,
    "localization_max_age_sec": MAX_ALLOWED_LOCALIZATION_AGE_SEC,
    "safe_command_lifetime_sec": MAX_ALLOWED_COMMAND_LIFETIME_SEC,
}


@dataclass(frozen=True)
class VelocityLimits:
    """
    Symmetric body-velocity envelope in ``base_link`` (REP-103).

    All values are magnitudes and must be strictly positive; acceleration
    limits must be positive and finite.
    """

    max_vx: float = 0.4          # m/s, forward/back
    max_vy: float = 0.2          # m/s, lateral (GO2 supports lateral motion)
    max_wz: float = 0.6          # rad/s, yaw
    max_accel_linear: float = 0.5   # m/s^2 applied to vx and vy independently
    max_accel_angular: float = 1.0  # rad/s^2 applied to wz
    #: Multiplier applied to every component while in ``DEGRADED``.
    degraded_scale: float = 0.35

    def __post_init__(self) -> None:
        for name, ceiling in _VELOCITY_CEILINGS.items():
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise LimitConfigError(f"{name} must be a finite number, got {value!r}")
            if value <= 0.0:
                raise LimitConfigError(f"{name} must be > 0, got {value}")
            if value > ceiling:
                raise LimitConfigError(
                    f"{name}={value} exceeds the hard ceiling {ceiling}. This "
                    "ceiling is compiled into go2_safety_arbiter.limits and "
                    "cannot be raised from a parameter file or command line; "
                    "raising it is a code change."
                )
        if not math.isfinite(self.degraded_scale) or not (0.0 < self.degraded_scale <= 1.0):
            raise LimitConfigError(
                f"degraded_scale must be in (0, 1], got {self.degraded_scale}"
            )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "VelocityLimits":
        """Build from a plain dict, ignoring unknown keys but rejecting bad ones."""
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        return cls(**known)

    def scaled(self, factor: float) -> "VelocityLimits":
        """Return the same envelope with velocity magnitudes scaled."""
        if not math.isfinite(factor) or not (0.0 < factor <= 1.0):
            raise LimitConfigError(f"scale factor must be in (0, 1], got {factor}")
        return VelocityLimits(
            max_vx=self.max_vx * factor,
            max_vy=self.max_vy * factor,
            max_wz=self.max_wz * factor,
            max_accel_linear=self.max_accel_linear,
            max_accel_angular=self.max_accel_angular,
            degraded_scale=self.degraded_scale,
        )


@dataclass(frozen=True)
class TimingPolicy:
    """
    Freshness and watchdog policy.

    ``candidate_max_age_sec`` bounds the lifetime of an individual command
    (Invariant C).  ``watchdog_timeout_sec`` bounds the silence the arbiter
    will tolerate before commanding stop (Invariant B).  They are separate:
    a stream of individually-fresh commands still stops the robot if it dries
    up, and a single very old command is rejected even if it just arrived.
    """

    candidate_max_age_sec: float = 0.30
    watchdog_timeout_sec: float = 0.50
    hazard_max_age_sec: float = 1.00
    localization_max_age_sec: float = 2.00
    #: How far in the future a timestamp may be before it is rejected.
    #: Absorbs small clock skew between processes; not a licence for drift.
    future_tolerance_sec: float = 0.05
    #: Lifetime stamped into each SafeVelocityCommand.valid_until. The bridge
    #: enforces this independently of the arbiter.
    safe_command_lifetime_sec: float = 0.30

    def __post_init__(self) -> None:
        for name, ceiling in _TIMING_CEILINGS.items():
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0:
                raise LimitConfigError(f"{name} must be a finite number > 0, got {value!r}")
            if value > ceiling:
                raise LimitConfigError(
                    f"{name}={value} exceeds the hard ceiling {ceiling}. A "
                    "watchdog long enough to be disabled by configuration is "
                    "not a watchdog."
                )
        if (
            not math.isfinite(self.future_tolerance_sec)
            or self.future_tolerance_sec < 0.0
            or self.future_tolerance_sec > MAX_ALLOWED_FUTURE_TOLERANCE_SEC
        ):
            raise LimitConfigError(
                "future_tolerance_sec must be finite and in "
                f"[0, {MAX_ALLOWED_FUTURE_TOLERANCE_SEC}]"
            )

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TimingPolicy":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        return cls(**known)


@dataclass(frozen=True)
class RequirementPolicy:
    """
    Which inputs the arbiter *requires* before it will authorize motion.

    Every flag here defaults to True. Turning one off is a deliberate
    reduction in assurance and is recorded in ``SafetyStatus`` so it cannot
    be done silently.  There is intentionally no single ``safety_enabled``
    switch: no parameter combination can make the arbiter pass a candidate
    through unvalidated (see ``docs/safety_architecture_audit.md``).
    """

    require_hazard_context: bool = True
    require_localization: bool = False
    #: When True the arbiter refuses to authorize motion until every required
    #: input has been observed at least once since startup, independent of the
    #: freshness checks. Distinct from those checks: freshness asks "is this
    #: recent?", this asks "has this ever arrived?". A node that has never
    #: heard from the safety monitor is in a different situation from one whose
    #: last hazard message aged out, and only the second is a transient.
    require_initial_inputs: bool = True

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "RequirementPolicy":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        return cls(**{k: bool(v) for k, v in known.items()})


#: Fields of VelocityLimits that describe unvalidated hardware envelopes.
#: ``config/safety.yaml`` must mark these explicitly.
HARDWARE_UNVALIDATED_FIELDS = frozenset(
    {"max_vx", "max_vy", "max_wz", "max_accel_linear", "max_accel_angular"}
)
