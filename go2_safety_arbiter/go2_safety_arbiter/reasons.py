"""
Machine-readable safety states and reason codes.

These strings are a public contract: they appear in ``go2_msgs/SafetyStatus``
and in ``go2_msgs/SafeVelocityCommand.reason_codes``, and downstream
diagnostics, logs and tests match on them. Do not rename them casually.
"""
from __future__ import annotations


class SafetyState:
    """States the arbiter can occupy. Ordered from most to least permissive."""

    #: Full authority granted; the candidate may pass subject to limits.
    SAFE_TO_MOVE = "SAFE_TO_MOVE"
    #: Motion permitted but derated, because a non-fatal condition holds.
    DEGRADED = "DEGRADED"
    #: Motion forbidden. Recoverable automatically once the cause clears.
    STOPPED = "STOPPED"
    #: Motion forbidden and latched. Requires an explicit release.
    EMERGENCY_STOP = "EMERGENCY_STOP"

    ALL = (SAFE_TO_MOVE, DEGRADED, STOPPED, EMERGENCY_STOP)

    #: States in which the authorized velocity is guaranteed to be zero.
    ZERO_STATES = (STOPPED, EMERGENCY_STOP)


class Reason:
    """Reason codes explaining why a candidate was modified or suppressed."""

    # ── Command validity ───────────────────────────────────────────────
    #: Candidate contained NaN or infinity in a velocity component.
    INVALID_COMMAND = "INVALID_COMMAND"
    #: Candidate carried a timestamp that is zero, negative or in the future
    #: beyond tolerance.
    INVALID_TIMESTAMP = "INVALID_TIMESTAMP"
    #: Candidate is older than ``candidate_max_age_sec``.
    STALE_COMMAND = "STALE_COMMAND"
    #: No candidate has arrived within ``watchdog_timeout_sec``.
    WATCHDOG_TIMEOUT = "WATCHDOG_TIMEOUT"
    #: No candidate has ever been received since arbiter start.
    NO_COMMAND = "NO_COMMAND"

    # ── Limits ─────────────────────────────────────────────────────────
    #: One or more velocity components exceeded a configured limit and was
    #: clamped.
    SPEED_LIMIT = "SPEED_LIMIT"
    #: The commanded change per control period exceeded an acceleration
    #: limit and was slew-rate limited.
    ACCEL_LIMIT = "ACCEL_LIMIT"

    # ── Safety context ─────────────────────────────────────────────────
    #: Hazard information required by policy is missing or older than
    #: ``hazard_max_age_sec``.
    SAFETY_CONTEXT_STALE = "SAFETY_CONTEXT_STALE"
    #: A hazard requiring a full stop is currently asserted.
    HAZARD_STOP = "HAZARD_STOP"
    #: A hazard requiring derated motion is currently asserted.
    HAZARD_SLOWDOWN = "HAZARD_SLOWDOWN"
    #: Localization required by policy is missing or stale.
    NO_LOCALIZATION = "NO_LOCALIZATION"
    #: Emergency stop is engaged.
    EMERGENCY_STOP = "EMERGENCY_STOP"
    #: The arbiter has not finished initialising; it fails closed until every
    #: required input has been seen at least once.
    NOT_INITIALIZED = "NOT_INITIALIZED"
    #: A safety rule raised an unexpected exception. Fail closed.
    RULE_EVALUATION_FAILED = "RULE_EVALUATION_FAILED"

    # ── Bridge-side (independent of the arbiter) ───────────────────────
    #: The bridge received a command whose ``valid_until`` has passed.
    COMMAND_EXPIRED = "COMMAND_EXPIRED"
    #: The bridge received a command carrying an authority token that does
    #: not match the one it latched.
    AUTHORITY_MISMATCH = "AUTHORITY_MISMATCH"
    #: The bridge saw a sequence number it had already accepted, or one that
    #: went backwards without a token change.
    SEQUENCE_REGRESSION = "SEQUENCE_REGRESSION"
    #: The bridge's own watchdog expired.
    BRIDGE_WATCHDOG_TIMEOUT = "BRIDGE_WATCHDOG_TIMEOUT"
    #: The adapter reported a failed transmission.
    TRANSMIT_FAILED = "TRANSMIT_FAILED"
    #: The bridge is shutting down.
    SHUTDOWN = "SHUTDOWN"
