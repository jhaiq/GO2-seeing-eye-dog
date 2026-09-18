"""
Caller-confirmation state machine, with no ROS dependency.

Pulled out of the node so the interaction can be tested exhaustively —
including the failure modes that a blind user experiences as "the robot did
nothing and never said why".

States
------
IDLE       No request. Detections are ignored.
LISTENING  A request arrived; waiting for the first detection frame.
SEARCHING  Request active, detections arriving, none acceptable yet.
CANDIDATE  An acceptable candidate is accumulating confirmation frames.
CONFIRMED  Lock achieved; a goal has been (or is about to be) emitted.
TIMED_OUT  The request expired without a lock. Terminal until re-requested.
STOPPED    The user said stop. Terminal until re-requested.

Two rules drive the whole design:

1. **A goal requires a request.**  The previous implementation would lock a
   target and publish a navigation goal after five good detection frames with
   no voice command ever received; the voice callback only *reset* state, it
   never *armed* anything.  Here, ``IDLE`` ignores detections outright.

2. **Every path terminates with a reason.**  ``SEARCHING`` cannot persist
   indefinitely: ``request_timeout_sec`` moves it to ``TIMED_OUT``, and every
   transition carries a machine-readable reason so a feedback layer can speak
   it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


class GroundingState:
    IDLE = "IDLE"
    LISTENING = "LISTENING"
    SEARCHING = "SEARCHING"
    CANDIDATE = "CANDIDATE"
    CONFIRMED = "CONFIRMED"
    TIMED_OUT = "TIMED_OUT"
    STOPPED = "STOPPED"

    ALL = (IDLE, LISTENING, SEARCHING, CANDIDATE, CONFIRMED, TIMED_OUT, STOPPED)
    #: States in which a request is active and the clock is running.
    ACTIVE = (LISTENING, SEARCHING, CANDIDATE)


class GroundingReason:
    NO_REQUEST = "NO_REQUEST"
    NO_DETECTIONS = "NO_DETECTIONS"
    BEARING_MISMATCH = "BEARING_MISMATCH"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    ACCUMULATING = "ACCUMULATING"
    CONFIRMED = "CONFIRMED"
    CONFIRMATION_TIMEOUT = "CONFIRMATION_TIMEOUT"
    TARGET_MOVED = "TARGET_MOVED"
    USER_STOP = "USER_STOP"
    AWAITING_DETECTIONS = "AWAITING_DETECTIONS"


@dataclass
class GroundingSnapshot:
    """What the node publishes on every tick, regardless of input activity."""

    state: str = GroundingState.IDLE
    reason: str = GroundingReason.NO_REQUEST
    consecutive_confirmations: int = 0
    required_confirmations: int = 5
    request_time_remaining_sec: float = -1.0
    #: True exactly on the tick a lock is achieved, so the caller knows to
    #: emit a goal once and only once.
    just_confirmed: bool = False


class GroundingStateMachine:
    """
    Deterministic caller-confirmation logic.

    Feed it :meth:`on_request`, :meth:`on_stop` and :meth:`on_detection_frame`,
    and call :meth:`tick` on a fixed period so timeouts fire even when no
    detections are arriving at all — which is precisely the case the old
    implementation could not report.
    """

    def __init__(
        self,
        required_confirmations: int = 5,
        request_timeout_sec: float = 15.0,
    ) -> None:
        if required_confirmations < 1:
            raise ValueError("required_confirmations must be >= 1")
        if request_timeout_sec <= 0.0:
            raise ValueError("request_timeout_sec must be > 0")
        self._required = int(required_confirmations)
        self._timeout = float(request_timeout_sec)

        self._state = GroundingState.IDLE
        self._reason = GroundingReason.NO_REQUEST
        self._count = 0
        self._request_time: Optional[float] = None
        self._just_confirmed = False

    # ── Introspection ─────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def count(self) -> int:
        return self._count

    # ── Events ────────────────────────────────────────────────────────

    def on_request(self, now: float) -> None:
        """A caller asked the robot to come. Arms the search and starts the clock."""
        self._state = GroundingState.LISTENING
        self._reason = GroundingReason.AWAITING_DETECTIONS
        self._count = 0
        self._request_time = now
        self._just_confirmed = False

    def on_stop(self) -> None:
        """The user said stop/wait/stay. Terminal until a new request."""
        self._state = GroundingState.STOPPED
        self._reason = GroundingReason.USER_STOP
        self._count = 0
        self._request_time = None
        self._just_confirmed = False

    def on_target_moved(self, now: float) -> None:
        """
        A locked target left the association gate.

        Rather than silently continuing to a stale goal, the machine drops
        back to SEARCHING and restarts the request clock, so the interaction
        either re-acquires or times out with a reason. The user is told the
        difference.
        """
        if self._state != GroundingState.CONFIRMED:
            return
        self._state = GroundingState.SEARCHING
        self._reason = GroundingReason.TARGET_MOVED
        self._count = 0
        self._request_time = now
        self._just_confirmed = False

    def on_detection_frame(
        self,
        now: float,
        num_detections: int,
        best_accepted: bool,
        best_reason: str,
    ) -> None:
        """
        Process one frame of detections.

        Args:
            best_accepted: Whether the best candidate cleared fusion.
            best_reason: Fusion reason for the best candidate (used verbatim
                when nothing was accepted, so the user hears the real cause).

        Note that this method never clears ``_just_confirmed``. The lock edge
        is a latch owned by :meth:`tick`, which is the only consumer. Clearing
        it here would lose the edge whenever another detection frame arrived
        between the confirming frame and the next timer tick — which, with
        detections at camera rate and status at 5 Hz, is almost always.
        """
        if self._state not in GroundingState.ACTIVE:
            # IDLE / CONFIRMED / TIMED_OUT / STOPPED: detections do not arm
            # anything. This is the fix for "goal published with no request".
            return

        if self._check_timeout(now):
            return

        if num_detections <= 0:
            self._state = GroundingState.SEARCHING
            self._reason = GroundingReason.NO_DETECTIONS
            self._count = 0
            return

        if not best_accepted:
            self._state = GroundingState.SEARCHING
            self._reason = best_reason or GroundingReason.LOW_CONFIDENCE
            self._count = 0
            return

        self._count += 1
        if self._count >= self._required:
            self._state = GroundingState.CONFIRMED
            self._reason = GroundingReason.CONFIRMED
            self._just_confirmed = True
            self._request_time = None
        else:
            self._state = GroundingState.CANDIDATE
            self._reason = GroundingReason.ACCUMULATING

    def tick(self, now: float) -> GroundingSnapshot:
        """
        Advance time-based transitions and return the current snapshot.

        Must be called on a fixed period. This is where a request with no
        detections at all eventually times out — the previous implementation
        had no timer and so could sit in SEARCHING forever.
        """
        self._check_timeout(now)
        snapshot = GroundingSnapshot(
            state=self._state,
            reason=self._reason,
            consecutive_confirmations=self._count,
            required_confirmations=self._required,
            request_time_remaining_sec=self._time_remaining(now),
            just_confirmed=self._just_confirmed,
        )
        # just_confirmed is a one-shot edge: consume it here so a goal is
        # emitted exactly once per lock.
        self._just_confirmed = False
        return snapshot

    # ── Internals ─────────────────────────────────────────────────────

    def _time_remaining(self, now: float) -> float:
        if self._request_time is None or self._state not in GroundingState.ACTIVE:
            return -1.0
        return max(0.0, self._timeout - (now - self._request_time))

    def _check_timeout(self, now: float) -> bool:
        if self._state not in GroundingState.ACTIVE or self._request_time is None:
            return False
        if (now - self._request_time) < self._timeout:
            return False
        self._state = GroundingState.TIMED_OUT
        self._reason = GroundingReason.CONFIRMATION_TIMEOUT
        self._count = 0
        self._request_time = None
        self._just_confirmed = False
        return True
