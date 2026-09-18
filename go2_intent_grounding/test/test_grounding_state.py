"""
Caller-confirmation state-machine tests.

The interaction failures these cover are the ones a blind user actually
experiences: a robot that sets off without being asked, and a robot that is
asked and then silently does nothing forever.
"""
import pytest
from go2_intent_grounding.grounding_state import (
    GroundingReason,
    GroundingState,
    GroundingStateMachine,
)


def machine(required=3, timeout=10.0):
    return GroundingStateMachine(
        required_confirmations=required, request_timeout_sec=timeout
    )


def feed(m, t0, n, accepted=True, reason="ACCEPTED", dt=0.1):
    """Feed n detection frames, returning the final snapshot."""
    snapshot = None
    for i in range(n):
        t = t0 + i * dt
        m.on_detection_frame(t, 1, accepted, reason)
        snapshot = m.tick(t)
    return snapshot


class TestRequestIsRequired:
    def test_starts_idle(self):
        m = machine()
        assert m.state == GroundingState.IDLE
        assert m.reason == GroundingReason.NO_REQUEST

    def test_detections_alone_never_confirm(self):
        """
        The defect: a goal used to be publishable with no voice command ever
        received, because the voice callback only reset state and never armed
        anything.
        """
        m = machine(required=3)
        snapshot = feed(m, 0.0, 50)
        assert m.state == GroundingState.IDLE
        assert m.count == 0
        assert not snapshot.just_confirmed

    def test_a_request_arms_the_search(self):
        m = machine()
        m.on_request(0.0)
        assert m.state == GroundingState.LISTENING
        assert m.reason == GroundingReason.AWAITING_DETECTIONS

    def test_request_then_detections_confirms(self):
        m = machine(required=3)
        m.on_request(0.0)
        snapshot = feed(m, 0.1, 3)
        assert m.state == GroundingState.CONFIRMED
        assert snapshot.just_confirmed

    def test_confirmation_edge_fires_exactly_once(self):
        """A single lock must produce a single goal, not one per frame."""
        m = machine(required=2)
        m.on_request(0.0)
        edges = 0
        for i in range(20):
            t = 0.1 + i * 0.1
            m.on_detection_frame(t, 1, True, "ACCEPTED")
            if m.tick(t).just_confirmed:
                edges += 1
        assert edges == 1

    def test_detections_after_confirmation_do_not_re_lock(self):
        m = machine(required=2)
        m.on_request(0.0)
        feed(m, 0.1, 2)
        assert m.state == GroundingState.CONFIRMED
        feed(m, 1.0, 10)
        assert m.state == GroundingState.CONFIRMED


class TestAccumulation:
    def test_partial_accumulation_reports_candidate(self):
        m = machine(required=5)
        m.on_request(0.0)
        snapshot = feed(m, 0.1, 3)
        assert snapshot.state == GroundingState.CANDIDATE
        assert snapshot.reason == GroundingReason.ACCUMULATING
        assert snapshot.consecutive_confirmations == 3
        assert snapshot.required_confirmations == 5

    def test_a_rejected_frame_resets_the_count(self):
        """Confirmation must be CONSECUTIVE, not cumulative."""
        m = machine(required=5)
        m.on_request(0.0)
        feed(m, 0.1, 4)
        assert m.count == 4
        m.on_detection_frame(1.0, 1, False, GroundingReason.LOW_CONFIDENCE)
        assert m.count == 0
        assert m.state == GroundingState.SEARCHING

    def test_rejection_reason_is_reported_verbatim(self):
        """
        The user needs to hear WHY, not just that nothing happened.
        "I can see you but I heard you somewhere else" is actionable;
        "searching" is not.
        """
        m = machine()
        m.on_request(0.0)
        m.on_detection_frame(0.1, 2, False, GroundingReason.BEARING_MISMATCH)
        snapshot = m.tick(0.1)
        assert snapshot.reason == GroundingReason.BEARING_MISMATCH

    def test_no_detections_is_distinguished_from_bad_detections(self):
        m = machine()
        m.on_request(0.0)
        m.on_detection_frame(0.1, 0, False, "")
        assert m.tick(0.1).reason == GroundingReason.NO_DETECTIONS


class TestTimeout:
    def test_a_request_with_no_detections_times_out(self):
        """
        The silent-deadlock defect: SEARCHING used to persist indefinitely
        with no output, because nothing ran on a timer.
        """
        m = machine(timeout=5.0)
        m.on_request(0.0)
        assert m.tick(1.0).state == GroundingState.LISTENING
        snapshot = m.tick(6.0)
        assert snapshot.state == GroundingState.TIMED_OUT
        assert snapshot.reason == GroundingReason.CONFIRMATION_TIMEOUT

    def test_timeout_fires_without_any_detection_frames_at_all(self):
        """The timer, not the data flow, is what guarantees an outcome."""
        m = machine(timeout=3.0)
        m.on_request(0.0)
        for t in (0.5, 1.0, 1.5, 2.0, 2.5):
            assert m.tick(t).state in GroundingState.ACTIVE
        assert m.tick(3.1).state == GroundingState.TIMED_OUT

    def test_time_remaining_is_reported_while_active(self):
        m = machine(timeout=10.0)
        m.on_request(0.0)
        assert m.tick(4.0).request_time_remaining_sec == pytest.approx(6.0)

    def test_time_remaining_is_negative_when_inactive(self):
        m = machine()
        assert m.tick(0.0).request_time_remaining_sec == -1.0

    def test_timeout_is_terminal_until_a_new_request(self):
        m = machine(required=2, timeout=3.0)
        m.on_request(0.0)
        m.tick(5.0)
        assert m.state == GroundingState.TIMED_OUT
        feed(m, 6.0, 10)
        assert m.state == GroundingState.TIMED_OUT

        m.on_request(20.0)
        feed(m, 20.1, 2)
        assert m.state == GroundingState.CONFIRMED

    def test_a_partially_accumulated_request_still_times_out(self):
        m = machine(required=10, timeout=2.0)
        m.on_request(0.0)
        feed(m, 0.1, 3)
        assert m.state == GroundingState.CANDIDATE
        assert m.tick(3.0).state == GroundingState.TIMED_OUT


class TestUserStop:
    def test_stop_is_terminal_and_reported(self):
        m = machine(required=2)
        m.on_request(0.0)
        feed(m, 0.1, 1)
        m.on_stop()
        assert m.state == GroundingState.STOPPED
        assert m.reason == GroundingReason.USER_STOP

    def test_detections_after_stop_do_not_confirm(self):
        m = machine(required=2)
        m.on_request(0.0)
        m.on_stop()
        feed(m, 1.0, 20)
        assert m.state == GroundingState.STOPPED

    def test_a_new_request_clears_a_stop(self):
        m = machine(required=2)
        m.on_stop()
        m.on_request(5.0)
        feed(m, 5.1, 2)
        assert m.state == GroundingState.CONFIRMED


class TestTargetMoved:
    def test_a_locked_target_leaving_the_gate_triggers_reacquisition(self):
        """
        Rather than silently pursuing a stale goal, the interaction reports
        TARGET_MOVED and either re-acquires or times out with a reason.
        """
        m = machine(required=2, timeout=10.0)
        m.on_request(0.0)
        feed(m, 0.1, 2)
        assert m.state == GroundingState.CONFIRMED

        m.on_target_moved(1.0)
        assert m.state == GroundingState.SEARCHING
        assert m.reason == GroundingReason.TARGET_MOVED

    def test_reacquisition_restarts_the_clock(self):
        m = machine(required=2, timeout=5.0)
        m.on_request(0.0)
        feed(m, 0.1, 2)
        m.on_target_moved(4.0)
        assert m.tick(6.0).state == GroundingState.SEARCHING
        assert m.tick(9.1).state == GroundingState.TIMED_OUT

    def test_target_moved_is_ignored_when_not_confirmed(self):
        m = machine()
        m.on_request(0.0)
        m.on_target_moved(1.0)
        assert m.state == GroundingState.LISTENING


class TestAlwaysReportsSomething:
    def test_every_reachable_state_has_a_reason(self):
        """
        The blind-user contract: there is no state in which the system is
        doing something and cannot say what.
        """
        scenarios = []

        m = machine()
        scenarios.append(m.tick(0.0))

        m = machine()
        m.on_request(0.0)
        scenarios.append(m.tick(0.1))

        m = machine(required=5)
        m.on_request(0.0)
        m.on_detection_frame(0.1, 1, False, GroundingReason.BEARING_MISMATCH)
        scenarios.append(m.tick(0.1))

        m = machine(required=5)
        m.on_request(0.0)
        m.on_detection_frame(0.1, 1, True, "ACCEPTED")
        scenarios.append(m.tick(0.1))

        m = machine(required=1)
        m.on_request(0.0)
        m.on_detection_frame(0.1, 1, True, "ACCEPTED")
        scenarios.append(m.tick(0.1))

        m = machine(timeout=1.0)
        m.on_request(0.0)
        scenarios.append(m.tick(2.0))

        m = machine()
        m.on_stop()
        scenarios.append(m.tick(0.0))

        seen_states = {s.state for s in scenarios}
        assert seen_states == set(GroundingState.ALL)
        for snapshot in scenarios:
            assert snapshot.reason, f"state {snapshot.state} has no reason"


class TestConfigValidation:
    @pytest.mark.parametrize("required", [0, -1])
    def test_invalid_confirmation_count_is_rejected(self, required):
        with pytest.raises(ValueError):
            GroundingStateMachine(required_confirmations=required)

    @pytest.mark.parametrize("timeout", [0.0, -5.0])
    def test_invalid_timeout_is_rejected(self, timeout):
        with pytest.raises(ValueError):
            GroundingStateMachine(request_timeout_sec=timeout)


class TestConfirmationEdgeSurvivesTraffic:
    """
    Regression: the lock edge must survive detection frames arriving between
    the confirming frame and the next timer tick.

    Detections arrive at camera rate (~30 Hz) while status ticks at 5 Hz, so
    several frames land in that gap on every real run. An implementation that
    cleared the edge on each incoming frame would drop the goal almost every
    time, and would do so intermittently, which is worse than never.
    """

    def test_edge_survives_frames_arriving_before_the_tick(self):
        m = machine(required=2, timeout=30.0)
        m.on_request(0.0)

        # Two frames confirm, then six more arrive before any tick.
        for i in range(8):
            m.on_detection_frame(0.1 + i * 0.01, 1, True, "ACCEPTED")

        assert m.state == GroundingState.CONFIRMED
        snapshot = m.tick(0.5)
        assert snapshot.just_confirmed, (
            "confirmation edge was lost to detection frames arriving before "
            "the timer consumed it"
        )

    def test_the_edge_is_still_consumed_exactly_once(self):
        m = machine(required=2, timeout=30.0)
        m.on_request(0.0)
        for i in range(8):
            m.on_detection_frame(0.1 + i * 0.01, 1, True, "ACCEPTED")

        assert m.tick(0.5).just_confirmed
        for t in (0.6, 0.7, 0.8):
            assert not m.tick(t).just_confirmed

    def test_a_new_request_clears_a_pending_edge(self):
        """A re-request must not immediately fire a stale confirmation."""
        m = machine(required=1, timeout=30.0)
        m.on_request(0.0)
        m.on_detection_frame(0.1, 1, True, "ACCEPTED")
        m.on_request(0.2)
        assert not m.tick(0.3).just_confirmed
