"""
Fusion regression tests.

Every assertion here is derived from the documented semantics in
``go2_bringup/config/fusion.yaml`` and the module docstring of
``go2_intent_grounding/fusion.py``. The expected behaviour was written down
BEFORE these tests were made to pass, and the derivations are reproduced in
the test bodies so a future reader can check the arithmetic rather than trust
a magic number.

Covers required Test 11 (caller-motion boundary) and Test 12 (visual-only
fallback reachability).
"""
import math

import pytest
from go2_intent_grounding.bearings import (
    body_yaw_from_optical_position,
    camera_azimuth,
    camera_azimuth_to_body_yaw,
)
from go2_intent_grounding.fusion import (
    FusionParams,
    FusionReason,
    compute_audio_score,
    fuse,
    signed_angle_difference,
)

PARAMS = FusionParams()


def score_at(delta_deg, visual=0.9, params=PARAMS):
    """Fuse a candidate sitting ``delta_deg`` away from the acoustic bearing."""
    return fuse(
        visual_score=visual,
        candidate_bearing_rad=math.radians(delta_deg),
        acoustic_bearing_rad=0.0,
        audio_available=True,
        params=params,
    )


# ── Test 11: caller movement ──────────────────────────────────────────────


class TestCallerMotionBoundary:
    """
    The reported defect: confirmation failed at ~18.1 deg despite a parameter
    named ``bearing_tolerance_deg = 25``.

    Reproduced analytically against the OLD model below, then asserted absent
    from the new one. Documented expected semantics: the gate is HARD at
    ``bearing_gate_deg``; inside it a caller with sufficient visual confidence
    is confirmable; outside it the caller is rejected with BEARING_MISMATCH
    rather than with a low score.
    """

    def test_the_old_model_really_did_break_at_18_125_degrees(self):
        """
        Confirm the reported breakpoint was real, not a measurement artefact.

        Old model: score = 0.4*(1 - d/25) + 0.6*visual, threshold 0.65.
        With visual = 0.9:
            0.4 - 0.016d + 0.54 >= 0.65
            0.94 - 0.016d      >= 0.65
            d                  <= 18.125
        """

        def old_score(delta_deg, visual=0.9):
            audio = max(0.0, 1.0 - delta_deg / 25.0)
            return 0.4 * audio + 0.6 * visual

        assert old_score(0.0) == pytest.approx(0.940)
        assert old_score(25.0) == pytest.approx(0.540)
        assert old_score(18.0) >= 0.65
        assert old_score(19.0) < 0.65
        assert old_score(18.125) == pytest.approx(0.65)

    @pytest.mark.parametrize("delta_deg", [0.0, 5.0, 10.0, 18.0, 18.125, 19.0, 24.9])
    def test_confirmation_holds_across_the_old_breakpoint(self, delta_deg):
        """
        The regression itself: a 0.9-confidence caller stays confirmable at
        every angle inside the gate, including the angles that previously
        failed.
        """
        result = score_at(delta_deg)
        assert result.accepted, (
            f"caller at {delta_deg} deg rejected: "
            f"score={result.fused_score:.4f} reason={result.reason}"
        )

    def test_exactly_at_the_gate_is_still_accepted(self):
        """
        25.0 deg is inside the documented tolerance, so it must be accepted.

        At the gate edge corroboration is zero, so the score is
        visual * 0.60 = 0.54, which clears the 0.45 threshold for a
        0.9-confidence detection.
        """
        result = score_at(25.0)
        assert result.accepted
        assert result.fused_score == pytest.approx(0.9 * 0.60)

    def test_just_beyond_the_gate_is_rejected_with_a_reason(self):
        """Rejection must be explicit and machine-readable, not a low number."""
        result = score_at(25.001)
        assert not result.accepted
        assert result.reason == FusionReason.BEARING_MISMATCH

    def test_the_boundary_is_where_the_parameter_says_it_is(self):
        """
        Bisect the actual acceptance boundary and check it equals the
        configured gate.

        Under the old model this test would have located ~18.125 for a
        0.9-confidence caller, and somewhere else entirely for a caller with
        different confidence, which is the real defect: the effective
        tolerance depended on the detector.
        """
        low, high = 0.0, 90.0
        for _ in range(60):
            mid = (low + high) / 2.0
            if score_at(mid).accepted:
                low = mid
            else:
                high = mid
        assert low == pytest.approx(PARAMS.bearing_gate_deg, abs=1e-6)

    @pytest.mark.parametrize("visual", [0.75, 0.85, 0.95, 1.0])
    def test_the_boundary_does_not_move_with_detector_confidence(self, visual):
        """
        The heart of defect 1: the advertised tolerance must be a property of
        the geometry, not of how confident YOLO happened to be this frame.

        Every caller at or above the documented 0.75 design confidence must
        have the same effective tolerance.
        """
        low, high = 0.0, 90.0
        for _ in range(60):
            mid = (low + high) / 2.0
            if score_at(mid, visual=visual).accepted:
                low = mid
            else:
                high = mid
        assert low == pytest.approx(PARAMS.bearing_gate_deg, abs=1e-6)

    def test_score_decreases_monotonically_with_bearing_error(self):
        deltas = [0, 5, 10, 15, 20, 24.9]
        scores = [score_at(d).fused_score for d in deltas]
        for earlier, later in zip(scores, scores[1:]):
            assert earlier >= later

    def test_perfect_corroboration_returns_the_detector_confidence(self):
        """
        A property the old additive model did not have: with the audio and
        visual channels in perfect agreement, the fused score is exactly the
        detector's own confidence, so the number keeps its meaning.
        """
        for visual in (0.5, 0.75, 0.9, 1.0):
            assert score_at(0.0, visual=visual).fused_score == pytest.approx(visual)


# ── Test 12: visual-only fallback ─────────────────────────────────────────


class TestVisualOnlyFallback:
    """
    The reported defect: the visual-only path required a detection confidence
    of 0.65/0.7 = 0.928571..., far above the 0.5 the detector was configured
    to emit, making the "fallback" effectively unreachable.
    """

    def test_the_old_fallback_really_was_near_unreachable(self):
        """0.65 / 0.7 = 0.9285714..., versus a 0.5 detector threshold."""
        old_required = 0.65 / 0.7
        assert old_required == pytest.approx(0.9285714285714286)
        assert old_required > 0.9

    def test_the_new_fallback_threshold_matches_its_derivation(self):
        """
        Required confidence is min_confidence / audio_absent_factor
        = 0.44 / 0.85 = 0.5176470...
        """
        assert PARAMS.min_visual_for_visual_only == pytest.approx(0.44 / 0.85)
        assert PARAMS.min_visual_for_visual_only == pytest.approx(0.5176470588, abs=1e-9)

    def test_the_fallback_is_reachable_by_the_detector_that_feeds_it(self):
        """
        The reachability question, asked properly: can a detection the
        perception node is configured to emit actually confirm?

        perception_node's confidence_threshold is 0.5, and a person at
        conversational range routinely scores well above 0.6.
        """
        required = PARAMS.min_visual_for_visual_only
        assert required < 0.6, (
            f"visual-only fallback needs confidence {required:.4f}, which is "
            "above what the detector routinely produces"
        )

    @pytest.mark.parametrize(
        "visual,expected",
        [
            (0.51, False),   # just below 0.51765
            (0.5176, False),
            (0.52, True),    # just above
            (0.60, True),
            (0.75, True),    # the documented design confidence
            (0.90, True),
        ],
    )
    def test_analytically_derived_boundary_cases(self, visual, expected):
        result = fuse(
            visual_score=visual,
            candidate_bearing_rad=0.0,
            acoustic_bearing_rad=None,
            audio_available=False,
            params=PARAMS,
        )
        assert result.accepted is expected
        assert result.fused_score == pytest.approx(visual * PARAMS.audio_absent_factor)

    def test_stale_audio_routes_to_the_fallback(self):
        result = fuse(
            visual_score=0.8,
            candidate_bearing_rad=1.2,
            acoustic_bearing_rad=0.0,
            audio_available=False,
            params=PARAMS,
        )
        assert not result.audio_available
        assert result.fused_score == pytest.approx(0.8 * PARAMS.audio_absent_factor)

    def test_design_confidence_confirms_with_and_without_audio(self):
        """
        The stated design contract, asserted directly:

            "A caller detected with visual confidence >= 0.75 must be
             confirmable anywhere inside the bearing gate, and must remain
             confirmable when the microphone array is unavailable."
        """
        for delta in (0.0, 12.5, 25.0):
            assert score_at(delta, visual=0.75).accepted
        assert fuse(0.75, 0.0, None, False, PARAMS).accepted

    def test_the_threshold_sits_strictly_below_the_contract_boundary(self):
        """
        Guard against someone "tidying" the threshold back onto the boundary.

        0.75 * 0.60 evaluates to 0.44999999999999996 in IEEE 754, so a
        threshold of exactly 0.45 would reject the design case by one ulp.
        """
        boundary = 0.75 * PARAMS.corroboration_floor
        assert PARAMS.min_confidence < boundary
        assert boundary - PARAMS.min_confidence > 1e-3, (
            "threshold has no meaningful margin against rounding"
        )


class TestEvidenceOrdering:
    """
    Defect 3: absent evidence must not be treated worse than contradicting
    evidence, nor better than corroborating evidence.
    """

    def test_missing_audio_ranks_between_contradicting_and_corroborating(self):
        visual = 0.8
        corroborating = score_at(0.0, visual=visual).fused_score
        contradicting = score_at(25.0, visual=visual).fused_score
        absent = fuse(visual, 0.0, None, False, PARAMS).fused_score

        assert contradicting < absent < corroborating

    def test_the_old_model_had_this_backwards(self):
        """
        Under the old model, losing the microphone entirely (0.7*v) scored
        HIGHER than having it disagree (0.6*v) — and worse, disagreement made
        confirmation mathematically impossible at any confidence, since
        0.6*1.0 = 0.6 < 0.65.
        """
        visual = 1.0
        old_stale = visual * 0.7
        old_disagreeing = 0.4 * 0.0 + 0.6 * visual
        assert old_stale > old_disagreeing
        assert old_disagreeing < 0.65, (
            "with the old model a perfectly-detected person could never be "
            "confirmed while audio disagreed"
        )

    def test_configuration_that_would_reintroduce_the_defect_is_rejected(self):
        """
        The ordering property is enforced in code, not left to whoever edits
        the YAML next.
        """
        with pytest.raises(ValueError):
            FusionParams(audio_absent_factor=0.4)  # below the 0.60 floor


class TestBearingFrameConversion:
    """
    Defect 2: visual and acoustic bearings were compared across frames with
    opposite sign conventions.
    """

    def test_a_person_on_the_left_is_negative_in_optical_and_positive_in_body(self):
        angle = math.radians(15.0)
        x_optical = -math.sin(angle) * 3.0  # left of the camera => -x
        z_optical = math.cos(angle) * 3.0

        assert math.degrees(camera_azimuth(x_optical, z_optical)) == pytest.approx(-15.0)
        assert math.degrees(
            body_yaw_from_optical_position(x_optical, z_optical)
        ) == pytest.approx(15.0)

    def test_a_person_on_the_right_is_positive_in_optical_and_negative_in_body(self):
        angle = math.radians(20.0)
        x_optical = math.sin(angle) * 3.0
        z_optical = math.cos(angle) * 3.0

        assert math.degrees(camera_azimuth(x_optical, z_optical)) == pytest.approx(20.0)
        assert math.degrees(
            body_yaw_from_optical_position(x_optical, z_optical)
        ) == pytest.approx(-20.0)

    def test_the_uncorrected_comparison_would_reject_a_correct_match(self):
        """
        The defect, demonstrated end to end.

        A caller 15 deg to the robot's left, heard at +15 deg in body frame.
        Correct handling: 0 deg disagreement, accepted.
        Old handling: -15 vs +15 => 30 deg apparent disagreement, beyond the
        25 deg gate, rejected.
        """
        angle = math.radians(15.0)
        x_optical = -math.sin(angle) * 3.0
        z_optical = math.cos(angle) * 3.0
        acoustic_body = math.radians(15.0)

        raw_optical = camera_azimuth(x_optical, z_optical)
        uncorrected_delta = abs(math.degrees(raw_optical - acoustic_body))
        assert uncorrected_delta == pytest.approx(30.0)
        assert uncorrected_delta > PARAMS.bearing_gate_deg

        corrected = fuse(
            visual_score=0.9,
            candidate_bearing_rad=body_yaw_from_optical_position(x_optical, z_optical),
            acoustic_bearing_rad=acoustic_body,
            audio_available=True,
            params=PARAMS,
        )
        assert corrected.accepted
        assert corrected.bearing_delta_deg == pytest.approx(0.0, abs=1e-9)
        assert corrected.fused_score == pytest.approx(0.9)

    def test_the_mirror_image_impostor_is_now_rejected(self):
        """
        The corollary of the sign error: it accepted the WRONG person.

        A caller heard at +15 deg (left) and a bystander seen at +15 deg
        optical (right, i.e. -15 deg body) are 30 deg apart and must not
        associate.
        """
        angle = math.radians(15.0)
        x_optical = math.sin(angle) * 3.0  # bystander to the RIGHT
        z_optical = math.cos(angle) * 3.0

        result = fuse(
            visual_score=0.95,
            candidate_bearing_rad=body_yaw_from_optical_position(x_optical, z_optical),
            acoustic_bearing_rad=math.radians(15.0),
            audio_available=True,
            params=PARAMS,
        )
        assert not result.accepted
        assert result.reason == FusionReason.BEARING_MISMATCH

    def test_camera_yaw_offset_is_applied(self):
        """A non-boresighted camera is accounted for by an explicit offset."""
        yaw = camera_azimuth_to_body_yaw(0.0, camera_yaw_offset_rad=math.radians(30.0))
        assert math.degrees(yaw) == pytest.approx(30.0)


class TestAngleArithmetic:
    def test_signed_difference_wraps_correctly_near_pi(self):
        assert math.degrees(
            signed_angle_difference(math.radians(179), math.radians(-179))
        ) == pytest.approx(-2.0)

    def test_signed_difference_handles_negative_inputs(self):
        """
        The old ``min(d, 2*pi - d)`` normalisation was only correct for inputs
        already in [0, 2*pi] and produced wrong magnitudes for the negative
        angles this system actually operates on.
        """
        assert math.degrees(
            signed_angle_difference(math.radians(-10), math.radians(10))
        ) == pytest.approx(-20.0)

    def test_audio_score_is_symmetric_in_sign(self):
        left = compute_audio_score(math.radians(-10), 0.0, math.radians(25))
        right = compute_audio_score(math.radians(10), 0.0, math.radians(25))
        assert left == pytest.approx(right)

    def test_audio_score_endpoints(self):
        tol = math.radians(25)
        assert compute_audio_score(0.0, 0.0, tol) == pytest.approx(1.0)
        assert compute_audio_score(tol, 0.0, tol) == pytest.approx(0.0)
        assert compute_audio_score(2 * tol, 0.0, tol) == 0.0


class TestRobustness:
    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_non_finite_detection_confidence_is_rejected(self, bad):
        result = fuse(bad, 0.0, 0.0, True, PARAMS)
        assert not result.accepted
        assert result.reason == FusionReason.INVALID_DETECTION

    def test_non_finite_acoustic_bearing_falls_back_to_visual_only(self):
        result = fuse(0.9, 0.0, float("nan"), True, PARAMS)
        assert not result.audio_available
        assert result.fused_score == pytest.approx(0.9 * PARAMS.audio_absent_factor)

    def test_score_is_always_bounded(self):
        import random

        rng = random.Random(0)
        for _ in range(500):
            result = fuse(
                visual_score=rng.random(),
                candidate_bearing_rad=rng.uniform(-math.pi, math.pi),
                acoustic_bearing_rad=rng.uniform(-math.pi, math.pi),
                audio_available=rng.choice([True, False]),
                params=PARAMS,
            )
            assert 0.0 <= result.fused_score <= 1.0

    def test_fused_score_never_exceeds_visual_confidence(self):
        """
        Corroboration may only discount the detector, never inflate it.

        A 5 cm microphone baseline with no measured extrinsic must not be able
        to make the system more confident than the vision system alone.
        """
        import random

        rng = random.Random(1)
        for _ in range(500):
            visual = rng.random()
            result = fuse(
                visual_score=visual,
                candidate_bearing_rad=rng.uniform(-0.5, 0.5),
                acoustic_bearing_rad=rng.uniform(-0.5, 0.5),
                audio_available=rng.choice([True, False]),
                params=PARAMS,
            )
            assert result.fused_score <= visual + 1e-12
