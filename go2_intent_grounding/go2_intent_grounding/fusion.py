"""
Audio-visual fusion for caller identification.

This module replaces an earlier additive scheme that had three defects, all
independently reproduced before the rewrite (see
``docs/END_TO_END_UPGRADE_REPORT.md`` §9).  Because those defects were
structural rather than numeric, the fix is a change of model, not a change of
constants.

The defects
-----------
1. **The advertised bearing tolerance was not the effective one.**  With
   ``score = 0.4·audio + 0.6·visual``, ``audio = 1 - Δ/25°`` and a threshold
   of 0.65, a caller with visual confidence 0.9 stopped being confirmable at
   Δ = 18.125°, not 25°::

       0.4·(1 - Δ/25) + 0.6·0.9 ≥ 0.65  ⟹  Δ ≤ 18.125°

   The effective tolerance was a function of the detector's confidence, which
   is not something a parameter named ``bearing_tolerance_deg`` can honestly
   describe.

2. **The visual-only fallback was very nearly unreachable.**  It returned
   ``visual · 0.7``, so clearing 0.65 required a detection confidence of
   ``0.65 / 0.7 = 0.928571…`` — far above the 0.5 the detector itself was
   configured to accept.  The "fallback" almost never fell back.

3. **Disagreeing audio was punished harder than absent audio.**  With audio
   fresh but inconsistent the ceiling was ``0.6 · visual = 0.6 < 0.65``, so
   confirmation was *impossible*; with audio simply stale the ceiling was
   ``0.7 · visual``.  Losing the microphone made the system more permissive
   than pointing it in a slightly wrong direction.  Combined with the
   camera-optical vs. body-frame sign error that existed upstream, a caller
   standing to the robot's left was scored as if they were on the right, so
   the audio channel actively opposed the correct answer.

The model
---------
Fusion is now **visual confidence modulated by acoustic corroboration**, with
association and confidence treated as two separate questions:

*Association* — "is this person the one who called?"  A hard gate at
``bearing_gate_deg``.  Beyond it the person is not a candidate at all and the
reason ``BEARING_MISMATCH`` is reported.  This is the parameter that means
what its name says.

*Confidence* — "am I sure enough to move a robot toward them?"::

    audio available:   fused = visual · (w_v + w_a·a) / (w_v + w_a)
    audio unavailable: fused = visual · audio_absent_factor

where ``a = clamp(1 - Δ/bearing_soft_deg, 0, 1)`` is corroboration strength.

Properties, all of which the previous model violated:

* Perfect corroboration returns the detector's own confidence unchanged
  (``fused = visual``), so the fused score keeps a meaning.
* The ordering is monotone and intuitive: full corroboration (1.00·v) >
  no audio at all (0.85·v) > weak corroboration (down to 0.60·v).  Absent
  evidence is never better than present evidence, and never worse than
  contradicting evidence.
* There is no discontinuity at the audio-timeout boundary large enough to
  flip a decision the wrong way.
* Contradicting evidence beyond the gate does not produce a low score to be
  compared against a threshold; it produces an explicit rejection with a
  reason code, which is what a blind user's feedback layer needs to say
  "I heard you over there but I can't see you there".

Threshold derivation
--------------------
``min_confidence_threshold`` is derived from a stated contract rather than
tuned until tests pass:

    A caller detected with visual confidence ≥ 0.75 must be confirmable
    anywhere inside the bearing gate, and must remain confirmable when the
    microphone array is unavailable.

    worst corroborated case:  0.75 · 0.60 = 0.450
    audio-unavailable case:   0.75 · 0.85 = 0.6375

    ⟹ threshold < 0.450.  Chosen: 0.44.

The inequality is **strict**, and the chosen value carries deliberate margin.
Setting the threshold to exactly 0.45 was tried first and rejected: in IEEE
754, ``0.75 * 0.6`` evaluates to 0.44999999999999996, so the design contract
would have failed by one unit in the last place. A safety-relevant acceptance
decision must not depend on floating-point rounding, so the threshold sits
below the boundary rather than on it.

At 0.44 the system still discriminates: a 0.5-confidence detection needs
strong corroboration to pass, and fails on visual-only (0.425 < 0.44).
Visual-only confirmation now needs ``0.44 / 0.85 = 0.518`` confidence rather
than 0.929.

Every constant is exposed in ``config/fusion.yaml`` with its unit, meaning and
derivation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


class FusionReason:
    """Why a candidate was accepted or rejected. Mirrors GroundingStatus.reason."""

    ACCEPTED = "ACCEPTED"
    #: Bearing difference exceeded the hard association gate.
    BEARING_MISMATCH = "BEARING_MISMATCH"
    #: Fused confidence below ``min_confidence``.
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    #: Detection confidence was not a usable number.
    INVALID_DETECTION = "INVALID_DETECTION"


@dataclass(frozen=True)
class FusionParams:
    """
    Fusion configuration. See ``config/fusion.yaml`` for provenance of each
    default.
    """

    #: Hard association gate (degrees). A detection whose bearing differs from
    #: the acoustic bearing by more than this is not the caller.
    bearing_gate_deg: float = 25.0
    #: Scale over which acoustic corroboration decays to zero (degrees).
    #: Equal to the gate by default, so corroboration reaches zero exactly at
    #: the gate rather than part-way through it.
    bearing_soft_deg: float = 25.0
    #: Relative weight of acoustic corroboration.
    audio_weight: float = 0.4
    #: Relative weight of visual detection confidence.
    visual_weight: float = 0.6
    #: Multiplier applied when no fresh acoustic bearing is available.
    #: Strictly between ``visual_weight/(visual_weight+audio_weight)`` and 1.0
    #: so that "no evidence" sits between "contradicting" and "corroborating".
    audio_absent_factor: float = 0.85
    #: Acceptance threshold on the fused score. Strictly below the design
    #: contract's worst case (0.450) so the decision does not hinge on
    #: floating-point rounding.
    min_confidence: float = 0.44

    def __post_init__(self) -> None:
        for name in ("bearing_gate_deg", "bearing_soft_deg", "audio_weight",
                     "visual_weight", "audio_absent_factor", "min_confidence"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number, got {value!r}")
        if self.bearing_gate_deg <= 0.0 or self.bearing_soft_deg <= 0.0:
            raise ValueError("bearing gates must be > 0")
        if self.audio_weight < 0.0 or self.visual_weight <= 0.0:
            raise ValueError("visual_weight must be > 0 and audio_weight >= 0")
        if not (0.0 < self.audio_absent_factor <= 1.0):
            raise ValueError("audio_absent_factor must be in (0, 1]")
        if not (0.0 < self.min_confidence <= 1.0):
            raise ValueError("min_confidence must be in (0, 1]")

        floor = self.visual_weight / (self.visual_weight + self.audio_weight)
        if not (floor <= self.audio_absent_factor <= 1.0):
            # If this fails, absent audio would rank outside the band between
            # contradicting and corroborating audio, reintroducing defect 3.
            raise ValueError(
                f"audio_absent_factor ({self.audio_absent_factor}) must lie in "
                f"[{floor:.4f}, 1.0] so that missing audio is never scored "
                "better than corroborating audio nor worse than contradicting "
                "audio"
            )

    @property
    def corroboration_floor(self) -> float:
        """Fused score multiplier when corroboration is zero at the gate edge."""
        return self.visual_weight / (self.visual_weight + self.audio_weight)

    @property
    def min_visual_for_visual_only(self) -> float:
        """Detector confidence needed to confirm with no audio at all."""
        return self.min_confidence / self.audio_absent_factor

    @property
    def min_visual_at_gate_edge(self) -> float:
        """Detector confidence needed to confirm at maximum bearing error."""
        return self.min_confidence / self.corroboration_floor

    @classmethod
    def from_mapping(cls, data) -> "FusionParams":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        return cls(**known)


@dataclass(frozen=True)
class FusionResult:
    fused_score: float
    audio_score: float
    visual_score: float
    #: Signed bearing difference in degrees (candidate minus acoustic), or NaN
    #: when no acoustic bearing was available.
    bearing_delta_deg: float
    audio_available: bool
    accepted: bool
    reason: str


def signed_angle_difference(a_rad: float, b_rad: float) -> float:
    """
    Shortest signed difference ``a - b``, wrapped to (-pi, pi].

    Uses atan2 rather than the ``min(d, 2π - d)`` form the previous
    implementation used: that form is only correct for inputs already in
    [0, 2π] and silently produces wrong magnitudes for negative angles, which
    is exactly the regime this system operates in (bearings are roughly
    ±90° about straight ahead).
    """
    return math.atan2(math.sin(a_rad - b_rad), math.cos(a_rad - b_rad))


def compute_audio_score(
    candidate_bearing_rad: float,
    acoustic_bearing_rad: float,
    soft_scale_rad: float,
) -> float:
    """
    Acoustic corroboration strength in [0, 1].

    1.0 when the candidate lies exactly on the acoustic bearing, decaying
    linearly to 0.0 at ``soft_scale_rad`` and staying at 0.0 beyond.  Both
    angles must already be in the SAME frame and sign convention — see
    :func:`go2_intent_grounding.bearings.camera_azimuth_to_body_yaw`.
    """
    if soft_scale_rad <= 0.0:
        return 0.0
    delta = abs(signed_angle_difference(candidate_bearing_rad, acoustic_bearing_rad))
    if delta >= soft_scale_rad:
        return 0.0
    return max(0.0, 1.0 - delta / soft_scale_rad)


def fuse(
    visual_score: float,
    candidate_bearing_rad: float,
    acoustic_bearing_rad: Optional[float],
    audio_available: bool,
    params: FusionParams,
) -> FusionResult:
    """
    Score one detection as a candidate caller.

    Args:
        visual_score: Detector confidence in [0, 1].
        candidate_bearing_rad: Candidate azimuth, **body frame, REP-103**
            (CCW-positive, 0 = straight ahead).
        acoustic_bearing_rad: Acoustic bearing in the same convention, or
            None.
        audio_available: Whether ``acoustic_bearing_rad`` is fresh enough to
            trust.
        params: Fusion configuration.

    Returns:
        A :class:`FusionResult`. ``accepted`` is the decision; ``reason`` says
        why, using codes that appear verbatim in ``go2_msgs/GroundingStatus``.
    """
    if not isinstance(visual_score, (int, float)) or not math.isfinite(visual_score):
        return FusionResult(
            0.0, 0.0, 0.0, float("nan"), False, False, FusionReason.INVALID_DETECTION
        )
    visual = max(0.0, min(1.0, float(visual_score)))

    have_audio = bool(audio_available) and acoustic_bearing_rad is not None
    if have_audio and not math.isfinite(acoustic_bearing_rad):  # type: ignore[arg-type]
        have_audio = False

    if not have_audio:
        fused = visual * params.audio_absent_factor
        accepted = fused >= params.min_confidence
        return FusionResult(
            fused_score=fused,
            audio_score=0.0,
            visual_score=visual,
            bearing_delta_deg=float("nan"),
            audio_available=False,
            accepted=accepted,
            reason=FusionReason.ACCEPTED if accepted else FusionReason.LOW_CONFIDENCE,
        )

    delta_rad = signed_angle_difference(
        candidate_bearing_rad, float(acoustic_bearing_rad)  # type: ignore[arg-type]
    )
    delta_deg = math.degrees(delta_rad)

    # Association gate first: a person outside the gate is not the caller, and
    # saying so is more useful than scoring them 0.54.
    if abs(delta_deg) > params.bearing_gate_deg:
        return FusionResult(
            fused_score=0.0,
            audio_score=0.0,
            visual_score=visual,
            bearing_delta_deg=delta_deg,
            audio_available=True,
            accepted=False,
            reason=FusionReason.BEARING_MISMATCH,
        )

    audio = compute_audio_score(
        candidate_bearing_rad,
        float(acoustic_bearing_rad),  # type: ignore[arg-type]
        math.radians(params.bearing_soft_deg),
    )
    total_w = params.visual_weight + params.audio_weight
    fused = visual * (params.visual_weight + params.audio_weight * audio) / total_w
    accepted = fused >= params.min_confidence

    return FusionResult(
        fused_score=fused,
        audio_score=audio,
        visual_score=visual,
        bearing_delta_deg=delta_deg,
        audio_available=True,
        accepted=accepted,
        reason=FusionReason.ACCEPTED if accepted else FusionReason.LOW_CONFIDENCE,
    )
