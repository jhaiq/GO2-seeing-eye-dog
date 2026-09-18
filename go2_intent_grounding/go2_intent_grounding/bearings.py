"""
Bearing frame conversions.

The previous implementation compared two angles that were expressed in
different frames with opposite sign conventions, and did so with a bare
subtraction.  This module exists so that conversion is explicit, named, and
testable, rather than an implicit assumption spread across two nodes.

The two conventions
-------------------
**Camera optical frame** (REP-105, ``*_color_optical_frame``): +x right,
+y down, +z forward.  ``perception_node`` back-projects detections into this
frame, so a person to the robot's *left* has **negative** x.  The azimuth
``atan2(x, z)`` is therefore **clockwise-positive** seen from above.

**Body frame** (REP-103, ``base_link``): +x forward, +y left, +z up, yaw
**counter-clockwise-positive**.  ``audio_perception_node`` asserts this
convention for its bearing (it publishes the matching unit vector with
``x=cos(az), y=sin(az)`` in ``base_link``).

So a caller 15° to the robot's left is ``-15°`` in the camera convention and
``+15°`` in the body convention.  Subtracting one from the other yields 30°
of apparent disagreement for a perfectly agreeing pair — and beyond the
25° gate, the correct match is rejected while its mirror image is accepted.
That is the sign error this module removes.

What this module does NOT do
----------------------------
It does not apply the camera-to-body **extrinsic**.  Doing that properly
requires the mounted pose of the RealSense relative to ``base_link``, which
must come from TF (a URDF or a static transform publisher), and this
repository does not yet ship a robot description.  The ``camera_yaw_offset_rad``
parameter is an explicit, documented placeholder for that rotation, defaulting
to zero (camera boresight aligned with the body x-axis).  It is marked
``hardware_validated: false`` in ``config/fusion.yaml``.

Treating a missing extrinsic as zero is an approximation; treating a sign
flip as correct was a defect.  These are different in kind, and only the
second one silently inverts the answer.
"""
from __future__ import annotations

import math


def camera_azimuth(x_optical: float, z_optical: float) -> float:
    """
    Azimuth of a point in the camera optical frame, clockwise-positive.

    This is the raw ``atan2(x, z)`` the perception pipeline produces. Returned
    for clarity at call sites; it is not directly comparable to a body yaw.
    """
    return math.atan2(x_optical, z_optical)


def camera_azimuth_to_body_yaw(
    azimuth_optical_rad: float,
    camera_yaw_offset_rad: float = 0.0,
) -> float:
    """
    Convert a clockwise-positive optical azimuth to a REP-103 body yaw.

    The negation is the whole point: it converts "positive means right" into
    "positive means left".  ``camera_yaw_offset_rad`` is added afterwards to
    account for a camera that is not boresighted along +x_body.

    Args:
        azimuth_optical_rad: ``atan2(x_optical, z_optical)``.
        camera_yaw_offset_rad: Yaw of the camera optical axis in ``base_link``,
            CCW-positive. Zero when the camera looks straight ahead.

    Returns:
        Yaw in ``base_link``, CCW-positive, wrapped to (-pi, pi].
    """
    yaw = -azimuth_optical_rad + camera_yaw_offset_rad
    return math.atan2(math.sin(yaw), math.cos(yaw))


def body_yaw_from_optical_position(
    x_optical: float,
    z_optical: float,
    camera_yaw_offset_rad: float = 0.0,
) -> float:
    """Convenience: optical position straight to body yaw."""
    return camera_azimuth_to_body_yaw(
        camera_azimuth(x_optical, z_optical), camera_yaw_offset_rad
    )
