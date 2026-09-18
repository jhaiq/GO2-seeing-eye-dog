"""
Robot-clock to local-clock correction for GO2 sensor stamps.

Measured on the real robot (2026-09-01 standing bag): every stamp on
/utlidar/robot_odom, /utlidar/cloud and /utlidar/imu was 27,605,481 s (about
320 days) behind the payload's receive time, with only ~4 ms jitter on odom.
A TF tree built from those stamps is unusable: every lookup against the
payload's clock extrapolates by most of a year.

Receipt-time restamping would fix the date but throw away the robot's own
relative timing (a LiDAR sweep is stamped at scan time, ~70 ms before it
arrives). Instead this estimator learns a single offset from a
low-latency stream and applies it to every robot-clock stamp:

    offset = min over a sliding window of (receive_time - stamp)

The minimum approximates the true clock offset plus the smallest transport
delay, and is robust to latency spikes, which only ever increase the
difference. A robot clock jump (reboot, NTP step) shows up as a sample far
outside the window's range and resets the estimate instead of blending.
"""
from __future__ import annotations

from collections import deque
from typing import Optional


class ClockOffsetEstimator:
    def __init__(
        self,
        window: int = 300,
        jump_threshold_s: float = 1.0,
        min_samples: int = 5,
    ) -> None:
        if window < 1:
            raise ValueError("window must be >= 1")
        if jump_threshold_s <= 0.0:
            raise ValueError("jump_threshold_s must be > 0")
        self._samples: deque[float] = deque(maxlen=window)
        self._jump_threshold = jump_threshold_s
        self._min_samples = max(1, min_samples)
        self.resets = 0

    def add(self, stamp_s: float, receive_s: float) -> None:
        """Record one (robot stamp, local receive time) pair, both in seconds."""
        diff = receive_s - stamp_s
        if self._samples:
            current = min(self._samples)
            # A negative jump larger than the threshold means the robot clock
            # moved forward (or local moved back); a positive jump larger than
            # threshold on EVERY recent sample cannot be latency. Either way
            # the old window describes a different clock.
            if abs(diff - current) > self._jump_threshold:
                self._samples.clear()
                self.resets += 1
        self._samples.append(diff)

    @property
    def ready(self) -> bool:
        return len(self._samples) >= self._min_samples

    @property
    def offset_s(self) -> Optional[float]:
        if not self.ready:
            return None
        return min(self._samples)

    def correct(self, stamp_s: float) -> Optional[float]:
        """Map a robot-clock stamp to local time, or None before the estimate is ready."""
        offset = self.offset_s
        if offset is None:
            return None
        return stamp_s + offset


def split_stamp(t_s: float) -> tuple[int, int]:
    """Seconds as float -> (sec, nanosec) with nanosec in [0, 1e9)."""
    total_ns = int(round(t_s * 1e9))
    sec, nsec = divmod(total_ns, 1_000_000_000)
    return int(sec), int(nsec)
