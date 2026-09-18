"""Clock-offset estimator: learns the measured GO2 robot-clock skew."""
import pytest
from go2_localization.clock_offset import ClockOffsetEstimator, split_stamp

MEASURED_SKEW = 27_605_481.016  # recv - stamp on the 2026-09-01 standing bag


def test_not_ready_before_min_samples():
    est = ClockOffsetEstimator(min_samples=5)
    for i in range(4):
        est.add(100.0 + i, 100.0 + i + MEASURED_SKEW)
    assert not est.ready
    assert est.correct(0.0) is None


def test_learns_offset_as_minimum_latency():
    est = ClockOffsetEstimator(min_samples=3)
    latencies = [0.012, 0.004, 0.030, 0.006, 0.250]  # spikes only ever add
    for i, lat in enumerate(latencies):
        stamp = 1000.0 + i * 0.01
        est.add(stamp, stamp + MEASURED_SKEW + lat)
    assert est.offset_s == pytest.approx(MEASURED_SKEW + 0.004)
    assert est.correct(2000.0) == pytest.approx(2000.0 + MEASURED_SKEW + 0.004)


def test_clock_jump_resets_instead_of_blending():
    est = ClockOffsetEstimator(min_samples=2, jump_threshold_s=1.0)
    for i in range(10):
        est.add(100.0 + i, 100.0 + i + MEASURED_SKEW)
    # Robot reboots and its clock is now correct (offset ~0).
    for i in range(3):
        est.add(5000.0 + i, 5000.0 + i + 0.005)
    assert est.resets == 1
    assert est.offset_s == pytest.approx(0.005)


def test_window_forgets_old_minimum():
    est = ClockOffsetEstimator(window=3, min_samples=1)
    est.add(0.0, 10.000)
    for i in range(3):
        est.add(1.0 + i, 11.0 + i + 0.050)
    assert est.offset_s == pytest.approx(10.050)


@pytest.mark.parametrize("bad", [{"window": 0}, {"jump_threshold_s": 0.0}])
def test_invalid_configuration_rejected(bad):
    with pytest.raises(ValueError):
        ClockOffsetEstimator(**bad)


def test_split_stamp_roundtrip_and_carry():
    assert split_stamp(1.5) == (1, 500_000_000)
    assert split_stamp(1788291312.999999999999) == (1788291313, 0)
    sec, nsec = split_stamp(1788291312.25)
    assert 0 <= nsec < 1_000_000_000 and sec == 1788291312
