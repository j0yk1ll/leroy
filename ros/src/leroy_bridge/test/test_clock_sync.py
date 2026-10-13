"""Deterministic clock-mapping regressions; no ROS, serial, or MCU required."""

import pytest

from leroy_bridge.clock_sync import ClockSyncError, McuClockSync


MASK = (1 << 32) - 1
EPOCH_NS = 1_797_000_000_000_000_000


def test_bounded_drift_compensation_over_six_minutes_and_uint32_rollover():
    sync = McuClockSync()
    base_raw = MASK - 10_000_000  # Force a natural wrap 10 seconds in.
    measurements = []

    # MCU tick clock is 42 ppm faster than ROS host clock. Include
    # deterministic USB delivery jitter, but no host wall-clock steps.
    for i in range(36001):
        mcu_elapsed_us = i * 10_000
        host_elapsed_ns = round(mcu_elapsed_us * 1000 * (1.0 - 42e-6))
        latency_ns = 7_000_000 + (i % 11) * 120_000
        recv_ns = EPOCH_NS + host_elapsed_ns + latency_ns
        raw = (base_raw + mcu_elapsed_us) & MASK
        stamp_ns = sync.observe(raw, recv_ns, recv_ns - EPOCH_NS)
        if i > 24000 and i % 1000 == 0:
            measurements.append((recv_ns - stamp_ns) / 1e6)

    assert sync.ready
    assert -50 < sync.rate_ppm < -34
    assert max(measurements) - min(measurements) < 3.0


def test_same_tick_for_imu_and_encoder_returns_identical_stamp():
    sync = McuClockSync()
    first = sync.observe(100, EPOCH_NS, 10_000_000)
    repeat = sync.observe(100, EPOCH_NS + 150_000, 10_150_000)
    following = sync.observe(10_100, EPOCH_NS + 10_000_000, 20_000_000)
    assert first == repeat
    assert following > repeat


def test_small_reorder_latches_fault():
    sync = McuClockSync()
    sync.observe(10_000, EPOCH_NS, 10_000_000)
    with pytest.raises(ClockSyncError, match='jumped, reordered or restarted'):
        sync.observe(9_990, EPOCH_NS + 100_000, 10_100_000)
    with pytest.raises(ClockSyncError):
        sync.observe(20_000, EPOCH_NS + 10_000_000, 20_000_000)


def test_firmware_reboot_latches_fault_instead_of_reanchoring():
    sync = McuClockSync()
    sync.observe(3_000_000_000, EPOCH_NS, 10_000_000)
    with pytest.raises(ClockSyncError, match='jumped, reordered or restarted'):
        sync.observe(40_000, EPOCH_NS + 10_000_000, 20_000_000)


def test_uart_pause_longer_than_two_seconds_is_not_silently_accepted():
    sync = McuClockSync()
    sync.observe(100, EPOCH_NS, 10_000_000)
    with pytest.raises(ClockSyncError, match='jumped, reordered or restarted'):
        sync.observe(3_010_100, EPOCH_NS + 3_010_000_000, 3_020_000_000)


def test_host_wall_clock_step_latches_fault():
    sync = McuClockSync()
    sync.observe(100, EPOCH_NS, 10_000_000)
    with pytest.raises(ClockSyncError, match='host ROS clock stepped'):
        sync.observe(10_100, EPOCH_NS + 1_010_000_000, 20_000_000)


def test_wrapped_timestamp_does_not_require_epoch_reset():
    sync = McuClockSync()
    first = sync.observe(MASK - 1_000, EPOCH_NS, 10_000_000)
    second = sync.observe(8_999, EPOCH_NS + 10_000_000, 20_000_000)
    assert second > first


def test_long_serial_delays_are_filtered_from_rate_estimator():
    sync = McuClockSync()
    for i in range(18001):
        t_us = i * 10_000
        host_ns = round(t_us * 1000 * (1.0 + 33e-6))
        # Delayed packets are not necessarily uniformly distributed. The
        # estimator should prefer low-delay observations in each window.
        latency_ns = 6_000_000 + (4_000_000 if i % 101 < 35 else 0)
        received = EPOCH_NS + host_ns + latency_ns
        sync.observe(t_us, received, received - EPOCH_NS)

    assert 25 < sync.rate_ppm < 40

def test_rate_change_does_not_rewrite_elapsed_timestamp_phase():
    """Changing the estimated rate must not retimestamp earlier MCU time."""
    sync = McuClockSync()
    # Isolate the mapper from estimator convergence. The physical MCU/host
    # rate changes after one hour and the estimate tracks it immediately.
    sync._estimate_rate = lambda: None
    rate_before = 1.0 - 40e-6
    rate_after = 1.0 - 10e-6
    sync._rate = rate_before

    offsets_ms = []
    host_elapsed_ns = 0
    last_stamp_ns = None
    for second in range(5401):
        if second == 3601:
            sync._rate = rate_after
        if second:
            rate = rate_before if second <= 3600 else rate_after
            host_elapsed_ns += round(1_000_000_000 * rate)
        received_ns = EPOCH_NS + host_elapsed_ns + 8_000_000
        # Cross the natural uint32 microsecond rollover at ~71.6 minutes.
        raw = (second * 1_000_000) & MASK
        stamp_ns = sync.observe(raw, received_ns, host_elapsed_ns)
        offsets_ms.append((received_ns - stamp_ns) / 1e6)
        if last_stamp_ns is not None:
            assert stamp_ns > last_stamp_ns
        last_stamp_ns = stamp_ns

    # The mapper initially anchors 5 ms before receipt. When frequency changes
    # but the mapping is integrated, this phase offset must remain unchanged.
    assert max(offsets_ms) - min(offsets_ms) < 0.01
    assert abs(offsets_ms[3601] - offsets_ms[3600]) < 0.01
    assert abs(offsets_ms[-1] - offsets_ms[0]) < 0.01
