"""Bounded MCU-to-ROS clock-rate estimator, independent of ROS imports.

MCU timestamps are uint32 microseconds. Incoming telemetry is a *one-way*
time observation: host receipt time is later than acquisition time by an
unknown and variable USB/serial latency. The low-delay envelope across
five-second windows estimates clock *rate*, not absolute clock offset.
"""

from collections import deque
from statistics import median


class ClockSyncError(RuntimeError):
    """Clock discontinuity: reject subsequent telemetry until bridge restart."""


class McuClockSync:
    """Unwrap MCU time and compensate relative drift without timestamp steps.

    The initial host receipt anchors the approximate epoch. The true one-way
    transport latency remains unknown; this mapper is NOT camera/IMU temporal
    calibration or a verified time-synchronization protocol.
    """

    _MASK = (1 << 32) - 1
    _BIN_US = 5_000_000
    _HISTORY_US = 240_000_000
    _MIN_SPAN_US = 45_000_000
    _MAX_GAP_US = 2_000_000
    _MAX_PPM = 200.0
    _MAX_HOST_STEP_NS = 100_000_000
    _INITIAL_TRANSPORT_GUARD_NS = 5_000_000

    def __init__(self):
        self._previous_raw = None
        self._elapsed_us = 0
        self._previous_ros_ns = None
        self._previous_steady_ns = None
        self._start_ros_ns = None
        self._last_stamp_ns = None
        self._rate = 1.0
        self._windows = deque()
        self._bucket = 0
        self._minimum = None  # (elapsed_us, received_ros_ns), lowest lag in bin
        self._bad_rates = 0
        self._fault = None

    @property
    def rate_ppm(self):
        """Host-clock microseconds per MCU microsecond, minus 1, in ppm."""
        return (self._rate - 1.0) * 1_000_000.0

    @property
    def ready(self):
        return len(self._windows) >= 2 and (
            self._windows[-1][0] - self._windows[0][0] >= self._MIN_SPAN_US
        )

    def _fail(self, reason):
        self._fault = reason
        raise ClockSyncError(reason)

    @staticmethod
    def _line(points):
        """Least-squares fit after subtracting origins to avoid epoch precision loss."""
        x0, y0 = points[0]
        xs = [(x - x0) / 1e6 for x, _ in points]
        ys = [(y - y0) / 1e9 for _, y in points]
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        denom = sum((x - mx) ** 2 for x in xs)
        if denom == 0:
            return 1.0, 0.0
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
        intercept = my - slope * mx
        return slope, intercept

    def _estimate_rate(self):
        points = list(self._windows)
        if len(points) < 8 or points[-1][0] - points[0][0] < self._MIN_SPAN_US:
            return
        slope, intercept = self._line(points)
        x0, y0 = points[0]
        # Discard windows with excessive positive serial delay, retaining
        # the low-latency envelope. Fit rate only, not an absolute offset.
        residual = [
            (y - y0) / 1e9 - (intercept + slope * ((x - x0) / 1e6))
            for x, y in points
        ]
        center = median(residual)
        keep = [
            point for point, r in zip(points, residual)
            if center - 0.015 <= r <= center + 0.005
        ]
        if len(keep) >= 8 and keep[-1][0] - keep[0][0] >= self._MIN_SPAN_US:
            slope, _ = self._line(keep)

        ppm = (slope - 1.0) * 1e6
        if abs(ppm) > self._MAX_PPM:
            self._bad_rates += 1
            if self._bad_rates >= 3:
                self._fail('estimated MCU/host clock-rate mismatch exceeds 200 ppm')
            return

        self._bad_rates = 0
        # Update gradually; the phase already accumulated by earlier rates
        # must not be recomputed when the estimate changes.
        self._rate += 0.25 * (slope - self._rate)

    def _sample_window(self, received_ros_ns):
        bucket = self._elapsed_us // self._BIN_US
        if bucket != self._bucket:
            self._windows.append(self._minimum)
            self._bucket = bucket
            self._minimum = None
            while self._windows and (
                self._elapsed_us - self._windows[0][0] > self._HISTORY_US
            ):
                self._windows.popleft()
            self._estimate_rate()

        point = (self._elapsed_us, received_ros_ns)
        if self._minimum is None or (
            received_ros_ns - self._elapsed_us * 1000
            < self._minimum[1] - self._minimum[0] * 1000
        ):
            self._minimum = point

    def observe(self, raw_us, received_ros_ns, received_steady_ns):
        """Return an int ROS epoch timestamp (ns), or latch a clock fault.

        Call once per serial telemetry line, in arrival order. Both host
        clocks must be sampled as close as possible to that receipt.
        """
        if self._fault is not None:
            raise ClockSyncError(self._fault)
        if not (0 <= raw_us <= self._MASK):
            self._fail('invalid uint32 MCU timestamp')

        if self._previous_raw is None:
            self._previous_raw = raw_us
            self._previous_ros_ns = received_ros_ns
            self._previous_steady_ns = received_steady_ns
            self._start_ros_ns = received_ros_ns - self._INITIAL_TRANSPORT_GUARD_NS
            self._last_stamp_ns = self._start_ros_ns
            self._minimum = (0, received_ros_ns)
            return self._last_stamp_ns

        delta_us = (raw_us - self._previous_raw) & self._MASK
        if delta_us > self._MAX_GAP_US:
            self._fail('MCU timestamp jumped, reordered or restarted')
        if received_steady_ns < self._previous_steady_ns:
            self._fail('host steady clock went backwards')
        if abs(
            (received_ros_ns - self._previous_ros_ns)
            - (received_steady_ns - self._previous_steady_ns)
        ) > self._MAX_HOST_STEP_NS:
            self._fail('host ROS clock stepped relative to monotonic clock')

        self._elapsed_us += delta_us
        self._sample_window(received_ros_ns)

        # Integrate the latest rate only over this MCU interval. Applying it
        # to total uptime would retroactively shift the accumulated phase.
        mapped = self._last_stamp_ns + round(delta_us * 1000 * self._rate)
        if mapped < self._last_stamp_ns or mapped > received_ros_ns + 100_000_000:
            self._fail('mapped timestamp is non-monotonic or far in the future')

        self._previous_raw = raw_us
        self._previous_ros_ns = received_ros_ns
        self._previous_steady_ns = received_steady_ns
        self._last_stamp_ns = mapped
        return mapped
