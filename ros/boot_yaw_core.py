"""Relative yaw from the real ROS 2 IMU source; zero motor authority.

A short stationary section measures this session's gyro-Z offset before
integrating heading. This is NOT absolute heading or calibrated odometry.
The source must provide the actual MCU-mapped ROS timestamps.
"""

# TODO(AUTONOMY) [heading quality]: Measure gyro bias/temperature dependence,
# vibration, timebase uncertainty, yaw drift across repeated power cycles,
# and agreement with independent rotation observations on the real chassis.
# TODO(AUTONOMY) [fusion]: Supply uncertainty/covariance and source status
# to a qualified metric pose estimator; relative integrated yaw is not
# absolute heading and must not become odom->base_link without calibration.
# Acceptance: hardware timestamp jumps, dropped samples and excess drift
# invalidate heading without silently extrapolating through faults.

from dataclasses import dataclass
import math
import statistics


MIN_STATIONARY_S = 2.5
MIN_STATIONARY_SAMPLES = 100
MAX_SAMPLE_GAP_S = 0.16
MAX_BIAS_WINDOW_RESTARTS = 5
MAX_RECEIPT_AGE_S = 0.35
MAX_GYRO_STATIONARY_RAD_S = 0.08
MAX_GYRO_Z_RAD_S = 1.5
MAX_STATIONARY_GYRO_SD_RAD_S = 0.025
GRAVITY_MIN = 8.8
GRAVITY_MAX = 10.8
# Once heading exists, real chassis handling adds linear acceleration.
# This broad bound only catches grossly invalid acceleration samples;
# stationary gravity conformity must NOT be required during rotation.
ACTIVE_ACCEL_NORM_MIN_M_S2 = 0.1
ACTIVE_ACCEL_NORM_MAX_M_S2 = 60.0


@dataclass(frozen=True)
class YawEstimate:
    state: str
    reason: str
    relative_yaw_rad: float | None
    gyro_z_bias_rad_s: float | None
    accepted_samples: int
    calibration_samples: int
    bias_window_restarts: int
    last_stamp_delta_s: float | None
    last_receipt_delta_s: float | None
    last_accel_norm_m_s2: float | None
    dynamic_accel_samples: int
    # Intentionally no motion enable, speed or physical yaw validity flag.


class BootYaw:
    """One session, one stationary bias estimate, unwrapped relative yaw.

    Backward time, invalid data or post-heading loss latches until restart.
    A forward delivery gap before heading exists resets only the stationary
    bias window (bounded retries); it never extrapolates a heading.
    A restart begins at WAITING_IMU and never auto-resumes a motor session.
    """

    def __init__(self):
        self.state = "WAITING_IMU"
        self.reason = "NO_SENSOR_MESSAGES"
        self.last_stamp_ns = None
        self.last_receipt_ns = None
        self.first_stamp_ns = None
        self.z_rates = []
        self.bias = None
        self.yaw = None
        self.prev_rate = None
        self.samples = 0
        self.bias_window_restarts = 0
        self.last_stamp_delta_s = None
        self.last_receipt_delta_s = None
        self.last_accel_norm_m_s2 = None
        self.dynamic_accel_samples = 0

    def fault(self, reason):
        self.state = "FAULT_LATCHED"
        self.reason = reason
        self.yaw = None
        return self.estimate()

    def estimate(self):
        return YawEstimate(
            self.state, self.reason, self.yaw, self.bias,
            self.samples, len(self.z_rates), self.bias_window_restarts,
            self.last_stamp_delta_s, self.last_receipt_delta_s,
            self.last_accel_norm_m_s2, self.dynamic_accel_samples)

    def _restart_uncommitted_bias(self):
        # This is allowed ONLY before publishing any relative yaw estimate.
        if self.state not in ("WAITING_IMU", "CALIBRATING"):
            return self.fault("BIAS_WINDOW_RESET_AFTER_HEADING_FORBIDDEN")
        self.bias_window_restarts += 1
        if self.bias_window_restarts > MAX_BIAS_WINDOW_RESTARTS:
            return self.fault("BIAS_WINDOW_TOO_FRAGMENTED")
        self.z_rates.clear()
        self.first_stamp_ns = None
        self.bias = None
        self.prev_rate = None
        self.yaw = None
        self.state = "WAITING_IMU"
        self.reason = "RESTARTING_STATIONARY_BIAS_WINDOW"
        return self.estimate()

    def update(self, stamp_ns, receipt_ns, *, accel, gyro):
        if self.state == "FAULT_LATCHED":
            return self.estimate()
        if (type(stamp_ns) is not int or stamp_ns <= 0
                or type(receipt_ns) is not int or receipt_ns <= 0):
            return self.fault("INVALID_TIMESTAMP")
        if (len(accel) != 3 or len(gyro) != 3 or
                not all(type(x) in (float, int) and math.isfinite(x)
                        for x in (*accel, *gyro))):
            return self.fault("INVALID_SENSOR_VECTOR")
        if self.last_receipt_ns is not None:
            gap = (receipt_ns - self.last_receipt_ns) * 1e-9
            self.last_receipt_delta_s = gap
            if gap <= 0:
                return self.fault("RECEIPT_CLOCK_REGRESSION")
            if gap > MAX_RECEIPT_AGE_S:
                if self.state == "HEADING_ACTIVE":
                    return self.fault("IMU_DELIVERY_GAP")
                self._restart_uncommitted_bias()
                if self.state == "FAULT_LATCHED":
                    return self.estimate()
        if self.last_stamp_ns is not None:
            gap = (stamp_ns - self.last_stamp_ns) * 1e-9
            self.last_stamp_delta_s = gap
            if gap <= 0:
                return self.fault("IMU_STAMP_REGRESSION")
            if gap > MAX_SAMPLE_GAP_S:
                if self.state == "HEADING_ACTIVE":
                    return self.fault("IMU_STAMP_GAP_ACTIVE")
                # Do not count one discontinuity twice if it also
                # triggered a receipt gap reset in this same callback.
                if self.first_stamp_ns is not None:
                    self._restart_uncommitted_bias()
                    if self.state == "FAULT_LATCHED":
                        return self.estimate()
        norm_a = math.sqrt(sum(v * v for v in accel))
        self.last_accel_norm_m_s2 = norm_a
        if self.state in ("WAITING_IMU", "CALIBRATING"):
            # Stationary gyro-bias calibration depends on a stationary
            # chassis. Require a near-gravity accelerometer vector here.
            if not GRAVITY_MIN <= norm_a <= GRAVITY_MAX:
                return self.fault("ACCEL_MAGNITUDE_IMPLAUSIBLE_BIAS_WINDOW")
        else:
            # Relative yaw integrates angular-rate Z, not acceleration.
            # A dynamic accelerometer magnitude during hand rotation
            # must not erase a previously acquired heading.
            if not (ACTIVE_ACCEL_NORM_MIN_M_S2
                    <= norm_a <= ACTIVE_ACCEL_NORM_MAX_M_S2):
                return self.fault("ACCEL_SENSOR_OUT_OF_RANGE")
            if not GRAVITY_MIN <= norm_a <= GRAVITY_MAX:
                self.dynamic_accel_samples += 1

        prev_stamp = self.last_stamp_ns
        self.last_stamp_ns = stamp_ns
        self.last_receipt_ns = receipt_ns
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        self.samples += 1
        z = float(gyro[2])

        if self.state in ("WAITING_IMU", "CALIBRATING"):
            # This is performed only with the robot physically stationary.
            if math.sqrt(sum(v * v for v in gyro)) > MAX_GYRO_STATIONARY_RAD_S:
                return self.fault("MOTION_DURING_BIAS_ESTIMATE")
            self.state = "CALIBRATING"
            self.reason = "MEASURING_STATIONARY_GYRO_Z_BIAS"
            self.z_rates.append(z)
            span = (stamp_ns - self.first_stamp_ns) * 1e-9
            if (len(self.z_rates) >= MIN_STATIONARY_SAMPLES
                    and span >= MIN_STATIONARY_S):
                if (statistics.stdev(self.z_rates)
                        > MAX_STATIONARY_GYRO_SD_RAD_S):
                    return self.fault("STATIONARY_GYRO_TOO_NOISY")
                self.bias = statistics.mean(self.z_rates)
                self.yaw = 0.0
                self.prev_rate = z
                self.state = "HEADING_ACTIVE"
                self.reason = "RELATIVE_YAW_OBSERVABLE_UNVALIDATED"
        elif self.state == "HEADING_ACTIVE":
            if abs(z) > MAX_GYRO_Z_RAD_S:
                return self.fault("YAW_RATE_EXCEEDS_ESTIMATOR_ENVELOPE")
            dt = (stamp_ns - prev_stamp) * 1e-9
            self.yaw += 0.5 * ((z - self.bias)
                               + (self.prev_rate - self.bias)) * dt
            if not math.isfinite(self.yaw):
                return self.fault("INTEGRATED_YAW_INVALID")
            self.prev_rate = z
        else:
            return self.fault("UNKNOWN_YAW_STATE")
        return self.estimate()

    def check_freshness(self, now_ns):
        if self.state == "FAULT_LATCHED":
            return self.estimate()
        if type(now_ns) is not int or now_ns <= 0:
            return self.fault("INVALID_MONOTONIC_TIME")
        if self.last_receipt_ns is not None:
            age = (now_ns - self.last_receipt_ns) * 1e-9
            if not 0 <= age <= MAX_RECEIPT_AGE_S:
                if self.state == "HEADING_ACTIVE" or age < 0:
                    return self.fault("IMU_STALE")
                # Before a heading is established there is nothing to
                # extrapolate. Do not latch merely due to a transient
                # source pause: require a new contiguous stationary window.
                self.state = "WAITING_IMU"
                self.reason = "AWAITING_CONTIGUOUS_STATIONARY_IMU"
                self.z_rates.clear()
                self.first_stamp_ns = None
        return self.estimate()
