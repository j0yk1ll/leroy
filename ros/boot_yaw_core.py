"""Relative yaw from the real ROS 2 IMU source; zero motor authority.

A short stationary section measures this session's gyro-Z offset before
integrating heading. This is NOT absolute heading or calibrated odometry.
The source must provide the actual MCU-mapped ROS timestamps.
"""
from dataclasses import dataclass
import math
import statistics


MIN_STATIONARY_S = 2.5
MIN_STATIONARY_SAMPLES = 100
MAX_SAMPLE_GAP_S = 0.16
MAX_RECEIPT_AGE_S = 0.35
MAX_GYRO_STATIONARY_RAD_S = 0.08
MAX_GYRO_Z_RAD_S = 1.5
MAX_STATIONARY_GYRO_SD_RAD_S = 0.025
GRAVITY_MIN = 8.8
GRAVITY_MAX = 10.8


@dataclass(frozen=True)
class YawEstimate:
    state: str
    reason: str
    relative_yaw_rad: float | None
    gyro_z_bias_rad_s: float | None
    accepted_samples: int
    calibration_samples: int
    # Intentionally no motion enable, speed or physical yaw validity flag.


class BootYaw:
    """One session, one stationary bias estimate, unwrapped relative yaw.

    Any timestamp/freshness/sensor fault latches until process restart.
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

    def fault(self, reason):
        self.state = "FAULT_LATCHED"
        self.reason = reason
        self.yaw = None
        return self.estimate()

    def estimate(self):
        return YawEstimate(
            self.state, self.reason, self.yaw, self.bias,
            self.samples, len(self.z_rates))

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
        if self.last_receipt_ns is not None and receipt_ns <= self.last_receipt_ns:
            return self.fault("RECEIPT_CLOCK_REGRESSION")
        if self.last_stamp_ns is not None:
            gap = (stamp_ns - self.last_stamp_ns) * 1e-9
            if not 0 < gap <= MAX_SAMPLE_GAP_S:
                return self.fault("IMU_STAMP_GAP_OR_REGRESSION")
        if self.last_receipt_ns is not None:
            gap = (receipt_ns - self.last_receipt_ns) * 1e-9
            if not 0 < gap <= MAX_RECEIPT_AGE_S:
                return self.fault("IMU_DELIVERY_GAP")
        norm_a = math.sqrt(sum(v * v for v in accel))
        if not GRAVITY_MIN <= norm_a <= GRAVITY_MAX:
            return self.fault("ACCEL_MAGNITUDE_IMPLAUSIBLE")

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
                return self.fault("IMU_STALE")
        return self.estimate()
