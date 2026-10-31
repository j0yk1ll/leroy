"""Automatic stationarity-based gyro calibration verification for real M18 IMU.

Leroy's BootYaw already estimates a boot-session gyro Z bias before heading
becomes active. This independent 8-second holdout checks that correction
against fresh, physically observed gyro-Z noise, split-window bias agreement,
and unwrapped yaw drift while the chassis remains stationary.

A passed candidate is diagnostic evidence of stationary consistency, not
absolute heading, camera/IMU extrinsics, speed, or permission to move.
"""

# TODO(AUTONOMY) #004 [repeatable stationary check]: Characterize gyro-Z bias,
# temperature, vibration, gravity magnitude and drift on multiple real
# boots; promote empirically measured thresholds rather than test guesses.
# Export calibration uncertainty and a versioned evidence timestamp to a
# supervisor. This pass remains diagnostic, not an actuator permission.
# Acceptance: slight chassis motion, delayed samples, temperature swings
# and repeated aborts cannot be mistaken for a verified stationary holdout.

from dataclasses import dataclass
import math
import statistics


MIN_WINDOW_NS = 8_000_000_000
MIN_SAMPLES = 350
MAX_STAMP_GAP_NS = 160_000_000
MAX_RECEIPT_GAP_NS = 350_000_000
MAX_GYRO_STATIONARY_RAD_S = 0.08
GRAVITY_MIN_M_S2 = 8.8
GRAVITY_MAX_M_S2 = 10.8
MAX_BIAS_DISAGREEMENT_RAD_S = 0.008
MAX_SPLIT_WINDOW_BIAS_DELTA_RAD_S = 0.006
MAX_GYRO_Z_STD_RAD_S = 0.020
MAX_STATIONARY_YAW_DRIFT_RAD = math.radians(0.5)
MAX_WINDOW_RESTARTS = 4


@dataclass(frozen=True)
class CalibrationStatus:
    state: str
    reason: str
    samples: int
    window_restarts: int
    window_elapsed_s: float | None
    independent_bias_rad_s: float | None
    heading_bias_rad_s: float | None
    bias_disagreement_rad_s: float | None
    split_bias_delta_rad_s: float | None
    gyro_z_noise_std_rad_s: float | None
    yaw_drift_deg: float | None
    motor_motion_authorized: bool = False
    physical_heading_accuracy_validated: bool = False
    camera_imu_extrinsics_validated: bool = False


class AutoImuCalibration:
    """One-shot post-heading calibration on the same physical MCU stream.

    Motion during an unfinished verification restarts from zero. No reported
    success is possible without a continuous, independently checked window.
    An invalid timestamp, source defect or excess restarts latches a fault.
    """

    def __init__(self):
        self.state = "WAITING_HEADING"
        self.reason = "NEEDS_INITIAL_LIVE_GYRO_BIAS"
        self.restarts = 0
        self.start_ns = None
        self.last_stamp_ns = None
        self.last_receipt_ns = None
        self.start_yaw = None
        self.samples = []
        self.bias = None
        self.last_result = None
        self.last_elapsed_s = None

    def status(self):
        return CalibrationStatus(
            state=self.state, reason=self.reason, samples=len(self.samples),
            window_restarts=self.restarts,
            window_elapsed_s=self.last_elapsed_s,
            independent_bias_rad_s=(
                self.last_result["bias"] if self.last_result else None),
            heading_bias_rad_s=self.bias,
            bias_disagreement_rad_s=(
                self.last_result["disagreement"] if self.last_result else None),
            split_bias_delta_rad_s=(
                self.last_result["split"] if self.last_result else None),
            gyro_z_noise_std_rad_s=(
                self.last_result["noise"] if self.last_result else None),
            yaw_drift_deg=(
                self.last_result["drift_deg"] if self.last_result else None))

    def fault(self, reason):
        self.state = "FAULT_LATCHED"
        self.reason = reason
        self.last_result = None
        return self.status()

    def _restart(self, reason):
        self.restarts += 1
        self.start_ns = None
        self.last_stamp_ns = None
        self.last_receipt_ns = None
        self.start_yaw = None
        self.bias = None
        self.samples.clear()
        self.last_result = None
        self.last_elapsed_s = None
        if self.restarts > MAX_WINDOW_RESTARTS:
            return self.fault("STATIONARY_CALIBRATION_TOO_FRAGMENTED")
        self.state = "WAITING_HEADING"
        self.reason = reason
        return self.status()

    def update(self, stamp_ns, receipt_ns, *, accel, gyro,
               heading_state, heading_yaw_rad, heading_bias_rad_s):
        if self.state in ("VERIFIED_STATIONARY_CANDIDATE", "FAULT_LATCHED"):
            return self.status()
        if (type(stamp_ns) is not int or stamp_ns <= 0
                or type(receipt_ns) is not int or receipt_ns <= 0
                or len(accel) != 3 or len(gyro) != 3
                or not all(type(v) in (float, int) and math.isfinite(v)
                           for v in (*accel, *gyro))):
            return self.fault("INVALID_IMU_CALIBRATION_SAMPLE")

        if heading_state != "HEADING_ACTIVE":
            if self.state == "COLLECTING":
                return self.fault("ACTIVE_HEADING_DISAPPEARED")
            return self.status()
        if (type(heading_yaw_rad) not in (float, int)
                or type(heading_bias_rad_s) not in (float, int)
                or not math.isfinite(heading_yaw_rad)
                or not math.isfinite(heading_bias_rad_s)):
            return self.fault("INVALID_HEADING_CALIBRATION_INPUT")

        if self.last_stamp_ns is not None:
            delta_ns = stamp_ns - self.last_stamp_ns
            receipt_delta = receipt_ns - self.last_receipt_ns
            if delta_ns <= 0 or receipt_delta <= 0:
                return self.fault("CALIBRATION_TIMESTAMP_REGRESSION")
            if (delta_ns > MAX_STAMP_GAP_NS
                    or receipt_delta > MAX_RECEIPT_GAP_NS):
                return self._restart("RESTART_AFTER_IMU_DELIVERY_GAP")

        a_norm = math.sqrt(sum(a * a for a in accel))
        g_norm = math.sqrt(sum(g * g for g in gyro))
        if (not GRAVITY_MIN_M_S2 <= a_norm <= GRAVITY_MAX_M_S2
                or g_norm > MAX_GYRO_STATIONARY_RAD_S):
            return self._restart("WAITING_FOR_STATIONARY_CHASSIS")

        if self.start_ns is None:
            self.start_ns = stamp_ns
            self.start_yaw = float(heading_yaw_rad)
            self.bias = float(heading_bias_rad_s)
            self.state = "COLLECTING"
            self.reason = "AUTOMATIC_STATIONARY_HOLDOUT"
        if abs(heading_bias_rad_s - self.bias) > 1e-12:
            return self.fault("HEADING_BIAS_CHANGED_WITHIN_SESSION")

        self.last_stamp_ns = stamp_ns
        self.last_receipt_ns = receipt_ns
        self.samples.append(float(gyro[2]))
        self.last_elapsed_s = (stamp_ns - self.start_ns) * 1e-9
        if (len(self.samples) < MIN_SAMPLES
                or stamp_ns - self.start_ns < MIN_WINDOW_NS):
            return self.status()

        middle = len(self.samples) // 2
        first = self.samples[:middle]
        last = self.samples[middle:]
        bias = statistics.mean(self.samples)
        split = abs(statistics.mean(first) - statistics.mean(last))
        disagreement = abs(bias - self.bias)
        noise = statistics.stdev(self.samples)
        drift = float(heading_yaw_rad) - self.start_yaw
        self.last_result = {
            "bias": bias, "split": split, "disagreement": disagreement,
            "noise": noise, "drift_deg": math.degrees(drift),
        }
        if (disagreement > MAX_BIAS_DISAGREEMENT_RAD_S
                or split > MAX_SPLIT_WINDOW_BIAS_DELTA_RAD_S
                or noise > MAX_GYRO_Z_STD_RAD_S
                or abs(drift) > MAX_STATIONARY_YAW_DRIFT_RAD):
            return self.fault("STATIONARY_BIAS_OR_YAW_DRIFT_UNSTABLE")
        self.state = "VERIFIED_STATIONARY_CANDIDATE"
        self.reason = "STATIONARY_GYRO_CONSISTENCY_PASSED_ONLY"
        return self.status()

    def on_timeout(self, now_ns):
        if self.state in ("FAULT_LATCHED", "VERIFIED_STATIONARY_CANDIDATE"):
            return self.status()
        if type(now_ns) is not int or now_ns <= 0:
            return self.fault("CALIBRATION_MONOTONIC_CLOCK_INVALID")
        if self.last_receipt_ns is not None:
            if not 0 <= now_ns - self.last_receipt_ns <= MAX_RECEIPT_GAP_NS:
                return self._restart("AWAITING_FRESH_STATIONARY_IMU")
        return self.status()
