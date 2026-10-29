"""Unpowered closed-loop yaw tracking for Leroy's real IMU heading.

Consumes relative heading from the already verified STM32 IMU source. Builds
a bounded *diagnostic* rate proposal and checks waypoint convergence, but
NEVER produces a ROS Twist, signs a motion envelope, enables the bridge,
opens a device, or authorizes any motor output. Physical control requires a
different, independently validated safety/authority boundary.
"""

# TODO(AUTONOMY) #031 [real yaw maneuver]: Characterize actual wheel-driven yaw
# rates, braking distance/overshoot, wheel-slip asymmetry, wheel signs and
# final error at -90, +90, 0 degrees on the physical robot.
# Keep the diagnostic observer separate from motor authority. Once real
# hardware stopping, clearance and exclusive command ownership are validated,
# use a separately reviewed closed-loop actuator that stops and settles
# at each waypoint; never auto-advance from this synthetic output alone.
# Acceptance: repeated attended runs plus injected IMU/command loss achieve
# measured tracking/stop bounds or fault-latch, with no runaway rotation.

from dataclasses import dataclass
import math


WAYPOINTS_RAD = (-math.pi / 2, math.pi / 2, 0.0)
HEADING_AGE_NS = 350_000_000
MAX_SESSION_NS = 240_000_000_000
MAX_WAYPOINT_NS = 100_000_000_000
SETTLE_NS = 500_000_000
MAX_TRACK_GAP_NS = 600_000_000
MAX_RELATIVE_ENVELOPE_RAD = math.radians(97.0)
TARGET_TOLERANCE_RAD = math.radians(3.0)
SETTLE_RATE_RAD_S = 0.05

# These are *illustrative unqualified controller coefficients*, not
# proven physically safe speed, acceleration, or braking limits.
DIAGNOSTIC_KP = 0.4
DIAGNOSTIC_RATE_CAP_RAD_S = 0.12


@dataclass(frozen=True)
class Heading:
    stamp_ns: int
    receipt_ns: int
    yaw_rad: float
    source_active: bool
    owner_verified: bool


@dataclass(frozen=True)
class TrackingDecision:
    state: str
    reason: str
    waypoint_index: int | None
    target_rad: float | None
    error_rad: float | None
    observed_rate_rad_s: float | None
    # Non-actuating diagnostic suggestion only. No command or grant token.
    diagnostic_wz_rad_s: float
    motor_motion_authorized: bool = False


class YawTracking:
    """Manually initiated manual-sweep witness with irreversible faults.

    Never substitutes the real physical stop, obstacle, swept footprint,
    signed motion authority, or calibrated odometry qualifications.
    """

    def __init__(self):
        self.state = "INHIBITED"
        self.reason = "EXPLICIT_LOCAL_START_REQUIRED"
        self.origin = None
        self.waypoint = 0
        self.started_at = None
        self.waypoint_at = None
        self.settle_at = None
        self.last_stamp = None
        self.last_yaw = None
        self.last_receipt = None
        self.observed_rate = None
        self.last_error = None

    def start(self):
        if self.state != "INHIBITED":
            return False
        self.state = "AWAITING_HEADING"
        self.reason = "WAITING_FOR_FRESH_GYRO_HEADING"
        return True

    def decision(self):
        target = None
        error = None
        proposal = 0.0
        if self.state in ("TRACKING", "SETTLING") and self.origin is not None:
            target = self.origin + WAYPOINTS_RAD[self.waypoint]
            error = self.last_error
            if self.state == "TRACKING" and error is not None:
                proposal = max(-DIAGNOSTIC_RATE_CAP_RAD_S, min(
                    DIAGNOSTIC_RATE_CAP_RAD_S, DIAGNOSTIC_KP * error))
        return TrackingDecision(
            self.state, self.reason,
            self.waypoint if target is not None else None,
            target, error, self.observed_rate, proposal)

    def _fault(self, why):
        self.state = "FAULT_LATCHED"
        self.reason = why
        return self.decision()

    def stop(self):
        if self.state in ("FAULT_LATCHED", "DONE"):
            return self.decision()
        self.state = "STOPPED"
        self.reason = "OPERATOR_STOPPED_NO_ACTUATION"
        return self.decision()

    def tick(self, sample: Heading, now_ns: int):
        if self.state in ("INHIBITED", "STOPPED", "FAULT_LATCHED", "DONE"):
            return self.decision()
        if (type(now_ns) is not int or now_ns <= 0
                or type(sample) is not Heading
                or type(sample.stamp_ns) is not int
                or type(sample.receipt_ns) is not int
                or type(sample.yaw_rad) not in (float, int)
                or not math.isfinite(sample.yaw_rad)):
            return self._fault("INVALID_HEADING_OR_CLOCK")
        if (sample.source_active is not True
                or sample.owner_verified is not True):
            return self._fault("HEADING_SOURCE_OR_OWNER_INVALID")
        if not (0 <= now_ns - sample.stamp_ns <= HEADING_AGE_NS
                and 0 <= now_ns - sample.receipt_ns <= HEADING_AGE_NS):
            return self._fault("HEADING_STALE")
        if self.last_stamp is not None:
            if (sample.stamp_ns <= self.last_stamp
                    or sample.receipt_ns <= self.last_receipt):
                return self._fault("HEADING_STAMP_REGRESSION")
            if sample.stamp_ns - self.last_stamp > MAX_TRACK_GAP_NS:
                return self._fault("HEADING_UPDATE_GAP")
            dt = (sample.stamp_ns - self.last_stamp) * 1e-9
            self.observed_rate = (sample.yaw_rad - self.last_yaw) / dt
            if not math.isfinite(self.observed_rate):
                return self._fault("YAW_RATE_INVALID")
        self.last_stamp = sample.stamp_ns
        self.last_receipt = sample.receipt_ns
        self.last_yaw = float(sample.yaw_rad)

        if self.origin is None:
            self.origin = float(sample.yaw_rad)
            self.started_at = now_ns
            self.waypoint_at = now_ns
            self.state = "TRACKING"
        if now_ns - self.started_at > MAX_SESSION_NS:
            return self._fault("MANUAL_SWEEP_SESSION_TIMEOUT")
        if now_ns - self.waypoint_at > MAX_WAYPOINT_NS:
            return self._fault("MANUAL_WAYPOINT_TIMEOUT")
        if abs(sample.yaw_rad - self.origin) > MAX_RELATIVE_ENVELOPE_RAD:
            return self._fault("YAW_ENVELOPE_EXCEEDED")

        target = self.origin + WAYPOINTS_RAD[self.waypoint]
        self.last_error = target - sample.yaw_rad
        if abs(self.last_error) <= TARGET_TOLERANCE_RAD:
            # One heading sample is not a settled physical orientation.
            if (self.observed_rate is not None and
                    abs(self.observed_rate) <= SETTLE_RATE_RAD_S):
                if self.settle_at is None:
                    self.settle_at = now_ns
                self.state = "SETTLING"
                self.reason = "TARGET_NEEDS_STATIONARY_DWELL"
                if now_ns - self.settle_at >= SETTLE_NS:
                    self.waypoint += 1
                    self.settle_at = None
                    self.waypoint_at = now_ns
                    self.last_error = None
                    if self.waypoint == len(WAYPOINTS_RAD):
                        self.state = "DONE"
                        self.reason = "MANUAL_SWEEP_OBSERVED_NO_MOTOR_RELEASE"
                    else:
                        self.state = "TRACKING"
                        self.reason = "NEXT_MANUAL_WAYPOINT"
            else:
                self.settle_at = None
                self.state = "TRACKING"
                self.reason = "TARGET_MOVING_NOT_SETTLED"
        else:
            self.settle_at = None
            self.state = "TRACKING"
            self.reason = "MANUAL_YAW_REQUIRED"
        return self.decision()

    def on_timeout(self, now_ns):
        if self.state in ("INHIBITED", "STOPPED", "FAULT_LATCHED", "DONE"):
            return self.decision()
        if type(now_ns) is not int or now_ns <= 0:
            return self._fault("CLOCK_INVALID")
        if (self.last_receipt is not None
                and now_ns - self.last_receipt > HEADING_AGE_NS):
            return self._fault("HEADING_LOST")
        if self.started_at is not None and now_ns - self.started_at > MAX_SESSION_NS:
            return self._fault("MANUAL_SWEEP_SESSION_TIMEOUT")
        return self.decision()
