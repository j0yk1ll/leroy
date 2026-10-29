"""Pure-Python, *non-actuating* Leroy boot-localization planning model.

Inputs are untrusted synthetic evidence. This module does not access ROS,
serial devices, motors, GPIO, networking, or the filesystem. A Decision only
contains a desired heading FOR DIAGNOSTICS, never Twist, PWM, or release bits.
Neither a passing simulation nor a checked True flag constitutes a real-world
safety interlock. Keep hardware Nav2 disabled.
"""

from dataclasses import dataclass
import math


# Illustration/test thresholds, NOT physically measured robot speed/stopping
# bounds and NOT approved for hardware deployment.
SYNTHETIC_MAX_AGE_S = 0.25
SYNTHETIC_PREFLIGHT_DWELL_S = 1.0
SYNTHETIC_YAW_TOLERANCE_RAD = math.radians(3.0)
SYNTHETIC_SETTLE_S = 0.3
SYNTHETIC_MAX_SESSION_S = 60.0
SYNTHETIC_MAX_WAYPOINT_S = 20.0
SWEEP_RELATIVE_YAWS_RAD = (-math.pi / 2, math.pi / 2, 0.0)


@dataclass(frozen=True)
class Observation:
    """Every physical qualification defaults to False, including motion permit.

    yaw_rad is externally supplied, continuous (unwrapped) yaw relative
    to independent qualified local odometry, not XRSLAM. All event
    timestamps are monotonic and in the same domain as now.
    """

    now: float
    image_at: float | None = None
    obstacle_at: float | None = None
    yaw_at: float | None = None
    localization_at: float | None = None
    yaw_rad: float | None = None
    localization_status: str = "unknown"
    metric_pose_validated: bool = False
    navigation_release_validated: bool = False

    independent_stop_proven: bool = False
    verified_stop_distance: bool = False
    exclusive_command_authority: bool = False
    qualified_yaw_odometry: bool = False
    validated_tf_geometry: bool = False
    measured_swept_footprint: bool = False
    calibrated_obstacle_classifier: bool = False
    full_swept_envelope_observed: bool = False
    full_swept_envelope_clear: bool = False
    physical_motion_release: bool = False


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    diagnostic_yaw_target_rad: float | None = None
    # Deliberately no speed, /cmd_vel, motor enable, or authority output.


class BootLocalizationModel:
    """Fail-closed startup model with a latched fault and no auto re-arm.

    request_start() is a synthetic invocation, not physical authorization.
    A tracking monocular trajectory alone cannot make the model READY; a
    separately validated metric pose and navigation release are also needed.
    A pure yaw sweep can finish in NEEDS_PARALLAX, never trigger translation.
    """

    def __init__(self):
        self.state = "INHIBITED"
        self.reason = "NOT_STARTED"
        self.last_time = None
        self.start_time = None
        self.preflight_since = None
        self.origin_yaw = None
        self.waypoint = 0
        self.waypoint_since = None
        self.at_target_since = None

    def request_start(self):
        if self.state != "INHIBITED":
            return False
        self.state = "PREFLIGHT"
        self.reason = "AWAITING_EVIDENCE"
        return True

    def decision(self):
        target = None
        if self.state == "SWEEP" and self.origin_yaw is not None:
            target = self.origin_yaw + SWEEP_RELATIVE_YAWS_RAD[self.waypoint]
        return Decision(self.state, self.reason, target)

    def _fault(self, reason):
        self.state = "FAULT_LATCHED"
        self.reason = reason
        return self.decision()

    @staticmethod
    def _age_ok(now, stamp):
        return (type(stamp) in (int, float) and math.isfinite(stamp)
                and 0.0 <= now - stamp <= SYNTHETIC_MAX_AGE_S)

    @staticmethod
    def _physical_evidence(o):
        qualifications = (
            ("INDEPENDENT_STOP_UNPROVEN", o.independent_stop_proven),
            ("STOP_DISTANCE_UNVERIFIED", o.verified_stop_distance),
            ("COMMAND_AUTHORITY_UNVERIFIED", o.exclusive_command_authority),
            ("YAW_ODOMETRY_UNVERIFIED", o.qualified_yaw_odometry),
            ("TF_GEOMETRY_UNVERIFIED", o.validated_tf_geometry),
            ("SWEPT_FOOTPRINT_UNMEASURED", o.measured_swept_footprint),
            ("OBSTACLE_CLASSIFICATION_UNVERIFIED", o.calibrated_obstacle_classifier),
            ("SWEPT_ENVELOPE_NOT_OBSERVED", o.full_swept_envelope_observed),
            ("SWEPT_ENVELOPE_NOT_CLEAR", o.full_swept_envelope_clear),
            ("MOTION_RELEASE_NOT_VERIFIED", o.physical_motion_release),
        )
        for reason, verified in qualifications:
            if verified is not True:
                return reason
        return None

    def tick(self, o: Observation):
        if not isinstance(o, Observation):
            return self._fault("INVALID_INPUT")
        if self.state in ("INHIBITED", "FAULT_LATCHED"):
            return self.decision()
        now = o.now
        if (type(now) not in (int, float) or not math.isfinite(now)
                or (self.last_time is not None and now <= self.last_time)):
            return self._fault("CLOCK_INVALID_OR_REGRESSED")
        self.last_time = now
        if self.start_time is None:
            self.start_time = now
        if (self.state != "READY" and
                now - self.start_time > SYNTHETIC_MAX_SESSION_S):
            return self._fault("STARTUP_DEADLINE_EXCEEDED")

        fail = self._physical_evidence(o)
        if fail:
            return self._fault(fail)
        for name, stamp in (
            ("IMAGE", o.image_at),
            ("OBSTACLE", o.obstacle_at),
            ("YAW", o.yaw_at),
            ("LOCALIZATION", o.localization_at),
        ):
            if not self._age_ok(now, stamp):
                return self._fault("STALE_OR_INVALID_" + name)
        if (type(o.yaw_rad) not in (float, int)
                or not math.isfinite(o.yaw_rad)):
            return self._fault("YAW_INVALID")
        if o.localization_status not in ("initializing", "tracking", "lost"):
            return self._fault("LOCALIZATION_STATUS_INVALID")
        if o.localization_status == "lost":
            return self._fault("LOCALIZATION_LOST")

        if (self.state == "READY" and
                (o.localization_status != "tracking" or
                 o.metric_pose_validated is not True or
                 o.navigation_release_validated is not True)):
            return self._fault("READY_LOCALIZATION_OR_RELEASE_LOST")

        if o.localization_status == "tracking":
            # Never confuse camera-only visual tracking with metric VIO.
            if o.metric_pose_validated and o.navigation_release_validated:
                self.state = "READY"
                self.reason = "METRIC_LOCALIZATION_AND_NAVIGATION_VERIFIED"
            else:
                self.state = "WAIT_NAVIGATION_RELEASE"
                self.reason = "TRACKING_NOT_VALIDATED_FOR_NAVIGATION"
            return self.decision()
        if self.state == "READY":
            return self.decision()
        if self.state == "WAIT_NAVIGATION_RELEASE":
            return self._fault("TRACKING_REVERTED_TO_INITIALIZING")

        if self.state == "PREFLIGHT":
            if self.preflight_since is None:
                self.preflight_since = now
                self.reason = "SYNTHETIC_PREFLIGHT_DWELL"
            elif now - self.preflight_since >= SYNTHETIC_PREFLIGHT_DWELL_S:
                self.state = "SWEEP"
                self.origin_yaw = o.yaw_rad
                self.waypoint = 0
                self.waypoint_since = now
                self.reason = "DIAGNOSTIC_HEADING_SETPOINT_ONLY"
            return self.decision()

        if self.state == "SWEEP":
            # Explicit synthetic guard against overrunning the planned
            # [-90,+90] sector. NOT a physical braking envelope.
            if abs(o.yaw_rad - self.origin_yaw) > (
                    math.pi / 2 + math.radians(7.0)):
                return self._fault("SWEEP_ANGLE_ENVELOPE_EXCEEDED")
            if now - self.waypoint_since > SYNTHETIC_MAX_WAYPOINT_S:
                return self._fault("WAYPOINT_DEADLINE_EXCEEDED")
            target = self.origin_yaw + SWEEP_RELATIVE_YAWS_RAD[self.waypoint]
            if abs(o.yaw_rad - target) <= SYNTHETIC_YAW_TOLERANCE_RAD:
                if self.at_target_since is None:
                    self.at_target_since = now
                if now - self.at_target_since >= SYNTHETIC_SETTLE_S:
                    self.waypoint += 1
                    self.at_target_since = None
                    self.waypoint_since = now
                    if self.waypoint == len(SWEEP_RELATIVE_YAWS_RAD):
                        self.state = "NEEDS_PARALLAX"
                        self.reason = "SWEEP_COMPLETE_NO_TRANSLATION_AUTHORIZED"
            else:
                self.at_target_since = None
            return self.decision()

        if self.state == "NEEDS_PARALLAX":
            # Never plan a blind translation. Remain stationary awaiting a
            # separately qualified localization/calibration capability.
            self.reason = "PARALLAX_REQUIRED_NO_AUTO_TRANSLATION"
            return self.decision()
        return self._fault("UNKNOWN_STATE")
