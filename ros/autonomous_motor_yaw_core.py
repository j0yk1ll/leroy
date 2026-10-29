"""Exclusive signed motor-yaw controller, with independently signed safety lease.

This is the ACTUATING controller path: a successfully authorized decision may
be signed as MotionEnvelope and published to the bridge's existing
/leroy/motor/signed_motion topic. It never writes the MCU or creates a second
serial owner. Safety leases MUST be issued by a separately commissioned
hardware safety monitor, never created by this controller.

The production robot currently lacks a commissioned lease issuer and swept-
envelope sensor. There is intentionally no fabricated local PASS switch.
"""
from dataclasses import dataclass
import hashlib
import hmac
import json
import math

from boot_yaw_tracking_core import Heading, YawTracking

# These are hard SOFTWARE caps; lower values must come from physically
# commissioned speed/stopping characterization in the signed safety lease.
HARD_MAX_YAW_RATE_RAD_S = 0.12
HARD_MAX_RATE_CHANGE_RAD_S2 = 0.20
MAX_YAW_TRACK_RATE_RAD_S = 1.5
LEASE_MAX_AGE_NS = 150_000_000
MAX_MONITOR_LAG_NS = 150_000_000
MIN_STOP_MARGIN_M = 0.05
MIN_STOP_DWELL_NS = 700_000_000
LEASE_SCHEMA = "leroy_swept_safety_lease_v1"
LEASE_DOMAIN = "leroy-actuated-calibration-safety-v1"
LEASE_FIELDS = frozenset((
    "schema", "session_id", "sequence", "issued_ns", "expires_ns",
    "swept_footprint_id", "whole_envelope_measured",
    "whole_envelope_observed", "whole_envelope_clear",
    "independent_stop_proven", "stop_distance_measured",
    "command_owner_exclusive", "motor_output_interlock_proven",
    "stopped_verified", "max_yaw_rate_rad_s", "measured_stop_distance_m",
    "free_clearance_margin_m", "safety_monitor_id", "mac",
))


def _mac(key, fields):
    payload = json.dumps(fields, sort_keys=True, separators=(",", ":"),
                         allow_nan=False, ensure_ascii=True).encode("ascii")
    return hmac.new(key, payload, hashlib.sha256).hexdigest()


@dataclass(frozen=True)
class VerifiedLease:
    session_id: str
    sequence: int
    expires_ns: int
    max_yaw_rate_rad_s: float
    safety_monitor_id: str
    stopped_verified: bool
    # Only SafetyLeaseVerifier can return this data as accepted evidence.
    # It is NEVER a physical stop circuit or a motor release by itself.


class SafetyLeaseVerifier:
    """Validate one real hardware-monitor-authenticated, fresh safety lease.

    Key custody, hardware independence and measured footprint certification
    have to be established OUTSIDE this process. Without the issuer, the
    bridge must stay inhibited. A ROS publisher with copied booleans does
    not become a trusted physical monitor.
    """

    def __init__(self, *, safety_key, bridge_session_id):
        if type(safety_key) is not bytes or len(safety_key) != 32:
            raise ValueError("distinct protected 32-byte monitor key required")
        if (type(bridge_session_id) is not str
                or not 16 <= len(bridge_session_id) <= 128
                or not all(c in "0123456789abcdef" for c in bridge_session_id)):
            raise ValueError("invalid bridge session")
        self._key = safety_key
        self.session_id = bridge_session_id
        self.last_sequence = -1

    def accept(self, text, now_ns):
        if type(text) is not str or not 0 < len(text) <= 2048:
            return None
        try:
            data = json.loads(
                text, object_pairs_hook=self._unique,
                parse_constant=self._bad_constant)
            if type(data) is not dict or frozenset(data) != LEASE_FIELDS:
                return None
            if (data["schema"] != LEASE_SCHEMA
                    or type(data["session_id"]) is not str
                    or data["session_id"] != self.session_id
                    or type(data["sequence"]) is not int
                    or data["sequence"] <= self.last_sequence
                    or type(now_ns) is not int
                    or type(data["issued_ns"]) is not int
                    or type(data["expires_ns"]) is not int
                    or not 0 <= now_ns - data["issued_ns"] <= LEASE_MAX_AGE_NS
                    or not now_ns < data["expires_ns"]
                    or data["expires_ns"] - data["issued_ns"] > LEASE_MAX_AGE_NS):
                return None
            for name in (
                "whole_envelope_measured", "whole_envelope_observed",
                "whole_envelope_clear", "independent_stop_proven",
                "stop_distance_measured", "command_owner_exclusive",
                "motor_output_interlock_proven"):
                if data[name] is not True:
                    return None
            for name in (
                "max_yaw_rate_rad_s", "measured_stop_distance_m",
                "free_clearance_margin_m"):
                if type(data[name]) is not float or not math.isfinite(data[name]):
                    return None
            if (not 0 < data["max_yaw_rate_rad_s"] <= HARD_MAX_YAW_RATE_RAD_S
                    or not 0 <= data["measured_stop_distance_m"]
                    or data["free_clearance_margin_m"]
                    < data["measured_stop_distance_m"] + MIN_STOP_MARGIN_M
                    or type(data["stopped_verified"]) is not bool
                    or type(data["swept_footprint_id"]) is not str
                    or not data["swept_footprint_id"]
                    or type(data["safety_monitor_id"]) is not str
                    or not data["safety_monitor_id"]):
                return None
            mac = data.pop("mac")
            if (type(mac) is not str or len(mac) != 64
                    or not hmac.compare_digest(
                        mac, _mac(self._key, {
                            "domain": LEASE_DOMAIN, **data}))):
                return None
            self.last_sequence = data["sequence"]
            return VerifiedLease(
                session_id=self.session_id,
                sequence=data["sequence"],
                expires_ns=data["expires_ns"],
                max_yaw_rate_rad_s=data["max_yaw_rate_rad_s"],
                safety_monitor_id=data["safety_monitor_id"],
                stopped_verified=data["stopped_verified"])
        except (ValueError, TypeError, KeyError, OverflowError):
            return None

    @staticmethod
    def _unique(pairs):
        values = {}
        for name, value in pairs:
            if name in values:
                raise ValueError("duplicate safety lease field")
            values[name] = value
        return values

    @staticmethod
    def _bad_constant(_value):
        raise ValueError("nonfinite safety lease")


@dataclass(frozen=True)
class MotorYawDecision:
    state: str
    reason: str
    wz_rad_s: float
    waypoint_index: int | None
    operator_or_motor_rearm_required: bool


class MotorYawCalibration:
    """One automatic, originally-oriented +/-90-degree feedback session.

    Generates actual bounded angular velocity commands *only* with a
    continually refreshed valid hardware safety lease and current gyro.
    A latched fault is permanent; no automatic rearm on power restoration.
    """

    def __init__(self, bridge_session_id):
        if not bridge_session_id:
            raise ValueError("real bridge session required")
        self.session_id = bridge_session_id
        self.tracker = YawTracking()
        self.started = False
        self.fault = None
        self.last_time_ns = None
        self.last_wz = 0.0
        self.lease = None
        self.last_lease_at = None
        self.done_at = None

    def fresh_lease(self, lease, now_ns):
        if (type(lease) is not VerifiedLease
                or lease.session_id != self.session_id
                or type(now_ns) is not int or now_ns >= lease.expires_ns):
            return self.abort("NO_VERIFIED_PHYSICAL_SAFETY_LEASE")
        if self.lease is not None and lease.sequence <= self.lease.sequence:
            return self.abort("SAFETY_LEASE_REPLAY")
        self.lease = lease
        self.last_lease_at = now_ns
        return True

    def start(self):
        if self.started or self.fault:
            return False
        self.started = self.tracker.start()
        return self.started

    def abort(self, reason):
        self.fault = reason
        self.tracker._fault(reason)
        self.last_wz = 0.0
        return False

    def step(self, heading, now_ns):
        if not self.started or self.fault:
            return MotorYawDecision(
                "INHIBITED", self.fault or "NO_OPERATOR_REARM", 0.0,
                None, True)
        if (type(now_ns) is not int or now_ns <= 0 or
                self.lease is None or now_ns >= self.lease.expires_ns
                or self.last_lease_at is None
                or now_ns - self.last_lease_at > MAX_MONITOR_LAG_NS):
            self.abort("SAFETY_LEASE_EXPIRED_OR_LOST")
        if self.fault:
            return MotorYawDecision("LATCHED", self.fault, 0.0, None, True)

        if heading is None:
            status = self.tracker.on_timeout(now_ns)
        else:
            status = self.tracker.tick(heading, now_ns)
        if status.state == "FAULT_LATCHED":
            self.abort(status.reason)
        if self.fault:
            return MotorYawDecision("LATCHED", self.fault, 0.0, None, True)

        if self.last_time_ns is None:
            dt = 0.0
        else:
            dt = (now_ns - self.last_time_ns) * 1e-9
            if not 0 < dt <= 0.2:
                self.abort("MOTOR_CONTROLLER_LOOP_STALL_OR_CLOCK")
        if self.fault:
            return MotorYawDecision("LATCHED", self.fault, 0.0, None, True)
        self.last_time_ns = now_ns

        if status.state == "DONE":
            if self.done_at is None:
                self.done_at = now_ns
            # Keep sending authenticated ZERO while physical stop is checked
            # by the bridge and separately monitored safety hardware.
            return MotorYawDecision(
                "ZERO_DWELL", "WAIT_FOR_STOP_CONFIRMATION", 0.0, None, False)

        proposed = (status.diagnostic_wz_rad_s
                    if status.state == "TRACKING" else 0.0)
        max_wz = min(HARD_MAX_YAW_RATE_RAD_S,
                     self.lease.max_yaw_rate_rad_s)
        proposed = max(-max_wz, min(max_wz, proposed))
        max_delta = HARD_MAX_RATE_CHANGE_RAD_S2 * dt
        wz = max(self.last_wz - max_delta,
                 min(self.last_wz + max_delta, proposed))
        if status.state == "SETTLING":
            wz = 0.0
        self.last_wz = wz
        return MotorYawDecision(
            "ACTUATED_TRACKING" if wz else "HOLD_ZERO",
            status.reason, wz, status.waypoint_index, False)

    def finish_with_stop_confirmation(self, *, now_ns, stop_confirmed):
        if (self.done_at is None or self.fault
                or type(now_ns) is not int
                or now_ns - self.done_at < MIN_STOP_DWELL_NS
                or stop_confirmed is not True):
            return False
        self.started = False
        self.last_wz = 0.0
        return True
