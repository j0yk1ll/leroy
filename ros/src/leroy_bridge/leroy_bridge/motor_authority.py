"""Single-session bridge motor authority; NO serial I/O and NO ROS subscriber.

This is the actuator-facing control core, not a diagnostic permission flag.
It constructs bounded, canonical MCU set_wheels packets ONLY after a
signed operator rearm, a one-second uninterrupted healthy dwell and
fresh, signed, non-replayed controller commands.

The production bridge's M22D nonzero serial-command inhibit is intentionally
NOT removed by this commit. Physical use requires trusted local health,
key provisioning and MCU-stop implementation at the sole serial writer.
"""

# TODO(AUTONOMY) [mode-specific motion permits]: Replace the one set of
# navigation health prerequisites with documented, independently evidenced
# authority policies for attended wheel checks, unlocalized boot motion,
# and fully localized Nav2; do not waive environmental/stop protection.
# TODO(AUTONOMY) [motion bounds]: Calibrate actual mecanum wheel signs,
# radii, track/wheelbase, PWM deadband, driver limits, encoder feedback
# and wheel-current/load response before converting twist to duty.
# Acceptance: each missing/stale independent observation, command replay,
# incompatible session or emergency cut forces a latched zero/STOP path.

from dataclasses import dataclass
import math

from leroy_bridge.authorized_session import (
    MotionEnvelope, RearmEnvelope, SessionVerifier,
)
from leroy_bridge.motor_pwm import mecanum_twist_to_pwm


ZERO_WHEELS = "set_wheels 0 0 0 0"
REARM_DWELL_NS = 1_000_000_000
MAX_TICK_GAP_NS = 200_000_000
MAX_COMMAND_AGE_NS = 250_000_000
VX_LIMIT_MPS = 0.15
VY_LIMIT_MPS = 0.15
WZ_LIMIT_RADPS = 0.30
# This is a *software maximum*, not a proven physical wheel duty/speed.
HARD_PWM_CEILING = 64
HEALTH_KEYS = frozenset((
    "odom_fresh", "depth_fresh", "tf_fresh",
    "costmap_valid", "localization_valid",
))


@dataclass(frozen=True)
class MotorDecision:
    state: str
    reason: str
    wheel_pwm: tuple[int, int, int, int]
    # Packet is informational UNTIL serial writer integration is reviewed.
    packet: str
    nonzero_candidate: bool


def _zero(state, reason):
    return MotorDecision(state, reason, (0, 0, 0, 0), ZERO_WHEELS, False)


class MotorAuthority:
    """Own exactly one motion verifier/session; fault recovery needs rearm.

    Caller MUST supply measured geometry, individually provisioned secrets,
    independent physical e-stop/motor-rail facts and trusted sensor health.
    Passing synthetic booleans into this class does NOT validate hardware.
    """

    def __init__(self, *, verifier, wheel_radius_m, half_wheelbase_m,
                 half_track_m, pwm_per_rad_s, max_pwm):
        if type(verifier) is not SessionVerifier:
            raise TypeError("exact SessionVerifier required")
        geometry = (wheel_radius_m, half_wheelbase_m, half_track_m,
                    pwm_per_rad_s)
        if (any(type(x) is not float or not math.isfinite(x) or x <= 0
                for x in geometry)
                or type(max_pwm) is not int
                or not 1 <= max_pwm <= HARD_PWM_CEILING):
            raise ValueError("invalid or unmeasured motor calibration")
        self._verifier = verifier
        self._geometry = geometry
        self._max_pwm = max_pwm
        self.state = "LATCHED"
        self.reason = "INITIAL_INHIBIT"
        self._last_tick_ns = None
        self._rearm_started_ns = None
        self._last_command_ns = None
        # Identity-bound, one-use handoff to the sole bridge serial writer.
        # There is no public constructor for a valid pending command.
        self._pending_decision = None
        self._pending_created_ns = None

    def _fault(self, reason, *, terminal=False):
        self.state = "TERMINAL" if terminal else "LATCHED"
        self.reason = reason
        self._rearm_started_ns = None
        self._last_command_ns = None
        self._pending_decision = None
        self._pending_created_ns = None
        return _zero(self.state, reason)

    @staticmethod
    def _healthy(health):
        return (type(health) is dict
                and frozenset(health) == HEALTH_KEYS
                and all(type(health[k]) is bool and health[k]
                        for k in HEALTH_KEYS))

    def _interlocks(self, *, health, estop_released, motor_rails_verified,
                    transport_healthy, clock_healthy, bridge_running,
                    single_controller):
        # Exact bool checks prevent a truthy object from becoming an
        # accidental motor authority.
        if transport_healthy is not True or bridge_running is not True:
            return "TRANSPORT_OR_BRIDGE_FAULT", True
        if clock_healthy is not True:
            return "CLOCK_FAULT", True
        if estop_released is not True or motor_rails_verified is not True:
            return "PHYSICAL_INTERLOCK_ABSENT", False
        if single_controller is not True:
            return "CONTROLLER_OWNERSHIP_UNVERIFIED", False
        if not self._healthy(health):
            return "SENSOR_HEALTH_UNVERIFIED", False
        return None, False

    def _time(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            return self._fault("INVALID_MONOTONIC_TIME", terminal=True)
        if self._last_tick_ns is not None:
            if now_ns <= self._last_tick_ns:
                return self._fault("MONOTONIC_TIME_REGRESSION", terminal=True)
            if (self.state in ("REARMING", "ARMED")
                    and now_ns - self._last_tick_ns > MAX_TICK_GAP_NS):
                return self._fault("AUTHORITY_LOOP_STALL")
        self._last_tick_ns = now_ns
        return None

    def rearm(self, *, grant, now_ns, health, estop_released=False,
              motor_rails_verified=False, transport_healthy=False,
              clock_healthy=False, bridge_running=False,
              single_controller=False):
        if self.state == "TERMINAL":
            return _zero(self.state, self.reason)
        self._pending_decision = None
        self._pending_created_ns = None
        time_fault = self._time(now_ns)
        if time_fault:
            return time_fault
        if self.state != "LATCHED":
            return self._fault("REARM_IN_WRONG_STATE")
        reason, terminal = self._interlocks(
            health=health, estop_released=estop_released,
            motor_rails_verified=motor_rails_verified,
            transport_healthy=transport_healthy,
            clock_healthy=clock_healthy, bridge_running=bridge_running,
            single_controller=single_controller)
        if reason:
            return self._fault(reason, terminal=terminal)
        if type(grant) is not RearmEnvelope:
            return self._fault("ABSENT_OPERATOR_GRANT")
        accepted, _ = self._verifier.verify_rearm(grant, now_ns=now_ns)
        if not accepted:
            return self._fault("INVALID_OR_REPLAYED_OPERATOR_GRANT")
        self.state = "REARMING"
        self.reason = "HEALTHY_REARM_DWELL"
        self._rearm_started_ns = now_ns
        return _zero(self.state, self.reason)

    def step(self, *, motion, now_ns, health, estop_released=False,
             motor_rails_verified=False, transport_healthy=False,
             clock_healthy=False, bridge_running=False,
             single_controller=False):
        # Every evaluation invalidates the previous serial handoff,
        # including evaluations returning ZERO or rejecting input.
        self._pending_decision = None
        self._pending_created_ns = None
        if self.state == "TERMINAL":
            return _zero(self.state, self.reason)
        time_fault = self._time(now_ns)
        if time_fault:
            return time_fault
        reason, terminal = self._interlocks(
            health=health, estop_released=estop_released,
            motor_rails_verified=motor_rails_verified,
            transport_healthy=transport_healthy,
            clock_healthy=clock_healthy, bridge_running=bridge_running,
            single_controller=single_controller)
        if reason:
            return self._fault(reason, terminal=terminal)
        if self.state == "LATCHED":
            return _zero(self.state, "EXPLICIT_OPERATOR_REARM_REQUIRED")
        if self.state == "REARMING":
            if now_ns - self._rearm_started_ns < REARM_DWELL_NS:
                if motion is not None:
                    return self._fault("MOTION_DURING_REARM")
                return _zero(self.state, "HEALTHY_REARM_DWELL")
            if motion is not None:
                return self._fault("MOTION_BEFORE_ARMED_TICK")
            self.state = "ARMED"
            self._last_command_ns = now_ns
            return _zero(self.state, "ARMED_WAITING_FRESH_COMMAND")
        if self.state != "ARMED":
            return self._fault("UNKNOWN_AUTHORITY_STATE", terminal=True)
        if now_ns - self._last_command_ns > MAX_COMMAND_AGE_NS:
            return self._fault("COMMAND_TIMEOUT")
        if motion is None:
            return _zero(self.state, "WAITING_FRESH_COMMAND")
        if type(motion) is not MotionEnvelope:
            return self._fault("INVALID_MOTION_FORMAT")
        accepted, _ = self._verifier.verify_motion(motion, now_ns=now_ns)
        if not accepted:
            return self._fault("INVALID_OR_REPLAYED_MOTION")
        if (abs(motion.vx) > VX_LIMIT_MPS
                or abs(motion.vy) > VY_LIMIT_MPS
                or abs(motion.wz) > WZ_LIMIT_RADPS):
            return self._fault("MOTION_EXCEEDS_SOFTWARE_LIMIT")
        wheels = mecanum_twist_to_pwm(
            motion.vx, motion.vy, motion.wz,
            *self._geometry, self._max_pwm)
        if wheels is None or len(wheels) != 4 or any(
                type(v) is not int or abs(v) > self._max_pwm for v in wheels):
            return self._fault("WHEEL_CONVERSION_INVALID")
        self._last_command_ns = now_ns
        packet = "set_wheels " + " ".join(str(v) for v in wheels)
        decision = MotorDecision(
            "ARMED", "FRESH_SIGNED_CONTROLLER_COMMAND",
            tuple(wheels), packet, any(v != 0 for v in wheels))
        self._pending_decision = decision
        self._pending_created_ns = now_ns
        return decision

    def consume_serial_payload(self, decision, *, now_ns):
        """Consume the exact fresh decision ONE time for bridge CDC I/O.

        No ROS message, signed envelope, tuple of wheel duties, or
        caller-constructed MotorDecision can authorize a serial write.
        The bridge must consume this inside its existing sole writer.
        """
        if (self.state != "ARMED"
                or type(decision) is not MotorDecision
                or decision is not self._pending_decision):
            return None
        issued = self._pending_created_ns
        # Consume before further validation to prevent replay and racing.
        self._pending_decision = None
        self._pending_created_ns = None
        if (type(now_ns) is not int or issued is None
                or not issued <= now_ns <= issued + 100_000_000):
            self._fault("EXPIRED_SERIAL_HANDOFF")
            return None
        if (decision.state != "ARMED"
                or decision.reason != "FRESH_SIGNED_CONTROLLER_COMMAND"
                or len(decision.wheel_pwm) != 4
                or any(type(v) is not int or abs(v) > self._max_pwm
                       for v in decision.wheel_pwm)
                or decision.nonzero_candidate != any(
                    v != 0 for v in decision.wheel_pwm)
                or decision.packet != "set_wheels " + " ".join(
                    str(v) for v in decision.wheel_pwm)):
            self._fault("INVALID_SERIAL_HANDOFF")
            return None
        return (decision.packet + "\n").encode("ascii")

    def heartbeat_allowed(self, *, now_ns):
        """Whether an MCU keepalive may extend an ACTIVE motion session.

        Heartbeats are not safe merely because the bridge process is alive:
        both the command and authority evaluation must remain fresh.
        Never extend stale motion using IMU/encoder telemetry alone.
        """
        return (
            self.state == "ARMED"
            and type(now_ns) is int
            and self._last_tick_ns is not None
            and self._last_command_ns is not None
            and 0 <= now_ns - self._last_tick_ns <= MAX_TICK_GAP_NS
            and 0 <= now_ns - self._last_command_ns < MAX_COMMAND_AGE_NS
        )

    def stop(self):
        """Permanently revoke this session (not an MCU STOP acknowledgement)."""
        return self._fault("EXPLICIT_SHUTDOWN", terminal=True)
