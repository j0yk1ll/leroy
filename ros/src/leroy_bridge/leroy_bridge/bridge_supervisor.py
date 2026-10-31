"""Bridge-owned fail-closed signed command supervisor, NO actuation.

Evaluates signed command provenance without granting motor authority.
It produces only status booleans and reasons, never Twist, duty or CDC
commands. The actual bridge may call this ONLY through the separately
injected _supervise_signed_diagnostic method; production startup does not
construct an auditor, provision keys, subscribe to signed ROS topics, or
activate nonzero wheel commands.

Health/source evidence passed by tests is ADVISORY; it is not trusted
source authentication or real sensor validation. No physical safety
claim or operator-identity mechanism is implemented.
"""
from dataclasses import dataclass
import math

from leroy_bridge.authorized_session import MotionEnvelope, OWNER
from leroy_bridge.bridge_signed_audit import BridgeSignedAudit

RECOVERY_NS = 1_000_000_000
MAX_EVALUATION_GAP_NS = 200_000_000
SYNTHETIC_VX_LIMIT = 0.15
SYNTHETIC_VY_LIMIT = 0.15
SYNTHETIC_WZ_LIMIT = 0.30
REQUIRED_HEALTH = (
    "odom_fresh", "depth_fresh", "tf_fresh",
    "costmap_valid", "localization_valid",
)


@dataclass(frozen=True)
class DiagnosticAssessment:
    state: str
    reason: str
    diagnostic_permission: bool
    physical_motion_authorized: bool = False
    diagnostic_only: bool = True


class BridgeSupervisor:
    """One signed proposal per evaluation, never cache any velocity.

    An operator grant is an edge that STARTS a 1-second healthy dwell.
    No motion command is generated even in state ARMED. A fault is
    terminal until another independent valid grant + dwell, except a
    terminal hardware transport failure, which is bound to the bridge
    event and cannot be cleared without bridge reconstruction.
    """

    def __init__(self, audit):
        if type(audit) is not BridgeSignedAudit:
            raise TypeError("bridge supervisor requires exact signed audit audit type")
        self._audit = audit
        self.state = "LATCHED"
        self.last_now_ns = None
        self.healthy_since_ns = None
        self.last_fault = "INITIAL_INHIBIT"

    def _trip(self, reason):
        self.state = "LATCHED"
        self.healthy_since_ns = None
        self.last_fault = reason
        return DiagnosticAssessment(self.state, reason, False)

    @staticmethod
    def _healthy(health):
        return (
            type(health) is dict
            and set(health) == set(REQUIRED_HEALTH)
            and all(type(health[name]) is bool and health[name]
                    for name in REQUIRED_HEALTH)
        )

    def evaluate(self, *, motion, rearm, now_ns, health, publishers,
                 transport_healthy, clock_healthy, bridge_running):
        if (type(now_ns) is not int or now_ns <= 0
                or (self.last_now_ns is not None
                    and now_ns <= self.last_now_ns)):
            return self._trip("BLOCKED_MONOTONIC_CLOCK")
        if (self.state in ("REARMING", "ARMED")
                and self.last_now_ns is not None
                and now_ns - self.last_now_ns > MAX_EVALUATION_GAP_NS):
            self.last_now_ns = now_ns
            return self._trip("BLOCKED_SUPERVISOR_STALL")
        self.last_now_ns = now_ns

        if transport_healthy is not True:
            return self._trip("BLOCKED_LOCAL_TRANSPORT_FAULT")
        if clock_healthy is not True:
            return self._trip("BLOCKED_LOCAL_CLOCK_FAULT")
        if bridge_running is not True:
            return self._trip("BLOCKED_BRIDGE_STOP_EVENT")
        if not self._healthy(health):
            return self._trip("BLOCKED_SENSOR_HEALTH")
        if (type(publishers) is not tuple
                or publishers != (OWNER,)):
            return self._trip("BLOCKED_SYNTHETIC_SOURCE_CONFLICT")

        assessment = self._audit.inspect(
            motion=motion, rearm=rearm, now_ns=now_ns)
        if not assessment.command_authenticated:
            return self._trip(assessment.command_reason)
        if rearm is not None and not assessment.operator_grant_authenticated:
            return self._trip(assessment.rearm_reason)

        # The signing contract already validates finite float fields.
        # Guard magnitude independently, even if an HMAC is valid.
        if type(motion) is not MotionEnvelope or (
                abs(motion.vx) > SYNTHETIC_VX_LIMIT
                or abs(motion.vy) > SYNTHETIC_VY_LIMIT
                or abs(motion.wz) > SYNTHETIC_WZ_LIMIT
                or not all(math.isfinite(value) for value in (
                    motion.vx, motion.vy, motion.wz))):
            return self._trip("BLOCKED_SYNTHETIC_LIMITS")

        if self.state == "LATCHED":
            if rearm is None:
                return DiagnosticAssessment(
                    self.state, "BLOCKED_EXPLICIT_REARM_REQUIRED", False)
            self.state = "REARMING"
            self.healthy_since_ns = now_ns
            return DiagnosticAssessment(
                self.state, "BLOCKED_HEALTHY_REARM_DWELL", False)

        if rearm is not None:
            return self._trip("BLOCKED_UNEXPECTED_REARM")

        if self.state == "REARMING":
            if now_ns - self.healthy_since_ns < RECOVERY_NS:
                return DiagnosticAssessment(
                    self.state, "BLOCKED_HEALTHY_REARM_DWELL", False)
            self.state = "ARMED"

        if self.state != "ARMED":
            return self._trip("BLOCKED_INVALID_SUPERVISOR_STATE")
        return DiagnosticAssessment(
            self.state, "DIAGNOSTIC_ONLY_AUTHORIZED_SYNTHETIC", True)
