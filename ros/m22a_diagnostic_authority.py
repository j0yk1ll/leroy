"""M22A: pure-Python, diagnostic-only command authority state machine.

This module cannot command hardware: it imports no ROS, serial, motors or
bridge. Its limits are SYNTHETIC EXAMPLES, NOT validated robot limits.
Its ROS graph source labels are advisory, NOT authenticated identities.
It is not wired to /cmd_vel or any motor interface.
"""
from dataclasses import dataclass
import math

ZERO = (0.0, 0.0, 0.0)
MAX_VX = 0.15  # illustrative M14 fixture values only
MAX_VY = 0.15
MAX_WZ = 0.30
MAX_AGE_NS = 250_000_000
RECOVERY_NS = 1_000_000_000
MAX_EVALUATION_GAP_NS = 200_000_000  # synthetic 10Hz gate deadline
EXPECTED_OWNER = "controller_server"


@dataclass(frozen=True)
class Inputs:
    odom_fresh: bool
    depth_fresh: bool
    tf_fresh: bool
    costmap_valid: bool
    localization_valid: bool


@dataclass(frozen=True)
class Proposal:
    owner: str
    sequence: int
    generated_mono_ns: int
    vx: float
    vy: float
    wz: float
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0


@dataclass(frozen=True)
class Decision:
    output: tuple
    reason: str
    state: str


class DiagnosticAuthority:
    """Explicitly rearmed authority for SYNTHETIC command proposals only.

    All time values refer to ONE common host monotonic clock. The caller
    must perform independent validation of actual ROS publishers and real
    sensor input acquisition time; neither is established by this class.
    Every evaluation consumes ONE new proposal; it never caches a Twist
    that could be reused after a publisher stalls.
    """

    def __init__(self):
        self.state = "LATCHED"
        self.last_now_ns = None
        self.last_sequence = None
        self.last_generated_ns = None
        self.healthy_since_ns = None
        self.last_fault = "INITIAL_INHIBIT"

    def _trip(self, reason):
        self.state = "LATCHED"
        self.healthy_since_ns = None
        self.last_fault = reason
        return Decision(ZERO, reason, self.state)

    @staticmethod
    def _valid_inputs(inputs):
        return (isinstance(inputs, Inputs)
                and all(type(value) is bool and value for value in (
                    inputs.odom_fresh, inputs.depth_fresh, inputs.tf_fresh,
                    inputs.costmap_valid, inputs.localization_valid)))

    def evaluate(self, *, now_ns, inputs, publishers, proposal,
                 operator_rearm=False):
        """Return an exact-zero or diagnostic-only bounded decision.

        No software API here authenticates an operator or publisher. A
        future integration must enforce identity, operator intent, timing,
        and command ownership outside this pure model.
        """
        if type(now_ns) is not int or now_ns <= 0 or (
                self.last_now_ns is not None and now_ns <= self.last_now_ns):
            return self._trip("BLOCKED_MONOTONIC_CLOCK")
        if (self.state in ("REARMING", "ARMED")
                and self.last_now_ns is not None
                and now_ns - self.last_now_ns > MAX_EVALUATION_GAP_NS):
            self.last_now_ns = now_ns
            return self._trip("BLOCKED_EVALUATOR_STALL")
        self.last_now_ns = now_ns

        if type(operator_rearm) is not bool:
            return self._trip("BLOCKED_INVALID_REARM")
        if not isinstance(publishers, (tuple, list)) or (
                tuple(publishers) != (EXPECTED_OWNER,)):
            return self._trip("BLOCKED_COMMAND_OWNERSHIP")
        if not self._valid_inputs(inputs):
            return self._trip("BLOCKED_SENSOR_HEALTH")
        if not isinstance(proposal, Proposal) or (
                type(proposal.owner) is not str
                or proposal.owner != EXPECTED_OWNER
                or type(proposal.sequence) is not int
                or not 0 <= proposal.sequence < (1 << 64)
                or type(proposal.generated_mono_ns) is not int
                or proposal.generated_mono_ns <= 0):
            return self._trip("BLOCKED_COMMAND_FORMAT")
        values = (proposal.vx, proposal.vy, proposal.wz,
                  proposal.vz, proposal.wx, proposal.wy)
        if any(type(value) not in (int, float) or not math.isfinite(value)
               for value in values):
            return self._trip("BLOCKED_NONFINITE_COMMAND")
        if (abs(proposal.vx) > MAX_VX or abs(proposal.vy) > MAX_VY
                or abs(proposal.wz) > MAX_WZ
                or any(value != 0 for value in (
                    proposal.vz, proposal.wx, proposal.wy))):
            return self._trip("BLOCKED_COMMAND_LIMITS")

        age_ns = now_ns - proposal.generated_mono_ns
        if not 0 <= age_ns <= MAX_AGE_NS:
            return self._trip("BLOCKED_STALE_OR_FUTURE_COMMAND")
        if (self.last_sequence is not None
                and proposal.sequence <= self.last_sequence):
            return self._trip("BLOCKED_REPLAY_SEQUENCE")
        if (self.last_generated_ns is not None
                and proposal.generated_mono_ns <= self.last_generated_ns):
            return self._trip("BLOCKED_REPLAY_GENERATION_TIME")

        # Advance anti-replay counters even when the gate is latched, so
        # observing another valid message cannot reuse an older proposal.
        self.last_sequence = proposal.sequence
        self.last_generated_ns = proposal.generated_mono_ns

        if self.state == "LATCHED":
            if not operator_rearm:
                return Decision(ZERO, "BLOCKED_MANUAL_REARM_REQUIRED",
                                self.state)
            self.state = "REARMING"
            self.healthy_since_ns = now_ns
            return Decision(ZERO, "BLOCKED_REARM_DWELL", self.state)

        if operator_rearm:
            # Rearm is a rising-edge control action, never a continuous
            # implicit authorization or a shortcut through the dwell.
            return self._trip("BLOCKED_UNEXPECTED_REARM")
        if self.state == "REARMING":
            if now_ns - self.healthy_since_ns < RECOVERY_NS:
                return Decision(ZERO, "BLOCKED_REARM_DWELL", self.state)
            self.state = "ARMED"
        if self.state != "ARMED":
            return self._trip("BLOCKED_INVALID_STATE")
        return Decision((float(proposal.vx), float(proposal.vy),
                         float(proposal.wz)),
                        "FORWARD_SYNTHETIC_DIAGNOSTIC_ONLY", self.state)
