"""Non-actuating command admission; bridge independently enforces motor safety."""
from dataclasses import dataclass
import math

from leroy_safety.motion_supervisor import MotionSupervisor
from leroy_safety.motor_authority import VX_LIMIT_MPS, VY_LIMIT_MPS, WZ_LIMIT_RADPS


@dataclass(frozen=True)
class MotionRequest:
    source: str
    sequence: int
    received_ns: int
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class GatewayDecision:
    accepted: bool
    reason: str
    velocity: tuple[float, float, float]


class CommandGateway:
    """Admission does not mean authorization to actuate.

    Caller must keep trusted inputs distinct from message-supplied fields.
    """
    def __init__(self, supervisor: MotionSupervisor, *, max_age_ns=100_000_000):
        if type(supervisor) is not MotionSupervisor:
            raise TypeError("MotionSupervisor required")
        if type(max_age_ns) is not int or not 0 < max_age_ns <= 250_000_000:
            raise ValueError("invalid freshness window")
        self.supervisor = supervisor
        self.max_age_ns = max_age_ns
        self._last_sequence = {}

    def admit(self, request, *, now_ns, prerequisites_verified=False,
              single_publisher_verified=False):
        zero = (0.0, 0.0, 0.0)
        if type(now_ns) is not int or now_ns <= 0:
            self.supervisor.fault("INVALID_CLOCK")
            return GatewayDecision(False, "INVALID_CLOCK", zero)
        if type(request) is not MotionRequest:
            self.supervisor.fault("INVALID_REQUEST")
            return GatewayDecision(False, "INVALID_REQUEST", zero)
        decision = self.supervisor.evaluate(
            now_ns=now_ns, source=request.source,
            prerequisites_verified=prerequisites_verified)
        if not decision.allowed:
            return GatewayDecision(False, decision.reason, zero)
        if single_publisher_verified is not True:
            self.supervisor.fault("SOURCE_NOT_EXCLUSIVE")
            return GatewayDecision(False, "SOURCE_NOT_EXCLUSIVE", zero)
        if (type(request.sequence) is not int or request.sequence < 0 or
                type(request.received_ns) is not int or
                not 0 <= now_ns - request.received_ns <= self.max_age_ns or
                request.sequence <= self._last_sequence.get(request.source, -1)):
            self.supervisor.fault("STALE_OR_REPLAYED_REQUEST")
            return GatewayDecision(False, "STALE_OR_REPLAYED_REQUEST", zero)
        values = (request.vx, request.vy, request.wz)
        if (any(type(x) is not float or not math.isfinite(x) for x in values)
                or any(abs(x) > limit for x, limit in
                       zip(values, (VX_LIMIT_MPS, VY_LIMIT_MPS, WZ_LIMIT_RADPS))):
            self.supervisor.fault("INVALID_OR_UNBOUNDED_VELOCITY")
            return GatewayDecision(False, "INVALID_OR_UNBOUNDED_VELOCITY", zero)
        self._last_sequence[request.source] = request.sequence
        return GatewayDecision(True, "ADMITTED_NOT_MOTOR_AUTHORIZED", values)
