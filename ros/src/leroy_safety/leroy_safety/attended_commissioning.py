"""Pure, non-actuating attended wheel-commissioning policy.

This module performs no ROS, serial, GPIO or motor I/O.

Decisions are diagnostic proposals only, not motor authorization.
It cannot verify an operator, physical stop or motor power state.
Do not connect its outputs directly to a motor driver or CDC writer.
"""

from dataclasses import dataclass
import math


MAX_PWM = 64
MAX_UPDATE_GAP_NS = 150_000_000
ZERO = (0, 0, 0, 0)


@dataclass(frozen=True)
class CommissioningDecision:
    state: str
    reason: str
    proposed_pwm: tuple[int, int, int, int]
    stop_required: bool


class AttendedCommissioning:
    """Fail-closed controller requiring fresh, held operator input."""

    def __init__(self):
        self.state = "DISARMED"
        self.reason = "INITIAL_INHIBIT"
        self._last_ns = None

    def _result(self, reason, pwm=ZERO, stop=False):
        self.reason = reason
        return CommissioningDecision(
            self.state, reason, pwm, stop
        )

    def _fault(self, reason):
        self.state = "FAULT"
        return self._result(reason, stop=True)

    def _time_valid(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            return False
        if self._last_ns is not None and now_ns <= self._last_ns:
            return False
        return True

    def arm(self, *, now_ns, operator_confirmed=False):
        """Explicitly arm from DISARMED; never arm from FAULT."""
        if not self._time_valid(now_ns):
            return self._fault("INVALID_MONOTONIC_TIME")

        if self.state != "DISARMED":
            return self._fault("INVALID_ARM_STATE")

        self._last_ns = now_ns

        if operator_confirmed is not True:
            return self._result("OPERATOR_CONFIRMATION_REQUIRED")

        self.state = "ARMED"
        return self._result("ARMED_WAITING_FOR_HELD_INPUT")

    def tick(self, *, now_ns, deadman_held=False, requested_pwm=None):
        """Evaluate one fresh operator sample.

        Any released, missing, stale or malformed input stops the
        session. A FAULT cannot be reset through tick().
        """
        if self.state == "FAULT":
            return self._result(self.reason, stop=True)

        if not self._time_valid(now_ns):
            return self._fault("INVALID_MONOTONIC_TIME")

        if self.state != "ARMED":
            self._last_ns = now_ns
            return self._result("DISARMED", stop=True)

        if now_ns - self._last_ns > MAX_UPDATE_GAP_NS:
            return self._fault("OPERATOR_UPDATE_TIMEOUT")

        self._last_ns = now_ns

        if deadman_held is not True:
            self.state = "DISARMED"
            return self._result("DEADMAN_RELEASED", stop=True)

        if type(requested_pwm) is not tuple or len(requested_pwm) != 4:
            return self._fault("MISSING_OR_INVALID_WHEEL_REQUEST")

        if any(
            type(value) is not int or abs(value) > MAX_PWM
            for value in requested_pwm
        ):
            return self._fault("WHEEL_REQUEST_EXCEEDS_LIMIT")

        return self._result(
            "FRESH_HELD_OPERATOR_PROPOSAL",
            pwm=requested_pwm,
        )

    def disarm(self):
        """Request STOP even when already disarmed."""
        if self.state != "FAULT":
            self.state = "DISARMED"
        return self._result("EXPLICIT_STOP", stop=True)
