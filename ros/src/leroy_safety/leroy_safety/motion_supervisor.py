"""Pure, fail-closed mode and source authority. Never verifies hardware itself."""
from dataclasses import dataclass
from enum import Enum
import math


class Mode(str, Enum):
    DISARMED = "DISARMED"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ModeDecision:
    mode: Mode
    allowed: bool
    reason: str


class MotionSupervisor:
    """Trusted caller must supply *independently verified* local safety evidence.

    Neither ROS messages nor caller claims are proof of physical interlocks.
    This class does not arm the MCU, provision keys or grant motor power.
    """
    def __init__(self):
        self.mode = Mode.DISARMED
        self.reason = "INITIAL_DISARM"
        self._last_ns = None

    def _time_ok(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            return False
        if self._last_ns is not None and now_ns <= self._last_ns:
            return False
        self._last_ns = now_ns
        return True

    def disarm(self):
        if self.mode != Mode.FAULT:
            self.mode = Mode.DISARMED
            self.reason = "EXPLICIT_DISARM"
        return ModeDecision(self.mode, False, self.reason)

    def fault(self, reason="SAFETY_FAULT"):
        self.mode = Mode.FAULT
        self.reason = str(reason)[:128] or "SAFETY_FAULT"
        return ModeDecision(self.mode, False, self.reason)

    def reset_fault(self, *, local_operator_confirmed=False):
        if self.mode != Mode.FAULT or local_operator_confirmed is not True:
            return ModeDecision(self.mode, False, "LOCAL_RESET_REQUIRED")
        self.mode = Mode.DISARMED
        self.reason = "FAULT_RESET_DISARMED"
        self._last_ns = None
        return ModeDecision(self.mode, False, self.reason)

    def select(self, mode, *, now_ns, local_operator_confirmed=False,
               physical_stop_verified=False, environmental_clearance_verified=False,
               localization_verified=False):
        if not self._time_ok(now_ns):
            return self.fault("INVALID_OR_REGRESSED_CLOCK")
        if self.mode == Mode.FAULT:
            return ModeDecision(self.mode, False, "LATCHED_FAULT")
        if mode not in (Mode.MANUAL, Mode.AUTONOMOUS):
            return ModeDecision(self.mode, False, "MODE_NOT_PERMITTED")
        if self.mode != Mode.DISARMED:
            return ModeDecision(self.mode, False, "DISARM_FIRST")
        if (local_operator_confirmed is not True or
                physical_stop_verified is not True or
                environmental_clearance_verified is not True or
                (mode == Mode.AUTONOMOUS and localization_verified is not True)):
            return ModeDecision(self.mode, False, "PREREQUISITES_UNVERIFIED")
        self.mode = mode
        self.reason = "MODE_SELECTED_NOT_MOTOR_ARMED"
        return ModeDecision(self.mode, False, self.reason)

    def evaluate(self, *, now_ns, source, prerequisites_verified=False):
        if not self._time_ok(now_ns):
            return self.fault("INVALID_OR_REGRESSED_CLOCK")
        if self.mode == Mode.FAULT:
            return ModeDecision(self.mode, False, "LATCHED_FAULT")
        if self.mode == Mode.DISARMED:
            return ModeDecision(self.mode, False, "DISARMED")
        expected = "nav2" if self.mode == Mode.AUTONOMOUS else "teleop"
        if source != expected:
            return self.fault("UNAUTHORIZED_CONTROL_SOURCE")
        if prerequisites_verified is not True:
            return self.fault("PREREQUISITES_LOST")
        return ModeDecision(self.mode, True, "POLICY_ONLY_BRIDGE_MUST_VERIFY")
