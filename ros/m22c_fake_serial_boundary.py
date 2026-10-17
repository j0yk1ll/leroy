#!/usr/bin/env python3
"""M22C standalone software-authority -> FAKE MCU serial command adapter.

Pure Python: no ROS, serial, devices, containers, network, subprocess or motors.
Uses the existing leroy_bridge.motor_pwm math, NOT the real bridge node.
The existing production bridge still directly subscribes to /cmd_vel; this
model DOES NOT close that deployment bypass or authorize physical motion.
"""
from dataclasses import dataclass
import re

from leroy_bridge.motor_pwm import mecanum_twist_to_pwm
from m22a_diagnostic_authority import DiagnosticAuthority, ZERO

# Values mirror the provisional bridge YAML only for synthetic regression.
# They are NOT physically calibrated motion, traction, or stopping limits.
SYNTHETIC_WHEEL_PARAMS = {
    "wheel_radius_m": 0.020,
    "half_wheelbase_m": 0.180,
    "half_track_m": 0.275,
    "wheel_velocity_to_pwm_gain": 12.75,
    "max_pwm": 64,
}
ZERO_PWM = (0, 0, 0, 0)
_ALLOWED = re.compile(rb"set_wheels (-?\d+) (-?\d+) (-?\d+) (-?\d+)\n\Z")


class FakeSerialTransport:
    """In-memory fake CDC receiver; NEVER opens a port or sends actual bytes."""

    def __init__(self):
        self.writes = []
        self.fail_next = False

    def write(self, packet):
        if type(packet) is not bytes:
            raise ValueError("fake CDC expects bytes")
        match = _ALLOWED.fullmatch(packet)
        if match is None:
            raise ValueError("fake CDC only accepts four signed wheel duties")
        pwm = tuple(int(value) for value in match.groups())
        if any(abs(value) > 255 for value in pwm):
            raise ValueError("fake CDC duty outside protocol range")
        if self.fail_next:
            self.fail_next = False
            raise OSError("injected fake CDC write failure")
        self.writes.append(packet)
        return len(packet)


@dataclass(frozen=True)
class FakeTransportReport:
    authority_reason: str
    authority_state: str
    emitted_pwm: tuple
    fake_write_completed: bool
    fake_transport_fault_latched: bool
    physical_stop_acknowledgement_validated: bool = False


class SyntheticBridgeBoundary:
    """One checked synthetic authority decision -> one fake serial write.

    The component contains its own M22A authority state, which prevents a
    caller from simply handing it an arbitrary "authorized" Decision.
    This is NOT a production trust boundary: callers can still access the
    public test transport, and no real bridge source code is changed.
    """

    def __init__(self, fake_transport, *, pwm_parameters=None):
        if type(fake_transport) is not FakeSerialTransport:
            raise TypeError("M22C accepts only an in-memory fake transport")
        self._fake_transport = fake_transport
        self._core = DiagnosticAuthority()
        self._pwm_params = dict(
            SYNTHETIC_WHEEL_PARAMS if pwm_parameters is None else pwm_parameters)
        self._transport_fault = False

    @property
    def state(self):
        return self._core.state

    @property
    def transport_fault(self):
        return self._transport_fault

    def evaluate_and_write(self, *, now_ns, inputs, publishers, proposal,
                           operator_rearm=False):
        if self._transport_fault:
            # No silent recovery or more writes following a transport error.
            return FakeTransportReport(
                "BLOCKED_FAKE_TRANSPORT_LATCHED", "LATCHED", ZERO_PWM,
                False, True)

        decision = self._core.evaluate(
            now_ns=now_ns, inputs=inputs, publishers=publishers,
            proposal=proposal, operator_rearm=operator_rearm)

        pwm = ZERO_PWM
        if decision.output != ZERO:
            converted = mecanum_twist_to_pwm(
                vx=decision.output[0],
                vy=decision.output[1],
                wz=decision.output[2],
                **self._pwm_params)
            if (converted is None or len(converted) != 4
                    or any(type(value) is not int
                           for value in converted)):
                decision = self._core._trip("BLOCKED_PWM_CONVERSION")
            else:
                pwm = tuple(converted)
        packet = ("set_wheels {} {} {} {}\n".format(*pwm)).encode("ascii")
        try:
            self._fake_transport.write(packet)
        except (OSError, ValueError, TypeError):
            self._transport_fault = True
            decision = self._core._trip("BLOCKED_FAKE_TRANSPORT_FAILURE")
            return FakeTransportReport(
                decision.reason, decision.state, ZERO_PWM, False, True)
        return FakeTransportReport(
            decision.reason, decision.state, pwm, True, False)
