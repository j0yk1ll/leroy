"""M22E: signed session verifier -> existing PWM math -> fake serial ONLY.

This test fixture MUST NOT be imported into the hardware bridge or used
to send a real serial command. Production M22D remains zero-wheel-only.
"""
from dataclasses import dataclass

from leroy_bridge.authorized_session import SessionVerifier
from leroy_bridge.motor_pwm import mecanum_twist_to_pwm
from m22a_diagnostic_authority import DiagnosticAuthority, Inputs, Proposal, ZERO
from m22c_fake_serial_boundary import (
    FakeSerialTransport, SYNTHETIC_WHEEL_PARAMS, ZERO_PWM,
)


@dataclass(frozen=True)
class FakeCycle:
    state: str
    reason: str
    pwm: tuple
    fake_write_completed: bool
    fake_transport_fault_latched: bool
    physical_stop_acknowledgement_validated: bool = False


class FakeSignedIngress:
    """Authorize synthetic motion while M22D's real writer remains locked."""

    def __init__(self, *, transport, session):
        if type(transport) is not FakeSerialTransport:
            raise TypeError("only M22C in-memory fake CDC is permitted")
        if type(session) is not SessionVerifier:
            raise TypeError("M22E requires session-bound verifier")
        self._transport = transport
        self._verifier = session
        self._core = DiagnosticAuthority()
        self._transport_latched = False

    @property
    def state(self):
        return "LATCHED" if self._transport_latched else self._core.state

    @property
    def transport_latched(self):
        return self._transport_latched

    def cycle(self, *, now_ns, health, publishers, command=None, rearm=None):
        if self._transport_latched:
            return FakeCycle("LATCHED", "BLOCKED_FAKE_CDC_LATCHED",
                             ZERO_PWM, False, True)

        good_command, command_reason = self._verifier.verify_motion(
            command, now_ns=now_ns)
        valid_rearm = False
        rearm_reason = None
        if rearm is not None:
            valid_rearm, rearm_reason = self._verifier.verify_rearm(
                rearm, now_ns=now_ns)

        proposal = None
        if good_command:
            proposal = Proposal(
                owner=command.owner, sequence=command.sequence,
                generated_mono_ns=command.generated_ns,
                vx=command.vx, vy=command.vy, wz=command.wz)
        decision = self._core.evaluate(
            now_ns=now_ns, inputs=health, publishers=publishers,
            proposal=proposal, operator_rearm=valid_rearm)

        # Invalid signed envelopes cause a latched fault even when a
        # previous proposal was accepted and the sensors remain healthy.
        if not good_command:
            decision = self._core._trip(command_reason)
        if rearm is not None and not valid_rearm:
            decision = self._core._trip(rearm_reason)

        pwm = ZERO_PWM
        if decision.output != ZERO:
            try:
                converted = mecanum_twist_to_pwm(
                    vx=decision.output[0], vy=decision.output[1],
                    wz=decision.output[2], **SYNTHETIC_WHEEL_PARAMS)
                if (converted is None or len(converted) != 4
                        or any(type(x) is not int for x in converted)):
                    raise ValueError("fake conversion rejected")
                pwm = tuple(converted)
            except (ValueError, TypeError, OverflowError, ZeroDivisionError):
                decision = self._core._trip("BLOCKED_FAKE_PWM_CONVERSION")
                pwm = ZERO_PWM

        fake_packet = ("set_wheels {} {} {} {}\n".format(*pwm)).encode("ascii")
        try:
            self._transport.write(fake_packet)
        except (OSError, ValueError, TypeError):
            self._transport_latched = True
            decision = self._core._trip("BLOCKED_FAKE_CDC_WRITE_FAILURE")
            return FakeCycle("LATCHED", decision.reason, ZERO_PWM,
                             False, True)
        return FakeCycle(decision.state, decision.reason, pwm, True, False)
