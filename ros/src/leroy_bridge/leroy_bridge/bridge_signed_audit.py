"""M22J bridge-owned signed-candidate AUDIT, never a motor command path.

This package module parses nothing from ROS and emits NO CDC bytes, duty
values or actionable motion output. It checks M22E signed command and
operator-rearm envelopes using an explicitly injected SessionVerifier.
Verification is diagnostic only and NEVER authorizes physical actuation.

No credential provisioning, ROS publisher authentication, real operator
approval or hardware protections are implemented here.
"""
from dataclasses import dataclass

from leroy_bridge.authorized_session import SessionVerifier


@dataclass(frozen=True)
class SignedAuditResult:
    command_authenticated: bool
    operator_grant_authenticated: bool
    command_reason: str
    rearm_reason: str
    physical_motion_authorized: bool = False
    diagnostic_only: bool = True


class BridgeSignedAudit:
    """Bridge-facing test seam with exactly ZERO motor output functions.

    The caller provides a trusted verifier explicitly; production startup
    never instantiates a verifier and no ROS topic routes to this audit.
    SessionVerifier retains its own monotonic replay counters.
    """

    def __init__(self, verifier):
        if type(verifier) is not SessionVerifier:
            raise TypeError("M22J audit requires exact SessionVerifier type")
        self._verifier = verifier
        self._inspections = 0

    @property
    def inspections(self):
        return self._inspections

    def inspect(self, *, motion, rearm=None, now_ns):
        good, reason = self._verifier.verify_motion(
            motion, now_ns=now_ns)
        if rearm is None:
            rearm_ok, rearm_reason = False, "REARM_NOT_SUBMITTED"
        else:
            rearm_ok, rearm_reason = self._verifier.verify_rearm(
                rearm, now_ns=now_ns)
        self._inspections += 1
        return SignedAuditResult(
            command_authenticated=good,
            operator_grant_authenticated=rearm_ok,
            command_reason=reason,
            rearm_reason=rearm_reason,
        )
