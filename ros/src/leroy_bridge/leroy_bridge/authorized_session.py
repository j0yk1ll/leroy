"""signed session: signed, session-bound command envelopes (pure Python, NO motor I/O).

This is a candidate protocol for offline tests. HMAC only proves possession
of a secret *if* key custody is independently established. This module
does not distribute credentials, authenticate an operator, attach to ROS,
unlock the command allowlist production CDC allowlist, or authorize physical motion.
A fresh verifier has a new random session ID to reject prior-session replay.
"""
from dataclasses import dataclass
import hashlib
import hmac
import json
import math
import re
import secrets

MAX_AGE_NS = 250_000_000
OWNER = "controller_server"
_HEX_MAC = re.compile(r"[0-9a-f]{64}\Z")


@dataclass(frozen=True)
class MotionEnvelope:
    session_id: str
    owner: str
    sequence: int
    generated_ns: int
    expires_ns: int
    vx: float
    vy: float
    wz: float
    mac: str


@dataclass(frozen=True)
class RearmEnvelope:
    session_id: str
    counter: int
    generated_ns: int
    expires_ns: int
    mac: str


def _command_fields(item):
    return {
        "domain": "leroy-command-v1",
        "session_id": item.session_id,
        "owner": item.owner,
        "sequence": item.sequence,
        "generated_ns": item.generated_ns,
        "expires_ns": item.expires_ns,
        "vx": item.vx,
        "vy": item.vy,
        "wz": item.wz,
    }


def _rearm_fields(item):
    return {
        "domain": "leroy-rearm-v1",
        "session_id": item.session_id,
        "counter": item.counter,
        "generated_ns": item.generated_ns,
        "expires_ns": item.expires_ns,
    }


def _mac(key, fields):
    encoded = json.dumps(
        fields, sort_keys=True, separators=(",", ":"),
        allow_nan=False, ensure_ascii=True).encode("ascii")
    return hmac.new(key, encoded, hashlib.sha256).hexdigest()


def sign_motion(*, key, session_id, sequence, generated_ns, expires_ns,
                     vx=0.04, vy=0.0, wz=0.07, owner=OWNER):
    """Sign one finite current-session command; never log the key."""
    packet = MotionEnvelope(session_id, owner, sequence, generated_ns,
                            expires_ns, vx, vy, wz, "")
    return MotionEnvelope(**{**packet.__dict__,
                             "mac": _mac(key, _command_fields(packet))})


def sign_rearm(*, key, session_id, counter, generated_ns, expires_ns):
    """Sign explicit operator request. Caller must authenticate locally."""
    packet = RearmEnvelope(session_id, counter, generated_ns, expires_ns, "")
    return RearmEnvelope(**{**packet.__dict__,
                            "mac": _mac(key, _rearm_fields(packet))})


# Preserve the existing offline fixture API without making fake test
# signers the production entry point.
sign_test_motion = sign_motion
sign_test_rearm = sign_rearm


def _valid_window(generated_ns, expires_ns, now_ns):
    return (type(now_ns) is int and now_ns > 0
            and type(generated_ns) is int and generated_ns > 0
            and type(expires_ns) is int
            and generated_ns < expires_ns
            and expires_ns - generated_ns <= MAX_AGE_NS
            and generated_ns <= now_ns <= expires_ns)


class SessionVerifier:
    """Offline candidate session verifier; state never resets in place.

    A caller holds distinct 32-byte motion and operator HMAC secrets.
    In real integration an authenticated, protected provisioning and
    restart/rearm lifecycle are REQUIRED; not provided here.
    """

    def __init__(self, *, motion_key, operator_key):
        if (type(motion_key) is not bytes or len(motion_key) != 32
                or type(operator_key) is not bytes
                or len(operator_key) != 32 or motion_key == operator_key):
            raise ValueError("distinct 32-byte fake test secrets required")
        self.session_id = secrets.token_hex(16)
        self._motion_key = motion_key
        self._operator_key = operator_key
        self._last_sequence = -1
        self._last_command_ns = -1
        self._last_rearm_counter = -1
        self._last_rearm_ns = -1

    def verify_motion(self, item, *, now_ns):
        if type(item) is not MotionEnvelope:
            return False, "BLOCKED_ABSENT_OR_BAD_ENVELOPE"
        if (type(item.session_id) is not str
                or item.session_id != self.session_id
                or type(item.owner) is not str or item.owner != OWNER):
            return False, "BLOCKED_SESSION_OR_OWNER"
        if (type(item.sequence) is not int
                or not 0 <= item.sequence < (1 << 64)
                or any(type(x) is not float or not math.isfinite(x)
                       for x in (item.vx, item.vy, item.wz))
                or type(item.mac) is not str
                or not _HEX_MAC.fullmatch(item.mac)):
            return False, "BLOCKED_MOTION_FORMAT"
        if not _valid_window(item.generated_ns, item.expires_ns, now_ns):
            return False, "BLOCKED_MOTION_FRESHNESS"
        if (item.sequence <= self._last_sequence
                or item.generated_ns <= self._last_command_ns):
            return False, "BLOCKED_MOTION_REPLAY"
        if not hmac.compare_digest(
                _mac(self._motion_key, _command_fields(item)), item.mac):
            return False, "BLOCKED_MOTION_AUTH"
        self._last_sequence = item.sequence
        self._last_command_ns = item.generated_ns
        return True, "MOTION_ENVELOPE_VERIFIED_SYNTHETIC"

    def verify_rearm(self, item, *, now_ns):
        if type(item) is not RearmEnvelope:
            return False, "BLOCKED_REARM_FORMAT"
        if (type(item.session_id) is not str
                or item.session_id != self.session_id
                or type(item.counter) is not int
                or not 0 <= item.counter < (1 << 64)
                or type(item.mac) is not str
                or not _HEX_MAC.fullmatch(item.mac)):
            return False, "BLOCKED_REARM_FORMAT"
        if not _valid_window(item.generated_ns, item.expires_ns, now_ns):
            return False, "BLOCKED_REARM_FRESHNESS"
        if (item.counter <= self._last_rearm_counter
                or item.generated_ns <= self._last_rearm_ns):
            return False, "BLOCKED_REARM_REPLAY"
        if not hmac.compare_digest(
                _mac(self._operator_key, _rearm_fields(item)), item.mac):
            return False, "BLOCKED_REARM_AUTH"
        self._last_rearm_counter = item.counter
        self._last_rearm_ns = item.generated_ns
        return True, "REARM_ENVELOPE_VERIFIED_SYNTHETIC"
