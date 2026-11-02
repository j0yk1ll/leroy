"""Strict signed ROS String wire envelopes and local signing-key custody.

The ROS topic is only transport: possession of a current HMAC and a
fresh bridge session is checked separately by MotorAuthority. Never
accept caller-provided health booleans, keys, or physical arm flags.
"""
from dataclasses import fields
import json
import os
import stat

from leroy_mcu.authorized_session import MotionEnvelope, RearmEnvelope

MAX_JSON_BYTES = 2048


class MotorInputRejected(ValueError):
    pass


def _strict_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise MotorInputRejected("duplicate JSON field")
        result[key] = value
    return result


def _bad_constant(value):
    raise MotorInputRejected("non-finite JSON token")


def parse_signed_envelope(text, envelope_type):
    """Parse exact flat JSON schema; verifier checks HMAC, time and replay."""
    if envelope_type not in (MotionEnvelope, RearmEnvelope):
        raise TypeError("unsupported signed envelope")
    if type(text) is not str:
        raise MotorInputRejected("message is not a string")
    try:
        data = text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise MotorInputRejected("non-ASCII message") from exc
    if not 0 < len(data) <= MAX_JSON_BYTES:
        raise MotorInputRejected("message size out of bounds")
    try:
        item = json.loads(
            text, object_pairs_hook=_strict_pairs,
            parse_constant=_bad_constant)
    except (ValueError, TypeError) as exc:
        raise MotorInputRejected("invalid signed JSON envelope") from exc
    names = {field.name for field in fields(envelope_type)}
    if type(item) is not dict or set(item) != names:
        raise MotorInputRejected("unexpected or missing signed fields")
    string_keys = ({"session_id", "mac", "owner"}
                   if envelope_type is MotionEnvelope
                   else {"session_id", "mac"})
    int_keys = ({"sequence", "generated_ns", "expires_ns"}
                if envelope_type is MotionEnvelope
                else {"counter", "generated_ns", "expires_ns"})
    float_keys = ({"vx", "vy", "wz"}
                  if envelope_type is MotionEnvelope else set())
    if (any(type(item[k]) is not str for k in string_keys)
            or any(type(item[k]) is not int for k in int_keys)
            or any(type(item[k]) is not float for k in float_keys)):
        raise MotorInputRejected("signed JSON field type mismatch")
    return envelope_type(**item)


def read_protected_key(path):
    """Read a 32-byte rootless local key; reject symlinks/loose permissions.

    Keys must be independently provisioned, owned by the running host
    process and mounted read-only. Never log or publish their content.
    """
    if type(path) is not str or not path.startswith("/"):
        raise MotorInputRejected("absolute key file path required")
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
    fd = os.open(path, flags)
    try:
        st = os.fstat(fd)
        if (not stat.S_ISREG(st.st_mode)
                or st.st_uid != os.geteuid()
                or st.st_mode & 0o077
                or st.st_nlink != 1
                or st.st_size != 32):
            raise MotorInputRejected(
                "key owner, size, links or permissions invalid")
        content = os.read(fd, 33)
        if len(content) != 32:
            raise MotorInputRejected("invalid key file length")
        return content
    finally:
        os.close(fd)
