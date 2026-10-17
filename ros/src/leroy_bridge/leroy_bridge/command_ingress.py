"""M22D: ROS-free serial-command allowlist for the existing Leroy bridge.

This is an ACTUATION-INHIBITING transition milestone. No nonzero wheel
command can be produced by this allowlist. There is intentionally NO
parameter, environment variable, mode switch, token, or hidden override
that can enable actuation. It is not a physical E-stop or MCU STOP ACK.

The next software milestone will design the actual accepted command
envelope, session-binding and bridge-side authorization mechanism.
"""
import re


class CommandRejected(ValueError):
    """Reject malformed, motion-capable, or non-allowlisted CDC command."""


ZERO_WHEELS = "set_wheels 0 0 0 0"
_ALLOWED_STREAM_RATES = frozenset((0, 10, 20, 50, 100))
_SETTING = re.compile(r"(?P<name>set_watchdog_ms|set_imu_stream_hz|set_encoder_stream_hz) (?P<value>[0-9]+)\Z")


def command_to_bytes(command):
    """Return canonical allowlisted ASCII bytes or reject before serial I/O.

    Only zero wheel actuation and non-motion telemetry/watchdog controls
    from the current bridge are admitted. No newline, extra argument,
    Unicode, negative setting, or arbitrary command can be smuggled in.
    """
    if type(command) is not str:
        raise CommandRejected("CDC command must be a Python str")
    if command in (ZERO_WHEELS, "keepalive"):
        return (command + "\n").encode("ascii")
    match = _SETTING.fullmatch(command)
    if match is None:
        raise CommandRejected("CDC command not in M22D allowlist")
    name = match.group("name")
    value_text = match.group("value")
    if len(value_text) > 5:
        raise CommandRejected("CDC numeric setting is too long")
    if str(int(value_text)) != value_text:
        raise CommandRejected("noncanonical numeric CDC setting")
    value = int(value_text)
    if name == "set_watchdog_ms":
        if not 1 <= value <= 60000:
            raise CommandRejected("watchdog must remain enabled")
    elif value not in _ALLOWED_STREAM_RATES:
        raise CommandRejected("invalid sensor stream rate")
    return (command + "\n").encode("ascii")
