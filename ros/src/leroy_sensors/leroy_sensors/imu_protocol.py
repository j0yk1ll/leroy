"""Strict STM32 IMU streaming packet contract; no ROS/hardware imports.

This parser is deliberately not part of the motor-capable Leroy bridge.
The only permitted device writes are stream control START and STOP in the
separately isolated sensor runner, not wheel, manipulator or keepalive actions.
"""
import math
import re

START = b"set_imu_stream_hz 50\r\n"
STOP = b"set_imu_stream_hz 0\r\n"
PORT = "/dev/ttyACM0"
STREAM_HZ = 50
FIELD_NAMES = (
    "seq", "t_us", "cal", "af_mm_s2", "al_mm_s2", "au_mm_s2",
    "gr_mrad_s", "gp_mrad_s", "gy_mrad_s",
    "mag_valid", "mx_raw", "my_raw", "mz_raw",
)
INTEGER = re.compile(r"^-?[0-9]+$")
UINT32_MAX = 0xffffffff


class PacketError(ValueError):
    pass


def parse_packet(line):
    """Read only exact firmware telemetry; returns integer fields or None."""
    if not line.startswith("telemetry imu "):
        return None
    pieces = line.split()
    if pieces[:2] != ["telemetry", "imu"] or len(pieces) != 2 + len(FIELD_NAMES):
        raise PacketError("wrong_field_count")
    values = {}
    for token in pieces[2:]:
        if token.count("=") != 1:
            raise PacketError("bad_field_syntax")
        key, value = token.split("=", 1)
        if key not in FIELD_NAMES or key in values or not INTEGER.fullmatch(value):
            raise PacketError("bad_or_duplicate_field")
        n = int(value)
        if key in ("seq", "t_us"):
            if not 0 <= n <= UINT32_MAX:
                raise PacketError("out_of_range_uint32")
        elif not -(1 << 31) <= n < (1 << 31):
            raise PacketError("out_of_range_int32")
        values[key] = n
    if set(values) != set(FIELD_NAMES):
        raise PacketError("missing_field")
    if values["cal"] not in (0, 1) or values["mag_valid"] not in (0, 1):
        raise PacketError("bad_flag")
    for key in ("af_mm_s2", "al_mm_s2", "au_mm_s2",
                "gr_mrad_s", "gp_mrad_s", "gy_mrad_s"):
        if not math.isfinite(values[key] / 1000.0):
            raise PacketError("nonfinite_si_value")
    return values


def sequence_gap(prev, current):
    """Return new skipped sample count or raise for duplicate/reboot/reorder."""
    if prev is None:
        return 0
    gap = (current - prev) & UINT32_MAX
    if not 0 < gap <= 1000:
        raise PacketError("sequence_duplicate_reordered_or_restart")
    return gap - 1
