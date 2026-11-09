"""ToF control must remain a strict non-actuating CDC allowlist entry."""
import pytest
from leroy_mcu.command_ingress import command_to_bytes, CommandRejected


def test_only_zero_and_five_hertz_allowed():
    assert command_to_bytes("set_tof_stream_hz 0") == b"set_tof_stream_hz 0\n"
    assert command_to_bytes("set_tof_stream_hz 5") == b"set_tof_stream_hz 5\n"


@pytest.mark.parametrize("value", ["1", "10", "-5", "05", "6", "500", "5 extra"])
def test_invalid_rates_rejected(value):
    with pytest.raises(CommandRejected):
        command_to_bytes("set_tof_stream_hz " + value)


def test_no_motion_bypass():
    with pytest.raises(CommandRejected):
        command_to_bytes("set_wheels 5 0 0 0")
