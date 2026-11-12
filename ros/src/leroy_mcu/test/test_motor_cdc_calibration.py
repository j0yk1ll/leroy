"""Calibration CDC packet tests; no ROS, serial or motor I/O."""

from leroy_mcu.motor_cdc_protocol import MotorCdcProtocol
from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger

T0 = 1_000_000_000


def test_calibration_packet_and_ack():
    protocol = MotorCdcProtocol()
    assert protocol.calibration_wheels(
        (12, -12, 0, 0), now_ns=T0
    ) == b"set_wheels_calibration_tx 1 12 -12 0 0\n"
    assert protocol.receive(
        "ok set_wheels_calibration_tx seq=1",
        now_ns=T0 + 10_000_000,
    )
    assert protocol.healthy


def test_calibration_rejects_out_of_range_and_malformed():
    protocol = MotorCdcProtocol()
    for pwm in (
        (13, 0, 0, 0),
        (-13, 0, 0, 0),
        (True, 0, 0, 0),
        (1, 2, 3),
        [1, 2, 3, 4],
    ):
        assert protocol.calibration_wheels(pwm, now_ns=T0) is None
    assert protocol.healthy


def test_calibration_stop_preempts_unacknowledged_motion():
    protocol = MotorCdcProtocol()
    assert protocol.calibration_wheels(
        (3, 3, 3, 3), now_ns=T0
    ) == b"set_wheels_calibration_tx 1 3 3 3 3\n"
    assert protocol.stop(
        now_ns=T0 + 10_000_000
    ) == b"stop_wheels_tx 2\n"
    assert protocol.receive(
        "ok stop_wheels_tx seq=2",
        now_ns=T0 + 20_000_000,
    )
    assert protocol.stop_confirmed


def test_calibration_ack_mismatch_latches_fault():
    protocol = MotorCdcProtocol()
    assert protocol.calibration_wheels((2, 2, 2, 2), now_ns=T0)
    assert protocol.receive(
        "ok set_wheels_tx seq=1",
        now_ns=T0 + 10_000_000,
    )
    assert not protocol.healthy
    assert protocol.calibration_wheels(
        (2, 2, 2, 2), now_ns=T0 + 20_000_000
    ) is None


def test_calibration_ack_timeout_latches_fault():
    protocol = MotorCdcProtocol()
    assert protocol.calibration_wheels((2, 2, 2, 2), now_ns=T0)
    assert protocol.expired(now_ns=T0 + 150_000_001)
    assert not protocol.healthy


def test_calibration_uses_persistent_sequence(tmp_path):
    path = tmp_path / "sequence.json"
    with MotorSequenceLedger(path, initialize=True) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)
        assert protocol.calibration_wheels(
            (2, 2, 2, 2), now_ns=T0
        ) == b"set_wheels_calibration_tx 1 2 2 2 2\n"

    with MotorSequenceLedger(path) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)
        assert protocol.stop(
            now_ns=T0 + 20_000_000
        ) == b"stop_wheels_tx 2\n"
