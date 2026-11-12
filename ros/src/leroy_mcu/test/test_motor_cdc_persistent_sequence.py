"""Persistent sequence integration for the existing MCU protocol.

No ROS, serial device or motor hardware is accessed.
"""

import pytest

from leroy_mcu.motor_cdc_protocol import MotorCdcProtocol
from leroy_mcu.motor_sequence_ledger import (
    MotorSequenceLedger,
    SequenceLedgerError,
)


T0 = 1_000_000_000


def test_wheel_and_stop_share_durable_sequence(tmp_path):
    path = tmp_path / "motor-sequence.json"

    with MotorSequenceLedger(path, initialize=True) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)

        wheel = protocol.wheels(
            (5, 5, 5, 5),
            now_ns=T0,
        )
        assert wheel == b"set_wheels_tx 1 5 5 5 5\n"

        assert protocol.receive(
            "ok set_wheels_tx seq=1",
            now_ns=T0 + 10_000_000,
        )

        stop = protocol.stop(
            now_ns=T0 + 20_000_000,
        )
        assert stop == b"stop_wheels_tx 2\n"

        assert protocol.receive(
            "ok stop_wheels_tx seq=2",
            now_ns=T0 + 30_000_000,
        )
        assert protocol.stop_confirmed


def test_new_protocol_resumes_without_sequence_reuse(tmp_path):
    path = tmp_path / "motor-sequence.json"

    with MotorSequenceLedger(path, initialize=True) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)
        assert protocol.wheels(
            (3, 3, 3, 3),
            now_ns=T0,
        ) == b"set_wheels_tx 1 3 3 3 3\n"

    with MotorSequenceLedger(path) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)
        assert protocol.stop(
            now_ns=T0 + 50_000_000,
        ) == b"stop_wheels_tx 2\n"


def test_unacknowledged_wheel_does_not_consume_next_number(tmp_path):
    path = tmp_path / "motor-sequence.json"

    with MotorSequenceLedger(path, initialize=True) as ledger:
        protocol = MotorCdcProtocol(ledger=ledger)

        assert protocol.wheels(
            (2, 2, 2, 2), now_ns=T0,
        ) is not None

        assert protocol.wheels(
            (2, 2, 2, 2), now_ns=T0 + 10_000_000,
        ) is None

        # STOP can preempt the pending transaction.
        assert protocol.stop(
            now_ns=T0 + 20_000_000,
        ) == b"stop_wheels_tx 2\n"


def test_closed_ledger_latches_protocol_failure(tmp_path):
    path = tmp_path / "motor-sequence.json"

    ledger = MotorSequenceLedger(path, initialize=True)
    protocol = MotorCdcProtocol(ledger=ledger)
    ledger.close()

    assert protocol.wheels(
        (2, 2, 2, 2), now_ns=T0,
    ) is None
    assert not protocol.healthy

    # Reopening the ledger does not clear an existing protocol fault.
    with MotorSequenceLedger(path):
        assert protocol.wheels(
            (2, 2, 2, 2),
            now_ns=T0 + 10_000_000,
        ) is None


def test_invalid_ledger_type_rejected():
    with pytest.raises(TypeError):
        MotorCdcProtocol(ledger=object())


def test_legacy_protocol_behavior_unchanged():
    protocol = MotorCdcProtocol()

    assert protocol.wheels(
        (4, 4, 4, 4),
        now_ns=T0,
    ) == b"set_wheels_tx 1 4 4 4 4\n"

    assert protocol.receive(
        "ok set_wheels_tx seq=1",
        now_ns=T0 + 10_000_000,
    )

    assert protocol.stop(
        now_ns=T0 + 20_000_000,
    ) == b"stop_wheels_tx 2\n"
