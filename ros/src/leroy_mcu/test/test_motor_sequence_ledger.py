"""Persistent sequence ownership regression tests."""

import json
import pytest

from leroy_mcu.motor_sequence_ledger import (
    MAX_SEQUENCE,
    MotorSequenceLedger,
    SequenceLedgerError,
)


def test_initialization_requires_explicit_permission(tmp_path):
    path = tmp_path / "sequence.json"
    with pytest.raises(SequenceLedgerError):
        MotorSequenceLedger(path)
    assert not path.exists()


def test_sequence_survives_process_style_reopen(tmp_path):
    path = tmp_path / "sequence.json"

    with MotorSequenceLedger(path, initialize=True) as ledger:
        assert ledger.reserve() == 1
        assert ledger.reserve() == 2

    with MotorSequenceLedger(path) as ledger:
        assert ledger.reserve() == 3


def test_second_owner_is_rejected(tmp_path):
    path = tmp_path / "sequence.json"

    with MotorSequenceLedger(path, initialize=True):
        with pytest.raises(BlockingIOError):
            MotorSequenceLedger(path)


def test_corrupt_ledger_fails_closed(tmp_path):
    path = tmp_path / "sequence.json"
    path.write_text("not valid json")

    with pytest.raises(SequenceLedgerError):
        MotorSequenceLedger(path)


def test_invalid_sequence_type_fails_closed(tmp_path):
    path = tmp_path / "sequence.json"
    path.write_text(json.dumps({
        "version": 1,
        "last_reserved": True,
    }))

    with pytest.raises(SequenceLedgerError):
        MotorSequenceLedger(path)


def test_sequence_exhaustion_fails_closed(tmp_path):
    path = tmp_path / "sequence.json"
    path.write_text(json.dumps({
        "version": 1,
        "last_reserved": MAX_SEQUENCE,
    }))

    with MotorSequenceLedger(path) as ledger:
        with pytest.raises(SequenceLedgerError):
            ledger.reserve()


def test_closed_ledger_cannot_issue_sequence(tmp_path):
    path = tmp_path / "sequence.json"
    ledger = MotorSequenceLedger(path, initialize=True)
    ledger.close()

    with pytest.raises(SequenceLedgerError):
        ledger.reserve()
