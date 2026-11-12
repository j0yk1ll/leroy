"""Runtime boundary checks. No device is opened."""

import pytest

from direct_commissioning_runtime import (
    SerialLines, check_mcu_line,
)


def test_fragmented_serial_ack():
    reader = SerialLines()
    assert reader.feed(b"ok stop_wheels_tx seq=") == []
    assert reader.feed(b"7\n") == [
        "ok stop_wheels_tx seq=7"
    ]


def test_multiple_serial_lines():
    reader = SerialLines()
    assert reader.feed(
        b"telemetry enc 1 2\nok set_watchdog_ms\r\n"
    ) == ["telemetry enc 1 2", "ok set_watchdog_ms"]


def test_reject_oversize_serial_line():
    reader = SerialLines()
    with pytest.raises(RuntimeError):
        reader.feed(b"x" * 513)


def test_reject_invalid_serial_bytes():
    reader = SerialLines()
    with pytest.raises(RuntimeError):
        reader.feed(b"\xff\n")


def test_watchdog_ack_recognized():
    assert check_mcu_line("ok set_watchdog_ms")


def test_mcu_command_rejection_is_terminal():
    with pytest.raises(RuntimeError):
        check_mcu_line("error set_wheels_calibration_tx")


def test_mcu_watchdog_stop_is_terminal():
    with pytest.raises(RuntimeError):
        check_mcu_line("event watchdog_stop")



class SimulatedMcu:
    """Respond to exact commands; never access a physical serial port."""

    def __init__(self, *, acknowledge_watchdog=True,
                 acknowledge_stop=True):
        self.acknowledge_watchdog = acknowledge_watchdog
        self.acknowledge_stop = acknowledge_stop
        self.responses = bytearray()
        self.commands = []

    @property
    def in_waiting(self):
        return len(self.responses)

    def write(self, packet):
        self.commands.append(packet)
        command = packet.decode("ascii").strip()

        if command == "set_watchdog_ms 150":
            if self.acknowledge_watchdog:
                self.responses.extend(b"ok set_watchdog_ms\n")
        elif command.startswith("stop_wheels_tx "):
            if self.acknowledge_stop:
                sequence = command.split()[-1]
                self.responses.extend(
                    f"ok stop_wheels_tx seq={sequence}\n".encode()
                )
        elif command.startswith("set_wheels_calibration_tx "):
            sequence = command.split()[1]
            self.responses.extend(
                f"ok set_wheels_calibration_tx seq={sequence}\n".encode()
            )
        else:
            raise AssertionError(f"unexpected MCU command: {command}")

        return len(packet)

    def read(self, count):
        data = bytes(self.responses[:count])
        del self.responses[:count]
        return data


def test_complete_simulated_mcu_handshake(tmp_path):
    from direct_commissioning_runtime import (
        SerialLines, require_watchdog_ack,
        require_startup_stop_ack, finish_with_stop,
    )
    from direct_commissioning_input import DirectCommissioningInput
    from direct_commissioning_writer import DirectCommissioningWriter
    from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger

    mcu = SimulatedMcu()
    decoder = SerialLines()
    keyboard = DirectCommissioningInput()

    with MotorSequenceLedger(
        tmp_path / "sequence.json", initialize=True
    ) as ledger:
        writer = DirectCommissioningWriter(
            ledger=ledger, write=mcu.write
        )

        require_watchdog_ack(mcu, decoder)
        require_startup_stop_ack(mcu, decoder, writer, keyboard)

        assert writer.healthy
        assert writer._startup_stop_ack
        assert mcu.commands == [
            b"set_watchdog_ms 150\n",
            b"stop_wheels_tx 1\n",
        ]

        # A clean exit also requires a new, sequence-matched STOP.
        assert finish_with_stop(mcu, decoder, writer)
        assert writer.stop_confirmed
        assert mcu.commands[-1] == b"stop_wheels_tx 2\n"


def test_missing_watchdog_ack_denies_startup():
    from direct_commissioning_runtime import (
        SerialLines, require_watchdog_ack,
    )

    mcu = SimulatedMcu(acknowledge_watchdog=False)

    # Avoid waiting for the real 2-second startup deadline.
    from unittest.mock import patch

    with patch(
        "direct_commissioning_runtime.time.monotonic",
        side_effect=[0.0, 3.0],
    ):
        with pytest.raises(
            RuntimeError,
            match="watchdog configuration unacknowledged",
        ):
            require_watchdog_ack(mcu, SerialLines())

    assert mcu.commands == [b"set_watchdog_ms 150\n"]


def test_missing_startup_stop_ack_denies_motion(tmp_path):
    from direct_commissioning_runtime import (
        SerialLines, require_startup_stop_ack,
    )
    from direct_commissioning_input import DirectCommissioningInput
    from direct_commissioning_writer import DirectCommissioningWriter
    from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger
    from unittest.mock import patch

    mcu = SimulatedMcu(acknowledge_stop=False)

    with MotorSequenceLedger(
        tmp_path / "sequence.json", initialize=True
    ) as ledger:
        writer = DirectCommissioningWriter(
            ledger=ledger, write=mcu.write
        )

        with patch(
            "direct_commissioning_runtime.time.monotonic",
            side_effect=[0.0, 1.0],
        ):
            with pytest.raises(
                RuntimeError,
                match="startup STOP unacknowledged",
            ):
                require_startup_stop_ack(
                    mcu,
                    SerialLines(),
                    writer,
                    DirectCommissioningInput(),
                )

        assert not writer._startup_stop_ack
        assert not any(
            packet.startswith(b"set_wheels_calibration_tx")
            for packet in mcu.commands
        )


def test_missing_final_stop_ack_is_unconfirmed(tmp_path):
    from direct_commissioning_runtime import (
        SerialLines, finish_with_stop,
    )
    from direct_commissioning_writer import DirectCommissioningWriter
    from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger
    from unittest.mock import patch

    mcu = SimulatedMcu(acknowledge_stop=False)

    with MotorSequenceLedger(
        tmp_path / "sequence.json", initialize=True
    ) as ledger:
        writer = DirectCommissioningWriter(
            ledger=ledger, write=mcu.write
        )

        with patch(
            "direct_commissioning_runtime.time.monotonic",
            side_effect=[0.0, 1.0],
        ):
            assert not finish_with_stop(mcu, SerialLines(), writer)

        assert not writer.stop_confirmed
        assert mcu.commands == [b"stop_wheels_tx 1\n"]
