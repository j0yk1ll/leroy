"""Keyboard + sequenced MCU writer integration, without hardware."""

from direct_commissioning_input import DirectCommissioningInput
from direct_commissioning_writer import DirectCommissioningWriter
from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger

T0 = 1_000_000_000
STOP1 = b"stop_wheels_tx 1\n"
WHEELS2 = b"set_wheels_calibration_tx 2 4 4 4 4\n"
STOP3 = b"stop_wheels_tx 3\n"


def session(tmp_path):
    ledger = MotorSequenceLedger(
        tmp_path / "motor-sequence.json", initialize=True
    )
    packets = []

    def transmit(packet):
        packets.append(packet)
        return len(packet)

    writer = DirectCommissioningWriter(
        ledger=ledger, write=transmit
    )
    keyboard = DirectCommissioningInput()
    return ledger, writer, keyboard, packets


def startup(writer, keyboard, packets):
    decision = keyboard.tick(T0, False)
    assert not writer.apply(decision, now_ns=T0)
    assert packets == [STOP1]
    assert not writer.stop_confirmed


def acknowledge_startup(writer):
    assert writer.receive(
        "ok stop_wheels_tx seq=1",
        now_ns=T0 + 10_000_000,
    )
    assert writer.healthy


def arm_and_press(keyboard):
    keyboard.key_event("arm", 1, T0 + 20_000_000)
    assert keyboard.policy.state == "DISARMED"
    keyboard.key_event("space", 1, T0 + 30_000_000)
    assert keyboard.policy.state == "ARMED"


def test_complete_attended_drive_and_stop(tmp_path):
    ledger, writer, keyboard, packets = session(tmp_path)
    try:
        startup(writer, keyboard, packets)
        acknowledge_startup(writer)
        arm_and_press(keyboard)

        decision = keyboard.tick(T0 + 40_000_000, True)
        assert decision.proposed_pwm == (4, 4, 4, 4)
        assert writer.apply(
            decision, now_ns=T0 + 40_000_000
        )
        assert packets == [STOP1, WHEELS2]

        assert writer.receive(
            "ok set_wheels_calibration_tx seq=2",
            now_ns=T0 + 45_000_000,
        )

        release = keyboard.key_event(
            "space", 0, T0 + 50_000_000
        )
        assert release.stop_required
        assert not writer.apply(
            release, now_ns=T0 + 50_000_000
        )
        assert packets == [STOP1, WHEELS2, STOP3]
        assert not writer.stop_confirmed

        assert writer.receive(
            "ok stop_wheels_tx seq=3",
            now_ns=T0 + 55_000_000,
        )
        assert writer.stop_confirmed
        assert keyboard.policy.state == "DISARMED"

        # A late/stale input cannot restart the stopped writer.
        count = len(packets)
        assert not writer.apply(
            keyboard.tick(T0 + 60_000_000, False),
            now_ns=T0 + 60_000_000,
        )
        assert len(packets) == count
    finally:
        ledger.close()


def test_wheel_commands_blocked_until_startup_ack(tmp_path):
    ledger, writer, keyboard, packets = session(tmp_path)
    try:
        startup(writer, keyboard, packets)
        arm_and_press(keyboard)

        proposal = keyboard.tick(T0 + 40_000_000, True)
        assert not writer.apply(
            proposal, now_ns=T0 + 40_000_000
        )
        assert packets == [STOP1]

        acknowledge_startup(writer)
        proposal = keyboard.tick(T0 + 50_000_000, True)
        assert writer.apply(
            proposal, now_ns=T0 + 50_000_000
        )
        assert packets[-1] == WHEELS2
    finally:
        ledger.close()


def test_missing_wheel_ack_requests_terminal_stop(tmp_path):
    ledger, writer, keyboard, packets = session(tmp_path)
    try:
        startup(writer, keyboard, packets)
        acknowledge_startup(writer)
        arm_and_press(keyboard)

        assert writer.apply(
            keyboard.tick(T0 + 40_000_000, True),
            now_ns=T0 + 40_000_000,
        )
        assert packets[-1] == WHEELS2

        # MCU ACK deadline expires. No second wheel packet allowed.
        assert not writer.apply(
            keyboard.tick(T0 + 210_000_000, True),
            now_ns=T0 + 210_000_000,
        )
        assert not writer.healthy
        assert packets == [STOP1, WHEELS2, STOP3]
        assert not writer.stop_confirmed
    finally:
        ledger.close()


def test_lost_key_release_requests_stop(tmp_path):
    ledger, writer, keyboard, packets = session(tmp_path)
    try:
        startup(writer, keyboard, packets)
        acknowledge_startup(writer)
        arm_and_press(keyboard)

        assert writer.apply(
            keyboard.tick(T0 + 40_000_000, True),
            now_ns=T0 + 40_000_000,
        )

        # Simulate the physical key being released but its event lost.
        decision = keyboard.tick(
            T0 + 50_000_000, actual_space_held=False
        )
        assert keyboard.fault
        assert decision.stop_required
        assert not writer.apply(
            decision, now_ns=T0 + 50_000_000
        )
        assert packets[-1] == STOP3
        assert not writer.stop_confirmed
    finally:
        ledger.close()
