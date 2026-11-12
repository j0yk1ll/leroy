"""Simulated serial writer tests; no physical device access."""

from direct_commissioning_writer import DirectCommissioningWriter
from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger
from leroy_safety.attended_commissioning import AttendedCommissioning

T0 = 1_000_000_000


def setup(tmp_path):
    ledger = MotorSequenceLedger(
        tmp_path / "sequence.json", initialize=True
    )
    packets = []

    writer = DirectCommissioningWriter(
        ledger=ledger,
        write=lambda packet: (
            packets.append(packet),
            len(packet),
        )[1],
    )

    control = AttendedCommissioning()
    control.arm(now_ns=T0, operator_confirmed=True)

    return ledger, writer, packets, control


def bootstrap(writer):
    """Require startup STOP acknowledgement before any wheel PWM."""
    from leroy_safety.attended_commissioning import AttendedCommissioning
    initial = AttendedCommissioning().disarm()
    assert not writer.apply(initial, now_ns=T0 - 100_000_000)
    assert not writer.stop_confirmed
    assert writer.receive(
        "ok stop_wheels_tx seq=1",
        now_ns=T0 - 90_000_000,
    )
    assert writer.healthy


def held(control, now_ns, pwm=(4, 4, 4, 4)):
    return control.tick(
        now_ns=now_ns,
        deadman_held=True,
        requested_pwm=pwm,
    )


def test_initial_motion_requires_armed_decision(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        decision = control.disarm()
        assert not writer.apply(decision, now_ns=T0 + 1)
        assert packets == [b"stop_wheels_tx 1\n"]
    finally:
        ledger.close()


def test_wheel_ack_gates_next_transaction(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        assert writer.apply(held(control, T0 + 1), now_ns=T0 + 1)
        assert packets == [
            b"stop_wheels_tx 1\n",
            b"set_wheels_calibration_tx 2 4 4 4 4\n",
        ]

        assert not writer.apply(
            held(control, T0 + 60_000_000),
            now_ns=T0 + 60_000_000,
        )
        assert len(packets) == 2

        assert writer.receive(
            "ok set_wheels_calibration_tx seq=2",
            now_ns=T0 + 65_000_000,
        )

        assert writer.apply(
            held(control, T0 + 70_000_000),
            now_ns=T0 + 70_000_000,
        )
        assert packets[-1] == (
            b"set_wheels_calibration_tx 3 4 4 4 4\n"
        )
    finally:
        ledger.close()


def test_release_preempts_unacknowledged_wheels(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        writer.apply(held(control, T0 + 1), now_ns=T0 + 1)

        release = control.disarm()
        assert not writer.apply(
            release, now_ns=T0 + 10_000_000
        )
        assert packets[-1] == b"stop_wheels_tx 3\n"
        assert not writer.stop_confirmed

        assert writer.receive(
            "ok stop_wheels_tx seq=3",
            now_ns=T0 + 20_000_000,
        )
        assert writer.stop_confirmed
    finally:
        ledger.close()


def test_wrong_ack_faults_and_inhibits(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        writer.apply(held(control, T0 + 1), now_ns=T0 + 1)

        writer.receive(
            "ok set_wheels_calibration_tx seq=999",
            now_ns=T0 + 10_000_000,
        )

        assert not writer.healthy
        writer.apply(
            held(control, T0 + 20_000_000),
            now_ns=T0 + 20_000_000,
        )
        assert packets[-1] == b"stop_wheels_tx 3\n"
    finally:
        ledger.close()


def test_missing_ack_faults_and_requests_stop(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        writer.apply(held(control, T0 + 1), now_ns=T0 + 1)

        writer.apply(
            held(control, T0 + 200_000_000),
            now_ns=T0 + 200_000_000,
        )
        assert not writer.healthy
        assert packets[-1] == b"stop_wheels_tx 3\n"
        assert not writer.stop_confirmed
    finally:
        ledger.close()


def test_oversize_pwm_never_transmitted(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        decision = held(
            control, T0 + 1, pwm=(13, 13, 13, 13)
        )
        assert decision.stop_required
        assert not writer.apply(decision, now_ns=T0 + 1)
        assert not any(
            packet.startswith(b"set_wheels_calibration_tx")
            for packet in packets
        )
    finally:
        ledger.close()


def test_partial_serial_write_inhibits(tmp_path):
    ledger = MotorSequenceLedger(
        tmp_path / "sequence.json", initialize=True
    )

    writer = DirectCommissioningWriter(
        ledger=ledger,
        write=lambda packet: len(packet) - 1,
    )

    control = AttendedCommissioning()
    control.arm(now_ns=T0, operator_confirmed=True)

    try:
        assert not writer.apply(
            held(control, T0 + 1),
            now_ns=T0 + 1,
        )
        assert not writer.healthy
        assert not writer.stop_confirmed
    finally:
        ledger.close()


def test_startup_stop_ack_required_before_wheels(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        proposal = held(control, T0 + 1)
        assert not writer.apply(proposal, now_ns=T0 + 1)
        assert packets == [b"stop_wheels_tx 1\n"]
        assert not writer.apply(proposal, now_ns=T0 + 2)
        assert len(packets) == 1

        assert writer.receive(
            "ok stop_wheels_tx seq=1",
            now_ns=T0 + 3,
        )
        assert writer.apply(proposal, now_ns=T0 + 4)
        assert packets[-1] == (
            b"set_wheels_calibration_tx 2 4 4 4 4\n"
        )
    finally:
        ledger.close()


def test_post_motion_stop_cannot_rearm_writer(tmp_path):
    ledger, writer, packets, control = setup(tmp_path)
    try:
        bootstrap(writer)
        assert writer.apply(
            held(control, T0 + 1), now_ns=T0 + 1
        )
        writer.apply(control.disarm(), now_ns=T0 + 2)
        assert packets[-1] == b"stop_wheels_tx 3\n"

        assert writer.receive(
            "ok stop_wheels_tx seq=3", now_ns=T0 + 3
        )
        assert writer.stop_confirmed

        count = len(packets)
        assert not writer.apply(
            held(control, T0 + 4), now_ns=T0 + 4
        )
        assert len(packets) == count
    finally:
        ledger.close()
