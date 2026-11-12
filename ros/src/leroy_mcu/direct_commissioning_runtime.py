"""Single-process keyboard/STM32 attended commissioning runtime.

Explicitly gated hardware entry point. Not a ROS node.
Never run concurrently with another MCU serial owner.
"""

import argparse
import select
import time

from direct_commissioning_input import DirectCommissioningInput
from direct_commissioning_writer import DirectCommissioningWriter
from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger

WATCHDOG_MS = 150
LINE_LIMIT = 512
STOP_ACK_WAIT_S = 0.20


class SerialLines:
    """Bounded incremental ASCII line decoder."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        if not isinstance(data, bytes):
            raise TypeError("serial input must be bytes")
        self.buffer.extend(data)
        if len(self.buffer) > LINE_LIMIT and b"\n" not in self.buffer:
            raise RuntimeError("MCU line exceeds limit")

        result = []
        while b"\n" in self.buffer:
            position = self.buffer.index(10)
            if position > LINE_LIMIT:
                raise RuntimeError("MCU line exceeds limit")
            line = bytes(self.buffer[:position])
            del self.buffer[:position + 1]
            try:
                result.append(line.rstrip(b"\r").decode("ascii"))
            except UnicodeDecodeError as exc:
                raise RuntimeError("invalid MCU ASCII") from exc

        if len(self.buffer) > LINE_LIMIT:
            raise RuntimeError("MCU line exceeds limit")
        return result


def check_mcu_line(line, writer=None):
    """Reject reported command errors and MCU watchdog stops."""
    if line == "event watchdog_stop":
        raise RuntimeError("MCU watchdog stopped motor")
    if line.startswith("error "):
        raise RuntimeError(f"MCU rejected command: {line}")
    if writer is not None:
        writer.receive(line, now_ns=time.monotonic_ns())
        if not writer.healthy:
            raise RuntimeError("MCU transaction ACK fault")
    return line == "ok set_watchdog_ms"


def poll_serial(port, decoder, writer=None):
    data = port.read(min(max(port.in_waiting, 1), 512))
    return [
        check_mcu_line(line, writer)
        for line in decoder.feed(data)
    ]


def require_watchdog_ack(port, decoder):
    packet = f"set_watchdog_ms {WATCHDOG_MS}\n".encode("ascii")
    if port.write(packet) != len(packet):
        raise RuntimeError("watchdog configuration write incomplete")

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if any(poll_serial(port, decoder)):
            return
        time.sleep(0.005)
    raise RuntimeError("MCU watchdog configuration unacknowledged")


def require_startup_stop_ack(port, decoder, writer, keyboard):
    now_ns = time.monotonic_ns()
    writer.apply(keyboard.tick(now_ns, False), now_ns=now_ns)

    deadline = time.monotonic() + STOP_ACK_WAIT_S
    while time.monotonic() < deadline:
        poll_serial(port, decoder, writer)
        if writer._startup_stop_ack and writer.healthy:
            return
        time.sleep(0.005)
    raise RuntimeError("startup STOP unacknowledged")


def finish_with_stop(port, decoder, writer):
    """Attempt a sequenced STOP; failure is NOT a physical stop."""
    if not writer._stopping:
        writer.stop(now_ns=time.monotonic_ns())

    deadline = time.monotonic() + STOP_ACK_WAIT_S
    while time.monotonic() < deadline:
        try:
            poll_serial(port, decoder, writer)
        except (OSError, RuntimeError) as exc:
            print(
                f"STOP ACK processing failed: "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
            print(
                f"STOP state: stopping={writer._stopping}, "
                f"confirmed={writer.stop_confirmed}, "
                f"protocol_fault={writer.protocol._fault!r}",
                flush=True,
            )
            break
        if writer.stop_confirmed:
            return True
        time.sleep(0.005)
    return writer.stop_confirmed


def run(*, device_path, serial_path, ledger_path, seconds):
    from evdev import InputDevice, ecodes
    import serial

    keyboard = DirectCommissioningInput()
    decoder = SerialLines()

    device = InputDevice(device_path)
    try:
        if set(device.active_keys()) & {
            ecodes.KEY_A, ecodes.KEY_SPACE
        }:
            raise RuntimeError("release A and Space before startup")

        with MotorSequenceLedger(ledger_path) as ledger:
            with serial.Serial(
                serial_path,
                baudrate=115200,
                timeout=0,
                write_timeout=0.05,
                exclusive=True,
            ) as port:
                writer = DirectCommissioningWriter(
                    ledger=ledger, write=port.write
                )
                stop_ok = False

                try:
                    require_watchdog_ack(port, decoder)
                    require_startup_stop_ack(
                        port, decoder, writer, keyboard
                    )

                    print(
                        "MCU watchdog and startup STOP acknowledged.",
                        flush=True,
                    )
                    print(
                        "A: request arm; hold Space within 10 seconds.",
                        flush=True,
                    )
                    print(
                        "Release Space to STOP; Ctrl+C to abort.",
                        flush=True,
                    )

                    deadline = time.monotonic() + seconds

                    while time.monotonic() < deadline:
                        poll_serial(port, decoder, writer)

                        ready, _, _ = select.select(
                            [device], [], [], 0.02
                        )

                        for active in ready:
                            for event in active.read():
                                if (
                                    event.type == ecodes.EV_SYN
                                    and event.code == ecodes.SYN_DROPPED
                                ):
                                    raise RuntimeError(
                                        "keyboard event overflow"
                                    )

                                if event.type != ecodes.EV_KEY:
                                    continue

                                if event.code == ecodes.KEY_A:
                                    key = "arm"
                                elif event.code == ecodes.KEY_SPACE:
                                    key = "space"
                                else:
                                    continue

                                now_ns = time.monotonic_ns()
                                decision = keyboard.key_event(
                                    key, event.value, now_ns
                                )
                                if (
                                    decision is not None
                                    and decision.stop_required
                                ):
                                    writer.apply(
                                        decision, now_ns=now_ns
                                    )

                        held = (
                            ecodes.KEY_SPACE
                            in set(device.active_keys())
                        )
                        now_ns = time.monotonic_ns()
                        decision = keyboard.tick(now_ns, held)
                        writer.apply(decision, now_ns=now_ns)

                        if (
                            keyboard.fault
                            or decision.state == "FAULT"
                            or not writer.healthy
                        ):
                            raise RuntimeError(
                                "commissioning session inhibited"
                            )

                        if writer._stopping and writer._motion_sent:
                            break

                finally:
                    keyboard.inhibit()
                    stop_ok = finish_with_stop(
                        port, decoder, writer
                    )
                    print(
                        "Final sequenced STOP "
                        + ("ACKNOWLEDGED" if stop_ok else "UNCONFIRMED"),
                        flush=True,
                    )

                if not stop_ok:
                    raise RuntimeError(
                        "MCU STOP unconfirmed; use independent cutoff"
                    )
    finally:
        device.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/input/event2")
    parser.add_argument("--serial", default="/dev/ttyACM0")
    parser.add_argument("--ledger", required=True)
    parser.add_argument("--seconds", type=float, default=15.0)

    parser.add_argument(
        "--firmware-verified",
        action="store_true",
        help="Confirm updated STM32 firmware was flashed and verified",
    )
    parser.add_argument(
        "--serial-handover-complete",
        action="store_true",
        help="Confirm previous MCU serial owner was stopped",
    )
    args = parser.parse_args()

    if not (
        args.firmware_verified
        and args.serial_handover_complete
    ):
        parser.error(
            "hardware execution denied: firmware verification and "
            "exclusive serial handover must both be confirmed"
        )

    if not 0 < args.seconds <= 30:
        parser.error("--seconds must be between 0 and 30")

    run(
        device_path=args.device,
        serial_path=args.serial,
        ledger_path=args.ledger,
        seconds=args.seconds,
    )


if __name__ == "__main__":
    main()
