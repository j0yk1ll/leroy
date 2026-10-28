"""Single attended wheel pulse for first STM32 bench commissioning.

Not ROS actuator authority and not a replacement for the independent
hardwired motor-power cutoff. No E-stop GPIO state is invented. Never
run concurrently with the ROS serial bridge. Requires PR #156 firmware.
"""
import argparse
import time

import serial


MAX_PWM = 48
MAX_PULSE_MS = 150
WATCHDOG_MARGIN_MS = 100
WHEELS = {"fl": 0, "fr": 1, "rl": 2, "rr": 3}


def _request(mcu, command, expected, *, ack_timeout_s=0.25):
    mcu.write((command + "\n").encode("ascii"))
    mcu.flush()
    deadline = time.monotonic() + ack_timeout_s
    while time.monotonic() < deadline:
        line = mcu.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        if line == expected:
            return
        if line.startswith(("error ", "event watchdog_stop")):
            raise RuntimeError("MCU refused or aborted command: " + line)
    raise RuntimeError("Missing exact MCU ACK: " + expected)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="One short (<=150ms) low-duty wheel pulse with MCU watchdog and STOP ACK")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--wheel", choices=tuple(WHEELS), required=True)
    parser.add_argument("--pwm", type=int, default=18)
    parser.add_argument("--duration-ms", type=int, default=60)
    opts = parser.parse_args(args)
    if not 1 <= opts.pwm <= MAX_PWM:
        parser.error("pwm must be 1..48")
    if not 20 <= opts.duration_ms <= MAX_PULSE_MS:
        parser.error("duration-ms must be 20..150")
    if not opts.port.startswith("/dev/"):
        parser.error("pass actual /dev/ttyACM* controller path")

    # An interactive local operator is required before ANY nonzero packet.
    # /dev/tty is not seekable. Python "r+" requires seekable input on
    # some systems and fails before a single serial command is sent.
    # Separate read/write handles work with normal interactive POSIX TTYs.
    try:
        with open("/dev/tty", "r", encoding="ascii") as tty_in, \
             open("/dev/tty", "w", encoding="ascii") as tty_out:
            tty_out.write(
                "ONE-WHEEL BENCH PULSE. STOP the ROS bridge first.\n"
                "Robot SECURELY SUPPORTED, wheels clear, hands/tools away.\n"
                "Independent HARDWIRED motor-power cutoff physically verified,\n"
                "accessible and interrupts motor supply despite USB power.\n"
                "This tool cannot read the E-stop state.\n"
                "Type WHEEL_TEST_READY (exactly) to proceed: "
            )
            tty_out.flush()
            if tty_in.readline().strip() != "WHEEL_TEST_READY":
                raise SystemExit("No wheel command sent")
    except OSError as exc:
        raise SystemExit(
            f"Interactive operator terminal unavailable: {exc}"
        ) from exc

    # A persistent MCU may have seen a prior host's sequence. Unix seconds
    # exceed typical session counters and rise between sessions; same-second
    # retries reject stale sequence, rather than rearm automatically.
    seq = int(time.time())
    if not 0 < seq <= 0xffffffff - 4:
        raise RuntimeError("MCU uint32 sequence range unavailable")
    pwm = [0, 0, 0, 0]
    pwm[WHEELS[opts.wheel]] = opts.pwm
    watchdog_ms = opts.duration_ms + WATCHDOG_MARGIN_MS
    print(f"Wheel={opts.wheel} duty={opts.pwm}/255 "
          f"commanded pulse target {opts.duration_ms}ms; "
          f"MCU watchdog {watchdog_ms}ms")
    try:
        # POSIX exclusive advisory lock: ensure ROS bridge already stopped.
        with serial.Serial(opts.port, baudrate=115200, timeout=0.025,
                           write_timeout=0.10, exclusive=True) as mcu:
            time.sleep(0.20)
            mcu.reset_input_buffer()
            try:
                _request(mcu, f"stop_wheels_tx {seq}",
                         f"ok stop_wheels_tx seq={seq}")
                _request(mcu, f"set_watchdog_ms {watchdog_ms}",
                         "ok set_watchdog_ms")
                pulse_started = time.monotonic()
                _request(
                    mcu,
                    f"set_wheels_tx {seq+1} " + " ".join(map(str, pwm)),
                    f"ok set_wheels_tx seq={seq+1}",
                    ack_timeout_s=0.040,
                )
                # Never send keepalive. Bound the host-requested pulse
                # from BEFORE the wheel packet, not after waiting for ACK.
                # MCU watchdog (pulse + 100ms) protects a stalled host while
                # its main loop remains responsive. The hardwired cutoff
                # remains essential.
                remaining = (opts.duration_ms / 1000.0) - (
                    time.monotonic() - pulse_started)
                if remaining > 0:
                    time.sleep(remaining)
                _request(mcu, f"stop_wheels_tx {seq+2}",
                         f"ok stop_wheels_tx seq={seq+2}")
                print("MCU_STOP_ACK=PASS; physical stopping not measured")
            finally:
                try:
                    _request(mcu, f"stop_wheels_tx {seq+3}",
                             f"ok stop_wheels_tx seq={seq+3}")
                except Exception as exc:
                    print(f"STOP_UNCONFIRMED: {exc}; use physical cutoff")
    except (serial.SerialException, OSError) as exc:
        raise SystemExit(f"Serial operation failed; use physical cutoff: {exc}")


if __name__ == "__main__":
    main()
