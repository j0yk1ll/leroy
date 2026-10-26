"""Single attended wheel pulse for first STM32 bench commissioning.

Not ROS actuator authority and not a replacement for the independent
hardwired motor-power cutoff. No E-stop GPIO state is invented. Never
run concurrently with the ROS serial bridge. Requires PR #156 firmware.
"""
import argparse
import time

import serial


MAX_PWM = 24
PULSE_SECONDS = 0.060
WATCHDOG_MS = 120
WHEELS = {"fl": 0, "fr": 1, "rl": 2, "rr": 3}


def _request(mcu, command, expected):
    mcu.write((command + "\n").encode("ascii"))
    mcu.flush()
    deadline = time.monotonic() + 0.25
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
        description="One <=60ms wheel pulse, with MCU watchdog and stop ACK")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--wheel", choices=tuple(WHEELS), required=True)
    parser.add_argument("--pwm", type=int, default=18)
    opts = parser.parse_args(args)
    if not 1 <= opts.pwm <= MAX_PWM:
        parser.error("pwm must be 1..24")
    if not opts.port.startswith("/dev/"):
        parser.error("pass actual /dev/ttyACM* controller path")

    # An interactive local operator is required before ANY nonzero packet.
    with open("/dev/tty", "r+", encoding="ascii") as tty:
        tty.write(
            "ONE-WHEEL BENCH PULSE. STOP the ROS bridge first.\n"
            "Robot SECURELY SUPPORTED, wheels clear, hands/tools away.\n"
            "Independent HARDWIRED motor-power cutoff physically verified,\n"
            "accessible and interrupts motor supply despite USB power.\n"
            "This tool cannot read the E-stop state.\n"
            "Type WHEEL_TEST_READY (exactly) to proceed: "
        )
        tty.flush()
        if tty.readline().strip() != "WHEEL_TEST_READY":
            raise SystemExit("No wheel command sent")

    # A persistent MCU may have seen a prior host's sequence. Unix seconds
    # exceed typical session counters and rise between sessions; same-second
    # retries reject stale sequence, rather than rearm automatically.
    seq = int(time.time())
    if not 0 < seq <= 0xffffffff - 4:
        raise RuntimeError("MCU uint32 sequence range unavailable")
    pwm = [0, 0, 0, 0]
    pwm[WHEELS[opts.wheel]] = opts.pwm
    print(f"Wheel={opts.wheel} duty={opts.pwm}/255 duration<=60ms")
    try:
        # POSIX exclusive advisory lock: ensure ROS bridge already stopped.
        with serial.Serial(opts.port, baudrate=115200, timeout=0.025,
                           write_timeout=0.10, exclusive=True) as mcu:
            time.sleep(0.20)
            mcu.reset_input_buffer()
            try:
                _request(mcu, f"stop_wheels_tx {seq}",
                         f"ok stop_wheels_tx seq={seq}")
                _request(mcu, f"set_watchdog_ms {WATCHDOG_MS}",
                         "ok set_watchdog_ms")
                _request(
                    mcu,
                    f"set_wheels_tx {seq+1} " + " ".join(map(str, pwm)),
                    f"ok set_wheels_tx seq={seq+1}",
                )
                # Never send keepalive. MCU watchdog zeros outputs after
                # 120 ms if the handler runs but our stop cannot be sent.
                time.sleep(PULSE_SECONDS)
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
