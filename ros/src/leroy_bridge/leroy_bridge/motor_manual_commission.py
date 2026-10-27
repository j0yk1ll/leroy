"""Attended ROS 2 step-motion commissioning; never an autonomous controller.

Run INSTEAD of leroy_bridge_node. The only motion input is a local TTY:
SPACE enables exactly one fresh direction key, which causes at most one
150-ms four-wheel pulse. No ROS topic, remote publisher, wheel encoder,
SLAM result, or fabricated E-stop GPIO can authorize movement. Every pulse
has an MCU watchdog, sequenced ACK, and explicit STOP; no keepalives.

A hardwired motor-power cutoff remains mandatory and independent of software.
"""
import argparse
import fcntl
import os
import select
import termios
import time
import tty


DIRECTIONS = {
    "w": (1, 1, 1, 1),       # forward
    "s": (-1, -1, -1, -1),   # backward
    "a": (-1, 1, 1, -1),     # left strafe
    "d": (1, -1, -1, 1),     # right strafe
    "q": (-1, 1, -1, 1),     # counterclockwise
    "e": (1, -1, 1, -1),     # clockwise
}
MAX_PWM = 64                    # Matches MCU's signed wheel command ceiling.
MAX_PULSE_MS = 150
ENABLE_WINDOW_NS = 500_000_000  # Each press of SPACE permits one direction.
WATCHDOG_MARGIN_MS = 100
RAMP_STEP = 4                    # MCU duty increment per ~10 ms update.


def wheel_polarities(text):
    values = text.split(",")
    if len(values) != 4 or any(v not in ("+1", "-1") for v in values):
        raise argparse.ArgumentTypeError(
            "expected explicit FL,FR,RL,RR signs like +1,+1,+1,+1")
    return tuple(int(v) for v in values)


class OneShotEnable:
    """A direction cannot move the robot without a recent LOCAL SPACE key."""

    def __init__(self):
        self._deadline_ns = None

    def press_space(self, now_ns):
        self._deadline_ns = now_ns + ENABLE_WINDOW_NS

    def consume(self, direction, now_ns):
        deadline = self._deadline_ns
        self._deadline_ns = None  # Consume even on invalid / expired input.
        return (direction in DIRECTIONS and deadline is not None
                and 0 <= deadline - now_ns <= ENABLE_WINDOW_NS)


class MotorSequenceLedger:
    """Persistent, exclusive write-ahead u32 command sequence for MCU sessions.

    The STM32 remembers its last accepted sequence across USB reconnects.
    Record each claimed number before transmitting, even if USB then fails.
    A Podman volume must persist this file between container instances.
    """

    def __init__(self, path):
        if not path or not os.path.isabs(path):
            raise ValueError("sequence state needs an absolute persistent path")
        self.file = open(path, "a+", encoding="ascii")
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.file.seek(0)
            raw = self.file.read().strip()
            if raw and (not raw.isdecimal() or int(raw) > 0xffffffff):
                raise ValueError("corrupt persistent motor sequence state")
            # Preserve progress even when the system clock jumps backwards.
            self.current = max(int(time.time()), int(raw) if raw else 0)
            if self.current >= 0xffff0000:
                raise RuntimeError("MCU sequence nearly exhausted: inhibit")
        except BaseException:
            self.close()
            raise

    def claim(self):
        if self.current >= 0xffff0000:
            raise RuntimeError("MCU sequence nearly exhausted: inhibit")
        next_seq = self.current + 1
        self.file.seek(0)
        self.file.write(str(next_seq) + "\n")
        self.file.truncate()
        self.file.flush()
        os.fsync(self.file.fileno())
        self.current = next_seq
        return next_seq

    def close(self):
        if not self.file.closed:
            self.file.close()


class ManualPulseProtocol:
    """One serial owner; a failed ACK permanently inhibits further motion."""

    def __init__(self, mcu, *, first_sequence, pulse_ms, sequence_ledger=None):
        if (type(first_sequence) is not int
                or not 0 < first_sequence < 0xffff0000):
            raise ValueError("invalid initial MCU sequence")
        if type(pulse_ms) is not int or not 20 <= pulse_ms <= MAX_PULSE_MS:
            raise ValueError("pulse outside 20..150 ms")
        self.mcu = mcu
        self.sequence = first_sequence
        self.sequence_ledger = sequence_ledger
        self.pulse_ms = pulse_ms
        self.faulted = False

    def _transaction(self, command, expected, timeout_s=0.25):
        try:
            payload = (command + "\n").encode("ascii")
            if self.mcu.write(payload) != len(payload):
                raise RuntimeError("short CDC write")
            self.mcu.flush()
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                response = self.mcu.readline().decode("ascii", errors="replace").strip()
                if response == expected:
                    return
                if response.startswith(("error ", "event watchdog_stop")):
                    raise RuntimeError("MCU rejected command: " + response)
            raise RuntimeError("missing exact MCU ACK: " + expected)
        except Exception:
            self.faulted = True
            raise

    def _next_seq(self):
        try:
            if self.sequence_ledger is not None:
                self.sequence = self.sequence_ledger.claim()
            else:
                if self.sequence >= 0xffffffff:
                    raise RuntimeError("MCU sequence exhausted")
                self.sequence += 1
            return self.sequence
        except Exception:
            self.faulted = True
            raise

    def stop(self):
        # STOP is still attempted after a fault. MCU STOP disables drivers
        # even when the sequence is stale; an ACK isn't physical stopping.
        seq = self._next_seq()
        self._transaction(f"stop_wheels_tx {seq}",
                          f"ok stop_wheels_tx seq={seq}")

    def configure(self):
        self.stop()  # First action after USB attach must request STOP.
        self._transaction(f"set_watchdog_ms {self.pulse_ms + WATCHDOG_MARGIN_MS}",
                          "ok set_watchdog_ms")
        self._transaction(f"set_platform_ramp {RAMP_STEP}",
                          "ok set_platform_ramp")

    def drive(self, wheel_pwm):
        """One bounded hold-mode update, ACKed; MCU watchdog needs refresh.

        No keepalive is ever sent. Caller must STOP on key release, and MCU
        watchdog disables wheel outputs if caller stalls/crashes.
        """
        if self.faulted:
            raise RuntimeError("motor session latched after CDC fault")
        if (type(wheel_pwm) is not tuple or len(wheel_pwm) != 4
                or any(type(x) is not int or abs(x) > MAX_PWM for x in wheel_pwm)
                or not any(wheel_pwm)):
            raise ValueError("invalid bounded hold-mode PWM")
        seq = self._next_seq()
        self._transaction(
            f"set_wheels_tx {seq} " + " ".join(map(str, wheel_pwm)),
            f"ok set_wheels_tx seq={seq}", timeout_s=0.06)

    def pulse(self, wheel_pwm):
        if self.faulted:
            raise RuntimeError("motor session latched after CDC fault")
        if (type(wheel_pwm) is not tuple or len(wheel_pwm) != 4
                or any(type(x) is not int or abs(x) > MAX_PWM for x in wheel_pwm)
                or not any(wheel_pwm)):
            raise ValueError("invalid bounded four-wheel pulse")
        seq = self._next_seq()
        started = time.monotonic()
        try:
            self._transaction(
                f"set_wheels_tx {seq} " + " ".join(map(str, wheel_pwm)),
                f"ok set_wheels_tx seq={seq}", timeout_s=0.04)
        finally:
            # No keepalive. Even on lost motion ACK, request STOP and let
            # the short MCU watchdog expire if Linux freezes/disconnects.
            try:
                remaining = self.pulse_ms / 1000.0 - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
            finally:
                # SIGINT during the pulse delay must not skip STOP.
                self.stop()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument(
        "--sequence-state-file", required=True,
        help="absolute path in a persistently mounted Podman volume; "
             "prevents stale MCU sequences after restarting container")
    parser.add_argument("--pwm", type=int, default=32)
    parser.add_argument("--duration-ms", type=int, default=150)
    parser.add_argument(
        "--keyboard-device", default=None,
        help="enable hold-to-drive from grabbed Linux /dev/input/eventN "
             "keyboard; without it use legacy SPACE+key pulses")
    parser.add_argument("--wheel-polarities", type=wheel_polarities,
                        required=True, help="explicit FL,FR,RL,RR polarity")
    opts = parser.parse_args(args)
    if not opts.port.startswith("/dev/ttyACM"):
        parser.error("commission only a real /dev/ttyACM* device")
    if not 1 <= opts.pwm <= MAX_PWM:
        parser.error(f"pwm must be 1..{MAX_PWM}")
    if not 20 <= opts.duration_ms <= MAX_PULSE_MS:
        parser.error("duration-ms must be 20..150")

    if opts.keyboard_device is not None:
        from leroy_bridge.motor_keyboard_hold import run_hold
        return run_hold(opts)

    import serial
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int16MultiArray, String

    # Deliberately no ROS motor command subscriber. Unauthenticated ROS
    # traffic cannot trigger a pulse; only local /dev/tty key input can.
    class CommissionNode(Node):
        def __init__(self, *, tfd, controller):
            super().__init__("leroy_motor_manual_commission")
            self.controller = controller
            self.tfd = tfd
            self.enabled = OneShotEnable()
            self._requested = self.create_publisher(
                Int16MultiArray, "/leroy/manual/requested_pwm", 10)
            self._status = self.create_publisher(
                String, "/leroy/manual/status", 10)
            self._timer = self.create_timer(0.02, self._poll_tty)
            self.get_logger().warning(
                "MANUAL PULSES ONLY, NO OBSTACLE AVOIDANCE. "
                "SPACE then w/s/a/d/q/e for one <=150ms pulse; x = stop, Ctrl-C = exit.")

        def _report(self, status, pwm=(0, 0, 0, 0)):
            self._requested.publish(Int16MultiArray(data=list(pwm)))
            self._status.publish(String(data=status))

        def _poll_tty(self):
            if not select.select([self.tfd], [], [], 0)[0]:
                return
            key = os.read(self.tfd, 1).decode("ascii", errors="ignore")
            now = time.monotonic_ns()
            if key == " ":
                self.enabled.press_space(now)
                self._report("ONE_PULSE_ENABLED_500MS")
            elif key == "x":
                self.enabled = OneShotEnable()
                try:
                    self.controller.stop()
                    self._report("STOP_ACK_NOT_PHYSICAL_STOP")
                except Exception as exc:
                    self._fatal(exc)
            elif self.enabled.consume(key, now):
                # Directions are nominal and MUST be visually checked on
                # supports; signed motor polarities are operator supplied.
                pwm = tuple(opts.pwm * a * b for a, b in zip(
                    DIRECTIONS[key], opts.wheel_polarities))
                try:
                    self._report("PULSE_REQUESTED_" + key, pwm)
                    self.controller.pulse(pwm)
                    self._report("STOP_ACK_NOT_PHYSICAL_STOP")
                except Exception as exc:
                    self._fatal(exc)
                finally:
                    # Never execute a backlog of held/repeated direction keys.
                    termios.tcflush(self.tfd, termios.TCIFLUSH)

        def _fatal(self, exc):
            self._report("FAULT_CUTOFF_REQUIRED")
            self.get_logger().error(
                f"Motor session inhibited: {exc}. OPERATE HARDWIRED CUTOFF if needed.")
            rclpy.shutdown()

    # No nonzero command is possible until a deliberate local confirmation.
    with open("/dev/tty", "r", encoding="ascii") as tty_in, \
         open("/dev/tty", "w", encoding="ascii") as tty_out:
        tty_out.write(
            "Stop the normal ROS bridge: this process must be sole USB owner.\n"
            "SECURELY SUPPORT robot, all wheels clear of people/tools/floor.\n"
            "PHYSICALLY VERIFY hardwired cutoff removes motor-driver power\n"
            "even while STM32 remains powered by laptop USB.\n"
            "Directions/polarities and physical stopping are NOT yet validated.\n"
            "This ROS node has NO obstacle protection or SLAM requirement.\n"
            "Type MANUAL_PULSE_READY (exactly) to start: ")
        tty_out.flush()
        if tty_in.readline().strip() != "MANUAL_PULSE_READY":
            raise SystemExit("Manual motor commissioning declined")
        original = termios.tcgetattr(tty_in.fileno())
        rclpy.init(args=[])
        node = None
        mcu = None
        controller = None
        ledger = None
        try:
            ledger = MotorSequenceLedger(opts.sequence_state_file)
            # Cooperative exclusive lock requires normal bridge ALSO use
            # pyserial exclusive=True. Do not run concurrently in any case.
            mcu = serial.Serial(opts.port, baudrate=115200, timeout=0.025,
                                write_timeout=0.10, exclusive=True)
            time.sleep(0.20)
            mcu.reset_input_buffer()
            controller = ManualPulseProtocol(
                mcu, first_sequence=ledger.current,
                pulse_ms=opts.duration_ms, sequence_ledger=ledger)
            controller.configure()
            tty.setcbreak(tty_in.fileno())
            node = CommissionNode(tfd=tty_in.fileno(), controller=controller)
            rclpy.spin(node)
        finally:
            termios.tcsetattr(tty_in.fileno(), termios.TCSADRAIN, original)
            if mcu is not None:
                try:
                    # Best effort on exit. The cutoff, not a software ACK,
                    # is the independent final protection.
                    controller.stop()
                except Exception as exc:
                    print(f"STOP_UNCONFIRMED: {exc}; use physical cutoff")
                mcu.close()
            if ledger is not None:
                ledger.close()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
