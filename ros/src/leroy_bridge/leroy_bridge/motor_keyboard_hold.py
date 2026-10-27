"""Bounded hold-to-drive keyboard commissioning (Linux evdev, no mapping/SLAM).

A physical keyboard is grabbed exclusively to keep background desktop typing
from issuing motor commands. Unlike a terminal, evdev exposes actual key-up.
The operator stays at the separate hardwired motor-power cutoff.
ONLY for a robot securely supported with ALL wheels clear of the ground.
"""
import fcntl
import os
import re
import struct
import time

from leroy_bridge.motor_manual_commission import DIRECTIONS, ManualPulseProtocol

# Linux input.h: EVIOCGKEY(N), EVIOCGRAB and input-event-codes.h.
_EVENT = struct.Struct("@llHHi")
_EV_KEY = 1
_EV_SYN = 0
_SYN_DROPPED = 3
_GRAB = 0x40044590                  # _IOW('E', 0x90, int)
_KEY_BYTES = 96
_GET_KEYS = 0x80000000 | (_KEY_BYTES << 16) | (ord("E") << 8) | 0x18
_KEY_CODES = {"w": 17, "s": 31, "a": 30, "d": 32, "q": 16, "e": 18}
_ESC = 1
_X = 45
# No unbounded first motor-on run without measured floor stopping/coverage.
MAX_HOLD_NS = 3_000_000_000
MAX_TICK_GAP_NS = 180_000_000
RESEND_NS = 50_000_000


class KeyboardExit(Exception):
    """Local ESC exits and requests MCU STOP."""


class HeldKeyboard:
    """Only one physical key held; missing release/fault fails closed."""

    def __init__(self, device):
        if (type(device) is not str or
                not re.fullmatch(r"/dev/input/event[0-9]+", device)):
            raise ValueError("use a mapped /dev/input/eventN keyboard device")
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        self._grabbed = False
        try:
            # Do not allow keys typed into unrelated GUI windows to drive.
            fcntl.ioctl(self.fd, _GRAB, 1)
            self._grabbed = True
            self._inhibited = True
            self._started_ns = None
            self._last_direction = None
            self._keys()     # Fail at startup if this is not a readable evdev.
        except Exception:
            self.close()
            raise

    def _keys(self):
        bits = fcntl.ioctl(self.fd, _GET_KEYS, bytes(_KEY_BYTES))
        return {key for key, code in _KEY_CODES.items()
                if bits[code // 8] & (1 << (code % 8))}

    def inhibit(self):
        self._inhibited = True
        self._started_ns = None
        self._last_direction = None

    def read(self, now_ns):
        # Drain events, rejecting loss of synchronization. The kernel's
        # current EVIOCGKEY bitmap is re-read on EVERY tick, so a lost
        # release event cannot leave a key latched in a user-space cache.
        while True:
            try:
                data = os.read(self.fd, _EVENT.size * 64)
            except BlockingIOError:
                break
            if not data:
                raise OSError("keyboard device disconnected")
            if len(data) % _EVENT.size:
                raise RuntimeError("partial Linux input_event read")
            for _, _, kind, code, value in _EVENT.iter_unpack(data):
                if kind == _EV_SYN and code == _SYN_DROPPED:
                    raise RuntimeError("keyboard event stream overrun")
                if kind == _EV_KEY and value == 1:
                    if code == _ESC:
                        raise KeyboardExit()
                    if code == _X:
                        raise RuntimeError("local X stop: restart to rearm")
        held = self._keys()
        if not held:
            self._inhibited = False
            self._started_ns = None
            self._last_direction = None
            return None
        if self._inhibited:
            return None
        if len(held) != 1:
            self.inhibit()
            return None
        direction = next(iter(held))
        if self._last_direction is not None and direction != self._last_direction:
            # Never reverse/change trajectory without all direction keys up.
            self.inhibit()
            return None
        if self._started_ns is None:
            self._started_ns = now_ns
        self._last_direction = direction
        if not 0 <= now_ns - self._started_ns <= MAX_HOLD_NS:
            self.inhibit()
            return None
        return direction

    def close(self):
        if getattr(self, "fd", None) is not None:
            fd = self.fd
            self.fd = None
            try:
                if self._grabbed:
                    # EVIOCGRAB tests its argument VALUE, not a pointer to an
                    # integer. Supplying packed zero incorrectly re-grabbed
                    # the device and raised EBUSY on exit.
                    try:
                        fcntl.ioctl(fd, _GRAB, 0)
                    except OSError:
                        pass  # closing fd also ungrabs in the Linux kernel
            finally:
                os.close(fd)


def run_hold(opts):
    import serial
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int16MultiArray, String

    class HoldNode(Node):
        def __init__(self, controller, keyboard):
            super().__init__("leroy_motor_manual_commission")
            self.controller = controller
            self.keyboard = keyboard
            self.active = None
            self.last_sent_ns = None
            self.last_tick_ns = None
            self._requested = self.create_publisher(
                Int16MultiArray, "/leroy/manual/requested_pwm", 10)
            self._status = self.create_publisher(
                String, "/leroy/manual/status", 10)
            self.create_timer(0.02, self.tick)
            self.get_logger().warning(
                "WHEELS-RAISED HOLD MODE, NO OBSTACLE PROTECTION. "
                "Hold one W/A/S/D/Q/E; release=STOP; X=latch STOP; ESC=exit. "
                "Max 3s per press; release fully to reenable.")

        def report(self, state, pwm=(0, 0, 0, 0)):
            self._requested.publish(Int16MultiArray(data=list(pwm)))
            self._status.publish(String(data=state))

        def stop_active(self):
            if self.active is not None:
                self.active = None
                self.last_sent_ns = None
                self.controller.stop()
                self.report("STOP_ACK_NOT_PHYSICAL_STOP")

        def fail(self, reason):
            self.report("FAULT_CUTOFF_REQUIRED")
            self.get_logger().error(
                f"{reason}; motion inhibited. USE HARDWIRED CUTOFF if needed.")
            self.keyboard.inhibit()
            try:
                self.controller.stop()
            except Exception as exc:
                self.get_logger().error(f"STOP_UNCONFIRMED: {exc}")
            self.active = None
            rclpy.shutdown()

        def tick(self):
            now = time.monotonic_ns()
            if (self.last_tick_ns is not None
                    and now - self.last_tick_ns > MAX_TICK_GAP_NS):
                self.fail("ROS control loop stalled")
                return
            self.last_tick_ns = now
            try:
                direction = self.keyboard.read(now)
                if direction is None:
                    self.stop_active()
                    return
                if (direction == self.active and self.last_sent_ns is not None
                        and now - self.last_sent_ns < RESEND_NS):
                    return
                values = tuple(
                    opts.pwm * sign * polarity
                    for sign, polarity in zip(
                        DIRECTIONS[direction], opts.wheel_polarities))
                self.controller.drive(values)
                self.last_sent_ns = time.monotonic_ns()
                if self.active != direction:
                    self.report("HELD_" + direction, values)
                self.active = direction
            except KeyboardExit:
                try:
                    self.stop_active()
                finally:
                    rclpy.shutdown()
            except Exception as exc:
                self.fail(exc)

    with open("/dev/tty", "r", encoding="ascii") as tty_in, \
         open("/dev/tty", "w", encoding="ascii") as tty_out:
        tty_out.write(
            "HOLD-TO-DRIVE commissioning (not obstacle-aware).\n"
            "All FOUR WHEELS MUST BE SECURELY RAISED OFF THE FLOOR.\n"
            "Physically VERIFY independent hardwired motor-power cutoff\n"
            "works even while STM32 is powered via USB.\n"
            "Selected keyboard will be EXCLUSIVELY GRABBED: its usual\n"
            "desktop/terminal shortcuts will stop working until ESC exits.\n"
            "Hold one W/A/S/D/Q/E; release=STOP, X=latch STOP, ESC=exit.\n"
            "Duty defaults to 32/255; no floor movements authorized.\n"
            "Type HOLD_MOTORS_READY (exactly) to continue: ")
        tty_out.flush()
        if tty_in.readline().strip() != "HOLD_MOTORS_READY":
            raise SystemExit("Operator declined hold-mode commissioning")

    rclpy.init(args=[])
    keyboard = None
    controller = None
    ledger = None
    mcu = None
    node = None
    try:
        # Exclusive evdev grab must succeed before opening motion-capable CDC.
        keyboard = HeldKeyboard(opts.keyboard_device)
        mcu = serial.Serial(opts.port, baudrate=115200, timeout=0.02,
                            write_timeout=0.08, exclusive=True)
        time.sleep(0.20)
        mcu.reset_input_buffer()
        # Configure a bounded 200ms MCU watchdog, independent of hold length.
        from leroy_bridge.motor_manual_commission import MotorSequenceLedger
        ledger = MotorSequenceLedger(opts.sequence_state_file)
        controller = ManualPulseProtocol(
            mcu, first_sequence=ledger.current, pulse_ms=100,
            sequence_ledger=ledger)
        controller.configure()
        node = HoldNode(controller, keyboard)
        from rclpy.executors import ExternalShutdownException
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        # Always attempt STOP even after input/ROS/CDC faults. A STOP ACK
        # does not establish physical stopping; the cutoff is independent.
        if controller is not None:
            try:
                controller.stop()
            except Exception as exc:
                print(f"STOP_UNCONFIRMED: {exc}; operate physical cutoff")
        if keyboard is not None:
            keyboard.close()
        if ledger is not None:
            ledger.close()
        if mcu is not None:
            mcu.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
