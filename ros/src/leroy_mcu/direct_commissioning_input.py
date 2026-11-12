"""Direct physical-keyboard commissioning input.

No ROS, signing, sockets, MCU serial, or motor actuation.
A single explicit arm is allowed per process.
"""

import argparse
import select
import time

from host_deadman_input import DeadmanInput
from leroy_safety.attended_commissioning import AttendedCommissioning

ARM_WINDOW_NS = 10_000_000_000
REQUEST_PWM = (64, 64, 64, 64)
ZERO = (0, 0, 0, 0)


class DirectCommissioningInput:
    def __init__(self):
        self.deadman = DeadmanInput()
        self.policy = AttendedCommissioning()
        self.arm_key_down = False
        self.arm_used = False
        self.arm_requested_ns = None
        self.fault = False

    def inhibit(self):
        self.fault = True
        self.arm_requested_ns = None
        self.deadman.fail()
        return self.policy.disarm()

    def key_event(self, key, value, now_ns):
        if self.fault or value not in (0, 1, 2):
            return self.inhibit()

        if key == "space":
            was_held = self.deadman.held
            self.deadman.key_event(value)
            if value == 0:
                self.arm_requested_ns = None
                # Release inhibits immediately, before the next timer tick.
                return self.policy.disarm()
            if value == 1 and not was_held:
                requested = self.arm_requested_ns
                self.arm_requested_ns = None
                if requested is not None:
                    elapsed_ns = now_ns - requested
                    print(
                        f"Arm-to-Space interval: {elapsed_ns / 1e9:.2f}s",
                        flush=True,
                    )
                    if not 0 <= elapsed_ns <= ARM_WINDOW_NS:
                        print("Arming window expired", flush=True)
                        return self.inhibit()
                    return self.policy.arm(
                        now_ns=now_ns, operator_confirmed=True
                    )
            return None

        if key != "arm":
            return self.inhibit()

        if value == 0:
            self.arm_key_down = False
            return None

        if value == 2 or self.arm_key_down:
            return None

        self.arm_key_down = True

        if self.arm_used or self.deadman.held:
            return None

        self.arm_used = True
        self.arm_requested_ns = now_ns
        return None

    def tick(self, now_ns, actual_space_held):
        if self.fault:
            return self.policy.disarm()

        if self.arm_requested_ns is not None:
            if not 0 <= now_ns - self.arm_requested_ns <= ARM_WINDOW_NS:
                return self.inhibit()

        # The kernel's current key state is checked independently
        # of remembered press/release events.
        if self.deadman.held and not actual_space_held:
            return self.inhibit()

        return self.policy.tick(
            now_ns=now_ns,
            deadman_held=(
                self.deadman.permitted and actual_space_held
            ),
            requested_pwm=REQUEST_PWM,
        )


def monitor(device_path, seconds):
    """Display live safety decisions; never transmit motor commands."""
    from evdev import InputDevice, ecodes

    control = DirectCommissioningInput()

    with InputDevice(device_path) as device:
        if set(device.active_keys()) & {
            ecodes.KEY_A, ecodes.KEY_SPACE
        }:
            raise RuntimeError(
                "Release A and Space before starting commissioning input"
            )

        print("DIRECT COMMISSIONING — NON-ACTUATING")
        print("A: request arm; press Space within 10s; Ctrl+C: exit")
        print("No motor serial connection will be opened.", flush=True)

        deadline = time.monotonic() + seconds
        previous = None

        try:
            while time.monotonic() < deadline:
                ready, _, _ = select.select([device], [], [], 0.02)

                for active_device in ready:
                    for event in active_device.read():
                        if (event.type == ecodes.EV_SYN
                                and event.code == ecodes.SYN_DROPPED):
                            control.inhibit()
                            raise RuntimeError(
                                "Keyboard event stream overflow"
                            )

                        if event.type != ecodes.EV_KEY:
                            continue

                        if event.code == ecodes.KEY_SPACE:
                            key = "space"
                        elif event.code == ecodes.KEY_A:
                            key = "arm"
                        else:
                            continue

                        control.key_event(
                            key, event.value, time.monotonic_ns()
                        )

                held = ecodes.KEY_SPACE in set(device.active_keys())
                decision = control.tick(time.monotonic_ns(), held)
                status = (
                    decision.state,
                    decision.reason,
                    decision.proposed_pwm,
                    decision.stop_required,
                )

                if status != previous:
                    print(
                        f"state={status[0]} reason={status[1]} "
                        f"pwm_proposal={status[2]} "
                        f"stop={status[3]}",
                        flush=True,
                    )
                    previous = status

                if control.fault or decision.state == "FAULT":
                    break
        finally:
            control.inhibit()
            print("Commissioning input DISARMED.", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device", default="/dev/input/event2"
    )
    parser.add_argument(
        "--seconds", type=float, default=15.0
    )
    parser.add_argument(
        "--dry-run", action="store_true", required=True,
        help="Required: no motor output is implemented"
    )
    args = parser.parse_args()

    if not 0 < args.seconds <= 60:
        parser.error("--seconds must be between 0 and 60")

    monitor(args.device, args.seconds)


if __name__ == "__main__":
    main()
