"""Local keyboard deadman input for attended commissioning.

Host-side input only. No ROS, sockets, serial or motor actuation.
A held key is not independently authenticated motor authorization.
"""

import argparse
import select
import time


class DeadmanInput:
    def __init__(self):
        self.held = False
        self.fault = False
        self.press_seen = False

    def key_event(self, value):
        """Linux EV_KEY: 0 release, 1 press, 2 autorepeat."""
        if self.fault:
            return
        if value == 1:
            if not self.held:
                self.press_seen = True
                self.held = True
        elif value == 0:
            self.held = False
        elif value == 2:
            # Autorepeat is not a fresh operator action.
            pass
        else:
            self.fail()

    def fail(self):
        self.held = False
        self.fault = True

    @property
    def permitted(self):
        return self.press_seen and self.held and not self.fault


def self_test():
    control = DeadmanInput()
    assert not control.permitted

    control.key_event(2)
    assert not control.permitted

    control.key_event(1)
    assert control.permitted

    for _ in range(100):
        control.key_event(2)
        assert control.permitted

    control.key_event(0)
    assert not control.permitted

    control.key_event(1)
    assert control.permitted

    control.fail()
    assert not control.permitted

    control.key_event(1)
    assert not control.permitted

    print("PASS: initial state inhibited")
    print("PASS: auto-repeat cannot initiate a hold")
    print("PASS: key release removes permission")
    print("PASS: input fault permanently inhibits session")


def monitor(device_path, seconds):
    from evdev import InputDevice, ecodes

    control = DeadmanInput()
    device = InputDevice(device_path)

    print(f"Input device: {device.path} ({device.name})")
    print("Hold Space to show HELD; release to show RELEASED.")
    print("Status only: no commands transmitted.", flush=True)

    deadline = time.monotonic() + seconds
    previous = None

    try:
        while time.monotonic() < deadline:
            ready, _, _ = select.select([device], [], [], 0.05)

            for active_device in ready:
                for event in active_device.read():
                    if (event.type == ecodes.EV_SYN
                            and event.code == ecodes.SYN_DROPPED):
                        control.fail()
                    elif (event.type == ecodes.EV_KEY
                            and event.code == ecodes.KEY_SPACE):
                        control.key_event(event.value)

            status = (
                "FAULT" if control.fault
                else "HELD" if control.permitted
                else "RELEASED"
            )

            if status != previous:
                print(f"Deadman: {status}", flush=True)
                previous = status

            if control.fault:
                break

    except (OSError, ValueError) as exc:
        control.fail()
        print(f"Input fault: {exc}", flush=True)
    finally:
        control.fail()
        device.close()
        print("Deadman: INHIBITED", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--device", default="/dev/input/event2")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    if args.self_test:
        self_test()
    else:
        monitor(args.device, args.seconds)


if __name__ == "__main__":
    main()
