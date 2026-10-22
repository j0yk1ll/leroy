#!/usr/bin/env python3
"""M23BC: device-free synthetic host-bound text persistence / crash probe.

No ROS, no camera, no image pixels, no MCU. Default inert. Special --crash
terminates with a deliberate nonzero exit AFTER writing+fsync to two existing
private host-bound files, proving persistence across podman --rm.
"""
import argparse
import os
from pathlib import Path
import sys

from m23bb_host_evidence_adapter import Blocked, staged_paths

GATE = "M23BC_SYNTHETIC_ONLY_APPROVED"
GATE_VALUE = "YES_NO_ROS_NO_HARDWARE"
CAM = b"M23BC_SYNTHETIC_CAMERA_TEXT_A\nM23BC_SYNTHETIC_CAMERA_TEXT_B\n"
OBS = b"M23BC_SYNTHETIC_SUBSCRIBER_TEXT_A\nM23BC_SYNTHETIC_SUBSCRIBER_TEXT_B\n"
CRASH_EXIT = 73


def no_devices_visible(*, globber=None, bus=None):
    if globber is None:
        globber = lambda pattern: list(Path("/").glob(pattern.lstrip("/")))
    if bus is None:
        bus = lambda: Path("/dev/bus/usb").exists()
    for pattern in ("/dev/video*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        if list(globber(pattern)):
            raise Blocked("camera/serial device visible: " + pattern)
    if bus():
        raise Blocked("USB bus exposed")
    return True


def payloads():
    return CAM, OBS


def write_only_synthetic(*, crash=False, paths=None, env=None):
    if env is None:
        env = os.environ
    if env.get(GATE) != GATE_VALUE:
        raise Blocked("synthetic persistence approval gate missing")
    no_devices_visible()
    if paths is None:
        paths = staged_paths(env=env)
    camera, observer = paths
    if camera == observer or camera.parent != observer.parent:
        raise Blocked("camera and subscriber paths not independent in one mount")
    # Existing source uses exclusive-create 'xb'; no write/append fallback.
    # Host text files survive a disposable guest's termination even when
    # this subprocess intentionally avoids Python shutdown handlers.
    with camera.open("xb") as c, observer.open("xb") as o:
        c.write(CAM)
        c.flush()
        os.fsync(c.fileno())
        o.write(OBS)
        o.flush()
        os.fsync(o.fileno())
    if crash:
        os._exit(CRASH_EXIT)
    print("M23BC_SYNTHETIC_PERSISTENCE=FILES_WRITTEN", flush=True)
    print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    exclusive = p.add_mutually_exclusive_group()
    exclusive.add_argument("--success", action="store_true")
    exclusive.add_argument("--crash", action="store_true")
    args = p.parse_args(argv)
    if not (args.success or args.crash):
        print("M23BC_STATUS=INERT_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        return write_only_synthetic(crash=args.crash)
    except (Blocked, OSError, ValueError) as exc:
        print("M23BC_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
