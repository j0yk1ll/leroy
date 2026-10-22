#!/usr/bin/env python3
"""M23BB: fail-closed host-bind evidence paths for a FUTURE reviewed copy of M23K.

No ROS imports and no device access. Default CLI is inert. This module alone
does not launch anything or authorize a camera test. On future instrumented
M23K module import, staged_paths() checks exact private host mount before
the old subprocess supervisor can start.
"""
import argparse
import os
from pathlib import Path
import stat
import sys

ROOT = Path("/evidence")
SENTINEL = ".m23bb-host-bind-ready"
SENTINEL_CONTENT = b"M23BB_HOST_BIND_READY_V1\n"
CAMERA_FILENAME = "m23bb-camera-launch.log"
OBSERVER_FILENAME = "m23bb-counter.log"
REQUIRED = {
    "M23BB_HOST_TEXT_EVIDENCE_REVIEWED": "YES_NO_LIVE_AUTH",
    "M23BB_EVIDENCE_MOUNT": "/evidence",
}
UNSAFE_FS = {
    "tmpfs", "ramfs", "overlay", "proc", "sysfs", "devtmpfs",
    "squashfs", "fuse.overlayfs", "fuse.fuse-overlayfs",
}


class Blocked(ValueError):
    pass


def verify_mountinfo(mountinfo):
    """Require exactly one explicit /evidence mount, not the image overlay."""
    matched = []
    for line in mountinfo.splitlines():
        halves = line.split(" - ", 1)
        if len(halves) != 2:
            continue
        left, right = halves
        parts = left.split()
        after = right.split()
        if len(parts) >= 6 and parts[4] == str(ROOT) and len(after) >= 2:
            if "rw" not in parts[5].split(","):
                raise Blocked("evidence mount not read-write")
            if after[0] in UNSAFE_FS:
                raise Blocked("evidence storage is ephemeral: " + after[0])
            matched.append((parts[4], after[0]))
    if len(matched) != 1:
        raise Blocked("exact /evidence independent host mount not verified")
    return matched[0][1]


def verify_private_dir(root, *, uid, marker=None):
    if root.is_symlink() or not root.is_dir():
        raise Blocked("evidence directory missing or symlinked")
    directory = root.stat()
    if directory.st_uid != uid or stat.S_IMODE(directory.st_mode) != 0o700:
        raise Blocked("evidence owner or 0700 mode incorrect")
    if marker is None:
        marker = root / SENTINEL
    if marker.is_symlink() or not marker.is_file():
        raise Blocked("exclusive host mount sentinel missing")
    s = marker.stat()
    if s.st_uid != uid or stat.S_IMODE(s.st_mode) != 0o600:
        raise Blocked("host sentinel owner or 0600 mode incorrect")
    if marker.read_bytes() != SENTINEL_CONTENT:
        raise Blocked("host sentinel contents mismatch")
    for name in (CAMERA_FILENAME, OBSERVER_FILENAME):
        target = root / name
        if target.exists() or target.is_symlink():
            raise Blocked("evidence file already exists: " + name)
    return True


def staged_paths(*, env=None, mountinfo=None, root=ROOT, uid=None):
    """Return fixed exclusive-create output names only after all preflights."""
    if env is None:
        env = os.environ
    if uid is None:
        uid = os.geteuid()
    for name, value in REQUIRED.items():
        if env.get(name) != value:
            raise Blocked("host evidence preflight required: " + name)
    if root != ROOT and env is os.environ:
        raise Blocked("cannot redirect evidence path at runtime")
    if mountinfo is None:
        mountinfo = Path("/proc/self/mountinfo").read_text()
    verify_mountinfo(mountinfo)
    verify_private_dir(root, uid=uid)
    return root / CAMERA_FILENAME, root / OBSERVER_FILENAME


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    if not args.check:
        print("M23BB_STATUS=INERT_NO_DEVICE_NO_ROS", flush=True)
        return 0
    try:
        camera, counter = staged_paths()
        print("M23BB_SOURCE_ADAPTER_PREFLIGHT=HOST_MOUNT_READY", flush=True)
        print("M23BB_CAMERA_TEXT_PATH=" + str(camera), flush=True)
        print("M23BB_COUNTER_TEXT_PATH=" + str(counter), flush=True)
        print("M23BB_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, ValueError) as exc:
        print("M23BB_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180], flush=True)
        print("M23BB_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
