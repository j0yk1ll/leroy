#!/usr/bin/env python3
"""M23Z: one explicitly gated no-device mapping snapshot during M23Y raw arm.

No new synthetic camera, no new image processing, no production launch.
The M23Y/M23X/M23R/M23P validation stack remains in place, with only
the raw-arm component-scoped loader tracer launch swapped temporarily.
Raw virtual addresses are copied solely to the protected host run log.
"""
import os
from pathlib import Path
import re
import sys

LAUNCH = "/opt/m23z/m23z_raw_rectifier.launch.py"
BEGIN = "M23Z_MAP_BEGIN=YES"
END = re.compile(r"M23Z_MAP_END count=([0-9]+) truncated=([01])$")
ENTRY = re.compile(
    r"M23Z_EXEC_MAP start=0x([0-9a-f]+) end=0x([0-9a-f]+) "
    r"module=([A-Za-z0-9_.+-]{1,96}|<main>)$"
)
MAX_LAUNCH_BYTES = 2 * 1024 * 1024
MAX_MAPPINGS = 400


def parse_snapshot(payload):
    """Reject duplicates, truncation, malformed ranges, or missing order."""
    lines = payload.splitlines()
    starts = [i for i, line in enumerate(lines) if BEGIN in line]
    # ROS launch may prefix component stderr, so isolate the unique marker.
    ends = [(i, END.fullmatch(line[line.index("M23Z_MAP_END "):]))
            for i, line in enumerate(lines) if "M23Z_MAP_END " in line]
    if len(starts) != 1 or len(ends) != 1 or ends[0][1] is None:
        raise ValueError("missing or duplicate executable-map snapshot")
    stop, footer = ends[0]
    start = starts[0]
    first_close = [i for i, line in enumerate(lines)
                   if "M23R_TRACE_EVENT=DLCLOSE_BEGIN " in line]
    if not first_close or not first_close[0] < start < stop:
        raise ValueError("snapshot not captured before first dlclose")
    expected = int(footer.group(1))
    if footer.group(2) != "0" or not 1 <= expected <= MAX_MAPPINGS:
        raise ValueError("missing or truncated executable mapping census")
    entries = []
    for i, line in enumerate(lines):
        if "M23Z_EXEC_MAP " not in line:
            continue
        if not start < i < stop:
            raise ValueError("mapping outside snapshot boundaries")
        event = line[line.index("M23Z_EXEC_MAP "):]
        match = ENTRY.fullmatch(event)
        if match is None:
            raise ValueError("malformed executable mapping")
        lo, hi = int(match.group(1), 16), int(match.group(2), 16)
        if not 0 < lo < hi < (1 << 64):
            raise ValueError("invalid executable mapping bounds")
        entries.append((lo, hi, match.group(3)))
    if len(entries) != expected or len(entries) != len(set(entries)):
        raise ValueError("mapping census count or identity mismatch")
    return entries


def run():
    if (os.environ.get("M23Z_MAP_ONLY") != "1"
            or os.environ.get("M23X_ARM") != "raw"
            or os.environ.get("M23Y_DRAIN_ONLY") != "1"
            or os.environ.get("M23S_DISTORTION_PROFILE") != "rectify"):
        raise ValueError("M23Z requires explicit raw-only synthetic profile")
    if os.environ.get("LD_PRELOAD") or not Path(LAUNCH).is_file():
        raise ValueError("global preload or missing M23Z child-only launch")

    import m23x_allowlist_ab as m23x
    import m23y_drained_ab as m23y
    import m23p_no_device_rectifier_shutdown as m23p

    original_launch = m23x.ARMS["raw"]
    m23x.ARMS["raw"] = LAUNCH
    try:
        status = m23y.run()
    finally:
        m23x.ARMS["raw"] = original_launch

    try:
        if m23p.LAUNCH_LOG.stat().st_size > MAX_LAUNCH_BYTES:
            raise ValueError("component launch log exceeds cap")
        entries = parse_snapshot(m23p.LAUNCH_LOG.read_text(errors="replace"))
    except (OSError, ValueError) as exc:
        print("M23Z_SNAPSHOT=BLOCKED_" + type(exc).__name__, flush=True)
        print("M23Z_ROOT_CAUSE=UNRESOLVED", flush=True)
        return 2

    # These private mapping lines carry ASLR addresses. The host redirects
    # stdout to a 0600 file. Never paste them into GitHub or public chat.
    for lo, hi, module in entries:
        print("M23Z_PRIVATE_EXEC_MAP=0x%x,0x%x,%s" % (lo, hi, module),
              flush=True)
    print("M23Z_SNAPSHOT=VERIFIED_BEFORE_FIRST_CLOSE", flush=True)
    print("M23Z_EXECUTABLE_MAPS_CAPTURED=" + str(len(entries)), flush=True)
    print("M23Z_LIFECYCLE_RUN_EXIT=" + str(status), flush=True)
    print("M23Z_SNAPSHOT_IDENTIFIES_FAULTING_LIBRARY=FALSE", flush=True)
    print("M23Z_ROOT_CAUSE=UNRESOLVED", flush=True)
    print("M23Z_DEVICE_ACCESS=NONE", flush=True)
    print("M23Z_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return status


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23Z_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
