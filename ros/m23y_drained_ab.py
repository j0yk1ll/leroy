#!/usr/bin/env python3
"""M23Y source-only experimental fixture selection for M23X's no-device A/B.

The original M23S/M23P publisher, ROS stock RectifyNode and both M23X
component launch definitions remain unchanged. Both A/B arms use the
*same* M23Y drained source, and inherit every M23X/M23S/M23R/M23P guard.
"""
import json
import os
from pathlib import Path
import sys

PUBLISHER = (
    "/opt/m23y-cpp-install/lib/leroy_m23y_drain_publisher/"
    "m23y_drained_publisher"
)
DRAIN_MARKERS = (
    "M23Y_PUBLISHER_DRAIN_STARTED=YES",
    "M23Y_PUBLISHER_DRAIN_ELAPSED_NS=",
    "M23Y_PUBLISHER_DRAIN_COMPLETED=YES",
)
MIN_DRAIN_NS = 1_500_000_000
MAX_DRAIN_NS = 10_000_000_000


def validate_drain(payload):
    """Pure function; verifies the publisher stayed alive after frame 900.

    A timer is NOT a DDS delivery acknowledgement. This validates fixture
    timing only, never declares raw/rectified delivery a success.
    """
    lines = payload.splitlines()
    positions = []
    for prefix in ("M23P_PUBLISHER_COUNTS=900,900",) + DRAIN_MARKERS:
        hits = [i for i, line in enumerate(lines) if
                (line == prefix if not prefix.endswith("=")
                 else line.startswith(prefix))]
        if len(hits) != 1:
            raise ValueError("missing or duplicate drain marker")
        positions.append(hits[0])
    if positions != sorted(set(positions)):
        raise ValueError("invalid final frame/drain event order")
    elapsed_line = lines[positions[2]]
    try:
        elapsed_ns = int(elapsed_line.split("=", 1)[1])
    except ValueError as exc:
        raise ValueError("invalid drain elapsed time") from exc
    if not MIN_DRAIN_NS <= elapsed_ns <= MAX_DRAIN_NS:
        raise ValueError("drain timer outside bounded interval")
    return elapsed_ns


def run():
    if os.environ.get("M23Y_DRAIN_ONLY") != "1":
        raise ValueError("M23Y_DRAIN_ONLY=1 is mandatory")
    if os.environ.get("M23S_DISTORTION_PROFILE") != "rectify":
        raise ValueError("nonzero-distortion fixture required")
    if os.environ.get("M23X_ARM") not in ("baseline", "raw"):
        raise ValueError("explicit M23X arm required")
    if not Path(PUBLISHER).is_file():
        raise ValueError("drained publisher binary missing")

    import m23s_rectification_ab as m23s
    import m23x_allowlist_ab as m23x
    import m23p_no_device_rectifier_shutdown as m23p

    original_publisher = m23s.PUBLISHER
    original_validator = m23s.validate_publisher_log

    def checked_publisher(payload, profile, original_parser):
        result = original_validator(payload, profile, original_parser)
        validate_drain(payload)
        return result

    m23s.PUBLISHER = PUBLISHER
    m23s.validate_publisher_log = checked_publisher
    try:
        status = m23x.run()
    finally:
        m23s.PUBLISHER = original_publisher
        m23s.validate_publisher_log = original_validator

    try:
        raw = m23p.PUBLISHER_LOG.read_text(errors="replace")
        elapsed = validate_drain(raw)
        print("M23Y_PUBLISHER_DRAIN=VERIFIED", flush=True)
        print("M23Y_PUBLISHER_DRAIN_ELAPSED_NS=" + str(elapsed), flush=True)
    except (OSError, ValueError):
        print("M23Y_PUBLISHER_DRAIN=BLOCKED", flush=True)
        status = 2

    print("M23Y_RESULT_JSON=" + json.dumps({
        "milestone": "M23Y",
        "arm": os.environ["M23X_ARM"],
        "inherited_m23x_exit_code": status,
        "production_acceptance": "BLOCKED",
        "real_camera_validated": False,
        "synthetic_fixture_only": True,
        "drain_is_dds_ack": False,
        "root_cause_verified": False,
    }, sort_keys=True), flush=True)
    print("M23Y_DEVICE_ACCESS=NONE", flush=True)
    print("M23Y_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return status


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23Y_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
