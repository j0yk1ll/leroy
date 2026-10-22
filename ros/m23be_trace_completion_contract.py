#!/usr/bin/env python3
"""M23BE: OFFLINE ONLY proposed clean-trace closure contract.

M23AY/M23BC DO NOT emit M23BE_PUBLISHER_FINAL. Until a separately
reviewed source-only patch and disconnected build create that marker,
their preserved camera logs cannot pass this structural completeness gate.
An output here is NEVER a DDS-delivery or physical camera acceptance.
"""
import argparse
import json
from pathlib import Path
import re
import sys

import m23ba_event_correlation as m23ba

MAX_BYTES = 4_000_000
EPILOGUE = re.compile(
    r"^(?:\[[^\]]+\] )?M23BE_PUBLISHER_FINAL"
    r" attempts=(\d+) publish_returns=(\d+) shutdown=clean$")
CHILDREN = ("usb_cam_node_exe", "component_container")


class Blocked(ValueError):
    pass


def complete_camera_log(camera_text):
    if not isinstance(camera_text, str) or not camera_text:
        raise Blocked("empty camera log")
    lines = camera_text.splitlines()
    final = []
    for pos, line in enumerate(lines):
        if "M23BE_PUBLISHER_FINAL" in line:
            matched = EPILOGUE.fullmatch(line)
            if not matched:
                raise Blocked("malformed publisher completion marker")
            final.append((pos, int(matched.group(1)), int(matched.group(2))))
    if len(final) != 1:
        raise Blocked("missing or duplicate authoritative publisher completion marker")
    last_marker, total, returned = final[0]
    if total < 3 or total > m23ba.CAP or returned != total:
        raise Blocked("publisher completion totals invalid")
    if re.search(r"process has died|Segmentation fault|M23AX_FATAL=",
                 camera_text, re.IGNORECASE):
        raise Blocked("camera process crash or metadata overflow")
    presend = m23ba.parse_publisher(camera_text)
    if len(presend) != total:
        raise Blocked("publisher completion count differs from logged attempts")
    if any("M23AX_PUBLISH" in line for line in lines[last_marker + 1:]):
        raise Blocked("pre-send event after publisher completion")
    if any("M23AX_PUBLISH" in line for line in lines[:last_marker]) is False:
        raise Blocked("no pre-send events before publisher completion")
    # M23AU established that 'ros2 launch' may exit 0 while a child dies.
    # Demand exactly one actual clean child exit marker for both processes.
    for child in CHILDREN:
        hits = [line for line in lines
                if re.search(re.escape(child) + r"-[0-9]+", line)
                and "process has finished cleanly" in line]
        if len(hits) != 1:
            raise Blocked("missing or duplicated child clean exit: " + child)
    return presend


def complete_observer_log(observer_text):
    try:
        return m23ba.parse_observer(observer_text)
    except (m23ba.Blocked, ValueError) as exc:
        raise Blocked("observer completion invalid: " + str(exc)) from exc


def assess(camera_text, observer_text):
    presend = complete_camera_log(camera_text)
    observed = complete_observer_log(observer_text)
    raw_stamps = [stamp for stamp, _ in observed["raw"]]
    camera_stamps = [item.stamp_ns for item in presend]
    return {
        "verdict": "OFFLINE_STRUCTURAL_TRACE_CONTRACT_ONLY",
        "publisher_pre_send_attempts": len(presend),
        "publisher_publish_returns_reported": len(presend),
        "observer_counts": {key: len(values)
                            for key, values in observed.items()},
        "publisher_nonpositive_header_intervals":
            m23ba.nonpositive_intervals(camera_stamps),
        "raw_nonpositive_header_intervals":
            m23ba.nonpositive_intervals(raw_stamps),
        "publisher_final_marker_present": True,
        "subscriber_final_marker_present": True,
        "camera_and_rectifier_clean_exit_markers": True,
        "publisher_completion_marker_implemented_in_current_image": False,
        "publication_return_is_not_dds_delivery_ack": True,
        "observer_and_publisher_capture_epochs_proven_equal": False,
        "data_continuity_or_stamp_monotonicity_accepted": False,
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def read_source(path):
    if path.is_symlink() or not path.is_file():
        raise Blocked("trace must be a regular non-symlink text file")
    st = path.stat()
    if not 0 < st.st_size <= MAX_BYTES:
        raise Blocked("trace empty or exceeds size bound")
    content = path.read_text(encoding="utf-8")
    if path.stat().st_size != st.st_size:
        raise Blocked("trace size changed while reading")
    return content


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-log", type=Path)
    parser.add_argument("--observer-log", type=Path)
    args = parser.parse_args(argv)
    if args.camera_log is None and args.observer_log is None:
        print("M23BE_STATUS=INERT_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        if args.camera_log is None or args.observer_log is None:
            raise Blocked("both camera and observer trace files required")
        if args.camera_log.resolve() == args.observer_log.resolve():
            raise Blocked("camera/observer trace files must differ")
        result = assess(read_source(args.camera_log),
                        read_source(args.observer_log))
        print("M23BE_RESULT_JSON=" + json.dumps(result, sort_keys=True), flush=True)
        print("M23BE_VERDICT=OFFLINE_STRUCTURAL_TRACE_CONTRACT_ONLY", flush=True)
        print("M23BE_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, m23ba.Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23BE_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23BE_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
