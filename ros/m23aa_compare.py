#!/usr/bin/env python3
"""M23AA: device-free stock component versus generated standalone executable.

Both arms use the original M23Y drained publisher, M23P reliable observer,
nonzero-D source calibration and graph checks. No LD_PRELOAD. One arm per run.
"""
import json
import os
from pathlib import Path
import re
import sys
import time

LAUNCHES = {
    "component": "/opt/m23aa/m23aa_component_rectifier.launch.py",
    "standalone": "/opt/m23aa/m23aa_standalone_rectifier.launch.py",
}
EXIT_LINE = re.compile(
    r"\[(component_container|rectify_node)-[0-9]+\].*?"
    r"process has (finished cleanly|died).*?(?:exit code (-?[0-9]+))?"
)


def child_outcome(log, arm):
    """Require one unambiguous process exit, independently of ros2 launch."""
    prefix = "component_container" if arm == "component" else "rectify_node"
    events = []
    for line in log.splitlines():
        if "process has finished cleanly" not in line and "process has died" not in line:
            continue
        match = EXIT_LINE.search(line)
        if match and match.group(1) == prefix:
            # Parse code from entire line: optional regex groups can stop early.
            code_match = re.search(r"exit code (-?[0-9]+)", line)
            code = int(code_match.group(1)) if code_match else None
            events.append(("clean" if match.group(2) == "finished cleanly" else "died", code))
    if len(events) != 1:
        raise ValueError("missing or ambiguous rectifier child exit")
    kind, code = events[0]
    if kind == "clean" and code in (None, 0):
        return "CLEAN", 0
    if kind == "died" and code == -11:
        return "SIGSEGV", -11
    return "OTHER_EXIT", code


def wait_for_start(proc, path, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("rectifier exited before process-start marker")
        if path.is_file() and "M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_START" in path.read_text(errors="replace"):
            return
        time.sleep(0.2)
    raise RuntimeError("rectifier child start not verified")


def run():
    arm = os.environ.get("M23AA_ARM")
    if arm not in LAUNCHES:
        raise ValueError("M23AA_ARM must be component or standalone")
    if os.environ.get("M23Y_DRAIN_ONLY") != "1" or os.environ.get("M23S_DISTORTION_PROFILE") != "rectify":
        raise ValueError("original M23Y drained nonzero-D fixture required")
    if os.environ.get("LD_PRELOAD") or os.environ.get("M23R_TRACE_ONLY") != "0":
        raise ValueError("M23AA forbids loader tracing and global preload")
    for path in LAUNCHES.values():
        if not Path(path).is_file():
            raise ValueError("missing M23AA launch source: " + path)

    import m23p_no_device_rectifier_shutdown as m23p
    import m23s_rectification_ab as m23s
    import m23y_drained_ab as m23y
    from m23k_camera_path_probe import parse_counter_log

    m23p.selected_profile(os.environ)
    m23p.prohibit_devices()
    if os.environ.get("M23P_PROFILE") != "synthetic":
        raise ValueError("synthetic mode required")
    if not Path(m23y.PUBLISHER).is_file():
        raise ValueError("M23Y publisher not installed")

    previous = (
        m23p.LAUNCH, m23p.PUBLISHER, m23p.parse_m23p_publisher_log,
        m23p.wait_for_rectifier, m23p.assess_container_exit,
        m23p.inspect_synthetic_graph,
    )
    audit = {"verified": False}
    def inspect():
        result = previous[5]()
        audit["verified"] = True
        return result
    def check_pub(payload):
        result = m23s.validate_publisher_log(payload, "rectify", previous[2])
        m23y.validate_drain(payload)
        return result
    def assess(log, code):
        try:
            state, _ = child_outcome(log, arm)
        except ValueError:
            return "BLOCKED_RECTIFIER_CHILD_UNKNOWN"
        if state == "CLEAN" and code in (0, 130, -2):
            return "CLEAN_COMPONENT_SHUTDOWN"
        return "BLOCKED_RECTIFIER_CHILD_" + state

    m23p.LAUNCH = LAUNCHES[arm]
    m23p.PUBLISHER = m23y.PUBLISHER
    m23p.parse_m23p_publisher_log = check_pub
    m23p.wait_for_rectifier = wait_for_start
    m23p.assess_container_exit = assess
    m23p.inspect_synthetic_graph = inspect
    try:
        parent_code = m23p.run()
    finally:
        (m23p.LAUNCH, m23p.PUBLISHER, m23p.parse_m23p_publisher_log,
         m23p.wait_for_rectifier, m23p.assess_container_exit,
         m23p.inspect_synthetic_graph) = previous

    verdict = "BLOCKED_INCOMPLETE_EVIDENCE"
    child_code = None
    counts = {}
    try:
        log = m23p.LAUNCH_LOG.read_text(errors="replace")
        child, child_code = child_outcome(log, arm)
        published = check_pub(m23p.PUBLISHER_LOG.read_text(errors="replace"))
        observed = parse_counter_log(m23p.COUNTER_LOG.read_text(errors="replace"), require_rect=True)
        counts = observed["counts"]
        good = (
            audit["verified"]
            and published["image"] == published["info"] == 900
            and counts.get("raw") == counts.get("rect") == counts.get("info") == 900
            and observed["raw_rect_identical_header_stamp_pairs"] == 900
            and observed["rect_only_observed_header_stamps"] == 0
            and observed["streams"]["rect"]["header_intervals"]["gaps_over_250ms"] == 0
            and not observed["faults"]
        )
        if good and child == "CLEAN" and parent_code == 0:
            verdict = "CLEAN_STOCK_RECTIFIER_900_OF_900"
        elif good and child == "SIGSEGV" and parent_code == 2:
            verdict = "SIGSEGV_STOCK_RECTIFIER_900_OF_900"
    except (OSError, ValueError, KeyError, TypeError):
        pass

    print("M23AA_RESULT_JSON=" + json.dumps({
        "arm": arm, "verdict": verdict, "parent_exit": parent_code,
        "child_exit": child_code, "counts": counts,
        "graph_verified": audit["verified"], "stock_rectifier": True,
        "synthetic_only": True, "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AA_VERDICT=" + verdict, flush=True)
    print("M23AA_DEVICE_ACCESS=NONE", flush=True)
    print("M23AA_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0 if verdict == "CLEAN_STOCK_RECTIFIER_900_OF_900" else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError, RuntimeError) as exc:
        print("M23AA_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
