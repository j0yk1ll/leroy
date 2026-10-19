#!/usr/bin/env python3
"""M23R: opt-in no-device loader-lifecycle trace around M23P harness.

Reuse M23P verbatim for strict profile gating, graph/observer checks,
900-image synthetic run and fail-closed shutdown exit assessment.
Only replace its launch path with M23R's traced RectifyNode component.
"""
import json
import os
from pathlib import Path
import re
import sys

TRACE_LAUNCH = "/opt/m23r/m23r_rectifier_trace.launch.py"
TRACE_LIB = "/opt/m23r/libm23r_loader_trace.so"

EVENT_RE = re.compile(
    r"M23R_TRACE_EVENT=(CONSTRUCTOR|DESTRUCTOR|DLOPEN_RETURN|"
    r"DLCLOSE_BEGIN|DLCLOSE_RETURN) "
    r"mono_ns=([0-9]+) pid=([0-9]+) tid=([0-9]+) "
    r"handle=([^ ]+) status=(-?[0-9]+) module=(.{1,400})$"
)

STAGES = (
    "CONSTRUCTOR", "DESTRUCTOR", "DLOPEN_RETURN",
    "DLCLOSE_BEGIN", "DLCLOSE_RETURN",
)
LAUNCH_MARKERS = (
    "M23R_LAUNCH_EVENT=CONTAINER_PROCESS_START",
    "M23R_LAUNCH_EVENT=CONTAINER_PROCESS_EXIT",
    "M23R_LAUNCH_EVENT=SHUTDOWN_REQUEST",
)


def summarize_trace(log):
    """Parse bounded loader markers only; do not infer which library crashed."""
    counts = {name: 0 for name in STAGES}
    process_ids = set()
    last_event = None
    constructor_status = None
    for line in log.splitlines():
        if "M23R_TRACE_EVENT=" not in line:
            continue
        payload = line[line.find("M23R_TRACE_EVENT="):].strip()
        match = EVENT_RE.fullmatch(payload)
        if match is None:
            raise ValueError("malformed or truncated M23R trace event")
        stage, ns, pid, tid, handle, status, module = match.groups()
        if not 0 < int(ns) or not 0 < int(pid) or not 0 < int(tid):
            raise ValueError("nonpositive trace timestamp or PID")
        if not (handle.startswith("0x") or handle == "(nil)"):
            raise ValueError("malformed traced loader handle")
        counts[stage] += 1
        if stage == "CONSTRUCTOR":
            constructor_status = int(status)
        process_ids.add(pid)
        last_event = {
            "stage": stage,
            "module_basename": Path(module).name[:100],
            "status": int(status),
            "monotonic_ns": int(ns),
        }
    if len(process_ids) != 1 or counts["CONSTRUCTOR"] != 1:
        raise ValueError("missing or duplicate component-only preload trace")
    if constructor_status != 0:
        raise ValueError("loader trace interposition not fully installed")
    if counts["DLCLOSE_RETURN"] > counts["DLCLOSE_BEGIN"]:
        raise ValueError("dlclose return without corresponding begin")
    if (counts["DLOPEN_RETURN"] == 0
            and counts["DLCLOSE_BEGIN"] == 0):
        raise ValueError("no dynamic loader activity was captured")
    markers = {name: log.count(name) for name in LAUNCH_MARKERS}
    if markers[LAUNCH_MARKERS[0]] != 1:
        raise ValueError("missing or duplicate component process-start event")
    return {
        "event_counts": counts,
        "launch_event_counts": markers,
        "last_loader_event": last_event,
        "dlclose_begun_not_returned":
            counts["DLCLOSE_BEGIN"] - counts["DLCLOSE_RETURN"],
        "process_count": len(process_ids),
        "diagnostic_only": True,
        "originating_caller_verified": False,
        "library_unload_cause_verified": False,
    }


def preconditions(environ):
    if environ.get("M23R_TRACE_ONLY") != "1":
        raise ValueError("M23R_TRACE_ONLY must be 1")
    if environ.get("LD_PRELOAD"):
        raise ValueError("global LD_PRELOAD forbidden; scope to child launch")
    if not Path(TRACE_LAUNCH).is_file() or not Path(TRACE_LIB).is_file():
        raise ValueError("M23R trace launch/library missing")
    # The M23P runner validates its own isolation settings, profile,
    # physical-device visibility and graph-QoS invariants unchanged.


def run():
    preconditions(os.environ)
    import m23p_no_device_rectifier_shutdown as m23p
    # Do not edit M23P or production; use its exactly preserved runner.
    m23p.LAUNCH = TRACE_LAUNCH
    status = m23p.run()
    trace_summary = {}
    faults = []
    raw_log = ""
    try:
        raw_log = m23p.LAUNCH_LOG.read_text(errors="replace")
        trace_summary = summarize_trace(raw_log)
    except (OSError, ValueError) as exc:
        faults.append("TRACE_NOT_VERIFIED:" + str(exc)[:150])

    # Preserve a bounded final trace window outside the ephemeral container.
    # These are diagnostic strings from the isolated component process,
    # not raw memory. Do not infer causality from the final dlclose event.
    if raw_log:
        trace_lines = [
            line[line.find("M23R_TRACE_EVENT="):].strip()
            for line in raw_log.splitlines()
            if "M23R_TRACE_EVENT=" in line
        ]
        print("M23R_TRACE_CAPTURED_TOTAL=" + str(len(trace_lines)),
              flush=True)
        print("M23R_TRACE_CAPTURED_WINDOW=LAST_80", flush=True)
        for line in trace_lines[-80:]:
            print("M23R_TRACE_CAPTURED=" + line, flush=True)

    verdict = (
        "BLOCKED_COMPONENT_LIFECYCLE" if status != 0
        else "CLEAN_OBSERVED_UNDER_TRACE_ONLY"
    )
    if faults:
        verdict = "BLOCKED_TRACE_EVIDENCE_INCOMPLETE"
    result = {
        "milestone": "M23R",
        "profile": os.environ.get("M23P_PROFILE"),
        "verdict": verdict,
        "m23p_runner_exit": status,
        "trace": trace_summary,
        "faults": faults,
        "device_access": "NONE",
        "real_camera_validated": False,
        "module_root_cause_verified": False,
        "physical_motion_authorized": False,
        "production_change_authorized": False,
    }
    print("M23R_TRACE_JSON=" + json.dumps(result, sort_keys=True), flush=True)
    print("M23R_LIFECYCLE_VERDICT=" + verdict, flush=True)
    print("M23R_DEVICE_ACCESS=NONE", flush=True)
    print("M23R_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 2 if status != 0 or faults else 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23R_PRECONDITION_BLOCKED=" + str(exc)[:200],
              file=sys.stderr, flush=True)
        sys.exit(2)
