#!/usr/bin/env python3
"""M23T: single-variable OpenCV thread API setting in original RectifyNode.

Reuse unmodified M23S nonzero-distortion publisher, M23P lifecycle and
M23R scoped loader trace; child OpenCV setup is the only A/B variable.
No ROS invocation or process creation in this wrapper.
"""
import json
import os
from pathlib import Path
import re
import sys

THREAD_LAUNCH = "/opt/m23t/m23t_rectifier_thread_trace.launch.py"
THREAD_LIBRARY = "/opt/m23t/libm23t_opencv_thread_probe.so"
THREAD_PROFILES = ("default", "single")
MARKER = re.compile(
    r"M23T_OPENCV_THREAD_SETUP profile=(default|single) "
    r"before=([0-9]+) after=([0-9]+) pid=([0-9]+)$"
)
TRACE_PID = re.compile(
    r"M23R_TRACE_EVENT=CONSTRUCTOR .*? pid=([0-9]+) "
)


def selected_profile(environ):
    required = {
        "M23T_THREAD_PROBE_ONLY": "1",
        "M23S_AB_ONLY": "1",
        "M23S_DISTORTION_PROFILE": "rectify",
        "M23P_PROFILE": "synthetic",
        "M23P_SYNTHETIC_ONLY": "1",
        "M23R_TRACE_ONLY": "1",
        "M23K_PROBE_MODE": "selftest",
        "M23N_CAMERA_QOS_ONLY": "0",
        "M23O_RECT_QOS_ONLY": "0",
        "LEROY_NAV2_DRY_RUN": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }
    for name, expected in required.items():
        if environ.get(name) != expected:
            raise ValueError("M23T isolation prerequisite: " + name)
    if environ.get("LD_PRELOAD"):
        raise ValueError("global LD_PRELOAD forbidden")
    profile = environ.get("M23T_OPENCV_THREAD_PROFILE")
    if profile not in THREAD_PROFILES:
        raise ValueError("OpenCV thread profile must be default or single")
    return profile


def setup_from_log(log, expected):
    setups = []
    constructor_pids = []
    for line in log.splitlines():
        if "M23T_OPENCV_THREAD_SETUP" in line:
            payload = line[line.find("M23T_OPENCV_THREAD_SETUP"):].strip()
            match = MARKER.fullmatch(payload)
            if not match:
                raise ValueError("invalid OpenCV thread setup marker")
            setups.append(match.groups())
        if "M23R_TRACE_EVENT=CONSTRUCTOR" in line:
            match = TRACE_PID.search(line)
            if match:
                constructor_pids.append(int(match.group(1)))
    if len(setups) != 1 or len(constructor_pids) != 1:
        raise ValueError("not exactly one OpenCV setup and tracer constructor")
    profile, before, after, pid = setups[0]
    before, after, pid = int(before), int(after), int(pid)
    if (profile != expected or pid <= 0 or constructor_pids != [pid]
            or not 1 <= before <= 4096 or not 1 <= after <= 4096):
        raise ValueError("OpenCV setup wrong process, mode or invalid count")
    if expected == "single" and after != 1:
        raise ValueError("cv::setNumThreads(1) not verified by API")
    if expected == "default" and before != after:
        raise ValueError("default arm changed OpenCV thread setting")
    return {
        "profile": profile,
        "api_threads_before": before,
        "api_threads_after": after,
        "component_pid": pid,
        "actual_worker_threads_verified": False,
        "threading_backend_verified": False,
    }


def run():
    profile = selected_profile(os.environ)
    import m23p_no_device_rectifier_shutdown as m23p
    import m23r_shutdown_trace_runner as m23r
    import m23s_rectification_ab as m23s

    m23p.selected_profile(os.environ)
    m23p.prohibit_devices()
    m23r.preconditions(os.environ)
    m23s.selected_profile(os.environ)
    if not Path(THREAD_LAUNCH).is_file() or not Path(THREAD_LIBRARY).is_file():
        raise ValueError("M23T child launch/library is absent")

    previous_launch = m23r.TRACE_LAUNCH
    m23r.TRACE_LAUNCH = THREAD_LAUNCH
    try:
        status = m23s.run()
    finally:
        m23r.TRACE_LAUNCH = previous_launch

    faults = []
    setup = {}
    try:
        with m23p.LAUNCH_LOG.open("r", errors="replace") as stream:
            log = stream.read(4 * 1024 * 1024 + 1)
        if len(log) > 4 * 1024 * 1024:
            raise ValueError("component log exceeded 4 MiB limit")
        setup = setup_from_log(log, profile)
    except (OSError, ValueError) as exc:
        faults.append("M23T_OPENCV_SETUP_UNVERIFIED:" + str(exc)[:140])

    verdict = ("BLOCKED_M23T_THREAD_SETUP" if faults else
               "BLOCKED_M23T_INHERITED_GATES" if status != 0 else
               "CLEAN_UNDER_THREAD_PROBE_ONLY")
    result = {
        "milestone": "M23T",
        "thread_profile": profile,
        "distortion_profile": "rectify",
        "source_d": "-0.08,0.01,0.001,-0.001,0",
        "source_component": "image_proc::RectifyNode",
        "m23s_runner_exit": status,
        "opencv_setup": setup,
        "faults": faults,
        "verdict": verdict,
        "root_cause_verified": False,
        "opencv_backend_verified": False,
        "device_access": "NONE",
        "physical_motion_authorized": False,
        "production_acceptance": "BLOCKED",
    }
    print("M23T_THREADS_JSON=" + json.dumps(result, sort_keys=True), flush=True)
    print("M23T_THREADS_VERDICT=" + verdict, flush=True)
    print("M23T_DEVICE_ACCESS=NONE", flush=True)
    print("M23T_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 2 if status != 0 or faults else 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23T_PRECONDITION_BLOCKED=" + str(exc)[:180],
              file=sys.stderr, flush=True)
        sys.exit(2)
