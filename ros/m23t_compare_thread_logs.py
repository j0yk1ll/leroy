#!/usr/bin/env python3
"""M23T: offline compare two saved synthetic component logs, no ROS or devices."""
import argparse
import json
from pathlib import Path
import sys

MAX_LOG_BYTES = 64 * 1024 * 1024
RECT_DISTORTION = "-0.08,0.01,0.001,-0.001,0"


def one_json(log, marker):
    values = [line[len(marker):] for line in log.splitlines()
              if line.startswith(marker)]
    if len(values) != 1:
        raise ValueError("missing or duplicate " + marker)
    data = json.loads(values[0])
    if not isinstance(data, dict):
        raise ValueError("invalid JSON object at " + marker)
    return data


def classify(log, profile):
    if profile not in ("default", "single"):
        raise ValueError("unexpected OpenCV profile")
    p = one_json(log, "M23P_LIFECYCLE_JSON=")
    r = one_json(log, "M23R_TRACE_JSON=")
    s = one_json(log, "M23S_AB_RESULT_JSON=")
    t = one_json(log, "M23T_THREADS_JSON=")
    if (log.count("M23T_DEVICE_ACCESS=NONE\n") != 1
            or log.count("M23T_PRODUCTION_ACCEPTANCE=BLOCKED\n") != 1):
        raise ValueError("missing distinct M23T no-device markers")
    if (p.get("device_access") != "NONE"
            or p.get("profile") != "synthetic"
            or p.get("actual_graph_qos_verified") is not True
            or p.get("synthetic_calibration_only") is not True
            or p.get("physical_motion_authorized") is not False
            or p.get("synthetic_publisher_exit_code") != 0
            or p.get("synthetic_observer_exit_code") != 0):
        raise ValueError("invalid M23P synthetic isolation or delivery")
    obs = p.get("observations") or {}
    counts = obs.get("synthetic_observer_counts") or {}
    if (obs.get("synthetic_published_image_count") != 900
            or obs.get("actual_graph_qos_verified") is not True
            or obs.get("synthetic_calibration") !=
                "FABRICATED_NONZERO_DISTORTION"
            or any(type(counts.get(k)) is not int or counts[k] < 810
                   for k in ("raw", "rect", "info"))
            or obs.get("unmatched_rectified_header_stamps") != 0
            or type(obs.get("raw_rect_identical_header_stamp_pairs")) is not int
            or obs["raw_rect_identical_header_stamp_pairs"] <
                0.90 * counts["rect"]
            or type(obs.get("synthetic_rect_max_header_gap_ms")) not in
                (float, int)
            or not 0 < obs["synthetic_rect_max_header_gap_ms"] <= 250):
        raise ValueError("rectification delivery was not proven")
    if (s.get("distortion_profile") != "rectify"
            or s.get("source_d") != RECT_DISTORTION
            or s.get("publisher_calibration_verified") is not True
            or s.get("faults") != []
            or s.get("device_access") != "NONE"
            or s.get("production_acceptance") != "BLOCKED"):
        raise ValueError("M23S same nonzero distortion provenance failed")
    if (r.get("device_access") != "NONE" or r.get("faults") != []
            or (r.get("trace") or {}).get("process_count") != 1
            or (r.get("trace") or {}).get("event_counts", {}).get(
                "CONSTRUCTOR") != 1
            or r.get("module_root_cause_verified") is not False):
        raise ValueError("M23R child tracer provenance failed")
    setup = t.get("opencv_setup") or {}
    if (t.get("thread_profile") != profile
            or t.get("distortion_profile") != "rectify"
            or t.get("source_d") != RECT_DISTORTION
            or t.get("source_component") != "image_proc::RectifyNode"
            or t.get("device_access") != "NONE"
            or t.get("production_acceptance") != "BLOCKED"
            or t.get("root_cause_verified") is not False
            or t.get("opencv_backend_verified") is not False
            or t.get("faults") != []
            or setup.get("profile") != profile
            or setup.get("threading_backend_verified") is not False
            or setup.get("actual_worker_threads_verified") is not False
            or type(setup.get("api_threads_before")) is not int
            or type(setup.get("api_threads_after")) is not int):
        raise ValueError("OpenCV child setup not verified")
    if profile == "single" and setup["api_threads_after"] != 1:
        raise ValueError("single-thread API setting not confirmed")
    if (profile == "default" and
            setup["api_threads_after"] != setup["api_threads_before"]):
        raise ValueError("default OpenCV API configuration changed")
    if (r.get("m23p_runner_exit") != s.get("m23r_runner_exit")
            or t.get("m23s_runner_exit") != s.get("m23r_runner_exit")):
        raise ValueError("inconsistent inherited exit statuses")
    status = p.get("component_exit_class")
    child_code = s.get("component_child_exit_code_if_reported")
    if (status == "CLEAN_COMPONENT_SHUTDOWN" and p.get("faults") == []
            and child_code is None
            and t.get("m23s_runner_exit") == 0
            and r.get("verdict") == "CLEAN_OBSERVED_UNDER_TRACE_ONLY"
            and t.get("verdict") == "CLEAN_UNDER_THREAD_PROBE_ONLY"):
        lifecycle = "CLEAN_IN_THIS_INSTRUMENTED_RUN"
    elif (status == "BLOCKED_RECTIFIER_CHILD_CRASH"
          and p.get("faults") == ["BLOCKED_RECTIFIER_CHILD_CRASH"]
          and child_code == -11
          and t.get("m23s_runner_exit") == 2
          and r.get("verdict") == "BLOCKED_COMPONENT_LIFECYCLE"
          and t.get("verdict") == "BLOCKED_M23T_INHERITED_GATES"):
        lifecycle = "SIGSEGV_CHILD_EXIT_MINUS11"
    else:
        raise ValueError("child exit not classifiable")
    return {
        "profile": profile,
        "opencv_api_before": setup["api_threads_before"],
        "opencv_api_after": setup["api_threads_after"],
        "rectified_images": counts["rect"],
        "matched_stamps": obs["raw_rect_identical_header_stamp_pairs"],
        "max_rect_gap_ms": obs["synthetic_rect_max_header_gap_ms"],
        "lifecycle": lifecycle,
        "opencv_backend_verified": False,
        "faulting_module_verified": False,
    }


def compare(a_log, b_log):
    a = classify(a_log, "default")
    b = classify(b_log, "single")
    pair = (a["lifecycle"], b["lifecycle"])
    verdicts = {
        ("SIGSEGV_CHILD_EXIT_MINUS11", "CLEAN_IN_THIS_INSTRUMENTED_RUN"):
            "THREAD_LIMIT_ASSOCIATION_SUPPORTED_NOT_CAUSAL",
        ("SIGSEGV_CHILD_EXIT_MINUS11", "SIGSEGV_CHILD_EXIT_MINUS11"):
            "THREAD_LIMIT_DOES_NOT_PREVENT_SIGSEGV_IN_PAIR",
        ("CLEAN_IN_THIS_INSTRUMENTED_RUN", "CLEAN_IN_THIS_INSTRUMENTED_RUN"):
            "BASELINE_CRASH_NOT_REPRODUCED_UNDER_PROBE",
        ("CLEAN_IN_THIS_INSTRUMENTED_RUN", "SIGSEGV_CHILD_EXIT_MINUS11"):
            "REVERSE_PATTERN_INCONCLUSIVE",
    }
    return {
        "milestone": "M23T", "pair_verdict": verdicts[pair],
        "default": a, "single": b,
        "one_pair_diagnostic_only": True,
        "causation_verified": False,
        "actual_worker_count_verified": False,
        "faulting_module_verified": False,
        "device_access": "NONE",
        "production_acceptance": "BLOCKED",
    }


def read_log(path):
    p = Path(path)
    if p.is_symlink() or not p.is_file() or not 0 < p.stat().st_size <= MAX_LOG_BYTES:
        raise ValueError("log missing, symlink or wrong size")
    return p.read_text(errors="replace")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--default-log", required=True)
    parser.add_argument("--single-log", required=True)
    args = parser.parse_args()
    try:
        if Path(args.default_log).resolve() == Path(args.single_log).resolve():
            raise ValueError("same log cannot be both profiles")
        result = compare(read_log(args.default_log), read_log(args.single_log))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("M23T_PAIR_BLOCKED=" + str(exc)[:200], flush=True)
        return 2
    print("M23T_PAIR_JSON=" + json.dumps(result, sort_keys=True), flush=True)
    print("M23T_PAIR_VERDICT=" + result["pair_verdict"], flush=True)
    print("M23T_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    print("M23T_DEVICE_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
