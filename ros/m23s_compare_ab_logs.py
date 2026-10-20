#!/usr/bin/env python3
"""M23S: offline pair classification of two *existing* no-device logs.

No ROS, Podman, procfs, network, hardware, GDB or core access. Classify
differences in shutdown evidence, never diagnose a specific failing library.
"""
import argparse
import json
from pathlib import Path
import stat
import sys

LIMIT_BYTES = 64 * 1024 * 1024


def sole_json(payload, marker):
    matches = [line[len(marker):] for line in payload.splitlines()
               if line.startswith(marker)]
    if len(matches) != 1:
        raise ValueError("missing or duplicate JSON marker: " + marker)
    result = json.loads(matches[0])
    if not isinstance(result, dict):
        raise ValueError("JSON marker is not an object: " + marker)
    return result


def classify_run(payload, expected_profile):
    """Require same validated M23P delivery and M23R component trace in both."""
    if expected_profile not in ("passthrough", "rectify"):
        raise ValueError("invalid requested comparison profile")
    from m23s_rectification_ab import CALIBRATION, D_VECTORS
    p = sole_json(payload, "M23P_LIFECYCLE_JSON=")
    r = sole_json(payload, "M23R_TRACE_JSON=")
    s = sole_json(payload, "M23S_AB_RESULT_JSON=")
    if (payload.count("\nM23S_DEVICE_ACCESS=NONE") != 1
            or payload.count("\nM23S_PRODUCTION_ACCEPTANCE=BLOCKED") != 1):
        raise ValueError("missing/duplicate no-device and production block markers")
    if (p.get("profile") != "synthetic"
            or p.get("device_access") != "NONE"
            or p.get("synthetic_calibration_only") is not True
            or p.get("physical_motion_authorized") is not False):
        raise ValueError("M23P synthetic safety metadata invalid")
    if (r.get("profile") != "synthetic"
            or r.get("device_access") != "NONE"
            or r.get("module_root_cause_verified") is not False
            or r.get("physical_motion_authorized") is not False
            or r.get("faults") != []):
        raise ValueError("M23R trace evidence invalid")
    if (s.get("distortion_profile") != expected_profile
            or s.get("source_d") != D_VECTORS[expected_profile]
            or s.get("publisher_calibration_verified") is not True
            or s.get("faults") != []
            or s.get("device_access") != "NONE"
            or s.get("production_acceptance") != "BLOCKED"
            or s.get("ab_causation_verified") is not False
            or s.get("synthetic_only") is not True):
        raise ValueError("M23S source-D/producer/safety evidence invalid")
    if (p.get("observations", {}).get("synthetic_calibration")
            != CALIBRATION[expected_profile]):
        raise ValueError("observed calibration contradicts requested profile")
    if p.get("actual_graph_qos_verified") is not True:
        raise ValueError("actual reliable/volatile graph not verified")
    obs = p.get("observations") or {}
    counts = obs.get("synthetic_observer_counts") or {}
    if (obs.get("actual_graph_qos_verified") is not True
            or any(type(counts.get(topic)) is not int
                   or counts[topic] < 810
                   for topic in ("raw", "rect", "info"))
            or obs.get("unmatched_rectified_header_stamps") != 0
            or type(obs.get("raw_rect_identical_header_stamp_pairs")) is not int
            or obs["raw_rect_identical_header_stamp_pairs"] <
                0.90 * counts["rect"]
            or obs.get("synthetic_published_image_count") != 900
            or type(obs.get("synthetic_rect_max_header_gap_ms")) not in
                (int, float)
            or not 0 < obs["synthetic_rect_max_header_gap_ms"] <= 250):
        raise ValueError("actual synthetic rectified delivery not verified")
    if (p.get("synthetic_publisher_exit_code") != 0
            or p.get("synthetic_observer_exit_code") != 0):
        raise ValueError("producer/observer did not exit successfully")
    trace = r.get("trace") or {}
    ev = trace.get("event_counts") or {}
    if (trace.get("process_count") != 1
            or trace.get("diagnostic_only") is not True
            or ev.get("CONSTRUCTOR") != 1
            or not isinstance(ev.get("DLOPEN_RETURN"), int)
            or ev["DLOPEN_RETURN"] < 1):
        raise ValueError("component trace never initialized")
    if (p.get("component_launch_exit_code") not in
            (0, 130, -2)
            or s.get("m23r_runner_exit") != r.get("m23p_runner_exit")):
        raise ValueError("inconsistent runner exit evidence")

    status = p.get("component_exit_class")
    child_code = s.get("component_child_exit_code_if_reported")
    if (status == "CLEAN_COMPONENT_SHUTDOWN"
            and p.get("faults") == []
            and r.get("verdict") == "CLEAN_OBSERVED_UNDER_TRACE_ONLY"
            and s.get("m23r_runner_exit") == 0
            and child_code is None):
        lifecycle = "CLEAN_UNDER_INSTRUMENTATION"
    elif (status == "BLOCKED_RECTIFIER_CHILD_CRASH"
          and p.get("faults") == ["BLOCKED_RECTIFIER_CHILD_CRASH"]
          and r.get("verdict") == "BLOCKED_COMPONENT_LIFECYCLE"
          and s.get("m23r_runner_exit") == 2
          and child_code == -11):
        lifecycle = "CHILD_SIGSEGV_MINUS11"
    else:
        raise ValueError("lifecycle outcome not classifiable for A/B")
    return {
        "profile": expected_profile,
        "child_lifecycle": lifecycle,
        "rectified_observed": counts["rect"],
        "matching_stamps": obs["raw_rect_identical_header_stamp_pairs"],
        "max_rect_header_gap_ms": obs["synthetic_rect_max_header_gap_ms"],
        "device_access": "NONE",
        "faulting_function_verified": False,
    }


def compare_payloads(passthrough, rectify):
    a = classify_run(passthrough, "passthrough")
    b = classify_run(rectify, "rectify")
    combo = (a["child_lifecycle"], b["child_lifecycle"])
    verdicts = {
        ("CLEAN_UNDER_INSTRUMENTATION", "CHILD_SIGSEGV_MINUS11"):
            "DISTORTION_BRANCH_ASSOCIATION_SUPPORTED_NOT_CAUSAL",
        ("CHILD_SIGSEGV_MINUS11", "CHILD_SIGSEGV_MINUS11"):
            "SHARED_COMPONENT_TEARDOWN_ASSOCIATION_SUPPORTED_NOT_CAUSAL",
        ("CLEAN_UNDER_INSTRUMENTATION", "CLEAN_UNDER_INSTRUMENTATION"):
            "NO_CRASH_REPRODUCED_UNDER_TRACE",
        ("CHILD_SIGSEGV_MINUS11", "CLEAN_UNDER_INSTRUMENTATION"):
            "REVERSE_ASSOCIATION_INCONCLUSIVE",
    }
    return {
        "milestone": "M23S",
        "comparison": verdicts[combo],
        "passthrough": a,
        "rectify": b,
        "diagnostic_pair_only": True,
        "causation_verified": False,
        "component_lifecycle_accepted": False,
        "physical_motion_authorized": False,
        "production_acceptance": "BLOCKED",
        "device_access": "NONE",
    }


def read_private_log(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError("private regular non-symlink log required")
    if not 0 < path.stat().st_size <= LIMIT_BYTES:
        raise ValueError("log missing/empty/oversize")
    return path.read_text(errors="replace")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--passthrough-log", required=True)
    parser.add_argument("--rectify-log", required=True)
    args = parser.parse_args()
    try:
        if (Path(args.passthrough_log).resolve() ==
                Path(args.rectify_log).resolve()):
            raise ValueError("comparison logs must be separate")
        result = compare_payloads(
            read_private_log(args.passthrough_log),
            read_private_log(args.rectify_log))
    except (ValueError, OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print("M23S_PAIR_BLOCKED=" + str(exc)[:180], flush=True)
        return 2
    print("M23S_PAIR_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23S_PAIR_VERDICT=" + result["comparison"], flush=True)
    print("M23S_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    print("M23S_DEVICE_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
