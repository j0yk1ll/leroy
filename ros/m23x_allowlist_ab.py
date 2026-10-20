#!/usr/bin/env python3
"""M23X: no-device M23S runner with two allowlist-only component launches.

Source for REVIEW, not runtime authorization. Reuses M23S->M23R->M23P
guards, publisher-first order, actual QoS graph and child-exit classifier.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

ARMS = {
    "baseline": "/opt/m23x/m23x_baseline_rectifier.launch.py",
    "raw": "/opt/m23x/m23x_raw_rectifier.launch.py",
}
INSPECTOR = "/opt/m23x/m23x_graph_param.py"
PARAM = "image_rect.enable_pub_plugins"
RAW = "image_transport/raw"


def single_output(log, key):
    result = [s[len(key) + 1:] for s in log.splitlines()
              if s.startswith(key + "=")]
    if len(result) != 1:
        raise ValueError("expected one " + key + " marker")
    return result[0]


def inspect_before_frames(state):
    """Called exactly where inherited M23P verifies the preframe DDS graph."""
    from m23p_qos_graph import validate_synthetic_graph

    try:
        audit = subprocess.run(
            ["python3", INSPECTOR],
            capture_output=True, text=True, check=False, timeout=7,
            env=dict(os.environ))
        if audit.returncode != 0:
            # The subprocess owns the exact ROS error context. Share only
            # one allowlisted stage marker; never expose private stderr.
            stages = [line.split("=", 1)[1] for line in audit.stdout.splitlines()
                      if line.startswith("M23X_PARAM_FAILURE_STAGE=")]
            allowed = {"GRAPH_QOS", "PARAMETER_SERVICE",
                       "PARAMETER_RESPONSE", "ALLOWLIST_CONTENT"}
            stage = stages[0] if len(stages) == 1 and stages[0] in allowed else "UNCLASSIFIED"
            print("M23X_PARAM_FAILURE_STAGE=" + stage, flush=True)
            raise ValueError("parameter/graph observer returned nonzero")
        if single_output(audit.stdout, "M23X_PARAM_VERDICT") != "PASS":
            raise ValueError("effective parameter audit was not PASS")
        observation = json.loads(single_output(audit.stdout, "M23X_PARAM_JSON"))
        graph = json.loads(single_output(audit.stdout, "M23P_GRAPH_JSON"))
        if (observation.get("arm") != state["arm"]
                or observation.get("node") != "/camera/rectify_node"
                or observation.get("parameter") != PARAM
                or observation.get("type") != "string_array"
                or observation.get("verified") is not True):
            raise ValueError("parameter identity/provenance mismatch")
        from m23x_graph_param import inspect_values
        inspect_values(state["arm"], observation["node"],
                       9, observation.get("effective_values"), 9)
        validate_synthetic_graph(graph)
        state["parameter"] = observation
        state["graph"] = graph
        state["audits"] += 1
        print("M23X_PARAMETER_AUDIT=PASS", flush=True)
        return graph
    except (ValueError, OSError, TypeError, KeyError,
            subprocess.TimeoutExpired) as exc:
        print("M23X_PARAMETER_AUDIT=BLOCKED_" + type(exc).__name__,
              flush=True)
        # In M23P, KeyboardInterrupt is caught at the *outer* run scope.
        # Unlike ValueError, it aborts BEFORE the 8-second delayed frames
        # and still executes every inherited child cleanup finally.
        raise KeyboardInterrupt("M23X_PRE_FRAME_AUDIT_BLOCKED") from None


def classify_run(state, status):
    import m23p_no_device_rectifier_shutdown as m23p
    import m23r_shutdown_trace_runner as m23r
    import m23s_rectification_ab as m23s
    from m23k_camera_path_probe import parse_counter_log

    outcome = {
        "arm": state["arm"],
        "effective_parameter_verified": bool(state["parameter"]),
        "preframe_audit_count": state["audits"],
        "published": None,
        "received_raw": None,
        "received_rect": None,
        "matched": None,
        "component_child_exit_code_if_reported": None,
        "verdict": "BLOCKED_INCOMPLETE_EVIDENCE",
        "production_acceptance": "BLOCKED",
        "root_cause_verified": False,
    }
    try:
        if state["audits"] != 1:
            raise ValueError("missing preframe parameter and graph audit")
        pub_log = m23p.PUBLISHER_LOG.read_text(errors="replace")
        evidence = m23s.validate_publisher_log(
            pub_log, "rectify", m23p.parse_m23p_publisher_log)
        result = parse_counter_log(
            m23p.COUNTER_LOG.read_text(errors="replace"),
            require_rect=True)
        log = m23p.LAUNCH_LOG.read_text(errors="replace")
        m23r.summarize_trace(log)
        child = m23s.child_exit_from_log(log)
        counts = result["counts"]
        matched = result["raw_rect_identical_header_stamp_pairs"]
        outcome.update({
            "published": evidence["image"],
            "received_raw": counts.get("raw"),
            "received_rect": counts.get("rect"),
            "received_camera_info": counts.get("info"),
            "matched": matched,
            "component_child_exit_code_if_reported": child,
        })
        good = (
            evidence["image"] == evidence["info"] == 900
            and counts.get("raw") == counts.get("rect") ==
                counts.get("info") == 900
            and matched == 900
            and result["rect_only_observed_header_stamps"] == 0
            and result["streams"]["rect"]["header_intervals"][
                "gaps_over_250ms"] == 0
            and not result["faults"]
            and bool(state["parameter"])
        )
        if not good:
            raise ValueError("incomplete synthetic rectified delivery")
        if state["arm"] == "baseline":
            if child == -11 and status == 2:
                outcome["verdict"] = "CONTROL_SIGSEGV_REPRODUCED"
            else:
                outcome["verdict"] = "BLOCKED_CONTROL_NOT_REPRODUCED"
        elif status == 0 and child is None:
            outcome["verdict"] = "RAW_CLEAN_OBSERVED_SYNTHETIC_ONLY"
        elif child == -11:
            outcome["verdict"] = "BLOCKED_RAW_SIGSEGV"
        else:
            outcome["verdict"] = "BLOCKED_AMBIGUOUS_RAW_OUTCOME"
    except (OSError, ValueError, KeyError, TypeError) as exc:
        outcome["verdict"] = "BLOCKED_" + type(exc).__name__
    return outcome


def run():
    arm = os.environ.get("M23X_ARM")
    if arm not in ARMS or os.environ.get("M23S_DISTORTION_PROFILE") != "rectify":
        raise ValueError("explicit M23X_ARM and nonzero-D rectify profile required")
    import m23p_no_device_rectifier_shutdown as m23p
    import m23r_shutdown_trace_runner as m23r
    import m23s_rectification_ab as m23s
    # Inherited source-level profile, global preload and device guards.
    m23s.selected_profile(os.environ)
    m23p.selected_profile(os.environ)
    m23p.prohibit_devices()
    if not Path(ARMS[arm]).is_file() or not Path(INSPECTOR).is_file():
        raise ValueError("M23X source dependencies missing")

    state = {"arm": arm, "audits": 0, "parameter": None, "graph": None}
    old_launch = m23r.TRACE_LAUNCH
    old_audit = m23p.inspect_synthetic_graph
    try:
        m23r.TRACE_LAUNCH = ARMS[arm]
        m23p.inspect_synthetic_graph = lambda: inspect_before_frames(state)
        status = m23s.run()
    finally:
        m23r.TRACE_LAUNCH = old_launch
        m23p.inspect_synthetic_graph = old_audit
    verdict = classify_run(state, status)
    print("M23X_RESULT_JSON=" + json.dumps(verdict, sort_keys=True), flush=True)
    print("M23X_RESULT=" + verdict["verdict"], flush=True)
    print("M23X_RUNTIME_SUCCESS_IS_PRODUCTION_ACCEPTANCE=FALSE", flush=True)
    print("M23X_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0 if (
        arm == "raw"
        and status == 0
        and verdict["verdict"] == "RAW_CLEAN_OBSERVED_SYNTHETIC_ONLY"
    ) else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23X_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
