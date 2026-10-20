#!/usr/bin/env python3
"""M23S: two separate no-device runs, differ only by fabricated CameraInfo.D.

Inherit M23R's component-scoped loader tracer and M23P's unmodified
publisher-first synthetic QoS, delivery, and child-shutdown checker.
This wrapper has no subprocess calls, ROS imports or device access.
"""
import json
import os
from pathlib import Path
import re
import sys

PROFILES = ("passthrough", "rectify")
PUBLISHER = (
    "/opt/m23s-cpp-install/lib/leroy_m23s_rectification_ab/"
    "m23s_ab_publisher"
)
CALIBRATION = {
    "passthrough": "FABRICATED_ZERO_DISTORTION",
    "rectify": "FABRICATED_NONZERO_DISTORTION",
}
D_VECTORS = {
    "passthrough": "0,0,0,0,0",
    "rectify": "-0.08,0.01,0.001,-0.001,0",
}
M23P_CALIBRATION_PREFIX = "M23P_SYNTHETIC_CALIBRATION="
CHILD_DEATH_LINE = re.compile(
    r"component_container-.*process has died.*?exit code (-?[0-9]+)"
)


def selected_profile(environ):
    """Allow only this exact A/B, using synthetic publisher, not live devices."""
    required = {
        "M23S_AB_ONLY": "1",
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
            raise ValueError("M23S isolation requirement: " + name)
    if environ.get("LD_PRELOAD"):
        raise ValueError("M23S globally inherited LD_PRELOAD forbidden")
    profile = environ.get("M23S_DISTORTION_PROFILE")
    if profile not in PROFILES:
        raise ValueError("M23S_DISTORTION_PROFILE must be passthrough or rectify")
    return profile


def _single_line(payload, marker):
    matches = [line[len(marker):] for line in payload.splitlines()
               if line.startswith(marker)]
    if len(matches) != 1:
        raise ValueError("missing or duplicate publisher marker: " + marker)
    return matches[0]


def validate_publisher_log(payload, profile, m23p_parser):
    """Enforce true mode and D, then reuse original M23P timing/count checks.

    M23P's strict validator only accepts the original nonzero marker.
    Convert a *private validation copy* solely for that validator;
    never modify the real log and return the truthful synthetic_calibration
    label for M23P_LIFECYCLE_JSON. All other M23P checks are unchanged.
    """
    if profile not in PROFILES:
        raise ValueError("unsupported M23S publisher fixture")
    expected_calibration = CALIBRATION[profile]
    marker = _single_line(payload, M23P_CALIBRATION_PREFIX)
    if marker != expected_calibration:
        raise ValueError("publisher declared unexpected calibration")
    if (_single_line(payload, "M23S_PUBLISHER_DISTORTION_PROFILE=")
            != profile):
        raise ValueError("publisher distortion profile mismatch")
    if (_single_line(payload, "M23S_PUBLISHER_D_VECTOR=")
            != D_VECTORS[profile]):
        raise ValueError("publisher distortion values mismatch")
    if profile == "passthrough":
        payload_for_original = payload.replace(
            M23P_CALIBRATION_PREFIX + expected_calibration + "\n",
            M23P_CALIBRATION_PREFIX + CALIBRATION["rectify"] + "\n",
        )
        # Handle a normal final line without newline, if present.
        if payload_for_original == payload:
            raise ValueError("cannot adapt passthrough marker for M23P checks")
    else:
        payload_for_original = payload
    result = m23p_parser(payload_for_original)
    result["synthetic_calibration"] = expected_calibration
    return result


def child_exit_from_log(log):
    """Only component's explicit ROS launch child death line, never parent exit."""
    codes = [int(match.group(1)) for line in log.splitlines()
             for match in [CHILD_DEATH_LINE.search(line)]
             if match is not None]
    return codes[0] if len(codes) == 1 else None


def run():
    profile = selected_profile(os.environ)
    import m23p_no_device_rectifier_shutdown as m23p
    import m23r_shutdown_trace_runner as m23r

    # Both pre-existing guards remain mandatory.
    m23p.selected_profile(os.environ)
    m23p.prohibit_devices()
    m23r.preconditions(os.environ)
    if not Path(PUBLISHER).is_file():
        raise ValueError("M23S-only publisher binary missing")

    original_publisher = m23p.PUBLISHER
    original_parser = m23p.parse_m23p_publisher_log
    m23p.PUBLISHER = PUBLISHER
    m23p.parse_m23p_publisher_log = (
        lambda payload: validate_publisher_log(
            payload, profile, original_parser))
    try:
        # M23R already injects the traced component launch and checks
        # actual child teardown independently of ros2 launch parent.
        code = m23r.run()
    finally:
        m23p.PUBLISHER = original_publisher
        m23p.parse_m23p_publisher_log = original_parser

    faults = []
    try:
        with m23p.PUBLISHER_LOG.open("r", errors="replace") as stream:
            publisher_log = stream.read(2 * 1024 * 1024 + 1)
        if len(publisher_log) > 2 * 1024 * 1024:
            raise ValueError("publisher log exceeds safety limit")
        verified = validate_publisher_log(
            publisher_log, profile, original_parser)
    except (OSError, ValueError) as exc:
        verified = {}
        faults.append("PUBLISHER_NOT_VERIFIED:" + str(exc)[:120])
    try:
        with m23p.LAUNCH_LOG.open("r", errors="replace") as stream:
            launch_log = stream.read(2 * 1024 * 1024 + 1)
        if len(launch_log) > 2 * 1024 * 1024:
            raise ValueError("component log exceeds safety limit")
        child_code = child_exit_from_log(launch_log)
    except (OSError, ValueError) as exc:
        child_code = None
        faults.append("COMPONENT_LOG_UNAVAILABLE:" + str(exc)[:120])

    verdict = ("BLOCKED_M23S_AB_EVIDENCE" if faults else
               "SYNTHETIC_PROFILE_CLEAN_UNDER_TRACE_ONLY" if code == 0 else
               "BLOCKED_M23S_COMPONENT_LIFECYCLE")
    result = {
        "milestone": "M23S",
        "distortion_profile": profile,
        "source_d": D_VECTORS[profile],
        "publisher_calibration_verified": not faults and bool(verified),
        "m23r_runner_exit": code,
        "component_child_exit_code_if_reported": child_code,
        "verdict": verdict,
        "faults": faults,
        "device_access": "NONE",
        "synthetic_only": True,
        "real_camera_validated": False,
        "ab_causation_verified": False,
        "production_acceptance": "BLOCKED",
        "physical_motion_authorized": False,
    }
    print("M23S_AB_RESULT_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23S_AB_VERDICT=" + verdict, flush=True)
    print("M23S_DEVICE_ACCESS=NONE", flush=True)
    print("M23S_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 2 if code != 0 or faults else 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, ValueError) as exc:
        print("M23S_PRECONDITION_BLOCKED=" + str(exc)[:180],
              file=sys.stderr, flush=True)
        sys.exit(2)
