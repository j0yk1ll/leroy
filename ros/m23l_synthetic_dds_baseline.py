#!/usr/bin/env python3
"""M23L: bounded SYNTHETIC 640x480 Image ROS DDS delivery baseline.

No camera, MCU, IMU, ROS bag, XRSLAM or motor access. Two independent
C++ ROS processes run inside ONE disconnected rootless container.
Publisher sends 900 fabricated RGB8 frames; subscriber is M23K's
already-tested C++ raw/CameraInfo header counter (no rectifier).
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from m23k_camera_path_probe import parse_counter_log

# M23M builds its QoS-capable counter at a separate isolated prefix.
# With no override the original accepted M23L path remains intact.
CPP_INSTALL_ROOT = os.environ.get(
    "M23M_CPP_INSTALL_ROOT", "/opt/m23l-cpp-install")
PUB_BINARY = (CPP_INSTALL_ROOT + "/lib/leroy_m23k_camera_counter/"
              "m23l_synthetic_image_publisher")
COUNTER_BINARY = (CPP_INSTALL_ROOT + "/lib/leroy_m23k_camera_counter/"
                  "m23k_camera_header_counter")
M23M_QOS_PROFILES = ("best_effort_5", "best_effort_100", "reliable_100")
PUB_LOG = Path("/tmp/m23l-synthetic-publisher.log")
COUNTER_LOG = Path("/tmp/m23l-synthetic-subscriber.log")
PUBLISHED_FRAMES = 900
EXPECTED_PAYLOAD_BYTES = 640 * 480 * 3
MAX_PUBLISH_SPAN_NS = 37_000_000_000
MIN_PUBLISH_SPAN_NS = 25_000_000_000
MIN_SUBSCRIBER_FRACTION = 0.90
FORBIDDEN_DEVICE_PATHS = ("/dev/ttyACM0", "/dev/ttyACM1",
                          "/dev/video2", "/dev/video8")
REQUIRED_ENV = {
    "M23L_SYNTHETIC_DDS_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


def parse_publisher_log(payload):
    fields = {}
    for line in payload.splitlines():
        if line.startswith("M23L_PUBLISHER_FATAL="):
            raise ValueError("synthetic publisher fatal: " + line[-240:])
        for key in ("COUNTS", "START_SPAN_NS", "MAX_CALLBACK_NS",
                    "PAYLOAD_BYTES"):
            name = "M23L_PUBLISHER_" + key + "="
            if line.startswith(name):
                if key in fields:
                    raise ValueError("duplicate synthetic publisher marker")
                fields[key] = line[len(name):]
    if set(fields) != {"COUNTS", "START_SPAN_NS",
                       "MAX_CALLBACK_NS", "PAYLOAD_BYTES"}:
        raise ValueError("incomplete synthetic publisher output")
    try:
        counts = [int(x) for x in fields["COUNTS"].split(",")]
        span = int(fields["START_SPAN_NS"])
        max_callback = int(fields["MAX_CALLBACK_NS"])
        payload_bytes = int(fields["PAYLOAD_BYTES"])
    except ValueError as exc:
        raise ValueError("invalid synthetic publisher statistics") from exc
    if counts != [PUBLISHED_FRAMES, PUBLISHED_FRAMES]:
        raise ValueError("synthetic publisher did not send exact frames")
    if span <= 0 or max_callback < 0 or payload_bytes != EXPECTED_PAYLOAD_BYTES:
        raise ValueError("invalid synthetic publisher duration/payload")
    return {"image": counts[0], "info": counts[1],
            "span_ns": span, "max_callback_ns": max_callback,
            "image_payload_bytes": payload_bytes}


def interpret(publisher, observer):
    """Describe DDS results without making unverifiable delivery claims."""
    published = publisher["image"]
    count = observer["counts"]["raw"]
    info_count = observer["counts"]["info"]
    publisher_slow = not (
        MIN_PUBLISH_SPAN_NS <= publisher["span_ns"] <= MAX_PUBLISH_SPAN_NS)
    ratio = count / published
    info_ratio = info_count / publisher["info"]
    gap = observer["streams"]["raw"]["header_intervals"]
    if publisher_slow:
        classification = "BLOCKED_SYNTHETIC_PUBLISHER_SCHEDULING"
    elif ratio < MIN_SUBSCRIBER_FRACTION:
        classification = "BLOCKED_FULL_IMAGE_DDS_SUBSCRIBER_DELIVERY"
    elif info_ratio < MIN_SUBSCRIBER_FRACTION:
        classification = "BLOCKED_CAMERA_INFO_SUBSCRIBER_DELIVERY"
    else:
        classification = "SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY"
    return {
        "milestone": "M23L",
        "verdict": classification,
        "published_image_messages": published,
        "published_camera_info_messages": publisher["info"],
        "subscribed_image_messages": count,
        "subscribed_camera_info_messages": info_count,
        "raw_delivery_fraction": round(ratio, 4),
        "camera_info_delivery_fraction": round(info_ratio, 4),
        "publisher_start_span_s": round(publisher["span_ns"] / 1e9, 4),
        "publisher_max_callback_ms":
            round(publisher["max_callback_ns"] / 1e6, 3),
        "image_payload_bytes": publisher["image_payload_bytes"],
        "subscriber_raw_header_gaps_over_250ms": gap["gaps_over_250ms"],
        "subscriber_raw_max_header_gap_ms": gap["max_gap_ms"],
        "no_physical_device_access": True,
        "synthetic_images_only": True,
        "real_camera_rate_verified": False,
        "camera_imu_calibrated": False,
        "xrslam_tracking_validated": False,
        "physical_motion_authorized": False,
        "limitations": [
            "DDS throughput under synthetic constant-pixel load is not usb_cam performance",
            "Publisher rate and subscriber receipts are measured with independent processes",
            "Default single-container loopback may differ from actual multi-node workload",
            "High receiver rate cannot validate pixel contents or VIO input readiness",
        ],
    }


def read_counter_qos_profile(output):
    """Require exactly one C++-reported receiver QoS profile marker."""
    rows = [line.split("=", 1)[1].strip()
            for line in output.splitlines()
            if line.startswith("M23M_COUNTER_QOS_PROFILE=")]
    if len(rows) != 1 or rows[0] not in M23M_QOS_PROFILES:
        raise ValueError("missing, duplicate or invalid receiver QoS marker")
    return rows[0]


def _stop_counter(proc):
    if proc is None:
        return "not-started"
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        code = proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        return "shutdown-timeout"
    return "exit-" + str(code)


def run():
    requested_qos = os.environ.get("M23M_SUB_QOS", "best_effort_5")
    if requested_qos not in M23M_QOS_PROFILES:
        raise ValueError("M23M receiver QoS profile must be allowlisted")
    for name, value in REQUIRED_ENV.items():
        if os.environ.get(name) != value:
            raise ValueError("isolation precondition: " + name)
    if any(Path(x).exists() for x in FORBIDDEN_DEVICE_PATHS):
        raise ValueError("physical camera/STM32 device mapped into synthetic test")
    if not all(Path(x).is_file() for x in (PUB_BINARY, COUNTER_BINARY)):
        raise ValueError("C++ publisher/counter missing")
    if any(x.exists() for x in (PUB_LOG, COUNTER_LOG)):
        raise ValueError("logs exist: refusing to overwrite prior measurement")

    counter = pub = None
    issues = []
    pub_exit = None
    with COUNTER_LOG.open("xb") as subscriber_out, PUB_LOG.open("xb") as pub_out:
        try:
            env = dict(os.environ)
            env["M23K_RAW_ONLY"] = "1"
            env["M23M_SUB_QOS"] = requested_qos
            counter = subprocess.Popen([COUNTER_BINARY], env=env,
                stdout=subscriber_out, stderr=subprocess.STDOUT,
                start_new_session=True)
            time.sleep(2)
            if counter.poll() is not None:
                raise RuntimeError("header counter exited before publisher")
            pub = subprocess.Popen([PUB_BINARY], stdout=pub_out,
                stderr=subprocess.STDOUT, start_new_session=True)
            pub_exit = pub.wait(timeout=75)
            if pub_exit != 0:
                issues.append("SYNTHETIC_PUBLISHER_EXIT_" + str(pub_exit))
            else:
                # Allow DDS delivery to drain after the final synthetic
                # publisher callback rather than cutting off in-flight
                # frames immediately. Rerun the baseline with this same
                # drain window before comparing QoS profiles.
                time.sleep(2.0)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            issues.append(type(exc).__name__ + ": " + str(exc))
        finally:
            if pub is not None and pub.poll() is None:
                try:
                    os.killpg(pub.pid, signal.SIGINT)
                    pub.wait(timeout=5)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    issues.append("SYNTHETIC_PUBLISHER_SHUTDOWN_UNVERIFIED")
            code = _stop_counter(counter)
            if code != "exit-0":
                issues.append("COUNTER_" + code)
            pub_out.flush()
            subscriber_out.flush()

    try:
        publisher = parse_publisher_log(PUB_LOG.read_text(errors="replace"))
        counter_output = COUNTER_LOG.read_text(errors="replace")
        observer = parse_counter_log(counter_output, require_rect=False)
        actual_qos = read_counter_qos_profile(counter_output)
        if actual_qos != requested_qos:
            raise ValueError("receiver QoS output differs from requested mode")
        result = interpret(publisher, observer)
        if observer["faults"]:
            issues.extend(observer["faults"])
    except (ValueError, TypeError, KeyError) as exc:
        issues.append(type(exc).__name__ + ": " + str(exc))
        result = {"milestone": "M23L",
                  "verdict": "BLOCKED_SYNTHETIC_INCOMPLETE",
                  "no_physical_device_access": True,
                  "synthetic_images_only": True,
                  "real_camera_rate_verified": False,
                  "physical_motion_authorized": False}
    if issues:
        result["verdict"] = "BLOCKED_SYNTHETIC_INCOMPLETE"
    result["receiver_qos_profile"] = requested_qos
    result["publisher_qos_profile"] = "reliable_100"
    result["subscriber_drain_window_s"] = 2.0
    result["rmw_implementation"] = os.environ.get(
        "RMW_IMPLEMENTATION", "unspecified_default")
    result["faults"] = issues[:12]
    print("M23M_QOS_JSON=" + json.dumps(result, sort_keys=True), flush=True)
    print("M23M_QOS_VERDICT=" + result["verdict"], flush=True)
    print("M23L_SYNTHETIC_DDS_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23L_VERDICT=" + result["verdict"], flush=True)
    print("M23L_DEVICE_ACCESS=NONE", flush=True)
    return 0 if result["verdict"] == \
        "SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY" else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (ValueError, OSError) as exc:
        print("M23L_PRECONDITION=" + str(exc), file=sys.stderr, flush=True)
        sys.exit(2)
