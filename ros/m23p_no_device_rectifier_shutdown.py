#!/usr/bin/env python3
"""M23P isolated no-device RectifyNode shutdown experiment.

Profiles: idle component vs synthetic M23L RGB8 + CameraInfo publisher,
with the same C++ observer used in M23O. Never touches real devices.
"""
import json
import os
from pathlib import Path
import glob
import signal
import subprocess
import sys
import time

from m23k_camera_path_probe import (
    camera_crash_context, camera_launch_crashed, parse_counter_log,
    verified_receiver_qos_log,
)

PROFILES = ("idle", "synthetic")
REQUIRED_ENV = {
    "M23P_SYNTHETIC_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
INSTALL = "/opt/m23m-cpp-install/lib/leroy_m23k_camera_counter/"
PUBLISHER = ("/opt/m23p-cpp-install/lib/leroy_m23p_calibrated_synthetic/"
             "m23p_calibrated_publisher")
PUBLISHED_FRAMES = 900
MIN_RECT_DELIVERY_FRACTION = 0.90
EXPECTED_PAYLOAD_BYTES = 640 * 480 * 3


def parse_m23p_publisher_log(payload):
    """Demand complete M23P-only publisher evidence and declared fake calibration."""
    fields = {}
    required = ("COUNTS", "START_SPAN_NS", "MAX_CALLBACK_NS",
                "PAYLOAD_BYTES", "SYNTHETIC_CALIBRATION")
    for line in payload.splitlines():
        if line.startswith("M23P_PUBLISHER_FATAL="):
            raise ValueError("M23P synthetic publisher reported fatal")
        for field in required:
            marker = "M23P_" + (field if field == "SYNTHETIC_CALIBRATION"
                                else "PUBLISHER_" + field) + "="
            if line.startswith(marker):
                if field in fields:
                    raise ValueError("duplicate M23P publisher marker")
                fields[field] = line[len(marker):]
    if set(fields) != set(required):
        raise ValueError("incomplete M23P fabricated publisher output")
    if fields["SYNTHETIC_CALIBRATION"] != "FABRICATED_NONZERO_DISTORTION":
        raise ValueError("publisher calibration fixture unverified")
    try:
        counts = [int(x) for x in fields["COUNTS"].split(",")]
        span = int(fields["START_SPAN_NS"])
        callback = int(fields["MAX_CALLBACK_NS"])
        payload_bytes = int(fields["PAYLOAD_BYTES"])
    except ValueError as exc:
        raise ValueError("M23P publisher invalid numeric fields") from exc
    if counts != [PUBLISHED_FRAMES, PUBLISHED_FRAMES]:
        raise ValueError("publisher count not exactly 900")
    if not 0 < span <= 37_000_000_000 or callback < 0:
        raise ValueError("publisher callback timing invalid")
    if payload_bytes != EXPECTED_PAYLOAD_BYTES:
        raise ValueError("publisher payload size incorrect")
    return {"image": counts[0], "info": counts[1],
            "span_ns": span, "payload_bytes": payload_bytes,
            "synthetic_calibration": fields["SYNTHETIC_CALIBRATION"]}
COUNTER = INSTALL + "m23k_camera_header_counter"
LAUNCH = "/opt/m23p/m23p_rectifier_only.launch.py"
LAUNCH_LOG = Path("/tmp/m23p-rectifier-launch.log")
PUBLISHER_LOG = Path("/tmp/m23p-synthetic-publisher.log")
COUNTER_LOG = Path("/tmp/m23p-synthetic-observer.log")


def selected_profile(environ):
    value = environ.get("M23P_PROFILE")
    if value not in PROFILES:
        raise ValueError("M23P_PROFILE must be idle or synthetic")
    for key, expected in REQUIRED_ENV.items():
        if environ.get(key) != expected:
            raise ValueError("M23P isolation requirement: " + key)
    return value


def prohibit_devices():
    forbidden = ("/dev/video*", "/dev/ttyACM*", "/dev/serial/by-id/*")
    hits = [path for pattern in forbidden for path in glob.glob(pattern)]
    if hits:
        raise ValueError("physical device visible: no-device experiment blocked")
    if os.environ.get("M23K_PROBE_MODE") == "live":
        raise ValueError("live camera mode forbidden in M23P")


def assess_container_exit(launch_output, code):
    """Read child exit independently of ros2 launch parent exit status."""
    if camera_launch_crashed(launch_output):
        return "BLOCKED_RECTIFIER_CHILD_CRASH"
    if not any(
        "component_container-" in line and "process has finished cleanly" in line
        for line in launch_output.splitlines()
    ):
        return "BLOCKED_RECTIFIER_SHUTDOWN_NOT_CONFIRMED"
    if code not in (0, 130, -int(signal.SIGINT)):
        return "BLOCKED_RECTIFIER_LAUNCH_EXIT_" + str(code)
    return "CLEAN_COMPONENT_SHUTDOWN"


def stop_group(proc, timeout=15):
    if proc is None:
        return None
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        return "SHUTDOWN_TIMEOUT"


def wait_for_rectifier(proc, out_path, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("rectifier launch exited before loading")
        payload = out_path.read_text(errors="replace")
        if "Loaded node '/camera/rectify_node'" in payload:
            return
        time.sleep(0.2)
    raise RuntimeError("rectifier component load not confirmed")


def run():
    profile = selected_profile(os.environ)
    prohibit_devices()
    for target in ([LAUNCH] if profile == "idle" else
                   [LAUNCH, PUBLISHER, COUNTER]):
        if not Path(target).is_file():
            raise ValueError("missing local no-device dependency: " + target)
    for path in (LAUNCH_LOG, PUBLISHER_LOG, COUNTER_LOG):
        if path.exists():
            raise ValueError("refusing to overwrite diagnostic log " + str(path))
    issues = []
    observations = {}
    launch_proc = counter = publisher = None
    launch_code = counter_code = publisher_code = None
    # Each mode executes in a fresh disposable isolated container.
    with LAUNCH_LOG.open("xb") as launch_out:
        try:
            launch_proc = subprocess.Popen(
                ["ros2", "launch", LAUNCH], stdout=launch_out,
                stderr=subprocess.STDOUT, start_new_session=True,
                env=dict(os.environ))
            wait_for_rectifier(launch_proc, LAUNCH_LOG)
            if profile == "idle":
                time.sleep(8.0)
            else:
                env = dict(os.environ)
                env["M23K_RAW_ONLY"] = "0"
                env["M23M_SUB_QOS"] = "reliable_100"
                with COUNTER_LOG.open("xb") as counter_out, \
                        PUBLISHER_LOG.open("xb") as publisher_out:
                    try:
                        counter = subprocess.Popen(
                            [COUNTER], stdout=counter_out,
                            stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
                        time.sleep(2)
                        if counter.poll() is not None:
                            raise RuntimeError("C++ observer stopped before publishing")
                        publisher = subprocess.Popen(
                            [PUBLISHER], stdout=publisher_out,
                            stderr=subprocess.STDOUT,
                            start_new_session=True, env=dict(os.environ))
                        publisher_code = publisher.wait(timeout=55)
                        if publisher_code != 0:
                            issues.append("SYNTHETIC_PUBLISHER_EXIT_" +
                                          str(publisher_code))
                        time.sleep(2.0)
                    except (OSError, RuntimeError,
                            subprocess.TimeoutExpired) as exc:
                        issues.append("SYNTHETIC_RUNTIME_" +
                                      type(exc).__name__ + ":" + str(exc)[:160])
                    finally:
                        if publisher is not None and publisher.poll() is None:
                            publisher_code = stop_group(publisher)
                        # Keep counter alive until after rectifier stops.
                        launch_code = stop_group(launch_proc)
                        counter_code = stop_group(counter)
            if profile == "idle":
                launch_code = stop_group(launch_proc)
        except (OSError, RuntimeError, KeyboardInterrupt) as exc:
            issues.append(type(exc).__name__ + ":" + str(exc)[:160])
        finally:
            if launch_proc is not None and launch_proc.poll() is None:
                launch_code = stop_group(launch_proc)
            if counter is not None and counter.poll() is None:
                counter_code = stop_group(counter)
            if publisher is not None and publisher.poll() is None:
                publisher_code = stop_group(publisher)
            launch_out.flush()

    launch_text = LAUNCH_LOG.read_text(errors="replace")
    launch_status = assess_container_exit(launch_text, launch_code)
    if launch_status != "CLEAN_COMPONENT_SHUTDOWN":
        issues.append(launch_status)
    if profile == "synthetic":
        try:
            published = parse_m23p_publisher_log(
                PUBLISHER_LOG.read_text(errors="replace"))
            observer_log = COUNTER_LOG.read_text(errors="replace")
            observed = parse_counter_log(observer_log, require_rect=True)
            verified_receiver_qos_log(observer_log, "reliable_100")
            observations = {
                "synthetic_published_image_count": published["image"],
                "synthetic_calibration": published["synthetic_calibration"],
                "synthetic_observer_counts": observed["counts"],
                "synthetic_observer_faults": observed["faults"],
                "synthetic_rect_max_header_gap_ms":
                    observed["streams"]["rect"]["header_intervals"]["max_gap_ms"],
                "raw_rect_identical_header_stamp_pairs":
                    observed["raw_rect_identical_header_stamp_pairs"],
                "unmatched_rectified_header_stamps":
                    observed["rect_only_observed_header_stamps"],
            }
            if (observed["counts"]["raw"] <
                    MIN_RECT_DELIVERY_FRACTION * PUBLISHED_FRAMES
                    or observed["counts"]["rect"] <
                    MIN_RECT_DELIVERY_FRACTION * PUBLISHED_FRAMES
                    or observed["raw_rect_identical_header_stamp_pairs"] <
                    MIN_RECT_DELIVERY_FRACTION * observed["counts"]["rect"]
                    or observed["rect_only_observed_header_stamps"] != 0
                    or observed["streams"]["rect"]["header_intervals"][
                        "gaps_over_250ms"] != 0
                    or observed["faults"]):
                issues.append("SYNTHETIC_RECTIFIED_DELIVERY_INSUFFICIENT")
            if counter_code != 0:
                issues.append("COUNTER_EXIT_" + str(counter_code))
            if publisher_code != 0:
                issues.append("PUBLISHER_EXIT_" + str(publisher_code))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            issues.append("SYNTHETIC_LOG_INVALID:" + str(exc)[:160])

    if issues:
        verdict = "BLOCKED_NO_DEVICE_RECTIFIER_LIFECYCLE"
    else:
        verdict = ("NO_DEVICE_IDLE_RECTIFIER_SHUTDOWN_ONLY" if profile == "idle"
                   else "NO_DEVICE_SYNTHETIC_RECTIFIER_SHUTDOWN_ONLY")
    result = {
        "milestone": "M23P",
        "profile": profile,
        "verdict": verdict,
        "faults": issues[:12],
        "component_exit_class": launch_status,
        "component_launch_exit_code": launch_code,
        "synthetic_publisher_exit_code": publisher_code,
        "synthetic_observer_exit_code": counter_code,
        "crash_context": camera_crash_context(launch_text),
        "observations": observations,
        "device_access": "NONE",
        "synthetic_calibration_only": True,
        "physical_motion_authorized": False,
        "real_camera_continuity_validated": False,
        "camera_imu_calibrated": False,
        "xrslam_tracking_validated": False,
    }
    print("M23P_LIFECYCLE_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23P_LIFECYCLE_VERDICT=" + verdict, flush=True)
    print("M23P_DEVICE_ACCESS=NONE", flush=True)
    return 0 if not issues else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (ValueError, OSError) as exc:
        print("M23P_PRECONDITION_BLOCKED=" + str(exc), file=sys.stderr,
              flush=True)
        sys.exit(2)
