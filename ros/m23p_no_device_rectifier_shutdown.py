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
from m23l_synthetic_dds_baseline import parse_publisher_log

PROFILES = ("idle", "synthetic")
REQUIRED_ENV = {
    "M23P_SYNTHETIC_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
INSTALL = "/opt/m23m-cpp-install/lib/leroy_m23k_camera_counter/"
PUBLISHER = INSTALL + "m23l_synthetic_image_publisher"
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
            published = parse_publisher_log(
                PUBLISHER_LOG.read_text(errors="replace"))
            observer_log = COUNTER_LOG.read_text(errors="replace")
            observed = parse_counter_log(observer_log, require_rect=True)
            verified_receiver_qos_log(observer_log, "reliable_100")
            observations = {
                "synthetic_published_image_count": published["image"],
                "synthetic_observer_counts": observed["counts"],
                "synthetic_observer_faults": observed["faults"],
                "synthetic_rect_max_header_gap_ms":
                    observed["streams"]["rect"]["header_intervals"]["max_gap_ms"],
            }
            if (observed["counts"]["raw"] < 3 or
                    observed["counts"]["rect"] < 3 or
                    observed["faults"]):
                issues.append("SYNTHETIC_RECTIFIED_DATA_NOT_OBSERVED")
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
