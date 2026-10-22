#!/usr/bin/env python3
"""M23AU: bounded REAL camera-only test wrapper, inert by default.

NOT an authorization to test hardware. Operator must separately verify
power rail isolation and current by-id identity before a physical run.
Reuses M23K/M23O's existing read-only header-only C++ observer / 45s
supervisor with a new M23AQ two-node camera-only launch. No recording.
"""
import argparse
import json
import os
from pathlib import Path
import re
import sys

CAMERA_LAUNCH = "/opt/m23au/m23aq_camera_only.launch.py"
ALTERNATE_OPENCV = "/opt/m23aj-opencv/lib"
M23AQ_ARGS = {
    "M23AQ_CAMERA_ONLY_OPERATOR_APPROVAL": "REVIEWED_AND_APPROVED",
    "M23AQ_MOTOR_POWER_ISOLATION_ATTESTED": "YES",
    "M23AQ_VIDEO_DEVICE": "/dev/video2",
    "M23AQ_CALIBRATION_VERIFIED": "YES",
    "M23AQ_CALIBRATION_PATH": "/config/camera/front_camera.yaml",
}
ISOLATION = {
    "M23AU_CAMERA_ONLY_OPERATOR_APPROVAL": "YES",
    "M23K_PROBE_MODE": "live",
    "M23K_PROBE_VARIANT": "full_mjpeg",
    "M23K_OPERATOR_CAMERA_ONLY_APPROVED": "YES",
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23O_RECT_QOS_ONLY": "1",
    "M23O_SUB_QOS": "reliable_100",
}


class Blocked(ValueError):
    pass


def gate(environ, exists=Path.is_file):
    for key, required in {**M23AQ_ARGS, **ISOLATION}.items():
        if environ.get(key) != required:
            raise Blocked("missing or wrong explicit gate: " + key)
    if environ.get("LD_PRELOAD"):
        raise Blocked("LD_PRELOAD not permitted")
    if environ.get("LD_LIBRARY_PATH", "").split(":")[0] != ALTERNATE_OPENCV:
        raise Blocked("pthreads OpenCV is not first")
    if environ.get("M23N_CAMERA_QOS_ONLY") == "1":
        raise Blocked("legacy M23N QoS variant forbidden")
    if not exists(Path(CAMERA_LAUNCH)):
        raise Blocked("reviewed two-node source launch not mounted")
    return True


def clean_exit_markers(log):
    """Require both ROS child executables to have logged clean shutdown."""
    if re.search(r"process has died|Segmentation fault", log, re.I):
        raise Blocked("ROS camera or component child crashed")
    for executable in ("usb_cam_node_exe", "component_container"):
        hits = [line for line in log.splitlines()
                if re.search(re.escape(executable) + r"-[0-9]+", line)
                and "process has finished cleanly" in line]
        if len(hits) != 1:
            raise Blocked("missing or duplicate clean exit for " + executable)
    return True


def run():
    gate(os.environ)
    # The trusted original M23K supervisor performs the existing 45s live
    # observation, actual ROS graph/QoS audit, 250ms continuity gate,
    # process-group graceful termination, and separate child-crash scan.
    import m23k_camera_path_probe as m23k

    # Replace only the launch file used by full_mjpeg. The M23AQ
    # source itself refuses launch without operator and calibration gates.
    m23k.PROBE_VARIANTS["full_mjpeg"] = (CAMERA_LAUNCH, True)
    status = m23k.run_camera_only()

    # M23K reports observed timing and faults even when validation fails.
    # A successful m23k return is necessary but NOT sufficient: require
    # BOTH ROS child processes independently finished cleanly at SIGINT.
    launch_log = m23k.CAMERA_LAUNCH_LOG.read_text(errors="replace")
    clean_exit_markers(launch_log)
    if status != 0:
        raise Blocked("inherited M23O rectified continuity and/or graph failed")

    print("M23AU_RESULT_JSON=" + json.dumps({
        "verdict": "BOUNDED_CAMERA_ONLY_DIAGNOSTIC_COMPLETE",
        "inherited_rectified_qos_diagnostic": True,
        "actual_clean_camera_and_rectifier_shutdown": True,
        "capture_duration_requested_s": 45,
        "camera_device_in_container": "/dev/video2",
        "stock_composable_rectifier": True,
        "one_tbb_absence_in_live_process_verified": False,
        "no_recording": True,
        "no_mcu_serial": True,
        "physical_motion_authorized": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AU_VERDICT=BOUNDED_CAMERA_ONLY_DIAGNOSTIC_COMPLETE",
          flush=True)
    print("M23AU_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    options = parser.parse_args(argv)
    if not options.run:
        print("M23AU_STATUS=NO_ROS_OR_CAMERA_BY_DEFAULT", flush=True)
        return 0
    try:
        return run()
    except (Blocked, OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print("M23AU_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:170],
              flush=True)
        print("M23AU_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
