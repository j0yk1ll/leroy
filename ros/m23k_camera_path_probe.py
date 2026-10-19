#!/usr/bin/env python3
"""M23K: bounded camera-only RAW vs RECT ROS topic observer.

Operator-invoked live mode opens ONLY the verified front USB camera via
M20's isolated camera-only launch. No STM32, serial port, IMU stream,
bridge, XRSLAM, cmd_vel, motors, recorder or physical calibration.
Subscription/deserialization adds CPU/DDS load; results are diagnostic,
not proof of which processing stage is at fault.
"""
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time

TOPICS = {
    "raw": "/camera/image_raw",
    "rect": "/camera/image_rect",
    "info": "/camera/camera_info",
}
DURATION_S = 45
CAMERA_WARMUP_S = 3
MAX_MESSAGES = 5000
GAP_NS = 250_000_000
EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23K_OPERATOR_CAMERA_ONLY_APPROVED": "YES",
}


class CameraStats:
    def __init__(self):
        self.stamps = {kind: [] for kind in TOPICS}
        self.receipts = {kind: [] for kind in TOPICS}
        self.faults = []

    def add(self, kind, header_ns, receipt_ns):
        if kind not in TOPICS:
            raise ValueError("unexpected sensor type")
        if (type(header_ns) is not int or header_ns <= 0
                or type(receipt_ns) is not int or receipt_ns <= 0):
            raise ValueError("invalid positive nanosecond timestamp")
        if len(self.stamps[kind]) >= MAX_MESSAGES:
            raise ValueError("bounded observations exceeded")
        if self.stamps[kind] and header_ns <= self.stamps[kind][-1]:
            if "NONMONOTONIC_" + kind.upper() not in self.faults:
                self.faults.append("NONMONOTONIC_" + kind.upper())
        self.stamps[kind].append(header_ns)
        self.receipts[kind].append(receipt_ns)

    @staticmethod
    def interval_summary(sequence):
        durations = [sequence[i] - sequence[i - 1]
                     for i in range(1, len(sequence))]
        positive = [dt for dt in durations if dt > 0]
        return {
            "max_gap_ms": (round(max(positive) / 1e6, 3)
                           if positive else None),
            "gaps_over_250ms": sum(dt > GAP_NS for dt in durations),
            "nonpositive_gaps": sum(dt <= 0 for dt in durations),
            "mean_interval_ms": (round(sum(positive) / len(positive) / 1e6, 3)
                                 if positive else None),
        }

    def report(self):
        counts = {k: len(v) for k, v in self.stamps.items()}
        if any(n < 3 for n in counts.values()) and "MISSING_CAMERA_STREAM" not in self.faults:
            self.faults.append("MISSING_CAMERA_STREAM")
        ranges = {}
        for kind in TOPICS:
            rx = self.receipts[kind]
            elapsed = ((rx[-1] - rx[0]) / 1e9 if len(rx) >= 2
                       else None)
            ranges[kind] = {
                "count": counts[kind],
                "observed_duration_s": (round(elapsed, 3)
                                        if elapsed is not None else None),
                "observed_rate_hz": (round((counts[kind] - 1) / elapsed, 3)
                                     if elapsed and elapsed > 0 else None),
                "header_intervals": self.interval_summary(self.stamps[kind]),
                "receipt_intervals": self.interval_summary(rx),
            }
        raw = set(self.stamps["raw"])
        rect = set(self.stamps["rect"])
        return {
            "milestone": "M23K",
            "verdict": ("INCOMPLETE_CAMERA_PATH_OBSERVATION"
                        if self.faults else "CAMERA_PATH_DIAGNOSTIC_ONLY"),
            "requested_duration_s": DURATION_S,
            "counts": counts,
            "streams": ranges,
            "raw_rect_identical_header_stamp_pairs": len(raw & rect),
            "raw_only_observed_header_stamps": len(raw - rect),
            "rect_only_observed_header_stamps": len(rect - raw),
            "rect_over_raw_count_ratio": (
                round(counts["rect"] / counts["raw"], 4)
                if counts["raw"] else None),
            "faults": list(self.faults),
            "independent_best_effort_subscribers_can_drop": True,
            "observer_cpu_and_dds_load_can_affect_rates": True,
            "no_stage_bottleneck_proven": True,
            "imu_recorded": False,
            "camera_imu_phase_calibrated": False,
            "xrslam_tracking_validated": False,
            "device_access": "CAMERA_ONLY_BY_OPERATOR",
            "stm32_serial_access": False,
            "physical_motion_authorized": False,
        }


def _stamp(header):
    stamp = header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("bad ROS header stamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _require_live_environment():
    for key, expected in EXPECTED_ENV.items():
        if os.environ.get(key) != expected:
            raise RuntimeError("M23K environment mismatch: " + key)
    if not stat.S_ISCHR(os.stat("/dev/video2").st_mode):
        raise RuntimeError("mapped camera device absent")
    if any(Path(p).exists() for p in ("/dev/ttyACM0", "/dev/ttyACM1")):
        raise RuntimeError("M23K camera-only probe refuses MCU device")
    if not Path("/config/camera/front_camera.yaml").is_file():
        raise RuntimeError("front camera calibration missing")


def run_camera_only():
    _require_live_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo

    rclpy.init(args=[])
    node = None
    camera = None
    stats = CameraStats()
    issues = []
    try:
        node = Node("m23k_raw_rect_timing_probe",
                    enable_rosout=False, start_parameter_services=False)
        for name in ("/cmd_vel", "/odom", "/tf", "/depthart/depth"):
            if node.count_publishers(name):
                raise RuntimeError("forbidden ROS graph publisher: " + name)

        def subscriber(kind):
            def callback(msg):
                try:
                    stats.add(kind, _stamp(msg.header), time.monotonic_ns())
                except ValueError as exc:
                    if "BAD_" + kind.upper() not in stats.faults:
                        stats.faults.append("BAD_" + kind.upper())
                    issues.append(str(exc))
            return callback

        subscriptions = [
            node.create_subscription(
                Image, TOPICS["raw"], subscriber("raw"), qos_profile_sensor_data),
            node.create_subscription(
                Image, TOPICS["rect"], subscriber("rect"), qos_profile_sensor_data),
            node.create_subscription(
                CameraInfo, TOPICS["info"], subscriber("info"),
                qos_profile_sensor_data),
        ]
        camera = subprocess.Popen(
            ["ros2", "launch", "/opt/m20/m20_camera_only.launch.py"],
            start_new_session=True)
        end = time.monotonic() + CAMERA_WARMUP_S + DURATION_S
        while time.monotonic() < end:
            if camera.poll() is not None:
                raise RuntimeError("camera launch exited before probe complete")
            for name in ("/cmd_vel", "/odom", "/tf", "/depthart/depth"):
                if node.count_publishers(name):
                    raise RuntimeError("forbidden ROS graph publisher: " + name)
            rclpy.spin_once(node, timeout_sec=0.05)
        del subscriptions
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        issues.append(type(exc).__name__ + ": " + str(exc))
    finally:
        if camera is not None and camera.poll() is None:
            try:
                os.killpg(camera.pid, signal.SIGINT)
                camera.wait(timeout=15)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                issues.append("CAMERA_LAUNCH_SHUTDOWN_UNVERIFIED")
                if camera.poll() is None:
                    os.killpg(camera.pid, signal.SIGTERM)
                    try:
                        camera.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        issues.append("CAMERA_LAUNCH_STILL_RUNNING")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = stats.report()
    if issues:
        result["faults"] += issues[:5]
        result["verdict"] = "INCOMPLETE_CAMERA_PATH_OBSERVATION"
    print("M23K_CAMERA_PROBE_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23K_VERDICT=" + result["verdict"], flush=True)
    print("M23K_MCU_DEVICE_ACCESS=NONE", flush=True)
    return 0 if result["verdict"] == "CAMERA_PATH_DIAGNOSTIC_ONLY" else 2


if __name__ == "__main__":
    sys.exit(run_camera_only())
