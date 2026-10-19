#!/usr/bin/env python3
"""M23K: bounded camera-only RAW vs RECT ROS topic observer.

Operator-invoked live mode opens ONLY the verified front USB camera via
M20's isolated camera-only launch. No STM32, serial port, IMU stream,
bridge, XRSLAM, cmd_vel, motors, recorder or physical calibration.
A separately built C++ node reads only image headers; results are
still affected by DDS transfer and image-message deserialization, and
are diagnostic rather than proof of any specific processing bottleneck.
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


COUNTER_PATH = (
    "/opt/m23k-cpp-install/lib/leroy_m23k_camera_counter/"
    "m23k_camera_header_counter"
)
COUNTER_LOG = Path("/tmp/m23k-cpp-counter.log")
CAMERA_LAUNCH_LOG = Path("/tmp/m23k-camera-launch.log")


def parse_counter_log(payload):
    """Never claim a complete measurement from absent/truncated C++ logs."""
    stats = CameraStats()
    summary = None
    faults = []
    for line in payload.splitlines():
        if line.startswith("M23K_COUNTER_FATAL="):
            faults.append(line[:240])
        elif line.startswith("M23K_COUNTER_COUNTS="):
            if summary is not None:
                raise ValueError("duplicate C++ counter summary")
            fields = line.split("=", 1)[1].split(",")
            if len(fields) != 3:
                raise ValueError("malformed C++ counts")
            summary = {k: int(n) for k, n in zip(TOPICS, fields)}
        elif line.startswith("M23K_SAMPLE "):
            fields = line.split()
            if len(fields) != 4:
                raise ValueError("malformed header sample line")
            _marker, kind, header_ns, receipt_ns = fields
            stats.add(kind, int(header_ns), int(receipt_ns))
    if faults:
        raise ValueError("counter failure: " + ", ".join(faults[:3]))
    if summary is None:
        raise ValueError("C++ counter did not finalize")
    if summary != {k: len(v) for k, v in stats.stamps.items()}:
        raise ValueError("C++ sample counts disagree with final summary")
    result = stats.report()
    result["observer_implementation"] = "CPP_HEADER_ONLY_NO_PIXEL_ACCESS"
    return result


def _stop_session(proc, sig, grace, issues, name):
    """Bounded graceful exit, never silently claim a clean shutdown."""
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass
    try:
        code = proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        issues.append(name + "_SHUTDOWN_TIMEOUT")
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            code = proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            issues.append(name + "_STILL_RUNNING")
            return
    if code != 0:
        issues.append(name + "_EXIT_CODE_" + str(code))


def run_camera_only():
    _require_live_environment()
    # C++ processes sample headers without transferring full image payloads
    # into Python. The camera launch and exact M20 rectifier are unchanged.
    if not Path(COUNTER_PATH).is_file():
        raise RuntimeError("offline-built C++ header-only observer absent")
    camera = counter = None
    issues = []
    stats_report = None
    with CAMERA_LAUNCH_LOG.open("xb") as camera_output, \
            COUNTER_LOG.open("xb") as counter_output:
        try:
            counter = subprocess.Popen([COUNTER_PATH], stdout=counter_output,
                                       stderr=subprocess.STDOUT,
                                       start_new_session=True)
            time.sleep(1.0)
            if counter.poll() is not None:
                raise RuntimeError("C++ header counter startup failed")
            camera = subprocess.Popen(
                ["ros2", "launch", "/opt/m20/m20_camera_only.launch.py"],
                stdout=camera_output, stderr=subprocess.STDOUT,
                start_new_session=True)
            # Includes a bounded warm-up; 45-second timing observation.
            deadline = time.monotonic() + CAMERA_WARMUP_S + DURATION_S
            while time.monotonic() < deadline:
                if camera.poll() is not None:
                    raise RuntimeError("camera launch exited early")
                if counter.poll() is not None:
                    raise RuntimeError("C++ header counter exited early")
                time.sleep(0.2)
        except (OSError, RuntimeError, KeyboardInterrupt) as exc:
            issues.append(type(exc).__name__ + ": " + str(exc))
        finally:
            # Stop camera first and wait for actual ROS launch children.
            # A component segfault may be masked by a zero ros2 launch code;
            # detect "process has died" explicitly in its saved log.
            _stop_session(camera, signal.SIGINT, 15, issues, "CAMERA_LAUNCH")
            _stop_session(counter, signal.SIGINT, 10, issues, "CPP_COUNTER")
            camera_output.flush()
            counter_output.flush()

    camera_text = CAMERA_LAUNCH_LOG.read_text(errors="replace")
    counter_text = COUNTER_LOG.read_text(errors="replace")
    if ("process has died" in camera_text
            or "Segmentation fault" in camera_text):
        issues.append("CAMERA_CHILD_PROCESS_CRASHED")
    if "M23K_COUNTER_FATAL=" in counter_text:
        issues.append("CPP_COUNTER_REPORTED_FATAL")

    try:
        stats_report = parse_counter_log(counter_text)
    except (ValueError, TypeError, KeyError) as exc:
        issues.append(type(exc).__name__ + ": " + str(exc))
        stats_report = CameraStats().report()
        stats_report["observer_implementation"] = "CPP_HEADER_ONLY_NO_PIXEL_ACCESS"

    # Preserve measurements even if only camera cleanup failed. A crash
    # cannot produce the original misleading clean diagnostic verdict.
    stats_report["camera_launch_log"] = str(CAMERA_LAUNCH_LOG)
    stats_report["cpp_observer_log"] = str(COUNTER_LOG)
    stats_report["shutdown_clean"] = not issues
    if issues:
        stats_report["faults"] += issues[:8]
        stats_report["verdict"] = "INCOMPLETE_CAMERA_PATH_OBSERVATION"
    print("M23K_CAMERA_PROBE_JSON=" +
          json.dumps(stats_report, sort_keys=True), flush=True)
    print("M23K_VERDICT=" + stats_report["verdict"], flush=True)
    print("M23K_MCU_DEVICE_ACCESS=NONE", flush=True)
    return 0 if stats_report["verdict"] == "CAMERA_PATH_DIAGNOSTIC_ONLY" else 2


if __name__ == "__main__":
    sys.exit(run_camera_only())
