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
# M23N opt-in has NO effect on original M23K observer defaults.
M23N_CAMERA_QOS_PROFILES = ("best_effort_100", "reliable_100")
M23N_COUNTER_PATH = (
    "/opt/m23m-cpp-install/lib/leroy_m23k_camera_counter/"
    "m23k_camera_header_counter"
)


def camera_qos_request(environ, variant):
    """Return explicit test-only receiver QoS; never silently use a mode."""
    if environ.get("M23N_CAMERA_QOS_ONLY") != "1":
        return None
    if variant != "raw_only_mjpeg":
        raise ValueError("M23N is restricted to raw_only_mjpeg without rectifier")
    qos = environ.get("M23N_SUB_QOS")
    if qos not in M23N_CAMERA_QOS_PROFILES:
        raise ValueError("M23N QoS must be best_effort_100 or reliable_100")
    return qos


def verified_receiver_qos_log(payload, requested):
    """Require the actual C++ counter's sole selected QoS marker."""
    markers = [line.split("=", 1)[1].strip()
               for line in payload.splitlines()
               if line.startswith("M23M_COUNTER_QOS_PROFILE=")]
    if len(markers) != 1 or markers[0] != requested:
        raise ValueError("missing, duplicate, or mismatched C++ receiver QoS")
    return markers[0]

def rectified_qos_request(environ, variant):
    """Separate opt-in reliable-only M23O diagnostic; never change M23N."""
    if environ.get("M23O_RECT_QOS_ONLY") != "1":
        return None
    if environ.get("M23N_CAMERA_QOS_ONLY") == "1":
        raise ValueError("M23N and M23O modes are mutually exclusive")
    if variant != "full_mjpeg" or environ.get("M23O_SUB_QOS") != "reliable_100":
        raise ValueError("M23O requires full_mjpeg and reliable_100")
    return "reliable_100"


PROBE_VARIANTS = {
    "full_mjpeg": ("/opt/m20/m20_camera_only.launch.py", True),
    "raw_only_mjpeg": ("/opt/m23k/m23k_camera_raw_only.launch.py", False),
    "raw_only_yuyv": ("/opt/m23k/m23k_camera_raw_only.launch.py", False),
}
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

    def report(self, *, required_streams=("raw", "rect", "info")):
        counts = {k: len(v) for k, v in self.stamps.items()}
        if any(counts[k] < 3 for k in required_streams) and "MISSING_CAMERA_STREAM" not in self.faults:
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
                if counts["raw"] and "rect" in required_streams else None),
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


def parse_counter_log(payload, *, require_rect=True):
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
    required = ("raw", "rect", "info") if require_rect else ("raw", "info")
    result = stats.report(required_streams=required)
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
    # ros2 launch can report 130 or -SIGINT for a normal requested
    # interrupt; that is not equivalent to a child-process crash.
    # Child failures (including the observed component SIGSEGV) are
    # identified separately from the launch output log.
    expected_interrupt_exit = (
        sig == signal.SIGINT and code in (130, -int(signal.SIGINT)))
    if code != 0 and not expected_interrupt_exit:
        issues.append(name + "_EXIT_CODE_" + str(code))


def camera_launch_crashed(log_text):
    """ros2 launch may return zero even when a child exited with SIGSEGV."""
    return ("process has died" in log_text
            or "Segmentation fault" in log_text)


def camera_crash_context(log_text, before=14, after=6):
    """Return a bounded, line-numbered window near the first child crash.

    The camera-only container is ephemeral; preserve a small excerpt in the
    operator's tee log without dumping unbounded ROS launch output.
    """
    lines = log_text.splitlines()
    for i, line in enumerate(lines):
        if "process has died" in line or "Segmentation fault" in line:
            start = max(0, i - before)
            end = min(len(lines), i + after + 1)
            return [str(n + 1) + ": " + lines[n][-500:]
                    for n in range(start, end)]
    return []


def run_camera_only():
    _require_live_environment()
    variant = os.environ.get("M23K_PROBE_VARIANT", "full_mjpeg")
    if variant not in PROBE_VARIANTS:
        raise RuntimeError("unrecognized M23K camera probe variant")
    camera_launch_path, require_rect = PROBE_VARIANTS[variant]
    m23o_qos = rectified_qos_request(os.environ, variant)
    requested_qos = camera_qos_request(os.environ, variant) or m23o_qos
    counter_binary = M23N_COUNTER_PATH if requested_qos else COUNTER_PATH
    # C++ processes sample headers without transferring full image payloads
    # into Python. The camera launch and exact M20 rectifier are unchanged.
    if not Path(counter_binary).is_file():
        raise RuntimeError("offline-built C++ header-only observer absent")
    camera = counter = None
    issues = []
    stats_report = None
    qos_verified = False
    graph_report = None
    graph_sampled = False
    with CAMERA_LAUNCH_LOG.open("xb") as camera_output, \
            COUNTER_LOG.open("xb") as counter_output:
        try:
            counter_env = dict(os.environ)
            counter_env["M23K_RAW_ONLY"] = "0" if require_rect else "1"
            if requested_qos:
                counter_env["M23M_SUB_QOS"] = requested_qos
            counter = subprocess.Popen([counter_binary], stdout=counter_output,
                                       stderr=subprocess.STDOUT,
                                       env=counter_env, start_new_session=True)
            time.sleep(1.0)
            if counter.poll() is not None:
                raise RuntimeError("C++ header counter startup failed")
            camera = subprocess.Popen(
                ["ros2", "launch", camera_launch_path],
                stdout=camera_output, stderr=subprocess.STDOUT,
                start_new_session=True)
            # Includes a bounded warm-up; 45-second timing observation.
            started = time.monotonic()
            deadline = started + CAMERA_WARMUP_S + DURATION_S
            while time.monotonic() < deadline:
                if m23o_qos and not graph_sampled and time.monotonic() >= started + 8:
                    graph_sampled = True
                    try:
                        from m23o_rect_qos_graph import validate_graph
                        graph_proc = subprocess.run(
                            ["python3", "/opt/m23o/m23o_rect_qos_graph.py"],
                            text=True, capture_output=True, timeout=12,
                            check=False, env=dict(os.environ),
                        )
                        markers = [
                            line.split("=", 1)[1]
                            for line in graph_proc.stdout.splitlines()
                            if line.startswith("M23O_GRAPH_JSON=")
                        ]
                        if len(markers) != 1:
                            raise ValueError("M23O graph snapshot missing or duplicated")
                        graph_report = json.loads(markers[0])
                        validate_graph(graph_report)
                        if graph_proc.returncode != 0:
                            raise RuntimeError("M23O graph inspector exited " +
                                               str(graph_proc.returncode))
                    except (OSError, ValueError, RuntimeError,
                            subprocess.TimeoutExpired) as exc:
                        issues.append("M23O_GRAPH_AUDIT_FAILED:" +
                                      type(exc).__name__ + ":" + str(exc)[:240])
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
            if requested_qos and counter is not None and counter.poll() is None:
                # Match M23M's 2s drain window after the producer exits.
                time.sleep(2.0)
            _stop_session(counter, signal.SIGINT, 10, issues, "CPP_COUNTER")
            camera_output.flush()
            counter_output.flush()

    camera_text = CAMERA_LAUNCH_LOG.read_text(errors="replace")
    counter_text = COUNTER_LOG.read_text(errors="replace")
    if camera_launch_crashed(camera_text):
        issues.append("CAMERA_CHILD_PROCESS_CRASHED")
        # The container is ephemeral: expose the exact child exit in
        # stdout before --rm destroys its /tmp launch log.
        for line in camera_text.splitlines():
            if ("process has died" in line
                    or "Segmentation fault" in line):
                print("M23K_CAMERA_CHILD_EXIT=" + line[-500:],
                      flush=True)
        if m23o_qos:
            for line in camera_crash_context(camera_text):
                print("M23O_CAMERA_CRASH_CONTEXT=" +
                      json.dumps(line), flush=True)
    if "M23K_COUNTER_FATAL=" in counter_text:
        issues.append("CPP_COUNTER_REPORTED_FATAL")

    try:
        stats_report = parse_counter_log(counter_text,
                                         require_rect=require_rect)
        if requested_qos:
            verified_receiver_qos_log(counter_text, requested_qos)
            qos_verified = True
    except (ValueError, TypeError, KeyError) as exc:
        issues.append(type(exc).__name__ + ": " + str(exc))
        required = ("raw", "rect", "info") if require_rect else ("raw", "info")
        stats_report = CameraStats().report(required_streams=required)
        stats_report["observer_implementation"] = "CPP_HEADER_ONLY_NO_PIXEL_ACCESS"

    if m23o_qos and not graph_sampled:
        issues.append("M23O_GRAPH_AUDIT_NOT_PERFORMED")

    # Preserve measurements even if only camera cleanup failed. A crash
    # cannot produce the original misleading clean diagnostic verdict.
    if requested_qos:
        stats_report["milestone"] = "M23N"
        stats_report["receiver_qos_profile"] = requested_qos
        stats_report["receiver_qos_verified"] = qos_verified
        stats_report["subscriber_drain_window_s"] = 2.0
        stats_report["camera_publisher_rate_verified"] = False
        stats_report["actual_publisher_offered_qos_verified"] = False
    if m23o_qos:
        stats_report["milestone"] = "M23O"
        stats_report["receiver_qos_profile"] = m23o_qos
        stats_report["receiver_qos_verified"] = qos_verified
        stats_report["graph_qos_verified"] = graph_report is not None and not any(
            issue.startswith("M23O_GRAPH_AUDIT_FAILED") for issue in issues)
        stats_report["graph_endpoints"] = graph_report
        stats_report["actual_publisher_offered_qos_verified"] = (
            stats_report["graph_qos_verified"])
        stats_report["rectifier_endpoint_qos_verified"] = (
            stats_report["graph_qos_verified"])
        stats_report["rmw_implementation"] = (
            graph_report.get("rmw_implementation") if graph_report else None)
        stats_report["camera_publisher_rate_verified"] = False
        stats_report["subscriber_drain_window_s"] = 2.0
        stats_report["rectified_image_continuity_validated"] = False
    stats_report["camera_probe_variant"] = variant
    stats_report["rectifier_enabled"] = require_rect
    stats_report["pixel_format"] = (
        "yuyv2rgb" if variant == "raw_only_yuyv" else "mjpeg2rgb")
    stats_report["camera_launch_log"] = str(CAMERA_LAUNCH_LOG)
    stats_report["cpp_observer_log"] = str(COUNTER_LOG)
    stats_report["shutdown_clean"] = not issues
    if issues:
        stats_report["faults"] += issues[:8]
        stats_report["verdict"] = "INCOMPLETE_CAMERA_PATH_OBSERVATION"
    if m23o_qos and stats_report["verdict"] == "CAMERA_PATH_DIAGNOSTIC_ONLY":
        # Diagnostic only: do not weaken M23A's existing 250 ms gate.
        raw = stats_report["streams"]["raw"]
        rect = stats_report["streams"]["rect"]
        good = (
            qos_verified
            and stats_report["graph_qos_verified"]
            and all(
                stream["header_intervals"]["gaps_over_250ms"] == 0
                and stream["observed_rate_hz"] is not None
                and stream["observed_rate_hz"] >= 25
                for stream in (raw, rect)
            )
            and stats_report["counts"]["rect"] >= 3
            and stats_report["rect_only_observed_header_stamps"] == 0
            and stats_report["raw_rect_identical_header_stamp_pairs"] >=
                0.95 * stats_report["counts"]["rect"]
        )
        if good:
            stats_report["rectified_image_continuity_validated"] = True
            stats_report["verdict"] = "RECTIFIED_CAMERA_QOS_DIAGNOSTIC_ONLY"
        else:
            stats_report["faults"].append("M23O_RECTIFIED_CONTINUITY_GATE_FAILED")
            stats_report["verdict"] = "BLOCKED_RECTIFIED_IMAGE_CONTINUITY"
    print("M23K_CAMERA_PROBE_JSON=" +
          json.dumps(stats_report, sort_keys=True), flush=True)
    print("M23K_VERDICT=" + stats_report["verdict"], flush=True)
    if requested_qos and not m23o_qos:
        print("M23N_CAMERA_QOS_JSON=" +
              json.dumps(stats_report, sort_keys=True), flush=True)
        print("M23N_CAMERA_QOS_VERDICT=" + stats_report["verdict"],
              flush=True)
    if m23o_qos:
        print("M23O_RECT_QOS_JSON=" +
              json.dumps(stats_report, sort_keys=True), flush=True)
        print("M23O_RECT_QOS_VERDICT=" + stats_report["verdict"], flush=True)
    print("M23K_MCU_DEVICE_ACCESS=NONE", flush=True)
    return 0 if stats_report["verdict"] in (
        "CAMERA_PATH_DIAGNOSTIC_ONLY",
        "RECTIFIED_CAMERA_QOS_DIAGNOSTIC_ONLY",
    ) else 2


if __name__ == "__main__":
    sys.exit(run_camera_only())
