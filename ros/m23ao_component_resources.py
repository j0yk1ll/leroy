#!/usr/bin/env python3
"""M23AO: observe ONLY the M23AF composable rectifier PID's CPU and peak RSS.

Default inert. Explicit --run reuses M23AN/M23AK 900/900 no-device
synthetic experiment. /proc is sampled by a Python thread, and the process
is matched to the exact M23AF post-factory nonce + container PID.
No camera, hardware, network, core inspection or package modification.
"""
import argparse
import contextlib
import io
import json
import os
from pathlib import Path
import re
import sys
import threading
import time

LOG = Path("/tmp/m23p-rectifier-launch.log")
MARKER = re.compile(
    r"M23AF_MAP_BEGIN phase=post_factory run_id=([0-9a-f]{32}) pid=([0-9]+)"
)
VMS = re.compile(r"^VmHWM:\s+([0-9]+)\s+kB$", re.M)
MIN_SAMPLES = 50
MIN_SPAN_S = 20.0


class Blocked(ValueError):
    pass


def parse_stat(line):
    """Use final ')' to handle Linux /proc/pid/stat's parenthesized comm."""
    pos = line.rfind(")")
    if pos == -1 or "(" not in line[:pos]:
        raise Blocked("invalid proc stat comm delimiters")
    fields = line[pos + 1:].strip().split()
    # Remaining fields start at /proc/pid/stat field 3 (state).
    if len(fields) < 22 or not fields[19].isdigit():
        raise Blocked("short proc stat")
    try:
        utime = int(fields[11])
        stime = int(fields[12])
        starttime = int(fields[19])
    except ValueError as exc:
        raise Blocked("non-integer CPU or process identity") from exc
    if min(utime, stime, starttime) < 0 or starttime == 0:
        raise Blocked("invalid CPU or process starttime")
    return starttime, utime + stime


def parse_hwm(status):
    match = VMS.search(status)
    if match is None or int(match.group(1)) <= 0:
        raise Blocked("VmHWM peak RSS unavailable")
    return int(match.group(1))


def compute(samples, ticks_per_sec):
    if len(samples) < MIN_SAMPLES or ticks_per_sec <= 0:
        raise Blocked("insufficient CPU/RSS samples")
    start, end = samples[0], samples[-1]
    elapsed = (end[0] - start[0]) / 1e9
    if elapsed < MIN_SPAN_S:
        raise Blocked("insufficient monitored workload duration")
    delta = end[1] - start[1]
    if delta <= 0:
        raise Blocked("no observable process CPU time")
    peaks = [x[2] for x in samples]
    if min(peaks) <= 0 or max(peaks) < min(peaks):
        raise Blocked("invalid VmHWM observation")
    cpu_seconds = delta / ticks_per_sec
    return {
        "sample_count": len(samples),
        "sampling_interval_target_ms": 200,
        "sample_span_s": round(elapsed, 3),
        "component_process_cpu_seconds": round(cpu_seconds, 3),
        "component_cpu_percent_of_one_logical_core": round(
            cpu_seconds / elapsed * 100, 2),
        "component_peak_rss_mib": round(max(peaks) / 1024, 2),
        "memory_source": "process_/proc/PID/status_VmHWM",
        "cpu_source": "process_/proc/PID/stat_utime_stime",
        "cpu_measurement_scope": "post_factory_to_last_sample",
        "camera_capture_latency_measured": False,
        "opencv_rectify_kernel_time_measured": False,
        "host_total_cpu_measured": False,
        "max_rss_after_last_sample_unobserved": True,
    }


class Monitor:
    def __init__(self):
        self.stop = threading.Event()
        self.error = None
        self.samples = []
        self.nonce = None
        self.pid = None
        self.starttime = None

    def observe(self):
        # Run inside the rootless container's isolated PID namespace.
        try:
            while not self.stop.is_set():
                if self.pid is None:
                    if LOG.is_file():
                        log = LOG.read_text(errors="replace")
                        found = MARKER.findall(log)
                        if len(found) > 1:
                            raise Blocked("ambiguous M23AF monitored child identity")
                        if len(found) == 1:
                            self.nonce, text_pid = found[0]
                            self.pid = int(text_pid)
                            if self.pid <= 1:
                                raise Blocked("invalid monitored child PID")
                    self.stop.wait(0.1)
                    continue
                proc = Path("/proc") / str(self.pid)
                try:
                    argv = (proc / "cmdline").read_bytes()
                    if (b"m23af_run_id:=" + self.nonce.encode()) not in argv:
                        raise Blocked("child argv does not contain exact run nonce")
                    stat_text = (proc / "stat").read_text()
                    status_text = (proc / "status").read_text()
                except FileNotFoundError:
                    break  # Child exited; do not sample a different process.
                identity, ticks = parse_stat(stat_text)
                if self.starttime is None:
                    self.starttime = identity
                elif identity != self.starttime:
                    raise Blocked("child process reused an existing PID")
                peak_kb = parse_hwm(status_text)
                self.samples.append((time.monotonic_ns(), ticks, peak_kb))
                self.stop.wait(0.2)
        except (Blocked, OSError, UnicodeError, ValueError) as exc:
            self.error = type(exc).__name__ + ":" + str(exc)[:150]


def one(log, name):
    matches = [x[len(name):] for x in log.splitlines() if x.startswith(name)]
    if len(matches) != 1:
        raise Blocked("missing or duplicate inherited " + name)
    return matches[0]


def run():
    if os.environ.get("M23AO_ARM") != "resources":
        raise Blocked("require M23AO_ARM=resources")
    import m23an_synthetic_timing as m23an
    monitor = Monitor()
    thread = threading.Thread(target=monitor.observe, daemon=True)
    thread.start()
    output = io.StringIO()
    try:
        with contextlib.redirect_stdout(output):
            result = m23an.run()
    finally:
        monitor.stop.set()
        thread.join(timeout=4)
    sys.stdout.write(output.getvalue())
    sys.stdout.flush()
    if thread.is_alive() or monitor.error:
        raise Blocked("resource observer: " + (monitor.error or "thread did not terminate"))
    if result != 0:
        raise Blocked("inherited M23AN timing trial failed")
    if one(output.getvalue(), "M23AN_VERDICT=") != \
            "SYNTHETIC_OBSERVER_TIMING_RECORDED":
        raise Blocked("inherited timing verdict missing")
    actual_nonce = one(output.getvalue(), "M23AK_RUN_ID=")
    if not monitor.nonce or actual_nonce != monitor.nonce:
        raise Blocked("resource observer monitored wrong ROS child")
    inherited = json.loads(one(output.getvalue(), "M23AK_RESULT_JSON="))
    if monitor.pid != inherited.get("pid_in_container"):
        raise Blocked("resource observer PID does not match mapped child")
    measured = compute(monitor.samples, os.sysconf("SC_CLK_TCK"))
    print("M23AO_RESULT_JSON=" + json.dumps({
        "verdict": "SYNTHETIC_COMPONENT_RESOURCE_MEASURED",
        "run_id": actual_nonce,
        "pid_in_container": monitor.pid,
        "immutable_opencv_image": "334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025",
        "measurements": measured,
        "inherited_m23an_success": True,
        "device_access": "NONE",
        "actual_application_cpu_budget_accepted": False,
        "production_acceptance": "BLOCKED",
        "physical_camera_authorized": False,
    }, sort_keys=True), flush=True)
    print("M23AO_VERDICT=SYNTHETIC_COMPONENT_RESOURCE_MEASURED", flush=True)
    print("M23AO_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    flags = parser.parse_args(argv)
    if not flags.run:
        print("M23AO_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT", flush=True)
        return 0
    try:
        return run()
    except (Blocked, OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print("M23AO_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23AO_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
