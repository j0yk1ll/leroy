#!/usr/bin/env python3
"""Bounded Stage 3 real-camera supervisor. NEVER opens MCU or commands motors."""
import os
import signal
import stat
import subprocess
import sys
import threading
import time

CAMERA_LAUNCH = "/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py"
REQUIRED = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23BV_CAMERA_READY_CANDIDATE": "1",
    "M23BV_RELIABLE_RECTIFIER_PROTO_V3": "1",
    "M23BV_STAGE3_PHYSICAL_CAMERA_ONLY": "YES",
}
READY_TIMEOUT_S = 45
MAX_RUN_S = 145


def check():
    for key, value in REQUIRED.items():
        if os.environ.get(key) != value:
            raise RuntimeError("BLOCKED_ENV_" + key)
    if os.path.exists("/dev/ttyACM0"):
        raise RuntimeError("BLOCKED_MOTOR_CAPABLE_SERIAL_VISIBLE")
    if not stat.S_ISCHR(os.stat("/dev/video2").st_mode):
        raise RuntimeError("BLOCKED_CAMERA_NOT_CHARACTER_DEVICE")
    if not os.path.isfile("/config/camera/front_camera.yaml"):
        raise RuntimeError("BLOCKED_CAMERA_CALIBRATION_MISSING")
    if not os.path.isfile(CAMERA_LAUNCH):
        raise RuntimeError("BLOCKED_ACCEPTED_CAMERA_LAUNCH_MISSING")


class Child:
    def __init__(self, name, argv):
        self.name = name
        self.lines = []
        self.proc = subprocess.Popen(
            argv, start_new_session=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, encoding="utf-8", errors="replace",
            bufsize=1)
        self.reader = threading.Thread(target=self._relay, daemon=True)
        self.reader.start()

    def _relay(self):
        for line in self.proc.stdout:
            self.lines.append(line.strip())
            if len(self.lines) > 1000:
                del self.lines[:500]
            print(self.name + " " + line, end="", flush=True)

    def contains(self, marker):
        return any(marker in line for line in self.lines)

    def stop(self):
        import os
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
        try:
            self.proc.wait(timeout=18)
        except subprocess.TimeoutExpired:
            if self.proc.poll() is None:
                os.killpg(self.proc.pid, signal.SIGTERM)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if self.proc.poll() is None:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
                return False
        self.reader.join(timeout=4)
        return (self.proc.returncode == 0 and not self.reader.is_alive())


def run():
    check()
    camera = depthart = None
    good = False
    clean = False
    try:
        camera = Child("STAGE3_CAMERA", ["ros2", "launch", CAMERA_LAUNCH])
        deadline = time.monotonic() + READY_TIMEOUT_S
        while time.monotonic() < deadline:
            if camera.proc.poll() is not None:
                raise RuntimeError("BLOCKED_CAMERA_LAUNCH_EARLY_EXIT")
            if camera.contains("M23BV_CANDIDATE_CAMERA_READINESS=PASS"):
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("BLOCKED_CAMERA_READINESS_TIMEOUT")
        print("M23BV_STAGE3_REAL_CAMERA_READY=PASS", flush=True)
        depthart = Child(
            "STAGE3_DEPTHART",
            ["ros2", "launch", "leroy_depthart", "depthart.launch.py"])
        start = time.monotonic()
        print("M23BV_STAGE3_DEPTHART_LAUNCHED=YES", flush=True)
        while time.monotonic() - start < MAX_RUN_S:
            if camera.proc.poll() is not None:
                raise RuntimeError("BLOCKED_CAMERA_PROCESS_EXIT")
            if depthart.proc.poll() is not None:
                raise RuntimeError("BLOCKED_DEPTHART_PROCESS_EXIT")
            if any("process has died" in line for line in camera.lines):
                raise RuntimeError("BLOCKED_CAMERA_CHILD_EXIT")
            if any("process has died" in line for line in depthart.lines):
                raise RuntimeError("BLOCKED_DEPTHART_CHILD_EXIT")
            time.sleep(0.2)
        # This is an upper duration bound, not a requirement to run the
        # camera for 145 s; the host audit normally ends the trial sooner.
        good = True
    except KeyboardInterrupt:
        good = True  # Only clean if child shutdown succeeds below.
    finally:
        depth_ok = depthart.stop() if depthart is not None else True
        camera_ok = camera.stop() if camera is not None else False
        clean = depth_ok and camera_ok
        print("M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=" +
              ("PASS" if clean else "BLOCKED_CHILD_EXIT"), flush=True)
        print("M23BV_STAGE3_MCU_ACCESS=NONE", flush=True)
    return 0 if good and clean else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except Exception as exc:
        print("M23BV_STAGE3_CAMERA_SUPERVISOR=BLOCKED_" + repr(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
