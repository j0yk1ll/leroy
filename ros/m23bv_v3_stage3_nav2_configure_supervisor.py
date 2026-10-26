#!/usr/bin/env python3
"""Stage 3 isolated Nav2 Configure/Cleanup supervisor. No hardware/motion.

Launch ROS via a new process session so SIGINT is not silently ignored by a
background child of noninteractive Bash. The audit NEVER activates Nav2.
This supervisor distinguishes audit PASS from process shutdown PASS.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

LAUNCH = "/opt/stage3_nav2_configure.launch.py"
AUDIT = "/opt/stage3_nav2_configure_audit.py"
LOG = Path("/tmp/stage3-nav2-launch.log")
AUDIT_TIMEOUT = 95
SHUTDOWN_GRACE = 22
KILL_GRACE = 5
FATAL = re.compile(
    r"process has died|exit code -11|exited with code -11|"
    r"segmentation fault|SIGSEGV|Fatal Python error",
    re.IGNORECASE,
)


def audit_environment():
    required = {
        "M23BV_STAGE3_NAV2_CONFIGURE_NO_DEVICE": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError("BLOCKED_ISOLATED_ENV")
    if any(Path(name).exists() for name in ("/dev/video2", "/dev/ttyACM0")):
        raise RuntimeError("BLOCKED_PHYSICAL_DEVICES")
    if not Path(LAUNCH).is_file() or not Path(AUDIT).is_file():
        raise RuntimeError("BLOCKED_REVIEWED_SOURCE_MISSING")


def tail_bounded():
    # This log is a private tmpfs file; output only the bounded final lines.
    try:
        lines = LOG.read_text(encoding="utf-8", errors="replace").splitlines()
        for line in lines[-90:]:
            print("STAGE3_NAV2_LAUNCH_LOG " + line[:1000], file=sys.stderr,
                  flush=True)
    except OSError as err:
        print("STAGE3_NAV2_LOG_READ_ERROR=" + type(err).__name__,
              file=sys.stderr, flush=True)


def stop_group(process, sig):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def shutdown(process):
    # Never leave a background wait unbounded. A clean launch should exit
    # itself on SIGINT and gracefully stop its three managed Nav2 children.
    # Ask the launch coordinator (not the whole process group) to
    # propagate SIGINT exactly once to its managed ROS children.
    # A double SIGINT delivered directly and via ros2 launch can itself
    # turn a graceful lifecycle teardown into a false abnormal exit.
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    forced = False
    try:
        rc = process.wait(timeout=SHUTDOWN_GRACE)
    except subprocess.TimeoutExpired:
        forced = True
        print("STAGE3_NAV2_SHUTDOWN=BLOCKED_SIGINT_TIMEOUT", flush=True)
        stop_group(process, signal.SIGTERM)
        try:
            rc = process.wait(timeout=KILL_GRACE)
        except subprocess.TimeoutExpired:
            stop_group(process, signal.SIGKILL)
            rc = process.wait(timeout=KILL_GRACE)
    print("STAGE3_NAV2_SHUTDOWN_LAUNCH_RC=" + str(rc), flush=True)
    try:
        log_text = LOG.read_text(encoding="utf-8", errors="replace")
    except OSError:
        print("STAGE3_NAV2_SHUTDOWN=BLOCKED_LOG_MISSING", flush=True)
        return False
    fatal = FATAL.search(log_text)
    if fatal:
        print("STAGE3_NAV2_SHUTDOWN=BLOCKED_NAV2_CHILD_CRASH", flush=True)
    elif rc not in (0, 130, -signal.SIGINT):
        print("STAGE3_NAV2_SHUTDOWN=BLOCKED_LAUNCH_EXIT", flush=True)
    elif forced:
        print("STAGE3_NAV2_SHUTDOWN=BLOCKED_FORCED_TERMINATION", flush=True)
    else:
        print("STAGE3_NAV2_SHUTDOWN=PASS_GRACEFUL_PROCESS_EXIT", flush=True)
        return True
    tail_bounded()
    return False


def main():
    audit_environment()
    audit_ok = False
    shutdown_ok = False
    with LOG.open("w", encoding="utf-8") as logfile:
        launch = subprocess.Popen(
            ["ros2", "launch", LAUNCH],
            start_new_session=True, stdin=subprocess.DEVNULL,
            stdout=logfile, stderr=subprocess.STDOUT,
        )
        print("STAGE3_NAV2_LAUNCH_PID_REGISTERED=YES", flush=True)
        try:
            try:
                audit = subprocess.run(
                    [sys.executable, "-B", AUDIT],
                    timeout=AUDIT_TIMEOUT, check=False,
                )
                audit_ok = audit.returncode == 0
                print("STAGE3_NAV2_AUDIT_RC=" +
                      str(audit.returncode), flush=True)
            except subprocess.TimeoutExpired:
                print("STAGE3_NAV2_AUDIT=BLOCKED_TIMEOUT", flush=True)
            if launch.poll() is not None:
                print("STAGE3_NAV2_LAUNCH=BLOCKED_EARLY_EXIT", flush=True)
                audit_ok = False
        finally:
            shutdown_ok = shutdown(launch)
    if audit_ok and shutdown_ok:
        print("STAGE3_NAV2_SUPERVISOR=PASS_AUDIT_AND_CLEAN_SHUTDOWN",
              flush=True)
        return 0
    print("STAGE3_NAV2_SUPERVISOR=BLOCKED_AUDIT_OR_SHUTDOWN",
          flush=True)
    if not shutdown_ok:
        tail_bounded()
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("STAGE3_NAV2_SUPERVISOR=BLOCKED_OPERATOR_INTERRUPT",
              flush=True)
        sys.exit(130)
    except Exception as exc:
        print("STAGE3_NAV2_SUPERVISOR=BLOCKED_" + type(exc).__name__ +
              ":" + str(exc), file=sys.stderr, flush=True)
        sys.exit(2)
