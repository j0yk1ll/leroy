#!/usr/bin/env python3
"""M23BD: read-only offline audit of both M23BC synthetic persistence cases.

Does not execute Podman, ROS, camera/MCU code or even open a device.
Default CLI is inert. Operator-provided exit codes are declarations,
NOT recovered from the persistent data (abnormal process log is empty).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

CAM = b"M23BC_SYNTHETIC_CAMERA_TEXT_A\nM23BC_SYNTHETIC_CAMERA_TEXT_B\n"
OBS = b"M23BC_SYNTHETIC_SUBSCRIBER_TEXT_A\nM23BC_SYNTHETIC_SUBSCRIBER_TEXT_B\n"
SENTINEL = b"M23BB_HOST_BIND_READY_V1\n"
CAM_NAME = "m23bb-camera-launch.log"
OBS_NAME = "m23bb-counter.log"
SENTINEL_NAME = ".m23bb-host-bind-ready"
CLEAN_LOG = (b"M23BC_SYNTHETIC_PERSISTENCE=FILES_WRITTEN\n"
             b"M23BC_CAMERA_RETEST_AUTHORIZED=FALSE\n")
EVIDENCE = {SENTINEL_NAME: SENTINEL, CAM_NAME: CAM, OBS_NAME: OBS}
MAX_EVIDENCE_BYTES = 4096


class Blocked(ValueError):
    pass


def require_regular(path, uid, *, max_bytes=MAX_EVIDENCE_BYTES):
    if path.is_symlink() or not path.is_file():
        raise Blocked("evidence missing/not regular or symlink: " + path.name)
    st = path.stat()
    if (not stat.S_ISREG(st.st_mode) or st.st_uid != uid
            or stat.S_IMODE(st.st_mode) != 0o600 or st.st_nlink != 1):
        raise Blocked("evidence ownership/mode/hardlink incorrect: " + path.name)
    if st.st_size > max_bytes:
        raise Blocked("evidence exceeds size bound: " + path.name)
    data = path.read_bytes()
    if len(data) != st.st_size:
        raise Blocked("evidence changed during audit: " + path.name)
    # Recheck size/metadata after reading to reject an obvious concurrent edit.
    now = path.stat()
    if (now.st_mtime_ns != st.st_mtime_ns or now.st_size != st.st_size
            or now.st_ino != st.st_ino):
        raise Blocked("evidence changed during read: " + path.name)
    return data


def digest(data):
    return hashlib.sha256(data).hexdigest()


def check_directory(directory, expected_name, uid):
    if directory.name != expected_name:
        raise Blocked("wrong evidence case directory: " + directory.name)
    if directory.is_symlink() or not directory.is_dir():
        raise Blocked("evidence directory missing/symlink")
    st = directory.stat()
    if st.st_uid != uid or stat.S_IMODE(st.st_mode) != 0o700:
        raise Blocked("private evidence directory ownership/0700 mismatch")
    files = {item.name for item in directory.iterdir()}
    if files != set(EVIDENCE):
        raise Blocked("missing/extra evidence entries in " + expected_name)
    out = {}
    for name, expected in EVIDENCE.items():
        actual = require_regular(directory / name, uid)
        if actual != expected:
            raise Blocked("synthetic text content differs: " + name)
        out[name] = {"sha256": digest(actual), "bytes": len(actual)}
    return out


def check_log(path, expected_name, expected, uid, parent):
    if path.name != expected_name or path.parent != parent:
        raise Blocked("wrong immutable stdout log location")
    raw = require_regular(path, uid)
    if raw != expected:
        raise Blocked("unexpected synthetic process log content")
    return {"sha256": digest(raw), "bytes": len(raw)}


def audit(clean_dir, abnormal_dir, clean_log, abnormal_log,
          *, clean_exit, abnormal_exit, uid=None):
    if uid is None:
        uid = os.geteuid()
    if type(clean_exit) is not int or type(abnormal_exit) is not int:
        raise Blocked("explicit integer operator exit declarations required")
    if (clean_exit, abnormal_exit) != (0, 73):
        raise Blocked("operator-reported exits do not match 0 and 73")
    if clean_dir.parent != abnormal_dir.parent or clean_dir == abnormal_dir:
        raise Blocked("clean and abnormal cases must be distinct siblings")
    clean = check_directory(clean_dir, "persistence-clean-01", uid)
    abnormal = check_directory(abnormal_dir, "persistence-abnormal-01", uid)
    if clean != abnormal:
        raise Blocked("normal and abnormal synthetic payload hashes differ")
    clean_report = check_log(clean_log, "persistence-clean-01.log",
                             CLEAN_LOG, uid, clean_dir.parent)
    abnormal_report = check_log(abnormal_log, "persistence-abnormal-01.log",
                                b"", uid, clean_dir.parent)
    return {
        "verdict": "OFFLINE_SYNTHETIC_EVIDENCE_POSTFLIGHT_PASS",
        "case_count": 2,
        "clean": {"operator_declared_guest_exit": clean_exit,
                  "process_log": clean_report, "evidence": clean},
        "abnormal": {"operator_declared_guest_exit": abnormal_exit,
                     "process_log": abnormal_report, "evidence": abnormal},
        "synthetic_payload_hashes_match": True,
        "expected_files_only": True,
        "text_only": True,
        "exit_codes_independently_proven_by_files": False,
        "fsync_survival_supported_by_operator_persistence_trial": True,
        "actual_camera_trace_completeness_checked": False,
        "ros_or_camera_node_executed": False,
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--clean-dir", type=Path)
    p.add_argument("--abnormal-dir", type=Path)
    p.add_argument("--clean-log", type=Path)
    p.add_argument("--abnormal-log", type=Path)
    p.add_argument("--clean-exit", type=int)
    p.add_argument("--abnormal-exit", type=int)
    args = p.parse_args(argv)
    provided = (args.clean_dir, args.abnormal_dir, args.clean_log,
                args.abnormal_log, args.clean_exit, args.abnormal_exit)
    if all(v is None for v in provided):
        print("M23BD_STATUS=INERT_READ_ONLY_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        if any(v is None for v in provided):
            raise Blocked("all six postflight inputs are mandatory")
        report = audit(args.clean_dir, args.abnormal_dir,
                       args.clean_log, args.abnormal_log,
                       clean_exit=args.clean_exit,
                       abnormal_exit=args.abnormal_exit)
        print("M23BD_RESULT_JSON=" + json.dumps(report, sort_keys=True),
              flush=True)
        print("M23BD_POSTFLIGHT_VERDICT=PASS_SYNTHETIC_ONLY", flush=True)
        print("M23BD_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (OSError, ValueError, TypeError) as exc:
        print("M23BD_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23BD_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
