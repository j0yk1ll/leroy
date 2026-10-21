#!/usr/bin/env python3
"""M23AL: five separate device-free M23AK composable shutdown runs.

Default is inert. --run uses only the pinned rootless image. Stop on first
failed trial, never overwrite old evidence and never touch physical devices.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

COUNT = 5
IMAGE_ID = "334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025"
BASE_ID = "a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777"
IMAGE_TAG = "localhost/leroy-sensors:m23aj-pthreads-opencv-v2"
BASE_TAG = "localhost/leroy-sensors:m23af-shutdown-maps-v1"
PREFIX = "/opt/m23aj-opencv/lib/"
MODULES = {"calib3d", "core", "features2d", "flann", "imgcodecs", "imgproc"}
STATE = Path.home() / ".local/state/leroy-m23al"
RECORD = Path.home() / ".local/state/leroy-m23aj/pthreads-image-id-02.txt"
SCRIPT_DIR = Path(__file__).resolve().parent
SETTINGS = (
    "M23AK_ARM=pthreads", "M23AF_ARM=", "M23AC_ARM=",
    "M23AA_ARM=", "M23AB_ARM=", "M23P_SYNTHETIC_ONLY=1",
    "LEROY_NAV2_DRY_RUN=1", "ROS_DOMAIN_ID=170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST", "M23P_PROFILE=synthetic",
    "M23Y_DRAIN_ONLY=1", "M23S_DISTORTION_PROFILE=rectify",
    "M23R_TRACE_ONLY=0",
)
SHELL = (
    ". /opt/ros/jazzy/setup.sh && . /leroy_ws/install/setup.sh && "
    ". /opt/m23ac-cpp-install/local_setup.sh && "
    ". /opt/m23af-cpp-install/local_setup.sh && "
    'export LD_LIBRARY_PATH="/opt/m23aj-opencv/lib:$LD_LIBRARY_PATH" && '
    "exec python3 -B -u /opt/m23af/m23ak_composable_compare.py"
)


class Blocked(ValueError):
    pass


def one(log, prefix):
    found = [line[len(prefix):] for line in log.splitlines()
             if line.startswith(prefix)]
    if len(found) != 1:
        raise Blocked("expected exactly one " + prefix)
    return found[0]


def obj(log, prefix):
    try:
        data = json.loads(one(log, prefix))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise Blocked("bad JSON " + prefix) from exc
    if not isinstance(data, dict):
        raise Blocked("result is not an object " + prefix)
    return data


def validate(log, exit_code, seen):
    if exit_code != 0:
        raise Blocked("wrapper exit " + str(exit_code))
    nonce = one(log, "M23AK_RUN_ID=")
    if re.fullmatch(r"[0-9a-f]{32}", nonce) is None or nonce in seen:
        raise Blocked("invalid or duplicate nonce")
    if one(log, "M23AA_VERDICT=") != "CLEAN_STOCK_RECTIFIER_900_OF_900":
        raise Blocked("M23AA synthetic validation failed")
    if one(log, "M23AK_VERDICT=") != "TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN":
        raise Blocked("M23AK verdict failed")
    if one(log, "M23AK_DEVICE_ACCESS=") != "NONE":
        raise Blocked("device access marker failed")
    if one(log, "M23AK_PRODUCTION_ACCEPTANCE=") != "BLOCKED":
        raise Blocked("production marker missing")
    base = obj(log, "M23AA_RESULT_JSON=")
    hook = obj(log, "M23AC_RESULT_JSON=")
    result = obj(log, "M23AK_RESULT_JSON=")
    frames = {"raw": 900, "rect": 900, "info": 900}
    if not (
        base.get("arm") == "component"
        and base.get("verdict") == "CLEAN_STOCK_RECTIFIER_900_OF_900"
        and base.get("parent_exit") == base.get("child_exit") == 0
        and base.get("counts") == frames
        and base.get("graph_verified") is True
        and base.get("stock_rectifier") is True
        and base.get("synthetic_only") is True
        and base.get("production_acceptance") == "BLOCKED"
        and hook.get("policy") == "control"
        and hook.get("factory_hook_verified_before_frames") is True
        and hook.get("inherited_m23aa_exit") == 0
        and hook.get("diagnostic_verdict") == "INHERITED_RUN_COMPLETE"
        and result.get("verdict") == "TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN"
        and result.get("run_id") == nonce
        and result.get("pid_in_container", 0) > 0
        and result.get("child_exit") == result.get("parent_exit") == 0
        and result.get("inherited_900_frame_counts") == frames
        and result.get("stock_composable_rectifier") is True
        and result.get("factory_policy") == "control"
        and result.get("opencv_prefix") == PREFIX
        and result.get("tbb_absent_from_two_executable_map_snapshots") is True
        and result.get("production_acceptance") == "BLOCKED"
        and result.get("physical_device_validation") == "NONE"
    ):
        raise Blocked("frame, child, ROS component or factory contract failed")
    maps = result.get("opencv_mapping_audit")
    if not isinstance(maps, dict) or set(maps) != {"post_factory", "on_shutdown"}:
        raise Blocked("incomplete same-process snapshots")
    for phase in ("post_factory", "on_shutdown"):
        values = maps[phase]
        if not isinstance(values, dict) or not (
            values.get("observed_opencv_prefix") == PREFIX
            and values.get("one_tbb_executable_mappings") == 0
            and values.get("rectifier_library_mapped") is True
            and set(values.get("opencv_modules", [])) == MODULES
            and values.get("executable_mapping_count", 0) > 0
        ):
            raise Blocked("mixed OpenCV or TBB mapping at " + phase)
    return nonce


def image_id(tag):
    result = subprocess.run(
        ["podman", "image", "inspect", tag, "--format", "{{.Id}}"],
        capture_output=True, text=True, check=False, timeout=12)
    if result.returncode != 0:
        raise Blocked("image inspect failed: " + tag)
    return result.stdout.strip()


def command():
    args = [
        "timeout", "--signal=TERM", "--kill-after=5s", "120s",
        "podman", "run", "--rm", "--pull=never", "--network=none",
        "--ipc=private", "--pid=private", "--cap-drop=all",
        "--security-opt=no-new-privileges", "--read-only",
        "--tmpfs", "/tmp:rw,nosuid,nodev,size=128m,notmpcopyup",
        "--pids-limit=128", "--memory=2g",
    ]
    for v in SETTINGS:
        args.extend(["--env", v])
    for name in ("m23ak_maps_gate.py", "m23ak_composable_compare.py"):
        path = SCRIPT_DIR / name
        if path.is_symlink() or not path.is_file():
            raise Blocked("missing source file " + name)
        args.extend(["--mount", "type=bind,source=" + str(path) +
                     ",destination=/opt/m23af/" + name + ",ro=true"])
    return args + ["--entrypoint", "/bin/bash", IMAGE_ID, "-lc", SHELL]


def run():
    if os.geteuid() == 0 or os.environ.get("LD_PRELOAD"):
        raise Blocked("rootless without LD_PRELOAD required")
    if RECORD.is_symlink() or not RECORD.is_file() or \
            RECORD.read_text().strip() != IMAGE_ID:
        raise Blocked("M23AJ recorded ID does not match")
    if image_id(IMAGE_TAG) != IMAGE_ID or image_id(BASE_TAG) != BASE_ID:
        raise Blocked("immutable image identity mismatch")
    if STATE.exists() or STATE.is_symlink():
        raise Blocked("refusing to overwrite an existing M23AL evidence directory")
    args = command()
    STATE.mkdir(mode=0o700)
    nonces = set()
    trials = []
    for index in range(1, COUNT + 1):
        log = STATE / ("pthreads-composable-%02d.log" % index)
        with log.open("xb") as output:
            result = subprocess.run(args, stdout=output, stderr=subprocess.STDOUT,
                                    check=False)
        try:
            nonce = validate(log.read_text(errors="replace"), result.returncode, nonces)
        except (Blocked, OSError, ValueError) as exc:
            print("M23AL_FAILED_TRIAL=" + str(index), flush=True)
            print("M23AL_FAILED_EXIT=" + str(result.returncode), flush=True)
            print("M23AL_BLOCKED=" + str(exc)[:180], flush=True)
            print("M23AL_STOP=NO_MORE_TRIALS", flush=True)
            return 2
        nonces.add(nonce)
        trials.append({"trial": index, "run_id": nonce, "child_exit": 0})
        print("M23AL_TRIAL=PASS %d/%d run_id=%s" % (index, COUNT, nonce),
              flush=True)
    print("M23AL_RESULT_JSON=" + json.dumps({
        "status": "FIVE_OF_FIVE_SYNTHETIC_COMPOSABLE_CLEAN",
        "image_id": IMAGE_ID, "trials": trials,
        "unique_run_ids": len(nonces), "camera_testing_authorized": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AL_VERDICT=FIVE_OF_FIVE_SYNTHETIC_COMPOSABLE_CLEAN", flush=True)
    print("M23AL_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    flags = parser.parse_args(argv)
    if not flags.run:
        print("M23AL_STATUS=NO_PODMAN_OR_ROS_BY_DEFAULT")
        return 0
    try:
        return run()
    except (Blocked, OSError, subprocess.SubprocessError, ValueError) as exc:
        print("M23AL_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23AL_STOP=NO_MORE_TRIALS", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
