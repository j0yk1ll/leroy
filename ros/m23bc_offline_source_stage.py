#!/usr/bin/env python3
"""M23BC: offline-only M23BB source copy and compiled-observer contract.

Default INERT. Only --stage under the explicit disconnected build gate writes
a new isolated M23K source copy; never modifies existing/pinned install.
No imports of ROS, device access or image capture.
"""
import argparse
import os
from pathlib import Path
import shutil
import sys

from m23bb_evidence_source_contract import Blocked, once, transform

SUPERVISOR_INPUT = Path("/opt/m23bc-input/m23k_camera_path_probe.py")
CPP_INPUT = Path("/opt/m23bc-input/m23k_cpp")
OUT = Path("/opt/m23bc-variant")
CPP_OUT = Path("/opt/m23bc-counter-source")
BINARY_ROOT = "/opt/m23bc-counter-install/lib/leroy_m23k_camera_counter/"
COUNTER_NAME = "m23k_camera_header_counter"
OLD_RELIABLE = (
    'M23N_COUNTER_PATH = (\n'
    '    "/opt/m23m-cpp-install/lib/leroy_m23k_camera_counter/"\n'
    '    "m23k_camera_header_counter"\n'
    ')'
)
OLD_BASELINE = (
    'COUNTER_PATH = (\n'
    '    "/opt/m23k-cpp-install/lib/leroy_m23k_camera_counter/"\n'
    '    "m23k_camera_header_counter"\n'
    ')'
)


def contract(supervisor, observer):
    new_sup, new_cpp = transform(supervisor, observer)
    new_sup = once(new_sup, OLD_RELIABLE,
        'M23N_COUNTER_PATH = ("' + BINARY_ROOT + '"\n'
        '                     "' + COUNTER_NAME + '")', "M23N_OBSERVER_BINARY")
    new_sup = once(new_sup, OLD_BASELINE,
        'COUNTER_PATH = ("' + BINARY_ROOT + '"\n'
        '                "' + COUNTER_NAME + '")', "M23K_OBSERVER_BINARY")
    if '["ros2", "launch", camera_launch_path]' not in new_sup:
        raise Blocked("M23K supervisor original launch contract absent")
    if new_sup.count('start_new_session=True') != 2:
        raise Blocked("M23K process groups changed")
    if new_sup.count('open("xb")') != 2:
        raise Blocked("exclusive camera+counter host file creation changed")
    if new_cpp.count('receipt_ns() << std::endl;') != 1:
        raise Blocked("per-sample observer flush missing")
    return new_sup, new_cpp


def stage():
    if os.environ.get("M23BC_OFFLINE_BUILD_ONLY") != "YES_NO_DEVICE":
        raise Blocked("disconnected no-device build gate absent")
    observer_src = CPP_INPUT / "src/m23k_camera_header_counter.cpp"
    manifest = CPP_INPUT / "package.xml"
    if any(x.exists() or x.is_symlink() for x in (OUT, CPP_OUT)):
        raise Blocked("isolated output directory already exists")
    if not SUPERVISOR_INPUT.is_file() or SUPERVISOR_INPUT.is_symlink():
        raise Blocked("supervisor input missing or symlink")
    if not observer_src.is_file() or observer_src.is_symlink() or not manifest.is_file():
        raise Blocked("C++ observer source/manifest missing")
    if "<name>leroy_m23k_camera_counter</name>" not in manifest.read_text():
        raise Blocked("wrong M23K C++ package manifest")
    new_sup, new_cpp = contract(SUPERVISOR_INPUT.read_text(), observer_src.read_text())
    # Output only into dedicated new locations in disposable build layer.
    CPP_OUT.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(CPP_INPUT, CPP_OUT)
    (CPP_OUT / "src/m23k_camera_header_counter.cpp").write_text(new_cpp)
    OUT.mkdir(mode=0o755)
    (OUT / "m23k_camera_path_probe.py").write_text(new_sup)
    shutil.copy2("/opt/m23bc-input/m23bb_host_evidence_adapter.py",
                 OUT / "m23bb_host_evidence_adapter.py")
    print("M23BC_STAGED=NEW_SOURCE_COPY_ONLY", flush=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", action="store_true")
    args = p.parse_args(argv)
    if not args.stage:
        print("M23BC_STAGE_STATUS=INERT_NO_WRITES_NO_DEVICE", flush=True)
        return 0
    try:
        stage()
        print("M23BC_STAGE_VERDICT=ISOLATED_SOURCE_COPY_READY", flush=True)
        print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (OSError, UnicodeError, Blocked, ValueError) as exc:
        print("M23BC_STAGE_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
