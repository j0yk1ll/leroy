#!/usr/bin/env python3
"""M23AM: no-device ELF dynamic-loader compatibility of Leroy's camera stack.

Default inert. Explicit --audit runs 'ldd -r' on *trusted, pinned image*
usb_cam/cv_bridge/image_geometry/rectify ELF files, without starting nodes,
reading devices, recording, or changing host/container packages.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

PREFIX = "/opt/m23aj-opencv/lib/"
MODULES = {"core", "imgproc", "imgcodecs", "flann", "calib3d", "features2d"}
TARGETS = {
    "usb_cam_node_exe":
        "/leroy_ws/install/usb_cam/lib/usb_cam/usb_cam_node_exe",
    "usb_cam_component": "/leroy_ws/install/usb_cam/lib/libusb_cam_node.so",
    "usb_cam_library": "/leroy_ws/install/usb_cam/lib/libusb_cam.so",
    "cv_bridge": "/opt/ros/jazzy/lib/libcv_bridge.so",
    "image_geometry": "/opt/ros/jazzy/lib/libimage_geometry.so",
    "rectify": "/opt/ros/jazzy/lib/librectify.so",
}
RESOLVED = re.compile(r"^\s*(libopencv_[^\s]+)\s+=>\s+(/[^\s]+)")
MISSING = re.compile(r"^\s*([^\s]+)\s+=>\s+not found(?:\s|$)")
TBB = re.compile(r"libtbb(?:malloc(?:_proxy)?)?\.so(?:[.\s]|$)")
UNDEFINED = re.compile(r"undefined symbol:|symbol lookup error|relocation error", re.I)


class Blocked(ValueError):
    pass


def interpret_ldd(output, status, role):
    """Parse ldd -r output, block unresolved symbols/TBB/mixed OpenCV."""
    if status != 0:
        raise Blocked(role + ": ldd -r returned " + str(status))
    if UNDEFINED.search(output):
        raise Blocked(role + ": unresolved relocation/symbol")
    if TBB.search(output):
        raise Blocked(role + ": oneTBB dependency still resolves")
    if "statically linked" in output or "not a dynamic executable" in output:
        raise Blocked(role + ": not dynamically linked")
    bindings = {}
    for line in output.splitlines():
        missing = MISSING.match(line)
        if missing:
            raise Blocked(role + ": missing needed ELF " + missing.group(1))
        match = RESOLVED.match(line)
        if match:
            name, path = match.groups()
            if not path.startswith(PREFIX):
                raise Blocked(role + ": distro/mixed OpenCV " + name)
            if not re.fullmatch(r"libopencv_[a-z0-9_]+\.so\.406", name):
                raise Blocked(role + ": wrong OpenCV SONAME " + name)
            if name in bindings and bindings[name] != path:
                raise Blocked(role + ": ambiguous OpenCV binding " + name)
            bindings[name] = path
        elif "libopencv_" in line:
            raise Blocked(role + ": unparsed OpenCV loader line")
    return bindings


def audit():
    # Rootless Podman normally maps the container process to UID 0;
    # enforce rootlessness at the HOST preflight, not inside the image.
    if os.environ.get("LD_PRELOAD"):
        raise Blocked("LD_PRELOAD is forbidden")
    if os.environ.get("LD_LIBRARY_PATH", "").split(":")[0] != PREFIX.rstrip("/"):
        raise Blocked("replacement OpenCV must have loader precedence")

    pkg = subprocess.run(
        ["ros2", "pkg", "prefix", "usb_cam"],
        capture_output=True, text=True, check=False, timeout=15)
    if pkg.returncode or pkg.stdout.strip() != "/leroy_ws/install/usb_cam":
        raise Blocked("patched usb_cam workspace overlay not selected")

    detail = {}
    all_modules = set()
    for role, file_name in TARGETS.items():
        path = Path(file_name)
        if not path.is_file():
            raise Blocked("required camera stack ELF missing: " + role)
        result = subprocess.run(
            ["ldd", "-r", file_name],
            capture_output=True, text=True, check=False, timeout=18)
        bindings = interpret_ldd(
            result.stdout + "\n" + result.stderr, result.returncode, role)
        for name in bindings:
            all_modules.add(name.removeprefix("libopencv_").split(".so.", 1)[0])
        detail[role] = {
            "needed_opencv": sorted(bindings),
            "resolved_opencv_paths": bindings,
            "dynamic_relocations_resolved": True,
            "tbb_dependencies_detected": 0,
        }
    if not MODULES.issubset(all_modules):
        raise Blocked("required OpenCV modules absent across camera-path ELF closure")
    print("M23AM_RESULT_JSON=" + json.dumps({
        "verdict": "CAMERA_STACK_ELF_LOADER_COMPATIBLE",
        "usb_cam_overlay": pkg.stdout.strip(),
        "loader_prefix": PREFIX,
        "audited_elf_files": detail,
        "all_opencv_modules": sorted(all_modules),
        "tbb_in_audited_dependency_closures": False,
        "ros_nodes_started": False,
        "physical_devices_accessed": False,
        "performance_test_completed": False,
        "full_runtime_abi_proven": False,
        "physical_camera_authorized": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AM_VERDICT=CAMERA_STACK_ELF_LOADER_COMPATIBLE", flush=True)
    print("M23AM_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", action="store_true")
    args = parser.parse_args(argv)
    print("M23AM_DEVICE_ACCESS=NONE", flush=True)
    if not args.audit:
        print("M23AM_STATUS=NO_ELF_READ_OR_COMMAND_BY_DEFAULT", flush=True)
        return 0
    try:
        return audit()
    except (Blocked, OSError, subprocess.TimeoutExpired, ValueError) as exc:
        print("M23AM_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23AM_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
