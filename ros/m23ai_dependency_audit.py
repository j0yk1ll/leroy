#!/usr/bin/env python3
"""M23AI Gate A: no-device, dependency-only OpenCV/TBB audit.

Default: no file read/subprocess. --audit: inspect only installed ELF
dynamic sections via readelf. No ROS, image processing, recording, or
physical devices. For use in the pinned M23AF rootless image.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

LIBDIR = Path("/usr/lib/x86_64-linux-gnu")
ROS_LIBDIR = Path("/opt/ros/jazzy/lib")
NEEDED = re.compile(r"\(NEEDED\)\s+Shared library:\s*\[([^\]]+)\]")
TBB = re.compile(r"^libtbb(?:malloc(?:_proxy)?)?\.so(?:\.|$)")
CORE = re.compile(r"^libopencv_core\.so(?:\.|$)")


class Blocked(RuntimeError):
    pass


def parse_needed(output):
    names = NEEDED.findall(output)
    if len(names) != len(set(names)):
        raise Blocked("duplicate DT_NEEDED entries")
    return names


def needed_of(path):
    p = subprocess.run(
        ["readelf", "-d", str(path)],
        text=True, capture_output=True, check=False, timeout=8)
    if p.returncode:
        raise Blocked("readelf failed for " + path.name)
    return parse_needed(p.stdout)


def find_opencv_files(root):
    if not root.is_dir():
        raise Blocked("system library directory missing")
    found = {}
    for path in sorted(root.glob("libopencv_*.so*")):
        if path.is_file():
            resolved = path.resolve()
            if resolved.is_file():
                found[str(resolved)] = resolved
    # Strict cap; never recurse outside the known distro library folder.
    if not 2 <= len(found) <= 160:
        raise Blocked("implausible number of installed OpenCV DSOs")
    return list(found.values())


def collect(roots=None, getter=needed_of):
    root, ros = roots if roots is not None else (LIBDIR, ROS_LIBDIR)
    opencv = find_opencv_files(root)
    members = []
    for path in opencv:
        needed = getter(path)
        members.append({
            "name": path.name,
            "needed": needed,
            "tbb_needed": sorted(n for n in needed if TBB.match(n)),
        })
    cores = [m for m in members if CORE.match(m["name"])]
    if len(cores) != 1:
        raise Blocked("OpenCV core DSO missing or duplicated")
    targets = {
        "rectify": ros / "librectify.so",
        "cv_bridge": ros / "libcv_bridge.so",
        "image_geometry": ros / "libimage_geometry.so",
    }
    ros_needed = {}
    for role, path in targets.items():
        if path.is_file():
            ros_needed[role] = getter(path)
        elif role == "rectify":
            raise Blocked("stock Jazzy librectify.so not found")
    core = cores[0]
    tbb_members = [m for m in members if m["tbb_needed"]]
    return {
        "core_dso": core["name"],
        "core_needed": core["needed"],
        "core_direct_tbb": bool(core["tbb_needed"]),
        "opencv_shared_objects_inspected": len(members),
        "opencv_direct_tbb_consumers": [
            {"dso": m["name"], "tbb": m["tbb_needed"]} for m in tbb_members],
        "ros_component_needed": ros_needed,
        "replacement_plan": "REBUILD_MATCHING_OPENCV_WITH_PTHREADS",
        "runtime_tbb_absence_verified": False,
        "alternative_built": False,
        "synthetic_component_trial_performed": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--audit", action="store_true")
    args = ap.parse_args(argv)
    print("M23AI_RUNTIME_ROS=NONE", flush=True)
    print("M23AI_DEVICE_ACCESS=NONE", flush=True)
    if not args.audit:
        print("M23AI_STATUS=NO_LIBRARY_INSPECTION_BY_DEFAULT", flush=True)
        return 0
    try:
        result = collect()
        version = subprocess.run(
            ["dpkg-query", "-W", "-f=${Version}", "libopencv-core406t64"],
            text=True, capture_output=True, check=False, timeout=8)
        if version.returncode != 0 or not version.stdout.strip().startswith("4.6.0"):
            raise Blocked("unexpected distro OpenCV core package version")
        result["distro_opencv_package_version"] = version.stdout.strip()
        print("M23AI_BASELINE_JSON=" + json.dumps(result, sort_keys=True),
              flush=True)
        if not result["core_direct_tbb"]:
            raise Blocked("OpenCV core has no direct TBB dependency; pause the rebuild hypothesis")
        print("M23AI_BASELINE_VERDICT=OPENCV_CORE_LINKS_TBB", flush=True)
        print("M23AI_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 0
    except (Blocked, OSError, ValueError, subprocess.TimeoutExpired) as e:
        print("M23AI_BLOCKED=" + type(e).__name__ + ":" + str(e)[:180],
              flush=True)
        print("M23AI_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
