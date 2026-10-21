#!/usr/bin/env python3
"""M23AK: fail-closed audit of in-process executable ELF maps.

No ROS, OS commands, network or files. Called with parsed M23AF same-process
post-factory/on-shutdown maps. This catches distro OpenCV mixing and *any*
observed oneTBB library in the exact composable component process.
"""
from pathlib import PurePosixPath
import re

PREFIX = "/opt/m23aj-opencv/lib/"
PHASES = ("post_factory", "on_shutdown")
REQUIRED = frozenset(("core", "flann", "imgproc", "imgcodecs", "calib3d"))
LIB = re.compile(r"libopencv_([a-z0-9_]+)\.so\.4\.6\.0(?: \(deleted\))?")


class Blocked(ValueError):
    pass


def inspect_maps(parsed):
    if parsed.get("run_id") is None or parsed.get("pid_in_container", 0) <= 0:
        raise Blocked("invalid M23AF same-process identity")
    maps = parsed.get("snapshots")
    if not isinstance(maps, dict) or set(maps) != set(PHASES):
        raise Blocked("require both M23AF mapping phases")
    summary = {}
    for phase in PHASES:
        entries = maps[phase]
        if not isinstance(entries, list) or not entries:
            raise Blocked("missing executable mappings at " + phase)
        modules = set()
        rectifier = False
        for entry in entries:
            path = entry.get("path")
            if not isinstance(path, str) or not path.startswith("/"):
                raise Blocked("invalid executable path at " + phase)
            filename = PurePosixPath(path).name
            if filename.startswith("libtbb"):
                raise Blocked("oneTBB executable mapping present at " + phase)
            if filename == "librectify.so":
                rectifier = True
            if not filename.startswith("libopencv_"):
                continue
            # Fail closed on mixed OpenCV from the old system prefix.
            if not path.startswith(PREFIX):
                raise Blocked("distro/foreign OpenCV executable map at " + phase)
            m = LIB.fullmatch(filename)
            if m is None:
                raise Blocked("wrong alternate OpenCV ABI or deleted mapping at " + phase)
            modules.add(m.group(1))
        if not rectifier:
            raise Blocked("rectifier component mapping absent at " + phase)
        missing = REQUIRED - modules
        if missing:
            raise Blocked("required alternate OpenCV mappings absent at " +
                          phase + ":" + ",".join(sorted(missing)))
        summary[phase] = {
            "opencv_modules": sorted(modules),
            "observed_opencv_prefix": PREFIX,
            "one_tbb_executable_mappings": 0,
            "rectifier_library_mapped": True,
            "executable_mapping_count": len(entries),
        }
    return summary
