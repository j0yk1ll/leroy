#!/usr/bin/env python3
"""M23AZ: device-free ctypes dlopen of isolated diagnostic camera component.

Default is inert. --run requires explicit no-device gates and strict absence
of V4L2/MCU nodes. It loads, but DOES NOT instantiate, UsbCamNode or ROS
executors; it does not open image devices, publish, subscribe or record.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import sys

NODE = "/opt/m23ay-install/lib/libusb_cam_node.so"
CORE = "/opt/m23ay-install/lib/libusb_cam.so"
OPENCV = "/opt/m23aj-opencv/lib"
# Exact Ament search roots: diagnostic usb_cam resource takes precedence; no legacy workspace.
AMENT_PREFIX_PATH = "/opt/m23ay-install:/opt/ros/jazzy"
DENIED_DEVICE_PATTERNS = ("/dev/video*", "/dev/ttyACM*", "/dev/ttyUSB*")
ENVIRONMENT = {
    "M23AZ_OFFLINE_DLOPEN_APPROVED": "YES_NO_DEVICES",
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


class Blocked(ValueError):
    pass


def gate(env, *, exists=Path.is_file, is_dir=Path.is_dir,
         globs=None, usb_bus_exists=None):
    if globs is None:
        globs = lambda pattern: list(Path("/").glob(pattern.lstrip("/")))
    if usb_bus_exists is None:
        usb_bus_exists = lambda: Path("/dev/bus/usb").exists()
    for key, required in ENVIRONMENT.items():
        if env.get(key) != required:
            raise Blocked("missing or wrong isolation gate: " + key)
    # ROS/ament plugin static constructors may terminate the whole Python
    # interpreter on dlopen when AMENT_PREFIX_PATH is absent; verify this
    # BEFORE entering ctypes.CDLL, not in a recoverable exception handler.
    if env.get("AMENT_PREFIX_PATH") != AMENT_PREFIX_PATH:
        raise Blocked("AMENT_PREFIX_PATH missing or wrong order: diagnostic first")
    for prefix in AMENT_PREFIX_PATH.split(":"):
        if not is_dir(Path(prefix)):
            raise Blocked("AMENT_PREFIX_PATH prefix directory missing: " + prefix)
    if not exists(Path("/opt/m23ay-install/share/ament_index/resource_index/packages/usb_cam")):
        raise Blocked("diagnostic usb_cam ament package index missing")
    if not exists(Path("/opt/ros/jazzy/share/ament_index/resource_index/packages/rclcpp_components")):
        raise Blocked("ROS Jazzy rclcpp_components ament package index missing")
    if env.get("LD_PRELOAD"):
        raise Blocked("LD_PRELOAD forbidden")
    if env.get("LD_LIBRARY_PATH", "").split(":")[:2] != [
        "/opt/m23ay-install/lib", OPENCV
    ]:
        raise Blocked("diagnostic core and pthreads OpenCV loader precedence absent")
    if not exists(Path(NODE)) or not exists(Path(CORE)):
        raise Blocked("compiled diagnostic libraries missing")
    for pattern in DENIED_DEVICE_PATTERNS:
        if list(globs(pattern)):
            raise Blocked("camera/serial device exposed: " + pattern)
    if usb_bus_exists():
        raise Blocked("USB bus exposed in device-free guest")
    return True


def mapped_libraries(text):
    """Inspect guest process maps; reject TBB or mixed camera/OpenCV DSOs."""
    if not isinstance(text, str) or not text:
        raise Blocked("no process loader maps")
    seen = {}
    for line in text.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        path = fields[5]
        basename = path.rsplit("/", 1)[-1]
        if re.match(r"libtbb(?:malloc|bind)?(?:[_.]|\.so)", basename):
            raise Blocked("TBB mapped into no-device camera component process")
        if basename.startswith("libusb_cam") and ".so" in basename:
            if not path.startswith("/opt/m23ay-install/lib/"):
                raise Blocked("legacy/unexpected camera DSO mapped: " + basename)
            seen[path] = "camera"
        if basename.startswith("libopencv_") and ".so" in basename:
            if not path.startswith(OPENCV + "/"):
                raise Blocked("mixed system OpenCV DSO mapped: " + basename)
            seen[path] = "opencv"
    camera = [p for p,v in seen.items() if v == "camera"]
    opencv = [p for p,v in seen.items() if v == "opencv"]
    if not any(p.endswith("/libusb_cam_node.so") for p in camera):
        raise Blocked("diagnostic component DSO not actually mapped")
    if not any(p.endswith("/libusb_cam.so") for p in camera):
        raise Blocked("diagnostic core DSO not actually mapped")
    if not any("/libopencv_core.so" in p for p in opencv):
        raise Blocked("pthreads OpenCV core DSO not actually mapped")
    return {
        "mapped_diagnostic_cam_libraries": sorted(set(camera)),
        "mapped_pthreads_opencv_libraries": sorted(set(opencv)),
        "tbb_mappings": 0,
        "mixed_camera_or_opencv_libraries": 0,
    }


def run():
    gate(os.environ)
    # A CDLL has no ROS node construction, no class_loader instantiation,
    # no camera configure/open, and no RCL/RMW initialization.
    flags = getattr(os, "RTLD_NOW", 2) | getattr(os, "RTLD_LOCAL", 0)
    handle = ctypes.CDLL(NODE, mode=flags)
    if not handle._handle:
        raise Blocked("dlopen did not return a library handle")
    evidence = mapped_libraries(Path("/proc/self/maps").read_text())
    print("M23AZ_RESULT_JSON=" + json.dumps({
        **evidence,
        "verdict": "OFFLINE_DIAGNOSTIC_COMPONENT_DLOPEN_PASS",
        "component_node_instantiated": False,
        "ros_graph_created": False,
        "physical_camera_opened": False,
        "mcu_serial_access": False,
        "image_recording": False,
        "libdl_unload_separately_validated": False,
        "live_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AZ_VERDICT=OFFLINE_DIAGNOSTIC_COMPONENT_DLOPEN_PASS", flush=True)
    print("M23AZ_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", action="store_true")
    args = p.parse_args(argv)
    if not args.run:
        print("M23AZ_STATUS=INERT_NO_ROS_NO_DEVICES", flush=True)
        return 0
    try:
        return run()
    except (Blocked, OSError, RuntimeError, ValueError, TypeError) as exc:
        print("M23AZ_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:170],
              flush=True)
        print("M23AZ_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
