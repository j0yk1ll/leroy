#!/usr/bin/env python3
"""M23AY: inert-by-default offline ELF and loader closure audit, no ROS/device."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path("/opt/m23ay-install")
OVERLAY = Path("/opt/m23aj-opencv/lib")
LIBS = {
    "core": ROOT / "lib/libusb_cam.so",
    "node": ROOT / "lib/libusb_cam_node.so",
    "executable": ROOT / "lib/usb_cam/usb_cam_node_exe",
}


class Blocked(ValueError):
    pass


def verify_link_output(output, *, require_diagnostic=False, require_core=False):
    """Validate *observed* ELF loader output; never claim ABI certification."""
    if not isinstance(output, str) or not output:
        raise Blocked("empty loader inventory")
    if re.search(r"not found|undefined symbol|symbol lookup error", output, re.I):
        raise Blocked("unresolved library or relocation")
    if re.search(r"libtbb(?:malloc|bind)?(?:\.so|_[a-z])", output, re.I):
        raise Blocked("oneTBB linked into diagnostic closure")
    if require_core and not re.search(
        r"(?m)^\s*libusb_cam\.so\s*=>\s*/opt/m23ay-install/lib/libusb_cam\.so(?:\s|$)",
        output,
    ):
        raise Blocked("diagnostic node did not load isolated core library")
    if require_diagnostic and "/leroy_ws/install/usb_cam/lib/libusb_cam" in output:
        raise Blocked("legacy usb_cam library appeared in loader closure")
    return True


def verify_symbols(node_strings, core_strings, executable_strings):
    for marker in ("M23AX_PUBLISH pub_index=", "M23AX_FATAL=METADATA_LOG_LIMIT",
                   " dequeue_count=", " v4l2_seq=", " ros_stamp_ns="):
        if marker not in node_strings:
            raise Blocked("missing publisher diagnostic string: " + marker)
    for marker in ("mmap_dequeued", "select_eintr", "mmap_dqbuf_eagain"):
        if marker not in core_strings:
            raise Blocked("missing acquisition diagnostic string: " + marker)
    # In ROS 2 Jazzy, rclcpp_components_register_node creates a standalone
    # wrapper that dynamically loads libusb_cam_node.so through class_loader.
    # The executable MUST embed that component's name, but it need not have
    # a static DT_NEEDED for libusb_cam.so; the component .so MUST resolve
    # its own diagnostic core through the isolated install prefix.
    for marker in ("libusb_cam_node.so", "usb_cam::UsbCamNode"):
        if marker not in executable_strings:
            raise Blocked("standalone launcher missing component loader reference: " + marker)
    return True


def run_command(argv):
    proc = subprocess.run(argv, text=True, capture_output=True, check=False,
                          timeout=30, env=os.environ.copy())
    if proc.returncode != 0:
        raise Blocked("command failed " + argv[0] + ":" +
                      str(proc.returncode) + ":" + proc.stderr[-180:])
    return proc.stdout


def audit():
    if os.environ.get("M23AY_OFFLINE_BINARY_ONLY") != "YES":
        raise Blocked("explicit offline build-only audit gate absent")
    paths = {}
    for key, target in LIBS.items():
        if not target.is_file():
            raise Blocked("isolated diagnostic ELF missing: " + key)
        paths[key] = target
    if os.environ.get("LD_LIBRARY_PATH", "").split(":")[:2] != [
        str(ROOT / "lib"), str(OVERLAY)
    ]:
        raise Blocked("diagnostic camera then pthreads OpenCV loader precedence absent")

    observed = {}
    for key, target in paths.items():
        hdr = run_command(["readelf", "-h", str(target)])
        if "ELF64" not in hdr or "Advanced Micro Devices X86-64" not in hdr:
            raise Blocked("wrong diagnostic executable architecture: " + key)
        closure = run_command(["ldd", "-r", str(target)])
        # The component DSO must resolve diagnostic libusb_cam.so; the
        # generated standalone wrapper loads that DSO with class_loader.
        verify_link_output(closure, require_diagnostic=True,
                           require_core=key == "node")
        print("M23AY_LOADER_CORE_CHECK=" + key + ":" +
              ("REQUIRED_AND_PASS" if key == "node" else "NOT_DIRECTLY_REQUIRED"),
              flush=True)
        observed[key] = {
            "loader_checked": True,
            "relocations_resolved": True,
            "no_tbb_linked_observed": True,
        }
    verify_symbols(
        run_command(["strings", "-a", str(paths["node"])]),
        run_command(["strings", "-a", str(paths["core"])]),
        run_command(["strings", "-a", str(paths["executable"])]),
    )
    return {
        "verdict": "OFFLINE_INSTRUMENTED_USB_CAM_ELF_LOADER_PASS",
        "installed_modules": sorted(paths),
        "diagnostic_core_resolved_in_overlay": True,
        "component_dso_requires_diagnostic_core": True,
        "generated_launcher_uses_component_dlopen": True,
        "generated_launcher_static_core_link_required": False,
        "pthreads_opencv_loader_precedence": True,
        "string_evidence_compiled_into_libraries": True,
        "ldd_r_closure_verified": True,
        "actual_runtime_camera_node_executed": False,
        "device_access": False,
        "camera_retest_authorized": False,
        "image_pixels_logged": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--audit", action="store_true")
    args = p.parse_args(argv)
    if not args.audit:
        print("M23AY_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT", flush=True)
        return 0
    try:
        result = audit()
        print("M23AY_RESULT_JSON=" + json.dumps(result, sort_keys=True), flush=True)
        print("M23AY_BINARY_AUDIT=PASS", flush=True)
        print("M23AY_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, subprocess.TimeoutExpired, ValueError) as e:
        print("M23AY_BLOCKED=" + type(e).__name__ + ":" + str(e)[:170],
              flush=True)
        print("M23AY_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
