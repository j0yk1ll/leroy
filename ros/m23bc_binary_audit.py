#!/usr/bin/env python3
"""M23BC: inert-by-default offline ELF audit for isolated synthetic observer."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

BINARY = Path("/opt/m23bc-counter-install/lib/leroy_m23k_camera_counter/"
              "m23k_camera_header_counter")
SOURCE = Path("/opt/m23bc-counter-source/src/m23k_camera_header_counter.cpp")


class Blocked(ValueError):
    pass


def validate_ldd(output):
    if not output or any(token in output.lower()
                         for token in ("not found", "undefined symbol",
                                       "symbol lookup error", "libtbb.so",
                                       "libtbbmalloc", "libtbbbind")):
        raise Blocked("missing dependency, relocation error, or TBB in observer")
    if "librclcpp" not in output or "libopencv_" in output:
        raise Blocked("unexpected observer dependency closure")
    return True


def command(args):
    r = subprocess.run(args, text=True, capture_output=True, check=False, timeout=30)
    if r.returncode != 0:
        raise Blocked("ELF command nonzero " + args[0] + ":" + str(r.returncode))
    return r.stdout


def audit():
    if os.environ.get("M23BC_OFFLINE_BINARY_ONLY") != "YES_NO_DEVICE":
        raise Blocked("explicit disconnected observer audit gate absent")
    if not BINARY.is_file() or not SOURCE.is_file():
        raise Blocked("isolated compiled observer or copied C++ source missing")
    src = SOURCE.read_text()
    if src.count('receipt_ns() << std::endl;') != 1 or (
            'receipt_ns() << "\\n";' in src):
        raise Blocked("C++ immediate per-message flush source absent")
    elf = command(["readelf", "-h", str(BINARY)])
    if "ELF64" not in elf or "Advanced Micro Devices X86-64" not in elf:
        raise Blocked("observer wrong ELF architecture")
    validate_ldd(command(["ldd", "-r", str(BINARY)]))
    strings = command(["strings", "-a", str(BINARY)])
    for marker in ("M23K_SAMPLE ", "M23K_COUNTER_COUNTS=",
                   "M23K_COUNTER_FATAL="):
        if marker not in strings:
            raise Blocked("observer binary string missing: " + marker)
    return {
        "verdict": "OFFLINE_OBSERVER_ELF_PASS",
        "compiled_observer": str(BINARY),
        "isolated_build_source": str(SOURCE),
        "reader_metadata_only": True,
        "source_per_sample_flush": True,
        "ldd_r_resolved": True,
        "tbb_absent_from_observer_linkage": True,
        "ros_process_executed": False,
        "camera_mapped": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--audit", action="store_true")
    args = p.parse_args(argv)
    if not args.audit:
        print("M23BC_ELF_STATUS=INERT_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        result = audit()
        print("M23BC_BINARY_RESULT_JSON=" + json.dumps(result, sort_keys=True), flush=True)
        print("M23BC_BINARY_AUDIT=PASS", flush=True)
        print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, subprocess.TimeoutExpired, ValueError) as exc:
        print("M23BC_BINARY_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:180],
              flush=True)
        print("M23BC_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
