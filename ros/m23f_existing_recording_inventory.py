#!/usr/bin/env python3
"""M23F: bounded, read-only inventory of EXPLICIT pre-existing rosbag folders.

Inventory does not authenticate recordings. Optional CDR audit invokes the
accepted M23E/M23D/M23A read-only checks, not XRSLAM or a ROS node.
No devices, camera access, serial ports, motor code or live capture.
"""
import argparse
import json
import os
from pathlib import Path
import sys

from m23d_recorded_bag_readiness import _ensure_regular_bag_files

MAX_ROOT_ENTRIES = 256
MAX_BAGS = 32
BLOCKED_ROOTS = ("/dev", "/proc", "/sys", "/run")
SAFE_ORIGIN = "unverified"


def _under_system_root(path):
    text = os.fspath(path.resolve())
    return any(text == x or text.startswith(x + "/") for x in BLOCKED_ROOTS)


def _root_check(root):
    if not isinstance(root, Path):
        raise TypeError("root must be a Path")
    if root.is_symlink() or not root.is_dir():
        raise ValueError("explicit data root must be a real directory")
    if _under_system_root(root) or root.resolve() == Path("/"):
        raise ValueError("system/device filesystem is not a bag inventory")
    return root


def discover_bags(root):
    """Only root itself or its IMMEDIATE children, never recursive traversal."""
    _root_check(root)
    if (root / "metadata.yaml").exists() or (root / "metadata.yaml").is_symlink():
        return [(".", root)]
    records = []
    with os.scandir(root) as scan:
        for count, item in enumerate(scan, start=1):
            if count > MAX_ROOT_ENTRIES:
                raise ValueError("directory entry limit exceeded")
            if item.is_symlink() or not item.is_dir(follow_symlinks=False):
                continue
            candidate = root / item.name
            if ((candidate / "metadata.yaml").exists()
                    or (candidate / "metadata.yaml").is_symlink()):
                records.append((item.name, candidate))
                if len(records) > MAX_BAGS:
                    raise ValueError("rosbag candidate limit exceeded")
    return sorted(records, key=lambda pair: pair[0])


def _audit_one(path, *, full_cdr):
    _ensure_regular_bag_files(path)
    if not full_cdr:
        return {
            "candidate_verdict": "REFERENCED_STORAGE_FILES_PRESENT_ONLY",
            "counts": None,
            "faults": [],
            "sha256_manifest": None,
        }
    # Optional import is intentional: a pure-Python host can inventory
    # candidates without ROS 2. A CDR audit requires the isolated Jazzy image.
    from m23e_offline_evidence_snapshot import inspect_existing_bag
    audit = inspect_existing_bag(path, origin_label=SAFE_ORIGIN)
    profile = audit["m23d_profile"]
    m23a = profile.get("m23a_preflight", {})
    report = {
        "candidate_verdict": audit["verdict"],
        "counts": audit["stream_evidence"]["counts"],
        "faults": sorted(set(
            audit["stream_evidence"]["faults"]
            + profile["faults"]
            + m23a.get("faults", []))),
        "m23a_verdict": m23a.get("verdict"),
        "m23a_faults": list(m23a.get("faults", [])),
        "m23a_matched_image_info_pairs":
            m23a.get("matched_image_info_pairs"),
        "m23a_camera_info_cache_evictions":
            m23a.get("camera_info_cache_evictions"),
        "gaps_over_250ms": profile.get("gaps_over_250ms"),
        "sha256_manifest": audit["bag_files"],
        "header_overlap_s": profile["header_overlap_s"],
        "max_header_gap_ms": profile["max_header_gap_ms"],
        "sampled_image_variation_observed": profile[
            "sampled_image_variation_observed"],
        "camera_info_projection_changes": audit["stream_evidence"][
            "camera_info_projection_changes"],
        "bag_files_stable_during_audit": audit[
            "bag_files_stable_during_audit"],
    }
    return report


def inventory(root, *, full_cdr=False):
    """Return a report. NO candidates is a BLOCKER, never an empty PASS."""
    bags = discover_bags(root)
    candidates = []
    for label, folder in bags:
        entry = {"relative_directory": label}
        try:
            entry.update(_audit_one(folder, full_cdr=full_cdr))
        except (OSError, ValueError, KeyError, AttributeError,
                TypeError, ImportError, RuntimeError) as exc:
            # Reject this candidate, but keep other candidates inspectable.
            entry.update({
                "candidate_verdict": "BLOCKED_CANDIDATE",
                "counts": None, "faults": [
                    type(exc).__name__ + ": " + str(exc)[:240]
                ],
                "sha256_manifest": None,
            })
        candidates.append(entry)
    if not candidates:
        verdict = "BLOCKED_NO_EXISTING_ROSBAG"
    elif any(x["candidate_verdict"] not in (
            "REFERENCED_STORAGE_FILES_PRESENT_ONLY",
            "OFFLINE_EVIDENCE_SNAPSHOT_ONLY") for x in candidates):
        verdict = "BLOCKED_CANDIDATE_STORAGE_OR_CDR"
    else:
        verdict = ("EXISTING_CDR_INPUTS_PROFILED_UNVERIFIED" if full_cdr
                   else "EXISTING_ROSBAG_REFERENCES_INVENTORIED_ONLY")
    return {
        "milestone": "M23F",
        "verdict": verdict,
        "audit_mode": "READ_ONLY_CDR" if full_cdr else "METADATA_REFERENCES_ONLY",
        "candidate_count": len(candidates),
        "candidates": candidates,
        "recorded_sensor_origin_verified": False,
        "recorded_sensor_calibration_verified": False,
        "real_xrslam_tracking_validated": False,
        "safe_for_metric_vio": False,
        "physical_motion_authorized": False,
        "device_access": False,
        "root_scanned_recursively": False,
        "other_host_filesystems_searched": False,
        "next_gate": ("Supply an already existing full Image/CameraInfo/Imu "
                      "rosbag and independently establish provenance, "
                      "camera-IMU acquisition timing and extrinsics"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True,
                        help="Explicit existing offline bag directory/parent")
    parser.add_argument("--audit-cdr", action="store_true",
                        help="Run merged M23E/M23D/M23A, requires Jazzy")
    args = parser.parse_args(argv)
    try:
        result = inventory(args.root, full_cdr=args.audit_cdr)
    except (OSError, ValueError, TypeError) as exc:
        print("M23F_PRECONDITION=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr, flush=True)
        return 2
    print("M23F_INVENTORY_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23F_VERDICT=" + result["verdict"], flush=True)
    print("M23F_REAL_RECORDING_VERIFIED=FALSE", flush=True)
    print("M23F_CALIBRATION_VALIDATED=FALSE", flush=True)
    print("M23F_DEVICE_ACCESS=NONE", flush=True)
    if result["verdict"] == "BLOCKED_NO_EXISTING_ROSBAG":
        return 3
    if result["verdict"].startswith("BLOCKED_"):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
