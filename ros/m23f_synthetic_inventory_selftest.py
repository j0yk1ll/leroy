#!/usr/bin/env python3
"""M23F Jazzy CDR inventory: only disposable FABRICATED rosbag examples.

The synthetic generator creates CDR files under /tmp; an optional existing
bag mount is a separate, explicitly requested read-only path.
"""
from pathlib import Path
import tempfile

from m23d_synthetic_rosbag_profile_selftest import generate_bag
from m23f_existing_recording_inventory import inventory


def check(expr, message):
    if not expr:
        raise AssertionError("M23F: " + message)


def main():
    with tempfile.TemporaryDirectory(prefix="m23f-fake-inventory-") as td:
        root = Path(td)
        none = root / "empty"
        none.mkdir()
        report = inventory(none, full_cdr=True)
        check(report["verdict"] == "BLOCKED_NO_EXISTING_ROSBAG",
              "empty directory was incorrectly accepted")
        check(report["candidate_count"] == 0, "empty directory had candidates")
        print("M23F_EMPTY_RECORDING_INVENTORY_BLOCKED=PASS", flush=True)

        data = root / "existing_offline_inputs"
        data.mkdir()
        generate_bag(data / "SYNTHETIC_NOT_REAL")
        (data / "ordinary_notes").mkdir()
        report = inventory(data, full_cdr=True)
        check(report["verdict"] == "EXISTING_CDR_INPUTS_PROFILED_UNVERIFIED",
              "known fabricated CDR failed format audit " + str(report))
        check(report["candidate_count"] == 1, "unexpected candidates")
        item = report["candidates"][0]
        check(item["counts"] == {"image": 16, "info": 16, "imu": 104},
              "synthetic bag CDR counts mismatch")
        check(item["candidate_verdict"] == "OFFLINE_EVIDENCE_SNAPSHOT_ONLY",
              "underlying M23E audit failed")
        check(bool(item["sha256_manifest"]), "no SHA256 manifest")
        for key in ("recorded_sensor_origin_verified",
                    "recorded_sensor_calibration_verified",
                    "real_xrslam_tracking_validated",
                    "safe_for_metric_vio",
                    "physical_motion_authorized", "device_access"):
            check(report[key] is False, "unverified flag was promoted: " + key)
        print("M23F_SYNTHETIC_CDR_CANDIDATE_INVENTORY=PASS", flush=True)

        unsafe = data / "UNSAFE_METADATA_NOT_REAL"
        unsafe.mkdir()
        # A symlink to metadata is never a legal bag input.
        (unsafe / "metadata.yaml").symlink_to(
            data / "SYNTHETIC_NOT_REAL" / "metadata.yaml")
        mixed = inventory(data, full_cdr=True)
        check(mixed["verdict"] == "BLOCKED_CANDIDATE_STORAGE_OR_CDR",
              "unsafe storage candidate was ignored")
        check(any(x["candidate_verdict"] == "BLOCKED_CANDIDATE"
                  for x in mixed["candidates"]),
              "unsafe candidate had no explicit fault")
        print("M23F_UNSAFE_RECORDING_CANDIDATE_BLOCKED=PASS", flush=True)
        print("M23F_JAZZY_CDR_INVENTORY_SELFTEST=PASS", flush=True)
        print("M23F_VERDICT=SYNTHETIC_INPUT_INVENTORY_ONLY", flush=True)
        print("M23F_REAL_RECORDED_DATA_FOUND=FALSE", flush=True)
        print("M23F_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23F_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23F_DEVICE_ACCESS=NONE", flush=True)


if __name__ == "__main__":
    main()
