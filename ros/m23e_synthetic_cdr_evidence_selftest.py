#!/usr/bin/env python3
"""M23E Jazzy real CDR reader test, using disposable synthetic bag files.

The *input* is a generated M23D test bag and NOT actual recorded Leroy
sensors. Nothing reaches ROS graph, camera, MCU, serial or motors.
"""
from pathlib import Path
import tempfile

from m23d_synthetic_rosbag_profile_selftest import generate_bag
from m23e_offline_evidence_snapshot import bag_fingerprints, inspect_existing_bag


def main():
    with tempfile.TemporaryDirectory(prefix="m23e-fake-evidence-") as tmp:
        bag = generate_bag(Path(tmp) / "generated_fake_rosbag")
        result = inspect_existing_bag(bag, origin_label="synthetic")
        required = {
            "image": 16, "info": 16, "imu": 104,
        }
        if (result["verdict"] != "OFFLINE_EVIDENCE_SNAPSHOT_ONLY"
                or result["m23d_profile"]["verdict"]
                    != "REPLAY_INPUT_RECORDING_PROFILE_ONLY"
                or result["stream_evidence"]["counts"] != required
                or not result["bag_files_stable_during_audit"]
                or result["recording_provenance_verified"]
                or result["physical_motion_authorized"]
                or result["device_access"]):
            raise AssertionError("synthetic evidence manifest failed: " +
                                 str(result))
        manifest = result["bag_files"]
        if (set(manifest) != {"metadata.yaml", "generated_fake_rosbag_0.db3"}
                or any(len(f["sha256"]) != 64 for f in manifest.values())):
            raise AssertionError("evidence file fingerprints missing")
        print("M23E_SYNTHETIC_CDR_EVIDENCE_SNAPSHOT=PASS", flush=True)

        # Fingerprint change test is on this disposable TEMP fixture only.
        original = bag_fingerprints(bag)
        db = bag / "generated_fake_rosbag_0.db3"
        with db.open("ab") as stream:
            stream.write(b"FAKE-ONLY-M23E-HASH-CHANGE")
        changed = bag_fingerprints(bag)
        if changed["generated_fake_rosbag_0.db3"]["sha256"] == (
                original["generated_fake_rosbag_0.db3"]["sha256"]):
            raise AssertionError("synthetic append did not alter digest")
        print("M23E_SYNTHETIC_MANIFEST_MUTATION_DETECTED=PASS", flush=True)
        print("M23E_JAZZY_CDR_EVIDENCE_SELFTEST=PASS", flush=True)
        print("M23E_VERDICT=SYNTHETIC_EVIDENCE_HASHING_ONLY", flush=True)
        print("M23E_REAL_RECORDED_BAG_VERIFIED=FALSE", flush=True)
        print("M23E_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23E_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23E_DEVICE_ACCESS=NONE", flush=True)


if __name__ == "__main__":
    main()
