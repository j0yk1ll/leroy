"""Offline artifact checks for Stage 4C capture storage."""
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from live_stage4c_capture_contract import CaptureError
from live_stage4c_storage import capture_hashes, write_capture_summary


class StorageEvidenceTest(unittest.TestCase):
    def test_hashes_complete_bag_without_following_symlinks(self):
        with TemporaryDirectory() as temp:
            path = Path(temp)
            (path / "metadata.yaml").write_bytes(b"metadata")
            (path / "data_0.db3").write_bytes(b"bag")
            result = capture_hashes(path)
            self.assertEqual(result["metadata.yaml"], hashlib.sha256(b"metadata").hexdigest())
            self.assertEqual(result["data_0.db3"], hashlib.sha256(b"bag").hexdigest())
            (path / "unwanted.txt").write_text("x")
            with self.assertRaisesRegex(CaptureError, "unexpected_output_filename"):
                capture_hashes(path)

    def test_missing_metadata_and_symlink_are_rejected(self):
        with TemporaryDirectory() as temp:
            path = Path(temp)
            (path / "data_0.db3").write_bytes(b"bag")
            with self.assertRaisesRegex(CaptureError, "incomplete_rosbag_storage"):
                capture_hashes(path)
            (path / "metadata.yaml").symlink_to("data_0.db3")
            with self.assertRaisesRegex(CaptureError, "unsafe_output_file"):
                capture_hashes(path)

    def test_exclusive_manifest_is_sorted_and_not_overwritten(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            output = write_capture_summary("stage4c-test123", {"z": 2, "a": 1}, root=root)
            self.assertEqual(json.loads(output.read_text()), {"a": 1, "z": 2})
            self.assertEqual(output.read_text(), '{"a": 1, "z": 2}\n')
            with self.assertRaisesRegex(CaptureError, "summary_already_exists"):
                write_capture_summary("stage4c-test123", {"a": 3}, root=root)
            self.assertEqual(json.loads(output.read_text())["a"], 1)


if __name__ == "__main__":
    unittest.main()
