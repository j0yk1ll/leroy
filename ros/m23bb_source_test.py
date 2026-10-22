"""M23BB synthetic source-only tests. No Podman, ROS, camera, subprocess or V4L2."""
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23bb_host_evidence_adapter as a
import m23bb_evidence_source_contract as p


SUP = (
    'COUNTER_LOG = Path("/tmp/m23k-cpp-counter.log")\n'
    'CAMERA_LAUNCH_LOG = Path("/tmp/m23k-camera-launch.log")\n'
    '\n'
    'with CAMERA_LAUNCH_LOG.open("xb") as camera_output, \\\n'
    '        COUNTER_LOG.open("xb") as counter_output:\n'
    '    counter = Popen(start_new_session=True)\n'
    '    camera = Popen(start_new_session=True)\n'
)
OBS = (
    'std::cout << "M23K_SAMPLE " << kind << " " << stamp\n'
    '              << " " << receipt_ns() << "\\n";\n'
    'std::cout << "M23K_COUNTER_COUNTS=" << counts_[0] << std::endl;\n'
)
MOUNT = (
    '52 36 259:2 /home/user/.local/state/m23bb-private /evidence '
    'rw,nosuid,nodev,relatime - ext4 /dev/nvme0n1p2 rw\n'
)
ENV = {**a.REQUIRED}


def directory_fixture():
    temp = tempfile.TemporaryDirectory()
    root = Path(temp.name) / "evidence"
    root.mkdir(mode=0o700)
    root.chmod(0o700)
    sentinel = root / a.SENTINEL
    sentinel.write_bytes(a.SENTINEL_CONTENT)
    sentinel.chmod(0o600)
    return temp, root


class Tests(unittest.TestCase):
    def test_host_adapter_default_inert(self):
        with patch.object(a, "staged_paths", side_effect=AssertionError("opened host path")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(a.main([]), 0)
        self.assertIn("M23BB_STATUS=INERT_NO_DEVICE_NO_ROS", out.getvalue())

    def test_patch_default_inert(self):
        with patch.object(p, "transform", side_effect=AssertionError("source read")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(p.main([]), 0)
        self.assertIn("M23BB_PATCH_STATUS=INERT_NO_WRITES_NO_DEVICE", out.getvalue())

    def test_source_transform_paths_and_flush(self):
        sup, obs = p.transform(SUP, OBS)
        self.assertIn("from m23bb_host_evidence_adapter import staged_paths", sup)
        self.assertIn("CAMERA_LAUNCH_LOG, COUNTER_LOG = staged_paths()", sup)
        self.assertIn('CAMERA_LAUNCH_LOG.open("xb")', sup)
        self.assertIn('COUNTER_LOG.open("xb")', sup)
        self.assertIn('<< " " << receipt_ns() << std::endl;', obs)
        self.assertNotIn('receipt_ns() << "\\n";', obs)
        self.assertNotIn('Path("/tmp/m23k', sup)

    def test_source_double_patch_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "already instrumented"):
            sup, obs = p.transform(SUP, OBS)
            p.transform(sup, obs)

    def test_changed_supervisor_paths_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "M23K_PATHS"):
            p.transform(SUP.replace("/tmp/m23k-cpp-counter.log", "/tmp/other"), OBS)

    def test_duplicate_supervisor_paths_blocks(self):
        lines = SUP.splitlines()
        with self.assertRaisesRegex(p.Blocked, "M23K_PATHS"):
            p.transform(SUP + "\n".join(lines[:2]) + "\n", OBS)

    def test_changed_camera_exclusive_mode_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "exclusive"):
            p.transform(SUP.replace('CAMERA_LAUNCH_LOG.open("xb")',
                                    'CAMERA_LAUNCH_LOG.open("wb")'), OBS)

    def test_changed_counter_exclusive_mode_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "exclusive"):
            p.transform(SUP.replace('COUNTER_LOG.open("xb")',
                                    'COUNTER_LOG.open("ab")'), OBS)

    def test_missing_process_isolation_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "isolation"):
            p.transform(SUP.replace("start_new_session=True", "start_new_session=False", 1),
                        OBS)

    def test_missing_observer_immediate_flush_anchor_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "M23K_IMMEDIATE"):
            p.transform(SUP, OBS.replace('<< "\\n";', '<< std::endl;'))

    def test_missing_final_counter_marker_blocks(self):
        with self.assertRaisesRegex(p.Blocked, "counts marker"):
            p.transform(SUP, OBS.replace("M23K_COUNTER_COUNTS=", "UNKNOWN="))

    def test_check_requires_two_source_files(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(p.main(["--check"]), 2)
        self.assertIn("M23BB_PATCH_BLOCKED=", out.getvalue())

    def test_source_cli_check_no_writes(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            s, o = root / "supervisor.py", root / "observer.cpp"
            s.write_text(SUP)
            o.write_text(OBS)
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(p.main(["--check", "--supervisor", str(s),
                                         "--observer", str(o)]), 0)
            self.assertIn("M23BB_PATCH_VERDICT=EXACT_SOURCE_ANCHORS_MATCH_NO_WRITES",
                          out.getvalue())
            self.assertEqual(s.read_text(), SUP)
            self.assertEqual(o.read_text(), OBS)

    def test_source_symlink_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            s, o, link = root / "s", root / "o", root / "linked"
            s.write_text(SUP)
            o.write_text(OBS)
            link.symlink_to(s)
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(p.main(["--check", "--supervisor", str(link),
                                         "--observer", str(o)]), 2)
            self.assertIn("symlink", out.getvalue())

    def test_valid_mount_info(self):
        self.assertEqual(a.verify_mountinfo(MOUNT), "ext4")

    def test_mount_missing_blocked(self):
        with self.assertRaisesRegex(a.Blocked, "mount not verified|independent host mount"):
            a.verify_mountinfo(MOUNT.replace(" /evidence ", " /elsewhere "))

    def test_duplicate_mount_blocked(self):
        with self.assertRaisesRegex(a.Blocked, "mount not verified|independent host mount"):
            a.verify_mountinfo(MOUNT + MOUNT)

    def test_read_only_mount_blocked(self):
        with self.assertRaisesRegex(a.Blocked, "not read-write"):
            a.verify_mountinfo(MOUNT.replace("/evidence rw,", "/evidence ro,"))

    def test_tmpfs_mount_blocked(self):
        with self.assertRaisesRegex(a.Blocked, "ephemeral"):
            a.verify_mountinfo(MOUNT.replace(" - ext4 ", " - tmpfs "))

    def test_overlay_mount_blocked(self):
        with self.assertRaisesRegex(a.Blocked, "ephemeral"):
            a.verify_mountinfo(MOUNT.replace(" - ext4 ", " - overlay "))

    def test_valid_private_dir(self):
        tmp, root = directory_fixture()
        try:
            self.assertTrue(a.verify_private_dir(root, uid=os.geteuid()))
        finally:
            tmp.cleanup()

    def test_wrong_owner_blocked(self):
        tmp, root = directory_fixture()
        try:
            with self.assertRaisesRegex(a.Blocked, "owner"):
                a.verify_private_dir(root, uid=os.geteuid()+100)
        finally:
            tmp.cleanup()

    def test_wrong_dir_mode_blocked(self):
        tmp, root = directory_fixture()
        try:
            root.chmod(0o755)
            with self.assertRaisesRegex(a.Blocked, "0700"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_missing_sentinel_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.SENTINEL).unlink()
            with self.assertRaisesRegex(a.Blocked, "sentinel missing"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_bad_sentinel_content_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.SENTINEL).write_bytes(b"WRONG")
            with self.assertRaisesRegex(a.Blocked, "contents mismatch"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_bad_sentinel_permission_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.SENTINEL).chmod(0o644)
            with self.assertRaisesRegex(a.Blocked, "0600"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_existing_camera_evidence_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.CAMERA_FILENAME).touch()
            with self.assertRaisesRegex(a.Blocked, "already exists"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_existing_observer_evidence_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.OBSERVER_FILENAME).touch()
            with self.assertRaisesRegex(a.Blocked, "already exists"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_existing_symlink_evidence_blocked(self):
        tmp, root = directory_fixture()
        try:
            (root / a.CAMERA_FILENAME).symlink_to(root / a.SENTINEL)
            with self.assertRaisesRegex(a.Blocked, "already exists"):
                a.verify_private_dir(root, uid=os.geteuid())
        finally:
            tmp.cleanup()

    def test_valid_stage_selector_returns_two_distinct_paths(self):
        tmp, root = directory_fixture()
        try:
            camera, observer = a.staged_paths(
                env=dict(ENV), root=root, uid=os.geteuid(), mountinfo=MOUNT)
            self.assertNotEqual(camera, observer)
            self.assertEqual(camera.parent, root)
            self.assertEqual(observer.parent, root)
            self.assertFalse(camera.exists())
            self.assertFalse(observer.exists())
        finally:
            tmp.cleanup()

    def test_missing_explicit_mount_env_blocks(self):
        with self.assertRaisesRegex(a.Blocked, "M23BB_EVIDENCE_MOUNT"):
            a.staged_paths(env={"M23BB_HOST_TEXT_EVIDENCE_REVIEWED": "YES_NO_LIVE_AUTH"},
                           mountinfo=MOUNT)

    def test_no_files_created_by_source_only_probe(self):
        tmp, root = directory_fixture()
        try:
            a.staged_paths(env=dict(ENV), root=root, uid=os.geteuid(), mountinfo=MOUNT)
            self.assertEqual(sorted(p.name for p in root.iterdir()), [a.SENTINEL])
        finally:
            tmp.cleanup()


if __name__ == "__main__":
    unittest.main()
