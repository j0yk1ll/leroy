"""M23BD: pure-stdlib synthetic postflight positive and negative controls."""
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23bd_synthetic_evidence_postflight as m


def put(path, data):
    path.write_bytes(data)
    path.chmod(0o600)


def setup(root):
    clean = root / "persistence-clean-01"
    abnormal = root / "persistence-abnormal-01"
    clean.mkdir(mode=0o700)
    abnormal.mkdir(mode=0o700)
    for case in (clean, abnormal):
        case.chmod(0o700)
        for filename, payload in m.EVIDENCE.items():
            put(case / filename, payload)
    clean_log = root / "persistence-clean-01.log"
    abnormal_log = root / "persistence-abnormal-01.log"
    put(clean_log, m.CLEAN_LOG)
    put(abnormal_log, b"")
    return clean, abnormal, clean_log, abnormal_log


def audit(root, **kwargs):
    cases = setup(root)
    arguments = {"clean_exit": 0, "abnormal_exit": 73}
    arguments.update(kwargs)
    return m.audit(*cases, **arguments)


class Tests(unittest.TestCase):
    def test_default_inert(self):
        with patch.object(m, "audit", side_effect=AssertionError("read host files")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main([]), 0)
        self.assertIn("M23BD_STATUS=INERT_READ_ONLY_NO_ROS_NO_DEVICE",
                      out.getvalue())

    def test_two_case_positive(self):
        with tempfile.TemporaryDirectory() as d:
            report = audit(Path(d))
        self.assertEqual(report["verdict"], "OFFLINE_SYNTHETIC_EVIDENCE_POSTFLIGHT_PASS")
        self.assertEqual(report["case_count"], 2)
        self.assertFalse(report["exit_codes_independently_proven_by_files"])
        self.assertEqual(report["production_acceptance"], "BLOCKED")

    def test_source_payload_matches_existing_m23bc(self):
        import m23bc_synthetic_persistence as upstream
        self.assertEqual(m.CAM, upstream.CAM)
        self.assertEqual(m.OBS, upstream.OBS)

    def test_exact_known_synthetic_sha256s(self):
        self.assertEqual(m.digest(m.CAM),
                         "2b3375ecb148ed23e3b051d97ac634f4bac3cd12c56d2777d355f632cdb9ed86")
        self.assertEqual(m.digest(m.OBS),
                         "0226d46be4834d2bd142d92b3abdef9caf20f6e567258c52f516148b403b4144")
        self.assertEqual(m.digest(b""),
                         "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855")

    def test_wrong_clean_exit(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(m.Blocked, "reported exits"):
                audit(Path(d), clean_exit=1)

    def test_wrong_abnormal_exit(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(m.Blocked, "reported exits"):
                audit(Path(d), abnormal_exit=139)

    def test_non_integer_exit(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(m.Blocked, "integer"):
                audit(Path(d), abnormal_exit="73")

    def test_missing_partial_inputs_cli(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main(["--clean-dir", "/invalid"]), 2)
        self.assertIn("M23BD_BLOCKED=", out.getvalue())

    def test_wrong_directory_name(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            with self.assertRaisesRegex(m.Blocked, "wrong evidence case"):
                m.audit(abnormal, clean, cl, al, clean_exit=0, abnormal_exit=73)

    def test_same_case_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            clean, _abnormal, cl, al = setup(Path(d))
            with self.assertRaisesRegex(m.Blocked, "distinct siblings"):
                m.audit(clean, clean, cl, al, clean_exit=0, abnormal_exit=73)

    def test_wrong_dir_mode(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            abnormal.chmod(0o755)
            with self.assertRaisesRegex(m.Blocked, "ownership/0700"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            (clean / m.OBS_NAME).unlink()
            with self.assertRaisesRegex(m.Blocked, "missing/extra"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_extra_file(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(clean / "unexpected.log", b"extra")
            with self.assertRaisesRegex(m.Blocked, "missing/extra"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_camera_file_symlink(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            (clean / m.CAM_NAME).unlink()
            (clean / m.CAM_NAME).symlink_to(abnormal / m.CAM_NAME)
            with self.assertRaisesRegex(m.Blocked, "symlink"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_camera_file_hardlink(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            (clean / m.CAM_NAME).unlink()
            os.link(abnormal / m.CAM_NAME, clean / m.CAM_NAME)
            with self.assertRaisesRegex(m.Blocked, "hardlink"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_observer_file_wrong_mode(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            (clean / m.OBS_NAME).chmod(0o644)
            with self.assertRaisesRegex(m.Blocked, "ownership/mode"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_marker_contents_changed(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(clean / m.SENTINEL_NAME, b"forged")
            with self.assertRaisesRegex(m.Blocked, "content differs"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_camera_contents_truncated(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(clean / m.CAM_NAME, m.CAM[:-1])
            with self.assertRaisesRegex(m.Blocked, "content differs"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_abnormal_counter_extra_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(abnormal / m.OBS_NAME, m.OBS + b"extra")
            with self.assertRaisesRegex(m.Blocked, "content differs"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_overlimit_evidence_file(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(clean / m.CAM_NAME, b"A" * (m.MAX_EVIDENCE_BYTES + 1))
            with self.assertRaisesRegex(m.Blocked, "size bound"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_clean_log_missing_marker(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(cl, b"")
            with self.assertRaisesRegex(m.Blocked, "process log content"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_abnormal_log_must_be_empty(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            put(al, b"unexpected printed line\n")
            with self.assertRaisesRegex(m.Blocked, "process log content"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_runlog_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            al.unlink()
            al.symlink_to(cl)
            with self.assertRaisesRegex(m.Blocked, "symlink"):
                m.audit(clean, abnormal, cl, al, clean_exit=0, abnormal_exit=73)

    def test_wrong_runlog_name_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            misplaced = Path(d) / "other.log"
            put(misplaced, m.CLEAN_LOG)
            with self.assertRaisesRegex(m.Blocked, "stdout log location"):
                m.audit(clean, abnormal, misplaced, al,
                        clean_exit=0, abnormal_exit=73)

    def test_audit_leaves_all_files_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            paths = setup(root)
            originals = {str(p): p.read_bytes() for p in root.rglob("*") if p.is_file()}
            m.audit(*paths, clean_exit=0, abnormal_exit=73)
            after = {str(p): p.read_bytes() for p in root.rglob("*") if p.is_file()}
            self.assertEqual(originals, after)

    def test_cli_emits_no_production_acceptance(self):
        with tempfile.TemporaryDirectory() as d:
            clean, abnormal, cl, al = setup(Path(d))
            args = ["--clean-dir", str(clean), "--abnormal-dir", str(abnormal),
                    "--clean-log", str(cl), "--abnormal-log", str(al),
                    "--clean-exit", "0", "--abnormal-exit", "73"]
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(args), 0)
            self.assertIn("M23BD_POSTFLIGHT_VERDICT=PASS_SYNTHETIC_ONLY",
                          out.getvalue())
            self.assertIn('"production_acceptance": "BLOCKED"', out.getvalue())
            self.assertIn("M23BD_CAMERA_RETEST_AUTHORIZED=FALSE",
                          out.getvalue())


if __name__ == "__main__":
    unittest.main()
