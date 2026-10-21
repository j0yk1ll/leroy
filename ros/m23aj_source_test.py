"""M23AJ source provenance tests: synthetic .dsc checksums, no downloads."""
import hashlib
from pathlib import Path
import tempfile
import unittest

import m23aj_source_gate as gate

MIN_BYTES = 1025


def fake_dsc(archive_data):
    entries = "\n".join(
        " " + hashlib.sha256(data).hexdigest() + " " +
        str(len(data)) + " " + name for name, data in archive_data.items()
    )
    return ("Format: 3.0 (quilt)\n"
            "Source: opencv\n"
            "Version: " + gate.VERSION + "\n"
            "Checksums-Sha256:\n" + entries + "\n"
            "Files:\n").encode()


class M23AJTests(unittest.TestCase):
    def test_exact_known_package_pin(self):
        self.assertEqual(gate.VERSION, "4.6.0+dfsg-13.1ubuntu1")
        self.assertEqual(len(gate.DSC_SHA256), 64)
        self.assertEqual(len(gate.DEBIAN_SHA256), 64)
        self.assertEqual(set(gate.EXPECTED),
                         {gate.ORIG, gate.CONTRIB, gate.DEBIAN_TAR})

    def test_checksum_section_parser(self):
        payload = {
            gate.ORIG: b"o" * MIN_BYTES,
            gate.CONTRIB: b"c" * MIN_BYTES,
            gate.DEBIAN_TAR: b"d" * MIN_BYTES,
        }
        meta = gate.parse_dsc_sha256(fake_dsc(payload).decode())
        self.assertEqual(set(meta), set(gate.EXPECTED))
        self.assertEqual(meta[gate.ORIG][0], MIN_BYTES)

    def test_duplicate_filename_rejected(self):
        line = " " + "a"*64 + " 123 " + gate.ORIG
        text = "\nChecksums-Sha256:\n" + line + "\n" + line + "\nFiles:\n"
        with self.assertRaises(gate.Blocked):
            gate.parse_dsc_sha256(text)

    def test_missing_or_duplicate_section_rejected(self):
        with self.assertRaises(gate.Blocked):
            gate.parse_dsc_sha256("Source: opencv\n")
        with self.assertRaises(gate.Blocked):
            gate.parse_dsc_sha256("\nChecksums-Sha256:\n\nChecksums-Sha256:\n")

    def test_bad_checksum_entry_rejected(self):
        with self.assertRaises(gate.Blocked):
            gate.parse_dsc_sha256("\nChecksums-Sha256:\n badhash 4 filename\n")

    def test_missing_file_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "sources"
            folder.mkdir(mode=0o700)
            # A public fake SHA is not expected to verify; this proves
            # failure occurs before any external executable is invoked.
            with self.assertRaises(gate.Blocked):
                gate.verify_source_set(folder)

    def test_disconnected_build_recipe_invariants(self):
        docker = (Path(__file__).resolve().parent /
                  "Dockerfile.m23aj-opencv-pthreads").read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23af-shutdown-maps-v1", docker)
        self.assertIn("verify_source_set(", docker)
        self.assertIn("-DWITH_TBB=OFF", docker)
        self.assertIn("-DWITH_OPENMP=OFF", docker)
        self.assertIn("-DWITH_PTHREADS_PF=ON", docker)
        self.assertIn("-DPARALLEL_ENABLE_PLUGINS=OFF", docker)
        self.assertIn("-DBUILD_LIST=core,flann,imgproc,imgcodecs,features2d,calib3d", docker)
        self.assertIn("M23AJ_OPENCV_ELF_AUDIT=ALL_SIX_NO_TBB_NEEDED", docker)
        self.assertIn("for module in core flann imgproc imgcodecs features2d calib3d", docker)
        self.assertIn("M23AJ_PTHREADS_BUILD_CONTRACT=PASS", docker)
        self.assertNotIn("apt-get", docker)
        self.assertNotIn("LD_PRELOAD", docker)

    def test_default_no_source_read(self):
        self.assertEqual(gate.main([]), 0)


if __name__ == "__main__":
    unittest.main()
