"""Offline tests of ELF-core note parsing: no ROS, hardware or saved core."""
import struct
import tempfile
import unittest
from pathlib import Path

from m23q_core_fault_mapping import PC, NT_FILE, InvalidCore, render


def fake_core(*, corrupt_magic=False, mapped=True):
    header = bytearray(64)
    header[:6] = b"\x7fELF\x02\x01"
    if corrupt_magic:
        header[:4] = b"NOPE"
    struct.pack_into("<H", header, 16, 4)  # ET_CORE
    struct.pack_into("<Q", header, 32, 64)  # program headers
    struct.pack_into("<HH", header, 54, 56, 2)
    start = PC - 0x100 if mapped else PC + 0x3000
    end = start + 0x200
    pt_load = struct.pack("<IIQQQQQQ", 1, 5, 0, start, 0,
                          0, 0x200, 0x1000)
    desc = (struct.pack("<QQQQQ", 1, 4096, start, end, 1) +
            b"/opt/ros/jazzy/lib/librectify.so\0")
    note = (struct.pack("<III", 5, len(desc), NT_FILE) + b"CORE\0\0\0\0" +
            desc + b"\0" * ((-len(desc)) % 4))
    pt_note = struct.pack("<IIQQQQQQ", 4, 4, 64 + 112, 0, 0,
                          len(note), len(note), 4)
    return bytes(header) + pt_load + pt_note + note


class M23QCoreMappingContracts(unittest.TestCase):
    def run_fixture(self, data):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "synthetic.elfcore"
            path.write_bytes(data)
            return render(path)

    def test_file_backed_fault_pc_is_identified(self):
        out = self.run_fixture(fake_core())
        self.assertIn("M23Q_NT_FILE_NOTES=1", out)
        self.assertIn("M23Q_FAULT_FILE_MATCHES=1", out)
        self.assertIn("M23Q_FAULT_LOAD_MATCHES=1", out)
        self.assertIn("librectify.so", out)
        self.assertIn("M23Q_MODULE_BUILD_ID_VERIFIED=FALSE", out)
        self.assertIn("M23Q_FAULTING_FUNCTION_IDENTIFIED=FALSE", out)

    def test_unmapped_fault_pc_is_explicit(self):
        out = self.run_fixture(fake_core(mapped=False))
        self.assertIn("M23Q_FAULT_FILE_MATCHES=0", out)
        self.assertIn("M23Q_FAULT_LOAD_MATCHES=0", out)

    def test_bad_elf_core_is_rejected(self):
        with self.assertRaises(InvalidCore):
            self.run_fixture(fake_core(corrupt_magic=True))

    def test_truncated_elf_core_is_rejected(self):
        with self.assertRaises(InvalidCore):
            self.run_fixture(fake_core()[:-5])


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23QCoreMappingContracts))
    if result.wasSuccessful():
        print("M23Q_ELF_MAPPING_SOURCE_CONTRACT=PASS", flush=True)
        print("M23Q_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
