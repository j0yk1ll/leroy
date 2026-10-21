"""M23AH synthetic ELF64 tests, host-only; no Podman, ROS or real libraries."""
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import m23ah_elf_offset as ah

S_OFFSET = 0x1A000
S_VADDR = 0x2A000
TARGET = ah.OFFSET
BUILD_ID = bytes.fromhex("3c" * 20)


def elf_fixture(*, flags=5, wrong_machine=False, duplicate_load=False,
                include_note=True):
    size = 0x20000
    data = bytearray(size)
    data[:6] = b"\x7fELF\x02\x01"
    struct.pack_into("<H", data, 16, 3)
    struct.pack_into("<H", data, 18, 3 if wrong_machine else 62)
    struct.pack_into("<Q", data, 32, 64)
    count = 1 + int(include_note) + int(duplicate_load)
    struct.pack_into("<HH", data, 54, 56, count)
    ph = 64
    struct.pack_into("<IIQQQQQQ", data, ph, 1, flags,
                     S_OFFSET, S_VADDR, 0, 0x3000, 0x3000, 0x1000)
    ph += 56
    if duplicate_load:
        struct.pack_into("<IIQQQQQQ", data, ph, 1, 5,
                         S_OFFSET, S_VADDR, 0, 0x3000, 0x3000, 0x1000)
        ph += 56
    if include_note:
        note = (struct.pack("<III", 4, len(BUILD_ID), 3)
                + b"GNU\0" + BUILD_ID)
        data[0x300:0x300 + len(note)] = note
        struct.pack_into("<IIQQQQQQ", data, ph, 4, 4,
                         0x300, 0, 0, len(note), len(note), 4)
    return bytes(data)


class M23AHSyntheticELFTests(unittest.TestCase):
    def with_elf(self, binary, fn):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "synthetic-elf.so"
            path.write_bytes(binary)
            return fn(path)

    def test_offset_constant_matches_same_run_result(self):
        self.assertEqual(ah.OFFSET, 110427)
        self.assertEqual(hex(ah.OFFSET), "0x1af5b")
        self.assertEqual(ah.IMAGE_ID,
            "a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777")

    def test_translate_file_offset_through_exec_program_header(self):
        ph = ah.program_headers
        got = self.with_elf(elf_fixture(), lambda p: ah.translate_offset(ph(p)))
        self.assertEqual(got["elf_virtual_address"],
                         S_VADDR + TARGET - S_OFFSET)
        self.assertEqual(got["target_file_offset"], TARGET)

    def test_translate_not_by_blindly_treating_file_offset_as_vma(self):
        got = self.with_elf(elf_fixture(), lambda p:
                            ah.translate_offset(ah.program_headers(p)))
        self.assertNotEqual(got["elf_virtual_address"], TARGET)

    def test_gnu_build_id_from_bounded_elf_note(self):
        got = self.with_elf(elf_fixture(), lambda p:
            ah.gnu_build_id(p, ah.program_headers(p)))
        self.assertEqual(got, BUILD_ID.hex())

    def test_no_executable_segment_fails_closed(self):
        with self.assertRaises(ah.Stop):
            self.with_elf(elf_fixture(flags=4), lambda p:
                ah.translate_offset(ah.program_headers(p)))

    def test_overlapping_executable_segments_ambiguous(self):
        with self.assertRaises(ah.Stop):
            self.with_elf(elf_fixture(duplicate_load=True), lambda p:
                ah.translate_offset(ah.program_headers(p)))

    def test_wrong_elf_machine_rejected(self):
        with self.assertRaises(ah.Stop):
            self.with_elf(elf_fixture(wrong_machine=True), ah.program_headers)

    def test_missing_build_id_fails_closed(self):
        with self.assertRaises(ah.Stop):
            self.with_elf(elf_fixture(include_note=False), lambda p:
                ah.gnu_build_id(p, ah.program_headers(p)))

    def test_file_offset_segment_boundary_exclusive(self):
        entry = {
            "type": ah.PT_LOAD, "flags": ah.PF_X,
            "file_offset": 100, "file_size": 20, "vaddr": 500,
        }
        self.assertEqual(ah.translate_offset([entry], 119)["elf_virtual_address"], 519)
        with self.assertRaises(ah.Stop):
            ah.translate_offset([entry], 120)

    def test_default_has_no_file_or_subprocess_side_effects(self):
        with patch.object(ah, "validate_staged", side_effect=AssertionError(
                "default must not read a library")), patch(
                "sys.argv", ["m23ah_elf_offset.py"]):
            self.assertEqual(ah.main(), 0)


if __name__ == "__main__":
    unittest.main()
