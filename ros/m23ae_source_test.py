"""M23AE metadata-only offline tests with synthetic ELF64 cores."""
from pathlib import Path
import struct
import tempfile
import unittest

import m23ae_saved_registers as m23ae

ROOT = Path(__file__).resolve().parent


def synthetic_core(pc, rsp=0x710000000000, tid=89,
                   dup_thread=False, bad_machine=False):
    hdr = bytearray(64)
    hdr[:6] = b"\x7fELF\x02\x01"
    struct.pack_into("<H", hdr, 16, 4)
    struct.pack_into("<H", hdr, 18, 40 if bad_machine else 62)
    struct.pack_into("<Q", hdr, 32, 64)
    struct.pack_into("<HH", hdr, 54, 56, 1)
    desc = bytearray(336)
    struct.pack_into("<i", desc, 32, tid)
    struct.pack_into("<Q", desc, 112 + 16 * 8, pc)
    struct.pack_into("<Q", desc, 112 + 19 * 8, rsp)
    struct.pack_into("<Q", desc, 112 + 4 * 8, rsp + 8)

    def note(d):
        return struct.pack("<III", 5, len(d), 1) + b"CORE\0\0\0\0" + d

    data = note(desc)
    if dup_thread:
        data += note(desc)
    phdr = struct.pack("<IIQQQQQQ", 4, 4, 64 + 56, 0, 0,
                       len(data), len(data), 4)
    return bytes(hdr) + phdr + data


class SavedRegistersTests(unittest.TestCase):
    def with_core(self, raw, fn):
        with tempfile.TemporaryDirectory() as d:
            file = Path(d) / "synthetic.core"
            file.write_bytes(raw)
            return fn(file)

    def test_extract_saved_rip_rsp_tid(self):
        out = self.with_core(synthetic_core(0x7F56A4912F5B),
                             m23ae.prstatus_threads)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0]["tid"], 89)
        self.assertEqual(out[0]["rip"], 0x7F56A4912F5B)
        self.assertEqual(out[0]["rsp"], 0x710000000000)

    def test_refuse_wrong_architecture(self):
        with self.assertRaises(m23ae.InvalidCore):
            self.with_core(synthetic_core(1, bad_machine=True),
                           m23ae.prstatus_threads)

    def test_refuse_duplicate_tid(self):
        with self.assertRaises(m23ae.InvalidCore):
            self.with_core(synthetic_core(1, dup_thread=True),
                           m23ae.prstatus_threads)

    def test_refuse_truncated_note(self):
        with self.assertRaises(m23ae.InvalidCore):
            self.with_core(synthetic_core(1)[:-1],
                           m23ae.prstatus_threads)

    def test_signal_and_register_agreement(self):
        pc = 0x7F56A4912F5B
        saved = m23ae.inspect_elf
        m23ae.inspect_elf = lambda path: (
            1, [], [(0x710000000000, 0x710000001000, 6, 0)],
            [(11, 0, 1, pc)])
        try:
            out = self.with_core(synthetic_core(pc),
                                 lambda f: m23ae.summarize(f, pc))
            self.assertEqual(out["matching_fault_thread_count"], 1)
            self.assertEqual(out["rip_page_offset"], "0xf5b")
            self.assertTrue(out["rsp_present_in_pt_load"])
            self.assertFalse(out["root_cause_proven"])
        finally:
            m23ae.inspect_elf = saved

    def test_disagreement_fails_closed(self):
        pc = 0x7F56A4912F5B
        saved = m23ae.inspect_elf
        m23ae.inspect_elf = lambda path: (1, [], [], [(11, 0, 1, pc + 8)])
        try:
            with self.assertRaises(m23ae.InvalidCore):
                self.with_core(synthetic_core(pc),
                               lambda f: m23ae.summarize(f, pc))
        finally:
            m23ae.inspect_elf = saved

    def test_default_no_core_access(self):
        source = (ROOT / "m23ae_saved_registers.py").read_text()
        self.assertIn("if not args.verify_existing:", source)
        self.assertNotIn("subprocess.run(", source)
        self.assertNotIn("coredumpctl", source)
        self.assertNotIn("podman run", source)

    def test_old_core_pc_signature(self):
        expected = [entry[2] for entry in m23ae.CASES]
        self.assertEqual(len(expected), 2)
        self.assertEqual(expected[0] & 4095, expected[1] & 4095)
        self.assertEqual(expected[0] & 4095, 0xf5b)


if __name__ == "__main__":
    unittest.main()
