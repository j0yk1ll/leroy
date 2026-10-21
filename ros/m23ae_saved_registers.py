#!/usr/bin/env python3
"""M23AE: read saved x86_64 NT_PRSTATUS from two historical M23Q cores.

Metadata-only. No core-memory or stack-byte reads; no GDB, Podman or ROS.
The RIP of a thread is independently compared with kernel NT_SIGINFO si_addr
and the mapped ranges reported by the existing M23Q ELF-note parser.
"""
import argparse
import json
from pathlib import Path
import struct
import sys

from m23q_core_fault_mapping import inspect_elf, read_at, InvalidCore

PT_NOTE = 4
NT_PRSTATUS = 1
X86_64 = 62
PRSTATUS_SIZE = 336
REGISTER_OFFSET = 112
REG_RIP = 16
REG_RSP = 19
REG_RBP = 4
MAX_PHDRS = 8192
MAX_NOTE_BYTES = 64 * 1024 * 1024

CASES = (
    ("prior", "prior-core-46312/core.46312", 0x00007FA7214BDF5B),
    ("latest", "gdb-firstpass.*/core.48349", 0x00007F56A4912F5B),
)


def prstatus_threads(core):
    """Only read NT_PRSTATUS metadata (no register-target memory reads)."""
    core = Path(core)
    total = core.stat().st_size
    records = []
    with core.open("rb") as stream:
        header = read_at(stream, 0, 64, total)
        if (header[:4] != b"\x7fELF" or header[4:6] != b"\x02\x01"
                or struct.unpack_from("<H", header, 16)[0] != 4
                or struct.unpack_from("<H", header, 18)[0] != X86_64):
            raise InvalidCore("only x86_64 little-endian ELF64 ET_CORE accepted")
        phoff = struct.unpack_from("<Q", header, 32)[0]
        entsize, count = struct.unpack_from("<HH", header, 54)
        if entsize < 56 or not (0 < count <= MAX_PHDRS):
            raise InvalidCore("invalid ELF program header count")
        for i in range(count):
            typ, _, offset, _, _, filesz, _, _ = struct.unpack(
                "<IIQQQQQQ", read_at(stream, phoff + i * entsize, 56, total))
            if typ != PT_NOTE:
                continue
            if filesz > MAX_NOTE_BYTES or offset > total or filesz > total - offset:
                raise InvalidCore("oversized PT_NOTE or invalid bounds")
            pos, end = offset, offset + filesz
            while pos < end:
                if end - pos < 12:
                    raise InvalidCore("partial note header")
                namesz, descsz, kind = struct.unpack(
                    "<III", read_at(stream, pos, 12, total))
                pos += 12
                nbytes, dbytes = (namesz + 3) & ~3, (descsz + 3) & ~3
                if nbytes > end - pos or dbytes > end - pos - nbytes:
                    raise InvalidCore("note extends beyond PT_NOTE")
                name = (read_at(stream, pos, namesz, total)
                        if namesz <= 64 else b"")
                desc_offset = pos + nbytes
                pos += nbytes + dbytes
                if name.rstrip(b"\0") != b"CORE" or kind != NT_PRSTATUS:
                    continue
                if not (PRSTATUS_SIZE <= descsz <= 4096):
                    raise InvalidCore("malformed x86_64 NT_PRSTATUS length")
                buf = read_at(stream, desc_offset, PRSTATUS_SIZE, total)
                tid = struct.unpack_from("<i", buf, 32)[0]
                registers = struct.unpack_from("<27Q", buf, REGISTER_OFFSET)
                if tid <= 0 or any(x["tid"] == tid for x in records):
                    raise InvalidCore("invalid or duplicated PRSTATUS thread id")
                records.append({"tid": tid, "rip": registers[REG_RIP],
                                "rsp": registers[REG_RSP],
                                "rbp": registers[REG_RBP]})
    if not records or len(records) > 4096:
        raise InvalidCore("NT_PRSTATUS threads unavailable or excessive")
    return records


def summarize(core, expected_pc):
    """Fail closed unless a unique crashing CPU context agrees with the core."""
    notes, file_maps, loads, signals = inspect_elf(core)
    threads = prstatus_threads(core)
    matches = [t for t in threads if t["rip"] == expected_pc]
    if len(signals) != 1 or len(matches) != 1:
        raise InvalidCore("missing/ambiguous kernel fault or crashing thread")
    signo, errno, code, addr = signals[0]
    if (signo, errno, code, addr) != (11, 0, 1, expected_pc):
        raise InvalidCore("SIGSEGV MAPERR signal address does not equal saved RIP")
    if (not notes or any(a <= expected_pc < b for a, b, *_ in file_maps)
            or any(a <= expected_pc < b for a, b, *_ in loads)):
        raise InvalidCore("fault RIP is mapped or file-mapping note missing")
    thread = matches[0]
    stack_mapped = any(a <= thread["rsp"] < b for a, b, *_ in loads)
    return {
        "prstatus_thread_count": len(threads),
        "matching_fault_thread_count": len(matches),
        "crashing_thread_tid": thread["tid"],
        "rip": f"0x{thread['rip']:016x}",
        "rip_page_offset": f"0x{expected_pc % 4096:03x}",
        "siginfo_equals_rip": True,
        "fault_rip_in_nt_file": False,
        "fault_rip_in_pt_load": False,
        "rsp_present_in_pt_load": stack_mapped,
        "prior_unmapped_dso_identity_known": False,
        "reliable_caller_recovered": False,
        "root_cause_proven": False,
        "production_acceptance": "BLOCKED",
    }


def locate_existing(saved_root, pattern):
    """No new export or unbounded discovery. Exactly one retained file."""
    matches = list(saved_root.glob(pattern))
    if len(matches) != 1:
        raise InvalidCore("expected exactly one retained historical core: " + pattern)
    candidate = matches[0]
    if candidate.is_symlink() or not candidate.is_file():
        raise InvalidCore("retained core must be a regular nonsymlink file")
    if candidate.stat().st_mode & 0o077:
        raise InvalidCore("retained core must not be group/world accessible")
    if not candidate.resolve().is_relative_to(saved_root.resolve()):
        raise InvalidCore("core path outside retained M23Q storage")
    return candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-existing", action="store_true",
                        help="read only both historically retained M23Q cores")
    args = parser.parse_args()
    print("M23AE_RUNTIME_ROS=NONE", flush=True)
    print("M23AE_CORE_MEMORY_BYTES_READ=NONE", flush=True)
    if not args.verify_existing:
        print("M23AE_STATUS=NO_CORE_ACCESS_UNLESS_EXPLICIT", flush=True)
        return 0
    base = Path.home() / ".local/state/leroy-m23q"
    if not base.is_dir() or base.is_symlink():
        print("M23AE_BLOCKED=HISTORICAL_CORE_BASE_UNAVAILABLE", flush=True)
        return 2
    outcomes = []
    try:
        for label, pattern, pc in CASES:
            core = locate_existing(base, pattern)
            result = summarize(core, pc)
            result["case"] = label
            outcomes.append(result)
    except (InvalidCore, OSError, ValueError, struct.error) as exc:
        print("M23AE_BLOCKED=" + type(exc).__name__ + ":" + str(exc), flush=True)
        return 2
    for o in outcomes:
        print("M23AE_CORE_RESULT_JSON=" + json.dumps(o, sort_keys=True), flush=True)
    print("M23AE_CROSS_CORE_RIP_AND_SIGINFO=VERIFIED_UNMAPPED_TARGET_BOTH", flush=True)
    print("M23AE_FAULTING_DSO=UNKNOWN", flush=True)
    print("M23AE_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
