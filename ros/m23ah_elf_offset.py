#!/usr/bin/env python3
"""M23AH: offline ELF64 translation of M23AG same-run TBB file offset.

No container execution, ELF code execution, GDB, core reads, ROS or devices.
Use only the private libtbb.so.12.11 copied from exact immutable M23AF image.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import stat
import struct
import subprocess
import sys

IMAGE_ID = "a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777"
LIBRARY = "/usr/lib/x86_64-linux-gnu/libtbb.so.12.11"
OFFSET = 110427  # 0x1af5b, M23AG verified SAME-RUN mapped file offset.
BASE = Path.home() / ".local/state/leroy-m23ah"
STAGED = BASE / "libtbb.so.12.11"
HASH_FILE = BASE / "image-library-sha256-01.txt"
PT_LOAD = 1
PT_NOTE = 4
PF_X = 1
MAX_PHDRS = 4096
MAX_NOTE_SIZE = 1024 * 1024


class Stop(ValueError):
    pass


def program_headers(path):
    size = path.stat().st_size
    with path.open("rb") as stream:
        hdr = stream.read(64)
        if len(hdr) != 64 or hdr[:6] != b"\x7fELF\x02\x01":
            raise Stop("expected ELF64 little-endian image")
        if (struct.unpack_from("<H", hdr, 16)[0] != 3 or  # ET_DYN
                struct.unpack_from("<H", hdr, 18)[0] != 62):  # x86_64
            raise Stop("expected x86_64 ET_DYN shared library")
        phoff = struct.unpack_from("<Q", hdr, 32)[0]
        phsize, num = struct.unpack_from("<HH", hdr, 54)
        if phsize < 56 or not 0 < num <= MAX_PHDRS:
            raise Stop("invalid ELF program header table")
        if phoff > size or num * phsize > size - phoff:
            raise Stop("program header table outside ELF")
        headers = []
        for index in range(num):
            stream.seek(phoff + index * phsize)
            raw = stream.read(56)
            if len(raw) != 56:
                raise Stop("truncated program header")
            typ, flags, offset, vaddr, _paddr, filesz, memsz, _align = (
                struct.unpack("<IIQQQQQQ", raw))
            if filesz and (offset > size or filesz > size - offset):
                raise Stop("program segment outside ELF file")
            if memsz < filesz:
                raise Stop("program segment memory size less than file size")
            headers.append({
                "type": typ, "flags": flags,
                "file_offset": offset, "vaddr": vaddr,
                "file_size": filesz, "memory_size": memsz,
            })
        return headers


def translate_offset(headers, file_offset=OFFSET):
    matches = [
        p for p in headers
        if p["type"] == PT_LOAD and p["flags"] & PF_X
        and p["file_offset"] <= file_offset
        and file_offset - p["file_offset"] < p["file_size"]
    ]
    if len(matches) != 1:
        raise Stop("expected unique executable PT_LOAD containing target offset")
    entry = matches[0]
    return {
        "target_file_offset": file_offset,
        "elf_virtual_address":
            entry["vaddr"] + file_offset - entry["file_offset"],
        "segment_file_offset": entry["file_offset"],
        "segment_virtual_address": entry["vaddr"],
        "segment_file_size": entry["file_size"],
        "segment_permissions": entry["flags"],
    }


def gnu_build_id(path, headers):
    """Read GNU NT_GNU_BUILD_ID only; do not read segment contents."""
    result = []
    total = path.stat().st_size
    with path.open("rb") as stream:
        for entry in headers:
            if entry["type"] != PT_NOTE:
                continue
            length = entry["file_size"]
            offset = entry["file_offset"]
            if length > MAX_NOTE_SIZE or offset > total or length > total - offset:
                raise Stop("oversized or invalid PT_NOTE")
            stream.seek(offset)
            data = stream.read(length)
            pos = 0
            while pos < len(data):
                if len(data) - pos < 12:
                    raise Stop("partial ELF note")
                namesz, descsz, kind = struct.unpack_from("<III", data, pos)
                pos += 12
                npad, dpad = (namesz + 3) & ~3, (descsz + 3) & ~3
                if npad > len(data) - pos or dpad > len(data) - pos - npad:
                    raise Stop("invalid ELF note size")
                name = data[pos:pos + namesz].rstrip(b"\0")
                desc = data[pos + npad:pos + npad + descsz]
                pos += npad + dpad
                if name == b"GNU" and kind == 3:
                    if not 8 <= len(desc) <= 64:
                        raise Stop("invalid GNU build ID")
                    result.append(desc.hex())
    if len(set(result)) != 1 or not result:
        raise Stop("unique GNU build ID not found")
    return result[0]


def validate_staged():
    if BASE.is_symlink() or not BASE.is_dir():
        raise Stop("private M23AH extraction directory missing or symlinked")
    if stat.S_IMODE(BASE.stat().st_mode) != 0o700:
        raise Stop("M23AH evidence directory must be owner-only 0700")
    for path in (STAGED, HASH_FILE):
        if path.is_symlink() or not path.is_file():
            raise Stop("saved ELF/hash record missing or symlinked")
        if stat.S_IMODE(path.stat().st_mode) & 0o077:
            raise Stop("ELF/hash record permissions not owner-only")
    recorded = HASH_FILE.read_text().strip()
    if re.fullmatch(r"[0-9a-f]{64}", recorded) is None:
        raise Stop("invalid saved sha256 record")
    h = hashlib.sha256()
    with STAGED.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    if h.hexdigest() != recorded:
        raise Stop("staged image ELF sha256 differs from saved record")
    if not (4096 <= STAGED.stat().st_size <= 64 * 1024 * 1024):
        raise Stop("staged library size unexpected")
    return recorded


def inspect():
    checksum = validate_staged()
    headers = program_headers(STAGED)
    address = translate_offset(headers)
    build_id = gnu_build_id(STAGED, headers)
    result = subprocess.run([
        "addr2line", "-e", str(STAGED), "-f", "-C",
        f"0x{address['elf_virtual_address']:x}"
    ], capture_output=True, text=True, check=False, timeout=8)
    if result.returncode:
        raise Stop("offline addr2line failed")
    rows = result.stdout.strip().splitlines()
    if len(rows) != 2:
        raise Stop("unexpected addr2line output")
    function, source_location = rows
    # Stripped objects may produce ??; do not substitute nearest symbol.
    symbolized = function not in ("??", "") and source_location not in ("??:0", "??:?", "")
    output = {
        "source_image_id": IMAGE_ID,
        "prior_mapped_path": LIBRARY,
        "file_sha256": checksum,
        "gnu_build_id": build_id,
        "former_rip_file_offset": f"0x{OFFSET:x}",
        "elf_virtual_address": f"0x{address['elf_virtual_address']:x}",
        "executable_segment_file_offset":
            f"0x{address['segment_file_offset']:x}",
        "executable_segment_virtual_address":
            f"0x{address['segment_virtual_address']:x}",
        "addr2line_function": function if symbolized else "UNKNOWN",
        "addr2line_source": source_location if symbolized else "UNKNOWN",
        "debug_symbolization_verified": symbolized,
        "mapping_proves_unload_cause": False,
        "root_cause_proven": False,
        "production_acceptance": "BLOCKED",
    }
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspect", action="store_true")
    args = parser.parse_args()
    print("M23AH_ROS_EXECUTION=NONE", flush=True)
    print("M23AH_DEVICE_ACCESS=NONE", flush=True)
    if not args.inspect:
        print("M23AH_STATUS=NO_LIBRARY_READ_BY_DEFAULT", flush=True)
        return 0
    try:
        out = inspect()
    except (Stop, OSError, ValueError, struct.error,
            subprocess.TimeoutExpired) as exc:
        print("M23AH_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:160],
              flush=True)
        print("M23AH_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2
    print("M23AH_ELF_RESULT_JSON=" + json.dumps(out, sort_keys=True), flush=True)
    print("M23AH_ELF_INSPECTION=PASS_METADATA_ONLY", flush=True)
    print("M23AH_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
