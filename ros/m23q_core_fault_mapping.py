#!/usr/bin/env python3
"""M23Q: read-only ELF64 core NT_FILE / PT_LOAD fault mapping inspection.

This script opens only the user-selected existing ELF core. It does not
execute its contents, attach, start ROS, access devices, or dump memory.
Only mappings containing the two previously documented suspect PCs print.
"""
import argparse
from pathlib import Path
import stat
import struct
import sys

PC = 0x00007F56A4912F5B
NEARBY_PC = 0x00007F56A491AF75
PT_LOAD = 1
PT_NOTE = 4
NT_FILE = 0x46494C45
NT_SIGINFO = 0x53494749  # Linux ELF note type SIGI
MAX_NOTE_BYTES = 64 * 1024 * 1024
MAX_MAPS = 100000
MAX_PHDRS = 8192


class InvalidCore(ValueError):
    pass


def read_at(stream, offset, size, total):
    if offset < 0 or size < 0 or offset > total or size > total - offset:
        raise InvalidCore("out-of-bounds ELF core structure")
    stream.seek(offset)
    data = stream.read(size)
    if len(data) != size:
        raise InvalidCore("truncated ELF core structure")
    return data


def inspect_elf(path):
    """Return selected filename mappings and all PT_LOAD ranges, not memory."""
    path = Path(path)
    total = path.stat().st_size
    file_maps, load_maps, siginfo_notes = [], [], []
    with path.open("rb") as stream:
        hdr = read_at(stream, 0, 64, total)
        if (hdr[:4] != b"\x7fELF" or hdr[4] != 2 or hdr[5] != 1
                or struct.unpack_from("<H", hdr, 16)[0] != 4):
            raise InvalidCore("not ELF64 little-endian ET_CORE")
        phoff = struct.unpack_from("<Q", hdr, 32)[0]
        phentsize, phnum = struct.unpack_from("<HH", hdr, 54)
        if phentsize < 56 or not (0 < phnum <= MAX_PHDRS):
            raise InvalidCore("invalid program header table")
        notes_found = 0
        for idx in range(phnum):
            raw = read_at(stream, phoff + idx * phentsize, 56, total)
            typ, flags, poff, vaddr, _, filesz, memsz, _ = struct.unpack(
                "<IIQQQQQQ", raw)
            if typ == PT_LOAD:
                if memsz > 0:
                    load_maps.append((vaddr, vaddr + memsz, flags, filesz))
                continue
            if typ != PT_NOTE:
                continue
            if filesz > MAX_NOTE_BYTES or poff + filesz > total:
                raise InvalidCore("PT_NOTE exceeds safe file bounds")
            pos, stop = poff, poff + filesz
            while pos < stop:
                if stop - pos < 12:
                    raise InvalidCore("partial note header")
                namesz, descsz, note_type = struct.unpack(
                    "<III", read_at(stream, pos, 12, total))
                pos += 12
                name_pad = (namesz + 3) & ~3
                desc_pad = (descsz + 3) & ~3
                if name_pad > stop - pos or desc_pad > stop - pos - name_pad:
                    raise InvalidCore("note extends beyond PT_NOTE")
                name = (read_at(stream, pos, namesz, total)
                        if namesz <= 64 else b"")
                descpos = pos + name_pad
                pos = descpos + desc_pad
                if name.rstrip(b"\0") != b"CORE":
                    continue
                if note_type == NT_SIGINFO:
                    # Linux x86_64 siginfo_t: signo, errno, code are
                    # int32 at bytes 0,4,8; _sifields starts at 16.
                    # For SIGSEGV, si_addr is an unsigned 64-bit pointer
                    # at byte 16. It is NOT necessarily the instruction PC.
                    if descsz < 24 or descsz > 4096:
                        raise InvalidCore("malformed NT_SIGINFO descriptor")
                    raw_siginfo = read_at(stream, descpos, 24, total)
                    signo, errno, code = struct.unpack_from(
                        "<iii", raw_siginfo)
                    address = struct.unpack_from(
                        "<Q", raw_siginfo, 16)[0]
                    siginfo_notes.append((signo, errno, code, address))
                    if len(siginfo_notes) > 1:
                        raise InvalidCore("duplicate NT_SIGINFO")
                    continue
                if note_type != NT_FILE:
                    continue
                notes_found += 1
                if descsz < 16:
                    raise InvalidCore("short NT_FILE descriptor")
                count, page_size = struct.unpack(
                    "<QQ", read_at(stream, descpos, 16, total))
                if not (0 < page_size <= 1024 * 1024
                        and 0 < count <= MAX_MAPS
                        and 16 + 24 * count <= descsz):
                    raise InvalidCore("invalid NT_FILE header")
                triples = read_at(stream, descpos + 16, 24 * count, total)
                relevant = []
                for i in range(count):
                    start, end, pages = struct.unpack_from(
                        "<QQQ", triples, i * 24)
                    if start >= end:
                        raise InvalidCore("invalid NT_FILE range")
                    if any(start <= pc < end for pc in (PC, NEARBY_PC)):
                        relevant.append((i, start, end, pages * page_size))
                filenames_pos = descpos + 16 + 24 * count
                names_size = descsz - 16 - 24 * count
                if names_size > MAX_NOTE_BYTES:
                    raise InvalidCore("NT_FILE names exceed bound")
                names = read_at(stream, filenames_pos, names_size, total).split(
                    b"\0")
                if len(names) < count:
                    raise InvalidCore("NT_FILE filename list truncated")
                for i, start, end, offset in relevant:
                    filename = names[i].decode("utf-8", "backslashreplace")
                    file_maps.append((start, end, offset, filename))
    return notes_found, file_maps, load_maps, siginfo_notes


def render(path):
    notes, mapped, loads, signal_notes = inspect_elf(path)
    lines = [f"M23Q_NT_FILE_NOTES={notes}",
             "M23Q_MAPPING_SOURCE=ELF_CORE_NT_FILE_AND_PT_LOAD",
             f"M23Q_NT_SIGINFO_NOTES={len(signal_notes)}"]
    if signal_notes:
        signo, errno, code, fault_address = signal_notes[0]
        lines.extend((
            f"M23Q_SIGINFO_SIGNAL={signo}",
            f"M23Q_SIGINFO_CODE={code}",
            f"M23Q_SIGINFO_ERRNO={errno}",
            f"M23Q_SIGINFO_FAULT_ADDRESS=0x{fault_address:016x}",
            f"M23Q_SIGINFO_EQUALS_FAULT_PC={'TRUE' if fault_address == PC else 'FALSE'}",
        ))
    else:
        lines.extend((
            "M23Q_SIGINFO_FAULT_ADDRESS=UNAVAILABLE",
            "M23Q_SIGINFO_EQUALS_FAULT_PC=UNKNOWN",
        ))
    for name, pc in (("FAULT", PC), ("NEARBY", NEARBY_PC)):
        matched = [(s, e, o, p) for s, e, o, p in mapped if s <= pc < e]
        segments = [(s, e, flags, size) for s, e, flags, size in loads
                    if s <= pc < e]
        lines.append(f"M23Q_{name}_PC=0x{pc:016x}")
        lines.append(f"M23Q_{name}_FILE_MATCHES={len(matched)}")
        for start, end, off, filename in matched:
            lines.append(f"M23Q_{name}_FILE_MAP=0x{start:016x}-0x{end:016x} "
                         f"file_offset=0x{off:x} "
                         f"pc_file_offset=0x{off + pc - start:x} "
                         f"path={filename!r}")
        lines.append(f"M23Q_{name}_LOAD_MATCHES={len(segments)}")
        for start, end, flags, size in segments:
            perms = "".join(letter if flags & bit else "-" for bit, letter in
                            ((4, "r"), (2, "w"), (1, "x")))
            lines.append(f"M23Q_{name}_CORE_SEGMENT="
                         f"0x{start:016x}-0x{end:016x} "
                         f"permissions={perms} core_bytes={size}")
    if signal_notes:
        signo, _, code, fault_address = signal_notes[0]
        if signo == 11 and code == 1 and fault_address == PC:
            lines.append("M23Q_UNMAPPED_EXECUTION_HYPOTHESIS=" +
                         ("SUPPORTED" if not any(
                             s <= PC < e for s, e, *_ in mapped + loads)
                          else "NOT_ESTABLISHED"))
        else:
            lines.append("M23Q_UNMAPPED_EXECUTION_HYPOTHESIS=NOT_ESTABLISHED")
    else:
        lines.append("M23Q_UNMAPPED_EXECUTION_HYPOTHESIS=UNKNOWN")
    lines.append("M23Q_MODULE_BUILD_ID_VERIFIED=FALSE")
    lines.append("M23Q_FAULTING_FUNCTION_IDENTIFIED=FALSE")
    return "\n".join(lines)


def validated_historical_core_path(candidate):
    """Refuse anything except the private, saved PID 48349 regular core."""
    path = candidate.resolve(strict=True)
    base = (Path.home() / ".local/state/leroy-m23q").resolve(strict=True)
    if (path.name != "core.48349"
            or path.parent.parent != base
            or not path.parent.name.startswith("gdb-firstpass.")
            or not stat.S_ISREG(path.stat().st_mode)):
        raise InvalidCore("expected saved regular core.48349 under private M23Q directory")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("core_file", type=Path)
    args = parser.parse_args()
    try:
        print(render(validated_historical_core_path(args.core_file)))
    except (InvalidCore, OSError, struct.error) as exc:
        print("M23Q_MAPPING_BLOCKED=" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
