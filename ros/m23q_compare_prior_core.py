#!/usr/bin/env python3
"""M23Q: compare only retained historical PID 46312 ELF-core signal metadata.

Uses the existing M23Q read-only ELF parser. No GDB, process attach, ROS,
Podman, devices, stack-memory reads or binary-core output are involved.
"""
import argparse
from pathlib import Path
import stat
import sys

from m23q_core_fault_mapping import InvalidCore, inspect_elf

PRIOR_PID = 46312
PRIOR_PC = 0x00007FA7214BDF5B
PRIOR_NEARBY = 0x00007FA7214C5F75
LATEST_PC = 0x00007F56A4912F5B
LATEST_NEARBY = 0x00007F56A491AF75


def validated_prior_core(candidate):
    path = candidate.resolve(strict=True)
    root = (Path.home() / ".local/state/leroy-m23q").resolve(strict=True)
    expected_parent = root / "prior-core-46312"
    if (path.name != "core.46312"
            or path.parent != expected_parent
            or not stat.S_ISREG(path.stat().st_mode)
            or stat.S_IMODE(path.parent.stat().st_mode) != 0o700
            or stat.S_IMODE(path.stat().st_mode) & 0o077):
        raise InvalidCore("expected private retained core.46312, mode 0600")
    return path


def summarize(notes, mapped, loads, siginfo):
    """Summarize only signal and mapping counts for *one* fixed historical PC."""
    lines = [
        "M23Q_PRIOR_PID=46312",
        f"M23Q_PRIOR_PC=0x{PRIOR_PC:016x}",
        f"M23Q_PRIOR_NEARBY_UNVERIFIED=0x{PRIOR_NEARBY:016x}",
        f"M23Q_PRIOR_NT_FILE_NOTES={notes}",
        f"M23Q_PRIOR_NT_SIGINFO_NOTES={len(siginfo)}",
        "M23Q_CROSS_CORE_PAGE_OFFSET_MATCH="
            + ("TRUE" if PRIOR_PC % 4096 == LATEST_PC % 4096 else "FALSE"),
        "M23Q_CROSS_CORE_RELATIVE_FRAME_DELTA_MATCH="
            + ("TRUE" if PRIOR_NEARBY - PRIOR_PC == LATEST_NEARBY - LATEST_PC
               else "FALSE"),
        f"M23Q_CROSS_CORE_DELTA=0x{PRIOR_NEARBY - PRIOR_PC:x}",
    ]
    file_hits = sum(start <= PRIOR_PC < end
                    for start, end, *_ in mapped)
    load_hits = sum(start <= PRIOR_PC < end
                    for start, end, *_ in loads)
    lines.append(f"M23Q_PRIOR_PC_FILE_MATCHES={file_hits}")
    lines.append(f"M23Q_PRIOR_PC_LOAD_MATCHES={load_hits}")
    if len(siginfo) != 1:
        lines.extend(("M23Q_PRIOR_SIGINFO_FAULT_ADDRESS=UNAVAILABLE",
                      "M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=UNKNOWN"))
    else:
        signo, errno, code, address = siginfo[0]
        lines.extend((
            f"M23Q_PRIOR_SIGINFO_SIGNAL={signo}",
            f"M23Q_PRIOR_SIGINFO_ERRNO={errno}",
            f"M23Q_PRIOR_SIGINFO_CODE={code}",
            f"M23Q_PRIOR_SIGINFO_FAULT_ADDRESS=0x{address:016x}",
            "M23Q_PRIOR_SIGINFO_EQUALS_PC="
                + ("TRUE" if address == PRIOR_PC else "FALSE"),
        ))
        supported = (signo == 11 and code == 1
                     and address == PRIOR_PC and not file_hits and not load_hits)
        lines.append("M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS="
                     + ("SUPPORTED" if supported else "NOT_ESTABLISHED"))
    lines.extend(("M23Q_PRIOR_FAULTING_LIBRARY_IDENTIFIED=FALSE",
                  "M23Q_PRIOR_CAUSE_VERIFIED=FALSE",
                  "M23Q_DEVICE_ACTION=NONE"))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("saved_core", type=Path)
    args = ap.parse_args()
    try:
        return_text = summarize(*inspect_elf(validated_prior_core(args.saved_core)))
    except (InvalidCore, OSError, ValueError) as exc:
        print("M23Q_PRIOR_CORE_BLOCKED=" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr)
        return 2
    print(return_text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
