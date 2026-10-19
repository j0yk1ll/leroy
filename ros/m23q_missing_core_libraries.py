#!/usr/bin/env python3
"""M23Q: strict, read-only list of missing ELF shared libs from GDB text log.

Only prints explicitly allowlisted filesystem paths for stopped-image staging.
This cannot inspect a live process or read device paths.
"""
import argparse
from pathlib import Path, PurePosixPath
import re
import sys

PREFIXES = (
    "/opt/ros/jazzy/lib/",
    "/usr/lib/x86_64-linux-gnu/",
    "/lib/x86_64-linux-gnu/",
)
WARN = re.compile(
    r"^warning: Can't open file ([^\r\n]+) "
    r"during file-backed mapping note processing\s*$"
)
BASENAME = re.compile(r"[A-Za-z0-9_.+-]+\.so(?:\.[A-Za-z0-9_.+-]+)?\Z")
MAX_LIBRARIES = 400


def parse_missing_libraries(text):
    results = set()
    for line in text.splitlines():
        match = WARN.fullmatch(line.strip())
        if not match:
            continue
        name = match.group(1)
        if (len(name) > 512 or "\0" in name
                or not name.startswith(PREFIXES)):
            continue
        path = PurePosixPath(name)
        if (not path.is_absolute()
                or ".." in path.parts
                or "." in path.parts
                or not BASENAME.fullmatch(path.name)
                or str(path) != name):
            continue
        results.add(name)
        if len(results) > MAX_LIBRARIES:
            raise ValueError("GDB library inventory exceeds hard cap")
    return sorted(results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firstpass_log", type=Path)
    args = parser.parse_args()
    target = args.firstpass_log.resolve(strict=True)
    root = (Path.home() / ".local/state/leroy-m23q").resolve(strict=True)
    if (target.name != "gdb-firstpass.log"
            or target.parent.parent != root
            or not target.parent.name.startswith("gdb-firstpass.")
            or not target.is_file()):
        print("M23Q_LIBRARY_MANIFEST_BLOCKED=LOG_PATH_NOT_VERIFIED",
              file=sys.stderr)
        return 2
    try:
        names = parse_missing_libraries(target.read_text(errors="replace"))
    except (OSError, ValueError) as exc:
        print("M23Q_LIBRARY_MANIFEST_BLOCKED=" + str(exc), file=sys.stderr)
        return 2
    if not names:
        print("M23Q_LIBRARY_MANIFEST_BLOCKED=NO_ELF_LIBRARY_WARNINGS",
              file=sys.stderr)
        return 2
    for name in names:
        print(name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
