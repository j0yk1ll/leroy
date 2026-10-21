#!/usr/bin/env python3
"""M23AJ: verify canonical Ubuntu Noble 4.6.0 source archive set for pthread experiment.

Default invokes no I/O. --verify-sources checks immutable .dsc SHA-256
against Ubuntu published source-package checksum and checks every
referenced tarball byte size and SHA-256. Nothing is extracted, built,
executed, installed, or uploaded. No ROS/Podman/robot commands.
"""
import argparse
import hashlib
from pathlib import Path
import re
import stat
import sys

VERSION = "4.6.0+dfsg-13.1ubuntu1"
DSC_NAME = f"opencv_{VERSION}.dsc"
DSC_SHA256 = "4194d944a274df50ebeb60d2fb511c464a392727afbe12403da13f53261d9f47"
PACKAGE_PREFIX = "opencv_4.6.0+dfsg."
DEBIAN_TAR = "opencv_4.6.0+dfsg-13.1ubuntu1.debian.tar.xz"
DEBIAN_SHA256 = "6b3fbe614353fc074eb121307ea39b0f802d69463aa86355cef1f5e9ae4dabcc"
ORIG = "opencv_4.6.0+dfsg.orig.tar.xz"
CONTRIB = "opencv_4.6.0+dfsg.orig-contrib.tar.xz"
EXPECTED = (ORIG, CONTRIB, DEBIAN_TAR)
SOURCE_DIR = Path.home() / ".local/state/leroy-m23aj/sources"


class Blocked(ValueError):
    pass


def checksum(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_owner_private(path):
    if path.is_symlink() or not path.is_file():
        raise Blocked("source file missing or symlinked: " + path.name)
    if stat.S_IMODE(path.stat().st_mode) & 0o077:
        raise Blocked("source file is not private: " + path.name)
    return path


def parse_dsc_sha256(text):
    """Return {filename: (size_bytes, sha256)} from unique Checksum section."""
    if text.count("\nChecksums-Sha256:\n") != 1:
        raise Blocked("missing or duplicated .dsc Checksums-Sha256 section")
    part = text.split("\nChecksums-Sha256:\n", 1)[1]
    result = {}
    for line in part.splitlines():
        if not line.startswith(" "):
            break
        match = re.fullmatch(r"\s+([a-fA-F0-9]{64})\s+(\d+)\s+([-\w.+]+)", line)
        if match is None:
            raise Blocked("invalid .dsc SHA256 line")
        digest, count, name = match.groups()
        if name in result:
            raise Blocked("duplicate .dsc source name")
        result[name] = int(count), digest.lower()
    return result


def verify_source_set(folder=SOURCE_DIR):
    if folder.is_symlink() or not folder.is_dir():
        raise Blocked("private source directory not found")
    if stat.S_IMODE(folder.stat().st_mode) != 0o700:
        raise Blocked("source directory must have mode 0700")
    dsc = verify_owner_private(folder / DSC_NAME)
    if checksum(dsc) != DSC_SHA256:
        raise Blocked("Ubuntu published .dsc SHA256 mismatch")
    contents = dsc.read_text(encoding="utf-8")
    if f"Version: {VERSION}" not in contents:
        raise Blocked("wrong package version in .dsc")
    if "Source: opencv" not in contents:
        raise Blocked("wrong package name in .dsc")
    entries = parse_dsc_sha256(contents)
    if set(entries) != set(EXPECTED):
        raise Blocked("unexpected source files in signed .dsc manifest")
    if entries[DEBIAN_TAR][1] != DEBIAN_SHA256:
        raise Blocked("Ubuntu distribution patch archive SHA mismatch")
    outputs = {}
    for name in EXPECTED:
        file = verify_owner_private(folder / name)
        size, digest = entries[name]
        if not 1024 < size < 256 * 1024 * 1024:
            raise Blocked("unreasonable source archive size")
        if file.stat().st_size != size or checksum(file) != digest:
            raise Blocked("archive length or SHA mismatch: " + name)
        outputs[name] = {"bytes": size, "sha256": digest}
    return outputs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify-sources", action="store_true")
    args = ap.parse_args(argv)
    print("M23AJ_ROS_EXECUTION=NONE")
    print("M23AJ_DEVICE_ACCESS=NONE")
    if not args.verify_sources:
        print("M23AJ_STATUS=NO_SOURCE_READ_BY_DEFAULT")
        return 0
    try:
        result = verify_source_set()
    except (Blocked, OSError, UnicodeError, ValueError) as exc:
        print("M23AJ_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:130])
        print("M23AJ_PRODUCTION_ACCEPTANCE=BLOCKED")
        return 2
    import json
    print("M23AJ_SOURCES_JSON=" + json.dumps(result, sort_keys=True))
    print("M23AJ_SOURCE_PROVENANCE=VERIFIED_UBUNTU_NOBLE")
    print("M23AJ_NO_BUILD_OR_RUNTIME=TRUE")
    print("M23AJ_PRODUCTION_ACCEPTANCE=BLOCKED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
