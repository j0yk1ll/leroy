#!/usr/bin/env python3
"""M23BB: pure offline source transformer and anchor check; NO OUTPUT WRITES.

A future *separately reviewed* diagnostic image builder must copy these
transformed files to a fresh path. Never patch original installed M23K
or the M23AY/M23AZ immutable diagnostic image in place.
"""
import argparse
from pathlib import Path
import sys


class Blocked(ValueError):
    pass


def once(src, old, new, label):
    count = src.count(old)
    if count != 1:
        raise Blocked("M23BB source anchor mismatch " + label + ":" + str(count))
    return src.replace(old, new, 1)


def transform(supervisor, observer):
    if "M23BB_" in supervisor or "m23bb_" in supervisor:
        raise Blocked("supervisor already instrumented")
    if "M23BB_" in observer or "m23bb_" in observer:
        raise Blocked("observer already instrumented")
    old_paths = (
        'COUNTER_LOG = Path("/tmp/m23k-cpp-counter.log")\n'
        'CAMERA_LAUNCH_LOG = Path("/tmp/m23k-camera-launch.log")'
    )
    new_paths = (
        '# M23BB: host-bind preflight executes BEFORE running M23K subprocesses.\n'
        'from m23bb_host_evidence_adapter import staged_paths\n'
        'COUNTER_LOG, CAMERA_LAUNCH_LOG = staged_paths()'
    )
    supervisor = once(supervisor, old_paths, new_paths, "M23K_PATHS")
    # The original exclusive-create xb path is preserved, not converted
    # to an overwrite/append; separate host mounted files persist after
    # disposable guest deletion and before crash cleanup.
    if supervisor.count('with CAMERA_LAUNCH_LOG.open("xb") as camera_output,') != 1:
        raise Blocked("M23K camera exclusive-create open changed")
    if supervisor.count('COUNTER_LOG.open("xb") as counter_output:') != 1:
        raise Blocked("M23K observer exclusive-create open changed")
    if supervisor.count('start_new_session=True') != 2:
        raise Blocked("M23K launch isolation unexpectedly changed")
    if "M23K_SAMPLE" not in observer:
        raise Blocked("M23K compiled sample marker missing")
    old_flush = '              << " " << receipt_ns() << "\\n";'
    new_flush = '              << " " << receipt_ns() << std::endl;'
    observer = once(observer, old_flush, new_flush,
                    "M23K_IMMEDIATE_SUBSCRIBER_TEXT_FLUSH")
    if observer.count("M23K_COUNTER_COUNTS=") != 1:
        raise Blocked("M23K final observer counts marker changed")
    if len(supervisor) > 90000 or len(observer) > 30000:
        raise Blocked("unexpected source size")
    return supervisor, observer


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--supervisor", type=Path)
    parser.add_argument("--observer", type=Path)
    args = parser.parse_args(argv)
    if not args.check and args.supervisor is None and args.observer is None:
        print("M23BB_PATCH_STATUS=INERT_NO_WRITES_NO_DEVICE", flush=True)
        return 0
    try:
        if not args.check or args.supervisor is None or args.observer is None:
            raise Blocked("check and both source file paths required")
        if args.supervisor.is_symlink() or args.observer.is_symlink():
            raise Blocked("source symlink forbidden")
        for path in (args.supervisor, args.observer):
            if not path.is_file() or path.stat().st_size > 90000:
                raise Blocked("source file missing or above bound")
        transformed = transform(args.supervisor.read_text(),
                                args.observer.read_text())
        if not all(transformed):
            raise Blocked("empty patched source")
        print("M23BB_PATCH_VERDICT=EXACT_SOURCE_ANCHORS_MATCH_NO_WRITES", flush=True)
        print("M23BB_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23BB_PATCH_BLOCKED=" + type(exc).__name__ + ":" +
              str(exc)[:180], flush=True)
        print("M23BB_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
