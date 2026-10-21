#!/usr/bin/env python3
"""M23AF: one bounded, same-process map capture during synthetic crash control.

Reuses M23AC + unchanged M23AA/M23Y acceptance. Full mapping metadata is
printed only to private host diagnostic log, never a raw-memory/core dump.
"""
import contextlib
import io
import json
import os
from pathlib import Path
import re
import secrets
import sys

LAUNCH = "/opt/m23af/m23af_rectifier_container.launch.py"
BINARY = ("/opt/m23af-cpp-install/lib/"
          "leroy_m23af_shutdown_maps/component_container")
PHASES = ("post_factory", "on_shutdown")
MARK = re.compile(
    r"M23AF_MAP_(BEGIN|END|ENTRY) phase=(post_factory|on_shutdown)"
    r" run_id=([0-9a-f]{32}) pid=([0-9]+)(?: (.+))?"
)


def parse_maps(log, run_id):
    """Reject partial, repeated, out-of-order, cross-PID snapshots."""
    if "M23AF_MAP_ERROR" in log or "M23AF_PRECONDITION_BLOCKED" in log:
        raise ValueError("child reported incomplete mapping snapshot")
    snapshots = {}
    phase_active = None
    shared_pid = None
    for line in log.splitlines():
        if "M23AF_MAP_" not in line or "M23AF_MAP_ENTRY" not in line and \
                "M23AF_MAP_BEGIN" not in line and "M23AF_MAP_END" not in line:
            continue
        found = MARK.search(line)
        if found is None:
            raise ValueError("invalid or partial mapping marker")
        kind, phase, nonce, pid_text, remainder = found.groups()
        pid = int(pid_text)
        if nonce != run_id or pid <= 0:
            raise ValueError("wrong run ID / process namespace PID")
        if shared_pid is None:
            shared_pid = pid
        elif shared_pid != pid:
            raise ValueError("snapshots did not come from same child process")

        if kind == "BEGIN":
            if phase_active is not None or phase in snapshots or remainder:
                raise ValueError("duplicate or overlapping mapping snapshot")
            phase_active = phase
            snapshots[phase] = []
        elif kind == "ENTRY":
            if phase_active != phase or not remainder or not remainder.startswith("text="):
                raise ValueError("unframed map entry")
            columns = remainder[5:].split(None, 5)
            if len(columns) != 6:
                raise ValueError("invalid /proc/self/maps entry layout")
            span, perms, offset, dev, inode, path = columns
            halves = span.split("-")
            if len(halves) != 2 or perms not in ("r-xp", "r-xs") or \
                    not path.startswith("/") or not inode.isdigit():
                raise ValueError("invalid executable map record")
            try:
                start, end = int(halves[0], 16), int(halves[1], 16)
                fileoff = int(offset, 16)
            except ValueError as exc:
                raise ValueError("invalid map address or file offset") from exc
            if start >= end or fileoff < 0:
                raise ValueError("invalid map address range")
            snapshots[phase].append({
                "start": start, "end": end, "file_offset": fileoff,
                "permissions": perms, "path": path,
            })
            if len(snapshots[phase]) > 2048:
                raise ValueError("map record cap exceeded")
        else:
            if phase_active != phase or not remainder or \
                    re.fullmatch(r"count=[0-9]+", remainder) is None:
                raise ValueError("missing/mismatched map end")
            count = int(remainder.partition("=")[2])
            if count != len(snapshots[phase]) or count == 0:
                raise ValueError("incomplete executable maps count")
            phase_active = None
    if phase_active is not None or set(snapshots) != set(PHASES):
        raise ValueError("both complete ordered snapshots required")
    if "librectify.so" not in {
        Path(s["path"]).name for v in snapshots.values() for s in v
    }:
        raise ValueError("rectifier module executable map not observed")
    if not all(any(Path(s["path"]).name == "librectify.so"
                   for s in snapshots[phase]) for phase in PHASES):
        raise ValueError("rectifier executable mapping absent from a phase")
    if not snapshots["post_factory"] or not snapshots["on_shutdown"]:
        raise ValueError("empty mapping snapshot")
    return {"run_id": run_id, "pid_in_container": shared_pid,
            "snapshots": snapshots}


def exactly_one_json(text, prefix):
    lines = [s[len(prefix):] for s in text.splitlines() if s.startswith(prefix)]
    if len(lines) != 1:
        raise ValueError("expected exactly one " + prefix)
    return json.loads(lines[0])


def validate_inherited_output(output, status):
    base = exactly_one_json(output, "M23AA_RESULT_JSON=")
    hook = exactly_one_json(output, "M23AC_RESULT_JSON=")
    if not (
        status == 2 and base.get("arm") == "component"
        and base.get("verdict") == "SIGSEGV_STOCK_RECTIFIER_900_OF_900"
        and base.get("child_exit") == -11 and base.get("parent_exit") == 2
        and base.get("counts") == {"raw": 900, "rect": 900, "info": 900}
        and base.get("graph_verified") is True
        and base.get("stock_rectifier") is True
        and base.get("synthetic_only") is True
        and base.get("production_acceptance") == "BLOCKED"
        and hook.get("policy") == "control"
        and hook.get("factory_hook_verified_before_frames") is True
        and hook.get("inherited_m23aa_exit") == 2
        and hook.get("diagnostic_verdict") == "INHERITED_RUN_COMPLETE"
        and hook.get("root_cause_proven") is False
    ):
        raise ValueError("inherited strict 900-frame SIGSEGV control not reproduced")
    return True


def run():
    if os.environ.get("M23AF_ARM") != "control":
        raise ValueError("M23AF_ARM must be explicit control")
    if os.environ.get("M23AC_ARM") or os.environ.get("M23AA_ARM") \
            or os.environ.get("M23AB_ARM") or os.environ.get("LD_PRELOAD"):
        raise ValueError("prior policy and preload arms must remain empty")
    for target in (LAUNCH, BINARY):
        if not Path(target).is_file():
            raise ValueError("M23AF diagnostic executable/launch missing")

    import m23ac_compare as m23ac
    import m23p_no_device_rectifier_shutdown as m23p
    from m23aa_compare import child_outcome

    nonce = secrets.token_hex(16)
    old_launch, old_bin = m23ac.LAUNCH, m23ac.BINARY
    old_arm, old_nonce = os.environ.get("M23AC_ARM"), os.environ.get("M23AF_RUN_ID")
    buffer = io.StringIO()
    try:
        m23ac.LAUNCH, m23ac.BINARY = LAUNCH, BINARY
        os.environ["M23AC_ARM"] = "control"
        os.environ["M23AF_RUN_ID"] = nonce
        print("M23AF_RUN_ID=" + nonce, flush=True)
        with contextlib.redirect_stdout(buffer):
            status = m23ac.run()
    finally:
        m23ac.LAUNCH, m23ac.BINARY = old_launch, old_bin
        if old_arm is None:
            os.environ.pop("M23AC_ARM", None)
        else:
            os.environ["M23AC_ARM"] = old_arm
        if old_nonce is None:
            os.environ.pop("M23AF_RUN_ID", None)
        else:
            os.environ["M23AF_RUN_ID"] = old_nonce
        sys.stdout.write(buffer.getvalue())
        sys.stdout.flush()

    # Inherited M23AA verifier is authoritative on full delivery/stamps,
    # graph/QoS, publisher drain and child status; inspect again independently.
    validate_inherited_output(buffer.getvalue(), status)
    log = m23p.LAUNCH_LOG.read_text(errors="replace")
    child_kind, child_code = child_outcome(log, "component")
    if (child_kind, child_code) != ("SIGSEGV", -11):
        raise ValueError("actual component child SIGSEGV absent")
    parsed = parse_maps(log, nonce)

    # The host's protected Podman log retains both complete executable-map
    # snapshots, not just aggregate counts. Future core may be correlated
    # only with the SAME run ID in its saved command line.
    for phase in PHASES:
        print("M23AF_MAP_SNAPSHOT_JSON=" + json.dumps({
            "phase": phase, "run_id": nonce,
            "pid_in_container": parsed["pid_in_container"],
            "segments": parsed["snapshots"][phase],
        }, sort_keys=True), flush=True)
    print("M23AF_RESULT_JSON=" + json.dumps({
        "run_id": nonce,
        "pid_in_container": parsed["pid_in_container"],
        "control_child_exit": child_code,
        "mapping_phases": list(PHASES),
        "executable_mappings": {
            phase: len(parsed["snapshots"][phase]) for phase in PHASES},
        "librectify_executable_map_in_both_phases": True,
        "same_process_core_correlated": False,
        "faulting_dso_identified": False,
        "root_cause_proven": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AF_VERDICT=CRASH_REPRODUCED_WITH_SAME_PROCESS_PRE_TEARDOWN_MAPS",
          flush=True)
    print("M23AF_DEVICE_ACCESS=NONE", flush=True)
    print("M23AF_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print("M23AF_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:160],
              flush=True)
        print("M23AF_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        sys.exit(2)
