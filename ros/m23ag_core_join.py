#!/usr/bin/env python3
"""M23AG: identity-gated read-only join of ONE saved M23AF core and maps.

Default: no filesystem reads, subprocess calls or artifacts.
--preflight: metadata/log inspection only; no core export.
--export-and-join: re-run preflight, export historical retained core once,
and inspect only ELF PT_NOTE / NT_PRSTATUS + NT_FILE/PT_LOAD metadata.
No ROS, Podman, GDB, ptrace, raw stack or device access.
"""
import argparse
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

from m23q_core_fault_mapping import InvalidCore, inspect_elf
from m23ae_saved_registers import prstatus_threads

HOST_PID = 32869
CONTAINER_PID = 34
RUN_ID = "a3510ea397df3225250b7f2080031307"
IMAGE_ID = "a56f89bd8d3a69264e846319f8915efcff1fcd04a2dd3bf54d8491b89a119777"
EXECUTABLE = ("/opt/m23af-cpp-install/lib/"
              "leroy_m23af_shutdown_maps/component_container")
BASE = Path.home() / ".local/state/leroy-m23af"
OUT = Path.home() / ".local/state/leroy-m23ag"
EXPORT_DIR = OUT / "retained-core-32869-v1"
PHASES = ("post_factory", "on_shutdown")


class Stop(ValueError):
    pass


def owner_private(path, directory=False):
    if path.is_symlink():
        raise Stop("symlink forbidden")
    mode = path.stat().st_mode
    if directory:
        if not stat.S_ISDIR(mode):
            raise Stop("expected directory")
    elif not stat.S_ISREG(mode):
        raise Stop("expected regular file")
    if stat.S_IMODE(mode) & 0o077:
        raise Stop("evidence permissions must exclude group/other")
    return path


def exactly_one(lines, prefix):
    matched = [x[len(prefix):] for x in lines if x.startswith(prefix)]
    if len(matched) != 1:
        raise Stop("expected exactly one " + prefix)
    return matched[0]


def numeric_id(value):
    match = re.fullmatch(r"(\d+)(?:\s+\([^()\n]*\))?", value.strip())
    return int(match.group(1)) if match else None


def parse_metadata(info):
    fields = {}
    for line in info.splitlines():
        match = re.match(
            r"^\s*(PID|UID|GID|Timestamp|Signal|Executable|Command Line|Storage):\s*(.*)$",
            line,
        )
        if match:
            fields.setdefault(match.group(1), []).append(match.group(2).strip())

    def one(key):
        values = fields.get(key, [])
        return values[0] if len(values) == 1 else ""

    cmdline = one("Command Line")
    checks = {
        "pid": bool(re.match(r"^32869(?:\s|$)", one("PID"))),
        "uid": numeric_id(one("UID")) == 1000,
        "gid": numeric_id(one("GID")) == 1000,
        "executable": one("Executable") == EXECUTABLE,
        "signal": bool(re.search(r"11\s*\(SEGV\)|SIGSEGV", one("Signal"))),
        # Same four-second journal-detail window as the operator's
        # previously accepted eight-field core-identity verifier.
        # Do not use coredumpctl-list display seconds as an exact
        # coredumpctl-info Timestamp requirement.
        "timestamp": bool(re.search(
            r"\b2026-10-21 12:45:0[6-9] CEST\b", one("Timestamp"))),
        "storage": "(present)" in one("Storage"),
        "nonce": cmdline.count("m23af_run_id:=") == 1
        and bool(re.search(r"m23af_run_id:=" + RUN_ID + r"(?![0-9a-fA-F])", cmdline)),
    }
    if not all(checks.values()):
        raise Stop("coredumpctl identity mismatch: " +
                   ",".join(k for k, v in checks.items() if not v))
    return {"host_pid": HOST_PID, "uid": 1000, "gid": 1000,
            "run_id": RUN_ID, "core_present": True,
            "identity_verified": True}


def parse_existing_log(content):
    lines = content.splitlines()
    base = json.loads(exactly_one(lines, "M23AA_RESULT_JSON="))
    af = json.loads(exactly_one(lines, "M23AF_RESULT_JSON="))
    ac = json.loads(exactly_one(lines, "M23AC_RESULT_JSON="))
    if not (
        exactly_one(lines, "M23AF_RUN_ID=") == RUN_ID
        and exactly_one(lines, "M23AF_VERDICT=")
            == "CRASH_REPRODUCED_WITH_SAME_PROCESS_PRE_TEARDOWN_MAPS"
        and exactly_one(lines, "M23AA_VERDICT=")
            == "SIGSEGV_STOCK_RECTIFIER_900_OF_900"
        and exactly_one(lines, "M23AF_DEVICE_ACCESS=") == "NONE"
        and exactly_one(lines, "M23AF_PRODUCTION_ACCEPTANCE=") == "BLOCKED"
        and base.get("child_exit") == -11
        and base.get("parent_exit") == 2
        and base.get("counts") == {"raw": 900, "rect": 900, "info": 900}
        and base.get("graph_verified") is True
        and base.get("verdict") == "SIGSEGV_STOCK_RECTIFIER_900_OF_900"
        and base.get("synthetic_only") is True
        and ac.get("factory_hook_verified_before_frames") is True
        and ac.get("policy") == "control"
        and af.get("run_id") == RUN_ID
        and af.get("pid_in_container") == CONTAINER_PID
        and af.get("control_child_exit") == -11
        and af.get("mapping_phases") == list(PHASES)
        and af.get("same_process_core_correlated") is False
        and af.get("faulting_dso_identified") is False
        and af.get("production_acceptance") == "BLOCKED"
    ):
        raise Stop("M23AF protected log does not match accepted control")
    raw = [x[len("M23AF_MAP_SNAPSHOT_JSON="):] for x in lines
           if x.startswith("M23AF_MAP_SNAPSHOT_JSON=")]
    if len(raw) != 2:
        raise Stop("expected exactly two published mapping snapshots")
    snapshots = {}
    for index, record in enumerate(raw):
        sample = json.loads(record)
        phase = PHASES[index]
        if (sample.get("phase") != phase
                or sample.get("run_id") != RUN_ID
                or sample.get("pid_in_container") != CONTAINER_PID
                or not isinstance(sample.get("segments"), list)
                or not 1 <= len(sample["segments"]) <= 2048):
            raise Stop("invalid or out-of-order mapping snapshot")
        if len(sample["segments"]) != af.get("executable_mappings", {}).get(phase):
            raise Stop("mapping count mismatch")
        for item in sample["segments"]:
            if (not isinstance(item, dict)
                    or not all(isinstance(item.get(k), int) and
                               not isinstance(item.get(k), bool)
                               for k in ("start", "end", "file_offset"))
                    or item["start"] < 0 or item["end"] <= item["start"]
                    or item["file_offset"] < 0
                    or item.get("permissions") not in ("r-xp", "r-xs")
                    or not isinstance(item.get("path"), str)
                    or not item["path"].startswith("/")):
                raise Stop("invalid mapping record")
        if not any(Path(x["path"]).name == "librectify.so"
                   for x in sample["segments"]):
            raise Stop("rectifier mapping absent in snapshot")
        snapshots[phase] = sample["segments"]
    return snapshots


def host_preflight():
    if os.geteuid() == 0:
        raise Stop("nonroot only")
    owner_private(BASE, directory=True)
    owner_private(BASE / "control-01.log")
    owner_private(BASE / "image-id-01.txt")
    if (BASE / "image-id-01.txt").read_text().strip() != IMAGE_ID:
        raise Stop("M23AF immutable image record changed")
    snapshots = parse_existing_log(
        (BASE / "control-01.log").read_text(errors="replace"))
    proc = subprocess.run(
        ["coredumpctl", "--no-pager", "info", str(HOST_PID)],
        text=True, capture_output=True, check=False, timeout=20)
    if proc.returncode:
        raise Stop("historical coredumpctl info unavailable")
    return parse_metadata(proc.stdout), snapshots


def join_core(core_path, snapshots):
    notes, file_maps, loads, siginfos = inspect_elf(core_path)
    if notes != 1 or len(siginfos) != 1:
        raise Stop("one NT_FILE and NT_SIGINFO note required")
    signo, errno, si_code, fault = siginfos[0]
    if (signo, errno, si_code) != (11, 0, 1) or fault <= 0:
        raise Stop("not SIGSEGV/SEGV_MAPERR with valid si_addr")
    regs = prstatus_threads(core_path)
    crash_threads = [r for r in regs if r["rip"] == fault]
    if len(crash_threads) != 1:
        raise Stop("no unique NT_PRSTATUS saved RIP equal to si_addr")
    if any(start <= fault < end for start, end, *_ in file_maps + loads):
        raise Stop("fault address still mapped in ELF core")
    phase_matches = {}
    for phase in PHASES:
        hits = [s for s in snapshots[phase]
                if s["start"] <= fault < s["end"]]
        if len(hits) > 1:
            raise Stop("ambiguous overlapping executable mappings")
        phase_matches[phase] = hits[0] if hits else None

    before, shutdown = (phase_matches[x] for x in PHASES)
    if shutdown is not None and before is not None:
        if (shutdown["path"] != before["path"] or
                shutdown["file_offset"] + fault - shutdown["start"] !=
                before["file_offset"] + fault - before["start"]):
            raise Stop("mapping identity/offset changed across phases")
    prior = shutdown or before
    category = ("MATCHED_SHUTDOWN_AND_FACTORY"
                if shutdown is not None and before is not None else
                "MATCHED_SHUTDOWN_ONLY" if shutdown is not None else
                "MATCHED_POST_FACTORY_ONLY" if before is not None else
                "NO_PRIOR_EXECUTABLE_MAP_MATCH")
    return {
        "host_pid": HOST_PID, "container_pid": CONTAINER_PID,
        "run_id": RUN_ID, "fault_rip": f"0x{fault:016x}",
        "fault_rip_page_offset": f"0x{fault & 0xfff:03x}",
        "crashing_thread_tid": crash_threads[0]["tid"],
        "rip_equals_kernel_si_addr": True,
        "rip_unmapped_at_core_time": True,
        "post_factory_executable_match": before is not None,
        "on_shutdown_executable_match": shutdown is not None,
        "map_correspondence": category,
        "prior_mapping_path": prior["path"] if prior else None,
        "prior_mapping_file_offset":
            prior["file_offset"] + fault - prior["start"] if prior else None,
        "prior_mapping_proves_module_loaded_at_fault": False,
        "actual_dlclose_unmap_observed": False,
        "faulting_function_identified": False,
        "root_cause_proven": False,
        "production_acceptance": "BLOCKED",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument("--preflight", action="store_true")
    choice.add_argument("--export-and-join", action="store_true")
    args = parser.parse_args()
    print("M23AG_ROS_EXECUTION=NONE", flush=True)
    print("M23AG_DEVICE_ACCESS=NONE", flush=True)
    if not args.preflight and not args.export_and_join:
        print("M23AG_STATUS=NO_CORE_ACCESS_BY_DEFAULT", flush=True)
        return 0
    try:
        meta, snapshots = host_preflight()
        print("M23AG_PREFLIGHT_JSON=" +
              json.dumps(meta, sort_keys=True), flush=True)
        print("M23AG_PREFLIGHT=PASS", flush=True)
        if args.preflight:
            print("M23AG_CORE_EXPORT=NOT_STARTED", flush=True)
            return 0
        os.umask(0o077)
        OUT.mkdir(parents=True, exist_ok=True)
        owner_private(OUT, directory=True)
        if EXPORT_DIR.exists() or EXPORT_DIR.is_symlink():
            raise Stop("one-shot export directory already exists: no overwrite or resume")
        EXPORT_DIR.mkdir(mode=0o700)
        core = EXPORT_DIR / "core.32869"
        logfile = EXPORT_DIR / "coredumpctl-export.log"
        with logfile.open("x", encoding="utf-8") as dest:
            result = subprocess.run([
                "coredumpctl", "--no-pager", f"--output={core}", "dump",
                str(HOST_PID)],
                stdout=dest, stderr=subprocess.STDOUT,
                timeout=60, check=False)
        if result.returncode:
            raise Stop("retained core export failed")
        owner_private(core)
        if core.stat().st_size < 4096:
            raise Stop("exported core is empty or truncated")
        summary = join_core(core, snapshots)
        print("M23AG_JOIN_RESULT_JSON=" +
              json.dumps(summary, sort_keys=True), flush=True)
        print("M23AG_VERDICT=CORE_RIP_JOIN_COMPLETED_NO_CAUSAL_PROOF",
              flush=True)
        print("M23AG_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 0
    except (Stop, InvalidCore, OSError, ValueError, TypeError,
            KeyError, json.JSONDecodeError, subprocess.TimeoutExpired) as exc:
        print("M23AG_BLOCKED=" + type(exc).__name__ + ":" +
              str(exc)[:180], flush=True)
        print("M23AG_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
