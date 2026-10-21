#!/usr/bin/env python3
"""M23AK: run ONE inherited, synthetic 900/900 stock composable shutdown.

Reuses M23AC pre-frame factory hook and M23AA strict data/graph/QoS/drain/
child-exit checks plus M23AF same-process executable maps in two phases.
The image is expected to supply the new pthreads OpenCV via LD_LIBRARY_PATH.
Never runs camera, serial, real robot or production nodes.
"""
import contextlib
import io
import json
import os
from pathlib import Path
import secrets
import sys

from m23ak_maps_gate import Blocked, PREFIX, inspect_maps


def exactly_one(output, marker):
    found = [line[len(marker):] for line in output.splitlines()
             if line.startswith(marker)]
    if len(found) != 1:
        raise Blocked("missing or duplicated " + marker)
    return json.loads(found[0])


def validate_clean(output, exit_code):
    base = exactly_one(output, "M23AA_RESULT_JSON=")
    hook = exactly_one(output, "M23AC_RESULT_JSON=")
    expected_counts = {"raw": 900, "rect": 900, "info": 900}
    if not (
        exit_code == 0
        and base.get("arm") == "component"
        and base.get("verdict") == "CLEAN_STOCK_RECTIFIER_900_OF_900"
        and base.get("parent_exit") == 0
        and base.get("child_exit") == 0
        and base.get("counts") == expected_counts
        and base.get("graph_verified") is True
        and base.get("stock_rectifier") is True
        and base.get("synthetic_only") is True
        and base.get("production_acceptance") == "BLOCKED"
        and hook.get("policy") == "control"
        and hook.get("factory_hook_verified_before_frames") is True
        and hook.get("inherited_m23aa_exit") == 0
        and hook.get("diagnostic_verdict") == "INHERITED_RUN_COMPLETE"
        and hook.get("root_cause_proven") is False
    ):
        raise Blocked("strict 900/900 stock component clean-child contract failed")
    return base


def run():
    if os.environ.get("M23AK_ARM") != "pthreads":
        raise Blocked("require explicit M23AK_ARM=pthreads")
    for key in ("M23AF_ARM", "M23AC_ARM", "M23AA_ARM", "M23AB_ARM"):
        if os.environ.get(key):
            raise Blocked("prior arm not empty: " + key)
    if os.environ.get("LD_PRELOAD"):
        raise Blocked("LD_PRELOAD is prohibited")
    if os.environ.get("LD_LIBRARY_PATH", "").split(":")[0] != PREFIX.rstrip("/"):
        raise Blocked("pthreads OpenCV must be first in LD_LIBRARY_PATH")
    for target in (
        "/opt/m23aj-opencv/lib/libopencv_core.so.4.6.0",
        "/opt/m23af/m23af_rectifier_container.launch.py",
        "/opt/m23af-cpp-install/lib/leroy_m23af_shutdown_maps/component_container",
    ):
        if not Path(target).is_file():
            raise Blocked("required diagnostic file missing: " + target)

    import m23ac_compare as m23ac
    import m23af_compare as m23af
    import m23p_no_device_rectifier_shutdown as m23p

    nonce = secrets.token_hex(16)
    m23ac.LAUNCH = m23af.LAUNCH
    m23ac.BINARY = m23af.BINARY
    os.environ["M23AC_ARM"] = "control"
    os.environ["M23AF_RUN_ID"] = nonce

    print("M23AK_RUN_ID=" + nonce, flush=True)
    buffer = io.StringIO()
    try:
        with contextlib.redirect_stdout(buffer):
            status = m23ac.run()
    finally:
        sys.stdout.write(buffer.getvalue())
        sys.stdout.flush()

    base = validate_clean(buffer.getvalue(), status)
    log = m23p.LAUNCH_LOG.read_text(errors="replace")
    parsed = m23af.parse_maps(log, nonce)
    audit = inspect_maps(parsed)
    print("M23AK_RESULT_JSON=" + json.dumps({
        "verdict": "TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN",
        "run_id": nonce,
        "pid_in_container": parsed["pid_in_container"],
        "stock_composable_rectifier": True,
        "factory_policy": "control",
        "opencv_prefix": PREFIX,
        "opencv_mapping_audit": audit,
        "inherited_900_frame_counts": base["counts"],
        "parent_exit": base["parent_exit"],
        "child_exit": base["child_exit"],
        "tbb_absent_from_two_executable_map_snapshots": True,
        "any_other_mapping_phase_checked": False,
        "abi_proven_for_all_ros_packages": False,
        "tbb_unload_cause_proven": False,
        "physical_device_validation": "NONE",
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AK_VERDICT=TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN",
          flush=True)
    print("M23AK_DEVICE_ACCESS=NONE", flush=True)
    print("M23AK_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (Blocked, OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print("M23AK_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:170],
              flush=True)
        print("M23AK_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        sys.exit(2)
