#!/usr/bin/env python3
"""M23AC: match component manager with/without NODELETE on loaded rectifier.

Inherit all unchanged M23AA+M23Y strict safety, calibration, QoS, drain and
child-exit checks. In both arms the custom component manager loads via stock
class_loader, then RTLD_NOLOAD probes and dlcloses the same module.
"""
import json
import os
from pathlib import Path
import sys

LAUNCH = "/opt/m23ac/m23ac_rectifier_container.launch.py"
BINARY = ("/opt/m23ac-cpp-install/lib/"
          "leroy_m23ac_loaded_rectify_lifetime/component_container")
POLICIES = ("control", "nodelete")


def check_hook(log, policy):
    lines = [
        line for line in log.splitlines()
        if "M23AC_FACTORY_HOOK=" in line
    ]
    marker = "M23AC_FACTORY_HOOK=VERIFIED policy=" + policy
    if len(lines) != 1 or marker not in lines[0]:
        raise ValueError("missing or duplicate exact policy factory hook")
    if "M23AC_PRECONDITION_BLOCKED=" in log:
        raise ValueError("child reported invalid precondition")
    return True


def run():
    policy = os.environ.get("M23AC_ARM")
    if policy not in POLICIES:
        raise ValueError("M23AC_ARM must be control or nodelete")
    if os.environ.get("M23AA_ARM") not in (None, ""):
        raise ValueError("M23AA_ARM must remain unset")
    if os.environ.get("M23AB_ARM") not in (None, ""):
        raise ValueError("M23AB_ARM must remain unset")
    if os.environ.get("LD_PRELOAD"):
        raise ValueError("global LD_PRELOAD forbidden")
    for required in (LAUNCH, BINARY):
        if not Path(required).is_file():
            raise ValueError("required M23AC file missing")

    import m23aa_compare as m23aa
    import m23p_no_device_rectifier_shutdown as m23p

    old_launches = dict(m23aa.LAUNCHES)
    old_inspector = m23p.inspect_synthetic_graph
    old_arm = os.environ.get("M23AA_ARM")
    audited = {"hook": False}

    def inspect():
        # The M23P publisher has a pre-frame warm-up window.
        # Require actual child hook marker before accepting graph audit.
        log = m23p.LAUNCH_LOG.read_text(errors="replace")
        audited["hook"] = check_hook(log, policy)
        return old_inspector()

    try:
        m23aa.LAUNCHES["component"] = LAUNCH
        m23p.inspect_synthetic_graph = inspect
        os.environ["M23AA_ARM"] = "component"
        result = m23aa.run()
    finally:
        m23aa.LAUNCHES.clear()
        m23aa.LAUNCHES.update(old_launches)
        m23p.inspect_synthetic_graph = old_inspector
        if old_arm is None:
            os.environ.pop("M23AA_ARM", None)
        else:
            os.environ["M23AA_ARM"] = old_arm

    # Return a marker without passing a clean process off as proof of cause.
    # M23AA_RESULT_JSON is the authoritative full-delivery classifier.
    verdict = "INHERITED_RUN_COMPLETE" if audited["hook"] else "BLOCKED_HOOK"
    print("M23AC_RESULT_JSON=" + json.dumps({
        "policy": policy,
        "factory_hook_verified_before_frames": audited["hook"],
        "inherited_m23aa_exit": result,
        "diagnostic_verdict": verdict,
        "actual_rtld_nodelete_applied_to_image": "UNPROVEN_BEYOND_DLOPEN_SUCCESS",
        "root_cause_proven": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AC_DEVICE_ACCESS=NONE", flush=True)
    print("M23AC_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return result if audited["hook"] else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, RuntimeError, ValueError) as exc:
        print("M23AC_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
