#!/usr/bin/env python3
"""M23AB: reuse *unchanged* M23AA strict synthetic validation and child exit.

One arm per fresh isolated container:
  component: original M23AA stock component control, no LD_PRELOAD;
  direct:    same stock librectify via directly-linked diagnostic executable.
"""
import os
from pathlib import Path
import sys

DIRECT = "/opt/m23ab/m23ab_direct_rectifier.launch.py"
ARMS = ("component", "direct")


def run():
    arm = os.environ.get("M23AB_ARM")
    if arm not in ARMS:
        raise ValueError("explicit M23AB_ARM=component or direct required")
    if os.environ.get("M23AA_ARM") not in (None, ""):
        raise ValueError("cannot combine M23AB_ARM with M23AA_ARM")
    if os.environ.get("LD_PRELOAD"):
        raise ValueError("global loader tracing/preload prohibited")
    if not Path(DIRECT).is_file():
        raise ValueError("direct-linked diagnostic launch missing")
    if not Path(
        "/opt/m23ab-cpp-install/lib/leroy_m23ab_direct_rectifier/rectify_node"
    ).is_file():
        raise ValueError("direct-linked diagnostic executable missing")

    import m23aa_compare as m23aa
    # For the direct arm, the launch action still uses executable basename
    # rectify_node, preserving M23AA's child-process exit-log classifier.
    # Keep the component arm identical to the accepted M23AA control.
    previous_launches = dict(m23aa.LAUNCHES)
    previous_arm = os.environ.get("M23AA_ARM")
    try:
        m23aa.LAUNCHES["direct"] = DIRECT
        os.environ["M23AA_ARM"] = arm
        print("M23AB_ARM=" + arm, flush=True)
        status = m23aa.run()
    finally:
        m23aa.LAUNCHES.clear()
        m23aa.LAUNCHES.update(previous_launches)
        if previous_arm is None:
            os.environ.pop("M23AA_ARM", None)
        else:
            os.environ["M23AA_ARM"] = previous_arm

    # M23AA_RESULT_JSON contains authoritative 900/900 counts, graph/QoS,
    # complete drain acceptance, and independent child-process exit.
    print("M23AB_INHERITED_RUN_EXIT=" + str(status), flush=True)
    print("M23AB_RUNTIME_KIND=" +
          ("STOCK_COMPOSABLE" if arm == "component" else "STOCK_DIRECT_LINKED"),
          flush=True)
    print("M23AB_DEVICE_ACCESS=NONE", flush=True)
    print("M23AB_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return status


if __name__ == "__main__":
    try:
        sys.exit(run())
    except (OSError, RuntimeError, ValueError) as exc:
        print("M23AB_PRECONDITION_BLOCKED=" + type(exc).__name__, flush=True)
        sys.exit(2)
