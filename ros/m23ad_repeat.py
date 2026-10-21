#!/usr/bin/env python3
"""M23AD: preregistered, fail-closed ABBA replication of M23AC.

Runs no ROS locally: four one-at-a-time rootless Podman containers, only with
explicit --execute and exact previously validated immutable image ID.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

PINNED_IMAGE = "a3ca7cef00be950a52da0685407f54e12bc2d14b7a0a327e745c3e5adc9392f6"
IMAGE_TAG = "localhost/leroy-sensors:m23ac-loaded-rectify-v1"
PARENT_TAG = "localhost/leroy-sensors:m23ab-direct-linked-v2"
BASE = Path.home() / ".local/state/leroy-m23ac"
DEST = Path.home() / ".local/state/leroy-m23ad"
TRIALS = (("01", "control"), ("02", "nodelete"),
          ("03", "nodelete"), ("04", "control"))


class EvidenceError(Exception):
    """Fail closed on incomplete evidence, unexpected outcome or provenance."""


def exactly_one(lines, prefix):
    values = [s[len(prefix):] for s in lines if s.startswith(prefix)]
    if len(values) != 1:
        raise EvidenceError("expected exactly one " + prefix)
    return values[0]


def validate(log, policy, rc):
    """Check both M23AA's exact 900/900 verdict and M23AC factory hook."""
    lines = log.splitlines()
    try:
        base = json.loads(exactly_one(lines, "M23AA_RESULT_JSON="))
        hook = json.loads(exactly_one(lines, "M23AC_RESULT_JSON="))
    except (ValueError, TypeError) as exc:
        raise EvidenceError("invalid result JSON") from exc
    expected_rc = 2 if policy == "control" else 0
    expected_child = -11 if policy == "control" else 0
    expected = ("SIGSEGV_STOCK_RECTIFIER_900_OF_900" if policy == "control"
                else "CLEAN_STOCK_RECTIFIER_900_OF_900")
    conditions = (
        rc == expected_rc,
        base.get("arm") == "component",
        base.get("child_exit") == expected_child,
        base.get("parent_exit") == expected_rc,
        base.get("verdict") == expected,
        base.get("counts") == {"raw": 900, "rect": 900, "info": 900},
        base.get("graph_verified") is True,
        base.get("stock_rectifier") is True,
        base.get("synthetic_only") is True,
        base.get("production_acceptance") == "BLOCKED",
        exactly_one(lines, "M23AA_VERDICT=") == expected,
        hook.get("policy") == policy,
        hook.get("factory_hook_verified_before_frames") is True,
        hook.get("diagnostic_verdict") == "INHERITED_RUN_COMPLETE",
        hook.get("inherited_m23aa_exit") == expected_rc,
        hook.get("production_acceptance") == "BLOCKED",
        hook.get("root_cause_proven") is False,
        exactly_one(lines, "M23AC_DEVICE_ACCESS=") == "NONE",
        exactly_one(lines, "M23AC_PRODUCTION_ACCEPTANCE=") == "BLOCKED",
    )
    if not all(conditions):
        raise EvidenceError("nonmatching or incomplete " + policy + " evidence")
    return {"policy": policy, "exit": rc, "child_exit": expected_child,
            "verdict": expected, "counts": base["counts"],
            "graph_verified": True, "factory_hook_verified": True}


def podman_field(tag, fmt):
    result = subprocess.run(["podman", "image", "inspect", tag,
                             "--format", fmt],
                            capture_output=True, text=True, check=True)
    return result.stdout.strip()


def preflight():
    if os.geteuid() == 0:
        raise EvidenceError("must use rootless Podman as nonroot user")
    if os.environ.get("LD_PRELOAD"):
        raise EvidenceError("global LD_PRELOAD forbidden")
    recorded = (BASE / "matched-image-id-02.txt").read_text().strip()
    current = podman_field(IMAGE_TAG, "{{.Id}}")
    parent = podman_field(PARENT_TAG, "{{.Id}}")
    if recorded != PINNED_IMAGE or current != PINNED_IMAGE or parent == PINNED_IMAGE:
        raise EvidenceError("control record, image tag or parent identity mismatch")
    env_lines = podman_field(IMAGE_TAG, "{{range .Config.Env}}{{println .}}{{end}}")
    env = set(env_lines.splitlines())
    required = {"M23AC_ARM=", "M23AB_ARM=", "M23AA_ARM=", "M23R_TRACE_ONLY=0",
                "M23Y_DRAIN_ONLY=1", "M23S_DISTORTION_PROFILE=rectify"}
    if not required.issubset(env) or any(s.startswith("LD_PRELOAD=") for s in env):
        raise EvidenceError("image environment contract not satisfied")

    # All original image-matched evidence must still be valid before adding trials.
    validate((BASE / "control-02.log").read_text(errors="replace"), "control", 2)
    validate((BASE / "nodelete-01.log").read_text(errors="replace"), "nodelete", 0)
    return PINNED_IMAGE


def run_trial(image_id, trial, policy):
    dest = DEST / ("trial-" + trial + "-" + policy + ".log")
    cmd = ["timeout", "--signal=TERM", "--kill-after=5s", "120s",
           "podman", "run", "--rm", "--pull=never", "--network=none",
           "--ipc=private", "--pid=private", "--cap-drop=all",
           "--security-opt=no-new-privileges", "--read-only",
           "--tmpfs", "/tmp:rw,nosuid,nodev,size=128m", "--pids-limit=128",
           "--memory=2g", "--env", "M23AC_ARM=" + policy, image_id]
    with dest.open("x", encoding="utf-8") as fh:
        try:
            result = subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT,
                                    check=False)
            rc = result.returncode
        except OSError as exc:
            raise EvidenceError("failed to start trial process") from exc
    raw = dest.read_text(errors="replace")
    evidence = validate(raw, policy, rc)
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="explicitly authorize exactly four sequenced Podman trials")
    args = parser.parse_args(argv)
    print("M23AD_PREDECLARED_SEQUENCE=control,nodelete,nodelete,control", flush=True)
    if not args.execute:
        print("M23AD_STATUS=NO_RUNTIME_NOT_AUTHORIZED", flush=True)
        return 0

    os.umask(0o077)
    try:
        image_id = preflight()
        DEST.mkdir(parents=True, exist_ok=True)
        lock = DEST / "one_shot_abba_lock"
        plan = DEST / "m23ad-plan-v1.json"
        if lock.exists() or plan.exists() or any(DEST.glob("trial-*.log")):
            raise EvidenceError("existing M23AD run evidence: will not overwrite or resume")
        # Atomic lock, intentionally permanent even after failures or interruption.
        lock.mkdir()
        with plan.open("x", encoding="utf-8") as fh:
            json.dump({"protocol": "M23AD_ABBA_V1", "image_id": image_id,
                       "trials": list(TRIALS), "production_acceptance": "BLOCKED"}, fh)
            fh.write("\n")
        print("M23AD_PREFLIGHT=PASS", flush=True)
        print("M23AD_IMAGE_ID=" + image_id, flush=True)
        for idx, policy in TRIALS:
            print("M23AD_TRIAL_START=" + idx + ":" + policy, flush=True)
            evidence = run_trial(image_id, idx, policy)
            print("M23AD_TRIAL_RESULT=" + json.dumps(
                {"trial": idx, **evidence}, sort_keys=True), flush=True)
        print("M23AD_ABBA_VERDICT=FOUR_OF_FOUR_MATCHED", flush=True)
        print("M23AD_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 0
    except (EvidenceError, OSError, subprocess.CalledProcessError) as exc:
        print("M23AD_STOP=" + type(exc).__name__ + ":" + str(exc), flush=True)
        print("M23AD_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
