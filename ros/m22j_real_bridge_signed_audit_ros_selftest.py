#!/usr/bin/env python3
"""M22J real ROS bridge + separate-process test signers; FAKE CDC ONLY.

No ROS signed motion subscription. The actual bridge owns an explicit
audit method whose result contains no actuator output. This test injects
synthetic signed packets DIRECTLY from separate process signer fixtures.
The M22D production CDC writer stays ZERO-WHEELS-only.
"""
import json
import os
import time

import rclpy

import leroy_bridge.node as bridge_module
from leroy_bridge.bridge_signed_audit import BridgeSignedAudit
from m22f_fake_bridge_ros_selftest import MemoryCDC, preflight, URI, STOP
from m22i_isolated_role_signing import FakeSessionSupervisor

ENV = {
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M22F_FAKE_SERIAL_ONLY": "1",
    "M22J_BRIDGE_AUDIT_ONLY": "1",
}


def require(yes, why):
    if not yes:
        raise AssertionError("M22J: " + why)


def run():
    preflight()
    for key, value in ENV.items():
        require(os.environ.get(key) == value, "isolation env " + key)
    saved_constructor = bridge_module.serial.Serial
    fakes = []
    bridge = None
    signer = None
    ros_started = False

    def memory_only(*args, **kwargs):
        require(not args, "positional serial args forbidden")
        transport = MemoryCDC(**kwargs)
        fakes.append(transport)
        return transport

    bridge_module.serial.Serial = memory_only
    try:
        rclpy.init(args=[
            "--ros-args", "-p", "serial_port:=" + URI,
            "-p", "imu_stream_hz:=0", "-p", "encoder_stream_hz:=0"])
        ros_started = True
        bridge = bridge_module.LeroyBridgeNode()
        require(len(fakes) == 1, "not exactly one fake CDC")
        fake = fakes[0]
        require(bridge._serial is fake, "bridge attached wrong CDC")
        require(bridge.get_subscriptions_info_by_topic("/cmd_vel") == [],
                "forbidden raw /cmd_vel subscription")
        require(all(not p.startswith(b"set_wheels ") or p == STOP
                    for p in fake.snapshot()),
                "startup wrote nonzero wheels")
        print("M22J_REAL_BRIDGE_FAKE_CDC_STARTUP=PASS", flush=True)

        signer = FakeSessionSupervisor()
        require(signer.motion.pid != signer.operator.pid,
                "signers must be distinct OS processes")
        audit = BridgeSignedAudit(signer.verifier)
        at = time.monotonic_ns()
        candidate = signer.issue_motion(0, at)
        operator = signer.issue_rearm(0, at)
        before = len(fake.snapshot())
        result = bridge._inspect_signed_diagnostic(
            audit, motion=candidate, rearm=operator, now_ns=at)
        require(result.command_authenticated,
                "valid isolated signer command rejected")
        require(result.operator_grant_authenticated,
                "valid isolated operator grant rejected")
        require(result.physical_motion_authorized is False,
                "diagnostic result claimed physical motion permission")
        require(len(fake.snapshot()) == before,
                "bridge audit unexpectedly wrote CDC bytes")
        print("M22J_SIGNED_ROLE_AUDIT_NO_CDC=PASS", flush=True)

        duplicate = bridge._inspect_signed_diagnostic(
            audit, motion=candidate, rearm=operator, now_ns=at + 1)
        require(not duplicate.command_authenticated
                and duplicate.command_reason == "BLOCKED_MOTION_REPLAY",
                "duplicate signed candidate accepted")
        require(not duplicate.operator_grant_authenticated
                and duplicate.rearm_reason == "BLOCKED_REARM_REPLAY",
                "duplicate operator grant accepted")
        require(not duplicate.physical_motion_authorized,
                "replayed packet claimed motion permission")
        print("M22J_REPLAY_AUDIT_FAIL_CLOSED=PASS", flush=True)

        old_motion = signer.issue_motion(1, time.monotonic_ns())
        old_grant = signer.issue_rearm(1, time.monotonic_ns())
        signer.rotate_session()
        new_audit = BridgeSignedAudit(signer.verifier)
        now = time.monotonic_ns()
        blocked = bridge._inspect_signed_diagnostic(
            new_audit, motion=old_motion, rearm=old_grant,
            now_ns=now)
        require(blocked.command_reason == "BLOCKED_SESSION_OR_OWNER",
                "previous session motion accepted")
        require(blocked.rearm_reason == "BLOCKED_REARM_FORMAT",
                "previous session operator grant accepted")
        require(not blocked.physical_motion_authorized,
                "previous session packet claimed permission")
        print("M22J_ROTATED_SESSION_REJECT=PASS", flush=True)

        current = time.monotonic_ns()
        valid = bridge._inspect_signed_diagnostic(
            new_audit,
            motion=signer.issue_motion(0, current),
            rearm=signer.issue_rearm(0, current),
            now_ns=current)
        require(valid.command_authenticated
                and valid.operator_grant_authenticated,
                "fresh session not accepted")
        require(not valid.physical_motion_authorized,
                "new session silently enabled motion")
        print("M22J_NEW_SESSION_STILL_INHIBITED=PASS", flush=True)

        # Real bridge's unchanged M22D/M22G serial gate remains decisive.
        previous = len(fake.snapshot())
        require(bridge._send_command("set_wheels 1 0 0 0") is False,
                "production nonzero wheel command accepted")
        require(len(fake.snapshot()) == previous,
                "fake CDC saw nonzero motor command")
        require(bridge._send_command("set_wheels 0 0 0 0") is True,
                "canonical STOP refused")
        require(fake.snapshot()[-1] == STOP, "wrong STOP packet")
        print("M22J_M22D_NONZERO_CDC_LOCKOUT=PASS", flush=True)

        # Normal fault-free destruction sends STOP and streams-off.
        bridge.destroy_node()
        bridge = None
        require(not fake.is_open, "fake CDC remained open after shutdown")
        record = fake.snapshot()
        require(record[-3:] == [
            STOP, b"set_imu_stream_hz 0\n",
            b"set_encoder_stream_hz 0\n"],
            "shutdown missing zero-wheel and stream disable sequence")
        require(all(not p.startswith(b"set_wheels ") or p == STOP
                    for p in record), "nonzero wheel write")
        print("M22J_REAL_BRIDGE_FAKE_SHUTDOWN=PASS", flush=True)

        print("M22J_REPORT_JSON=" + json.dumps({
            "milestone": "M22J",
            "actual_ros_node": "LeroyBridgeNode",
            "audit_only": True,
            "signer_processes_distinct": True,
            "signed_audit_checks": audit.inspections + new_audit.inspections,
            "actual_bridge_cdc_writes": len(record),
            "actual_bridge_nonzero_writes": 0,
            "physical_motion_authorized": False,
            "operator_authenticated": False,
            "device_access": False,
            "physical_stop_acknowledgement_validated": False,
        }, sort_keys=True), flush=True)
        print("M22J_REAL_BRIDGE_SIGNED_AUDIT_SELFTEST=PASS", flush=True)
        print("M22J_VERDICT=INJECTED_SIGNED_AUDIT_FAKE_CDC_ONLY", flush=True)
        print("M22J_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        if signer is not None:
            signer.close()
        if bridge is not None:
            bridge.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()
        bridge_module.serial.Serial = saved_constructor


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M22J_REAL_BRIDGE_SIGNED_AUDIT_SELFTEST=FAIL", flush=True)
        print("M22J_ERROR=" + type(exc).__name__ + ": " + str(exc),
              flush=True)
        raise
