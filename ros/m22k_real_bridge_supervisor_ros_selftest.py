#!/usr/bin/env python3
"""M22K actual LeroyBridgeNode + signed DIAGNOSTIC supervisor, fake CDC.

Uses M22I separately spawned fake motion/rearm signers and injected
M22F MemoryCDC. All timestamps and health are SYNTHETIC; no signed ROS
subscription, authenticated operator, actual sensor, motor command,
physical CDC port or protection mechanism is involved.
"""
import json
import os
import time

import rclpy

import leroy_bridge.node as bridge_module
from leroy_bridge.bridge_signed_audit import BridgeSignedAudit
from leroy_bridge.bridge_diagnostic_supervisor import (
    BridgeDiagnosticSupervisor, RECOVERY_NS,
)
from m22f_fake_bridge_ros_selftest import MemoryCDC, preflight, URI, STOP
from m22i_isolated_role_signing import FakeSessionSupervisor

ENV = {
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M22F_FAKE_SERIAL_ONLY": "1",
    "M22K_DIAGNOSTIC_SUPERVISOR_ONLY": "1",
}
HEALTH = {
    "odom_fresh": True, "depth_fresh": True, "tf_fresh": True,
    "costmap_valid": True, "localization_valid": True,
}
OWNER = ("controller_server",)


def require(ok, detail):
    if not ok:
        raise AssertionError("M22K: " + detail)


def run():
    preflight()
    for name, expected in ENV.items():
        require(os.environ.get(name) == expected,
                "isolation environment differs: " + name)
    actual_constructor = bridge_module.serial.Serial
    fakes = []
    initialized = False
    bridge = None
    signers = None

    def fake_factory(*args, **kwargs):
        require(not args, "fake CDC must use named arguments only")
        fake = MemoryCDC(**kwargs)
        fakes.append(fake)
        return fake

    bridge_module.serial.Serial = fake_factory
    try:
        rclpy.init(args=[
            "--ros-args", "-p", "serial_port:=" + URI,
            "-p", "imu_stream_hz:=0", "-p", "encoder_stream_hz:=0"])
        initialized = True
        bridge = bridge_module.LeroyBridgeNode()
        require(len(fakes) == 1 and bridge._serial is fakes[0],
                "actual bridge constructed with unexpected serial")
        fake = fakes[0]
        require(bridge.get_subscriptions_info_by_topic("/cmd_vel") == [],
                "raw velocity subscriber unexpectedly reintroduced")
        signers = FakeSessionSupervisor()
        require(signers.motion.pid != signers.operator.pid,
                "synthetic signer roles share process")
        supervisor = BridgeDiagnosticSupervisor(
            BridgeSignedAudit(signers.verifier))

        def check(number, at_ns, *, rearm_number=None, health=HEALTH,
                  sources=OWNER):
            cmd = signers.issue_motion(number, at_ns)
            rearm = (None if rearm_number is None
                     else signers.issue_rearm(rearm_number, at_ns))
            return bridge._supervise_signed_diagnostic(
                supervisor, motion=cmd, rearm=rearm, now_ns=at_ns,
                health=health, publishers=sources)

        start = time.monotonic_ns()
        initial_count = len(fake.snapshot())
        inhibited = check(0, start)
        require(inhibited.state == "LATCHED"
                and not inhibited.diagnostic_permission
                and not inhibited.physical_motion_authorized,
                "startup automatically authorized motion")
        require(len(fake.snapshot()) == initial_count,
                "bridge supervisor unexpectedly emitted CDC command")
        print("M22K_REAL_BRIDGE_DIAGNOSTIC_STARTUP_LATCH=PASS", flush=True)

        rearm_at = start + 100_000_000
        beginning = check(1, rearm_at, rearm_number=0)
        require(beginning.state == "REARMING"
                and not beginning.diagnostic_permission,
                "synthetic rearm did not begin inhibited dwell")
        for i in range(2, 11):
            at = rearm_at + (i - 1) * 100_000_000
            pending = check(i, at)
            require(pending.state == "REARMING"
                    and not pending.diagnostic_permission,
                    "rearm dwell shortcut")
        armed_at = rearm_at + RECOVERY_NS
        armed = check(11, armed_at)
        require(armed.state == "ARMED"
                and armed.diagnostic_permission
                and not armed.physical_motion_authorized,
                "valid signed synthetic session did not arm diagnostics")
        require(len(fake.snapshot()) == initial_count,
                "valid diagnostic permission created real bridge CDC I/O")
        print("M22K_SIGNED_REARM_DWELL_DIAGNOSTIC_ONLY=PASS", flush=True)
        print("M22K_SUPERVISOR_NO_CDC_IO=PASS", flush=True)

        bad_health_at = armed_at + 100_000_000
        sensor_loss = check(
            12, bad_health_at,
            health=dict(HEALTH, localization_valid=False))
        require(sensor_loss.state == "LATCHED"
                and sensor_loss.reason == "BLOCKED_SENSOR_HEALTH",
                "synthetic health loss was ignored")
        recovered_at = bad_health_at + 100_000_000
        recovered = check(13, recovered_at)
        require(recovered.state == "LATCHED"
                and recovered.reason == "BLOCKED_EXPLICIT_REARM_REQUIRED",
                "healthy data silently restored authorization")
        print("M22K_SENSOR_FAULT_NO_AUTOMATIC_REARM=PASS", flush=True)

        duplicate_at = recovered_at + 100_000_000
        duplicate = check(
            14, duplicate_at,
            sources=("controller_server", "untrusted"))
        require(duplicate.state == "LATCHED"
                and duplicate.reason == "BLOCKED_SYNTHETIC_SOURCE_CONFLICT",
                "fake competing source not rejected")
        print("M22K_DUPLICATE_SOURCE_LABEL_INHIBIT=PASS", flush=True)

        rearmed_at = duplicate_at + 100_000_000
        again = check(15, rearmed_at, rearm_number=1)
        require(again.state == "REARMING"
                and not again.diagnostic_permission,
                "rearm bypassed dwell after source conflict")

        # The actual bridge's OWN clock-fault flag cannot be overridden
        # by otherwise valid test HMAC signatures and healthy input.
        bridge._clock_fault = True
        clock_at = rearmed_at + 100_000_000
        clock_fault = check(16, clock_at)
        require(clock_fault.state == "LATCHED"
                and clock_fault.reason == "BLOCKED_LOCAL_CLOCK_FAULT"
                and not clock_fault.physical_motion_authorized,
                "actual bridge clock fault failed to veto audit")
        bridge._clock_fault = False
        after_clock_at = clock_at + 100_000_000
        after_clock = check(17, after_clock_at)
        require(after_clock.state == "LATCHED"
                and not after_clock.diagnostic_permission,
                "clock fault recovery silently rearmed diagnostics")
        print("M22K_LOCAL_BRIDGE_CLOCK_FAULT_VETO=PASS", flush=True)

        # Terminal fake serial error sets actual bridge fault flags.
        fake.fail_next_write = True
        before_fault = len(fake.snapshot())
        require(bridge._send_command("set_wheels 0 0 0 0") is False,
                "injected fake CDC error was not surfaced")
        require(bridge._transport_fault.is_set()
                and bridge._stop_event.is_set(),
                "actual bridge terminal CDC fault was not latched")
        fault_at = after_clock_at + 100_000_000
        local_fault = check(18, fault_at)
        require(local_fault.reason == "BLOCKED_LOCAL_TRANSPORT_FAULT"
                and local_fault.state == "LATCHED"
                and not local_fault.physical_motion_authorized,
                "bridge transport failure did not veto supervisor")
        require(bridge._send_command("keepalive") is False,
                "post-fault bridge keepalive accepted")
        require(len(fake.snapshot()) == before_fault,
                "bridge CDC wrote again after terminal fault")
        print("M22K_LOCAL_BRIDGE_TRANSPORT_FAULT_VETO=PASS", flush=True)
        print("M22K_M22G_TERMINAL_CDC_LATCH=PASS", flush=True)

        bridge.destroy_node()
        bridge = None
        require(fake.is_open is False, "fake CDC remained open")
        packets = fake.snapshot()
        require(len(packets) == before_fault,
                "CDC commands written during faulted shutdown")
        require(all(not p.startswith(b"set_wheels ") or p == STOP
                    for p in packets),
                "nonzero wheel packet seen in bridge fake CDC")
        print("M22K_POST_FAULT_SHUTDOWN_SILENT=PASS", flush=True)

        print("M22K_ROS_REPORT_JSON=" + json.dumps({
            "milestone": "M22K",
            "actual_ros_node": "LeroyBridgeNode",
            "signed_roles_in_distinct_processes": True,
            "diagnostic_only": True,
            "synthetic_sensor_health": True,
            "synthetic_source_labels_only": True,
            "signed_diagnostic_permission_observed": True,
            "fake_cdc_writes": len(packets),
            "nonzero_bridge_wheel_writes": 0,
            "bridge_transport_fault_latched": True,
            "operator_authenticated": False,
            "production_motion_enabled": False,
            "physical_stop_acknowledgement_validated": False,
            "physical_device_access": False,
        }, sort_keys=True), flush=True)
        print("M22K_REAL_BRIDGE_SUPERVISOR_SELFTEST=PASS", flush=True)
        print("M22K_VERDICT=SIGNED_DIAGNOSTIC_VETO_WITH_FAKE_CDC_ONLY",
              flush=True)
        print("M22K_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        if signers is not None:
            signers.close()
        if bridge is not None:
            bridge.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        bridge_module.serial.Serial = actual_constructor


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M22K_REAL_BRIDGE_SUPERVISOR_SELFTEST=FAIL", flush=True)
        print("M22K_ERROR=" + type(exc).__name__ + ": " + str(exc),
              flush=True)
        raise
