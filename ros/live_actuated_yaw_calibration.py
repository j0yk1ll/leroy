#!/usr/bin/env python3
"""Motor-capable automatic yaw calibration over the bridge's signed ROS ingress.

THIS IS AN ACTUATING NODE. It can sign and publish a nonzero wz after:
* one exclusive, live /leroy_bridge MCU/IMU + signed motor session;
* automatic stationary gyro bias and independent 8s holdout PASS;
* authenticated, fresh swept-volume + independent-stop hardware leases;
* a distinct locally provisioned operator rearm and motion signing key.

The bridge retains the sole physical MCU writer, its own independent local
interlocks, signed session verifier, 500ms watchdog and zero/STOP handling.
No ROS parameter or self-reported Bool can substitute for those interlocks.
There is no fallback to M18's concurrent serial sidecar, raw cmd_vel, or
synthetic costmap clearance.
"""
import json
import math
import os
import time
from dataclasses import asdict

from boot_yaw_core import BootYaw
from boot_auto_calibration_core import AutoImuCalibration
from boot_yaw_tracking_core import Heading
from autonomous_motor_yaw_core import (
    MotorYawCalibration, SafetyLeaseVerifier,
)

from leroy_bridge.authorized_session import sign_motion, sign_rearm
from leroy_bridge.motor_signed_ingress import read_protected_key

IMU_TOPIC = "/imu/data_raw"
BRIDGE_SESSION = "/leroy/motor/session_id"
LEASE_TOPIC = "/leroy/safety/rotation_lease"
SIGNED_REARM = "/leroy/motor/signed_rearm"
SIGNED_MOTION = "/leroy/motor/signed_motion"
STATUS = "/leroy_boot/actuated_calibration_status"
IMU_OWNER = ("leroy_bridge", "/", "sensor_msgs/msg/Imu")
SESSION_OWNER = ("leroy_bridge", "/", "std_msgs/msg/String")
# A separately physically commissioned, keyed monitor—not the controller.
LEASE_OWNER = ("leroy_physical_safety_monitor", "/", "std_msgs/msg/String")
REQUIRED_ENV = {
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "LEROY_SIGNED_YAW_CONTROLLER": "1",
}
STOP_QUIET_RAD_S = 0.04
MAX_ACTUATION_WITHOUT_YAW_NS = 2_500_000_000
START_TIMEOUT_NS = 25_000_000_000


def preflight(env=None, exists=None):
    env = os.environ if env is None else env
    exists = os.path.exists if exists is None else exists
    for k, value in REQUIRED_ENV.items():
        if env.get(k) != value:
            raise RuntimeError("MOTOR_YAW_ENV_MISSING_" + k)
    if any(exists(device) for device in (
            "/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0")):
        raise RuntimeError("MOTOR_YAW_CONTROLLER_DEVICE_ACCESS_FORBIDDEN")
    return True


def _strict_owner(node, topic, expected):
    pubs = node.get_publishers_info_by_topic(topic)
    return (
        len(pubs) == 1
        and (pubs[0].node_name, pubs[0].node_namespace,
             pubs[0].topic_type) == expected)


def _key(envname):
    path = os.environ.get(envname)
    if not path or not path.startswith("/run/secrets/"):
        raise RuntimeError("MOTOR_YAW_PROTECTED_KEY_NOT_MOUNTED_" + envname)
    return read_protected_key(path)


def run():
    preflight()
    motion_key = _key("LEROY_MOTION_KEY")
    operator_key = _key("LEROY_OPERATOR_KEY")
    safety_key = _key("LEROY_SAFETY_MONITOR_KEY")
    if len({motion_key, operator_key, safety_key}) != 3:
        raise RuntimeError("MOTOR_YAW_ROLE_KEYS_MUST_BE_DISTINCT")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy)
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=32,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    durable_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    live_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    class Controller(Node):
        def __init__(self):
            super().__init__(
                "leroy_autonomous_yaw_controller", enable_rosout=False,
                start_parameter_services=False)
            self.yaw = BootYaw()
            self.cal = AutoImuCalibration()
            self.motor = None
            self.verifier = None
            self.session_id = None
            self.last_heading = None
            self.last_imu_stamp = None
            self.last_imu_receipt = None
            self.start_ns = time.monotonic_ns()
            self.rearm_issued_ns = None
            self.sequence = 0
            self.motion_start_ns = None
            self.motion_start_yaw = None
            self.last_nonzero_ns = None
            self.latest_safety_lease = None
            self.stop_requested = False
            self.finished = False
            self.failed_reason = None
            self.last_status_log = 0
            self._imu_sub = self.create_subscription(
                Imu, IMU_TOPIC, self.on_imu, sensor_qos)
            self._session_sub = self.create_subscription(
                String, BRIDGE_SESSION, self.on_session, durable_qos)
            self._safety_sub = self.create_subscription(
                String, LEASE_TOPIC, self.on_safety, live_qos)
            self._motion_pub = self.create_publisher(
                String, SIGNED_MOTION, live_qos)
            self._rearm_pub = self.create_publisher(
                String, SIGNED_REARM, live_qos)
            self._status_pub = self.create_publisher(String, STATUS, 1)
            if (self._imu_sub.topic_name != IMU_TOPIC
                    or self._session_sub.topic_name != BRIDGE_SESSION
                    or self._safety_sub.topic_name != LEASE_TOPIC
                    or self._motion_pub.topic_name != SIGNED_MOTION
                    or self._rearm_pub.topic_name != SIGNED_REARM):
                raise RuntimeError("MOTOR_YAW_ROS_REMAP_NOT_ALLOWED")
            self.timer = self.create_timer(0.05, self.tick)

        def graph_ok(self):
            return (
                _strict_owner(self, IMU_TOPIC, IMU_OWNER)
                and _strict_owner(self, BRIDGE_SESSION, SESSION_OWNER)
                and _strict_owner(self, LEASE_TOPIC, LEASE_OWNER)
                and self._motion_pub.get_subscription_count() == 1
                and self._rearm_pub.get_subscription_count() == 1
                and not any(name == "m18_imu_source"
                            for name, _ in self.get_node_names_and_namespaces()))

        def on_session(self, msg):
            if not _strict_owner(self, BRIDGE_SESSION, SESSION_OWNER):
                self.fail("MOTOR_BRIDGE_SESSION_OWNER_CHANGED")
                return
            sid = msg.data
            if (type(sid) is not str or len(sid) != 32
                    or not all(k in "0123456789abcdef" for k in sid)):
                self.fail("INVALID_BRIDGE_SESSION")
                return
            if self.session_id is not None and self.session_id != sid:
                self.fail("BRIDGE_SESSION_REPLACED")
                return
            if self.session_id is None:
                self.session_id = sid
                self.verifier = SafetyLeaseVerifier(
                    safety_key=safety_key, bridge_session_id=sid)
                self.motor = MotorYawCalibration(sid)

        def on_safety(self, msg):
            if not _strict_owner(self, LEASE_TOPIC, LEASE_OWNER):
                self.fail("SAFETY_MONITOR_PUBLISHER_CHANGED")
                return
            if self.verifier is None:
                return
            now_ns = time.monotonic_ns()
            lease = self.verifier.accept(msg.data, now_ns)
            if lease is None:
                # A malformed or unauthenticated safety lease latches once
                # the motor session is armed. Before that, never rearm.
                if self.rearm_issued_ns is not None:
                    self.fail("INVALID_HARDWARE_SAFETY_LEASE")
                return
            self.latest_safety_lease = lease
            if self.motor is not None and self.motor.started:
                if self.motor.fresh_lease(lease, now_ns) is not True:
                    self.fail("SAFETY_LEASE_REVOKED")

        def on_imu(self, msg):
            now_ns = time.monotonic_ns()
            if not _strict_owner(self, IMU_TOPIC, IMU_OWNER):
                self.fail("DUPLICATE_OR_REPLACED_BRIDGE_IMU")
                return
            if msg.header.frame_id != "imu_link":
                self.fail("INVALID_BRIDGE_IMU_FRAME")
                return
            stamp_ns = (
                int(msg.header.stamp.sec)*1_000_000_000
                + int(msg.header.stamp.nanosec))
            age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            if not math.isfinite(age) or not -0.1 <= age <= 0.5:
                self.fail("IMU_HEADER_TIMESTAMP_OUTSIDE_AGE")
                return
            acc = msg.linear_acceleration
            gyro = msg.angular_velocity
            e = self.yaw.update(
                stamp_ns, now_ns,
                accel=(acc.x, acc.y, acc.z),
                gyro=(gyro.x, gyro.y, gyro.z))
            if e.state == "FAULT_LATCHED":
                self.fail("IMU_YAW_" + e.reason)
                return
            c = self.cal.update(
                stamp_ns, now_ns,
                accel=(acc.x, acc.y, acc.z),
                gyro=(gyro.x, gyro.y, gyro.z),
                heading_state=e.state, heading_yaw_rad=e.relative_yaw_rad,
                heading_bias_rad_s=e.gyro_z_bias_rad_s)
            if c.state == "FAULT_LATCHED":
                self.fail("IMU_CALIBRATION_" + c.reason)
                return
            if e.state == "HEADING_ACTIVE":
                self.last_heading = Heading(
                    stamp_ns=now_ns,
                    receipt_ns=now_ns,
                    yaw_rad=e.relative_yaw_rad,
                    source_active=True, owner_verified=True)
                self.last_imu_receipt = now_ns

        def fail(self, reason):
            if self.failed_reason is None:
                self.failed_reason = reason
                if self.motor is not None:
                    self.motor.abort(reason)
                self.stop_requested = True

        def publish_signed_motion(self, wz):
            if self.session_id is None:
                return
            self.sequence += 1
            t = time.monotonic_ns()
            envelope = sign_motion(
                key=motion_key, session_id=self.session_id,
                sequence=self.sequence,
                generated_ns=t, expires_ns=t+150_000_000,
                vx=0.0, vy=0.0, wz=float(wz))
            self._motion_pub.publish(
                String(data=json.dumps(
                    asdict(envelope), sort_keys=True, allow_nan=False)))
            if wz != 0.0:
                self.last_nonzero_ns = t

        def publish_signed_rearm(self):
            t = time.monotonic_ns()
            grant = sign_rearm(
                key=operator_key, session_id=self.session_id,
                counter=1, generated_ns=t, expires_ns=t+150_000_000)
            self._rearm_pub.publish(
                String(data=json.dumps(asdict(grant),
                                       sort_keys=True, allow_nan=False)))
            self.rearm_issued_ns = t
            print("MOTOR_YAW_SIGNED_REARM_SENT=1", flush=True)

        def status(self, now_ns):
            if now_ns - self.last_status_log < 1_000_000_000:
                return
            self.last_status_log = now_ns
            state = {
                "schema": "leroy_actuated_yaw_calibration_v1",
                "motion_path": "signed_bridge_only",
                "yaw_state": self.yaw.state,
                "auto_calibration_state": self.cal.state,
                "safety_lease_present": self.latest_safety_lease is not None,
                "bridge_session_present": self.session_id is not None,
                "signed_rearm_sent": self.rearm_issued_ns is not None,
                "last_nonzero_issued": self.last_nonzero_ns is not None,
                "controller_fault": self.failed_reason,
                "yaw_deg": (
                    math.degrees(self.last_heading.yaw_rad)
                    if self.last_heading is not None else None),
            }
            self._status_pub.publish(String(
                data=json.dumps(state, sort_keys=True, allow_nan=False)))
            print("MOTOR_YAW_STATUS_JSON=" +
                  json.dumps(state, sort_keys=True), flush=True)

        def tick(self):
            now_ns = time.monotonic_ns()
            self.status(now_ns)
            if self.finished or self.failed_reason:
                return
            if not self.graph_ok():
                if (self.rearm_issued_ns is not None
                        or now_ns-self.start_ns > START_TIMEOUT_NS):
                    self.fail("REQUIRED_MOTOR_OR_MONITOR_GRAPH_ABSENT")
                return
            self.yaw.check_freshness(now_ns)
            self.cal.on_timeout(now_ns)
            if self.yaw.state == "FAULT_LATCHED":
                self.fail("GYRO_HEADING_STALE_OR_INVALID")
                return
            if self.cal.state == "FAULT_LATCHED":
                self.fail("AUTO_CALIBRATION_FAILED")
                return
            if self.last_heading is None or not (
                    0 <= now_ns-self.last_heading.receipt_ns <= 350_000_000):
                if self.rearm_issued_ns is not None:
                    self.fail("GYRO_HEADING_LOST")
                return
            if self.motor is None or self.latest_safety_lease is None:
                return
            if self.cal.state != "VERIFIED_STATIONARY_CANDIDATE":
                return
            if (now_ns >= self.latest_safety_lease.expires_ns
                    or self.motor.session_id != self.session_id):
                if self.rearm_issued_ns is not None:
                    self.fail("SAFETY_LEASE_STALE_OR_SESSION_CHANGED")
                return
            if self.rearm_issued_ns is None:
                if self.motor.fresh_lease(
                        self.latest_safety_lease, now_ns) is not True:
                    return
                if not self.motor.start():
                    self.fail("MOTOR_YAW_START_REJECTED")
                    return
                # The real bridge performs its OWN verified physical
                # interlock check and explicit >1s rearm dwell. This
                # controller cannot spoof an ACK of that transition.
                self.publish_signed_rearm()
                return
            # Bridge must pass the 1s rearm dwell with ZERO packets issued
            # before the first motion command; premature commands latch it.
            if now_ns-self.rearm_issued_ns < 1_150_000_000:
                return
            decision = self.motor.step(self.last_heading, now_ns)
            if decision.state in ("LATCHED", "INHIBITED"):
                self.fail("YAW_CONTROL_" + decision.reason)
                return
            # A real closed-loop response must follow nonzero command
            # within a short bound; otherwise assume bridge interlocked.
            if decision.wz_rad_s != 0 and self.motion_start_ns is None:
                self.motion_start_ns = now_ns
                self.motion_start_yaw = self.last_heading.yaw_rad
            if (self.motion_start_ns is not None
                    and abs(self.last_heading.yaw_rad
                            - self.motion_start_yaw) < math.radians(1.0)
                    and now_ns - self.motion_start_ns
                    > MAX_ACTUATION_WITHOUT_YAW_NS):
                self.fail("NO_MEASURED_MOTOR_YAW_RESPONSE")
                return
            if decision.state == "ZERO_DWELL":
                self.publish_signed_motion(0.0)
                stopped_by_monitor = (
                    self.latest_safety_lease.stopped_verified is True)
                slow_by_gyro = (
                    self.motor.tracker.observed_rate is not None
                    and abs(self.motor.tracker.observed_rate)
                    < STOP_QUIET_RAD_S)
                if (self.motor.finish_with_stop_confirmation(
                        now_ns=now_ns, stop_confirmed=(
                            stopped_by_monitor and slow_by_gyro))):
                    self.finished = True
                    print("MOTOR_YAW_CALIBRATION=PASS_STOP_CONFIRMED",
                          flush=True)
                elif now_ns - self.motor.done_at > 4_000_000_000:
                    self.fail("HARDWARE_STOP_NOT_CONFIRMED")
                return
            self.publish_signed_motion(decision.wz_rad_s)

    rclpy.init(args=[])
    node = None
    try:
        node = Controller()
        try:
            while rclpy.ok() and not node.finished and not node.failed_reason:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            node.fail("OPERATOR_INTERRUPTED")
        # Zero is sent through the same signed bridge command path, even
        # when controller aborts. Independent cutoff/watchdog still required.
        if node.rearm_issued_ns is not None:
            for _ in range(3):
                node.publish_signed_motion(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        if node.finished and not node.failed_reason:
            return 0
        print("MOTOR_YAW_CALIBRATION=BLOCKED_" +
              (node.failed_reason or "PROCESS_SHUTDOWN"), flush=True)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
