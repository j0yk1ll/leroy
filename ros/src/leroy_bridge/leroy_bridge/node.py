import math
import re
import threading
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from leroy_msgs.msg import EncoderCounts
from leroy_bridge.odometry import integrate_yaw
from leroy_bridge.clock_sync import ClockSyncError, McuClockSync
from leroy_bridge.command_ingress import command_to_bytes, CommandRejected
from leroy_bridge.bridge_signed_audit import BridgeSignedAudit
from leroy_bridge.bridge_diagnostic_supervisor import BridgeDiagnosticSupervisor
from leroy_bridge.authorized_session import (
    MotionEnvelope, RearmEnvelope, SessionVerifier,
)
from leroy_bridge.motor_authority import MotorAuthority, HEALTH_KEYS
from leroy_bridge.motor_cdc_protocol import MotorCdcProtocol
from leroy_bridge.motor_sensor_health import MotorSensorHealth
from leroy_bridge.motor_gpio_feedback import PhysicalMotorFeedback
from leroy_bridge.motor_signed_ingress import (
    MotorInputRejected, parse_signed_envelope, read_protected_key,
)

import serial


_MOTOR_STOP = object()
_KEY_VALUE_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=(-?\d+)')
class McuClock:
    """Wrap the hardware-clock estimator with the ROS clock and message type."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._sync = McuClockSync()
        self._last_diagnostic_ns: Optional[int] = None

    def stamp(self, raw_us: int):
        # Sample both clocks together for step detection. One-way USB serial
        # latency still prevents verification of absolute acquisition time.
        steady_ns = time.monotonic_ns()
        ros_ns = self._node.get_clock().now().nanoseconds
        stamp_ns = self._sync.observe(raw_us, ros_ns, steady_ns)

        if self._sync.ready and (
            self._last_diagnostic_ns is None
            or steady_ns - self._last_diagnostic_ns >= 60_000_000_000
        ):
            self._last_diagnostic_ns = steady_ns
            self._node.get_logger().info(
                f'MCU clock-rate estimate: {self._sync.rate_ppm:+.1f} ppm '
                '(host relative to MCU; absolute USB delay not calibrated)'
            )

        from builtin_interfaces.msg import Time

        msg = Time()
        msg.sec, msg.nanosec = divmod(stamp_ns, 1_000_000_000)
        return msg


class LeroyBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('leroy_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_stream_hz', 100)
        self.declare_parameter('encoder_stream_hz', 50)
        self.declare_parameter('watchdog_ms', 500,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('keepalive_period_s', 0.10,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('cmd_vel_timeout_s', 0.25,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('wheel_radius_m', 0.0,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('half_wheelbase_m', 0.0,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('half_track_m', 0.0,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('wheel_velocity_to_pwm_gain', 0.0,
                               descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter('max_pwm', 255,
                               descriptor=ParameterDescriptor(read_only=True))
        # Both role keys must come from protected files. Empty-by-default
        # means no signed actuator ROS ingress or signing session at all.
        self.declare_parameter(
            'motor_motion_key_file', '',
            descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter(
            'motor_operator_key_file', '',
            descriptor=ParameterDescriptor(read_only=True))
        # Read-only FEEDBACK from an independent external motor cutoff.
        # No GPIO line IDs are invented: unconfigured means no motor release.
        self.declare_parameter(
            'motor_gpio_chip', '',
            descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter(
            'motor_estop_feedback_offset', -1,
            descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter(
            'motor_power_feedback_offset', -1,
            descriptor=ParameterDescriptor(read_only=True))

        # Encoder/odometry calibration. The right-side encoders are currently
        # unavailable, so odometry can explicitly run in a degraded mode using
        # FL + RL encoder velocities and the IMU yaw rate.
        self.declare_parameter('encoder_counts_per_rev', 2464)
        self.declare_parameter('front_left_encoder_sign', -1)
        self.declare_parameter('rear_left_encoder_sign', -1)
        self.declare_parameter('degraded_odometry_enabled', False)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', False)

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._imu_stream_hz = int(self.get_parameter('imu_stream_hz').value)
        self._encoder_stream_hz = int(self.get_parameter('encoder_stream_hz').value)
        self._watchdog_ms = int(self.get_parameter('watchdog_ms').value)
        self._keepalive_period_s = float(self.get_parameter('keepalive_period_s').value)
        self._cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)
        self._imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self._wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self._half_wheelbase_m = float(self.get_parameter('half_wheelbase_m').value)
        self._half_track_m = float(self.get_parameter('half_track_m').value)
        self._wheel_velocity_to_pwm_gain = float(
            self.get_parameter('wheel_velocity_to_pwm_gain').value
        )
        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._encoder_counts_per_rev = int(
            self.get_parameter('encoder_counts_per_rev').value
        )
        self._front_left_encoder_sign = int(
            self.get_parameter('front_left_encoder_sign').value
        )
        self._rear_left_encoder_sign = int(
            self.get_parameter('rear_left_encoder_sign').value
        )
        self._degraded_odometry_enabled = bool(
            self.get_parameter('degraded_odometry_enabled').value
        )
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)

        self._validate_parameters()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._imu_pub = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self._encoder_pub = self.create_publisher(
            EncoderCounts, '/wheel/encoder_counts', sensor_qos
        )
        # Opt-in diagnostic fallback only. With XRSLAM providing odometry, do
        # not even advertise a competing /odom publisher or TF broadcaster.
        self._odom_pub = (
            self.create_publisher(Odometry, '/odom', 10)
            if self._degraded_odometry_enabled else None
        )
        self._tf_broadcaster = (
            TransformBroadcaster(self)
            if self._degraded_odometry_enabled and self._publish_odom_tf else None
        )
        # M22D deliberately has NO raw /cmd_vel subscription. The new
        # signed-only motion/rearm interfaces are provisioned separately;
        # without independent physical readiness they cannot reach CDC.

        # The signed MotorAuthority handoff is now recognized by the sole
        # CDC writer. This deployment has no trusted hardware interlock
        # provider or key custody yet; production physical release remains
        # structurally unavailable (not a user-togglable ROS parameter).
        self._motor_authority = None
        self._motor_cdc = None
        self._motor_stop_requested = False
        self._motor_feedback = None
        self._motor_health = None
        self._motor_hardware_release_verified = False
        self._serial_lock = threading.Lock()
        self._transport_fault = threading.Event()
        self._clock_mapper = McuClock(self)
        self._clock_fault = False
        self._stop_event = threading.Event()
        # Firmware boots with watchdog DISABLED. Trust only MCU ACK.
        self._motor_watchdog_ack = threading.Event()
        # Validate optional signer custody BEFORE opening real MCU CDC.
        self._configure_signed_motor_ingress()
        self._last_imu_seq: Optional[int] = None
        self._last_encoder_seq: Optional[int] = None

        # Degraded odometry state. Raw encoder telemetry remains unchanged;
        # these values are used only by the odometry estimator.
        self._last_fl_count: Optional[int] = None
        self._last_rl_count: Optional[int] = None
        self._last_encoder_t_us: Optional[int] = None
        self._last_imu_t_us: Optional[int] = None
        self._last_gyro_z_rad_s: Optional[float] = None
        self._yaw = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0

        self._serial = serial.Serial(
            port=self._serial_port,
            baudrate=self._baudrate,
            timeout=0.10,
            write_timeout=0.10,
        )

        # USB CDC devices commonly reset/re-enumerate when opened. Give the
        # firmware a short quiet period before configuring streams.
        time.sleep(0.20)
        self._serial.reset_input_buffer()

        self._reader = threading.Thread(
            target=self._reader_loop,
            name='leroy-serial-reader',
            daemon=True,
        )
        self._reader.start()

        watchdog_write_ok = self._send_command(
            f'set_watchdog_ms {self._watchdog_ms}')
        if not watchdog_write_ok and self._motor_authority is not None:
            self._motor_authority.stop()
        self._send_command(f'set_imu_stream_hz {self._imu_stream_hz}')
        self._send_command(f'set_encoder_stream_hz {self._encoder_stream_hz}')

        self._keepalive_timer = self.create_timer(
            self._keepalive_period_s, self._send_keepalive
        )
        if self._motor_authority is not None:
            # A single 20 Hz bridge-owned authority evaluator.
            self._motor_timer = self.create_timer(0.05, self._motor_tick)

        if self._publish_odom_tf and not self._degraded_odometry_enabled:
            self.get_logger().warning(
                'publish_odom_tf is ignored while degraded odometry is disabled.'
            )

        if self._degraded_odometry_enabled and not self._odometry_configured():
            self.get_logger().warning(
                'Degraded odometry is enabled but wheel geometry / encoder calibration '
                'is incomplete; /odom will not be published.'
            )
        elif self._degraded_odometry_enabled:
            self.get_logger().warning(
                'Using degraded odometry: FL + RL encoders with IMU yaw rate; '
                'right encoder telemetry is not synthesized.'
            )

        # No geometry, PWM gain, ROS remap or runtime parameter can
        # enable motion under the M22D CDC allowlist.
        self.get_logger().warning(
            'M22D: raw /cmd_vel disabled; arbitrary string commands '
            'remain ZERO-wheel only. Signed actuator topics are off '
            'unless protected role keys are provisioned; independent '
            'physical-interlock release is NOT implemented.'
        )

        self.get_logger().info(
            f'Connected to {self._serial_port}; IMU={self._imu_stream_hz} Hz, '
            f'encoders={self._encoder_stream_hz} Hz, watchdog={self._watchdog_ms} ms'
        )

    def _configure_signed_motor_ingress(self) -> None:
        """Accept signed ROS messages only with two protected local keys.

        These inputs are NOT motor-power or E-stop assertions. The physical
        motor release is deliberately false until a real independent
        hardware interlock provider and sensor readiness exist.
        """
        motion_path = self.get_parameter('motor_motion_key_file').value
        operator_path = self.get_parameter('motor_operator_key_file').value
        if type(motion_path) is not str or type(operator_path) is not str:
            raise ValueError('motor signing key paths must be strings')
        if bool(motion_path) != bool(operator_path):
            raise ValueError('both motor signing roles must be provisioned')
        self._pending_motor_motion = None
        self._motor_last_state = 'LATCHED'
        if not motion_path:
            return
        if not self._kinematics_configured():
            raise ValueError('motor authority requires configured geometry')
        # A signed motor session must configure a short MCU timeout.
        if not 1 <= self._watchdog_ms <= 500:
            raise ValueError('signed motor authority requires watchdog <=500 ms')
        if self._keepalive_period_s > 0.10:
            raise ValueError('signed motor authority requires keepalive <=100 ms')
        motion_key = read_protected_key(motion_path)
        operator_key = read_protected_key(operator_path)
        verifier = SessionVerifier(
            motion_key=motion_key, operator_key=operator_key)
        self._motor_authority = MotorAuthority(
            verifier=verifier,
            wheel_radius_m=float(self._wheel_radius_m),
            half_wheelbase_m=float(self._half_wheelbase_m),
            half_track_m=float(self._half_track_m),
            pwm_per_rad_s=float(self._wheel_velocity_to_pwm_gain),
            max_pwm=int(self._max_pwm))
        self._motor_cdc = MotorCdcProtocol()
        self._motor_health = MotorSensorHealth()
        chip = self.get_parameter('motor_gpio_chip').value
        estop_offset = self.get_parameter(
            'motor_estop_feedback_offset').value
        power_offset = self.get_parameter(
            'motor_power_feedback_offset').value
        if chip or estop_offset != -1 or power_offset != -1:
            self._motor_feedback = PhysicalMotorFeedback(
                chip, estop_offset, power_offset)
        # Only real, typed source messages can advance a motor health gate.
        observer_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self._motor_status_sub = self.create_subscription(
            String, '/xrslam/tracking_status',
            self._motor_health.tracking_state, status_qos)
        self._motor_valid_sub = self.create_subscription(
            Bool, '/xrslam/localization_valid',
            self._motor_health.localization_state, status_qos)
        self._motor_odom_sub = self.create_subscription(
            Odometry, '/odom', self._motor_health.odom, observer_qos)
        self._motor_depth_sub = self.create_subscription(
            Image, '/depthart/depth', self._motor_health.depth, observer_qos)
        self._motor_tf_sub = self.create_subscription(
            TFMessage, '/tf', self._motor_health.dynamic_tf, observer_qos)
        self._motor_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._motor_health.costmap, observer_qos)
        self._motor_session_pub = self.create_publisher(
            String, '/leroy/motor/session_id',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._motor_session_pub.publish(
            String(data=verifier.session_id))
        self._motor_motion_sub = self.create_subscription(
            String, '/leroy/motor/signed_motion',
            self._on_signed_motor_motion, 10)
        self._motor_rearm_sub = self.create_subscription(
            String, '/leroy/motor/signed_rearm',
            self._on_signed_motor_rearm, 10)
        self.get_logger().warning(
            'Signed motion ingress configured, but physical motor '
            'interlock and validated sensor health are still unavailable; '
            'nonzero serial release stays inhibited.')

    def _motor_trusted_inputs(self):
        """Sample local GPIO feedback and measured sensor/nav messages.

        The GPIO inputs MUST come from an independent hardwired motor power
        cutoff circuit. Software cannot substitute for that circuit.
        ROS graph counts are collision checks, not authentication; signed
        envelopes separately enforce session ownership.
        """
        estop, power = (False, False)
        if self._motor_feedback is not None:
            estop, power = self._motor_feedback.read()
        health = (self._motor_health.state()
                  if self._motor_health is not None
                  else {key: False for key in HEALTH_KEYS})
        source_counts_ok = (
            self._motor_odom_sub.get_publisher_count() == 1
            and self._motor_depth_sub.get_publisher_count() == 1
            and self._motor_costmap_sub.get_publisher_count() == 1
            and self._motor_valid_sub.get_publisher_count() == 1
        ) if self._motor_health is not None else False
        controller_ok = (
            self._motor_motion_sub.get_publisher_count() == 1
            if hasattr(self, '_motor_motion_sub') else False
        )
        if not source_counts_ok:
            health = {key: False for key in HEALTH_KEYS}
        transport_ok = (
            not self._transport_fault.is_set()
            and self._motor_watchdog_ack.is_set()
            and self._motor_cdc is not None
            and self._motor_cdc.healthy)
        # This software release flag is advisory; the power cutoff itself
        # operates independently of Linux, ROS and the MCU.
        self._motor_hardware_release_verified = bool(
            estop and power and source_counts_ok and controller_ok
            and all(health.values()) and transport_ok
            and not self._clock_fault and not self._stop_event.is_set())
        return dict(
            health=health,
            estop_released=estop,
            motor_rails_verified=power,
            transport_healthy=transport_ok,
            clock_healthy=not self._clock_fault,
            bridge_running=not self._stop_event.is_set(),
            single_controller=controller_ok,
        )

    def _on_signed_motor_motion(self, message):
        if self._motor_authority is None:
            return
        try:
            self._pending_motor_motion = parse_signed_envelope(
                message.data, MotionEnvelope)
        except MotorInputRejected as exc:
            self._pending_motor_motion = None
            self._motor_authority.stop()
            self.get_logger().error(
                f'Signed motor proposal rejected and authority revoked: {exc}')

    def _on_signed_motor_rearm(self, message):
        if self._motor_authority is None:
            return
        try:
            grant = parse_signed_envelope(message.data, RearmEnvelope)
        except MotorInputRejected as exc:
            self._motor_authority.stop()
            self.get_logger().error(
                f'Signed operator grant malformed; authority revoked: {exc}')
            return
        decision = self._motor_authority.rearm(
            grant=grant, now_ns=time.monotonic_ns(),
            **self._motor_trusted_inputs())
        self.get_logger().info(
            f'Motor rearm result: {decision.state} {decision.reason}')

    def _motor_tick(self):
        if self._motor_authority is None:
            return
        if self._motor_cdc.expired(now_ns=time.monotonic_ns()):
            self._motor_authority.stop()
        motion = self._pending_motor_motion
        self._pending_motor_motion = None
        decision = self._motor_authority.step(
            motion=motion, now_ns=time.monotonic_ns(),
            **self._motor_trusted_inputs())
        if decision.nonzero_candidate:
            if not self._send_command(decision):
                self._motor_authority.stop()
        if decision.state == 'ARMED':
            self._motor_stop_requested = False
        if (self._motor_last_state == 'ARMED'
                and decision.state in ('LATCHED', 'TERMINAL')):
            if not self._motor_stop_requested:
                self._motor_stop_requested = True
                # Independent MCU stop request; the reader must correlate its
                # ACK, otherwise the session stays terminally inhibited.
                if not self._send_command(_MOTOR_STOP):
                    self._motor_authority.stop()
        self._motor_last_state = decision.state

    def _validate_parameters(self) -> None:
        allowed_stream_hz = {0, 10, 20, 50, 100}
        if self._imu_stream_hz not in allowed_stream_hz:
            raise ValueError('imu_stream_hz must be one of 0, 10, 20, 50, 100')
        if self._encoder_stream_hz not in allowed_stream_hz:
            raise ValueError('encoder_stream_hz must be one of 0, 10, 20, 50, 100')
        if self._watchdog_ms < 1 or self._watchdog_ms > 60000:
            raise ValueError('watchdog_ms must be in 1..60000 during M22D')
        if self._keepalive_period_s <= 0.0:
            raise ValueError('keepalive_period_s must be > 0')
        if self._cmd_vel_timeout_s <= 0.0:
            raise ValueError('cmd_vel_timeout_s must be > 0')
        if self._max_pwm < 1 or self._max_pwm > 255:
            raise ValueError('max_pwm must be in 1..255')
        if not math.isfinite(self._wheel_velocity_to_pwm_gain) or self._wheel_velocity_to_pwm_gain < 0.0:
            raise ValueError('wheel_velocity_to_pwm_gain must be finite and >= 0')
        if self._encoder_counts_per_rev <= 0:
            raise ValueError('encoder_counts_per_rev must be > 0')
        if self._front_left_encoder_sign not in (-1, 1):
            raise ValueError('front_left_encoder_sign must be -1 or 1')
        if self._rear_left_encoder_sign not in (-1, 1):
            raise ValueError('rear_left_encoder_sign must be -1 or 1')

    def _kinematics_configured(self) -> bool:
        return (
            self._wheel_radius_m > 0.0
            and self._half_wheelbase_m > 0.0
            and self._half_track_m > 0.0
            and self._wheel_velocity_to_pwm_gain > 0.0
        )

    def _odometry_configured(self) -> bool:
        return (
            self._wheel_radius_m > 0.0
            and self._half_wheelbase_m > 0.0
            and self._half_track_m > 0.0
            and self._encoder_counts_per_rev > 0
        )

    @staticmethod
    def _uint32_delta_us(current: int, previous: int) -> int:
        return ((current & 0xFFFFFFFF) - (previous & 0xFFFFFFFF)) & 0xFFFFFFFF

    @staticmethod
    def _int32_delta(current: int, previous: int) -> int:
        # Return the shortest signed delta, including across int32 wrap.
        return ((current - previous + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)

    @staticmethod
    def _yaw_quaternion(yaw: float):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _inspect_signed_diagnostic(self, audit, *, motion, rearm=None,
                                   now_ns):
        # M22J is not a ROS subscriber or motor ingress. This method
        # reports HMAC/session verification ONLY and deliberately never
        # calls _send_command, motor PWM conversion or any serial method.
        # In production no verifier or audit is constructed by this node.
        if type(audit) is not BridgeSignedAudit:
            raise TypeError('M22J requires an exact bridge diagnostic auditor')
        return audit.inspect(motion=motion, rearm=rearm, now_ns=now_ns)

    def _supervise_signed_diagnostic(
            self, supervisor, *, motion, rearm=None, now_ns,
            health, publishers):
        # M22K: diagnostic-only, no ROS command subscriber/serial write.
        # Real local clock/CDC failures override all synthetic claims.
        # The caller cannot override these bridge-owned fault flags.
        if type(supervisor) is not BridgeDiagnosticSupervisor:
            raise TypeError('M22K requires exact diagnostic supervisor')
        return supervisor.evaluate(
            motion=motion, rearm=rearm, now_ns=now_ns,
            health=health, publishers=publishers,
            transport_healthy=not self._transport_fault.is_set(),
            clock_healthy=not self._clock_fault,
            bridge_running=not self._stop_event.is_set(),
        )

    def _send_command(self, command: str) -> bool:
        # Raw strings ALWAYS take the unchanged M22D zero-only allowlist.
        # A non-string command must be the exact identity-bound one-shot
        # MotorAuthority decision, with independent verified hardware
        # release. No ROS message or arbitrary PWM string can bypass it.
        authority = None
        is_stop = False
        try:
            if type(command) is str:
                # No later in-process call may silently lengthen or
                # disable a watchdog protecting an active signed session.
                if (getattr(self, '_motor_authority', None) is not None
                        and command.startswith('set_watchdog_ms ')
                        and command != f'set_watchdog_ms {self._watchdog_ms}'):
                    raise CommandRejected(
                        'cannot weaken signed-session MCU watchdog')
                if getattr(self, '_motor_authority', None) is not None:
                    if (command == 'keepalive'
                            and (not self._motor_watchdog_ack.is_set()
                                 or not self._motor_cdc.healthy
                                 or not self._motor_authority.heartbeat_allowed(
                                     now_ns=time.monotonic_ns()))):
                        raise CommandRejected('signed motion heartbeat stale')
                    if (command.startswith('set_watchdog_ms ')
                            and self._motor_watchdog_ack.is_set()):
                        raise CommandRejected(
                            'signed session MCU watchdog is already configured')
                payload = command_to_bytes(command)
            elif command is _MOTOR_STOP:
                # STOP may be transmitted after the motion session latches.
                is_stop = True
                if self._motor_cdc is None:
                    raise CommandRejected('MCU stop protocol unavailable')
                payload = None
            else:
                authority = getattr(self, '_motor_authority', None)
                release = getattr(
                    self, '_motor_hardware_release_verified', False)
                if authority is None or release is not True:
                    raise CommandRejected(
                        'physical motor authority release unavailable')
                # Consume only inside the serial lock, immediately before
                # the one actual .write(), to avoid stale queued duties.
                payload = None
        except CommandRejected as exc:
            self.get_logger().error(f'Motor CDC command rejected: {exc}')
            return False
        # M22G: once a serial write/flush/short-write/read error occurs,
        # this process cannot resume CDC writes, even for a STOP packet.
        # No physical stopping or MCU acknowledgement is asserted here.
        if self._transport_fault.is_set():
            return False
        with self._serial_lock:
            if self._transport_fault.is_set():
                return False
            if (type(command) is str and command == 'keepalive'
                    and self._motor_authority is not None
                    and (not self._motor_cdc.healthy
                         or not self._motor_authority.heartbeat_allowed(
                             now_ns=time.monotonic_ns()))):
                return False
            if is_stop:
                payload = self._motor_cdc.stop(now_ns=time.monotonic_ns())
            elif authority is not None:
                # Recheck the independent physical feedback at the final
                # locked writer boundary. Old software readiness booleans
                # cannot authorize transmission after power loss / E-stop.
                if (self._motor_feedback is None
                        or self._motor_feedback.read() != (True, True)
                        or not self._motor_hardware_release_verified):
                    authority.stop()
                    return False
                # The signed decision alone cannot supply arbitrary serial
                # bytes. Its identity-bound one-time payload must match the
                # exact PWM tuple before the MCU transaction is constructed.
                candidate = authority.consume_serial_payload(
                    command, now_ns=time.monotonic_ns())
                if candidate == (command.packet + '\n').encode('ascii'):
                    payload = self._motor_cdc.wheels(
                        command.wheel_pwm, now_ns=time.monotonic_ns())
            if payload is None:
                self.get_logger().error(
                    'Motor CDC packet blocked: invalid/spent decision, '
                    'unacknowledged transaction or exhausted sequence')
                return False
            try:
                written = self._serial.write(payload)
                if written != len(payload):
                    raise serial.SerialException(
                        f'short CDC write: {written}/{len(payload)} bytes')
                self._serial.flush()
            except (serial.SerialException, serial.SerialTimeoutException,
                    OSError) as exc:
                self._transport_fault.set()
                self._stop_event.set()
                if getattr(self, '_motor_authority', None) is not None:
                    self._motor_authority.stop()
                self.get_logger().error(
                    f'Terminal MCU transport fault; CDC inhibited until '
                    f'bridge restart: {exc}'
                )
                return False
        return True

    def _send_keepalive(self) -> None:
        # MCU watchdog accepts any keepalive, even when control is stale.
        # Once a motor authority exists, only a fresh signed ARMED session
        # can extend motion. The keyless M22D telemetry bridge is unchanged.
        if (self._clock_fault or self._transport_fault.is_set()
                or self._stop_event.is_set()):
            return
        authority = getattr(self, '_motor_authority', None)
        if authority is not None and (
                not self._motor_watchdog_ack.is_set()
                or self._motor_cdc.expired(now_ns=time.monotonic_ns())
                or not authority.heartbeat_allowed(
                    now_ns=time.monotonic_ns())):
            return
        self._send_command('keepalive')

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline()
            except (serial.SerialException, serial.SerialTimeoutException,
                    OSError) as exc:
                if not self._stop_event.is_set():
                    # Serialize the fault transition with the sole CDC
                    # writer; no subsequent write may pass the lock guard.
                    with self._serial_lock:
                        self._transport_fault.set()
                        self._stop_event.set()
                        if getattr(self, '_motor_authority', None) is not None:
                            self._motor_authority.stop()
                    self.get_logger().error(
                        f'Terminal MCU read fault; CDC inhibited until '
                        f'bridge restart: {exc}'
                    )
                return

            if self._stop_event.is_set() or not rclpy.ok():
                return
            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='strict').strip()
            except UnicodeDecodeError:
                self.get_logger().warning('Discarding non-ASCII serial line')
                continue

            if not line:
                continue

            try:
                self._handle_line(line)
            except ClockSyncError as exc:
                # Fail closed. Never silently re-anchor a rebooted/stepped
                # sensor clock into the same VIO or odometry session.
                with self._serial_lock:
                    self._clock_fault = True
                    self._stop_event.set()
                    if getattr(self, '_motor_authority', None) is not None:
                        self._motor_authority.stop()
                self._send_command('set_wheels 0 0 0 0')
                self.get_logger().error(
                    f'MCU clock synchronization fault: {exc}; '
                    'telemetry inhibited until supervised bridge restart'
                )
                return
            except Exception as exc:  # Keep one malformed line from killing telemetry.
                if rclpy.ok() and not self._stop_event.is_set():
                    self.get_logger().warning(f'Failed to parse serial line {line!r}: {exc}')

    @staticmethod
    def _fields(line: str) -> Dict[str, int]:
        return {key: int(value) for key, value in _KEY_VALUE_RE.findall(line)}

    def _handle_line(self, line: str) -> None:
        if self._motor_cdc is not None and self._motor_cdc.receive(
                line, now_ns=time.monotonic_ns()):
            if not self._motor_cdc.healthy:
                with self._serial_lock:
                    self._motor_authority.stop()
            return
        if line == 'ok set_watchdog_ms':
            # MCU accepted the timeout; this is NOT a stop acknowledgement.
            self._motor_watchdog_ack.set()
            return
        if line.startswith('telemetry imu '):
            self._publish_imu(self._fields(line))
            return
        if line.startswith('telemetry enc '):
            self._publish_encoders(self._fields(line))
            return
        if line == 'event watchdog_stop':
            if self._motor_authority is not None:
                with self._serial_lock:
                    self._motor_authority.stop()
                    self._motor_cdc.abort('MCU_WATCHDOG_STOP')
            self.get_logger().warning(
                'MCU reported watchdog stop; signed motor session revoked')
            return
        if line.startswith('error '):
            if (self._motor_authority is not None
                    and (line.startswith('error set_wheels_tx')
                         or line.startswith('error stop_wheels_tx'))):
                with self._serial_lock:
                    self._motor_authority.stop()
                    self._motor_cdc.abort('MCU_REJECTED_MOTOR_TRANSACTION')
            self.get_logger().error(f'MCU: {line}')
            return
        # Command acknowledgements are intentionally quiet; they would be very
        # noisy because keepalive produces an acknowledgement each period.

    def _check_sequence(self, kind: str, sequence: int, previous: Optional[int]) -> None:
        if previous is None:
            return
        expected = (previous + 1) & 0xFFFFFFFF
        if sequence != expected:
            lost = (sequence - expected) & 0xFFFFFFFF
            self.get_logger().warning(
                f'{kind} telemetry sequence gap: expected {expected}, got {sequence} '
                f'(~{lost} sample(s) skipped)'
            )

    def _publish_imu(self, fields: Dict[str, int]) -> None:
        required = ('seq', 't_us', 'af_mm_s2', 'al_mm_s2', 'au_mm_s2',
                    'gr_mrad_s', 'gp_mrad_s', 'gy_mrad_s')
        if any(name not in fields for name in required):
            raise ValueError('missing IMU field')

        seq = fields['seq'] & 0xFFFFFFFF
        self._check_sequence('IMU', seq, self._last_imu_seq)
        self._last_imu_seq = seq

        msg = Imu()
        msg.header.stamp = self._clock_mapper.stamp(fields['t_us'])
        msg.header.frame_id = self._imu_frame_id

        # Firmware already reports robot axes: +x forward, +y left, +z up.
        msg.linear_acceleration.x = fields['af_mm_s2'] / 1000.0
        msg.linear_acceleration.y = fields['al_mm_s2'] / 1000.0
        msg.linear_acceleration.z = fields['au_mm_s2'] / 1000.0
        msg.angular_velocity.x = fields['gr_mrad_s'] / 1000.0
        msg.angular_velocity.y = fields['gp_mrad_s'] / 1000.0
        msg.angular_velocity.z = fields['gy_mrad_s'] / 1000.0

        # Integrate yaw directly from the MCU-timestamped gyro stream. This is
        # deliberately only used for temporary/degraded wheel odometry; the raw
        # IMU message still reports orientation as unavailable.
        raw_t_us = fields['t_us'] & 0xFFFFFFFF
        wz = msg.angular_velocity.z
        if self._last_imu_t_us is not None:
            delta_us = self._uint32_delta_us(raw_t_us, self._last_imu_t_us)
            dt = delta_us * 1e-6
            if 0.0 < dt <= 0.1:
                self._yaw = integrate_yaw(
                    self._yaw, self._last_gyro_z_rad_s, wz, dt
                )
        self._last_imu_t_us = raw_t_us
        self._last_gyro_z_rad_s = wz

        # No fused orientation is supplied by the MCU.
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    def _publish_encoders(self, fields: Dict[str, int]) -> None:
        required = ('seq', 't_us', 'fl', 'fr', 'rl', 'rr')
        if any(name not in fields for name in required):
            raise ValueError('missing encoder field')

        seq = fields['seq'] & 0xFFFFFFFF
        self._check_sequence('encoder', seq, self._last_encoder_seq)
        self._last_encoder_seq = seq

        msg = EncoderCounts()
        msg.header.stamp = self._clock_mapper.stamp(fields['t_us'])
        msg.header.frame_id = self._base_frame_id
        msg.sequence = seq
        msg.front_left = fields['fl']
        msg.front_right = fields['fr']
        msg.rear_left = fields['rl']
        msg.rear_right = fields['rr']
        self._encoder_pub.publish(msg)

        if self._degraded_odometry_enabled:
            self._update_degraded_odometry(fields, msg.header.stamp)

    def _update_degraded_odometry(self, fields: Dict[str, int], stamp) -> None:
        if not self._odometry_configured():
            return

        fl = fields['fl']
        rl = fields['rl']
        raw_t_us = fields['t_us'] & 0xFFFFFFFF

        if (
            self._last_fl_count is None
            or self._last_rl_count is None
            or self._last_encoder_t_us is None
        ):
            self._last_fl_count = fl
            self._last_rl_count = rl
            self._last_encoder_t_us = raw_t_us
            return

        delta_us = self._uint32_delta_us(raw_t_us, self._last_encoder_t_us)
        dt = delta_us * 1e-6
        delta_fl = self._int32_delta(fl, self._last_fl_count)
        delta_rl = self._int32_delta(rl, self._last_rl_count)

        self._last_fl_count = fl
        self._last_rl_count = rl
        self._last_encoder_t_us = raw_t_us

        # Reject implausible timing gaps rather than converting them into a
        # large velocity spike after startup, disconnect, or telemetry loss.
        if dt <= 0.0 or dt > 0.2:
            return

        radians_per_count = 2.0 * math.pi / self._encoder_counts_per_rev
        omega_fl = (
            self._front_left_encoder_sign * delta_fl * radians_per_count / dt
        )
        omega_rl = (
            self._rear_left_encoder_sign * delta_rl * radians_per_count / dt
        )

        # Temporary mecanum estimator using only the two left wheels plus IMU
        # yaw rate. From the normal mecanum inverse equations:
        #   vx = r/2 * (w_fl + w_rl) + (Lx + Ly) * wz
        #   vy = r/2 * (w_rl - w_fl)
        # Right encoder readings remain raw/unmodified on /wheel/encoder_counts.
        wz = self._last_gyro_z_rad_s if self._last_gyro_z_rad_s is not None else 0.0
        lever = self._half_wheelbase_m + self._half_track_m
        vx = 0.5 * self._wheel_radius_m * (omega_fl + omega_rl) + lever * wz
        vy = 0.5 * self._wheel_radius_m * (omega_rl - omega_fl)

        # Integrate translation in the odom frame using the current gyro yaw.
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        self._odom_x += (cos_yaw * vx - sin_yaw * vy) * dt
        self._odom_y += (sin_yaw * vx + cos_yaw * vy) * dt

        qx, qy, qz, qw = self._yaw_quaternion(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Deliberately conservative because two wheel encoders are missing and
        # mecanum lateral motion is slip-sensitive. Values are variances.
        odom.pose.covariance[0] = 0.05 ** 2      # x
        odom.pose.covariance[7] = 0.10 ** 2      # y
        odom.pose.covariance[14] = 1e6           # z unused
        odom.pose.covariance[21] = 1e6           # roll unused
        odom.pose.covariance[28] = 1e6           # pitch unused
        odom.pose.covariance[35] = 0.10 ** 2      # yaw
        odom.twist.covariance[0] = 0.10 ** 2
        odom.twist.covariance[7] = 0.20 ** 2
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.10 ** 2

        if self._odom_pub is not None:
            self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame_id
            transform.child_frame_id = self._base_frame_id
            transform.transform.translation.x = self._odom_x
            transform.transform.translation.y = self._odom_y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        # Revoke any outstanding motor decision before serial teardown.
        if getattr(self, '_motor_authority', None) is not None:
            self._motor_authority.stop()
            # Request MCU driver disable, retaining the reader briefly to
            # collect its sequence-matched stop ACK. Timeout is NOT a stop.
            if not self._transport_fault.is_set():
                if self._send_command(_MOTOR_STOP):
                    end = time.monotonic() + 0.15
                    while (time.monotonic() < end and
                           not self._motor_cdc.stop_confirmed):
                        time.sleep(0.005)
                if not self._motor_cdc.stop_confirmed:
                    self.get_logger().error(
                        'MCU motor STOP unconfirmed; independent cutoff required')
        # Stop asynchronous telemetry before ROS context teardown.
        # After a transport fault the zero/stream-disable writes below
        # are intentionally refused by _send_command. This does NOT
        # establish a physical stop and relies on independent protection.
        self._stop_event.set()
        try:
            self._send_command('set_wheels 0 0 0 0')
            self._send_command('set_imu_stream_hz 0')
            self._send_command('set_encoder_stream_hz 0')
        finally:
            self._stop_event.set()
            if hasattr(self, '_reader'):
                self._reader.join(timeout=1.0)
            if hasattr(self, '_serial') and self._serial.is_open:
                self._serial.close()
            if self._motor_feedback is not None:
                self._motor_feedback.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LeroyBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
