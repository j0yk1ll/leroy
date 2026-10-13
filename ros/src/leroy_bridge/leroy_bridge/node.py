import math
import re
import threading
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from leroy_msgs.msg import EncoderCounts
from leroy_bridge.odometry import integrate_yaw

import serial


_KEY_VALUE_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=(-?\d+)')
_UINT32_MODULUS = 1 << 32
_UINT32_HALF = 1 << 31


class McuClock:
    """Map wrapped MCU microsecond timestamps into the ROS clock domain."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._last_raw: Optional[int] = None
        self._wraps = 0
        self._offset_ns: Optional[int] = None

    def stamp(self, raw_us: int):
        raw_us &= 0xFFFFFFFF

        if self._last_raw is not None:
            # A large backwards jump is the normal uint32 micros() wrap.
            if raw_us < self._last_raw and (self._last_raw - raw_us) > _UINT32_HALF:
                self._wraps += 1

        self._last_raw = raw_us
        unwrapped_us = raw_us + self._wraps * _UINT32_MODULUS
        sample_ns = unwrapped_us * 1000

        if self._offset_ns is None:
            self._offset_ns = self._node.get_clock().now().nanoseconds - sample_ns

        stamp_ns = self._offset_ns + sample_ns
        sec, nsec = divmod(stamp_ns, 1_000_000_000)

        from builtin_interfaces.msg import Time

        msg = Time()
        msg.sec = int(sec)
        msg.nanosec = int(nsec)
        return msg


class LeroyBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('leroy_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_stream_hz', 100)
        self.declare_parameter('encoder_stream_hz', 50)
        self.declare_parameter('watchdog_ms', 500)
        self.declare_parameter('keepalive_period_s', 0.10)
        self.declare_parameter('cmd_vel_timeout_s', 0.25)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('wheel_radius_m', 0.0)
        self.declare_parameter('half_wheelbase_m', 0.0)
        self.declare_parameter('half_track_m', 0.0)
        self.declare_parameter('max_wheel_angular_velocity_rad_s', 0.0)
        self.declare_parameter('max_pwm', 255)

        # Encoder/odometry calibration. The right-side encoders are currently
        # unavailable, so odometry can explicitly run in a degraded mode using
        # FL + RL encoder velocities and the IMU yaw rate.
        self.declare_parameter('encoder_counts_per_rev', 2464)
        self.declare_parameter('front_left_encoder_sign', -1)
        self.declare_parameter('rear_left_encoder_sign', -1)
        self.declare_parameter('degraded_odometry_enabled', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)

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
        self._max_wheel_rad_s = float(
            self.get_parameter('max_wheel_angular_velocity_rad_s').value
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
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )

        self._serial_lock = threading.Lock()
        self._clock_mapper = McuClock(self)
        self._stop_event = threading.Event()
        self._last_cmd_vel_monotonic: Optional[float] = None
        self._motion_commanded = False
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

        self._send_command(f'set_watchdog_ms {self._watchdog_ms}')
        self._send_command(f'set_imu_stream_hz {self._imu_stream_hz}')
        self._send_command(f'set_encoder_stream_hz {self._encoder_stream_hz}')

        self._keepalive_timer = self.create_timer(
            self._keepalive_period_s, self._send_keepalive
        )
        self._cmd_timeout_timer = self.create_timer(0.05, self._check_cmd_timeout)

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

        if not self._kinematics_configured():
            self.get_logger().warning(
                'cmd_vel motor output is disabled until wheel_radius_m, '
                'half_wheelbase_m, half_track_m, and '
                'max_wheel_angular_velocity_rad_s are configured.'
            )

        self.get_logger().info(
            f'Connected to {self._serial_port}; IMU={self._imu_stream_hz} Hz, '
            f'encoders={self._encoder_stream_hz} Hz, watchdog={self._watchdog_ms} ms'
        )

    def _validate_parameters(self) -> None:
        allowed_stream_hz = {0, 10, 20, 50, 100}
        if self._imu_stream_hz not in allowed_stream_hz:
            raise ValueError('imu_stream_hz must be one of 0, 10, 20, 50, 100')
        if self._encoder_stream_hz not in allowed_stream_hz:
            raise ValueError('encoder_stream_hz must be one of 0, 10, 20, 50, 100')
        if self._watchdog_ms < 0 or self._watchdog_ms > 60000:
            raise ValueError('watchdog_ms must be in 0..60000')
        if self._keepalive_period_s <= 0.0:
            raise ValueError('keepalive_period_s must be > 0')
        if self._cmd_vel_timeout_s <= 0.0:
            raise ValueError('cmd_vel_timeout_s must be > 0')
        if self._max_pwm < 1 or self._max_pwm > 255:
            raise ValueError('max_pwm must be in 1..255')
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
            and self._max_wheel_rad_s > 0.0
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

    def _send_command(self, command: str) -> None:
        payload = (command + '\n').encode('ascii')
        try:
            with self._serial_lock:
                self._serial.write(payload)
                self._serial.flush()
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def _send_keepalive(self) -> None:
        self._send_command('keepalive')

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f'Serial read failed: {exc}')
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
            except Exception as exc:  # Keep one malformed line from killing telemetry.
                self.get_logger().warning(f'Failed to parse serial line {line!r}: {exc}')

    @staticmethod
    def _fields(line: str) -> Dict[str, int]:
        return {key: int(value) for key, value in _KEY_VALUE_RE.findall(line)}

    def _handle_line(self, line: str) -> None:
        if line.startswith('telemetry imu '):
            self._publish_imu(self._fields(line))
            return
        if line.startswith('telemetry enc '):
            self._publish_encoders(self._fields(line))
            return
        if line == 'event watchdog_stop':
            self._motion_commanded = False
            self.get_logger().warning('MCU watchdog stopped the platform')
            return
        if line.startswith('error '):
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

        self._odom_pub.publish(odom)

        if self._publish_odom_tf:
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

    def _twist_to_pwm(self, msg: Twist):
        if not self._kinematics_configured():
            return None

        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        radius = self._wheel_radius_m
        lever = self._half_wheelbase_m + self._half_track_m

        # Mecanum inverse kinematics. Signs correspond to FL, FR, RL, RR.
        wheel = [
            (vx - vy - lever * wz) / radius,
            (vx + vy + lever * wz) / radius,
            (vx + vy - lever * wz) / radius,
            (vx - vy + lever * wz) / radius,
        ]

        peak = max(abs(value) for value in wheel)
        if peak > self._max_wheel_rad_s:
            scale = self._max_wheel_rad_s / peak
            wheel = [value * scale for value in wheel]

        return [
            max(-self._max_pwm, min(self._max_pwm,
                int(round(value / self._max_wheel_rad_s * self._max_pwm))))
            for value in wheel
        ]

    def _on_cmd_vel(self, msg: Twist) -> None:
        pwm = self._twist_to_pwm(msg)
        if pwm is None:
            self.get_logger().warning(
                'Ignoring /cmd_vel because mecanum geometry is not configured',
                throttle_duration_sec=5.0,
            )
            return

        self._last_cmd_vel_monotonic = time.monotonic()
        self._motion_commanded = any(value != 0 for value in pwm)
        self._send_command(
            f'set_wheels {pwm[0]} {pwm[1]} {pwm[2]} {pwm[3]}'
        )

    def _check_cmd_timeout(self) -> None:
        if not self._motion_commanded or self._last_cmd_vel_monotonic is None:
            return
        if time.monotonic() - self._last_cmd_vel_monotonic <= self._cmd_vel_timeout_s:
            return

        self._send_command('set_wheels 0 0 0 0')
        self._motion_commanded = False
        self.get_logger().warning('cmd_vel timeout; commanded wheel stop')

    def destroy_node(self):
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
