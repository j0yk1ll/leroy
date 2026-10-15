"""DepthART diagnostic point-cloud adapter: never authorizes robot motion."""

from __future__ import annotations

from collections import OrderedDict

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import (
    Projection, StampGate, decode_depth_32fc1, depth_to_optical_points,
    select_obstacles, transform_xyz,
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def xyz_transform(tf: TransformStamped):
    t = tf.transform.translation
    q = tf.transform.rotation
    return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)


class DepthObstacles(Node):
    def __init__(self):
        super().__init__('depth_obstacles')
        defaults = {
            'depth_topic': '/depthart/depth',
            'camera_info_topic': '/camera/camera_info',
            'raw_cloud_topic': '/leroy_nav2/diagnostics/raw_depth_points',
            'obstacle_cloud_topic': '/leroy_nav2/diagnostics/obstacle_points',
            'stride': 8,
            'min_depth_m': 0.2,
            'max_depth_m': 3.0,
            'max_acquisition_age_s': 2.0,
            'max_future_skew_s': 0.1,
            'camera_info_cache_size': 512,
            'publish_obstacles': False,
            'extrinsics_validated': False,
            'ground_reference_validated': False,
            # Only meaningful AFTER measured calibration or synthetic fixture.
            'ground_z_in_base_m': 0.0,
            'min_clearance_above_ground_m': 0.08,
            'max_height_above_ground_m': 1.2,
            'max_xy_range_m': 3.0,
            'minimum_obstacle_points': 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self._gate = StampGate(
            float(self.p['max_acquisition_age_s']),
            float(self.p['max_future_skew_s'])
        )
        if self.p['camera_info_cache_size'] < 1 or self.p['stride'] < 1:
            raise ValueError('Invalid cache size or sampling stride')
        if self.p['minimum_obstacle_points'] < 1:
            raise ValueError('minimum_obstacle_points must be positive')
        if self.p['publish_obstacles'] and not (
            self.p['extrinsics_validated'] and self.p['ground_reference_validated']
        ):
            raise RuntimeError(
                'Obstacle output requires validated TF extrinsics and floor datum'
            )
        # Verify numeric geometry settings at startup, including diagnostic mode.
        Projection.from_rectified_p([1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0])
        depth_to_optical_points(
            __import__('numpy').ones((1, 1), dtype='float32'),
            Projection(1, 1, 0, 0),
            stride=int(self.p['stride']),
            min_depth_m=float(self.p['min_depth_m']),
            max_depth_m=float(self.p['max_depth_m']),
        )
        self._infos = OrderedDict()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=15))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._raw_pub = self.create_publisher(
            __import__('sensor_msgs.msg', fromlist=['PointCloud2']).PointCloud2,
            self.p['raw_cloud_topic'], qos_profile_sensor_data)
        self._obstacle_pub = self.create_publisher(
            __import__('sensor_msgs.msg', fromlist=['PointCloud2']).PointCloud2,
            self.p['obstacle_cloud_topic'], qos_profile_sensor_data)
        self._info_sub = self.create_subscription(
            CameraInfo, self.p['camera_info_topic'],
            self._on_camera_info, qos_profile_sensor_data)
        self._depth_sub = self.create_subscription(
            Image, self.p['depth_topic'],
            self._on_depth, qos_profile_sensor_data)
        self._stats = {'accepted': 0, 'obstacle_frames': 0, 'rejected': 0}
        self._last_depth_arrival_ns = None
        self._last_age_ms = None
        self.create_timer(5.0, self._report)
        self.get_logger().warning(
            'DepthART diagnostic adapter: not a safety sensor; '
            f'obstacle_output={self.p["publish_obstacles"]}. '
            'No commands or free-space clearing will be published.'
        )

    def _reject(self, reason):
        self._stats['rejected'] += 1
        self.get_logger().warning(
            f'Depth frame rejected: {reason}', throttle_duration_sec=2.0)

    def _on_camera_info(self, msg):
        key = stamp_ns(msg.header.stamp)
        if key <= 0:
            return
        self._infos[key] = msg
        self._infos.move_to_end(key)
        while len(self._infos) > self.p['camera_info_cache_size']:
            self._infos.popitem(last=False)

    def _on_depth(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self._last_depth_arrival_ns = now_ns
        acquisition_ns = stamp_ns(msg.header.stamp)
        reason = self._gate.check(acquisition_ns, now_ns)
        if reason:
            self._reject(reason)
            return
        self._last_age_ms = (now_ns - acquisition_ns) / 1e6

        info = self._infos.get(acquisition_ns)
        if info is None:
            self._reject('missing_exact_timestamp_camera_info')
            return
        if (not msg.header.frame_id or msg.header.frame_id != info.header.frame_id
                or msg.width != info.width or msg.height != info.height
                or info.binning_x not in (0, 1)
                or info.binning_y not in (0, 1)
                or info.roi.x_offset or info.roi.y_offset
                or info.roi.width or info.roi.height):
            self._reject('camera_info_frame_dimensions_or_roi_mismatch')
            return
        try:
            depth = decode_depth_32fc1(
                encoding=msg.encoding, width=msg.width, height=msg.height,
                step=msg.step, data=msg.data, is_bigendian=msg.is_bigendian)
            projection = Projection.from_rectified_p(info.p)
            optical = depth_to_optical_points(
                depth, projection,
                stride=int(self.p['stride']),
                min_depth_m=float(self.p['min_depth_m']),
                max_depth_m=float(self.p['max_depth_m']))
            if len(optical) == 0:
                self._reject('no_valid_depth_pixels')
                return
            # Deliberately preserve optical frame and ORIGINAL acquisition stamp.
            header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
            self._raw_pub.publish(create_cloud_xyz32(header, optical))
            self._stats['accepted'] += 1

            if not self.p['publish_obstacles']:
                return

            # Never substitute an identity/extrapolated/fictitious TF.
            tf = self._tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05))
            trans, quat = xyz_transform(tf)
            base_points = transform_xyz(optical, trans, quat)
            obstacles = select_obstacles(
                base_points,
                ground_z_m=float(self.p['ground_z_in_base_m']),
                min_clearance_m=float(self.p['min_clearance_above_ground_m']),
                max_height_m=float(self.p['max_height_above_ground_m']),
                max_xy_range_m=float(self.p['max_xy_range_m']))
            if len(obstacles) < self.p['minimum_obstacle_points']:
                self._reject('no_classified_obstacles')
                return
            self._obstacle_pub.publish(create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id='base_link'),
                obstacles))
            self._stats['obstacle_frames'] += 1
        except (ValueError, TransformException) as exc:
            self._reject(f'{type(exc).__name__}: {exc}')

    def _report(self):
        now_ns = self.get_clock().now().nanoseconds
        stale = (self._last_depth_arrival_ns is None
                 or (now_ns - self._last_depth_arrival_ns)
                 > round(float(self.p['max_acquisition_age_s']) * 1e9))
        self.get_logger().info(
            f'depth input_stale={stale} '
            f'acquisition_age_ms={self._last_age_ms} '
            f'accepted={self._stats["accepted"]} '
            f'obstacle_frames={self._stats["obstacle_frames"]} '
            f'rejected={self._stats["rejected"]}; '
            'STALE/INVALID INPUT NEVER CLEARS COSTMAP')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthObstacles()
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
