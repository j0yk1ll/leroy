"""M4 isolated end-to-end diagnostic probe; no subscriptions to commands."""

import math
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from .probe_math import spans, tracks_robot


def _stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class IntegratedProbe(Node):
    def __init__(self):
        super().__init__('integrated_probe')
        self.last_odom_ns = 0
        self.latest_pose = None
        self.odom_by_stamp = {}
        self.tf_by_stamp = {}
        self.odom_tf_matches = 0
        self.x_samples = []
        self.y_samples = []
        self.yaw_samples = []
        self.origins_x = []
        self.origins_y = []
        self.center_matches = 0
        self.lethal_and_unknown = False
        self.raw_stamps = set()
        self.obstacle_stamps = set()
        self.cloud_pairs = 0
        self.stopped = False
        self.forward = False
        self.strafe = False
        self.turn = False

        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/costmap', self._on_map, grid_qos)
        self.create_subscription(
            Odometry, '/leroy_nav2/diagnostics/synthetic_odom',
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/leroy_nav2/diagnostics/raw_depth_points',
            self._on_raw, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/leroy_nav2/diagnostics/obstacle_points',
            self._on_obstacle, qos_profile_sensor_data)

    def _on_raw(self, msg):
        if msg.width and msg.header.frame_id == 'sim_camera_optical_frame':
            self.raw_stamps.add(_stamp_ns(msg.header.stamp))
            self._pair_clouds()

    def _on_obstacle(self, msg):
        if msg.width and msg.header.frame_id == 'base_link':
            self.obstacle_stamps.add(_stamp_ns(msg.header.stamp))
            self._pair_clouds()

    def _pair_clouds(self):
        matches = self.raw_stamps & self.obstacle_stamps
        self.cloud_pairs += len(matches)
        self.raw_stamps.difference_update(matches)
        self.obstacle_stamps.difference_update(matches)
        if len(self.raw_stamps) > 60 or len(self.obstacle_stamps) > 60:
            raise RuntimeError('FAIL: synthetic cloud stamp backlog')

    def _on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise RuntimeError('FAIL: incorrect odometry frame')
        stamp = _stamp_ns(msg.header.stamp)
        if stamp <= self.last_odom_ns:
            raise RuntimeError('FAIL: nonmonotonic odometry stamp')
        self.last_odom_ns = stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        vals = (p.x, p.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z)
        if not all(math.isfinite(v) for v in vals):
            raise RuntimeError('FAIL: nonfinite synthetic odometry')
        if abs(q.z * q.z + q.w * q.w - 1.0) > 1e-5:
            raise RuntimeError('FAIL: invalid planar pose quaternion')
        if any(msg.pose.covariance[i] <= 0 or msg.twist.covariance[i] <= 0
               for i in (0, 7, 35)):
            raise RuntimeError('FAIL: missing nonzero synthetic covariances')
        self.x_samples.append(p.x)
        self.y_samples.append(p.y)
        self.yaw_samples.append(2.0 * math.atan2(q.z, q.w))
        self.latest_pose = (p.x, p.y)
        self.forward |= t.linear.x > 0.04
        self.strafe |= t.linear.y > 0.04
        self.turn |= t.angular.z > 0.1
        self.stopped |= (abs(t.linear.x) < 1e-6 and
                         abs(t.linear.y) < 1e-6 and
                         abs(t.angular.z) < 1e-6)
        self.odom_by_stamp[stamp] = (p.x, p.y, q.z, q.w)
        self._pair_tf(stamp)

    def _on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                stamp = _stamp_ns(tf.header.stamp)
                self.tf_by_stamp[stamp] = (
                    tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w)
                self._pair_tf(stamp)

    def _pair_tf(self, stamp):
        if stamp not in self.odom_by_stamp or stamp not in self.tf_by_stamp:
            return
        a = self.odom_by_stamp.pop(stamp)
        b = self.tf_by_stamp.pop(stamp)
        if not all(math.isfinite(v) for v in b):
            raise RuntimeError('FAIL: nonfinite odom->base_link TF')
        if any(abs(x - y) > 1e-6 for x, y in zip(a, b)):
            raise RuntimeError('FAIL: TF/odom disagree at acquisition time')
        self.odom_tf_matches += 1
        if len(self.odom_by_stamp) > 250 or len(self.tf_by_stamp) > 250:
            raise RuntimeError('FAIL: TF/odom matching backlog')

    def _on_map(self, msg):
        if msg.header.frame_id != 'odom':
            raise RuntimeError('FAIL: rolling costmap not in odom frame')
        info = msg.info
        if info.width < 1 or info.height < 1 or info.resolution <= 0:
            return
        if len(msg.data) != info.width * info.height:
            raise RuntimeError('FAIL: incomplete OccupancyGrid')
        self.lethal_and_unknown |= (100 in msg.data and -1 in msg.data)
        ox = info.origin.position.x
        oy = info.origin.position.y
        if not math.isfinite(ox) or not math.isfinite(oy):
            raise RuntimeError('FAIL: invalid rolling map origin')
        self.origins_x.append(ox)
        self.origins_y.append(oy)
        if self.latest_pose is not None and tracks_robot(
                ox, oy, info.width, info.height, info.resolution,
                self.latest_pose[0], self.latest_pose[1]):
            self.center_matches += 1

    def ready(self):
        return all((
            self.odom_tf_matches >= 5,
            self.cloud_pairs >= 2,
            self.lethal_and_unknown,
            self.center_matches >= 2,
            self.forward, self.strafe, self.turn, self.stopped,
            spans(self.x_samples, 0.10),
            spans(self.y_samples, 0.10),
            spans(self.yaw_samples, 0.20),
            spans(self.origins_x, 0.05),
            spans(self.origins_y, 0.05),
        ))


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedProbe()
    try:
        deadline = time.monotonic() + 28.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            topics = {name for name, _ in node.get_topic_names_and_types()}
            if '/cmd_vel' in topics or '/odom' in topics:
                raise RuntimeError('FAIL: motor command or live odometry topic exists')
            node_names = set(node.get_node_names())
            if any(
                any(blocked in name for blocked in (
                    'controller_server', 'leroy_bridge', 'bt_navigator',
                    'planner_server', 'velocity_smoother'))
                for name in node_names
            ):
                raise RuntimeError('FAIL: navigation or hardware command node exists')
            if node.ready():
                print(
                    'PASS: synthetic odom->base_link->camera TF, same-stamp '
                    'depth clouds, odom-frame lethal+unknown rolling costmap '
                    'tracks forward/lateral motion; no /cmd_vel or /odom')
                return
        raise RuntimeError(
            'FAIL: incomplete M4 integration: '
            f'tf={node.odom_tf_matches} clouds={node.cloud_pairs} '
            f'lethal_unknown={node.lethal_and_unknown} '
            f'center={node.center_matches} '
            f'odom_x={spans(node.x_samples, 0.10)} '
            f'odom_y={spans(node.y_samples, 0.10)} '
            f'rolling_x={spans(node.origins_x, 0.05)} '
            f'rolling_y={spans(node.origins_y, 0.05)} '
            f'phases={node.forward,node.strafe,node.turn,node.stopped}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
