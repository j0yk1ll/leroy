"""M15 replay of real ROS MESSAGE SHAPES, not real sensor measurements.

One-shot artificial depth, odom or XRSLAM-tracking fault via Trigger.
Publishes ONLY m15_replay diagnostic namespace and matching TFMessage,
never physical /odom, /tf, /cmd_vel or a motor topic.
"""
import json
import math
import os
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .m9_mppi_shadow_probe import require_offline
from .m15_real_input_gate import REPLAY_PREFIX, topics_for, PROFILE_REPLAY


FAULT_DURATION_S = 4.0
MODES = ('odom', 'depth', 'tracking')


class M15ReplaySource(Node):
    def __init__(self):
        require_offline(os.environ)
        super().__init__('m15_replay_source')
        self.declare_parameter('fault_mode', 'depth')
        self.mode = self.get_parameter('fault_mode').value
        if self.mode not in MODES:
            raise RuntimeError('M15 unknown synthetic fault mode')
        self.p = topics_for(PROFILE_REPLAY)
        self.start = self.end = None
        self.last_sample_bucket = None
        self.last_tracking_state = None
        self.last_valid = None
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, self.p['odom'], 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.p['cloud'], 10)
        self.tf_pub = self.create_publisher(TFMessage, self.p['tf'], 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.p['map'], latched)
        self.proposal_pub = self.create_publisher(
            Twist, self.p['proposal'], 10)
        self.status_pub = self.create_publisher(
            String, self.p['tracking'], latched)
        self.valid_pub = self.create_publisher(
            Bool, self.p['valid'], latched)
        self.trigger = self.create_service(
            Trigger, REPLAY_PREFIX + '/trigger', self._trigger)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            'M15 isolated synthetic, XRSLAM/DepthART-shaped input replay; '
            'no real sensors, no motor channels or calibrations')

    def _trigger(self, request, response):
        del request
        if self.start is not None:
            response.success = False
            response.message = 'M15 replay already triggered'
            return response
        self.start = time.monotonic()
        self.end = self.start + FAULT_DURATION_S
        response.success = True
        response.message = json.dumps(
            {'start': self.start, 'end': self.end, 'mode': self.mode})
        print('M15 REPLAY: START ' + self.mode, flush=True)
        return response

    def _tick(self):
        now = time.monotonic()
        blocked = self.start is not None and self.start <= now < self.end
        tracking_lost = blocked and self.mode == 'tracking'
        state = 'tracking_lost' if tracking_lost else 'tracking'
        valid = not tracking_lost
        if state != self.last_tracking_state:
            msg = String()
            msg.data = state
            self.status_pub.publish(msg)
            self.last_tracking_state = state
        if valid != self.last_valid:
            msg = Bool()
            msg.data = valid
            self.valid_pub.publish(msg)
            self.last_valid = valid

        proposal = Twist()
        proposal.linear.x = .08
        proposal.linear.y = .02
        proposal.angular.z = .04
        self.proposal_pub.publish(proposal)

        # Samples at 2Hz, proposals at 10Hz.
        bucket = int(now * 2)
        if bucket == self.last_sample_bucket:
            return
        self.last_sample_bucket = bucket
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = 'odom'
        cells = OccupancyGrid()
        cells.header = header
        cells.info.width = 4
        cells.info.height = 4
        cells.info.resolution = .25
        cells.data = [-1] * 15 + [100]
        self.map_pub.publish(cells)

        if not blocked or self.mode not in ('odom', 'tracking'):
            odom = Odometry()
            odom.header = header
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x = .2
            self.odom_pub.publish(odom)
            tf = TransformStamped()
            tf.header = header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = .2
            tf.transform.rotation.w = 1.0
            self.tf_pub.publish(TFMessage(transforms=[tf]))

        if not blocked or self.mode != 'depth':
            depth_header = Header()
            depth_header.stamp = stamp
            depth_header.frame_id = 'base_link'
            cloud = create_cloud_xyz32(
                depth_header, [(1.0, .2, .4), (1.2, -.2, .4)])
            self.cloud_pub.publish(cloud)


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = M15ReplaySource()
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
