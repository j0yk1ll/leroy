"""M14 runtime SOFTWARE gate for isolated MPPI DIAGNOSTICS ONLY.

Consumes the ACTUAL remapped MPPI Twist and synthetic odom/cloud/map;
publishes a new, physically UNCONSUMED gated diagnostic Twist plus
per-tick diagnostic state. Unlike M7's observer it really emits ZERO
instead of a nonzero proposal whenever inputs are rejected.

No action client, motor publisher, /cmd_vel, serial, device mapping,
physical trust/authorization, or safe-stop guarantee.
"""

import json
import math
import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .controller_probe import ODOM, MAP
from .fault_contract import _recent
from .m9_mppi_shadow_probe import CLOUD, _stamp, require_offline
from .m9_mppi_source import MPPI_TOPIC, inspect_mppi_source
from .shadow_policy import DiagnosticProposal, ShadowSnapshot
from .m14_gate_core import DiagnosticCommandGate, ZERO, is_nonzero


GATED_TOPIC = '/leroy_nav2/diagnostics/gated_cmd_vel'
STATUS_TOPIC = '/leroy_nav2/diagnostics/m14_gate_status'
EXPECTED_SOURCE = 'controller_server'
EXPECTED_GATED_SOURCE = 'm14_software_gate'


class M14SoftwareGate(Node):
    def __init__(self):
        require_offline(os.environ)
        super().__init__(EXPECTED_GATED_SOURCE)
        self.gate = DiagnosticCommandGate()
        self.proposal = None
        self.odom_recv = self.odom_stamp = self.last_odom_stamp = None
        self.depth_recv = self.depth_stamp = self.last_depth_stamp = None
        self.tf_recv = self.tf_stamp = self.last_tf_stamp = None
        self.map_recv = None
        self.map_valid = False
        self.last_label = None
        self.sequence = 0
        self.odom_sub = self.create_subscription(
            Odometry, ODOM, self._on_odom, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            PointCloud2, CLOUD, self._on_depth, qos_profile_sensor_data)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self._on_tf, qos_profile_sensor_data)
        self.map_sub = self.create_subscription(
            OccupancyGrid, MAP, self._on_map, QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.proposal_sub = self.create_subscription(
            Twist, MPPI_TOPIC, self._on_proposal, 10)
        self.out = self.create_publisher(Twist, GATED_TOPIC, 10)
        self.status = self.create_publisher(String, STATUS_TOPIC, 10)

        # Reject ROS remapping attempts; no argument may turn our output
        # into a motor-compatible command. Primary isolation is Podman.
        required_topics = (
            (self.odom_sub, ODOM), (self.depth_sub, CLOUD),
            (self.map_sub, MAP), (self.tf_sub, '/tf'),
            (self.proposal_sub, MPPI_TOPIC),
            (self.out, GATED_TOPIC), (self.status, STATUS_TOPIC),
        )
        if any(endpoint.topic_name != expected
               for endpoint, expected in required_topics):
            raise RuntimeError('FAIL: M14 diagnostic topic remapping forbidden')
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            'M14 REAL SOFTWARE SUPPRESSION on a GATED DIAGNOSTIC topic '
            'ONLY; never connect to /cmd_vel or motor bridge')

    def _on_odom(self, msg):
        stamp = _stamp(msg.header.stamp)
        if (msg.header.frame_id != 'odom'
                or msg.child_frame_id != 'base_link'
                or not math.isfinite(stamp) or stamp <= 0
                or (self.last_odom_stamp is not None
                    and stamp <= self.last_odom_stamp)):
            self.odom_recv = self.odom_stamp = None
            return
        self.last_odom_stamp = stamp
        self.odom_stamp = stamp
        self.odom_recv = time.monotonic()

    def _on_depth(self, msg):
        stamp = _stamp(msg.header.stamp)
        if (msg.header.frame_id != 'base_link' or msg.width <= 0
                or msg.height <= 0 or not math.isfinite(stamp) or stamp <= 0
                or (self.last_depth_stamp is not None
                    and stamp <= self.last_depth_stamp)):
            self.depth_recv = self.depth_stamp = None
            return
        self.last_depth_stamp = stamp
        self.depth_stamp = stamp
        self.depth_recv = time.monotonic()

    def _on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id != 'odom'
                    or transform.child_frame_id != 'base_link'):
                continue
            stamp = _stamp(transform.header.stamp)
            q = transform.transform.rotation
            t = transform.transform.translation
            quat = (q.x, q.y, q.z, q.w)
            components = quat + (t.x, t.y, t.z, stamp)
            if (not all(math.isfinite(v) for v in components)
                    or stamp <= 0
                    or abs(sum(v*v for v in quat) - 1.0) > .05
                    or (self.last_tf_stamp is not None
                        and stamp <= self.last_tf_stamp)):
                self.tf_recv = self.tf_stamp = None
                return
            self.last_tf_stamp = stamp
            self.tf_recv = time.monotonic()
            self.tf_stamp = stamp

    def _on_map(self, msg):
        info = msg.info
        self.map_valid = (
            msg.header.frame_id == 'odom'
            and info.width > 0 and info.height > 0
            and len(msg.data) == info.width * info.height
            and 100 in msg.data and -1 in msg.data)
        self.map_recv = time.monotonic() if self.map_valid else None

    def _on_proposal(self, msg):
        self.proposal = DiagnosticProposal(
            vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
            wx=msg.angular.x, wy=msg.angular.y, wz=msg.angular.z,
            received_mono=time.monotonic())

    def _graph_health(self):
        topics = dict(self.get_topic_names_and_types())
        if '/cmd_vel' in topics or '/odom' in topics:
            return False, False
        if any(any(blocked in node for blocked in (
                'leroy_bridge', 'bt_navigator',
                'planner_server', 'velocity_smoother'))
                for node in self.get_node_names()):
            return False, False
        for topic, expected_type in (
                (MPPI_TOPIC, 'geometry_msgs/msg/Twist'),
                (GATED_TOPIC, 'geometry_msgs/msg/Twist'),
                (STATUS_TOPIC, 'std_msgs/msg/String')):
            if topic in topics and topics[topic] != [expected_type]:
                return False, False
        endpoints = self.get_publishers_info_by_topic(MPPI_TOPIC)
        verified, pending, _evidence = inspect_mppi_source(endpoints)
        # Pending and unexpected nodes are both inhibit conditions;
        # graph metadata is NOT physical source authentication.
        return True, verified and not pending

    def _snapshot(self, now):
        return ShadowSnapshot(
            now_mono=now,
            now_ros=self.get_clock().now().nanoseconds * 1e-9,
            odom_recv=self.odom_recv, odom_stamp=self.odom_stamp,
            depth_recv=self.depth_recv, depth_stamp=self.depth_stamp,
            map_recv=self.map_recv, map_valid=self.map_valid,
            proposal=self.proposal)

    def _tick(self):
        now = time.monotonic()
        graph_ok, source_ok = self._graph_health()
        snapshot = self._snapshot(now)
        tf_ok = _recent(now, snapshot.now_ros,
                        self.tf_recv, self.tf_stamp)
        output, reason = self.gate.evaluate(
            snapshot, graph_ok=graph_ok,
            source_ok=source_ok, tf_ok=tf_ok)
        if reason != self.last_label:
            print('M14 GATE: ' + reason, flush=True)
            self.last_label = reason
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = output
        # This new diagnostic output is always exactly ZERO on reject.
        self.out.publish(cmd)

        self.sequence += 1
        incoming = self.proposal
        input_nonzero = False
        if incoming is not None:
            xyz = (incoming.vx, incoming.vy, incoming.wz)
            if all(math.isfinite(v) for v in xyz):
                input_nonzero = is_nonzero(xyz)
        status = {
            'm14_diagnostic_only': True,
            'sequence': self.sequence,
            'now_mono': now,
            'reason': reason,
            'input_nonzero': input_nonzero,
            'output_nonzero': is_nonzero(output),
            'output_xyz': list(output),
        }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status.publish(msg)


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = M14SoftwareGate()
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
