"""sensor observer bounded PASSIVE monitoring of Leroy's actual ROS sensor topics.

Subscriptions + graph introspection only. No command publisher, Nav2 goal,
lifecycle transition, parameter client, actuator, USB/serial, or input
remapping. Prints JSON summary containing metadata, not image/IMU payloads.
"""

from collections import deque
import json
import os
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
import rclpy

from .sensor_contract import (
    stamp_seconds, validate_odom, validate_tf, validate_base_cloud,
)
from .sensor_health import (
    INPUTS, StreamStats, check_image, check_camera_info,
    check_imu, depth_sample,
)

DEFAULT_DURATION_S = 30.0
MIN_DURATION_S = 5.0
MAX_DURATION_S = 180.0


def requested_duration(env):
    try:
        seconds = float(env.get('LEROY_SENSOR_OBSERVER_DURATION_S', str(DEFAULT_DURATION_S)))
    except (ValueError, TypeError) as exc:
        raise ValueError('LEROY_SENSOR_OBSERVER_DURATION_S must be numeric') from exc
    if not MIN_DURATION_S <= seconds <= MAX_DURATION_S:
        raise ValueError('sensor observer duration must be 5 through 180 seconds')
    return seconds


def endpoint_evidence(endpoints):
    """Publisher graph names are observations, not authenticated sources."""
    return sorted([
        {
            'node': str(getattr(p, 'node_name', '?')),
            'namespace': str(getattr(p, 'node_namespace', '?')),
            'type': str(getattr(p, 'topic_type', '?')),
        } for p in endpoints
    ], key=lambda p: (p['node'], p['namespace'], p['type']))


class SensorMonitor(Node):
    def __init__(self):
        if os.environ.get('LEROY_NAV2_DRY_RUN') != '1':
            raise RuntimeError('sensor observer requires LEROY_NAV2_DRY_RUN=1')
        super().__init__(
            'leroy_sensor_observer', enable_rosout=False,
            start_parameter_services=False)
        self.stats = {key: StreamStats() for key in INPUTS}
        self.status = {'tracking_status': None, 'localization_valid': None}
        self.status_seen = {'tracking_status': 0, 'localization_valid': 0}
        self.latest_odom_stamp = None
        self.latest_tf_stamp = None
        self.camera_info_stamp = None
        self.camera_info_recent = deque(maxlen=512)
        self.camera_info_frame = None
        self.depth_caminfo_matches = 0
        self.depth_caminfo_mismatches = 0
        self.ros_clock_at_start = self.get_clock().now().nanoseconds*1e-9
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        # rclpy.Node.subscriptions is a read-only property on Jazzy.
        # Hold our handles in a private attribute instead; the Node owns
        # subscriptions and never creates publishers or service clients.
        self._subscriptions = [
            self.create_subscription(
                Image, INPUTS['camera_image'][0],
                lambda m: self._image('camera_image', m),
                qos_profile_sensor_data),
            self.create_subscription(
                CameraInfo, INPUTS['camera_info'][0],
                self._camera_info, qos_profile_sensor_data),
            self.create_subscription(
                Image, INPUTS['depth_image'][0],
                lambda m: self._image('depth_image', m),
                qos_profile_sensor_data),
            self.create_subscription(
                Imu, INPUTS['imu'][0], self._imu,
                qos_profile_sensor_data),
            self.create_subscription(
                String, INPUTS['xrslam_tracking'][0],
                self._tracking, latched),
            self.create_subscription(
                Bool, INPUTS['xrslam_valid'][0],
                self._valid, latched),
            self.create_subscription(
                Odometry, INPUTS['odom'][0],
                self._odom, qos_profile_sensor_data),
            self.create_subscription(
                TFMessage, INPUTS['tf'][0],
                self._tf, qos_profile_sensor_data),
            self.create_subscription(
                PointCloud2, INPUTS['obstacle_cloud'][0],
                self._cloud, qos_profile_sensor_data),
        ]
        if any(s.topic_name != INPUTS[k][0] for k,s in zip(INPUTS,self._subscriptions)):
            raise RuntimeError('sensor observer refuses ROS subscription remapping')
        self.get_logger().info(
            'sensor observer passive observer: subscriptions only, never commands motors')

    def _now(self):
        return time.monotonic(), self.get_clock().now().nanoseconds*1e-9

    def _observe(self, key, header, error=None, *, stamp=None, **kwargs):
        mono, ros_now = self._now()
        if stamp is None:
            stamp = stamp_seconds(header.stamp)
        return self.stats[key].observe(
            stamp=stamp, frame=header.frame_id, mono=mono,
            ros_now=ros_now, error=error, **kwargs)

    def _image(self, key, msg):
        error = check_image(msg, depth=(key == 'depth_image'))
        if self._observe(key, msg.header, error):
            self.stats[key].details['encoding'] = msg.encoding
            self.stats[key].details['dimensions'] = [msg.width, msg.height]
            if key == 'depth_image':
                count = depth_sample(msg)
                self.stats[key].details['positive_finite_sample_fraction'] = (
                    round(count[0] / count[1], 3) if count else None)
                depth_stamp = stamp_seconds(msg.header.stamp)
                if (depth_stamp is not None and
                        (msg.header.frame_id, depth_stamp)
                        in self.camera_info_recent):
                    self.depth_caminfo_matches += 1
                else:
                    self.depth_caminfo_mismatches += 1

    def _camera_info(self, msg):
        good = self._observe(
            'camera_info', msg.header, check_camera_info(msg))
        if good:
            self.camera_info_stamp = stamp_seconds(msg.header.stamp)
            self.camera_info_frame = msg.header.frame_id
            self.camera_info_recent.append((msg.header.frame_id, self.camera_info_stamp))
            self.stats['camera_info'].details['dimensions'] = [
                msg.width, msg.height]
            self.stats['camera_info'].details['rectified_p_fx_fy'] = [
                round(msg.p[0], 3), round(msg.p[5], 3)]

    def _imu(self, msg):
        self._observe('imu', msg.header, check_imu(msg))

    def _tracking(self, msg):
        self.status_seen['tracking_status'] += 1
        self.status['tracking_status'] = msg.data
        st = self.stats['xrslam_tracking']
        st.received += 1
        st.accepted += 1
        st.details['last_value'] = msg.data

    def _valid(self, msg):
        self.status_seen['localization_valid'] += 1
        self.status['localization_valid'] = bool(msg.data)
        st = self.stats['xrslam_valid']
        st.received += 1
        st.accepted += 1
        st.details['last_value'] = bool(msg.data)

    def _odom(self, msg):
        _, ros_now = self._now()
        stamp = validate_odom(
            msg, now_ros=ros_now,
            previous_stamp=self.latest_odom_stamp)
        if self._observe(
                'odom', msg.header,
                None if stamp is not None else 'invalid_odom_contract'):
            self.latest_odom_stamp = stamp

    def _tf(self, msg):
        for trans in msg.transforms:
            if (trans.header.frame_id == 'odom'
                    and trans.child_frame_id == 'base_link'):
                _, ros_now = self._now()
                stamp = validate_tf(
                    trans, now_ros=ros_now,
                    previous_stamp=self.latest_tf_stamp)
                if self._observe(
                        'tf', trans.header,
                        None if stamp is not None
                        else 'invalid_dynamic_odom_base_tf'):
                    self.latest_tf_stamp = stamp

    def _cloud(self, msg):
        _, ros_now = self._now()
        stamp = validate_base_cloud(
            msg, now_ros=ros_now,
            previous_stamp=self.stats['obstacle_cloud'].last_stamp)
        self._observe(
            'obstacle_cloud', msg.header,
            None if stamp is not None
            else 'invalid_base_obstacle_cloud')

    def report(self, duration_s):
        now = time.monotonic()
        names = dict(self.get_topic_names_and_types())
        topics = {}
        for key, (topic, msg_type) in INPUTS.items():
            pubs = endpoint_evidence(
                self.get_publishers_info_by_topic(topic))
            stats = self.stats[key].export(now)
            if key in ('xrslam_tracking', 'xrslam_valid'):
                stats['fresh_valid_receipt'] = None
                stats['note'] = 'unstamped latched status: NOT freshness evidence'
            topics[key] = {
                'topic': topic, 'expected_type': msg_type,
                'discovered_types': names.get(topic, []),
                'publishers': pubs,
                'publisher_count': len(pubs),
                **stats,
            }
        imu = topics['imu']
        depth = topics['depth_image']
        odom = topics['odom']
        tf = topics['tf']
        xrslam_state = (
            'NOT_OBSERVED'
            if not self.status_seen['tracking_status'] or
            not self.status_seen['localization_valid']
            else 'TRACKING_INDICATORS_POSITIVE' if (
                self.status['tracking_status'] == 'tracking'
                and self.status['localization_valid'] is True)
            else 'TRACKING_NOT_VALID')
        tf_delta = (
            abs(self.latest_odom_stamp-self.latest_tf_stamp)
            if (self.latest_odom_stamp is not None
                and self.latest_tf_stamp is not None)
            else None)
        xrslam_observed = (
            xrslam_state == 'TRACKING_INDICATORS_POSITIVE'
            and odom['accepted'] >= 2 and tf['accepted'] >= 2
            and odom['fresh_valid_receipt'] and tf['fresh_valid_receipt']
            and tf_delta is not None and tf_delta <= 0.25)
        depth_observed = (
            depth['accepted'] >= 2
            and depth['fresh_valid_receipt']
            and topics['camera_info']['accepted'] >= 2
            and topics['camera_info']['fresh_valid_receipt']
            and self.depth_caminfo_matches >= 1)
        obstacle_observed = (
            topics['obstacle_cloud']['accepted'] >= 2
            and topics['obstacle_cloud']['fresh_valid_receipt'])
        graph_warning = []
        expected_source = {
            'depth_image': 'depthart', 'imu': 'leroy_bridge',
            'odom': 'leroy_xrslam', 'xrslam_valid': 'leroy_xrslam',
            'xrslam_tracking': 'leroy_xrslam',
            'obstacle_cloud': 'depth_obstacles',
        }
        for key, source in expected_source.items():
            entry = topics[key]
            publishers = entry['publishers']
            if publishers and (
                len(publishers) != 1
                or publishers[0]['node'] != source
                or publishers[0]['namespace'] != '/'
                or publishers[0]['type'] != entry['expected_type']):
                graph_warning.append({
                    'stream': key, 'expected_source': source,
                    'observed_publishers': publishers})
        result = {
            'component': 'sensor_observer',
            'observation_only': True,
            'duration_requested_s': duration_s,
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'xrslam_indicators': self.status,
            'xrslam_state': xrslam_state,
            'odom_tf_latest_stamp_delta_s': (
                round(tf_delta, 4) if tf_delta is not None else None),
            'depth_camera_info_stamp_frame_matches': self.depth_caminfo_matches,
            'depth_camera_info_stamp_frame_mismatches': self.depth_caminfo_mismatches,
            'stream_contract_observations': {
                'xrslam': bool(xrslam_observed),
                'depthart': bool(depth_observed),
                'base_obstacle_cloud': bool(obstacle_observed),
                'imu': bool(imu['accepted'] >= 2 and imu['fresh_valid_receipt']),
            },
            'publisher_graph_warnings': graph_warning,
            'topics': topics,
        }
        result['complete_real_sensor_contract_observation'] = (
            all(result['stream_contract_observations'].values())
            and not graph_warning)
        result['verdict'] = (
            'OBSERVED_MESSAGE_CONTRACTS_ONLY'
            if result['complete_real_sensor_contract_observation']
            else 'INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS')
        # JSON output deliberately omits pixels, paths, image payloads,
        # frame images, serial content, ROS messages or personal data.
        return result


def main(args=None):
    seconds = requested_duration(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = SensorMonitor()
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = node.report(seconds)
        print('SENSOR_MONITOR_REPORT_JSON=' +
              json.dumps(summary, sort_keys=True), flush=True)
        print('SENSOR_MONITOR_VERDICT=' + summary['verdict'], flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
