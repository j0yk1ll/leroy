"""Read-only sensors stream health reporter; never certifies physical readiness."""
import json
import time

COMPONENT = "sensors"
INPUT_TOPIC = "/imu/data_raw"
OUTPUT_TOPIC = "/leroy/health/sensors"
SOURCE = "leroy_sensors_health"


def observed_valid(message):
    """Plausibility only; calibration and physical verification are separate."""
    return (message is not None and bool(message.header.frame_id))


class HealthReporter:
    def __init__(self, node):
        from std_msgs.msg import String
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        self.node = node
        self.last_ns = None
        self.plausible = False
        self.publisher = node.create_publisher(String, OUTPUT_TOPIC, 10)
        self.subscriber = node.create_subscription(
            Imu, INPUT_TOPIC, self.on_message, qos_profile_sensor_data)
        self.timer = node.create_timer(0.2, self.tick)

    def on_message(self, message):
        self.last_ns = time.monotonic_ns()
        self.plausible = bool(observed_valid(message))

    def tick(self):
        from std_msgs.msg import String
        now = time.monotonic_ns()
        publishers = self.node.get_publishers_info_by_topic(INPUT_TOPIC)
        observed = (self.last_ns is not None and self.plausible
                    and 0 <= now - self.last_ns <= 500_000_000
                    and len(publishers) == 1
                    and bool(publishers[0].node_name))
        # Published DDS claims, timing, graph owners and packet plausibility
        # cannot independently prove calibration, metric pose, motor stop or
        # physical power isolation. The verified field is intentionally false.
        msg = String()
        msg.data = json.dumps({
            "schema": "leroy_health_v1", "component": COMPONENT,
            "source": SOURCE, "observed": bool(observed),
            "verified": False,
            "reason": "stream_observed_unqualified" if observed else "stream_missing_or_invalid",
        }, sort_keys=True)
        self.publisher.publish(msg)


def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node(SOURCE, enable_rosout=False)
    reporter = HealthReporter(node)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
