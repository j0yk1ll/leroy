"""Continuous, read-only ROS health aggregation; never authorizes motion."""
import json
import time

from leroy_bringup.readiness import HealthEvidence, ReadinessAggregator
from leroy_bringup.health_contract import COMPONENTS, OWNERS, TOPICS, decode

READINESS_TOPIC = "/leroy/readiness/status"


class ReadinessNode:
    def __init__(self, node):
        from std_msgs.msg import String
        self.node = node
        self.aggregator = ReadinessAggregator(max_age_s=0.5)
        self._publish = node.create_publisher(String, READINESS_TOPIC, 10)
        self._subs = [
            node.create_subscription(
                String, TOPICS[component],
                lambda msg, component=component: self.on_health(component, msg),
                10)
            for component in OWNERS
        ]
        self._timer = node.create_timer(0.2, self.tick)

    def on_health(self, component, msg):
        report = decode(msg.data, component)
        if report is None:
            return
        publishers = self.node.get_publishers_info_by_topic(TOPICS[component])
        if (len(publishers) != 1
                or publishers[0].node_name != OWNERS[component]
                or publishers[0].node_namespace != "/"
                or publishers[0].topic_type != "std_msgs/msg/String"):
            return
        # Observe only. No external message is authoritative calibration,
        # driver-power or physical stop evidence.
        self.aggregator.record(HealthEvidence(
            component=component,
            healthy=report["observed"], verified=False,
            observed_ns=time.monotonic_ns(), source=OWNERS[component]))

    def tick(self):
        from std_msgs.msg import String
        status = self.aggregator.evaluate(now_ns=time.monotonic_ns())
        message = String()
        message.data = json.dumps({
            "schema": "leroy_readiness_v1",
            "sensors_healthy": status.sensors_healthy,
            "perception_valid": status.perception_valid,
            "localization_valid": status.localization_valid,
            "mcu_healthy": status.mcu_healthy,
            "physical_stop_verified": status.physical_stop_verified,
            "navigation_prerequisites_met": status.navigation_prerequisites_met,
            "motion_authorized": False,
            "failure_reasons": list(status.failure_reasons),
        }, sort_keys=True)
        self._publish.publish(message)


def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("leroy_readiness", enable_rosout=False)
    monitor = ReadinessNode(node)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
