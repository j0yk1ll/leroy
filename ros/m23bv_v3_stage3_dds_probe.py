"""Stage 3 isolated DDS smoke probe. Publishes/receives a nonce; no devices."""
import argparse
import os
import time

import rclpy
from std_msgs.msg import String

TOPIC = "/leroy_nav2/diagnostics/stage3_dds_ping"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=["publish", "XRSLAM", "NAV2"])
    parser.add_argument("--nonce", required=True)
    args = parser.parse_args()
    if (os.environ.get("ROS_DOMAIN_ID") != "170"
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"):
        raise RuntimeError("Isolated ROS 2 Jazzy domain 170 and FastDDS required")
    rclpy.init()
    node = rclpy.create_node("stage3_dds_" + args.role.lower())
    try:
        if args.role == "publish":
            sender = node.create_publisher(String, TOPIC, 10)
            until = time.monotonic() + 65.0
            while time.monotonic() < until and rclpy.ok():
                message = String()
                message.data = args.nonce
                sender.publish(message)
                rclpy.spin_once(node, timeout_sec=0)
                time.sleep(0.1)
            return
        got = [False]

        def recv(message):
            if message.data == args.nonce:
                got[0] = True

        subscription = node.create_subscription(String, TOPIC, recv, 10)
        deadline = time.monotonic() + 18.0
        while not got[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not got[0]:
            raise RuntimeError("No matched cross-container DDS nonce received")
        print("M23BV_V3_STAGE3_DDS_" + args.role + "=RECEIVED_MATCHED_NONCE", flush=True)
        assert subscription is not None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
