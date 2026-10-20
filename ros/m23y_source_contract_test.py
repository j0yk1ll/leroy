#!/usr/bin/env python3
"""M23Y offline tests; no ROS imports, process launch, containers or hardware."""
import ast
from pathlib import Path
import unittest

from m23y_drained_ab import MIN_DRAIN_NS, MAX_DRAIN_NS, validate_drain

ROOT = Path(__file__).resolve().parent
OLD_CPP = ROOT / "m23s_reference.cpp"
NEW_CPP = ROOT / "m23y_cpp/src/m23y_drained_publisher.cpp"

OLD_SHUTDOWN = "      rclcpp::shutdown();\n"
DRAIN_BLOCK = """      // M23Y synthetic-only experiment: keep both reliable publishers alive
      // after the last 900th pair, without publishing any extra frames.
      timer_->cancel();
      drain_started_ns_ = steady_ns();
      std::cout << "M23Y_PUBLISHER_DRAIN_STARTED=YES" << std::endl;
      drain_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        drain_timer_->cancel();
        const std::int64_t elapsed = steady_ns() - drain_started_ns_;
        std::cout << "M23Y_PUBLISHER_DRAIN_ELAPSED_NS=" << elapsed << std::endl;
        std::cout << "M23Y_PUBLISHER_DRAIN_COMPLETED=YES" << std::endl;
        rclcpp::shutdown();
      });
"""
OLD_MEMBER = "  rclcpp::TimerBase::SharedPtr timer_;\n"
NEW_MEMBER = OLD_MEMBER + """  rclcpp::TimerBase::SharedPtr drain_timer_;
  std::int64_t drain_started_ns_ = 0;
"""


class M23YSourceContracts(unittest.TestCase):
    def test_exact_cpp_delta_only_stop_timer_and_delayed_shutdown(self):
        old = OLD_CPP.read_text()
        new = NEW_CPP.read_text()
        self.assertEqual(old.count(OLD_SHUTDOWN), 1)
        self.assertEqual(old.count(OLD_MEMBER), 1)
        self.assertEqual(new, old.replace(OLD_SHUTDOWN, DRAIN_BLOCK)
                         .replace(OLD_MEMBER, NEW_MEMBER))
        for shared in (
            'constexpr unsigned int kFrames = 900;',
            'constexpr std::int64_t kPeriodNs = 33333333LL;',
            '"/camera/image_raw"',
            '"/camera/camera_info"',
            'M23S_DISTORTION_PROFILE',
            'info_.d = {-0.08, 0.01, 0.001, -0.001, 0.0};',
            'rclcpp::QoS(rclcpp::KeepLast(100))',
            'image_pub_->publish(image_);',
            'info_pub_->publish(info_);',
        ):
            self.assertIn(shared, new)

    def test_drain_marker_strict_order_and_elapsed_bounds(self):
        def markers(elapsed=2_010_000_000):
            return "\n".join((
                "M23P_PUBLISHER_COUNTS=900,900",
                "M23Y_PUBLISHER_DRAIN_STARTED=YES",
                "M23Y_PUBLISHER_DRAIN_ELAPSED_NS=" + str(elapsed),
                "M23Y_PUBLISHER_DRAIN_COMPLETED=YES",
            ))
        self.assertEqual(validate_drain(markers()), 2_010_000_000)
        for data in (
            markers(10),
            markers(MAX_DRAIN_NS + 1),
            markers(MIN_DRAIN_NS - 1),
            markers().replace("M23Y_PUBLISHER_DRAIN_COMPLETED=YES", ""),
            markers() + "\nM23Y_PUBLISHER_DRAIN_COMPLETED=YES",
            markers().replace("M23Y_PUBLISHER_DRAIN_ELAPSED_NS=2010000000",
                              "M23Y_PUBLISHER_DRAIN_ELAPSED_NS=x"),
            "\n".join(reversed(markers().splitlines())),
        ):
            with self.subTest(data=data[:50]), self.assertRaises(ValueError):
                validate_drain(data)

    def test_wrapper_only_substitutes_fixture_both_arms_and_inherits_gates(self):
        code = (ROOT / "m23y_drained_ab.py").read_text()
        tree = ast.parse(code)
        self.assertIn("m23s.PUBLISHER = PUBLISHER", code)
        self.assertIn("m23s.validate_publisher_log = checked_publisher", code)
        self.assertIn("status = m23x.run()", code)
        self.assertIn('M23Y_DRAIN_ONLY', code)
        self.assertIn('M23S_DISTORTION_PROFILE', code)
        self.assertIn('("baseline", "raw")', code)
        self.assertIn('M23Y_DEVICE_ACCESS=NONE', code)
        self.assertNotIn('rclpy', code)
        forbidden = {"subprocess", "serial", "rclpy"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                self.assertTrue(all(a.name.split(".")[0] not in forbidden
                                    for a in node.names))
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn((node.module or "").split(".")[0], forbidden)

    def test_package_and_disconnected_image_contract(self):
        cmake = (ROOT / "m23y_cpp/CMakeLists.txt").read_text()
        xml = (ROOT / "m23y_cpp/package.xml").read_text()
        docker = (ROOT / "Dockerfile.m23y-synthetic-drain").read_text()
        self.assertIn("project(leroy_m23y_drain_publisher", cmake)
        self.assertIn("<name>leroy_m23y_drain_publisher</name>", xml)
        self.assertIn("FROM localhost/leroy-sensors:m23x-raw-publisher-controlled-v2",
                      docker)
        self.assertIn("M23Y_DRAIN_ONLY=1", docker)
        self.assertIn("M23S_DISTORTION_PROFILE=rectify", docker)
        self.assertIn("m23y_source_contract_test.py", docker)
        self.assertIn("--packages-select leroy_m23y_drain_publisher", docker)
        self.assertIn("M23Y_SOURCE_IMAGE_CONTRACT=PASS", docker)
        for banned in (
            "usb_cam", "/dev/video", "/dev/ttyACM",
            "ros2 launch", "podman run", "M23T_OPENCV_THREAD_PROFILE",
            "cv::setNumThreads",
        ):
            self.assertNotIn(banned, docker)


if __name__ == "__main__":
    unittest.main()
