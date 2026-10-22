"""M23BF: isolated source-only publisher completion synthetic regression gates."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23ax_instrument_usb_cam_source as ax
import m23ax_source_test as ax_tests
import m23bf_publisher_completion_source as m


def fixtures():
    # Construct M23AX-instrumented source from its existing 15-test pinned
    # synthetic fixtures. The separate C++ node destructor/header fragments
    # match the original pinned usb_cam 0.8.1 source reviewed for this gate.
    src = ax.instrument(ax_tests.fixtures())
    cpp = src["src/ros2/usb_cam_node.cpp"]
    cpp += """
UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}
void UsbCamNode::update()
{
}
"""
    header = """#include <memory>
class UsbCamNode {
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;
};
"""
    return {
        "src/ros2/usb_cam_node.cpp": cpp,
        "include/usb_cam/usb_cam_node.hpp": header,
    }


class Tests(unittest.TestCase):
    def test_default_inert(self):
        with patch.object(m, "check_tree", side_effect=AssertionError("read source")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main([]), 0)
        self.assertIn("M23BF_STATUS=INERT_NO_WRITES_NO_ROS_NO_DEVICE",
                      out.getvalue())

    def test_patch_returns_exact_node_pair(self):
        result = m.instrument(fixtures())
        self.assertEqual(set(result), set(m.FILES))

    def test_pre_send_event_index_now_per_node(self):
        original = fixtures()
        changed = m.instrument(original)
        cpp = changed[m.FILES[0]]
        self.assertIn("++m_m23bf_attempts;", cpp)
        self.assertIn("M23AX_PUBLISH pub_index=", cpp)
        self.assertIn('<< m_m23bf_attempts', cpp)
        self.assertNotIn("m23ax_publication_no", cpp)
        self.assertNotIn("static uint32_t", cpp)
        self.assertEqual(original[m.FILES[0]].count("m23ax_publication_no"), 5)

    def test_publish_returns_increment_only_after_publish(self):
        cpp = m.instrument(fixtures())[m.FILES[0]]
        publish = cpp.index(
            "m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);")
        returned = cpp.index("++m_m23bf_publish_returns;")
        ret = cpp.index("  return true;", returned)
        self.assertLess(publish, returned)
        self.assertLess(returned, ret)

    def test_teardown_epilogue_occurs_after_camera_delete(self):
        cpp = m.instrument(fixtures())[m.FILES[0]]
        deleter = cpp.index("  delete (m_camera);")
        marker = cpp.index('M23BE_PUBLISHER_FINAL attempts=')
        self.assertLess(deleter, marker)
        self.assertIn('shutdown=clean" << std::endl;', cpp)

    def test_per_node_fields_are_zero_initialized(self):
        header = m.instrument(fixtures())[m.FILES[1]]
        self.assertIn("std::uint32_t m_m23bf_attempts = 0;", header)
        self.assertIn("std::uint32_t m_m23bf_publish_returns = 0;", header)
        self.assertIn("#include <cstdint>", header)

    def test_teardown_mismatch_fatal_no_false_clean_marker(self):
        cpp = m.instrument(fixtures())[m.FILES[0]]
        self.assertIn("m_m23bf_publish_returns == m_m23bf_attempts", cpp)
        self.assertIn("m_m23bf_attempts <= 5000", cpp)
        self.assertIn("m_m23bf_attempts >= 3", cpp)
        self.assertIn("M23BF_FATAL=INCOMPLETE_PUBLISHER_TEARDOWN", cpp)

    def test_existing_m23ax_fatal_cap_unchanged(self):
        cpp = m.instrument(fixtures())[m.FILES[0]]
        self.assertIn("M23AX_FATAL=METADATA_LOG_LIMIT", cpp)
        self.assertIn("m_m23bf_attempts == 5001", cpp)

    def test_original_mjpeg_branch_is_not_changed(self):
        src = fixtures()
        fragment = """bool UsbCamNode::take_and_send_image_mjpeg()
{
  m_compressed_image_publisher->publish(*m_compressed_img_msg);
  return true;
}
"""
        src[m.FILES[0]] += fragment
        new = m.instrument(src)
        self.assertIn(fragment, new[m.FILES[0]])

    def test_cannot_patch_same_source_twice(self):
        once = m.instrument(fixtures())
        with self.assertRaisesRegex(m.Blocked, "already instrumented"):
            m.instrument(once)

    def test_require_exact_file_set(self):
        src = fixtures()
        src["unknown.cpp"] = ""
        with self.assertRaisesRegex(m.Blocked, "exact node"):
            m.instrument(src)

    def test_uninstrumented_publisher_is_blocked(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace(
            "M23AX_PUBLISH pub_index=", "WRONG")
        with self.assertRaisesRegex(m.Blocked, "pre-send"):
            m.instrument(src)

    def test_removed_m23ax_metadata_cap_blocked(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace(
            "M23AX_FATAL=METADATA_LOG_LIMIT", "WRONG")
        with self.assertRaisesRegex(m.Blocked, "pre-send"):
            m.instrument(src)

    def test_duplicate_m23ax_event_index_consumer_blocked(self):
        src = fixtures()
        src[m.FILES[0]] += "m23ax_publication_no;\n"
        with self.assertRaisesRegex(m.Blocked, "index consumers"):
            m.instrument(src)

    def test_modified_publish_call_blocks(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace(
            "m_image_publisher->publish(", "m_image_publisher->publish_other(")
        with self.assertRaisesRegex(m.Blocked, "pre-send"):
            m.instrument(src)

    def test_changed_destructor_blocks(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace("delete (m_camera);",
                                                   "m_camera = nullptr;")
        with self.assertRaisesRegex(m.Blocked, "node_teardown"):
            m.instrument(src)

    def test_changed_timer_order_blocks(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace(
            "  m_timer.reset();", "  m_timer.cancel();")
        with self.assertRaisesRegex(m.Blocked, "node_teardown"):
            m.instrument(src)

    def test_changed_member_layout_blocks(self):
        src = fixtures()
        src[m.FILES[1]] = src[m.FILES[1]].replace(
            "m_parameters_callback_handle;", "different_handle;")
        with self.assertRaisesRegex(m.Blocked, "node_state_members"):
            m.instrument(src)

    def test_missing_update_blocks(self):
        src = fixtures()
        src[m.FILES[0]] = src[m.FILES[0]].replace(
            "void UsbCamNode::update()", "void UsbCamNode::other()")
        with self.assertRaisesRegex(m.Blocked, "pinned timer callback"):
            m.instrument(src)

    def test_wrong_source_version_blocks(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "package.xml").write_text("<version>9.9.9</version>")
            with self.assertRaisesRegex(m.Blocked, "version changed"):
                m.check_tree(root)

    def test_source_symlink_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            other = root / "target"
            other.mkdir()
            link = root / "link"
            link.symlink_to(other)
            with self.assertRaisesRegex(m.Blocked, "symlinked"):
                m.check_tree(link)

    def test_host_check_cannot_be_redirected(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main(["--check", "--source", "/home/unreviewed"]), 2)
        self.assertIn("M23BF_CAMERA_RETEST_AUTHORIZED=FALSE", out.getvalue())

    def test_transform_never_writes_originals(self):
        src = fixtures()
        original = dict(src)
        m.instrument(src)
        self.assertEqual(src, original)

    def test_double_instrumentation_marker_blocks(self):
        src = fixtures()
        src[m.FILES[0]] += "M23BF_FORGED\n"
        with self.assertRaisesRegex(m.Blocked, "already instrumented"):
            m.instrument(src)


if __name__ == "__main__":
    unittest.main()
