"""M23AX source-only synthetic contract tests. No device, Podman, ROS."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23ax_instrument_usb_cam_source as m


def fixtures():
    header = """#include <chrono>
class UsbCam {
 public:
  inline timespec get_image_timestamp()
  {
    return m_image.stamp;
  }
 private:
  std::vector<capture_format_t> m_supported_formats;
};
"""
    camera = """void UsbCam::read_frame()
{
    case io_method_t::IO_METHOD_MMAP:
      // Get current v4l2 pixel format
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {
        switch (errno) {
          case EAGAIN:
            return;
        }
      }
      /// Dequeue buffer with the new image
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
        }
      }
      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);
      process_image(m_buffers[buf.index].start, m_image.data, buf.bytesused);
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with the driver");
      }
      return;
    case io_method_t::IO_METHOD_USERPTR:
}
void UsbCam::grab_image()
{
  if (-1 == r) {
    if (EINTR == errno) {
      // interruped (e.g. maybe Ctrl + c) so don't throw anything
      return;
    }
  }
  read_frame();
}

// enables/disables auto focus
"""
    node = """#include <memory>
bool UsbCamNode::take_and_send_image()
{
  m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));
  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  return true;
}
"""
    return {
        "src/usb_cam.cpp": camera,
        "include/usb_cam/usb_cam.hpp": header,
        "src/ros2/usb_cam_node.cpp": node,
    }


class Tests(unittest.TestCase):
    def test_inert_by_default(self):
        with patch.object(m, "check_source_tree",
                          side_effect=AssertionError("device or source accessed")):
            self.assertEqual(m.main([]), 0)

    def test_expected_three_file_patching(self):
        originals = fixtures()
        changed = m.instrument(originals)
        self.assertEqual(set(changed), set(m.FILES))
        self.assertIn('m_m23ax_acquisition.status = "mmap_dequeued"', changed["src/usb_cam.cpp"])
        self.assertIn('m_m23ax_acquisition.status = "select_eintr"', changed["src/usb_cam.cpp"])
        self.assertIn('m_m23ax_acquisition.status = "mmap_dqbuf_eagain"', changed["src/usb_cam.cpp"])
        self.assertIn("M23AXAcquisition m_m23ax_acquisition", changed["include/usb_cam/usb_cam.hpp"])
        self.assertIn('M23AX_PUBLISH pub_index=', changed["src/ros2/usb_cam_node.cpp"])
        self.assertIn("m_camera_info_msg->header = m_image_msg->header;", changed["src/ros2/usb_cam_node.cpp"])

    def test_double_instrumentation_refused(self):
        with self.assertRaisesRegex(m.Blocked, "already instrumented"):
            m.instrument(m.instrument(fixtures()))

    def test_refuse_extra_or_missing_files(self):
        originals = fixtures()
        originals.pop("src/usb_cam.cpp")
        with self.assertRaisesRegex(m.Blocked, "exact three"):
            m.instrument(originals)

    def test_mutated_mmap_dequeue_refused(self):
        originals = fixtures()
        key = "src/usb_cam.cpp"
        self.assertIn("VIDIOC_DQBUF", originals[key])
        originals[key] = originals[key].replace("VIDIOC_DQBUF", "VIDIOC_WRONG")
        with self.assertRaisesRegex(m.Blocked, "mmap dequeue"):
            m.instrument(originals)

    def test_mutated_mmap_requeue_refused(self):
        originals = fixtures()
        key = "src/usb_cam.cpp"
        self.assertIn("VIDIOC_QBUF", originals[key])
        originals[key] = originals[key].replace("VIDIOC_QBUF", "VIDIOC_NOT_QBUF")
        # The patch has no executable anchor for VIDIOC_QBUF (it appears
        # in a preceding line); this source contract test must detect that.
        with self.assertRaisesRegex(m.Blocked, "mmap"):
            m.instrument(originals)

    def test_mutated_eintr_path_refused(self):
        originals = fixtures()
        key = "src/usb_cam.cpp"
        originals[key] = originals[key].replace("EINTR == errno", "EIO == errno")
        with self.assertRaisesRegex(m.Blocked, "grab_eintr"):
            m.instrument(originals)

    def test_mutated_ros_publisher_refused(self):
        originals = fixtures()
        key = "src/ros2/usb_cam_node.cpp"
        originals[key] = originals[key].replace(
            "m_image_publisher->publish", "m_image_publisher->publish_once")
        with self.assertRaisesRegex(m.Blocked, "ros_publisher_before_publish"):
            m.instrument(originals)

    def test_mutated_header_timestamp_accessor_refused(self):
        originals = fixtures()
        key = "include/usb_cam/usb_cam.hpp"
        originals[key] = originals[key].replace("return m_image.stamp;", "return now();")
        with self.assertRaisesRegex(m.Blocked, "header_accessor"):
            m.instrument(originals)

    def test_altered_anchor_duplicate_refused(self):
        originals = fixtures()
        key = "src/usb_cam.cpp"
        original = originals[key]
        anchor = "void UsbCam::grab_image()\n{\n"
        self.assertIn(anchor, original)
        originals[key] = original + anchor
        with self.assertRaisesRegex(m.Blocked, "grab_begin"):
            m.instrument(originals)

    def test_write_gate_cannot_write_to_host(self):
        with patch.object(m, "check_source_tree", return_value=fixtures()):
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(m.main(["--write"]), 2)

    def test_check_reads_only_source(self):
        with patch.object(m, "check_source_tree", return_value=fixtures()):
            self.assertEqual(m.main(["--check"]), 0)

    def test_expected_package_version_gate(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "package.xml").write_text("<version>0.9.0</version>")
            for rel, value in fixtures().items():
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(value)
            with self.assertRaisesRegex(m.Blocked, "wrong usb_cam package version"):
                m.check_source_tree(root)

    def test_missing_manifest_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(m.Blocked, "package manifest"):
                m.check_source_tree(Path(folder))

    def test_metadata_record_cap_exposed(self):
        node = m.instrument(fixtures())["src/ros2/usb_cam_node.cpp"]
        self.assertIn("<= 5000", node)
        self.assertIn("m23ax_publication_no == 5001", node)
        self.assertIn("M23AX_FATAL=METADATA_LOG_LIMIT", node)


if __name__ == "__main__":
    unittest.main()
