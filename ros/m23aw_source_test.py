"""M23AW: 11 completely offline source control-flow tests. No camera/ROS."""
import unittest
from unittest.mock import patch

import m23aw_stale_stamp_source_audit as m


def fixtures():
    grab = '''
void UsbCam::grab_image()
{
  r = select(m_fd + 1, &fds, NULL, NULL, &tv);
  if (-1 == r) {
    if (EINTR == errno) {
      // interrupted, leave with previous frame contents
      return;
    }
    throw errno;
  }
  read_frame();
}
// enables/disables auto focus
'''
    reader = '''
void UsbCam::read_frame()
{
  switch (m_io) {
    case io_method_t::IO_METHOD_MMAP:
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default: throw 1;
        }
      }
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);
      break;
    case io_method_t::IO_METHOD_USERPTR:
      break;
  }
}
void UsbCam::start_capturing() {}
'''
    getter = '''
void UsbCam::get_image(char * destination)
{
  m_image.data = destination;
  grab_image();
}
std::vector<capture_format_t> UsbCam::get_supported_formats() {}
'''
    publish = '''
bool UsbCamNode::take_and_send_image()
{
  m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));
  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;
  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  return true;
}
bool UsbCamNode::take_and_send_image_mjpeg() {return false;}
'''
    return grab + reader + getter, publish


class Tests(unittest.TestCase):
    def test_default_inert(self):
        with patch.object(m, "inspect", side_effect=AssertionError("audit ran")):
            self.assertEqual(m.main([]), 0)

    def test_fixture_matches_potential_reuse(self):
        cpp, node = fixtures()
        report = m.inspect(cpp, node)
        self.assertTrue(report["get_image_does_not_report_freshness"])
        self.assertFalse(report["usb_cam_published_duplicate_proven"])

    def test_select_interruption_path_required(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "select"):
            m.inspect(cpp.replace("EINTR == errno", "EIO == errno"), node)

    def test_dqbuf_nonblocking_path_required(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "dequeued"):
            m.inspect(cpp.replace("case EAGAIN:", "case EINVAL:"), node)

    def test_timestamp_from_dequeued_buffer_required(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "timestamp"):
            m.inspect(cpp.replace("m_image.stamp = usb_cam::utils::calc_img_timestamp",
                                  "auto current = usb_cam::utils::calc_img_timestamp"), node)

    def test_image_getter_assigns_destination(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "get_image caller"):
            m.inspect(cpp.replace("m_image.data = destination;", "dest = destination;"), node)

    def test_publisher_gets_current_stored_stamp(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "publish/get_image/stamp"):
            m.inspect(cpp.replace("get_image_timestamp()", "get_monotonic_time()"), node)

    def test_publisher_requires_both_header_parts(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "cached timestamp"):
            m.inspect(cpp, node.replace("m_image_msg->header.stamp.nanosec",
                                        "m_image_msg->header.stamp.nsec"))

    def test_publisher_guard_change_blocks_inference(self):
        cpp, node = fixtures()
        node = node.replace("auto stamp = ", "if (!fresh_frame) return false;\n  auto stamp = ")
        with self.assertRaisesRegex(m.Blocked, "new conditional"):
            m.inspect(cpp, node)

    def test_missing_unique_function_blocks(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "source function"):
            m.inspect(cpp.replace("void UsbCam::grab_image()", "void UsbCam::acquire()"), node)

    def test_duplicate_function_blocks(self):
        cpp, node = fixtures()
        with self.assertRaisesRegex(m.Blocked, "source function"):
            m.inspect(cpp + "\nvoid UsbCam::grab_image() {}", node)


if __name__ == "__main__":
    unittest.main()
