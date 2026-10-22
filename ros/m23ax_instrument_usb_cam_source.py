#!/usr/bin/env python3
"""M23AX: additive diagnostic instrumentation of PINNED usb_cam 0.8.1 source.

Default is INERT; --check reads three trusted source files without writing.
--write may ONLY write a fresh output copy after explicit, offline-build-only
environment gate. Never modifies the original underlay or production code.
Do not use this patch as an authorization to open a physical camera.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys

FILES = {
    "src/usb_cam.cpp": "camera",
    "include/usb_cam/usb_cam.hpp": "header",
    "src/ros2/usb_cam_node.cpp": "node",
}
SRC = Path("/leroy_ws/src/usb_cam")
DEST = Path("/opt/m23ax-source/usb_cam")
EXPECTED_IMAGE_SOURCE_VERSION = "0.8.1"


class Blocked(ValueError):
    pass


def replace_once(src, old, new, label):
    n = src.count(old)
    if n != 1:
        raise Blocked("source anchor count is not one: " + label + "=" + str(n))
    if "M23AX_" in old:
        raise Blocked("never update already instrumented files")
    return src.replace(old, new, 1)


def instrument(sources):
    if set(sources) != set(FILES):
        raise Blocked("exact three source paths required")
    camera = sources["src/usb_cam.cpp"]
    header = sources["include/usb_cam/usb_cam.hpp"]
    node = sources["src/ros2/usb_cam_node.cpp"]
    if any("M23AX_" in s or "m23ax_" in s for s in sources.values()):
        raise Blocked("source already instrumented: refuse double patch")
    # Compile the library and ROS wrapper from a *single* amended header
    # in a fresh overlay; ABI changes must NEVER mix with old usb_cam.so.
    header = replace_once(
        header, "#include <chrono>\n", "#include <chrono>\n#include <cstdint>\n",
        "header_cstdint")
    header = replace_once(
        header,
        "  inline timespec get_image_timestamp()\n"
        "  {\n    return m_image.stamp;\n  }",
        "  inline timespec get_image_timestamp()\n"
        "  {\n    return m_image.stamp;\n  }\n\n"
        "  // M23AX: evidence only, does not change camera driver output.\n"
        "  struct M23AXAcquisition {\n"
        "    bool fresh = false;\n"
        "    uint32_t v4l2_sequence = 0;\n"
        "    int64_t v4l2_timestamp_us = -1;\n"
        "    uint64_t successful_dequeues = 0;\n"
        '    const char * status = "not_started";\n'
        "  };\n"
        "  M23AXAcquisition m23ax_last_acquisition() const\n"
        "  {\n    return m_m23ax_acquisition;\n  }",
        "header_accessor")
    header = replace_once(
        header, "  std::vector<capture_format_t> m_supported_formats;",
        "  std::vector<capture_format_t> m_supported_formats;\n"
        "  M23AXAcquisition m_m23ax_acquisition;",
        "header_member")
    camera = replace_once(
        camera, "void UsbCam::grab_image()\n{\n",
        "void UsbCam::grab_image()\n{\n"
        "  m_m23ax_acquisition.fresh = false;\n"
        '  m_m23ax_acquisition.status = "await_select";\n',
        "grab_begin")
    camera = replace_once(
        camera, "    if (EINTR == errno) {\n"
        "      // interruped (e.g. maybe Ctrl + c) so don't throw anything\n"
        "      return;",
        "    if (EINTR == errno) {\n"
        '      m_m23ax_acquisition.status = "select_eintr";\n'
        "      // interruped (e.g. maybe Ctrl + c) so don't throw anything\n"
        "      return;",
        "grab_eintr")
    camera = replace_once(
        camera, "  read_frame();\n}\n\n// enables/disables auto focus",
        '  m_m23ax_acquisition.status = "select_ready";\n'
        "  read_frame();\n}\n\n// enables/disables auto focus",
        "grab_read")
    camera = replace_once(
        camera, "      // Get current v4l2 pixel format\n"
        "      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {\n"
        "        switch (errno) {\n"
        "          case EAGAIN:\n"
        "            return;",
        "      // Get current v4l2 pixel format\n"
        "      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_FMT), &m_image.v4l2_fmt)) {\n"
        "        switch (errno) {\n"
        "          case EAGAIN:\n"
        '            m_m23ax_acquisition.status = "mmap_g_fmt_eagain";\n'
        "            return;",
        "mmap_g_fmt_eagain")
    camera = replace_once(
        camera, "      /// Dequeue buffer with the new image\n"
        "      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {\n"
        "        switch (errno) {\n"
        "          case EAGAIN:\n"
        "            return;",
        "      /// Dequeue buffer with the new image\n"
        "      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {\n"
        "        switch (errno) {\n"
        "          case EAGAIN:\n"
        '            m_m23ax_acquisition.status = "mmap_dqbuf_eagain";\n'
        "            return;",
        "mmap_dqbuf_eagain")
    camera = replace_once(
        camera, '        throw std::runtime_error("Unable to exchange buffer with the driver");\n'
        "      }\n      return;\n"
        "    case io_method_t::IO_METHOD_USERPTR:",
        '        throw std::runtime_error("Unable to exchange buffer with the driver");\n'
        "      }\n"
        "      // Report only *successfully processed and requeued* V4L2 buffers.\n"
        "      m_m23ax_acquisition.fresh = true;\n"
        '      m_m23ax_acquisition.status = "mmap_dequeued";\n'
        "      m_m23ax_acquisition.v4l2_sequence = buf.sequence;\n"
        "      m_m23ax_acquisition.v4l2_timestamp_us =\n"
        "        static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000LL +\n"
        "        static_cast<int64_t>(buf.timestamp.tv_usec);\n"
        "      ++m_m23ax_acquisition.successful_dequeues;\n"
        "      return;\n"
        "    case io_method_t::IO_METHOD_USERPTR:",
        "mmap_success")
    # Log *before* publish with no JPEG/RGB payload and no publish change.
    # Each record includes the event index, kernel sequence/timestamp,
    # whether dequeue succeeded, and the ROS header stamp. Observer M23K
    # records separate raw/rect callback stamps and steady receipt times.
    node = replace_once(
        node, "  *m_camera_info_msg = m_camera_info->getCameraInfo();\n"
        "  m_camera_info_msg->header = m_image_msg->header;\n"
        "  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);",
        "  // M23AX: bounded, metadata-only publication/aquisition event log.\n"
        "  static uint32_t m23ax_publication_no = 0;\n"
        "  ++m23ax_publication_no;\n"
        "  if (m23ax_publication_no <= 5000) {\n"
        "    const auto evidence = m_camera->m23ax_last_acquisition();\n"
        '    std::cerr << "M23AX_PUBLISH pub_index=" << m23ax_publication_no\n'
        '      << " fresh=" << (evidence.fresh ? 1 : 0)\n'
        '      << " status=" << evidence.status\n'
        '      << " dequeue_count=" << evidence.successful_dequeues\n'
        '      << " v4l2_seq=" << evidence.v4l2_sequence\n'
        '      << " v4l2_timestamp_us=" << evidence.v4l2_timestamp_us\n'
        '      << " ros_stamp_ns="\n'
        "      << (static_cast<int64_t>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec)\n"
        "      << std::endl;\n"
        "  } else if (m23ax_publication_no == 5001) {\n"
        '    std::cerr << "M23AX_FATAL=METADATA_LOG_LIMIT" << std::endl;\n'
        "  }\n"
        "  *m_camera_info_msg = m_camera_info->getCameraInfo();\n"
        "  m_camera_info_msg->header = m_image_msg->header;\n"
        "  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);",
        "ros_publisher_before_publish")
    return {
        "src/usb_cam.cpp": camera,
        "include/usb_cam/usb_cam.hpp": header,
        "src/ros2/usb_cam_node.cpp": node,
    }


def check_source_tree(root):
    if root.is_symlink() or not root.is_dir():
        raise Blocked("source root missing or symlink")
    pkg = root / "package.xml"
    if not pkg.is_file() or pkg.is_symlink():
        raise Blocked("package manifest missing")
    manifest = pkg.read_text()
    if manifest.count("<version>" + EXPECTED_IMAGE_SOURCE_VERSION + "</version>") != 1:
        raise Blocked("wrong usb_cam package version")
    code = {}
    for rel in FILES:
        path = root / rel
        if not path.is_file() or path.is_symlink():
            raise Blocked("required source path missing/symlink: " + rel)
        if path.stat().st_size > 200_000:
            raise Blocked("source file unexpectedly large: " + rel)
        code[rel] = path.read_text()
    return code


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true")
    p.add_argument("--write", action="store_true")
    p.add_argument("--source", type=Path, default=SRC)
    p.add_argument("--dest", type=Path, default=DEST)
    args = p.parse_args(argv)
    if not args.check and not args.write:
        print("M23AX_STATUS=INERT_NO_ROS_NO_DEVICE_BY_DEFAULT", flush=True)
        return 0
    try:
        if args.check and args.write:
            raise Blocked("choose check OR write")
        changed = instrument(check_source_tree(args.source))
        if args.write:
            if os.environ.get("M23AX_OFFLINE_BUILD_ONLY") != "YES":
                raise Blocked("offline-build-only gate absent")
            if args.source != SRC or args.dest != DEST:
                raise Blocked("only pinned-image source and isolated destination permitted")
            if args.dest.exists() or args.dest.is_symlink():
                raise Blocked("destination exists")
            # Copy the full package once for a future disconnected build.
            # Does not build or execute ROS and cannot access a device.
            shutil.copytree(args.source, args.dest, symlinks=True)
            for rel, value in changed.items():
                (args.dest / rel).write_text(value)
            print("M23AX_SOURCE_OVERLAY_WRITTEN=ISOLATED_COPY_ONLY", flush=True)
        print("M23AX_RESULT_JSON=" + json.dumps({
            "verdict": "INSTRUMENTATION_SOURCE_ANCHORS_MATCH",
            "source_files": sorted(changed),
            "publisher_log_prefix": "M23AX_PUBLISH",
            "max_publisher_metadata_records": 5000,
            "v4l2_statuses": ["select_eintr", "mmap_g_fmt_eagain",
                              "mmap_dqbuf_eagain", "mmap_dequeued"],
            "original_package_modified": False,
            "image_pixels_logged": False,
            "live_behavior_compiled_or_verified": False,
            "physical_camera_retest_authorized": False,
            "production_acceptance": "BLOCKED",
        }, sort_keys=True), flush=True)
        print("M23AX_VERDICT=INSTRUMENTATION_SOURCE_ANCHORS_MATCH", flush=True)
        return 0
    except (Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23AX_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:170], flush=True)
        print("M23AX_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
