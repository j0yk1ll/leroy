#!/usr/bin/env python3
"""M23BF: SOURCE ONLY, no-write publisher completion patch for M23AX usb_cam.

Requires an already instrumented usb_cam 0.8.1 source COPY from immutable
M23AY image. Does not touch original sources/install, start ROS, or access
camera. The --check mode only reads and validates file anchors.
"""
import argparse
import json
from pathlib import Path
import sys

SOURCE = Path("/opt/m23ax-source/usb_cam")
FILES = (
    "src/ros2/usb_cam_node.cpp",
    "include/usb_cam/usb_cam_node.hpp",
)
VERSION = "0.8.1"
MAX_BYTES = 160_000


class Blocked(ValueError):
    pass


def once(text, anchor, replacement, name):
    occurrences = text.count(anchor)
    if occurrences != 1:
        raise Blocked("expected one pinned source anchor: " +
                      name + "=" + str(occurrences))
    return text.replace(anchor, replacement, 1)


def instrument(sources):
    if set(sources) != set(FILES):
        raise Blocked("exact node implementation/header source pair required")
    cpp = sources[FILES[0]]
    header = sources[FILES[1]]
    if any(token in cpp or token in header for token in
           ("M23BF_", "m_m23bf_", "M23BE_PUBLISHER_FINAL")):
        raise Blocked("refuse already instrumented completion source")
    for token in ("M23AX_PUBLISH pub_index=", "M23AX_FATAL=METADATA_LOG_LIMIT",
                  "m_camera->m23ax_last_acquisition()",
                  "m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);"):
        if cpp.count(token) != 1:
            raise Blocked("missing or duplicate M23AX pre-send anchor: " + token)
    # Only the uncompressed rgb path is instrumented by M23AX. The M23AQ
    # camera uses pixel_format=mjpeg2rgb and therefore calls this branch.
    if cpp.count("bool UsbCamNode::take_and_send_image()\n{") != 1:
        raise Blocked("wrong M23AX camera callback contract")
    if cpp.count("void UsbCamNode::update()\n{") != 1:
        raise Blocked("expected pinned timer callback absent")

    header = once(header, "#include <memory>\n",
                  "#include <cstdint>\n#include <memory>\n", "node_header_cstdint")
    members = (
        "  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr "
        "m_parameters_callback_handle;"
    )
    header = once(header, members, members + "\n"
                  "  // M23BF: diagnostic counts are per node, not static across nodes.\n"
                  "  std::uint32_t m_m23bf_attempts = 0;\n"
                  "  std::uint32_t m_m23bf_publish_returns = 0;",
                  "node_state_members")
    cpp = once(cpp,
               "  static uint32_t m23ax_publication_no = 0;\n"
               "  ++m23ax_publication_no;",
               "  ++m_m23bf_attempts;",
               "per_node_attempt_increment")
    if cpp.count("m23ax_publication_no") != 3:
        raise Blocked("unexpected M23AX publication index consumers")
    cpp = cpp.replace("m23ax_publication_no", "m_m23bf_attempts")

    # Increment only *after* image_transport::CameraPublisher::publish
    # returned normally. No return means no increment; no DDS ack implied.
    cpp = once(cpp,
               "  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);\n"
               "  return true;",
               "  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);\n"
               "  ++m_m23bf_publish_returns;\n"
               "  return true;",
               "publish_return_accounting")

    destructor = (
        "  m_timer.reset();\n"
        "  m_service_capture.reset();\n"
        "  m_parameters_callback_handle.reset();\n\n"
        "  delete (m_camera);\n"
        "}"
    )
    amended = (
        "  m_timer.reset();\n"
        "  m_service_capture.reset();\n"
        "  m_parameters_callback_handle.reset();\n\n"
        "  delete (m_camera);\n"
        "  // M23BF: teardown epilogue follows camera and timer destruction.\n"
        "  // Clean ROS child process exit must STILL be independently checked.\n"
        "  // This is NOT a DDS delivery acknowledgment.\n"
        "  if (m_m23bf_attempts >= 3 && m_m23bf_attempts <= 5000 &&\n"
        "      m_m23bf_publish_returns == m_m23bf_attempts) {\n"
        '    std::cerr << "M23BE_PUBLISHER_FINAL attempts="\n'
        "              << m_m23bf_attempts\n"
        '              << " publish_returns=" << m_m23bf_publish_returns\n'
        '              << " shutdown=clean" << std::endl;\n'
        "  } else {\n"
        '    std::cerr << "M23BF_FATAL=INCOMPLETE_PUBLISHER_TEARDOWN"\n'
        "              << std::endl;\n"
        "  }\n"
        "}"
    )
    cpp = once(cpp, destructor, amended, "node_teardown_epilogue")
    if cpp.count("M23BE_PUBLISHER_FINAL attempts=") != 1:
        raise Blocked("missing unique final marker after transform")
    if cpp.count("M23BF_FATAL=INCOMPLETE_PUBLISHER_TEARDOWN") != 1:
        raise Blocked("missing teardown count rejection")
    if cpp.count("++m_m23bf_publish_returns;") != 1:
        raise Blocked("missing unique publish return count")
    if cpp.count("++m_m23bf_attempts;") != 1:
        raise Blocked("missing unique attempted publish increment")
    if len(cpp) > MAX_BYTES or len(header) > MAX_BYTES:
        raise Blocked("generated source size exceeded")
    return {FILES[0]: cpp, FILES[1]: header}


def check_tree(root):
    if root.is_symlink() or not root.is_dir():
        raise Blocked("pinned source directory missing/symlinked")
    manifest = root / "package.xml"
    if manifest.is_symlink() or not manifest.is_file():
        raise Blocked("package.xml unavailable or symlinked")
    if manifest.read_text().count("<version>" + VERSION + "</version>") != 1:
        raise Blocked("pinned usb_cam source version changed")
    src = {}
    for rel in FILES:
        path = root / rel
        if path.is_symlink() or not path.is_file():
            raise Blocked("missing or symlinked source file " + rel)
        if not 0 < path.stat().st_size <= MAX_BYTES:
            raise Blocked("source file exceeds safe size " + rel)
        src[rel] = path.read_text(encoding="utf-8")
    return src


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true")
    p.add_argument("--source", type=Path, default=SOURCE)
    args = p.parse_args(argv)
    if not args.check:
        print("M23BF_STATUS=INERT_NO_WRITES_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        if args.source != SOURCE:
            raise Blocked("only immutable pinned diagnostic source root permitted")
        changed = instrument(check_tree(args.source))
        print("M23BF_RESULT_JSON=" + json.dumps({
            "verdict": "PINNED_PUBLISHER_COMPLETION_SOURCE_ANCHORS_MATCH",
            "files_checked": sorted(changed),
            "publisher_return_is_not_dds_ack": True,
            "no_source_files_modified": True,
            "future_rebuild_of_node_libraries_required": True,
            "camera_node_teardown_not_actual_process_exit_proof": True,
            "camera_retest_authorized": False,
            "production_acceptance": "BLOCKED",
        }, sort_keys=True), flush=True)
        print("M23BF_SOURCE_VERDICT=SOURCE_PATCH_ANCHORS_MATCH_NO_WRITES", flush=True)
        print("M23BF_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23BF_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:170],
              flush=True)
        print("M23BF_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
