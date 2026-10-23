#!/usr/bin/env python3
"""M23AW: inert offline inspection of the PINNED installed usb_cam 0.8.1 SOURCE.

Inspects one plausible stale-frame/reused-stamp control flow:
select EINTR -> grab_image returns without fresh acquisition;
VIDIOC_DQBUF EAGAIN -> read_frame returns without fresh acquisition;
get_image ignores acquisition success and publisher reuses stored stamp.
SOURCE POSSIBILITY only: not proof this happened in live M23AU, nor DDS exclusion.
"""
import argparse
import json
from pathlib import Path
import re
import sys

CPP = Path("/leroy_ws/src/usb_cam/src/usb_cam.cpp")
ROS = Path("/leroy_ws/src/usb_cam/src/ros2/usb_cam_node.cpp")


class Blocked(ValueError):
    pass


def section(src, start, end):
    if src.count(start) != 1 or src.count(end) != 1:
        raise Blocked("missing/duplicate source function marker: " + start)
    return src.split(start, 1)[1].split(end, 1)[0]


def inspect(usb_cpp, node_cpp):
    grab = section(usb_cpp, "void UsbCam::grab_image()", "// enables/disables auto focus")
    read = section(usb_cpp, "void UsbCam::read_frame()", "void UsbCam::start_capturing()")
    getter = section(usb_cpp, "void UsbCam::get_image(char * destination)",
                     "std::vector<capture_format_t> UsbCam::get_supported_formats()")
    publisher = section(node_cpp, "bool UsbCamNode::take_and_send_image()",
                        "bool UsbCamNode::take_and_send_image_mjpeg()")
    # Reject unexpected / changed upstream rather than guessing about its flow.
    if not re.search(
        r"if\s*\(\s*-1\s*==\s*r\s*\)\s*\{\s*"
        r"if\s*\(\s*EINTR\s*==\s*errno\s*\)\s*\{"
        r".*?\breturn\s*;", grab, re.S
    ):
        raise Blocked("interrupted select path no longer returns without acquisition")
    if not re.search(
        r"VIDIOC_DQBUF.*?case\s+EAGAIN\s*:\s*return\s*;",
        read.split("case io_method_t::IO_METHOD_USERPTR", 1)[0], re.S
    ):
        raise Blocked("nonblocking dequeued-frame EAGAIN path changed")
    if not re.search(
        r"VIDIOC_DQBUF.*?m_image\.stamp\s*=\s*"
        r"usb_cam::utils::calc_img_timestamp",
        read.split("case io_method_t::IO_METHOD_USERPTR", 1)[0], re.S
    ):
        raise Blocked("timestamp assignment not linked to mmap dequeue")
    if "m_image.data = destination;" not in getter or        "grab_image();" not in getter or        re.search(r"\bif\s*\(\s*!\s*grab_image", getter):
        raise Blocked("get_image caller control path changed")
    get_idx = publisher.find("m_camera->get_image(")
    ts_idx = publisher.find("m_camera->get_image_timestamp()")
    pub_idx = publisher.find("m_image_publisher->publish(")
    if not (0 <= get_idx < ts_idx < pub_idx):
        raise Blocked("publish/get_image/stamp control path changed")
    if "m_image_msg->header.stamp.sec" not in publisher or        "m_image_msg->header.stamp.nanosec" not in publisher:
        raise Blocked("source no longer assigns cached timestamp to ROS image")
    # There is no return condition based on acquisition success in
    # the intervening get_image->publish source path.
    if re.search(r"\bif\s*\(", publisher[get_idx:pub_idx]):
        raise Blocked("acquisition-to-publish path has new conditional; review manually")
    return {
        "verdict": "STALE_STAMP_SOURCE_CONTROL_FLOW_PLAUSIBLE",
        "select_eintr_returns_without_new_frame": True,
        "mmap_dqbuf_eagain_returns_without_new_frame": True,
        "get_image_does_not_report_freshness": True,
        "publisher_uses_stored_frame_timestamp": True,
        "source_only_audit": True,
        "live_duplicate_explained_or_proven": False,
        "usb_cam_published_duplicate_proven": False,
        "dds_delivery_fault_excluded": False,
        "software_modified": False,
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true")
    flags = p.parse_args(argv)
    if not flags.check:
        print("M23AW_STATUS=INERT_NO_DEVICE_SOURCE_ONLY", flush=True)
        return 0
    try:
        if not CPP.is_file() or not ROS.is_file() or CPP.is_symlink() or ROS.is_symlink():
            raise Blocked("pinned image source paths missing/nonregular")
        report = inspect(CPP.read_text(), ROS.read_text())
        print("M23AW_RESULT_JSON=" + json.dumps(report, sort_keys=True), flush=True)
        print("M23AW_VERDICT=STALE_STAMP_SOURCE_CONTROL_FLOW_PLAUSIBLE", flush=True)
        print("M23AW_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, UnicodeError, ValueError) as e:
        print("M23AW_BLOCKED=" + type(e).__name__ + ":" + str(e)[:175], flush=True)
        print("M23AW_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
