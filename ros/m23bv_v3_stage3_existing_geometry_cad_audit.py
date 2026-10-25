#!/usr/bin/env python3
"""Read-only Stage 3 provenance check for existing FreeCAD/ROS geometry.

Reads the tracked FreeCAD ZIP Document.xml and canonical Xacro; does NOT
launch FreeCAD, read physical devices, move parts, record, or calibrate.
Individual CAD part placements are NOT calibrated camera/IMU extrinsics.
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
STAGE3_BRANCH = "feature/m23bv-v3-stage3-fullstack-diagnostic"
CAD_REL = "hardware/parts.FCStd"
CAD_BLOB = "136b5ab4a14d236f43551addf058ac4eb494870b"
URDF_REL = "ros/src/leroy_description/urdf/leroy.urdf.xacro"
URDF_BLOB = "353e79ac7bd349c0291f7e5c07e5473fde6f9ff6"
MAX_DOCUMENT_BYTES = 64 * 1024 * 1024
MAX_CANDIDATES = 40
SENSOR_LABEL = re.compile(
    r"imu|icm|camera|chassis|base|body|mount|controller|platform|frame|sensor",
    re.IGNORECASE,
)


def run_git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args],
        check=True, capture_output=True, text=True,
    ).stdout.strip()


def pinned_clean_sources() -> None:
    if run_git("branch", "--show-current") != STAGE3_BRANCH:
        raise RuntimeError("REQUIRES_STAGE3_BRANCH")
    if run_git("status", "--porcelain"):
        raise RuntimeError("DIRTY_WORKTREE")
    for rel, expected in ((CAD_REL, CAD_BLOB), (URDF_REL, URDF_BLOB)):
        src = ROOT / rel
        if not src.is_file() or src.is_symlink():
            raise RuntimeError("SOURCE_MISSING_OR_SYMLINK:" + rel)
        if run_git("hash-object", "--", str(src)) != expected:
            raise RuntimeError("SOURCE_BYTES_CHANGED:" + rel)
        if run_git("rev-parse", "HEAD:" + rel) != expected:
            raise RuntimeError("COMMITTED_SOURCE_DRIFT:" + rel)


def load_cad_xml() -> ET.Element:
    file = ROOT / CAD_REL
    if not zipfile.is_zipfile(file):
        raise RuntimeError("FREECAD_NOT_ZIP")
    with zipfile.ZipFile(file) as archive:
        try:
            info = archive.getinfo("Document.xml")
        except KeyError as err:
            raise RuntimeError("FREECAD_DOCUMENT_XML_MISSING") from err
        if info.file_size > MAX_DOCUMENT_BYTES:
            raise RuntimeError("FREECAD_DOCUMENT_XML_OVERSIZE")
        xml = archive.read(info)
        if len(xml) > MAX_DOCUMENT_BYTES:
            raise RuntimeError("FREECAD_XML_OVERSIZE_AFTER_READ")
        root = ET.fromstring(xml)
    print("STAGE3_EXISTING_GEOMETRY_FREECAD_FILE=hardware/parts.FCStd")
    print("STAGE3_EXISTING_GEOMETRY_FREECAD_BLOB=" + CAD_BLOB)
    print("STAGE3_EXISTING_GEOMETRY_XML_BYTES=" + str(len(xml)))
    return root


def property_value(prop: ET.Element) -> str:
    # FreeCAD stores Label as <Property name="Label"><String value="..."/>.
    for child in prop.iter():
        if child is prop:
            continue
        if child.tag.endswith("String"):
            return child.get("value") or (child.text or "")
    return ""


def properties_for_object(obj: ET.Element) -> dict[str, ET.Element]:
    props = obj.find("Properties")
    if props is None:
        return {}
    return {p.get("name", ""): p for p in props.findall("Property")}


def collect_cad_candidates(root: ET.Element) -> list[dict[str, object]]:
    data = root.find(".//ObjectData")
    if data is None:
        raise RuntimeError("FREECAD_OBJECT_DATA_NOT_FOUND")
    objs = data.findall("Object")
    matches = []
    for obj in objs:
        obj_id = obj.get("name", "")
        props = properties_for_object(obj)
        label = property_value(props["Label"]) if "Label" in props else ""
        if not SENSOR_LABEL.search(label + " " + obj_id):
            continue
        placements = []
        for prop_name in ("Placement", "LinkPlacement"):
            if prop_name not in props:
                continue
            for elt in props[prop_name].iter():
                if elt is props[prop_name] or not elt.tag.endswith("Placement"):
                    continue
                numeric = {}
                for key in ("Px", "Py", "Pz", "Q0", "Q1", "Q2", "Q3",
                            "px", "py", "pz", "q0", "q1", "q2", "q3"):
                    if key in elt.attrib:
                        try:
                            numeric[key] = round(float(elt.attrib[key]), 6)
                        except ValueError:
                            continue
                if numeric:
                    placements.append({"property": prop_name, "values": numeric})
        matches.append({
            "object": obj_id[:90],
            "label": label[:100],
            "placements": placements[:2],
        })
    print("STAGE3_EXISTING_GEOMETRY_FREECAD_OBJECT_COUNT=" + str(len(objs)))
    print("STAGE3_EXISTING_GEOMETRY_CANDIDATE_COUNT=" + str(len(matches)))
    return matches


def xacro_joint_origins() -> None:
    tree = ET.parse(ROOT / URDF_REL)
    joints = {joint.get("name"): joint for joint in tree.findall(".//joint")}
    for key in ("base_to_imu", "base_to_front_camera"):
        joint = joints.get(key)
        if joint is None:
            raise RuntimeError("CANONICAL_TF_JOINT_MISSING:" + key)
        origin = joint.find("origin")
        if origin is None:
            raise RuntimeError("CANONICAL_TF_ORIGIN_MISSING:" + key)
        xyz = origin.attrib.get("xyz")
        rpy = origin.attrib.get("rpy")
        print("STAGE3_EXISTING_GEOMETRY_URDF_" + key.upper() + "=" +
              json.dumps({"xyz_m": xyz, "rpy_rad": rpy},
                         sort_keys=True, separators=(",", ":")))
    if ("0.095 -0.095 0.0" != joints["base_to_imu"].find("origin").get("xyz")
            or "0 0 0" != joints["base_to_front_camera"].find("origin").get("xyz")):
        raise RuntimeError("CANONICAL_TF_PLACEHOLDERS_CHANGED")
    print("STAGE3_EXISTING_GEOMETRY_URDF_BLOB=" + URDF_BLOB)
    print("STAGE3_EXISTING_GEOMETRY_URDF_POSITION_VALIDATION=PROVISIONAL")


def main() -> None:
    pinned_clean_sources()
    cad = load_cad_xml()
    matches = collect_cad_candidates(cad)
    for entry in matches[:MAX_CANDIDATES]:
        print("STAGE3_EXISTING_GEOMETRY_CAD_CANDIDATE_JSON=" +
              json.dumps(entry, sort_keys=True, separators=(",", ":")))
    if len(matches) > MAX_CANDIDATES:
        print("STAGE3_EXISTING_GEOMETRY_CAD_CANDIDATES_TRUNCATED=" +
              str(len(matches) - MAX_CANDIDATES))
    if not matches:
        print("STAGE3_EXISTING_GEOMETRY_CAD_CANDIDATES=NONE_BY_LABEL")
    xacro_joint_origins()
    print("STAGE3_EXISTING_GEOMETRY_CAD_PLACEMENTS=MODEL_SPACE_ONLY_UNVERIFIED")
    print("STAGE3_EXISTING_GEOMETRY_CAMERA_OPTICAL_CENTER=NOT_DERIVED")
    print("STAGE3_EXISTING_GEOMETRY_BASE_LINK_ORIGIN=NOT_DERIVED")
    print("STAGE3_EXISTING_GEOMETRY_IMU_DIE_CENTER=NOT_DERIVED")
    print("STAGE3_EXISTING_GEOMETRY=PASS_READ_ONLY_SOURCE_INVENTORY")
    print("STAGE3_EXISTING_GEOMETRY_HARDWARE_ACCESS=NONE")
    print("STAGE3_EXISTING_GEOMETRY_MOTION_RECORDING_MCU=NOT_AUTHORIZED")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, zipfile.BadZipFile, ET.ParseError,
            subprocess.CalledProcessError) as exc:
        print("STAGE3_EXISTING_GEOMETRY=BLOCKED_" +
              type(exc).__name__ + ":" + str(exc), file=sys.stderr)
        sys.exit(2)
