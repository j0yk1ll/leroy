"""Stage 4C bounded rosbag storage evidence; no ROS or hardware imports."""
import hashlib
import json
from pathlib import Path

from live_stage4c_capture_contract import CaptureError


def capture_hashes(bag_dir):
    """SHA256 ordinary rosbag2 files, never following symlinks."""
    result = {}
    files = list(bag_dir.iterdir())
    if len(files) > 16:
        raise CaptureError("unexpected_output_file_count")
    for path in sorted(files):
        if path.is_symlink() or not path.is_file():
            raise CaptureError("unsafe_output_file")
        if path.name not in ("metadata.yaml",) and not path.name.endswith(".db3"):
            raise CaptureError("unexpected_output_filename")
        digest = hashlib.sha256()
        with path.open("rb") as fp:
            for block in iter(lambda: fp.read(1024 * 1024), b""):
                digest.update(block)
        result[path.name] = digest.hexdigest()
    if not result or "metadata.yaml" not in result or not any(k.endswith(".db3") for k in result):
        raise CaptureError("incomplete_rosbag_storage")
    return result


def write_capture_summary(name, manifest, *, root=Path("/capture")):
    """Write an exclusive, non-overwriting summary beside the bag."""
    out = root / (name + "-capture-summary.json")
    if out.exists() or out.is_symlink():
        raise CaptureError("summary_already_exists")
    with out.open("x", encoding="utf8") as fp:
        json.dump(manifest, fp, sort_keys=True)
        fp.write("\n")
    return out
