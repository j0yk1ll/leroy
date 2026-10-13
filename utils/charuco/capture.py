#!/usr/bin/env python3
"""
capture.py

Interactive image-capture helper for ChArUco camera calibration.

Controls:
  SPACE / C   Capture current frame
  Q / ESC     Quit

Features:
  - Select a camera device index or video source.
  - Request a capture resolution and FPS.
  - Live preview with capture count and sharpness estimate.
  - Saves lossless PNG frames with sequential filenames.
  - Avoids accidental rapid duplicate captures.

Install:
    python -m pip install opencv-contrib-python numpy

Examples:
    python capture.py --output-dir calibration_images --camera 0
    python capture.py --output-dir calibration_images --camera 0 --width 1280 --height 720
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path


def configure_qt_environment() -> None:
    if os.environ.get("DISPLAY") and not os.environ.get("QT_QPA_PLATFORM"):
        os.environ["QT_QPA_PLATFORM"] = "xcb"

    fontdir = os.environ.get("QT_QPA_FONTDIR")
    if fontdir and os.path.isdir(fontdir):
        return

    for font_dir in (
        "/usr/share/fonts/TTF",
        "/usr/share/fonts/truetype/dejavu",
        "/usr/share/fonts/truetype",
        "/usr/share/fonts",
    ):
        if os.path.isdir(font_dir):
            os.environ["QT_QPA_FONTDIR"] = font_dir
            return


configure_qt_environment()

import cv2
import numpy as np

configure_qt_environment()


COMMON_CAMERA_FOURCCS = ("MJPG", "YUYV", "YUY2")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture ChArUco calibration images interactively.")
    p.add_argument("--output-dir", type=Path, default=Path("calibration_images"))
    p.add_argument("--camera", default="0", help="Camera index such as 0, or a video source path.")
    p.add_argument("--width", type=int, default=None, help="Requested capture width.")
    p.add_argument("--height", type=int, default=None, help="Requested capture height.")
    p.add_argument("--fps", type=float, default=None, help="Requested capture FPS.")
    p.add_argument(
        "--backend",
        choices=("auto", "default", "v4l2"),
        default="auto",
        help="Capture backend. Auto prefers V4L2 for Linux camera devices.",
    )
    p.add_argument(
        "--fourcc",
        default=None,
        help="Optional camera pixel format such as MJPG, YUYV, or YUY2.",
    )
    p.add_argument("--min-delay", type=float, default=0.5, help="Minimum seconds between captures.")
    p.add_argument("--prefix", default="calib", help="Output filename prefix.")
    p.add_argument(
        "--sharpness-warn",
        type=float,
        default=80.0,
        help="Warn if Laplacian-variance sharpness falls below this value.",
    )
    return p.parse_args()


def parse_camera_source(value: str):
    try:
        return int(value)
    except ValueError:
        return value


def is_linux_camera_source(raw_value: str, source) -> bool:
    return os.name == "posix" and (
        isinstance(source, int) or raw_value.startswith("/dev/video")
    )


def normalize_fourcc(value: str | None) -> str | None:
    if value is None:
        return None
    fourcc = value.strip().upper()
    if not fourcc:
        return None
    if len(fourcc) != 4:
        raise ValueError("--fourcc must be exactly four characters, e.g. MJPG or YUYV.")
    return fourcc


def open_video_capture(source, use_v4l2: bool):
    if use_v4l2:
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        backend = "V4L2"
        if cap.isOpened():
            return cap, backend
        cap.release()

    cap = cv2.VideoCapture(source)
    return cap, "default"


def apply_capture_settings(cap, width: int | None, height: int | None, fps: float | None, fourcc: str | None) -> None:
    if fourcc is not None:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if width is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps is not None:
        cap.set(cv2.CAP_PROP_FPS, fps)


def capture_mode(cap) -> tuple[int, int, float, str]:
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_value = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_value >> 8 * i) & 0xFF) for i in range(4)).strip()
    return width, height, fps, fourcc


def requested_size_matches(width: int, height: int, requested_w: int | None, requested_h: int | None) -> bool:
    if requested_w is not None and width != requested_w:
        return False
    if requested_h is not None and height != requested_h:
        return False
    return True


def open_configured_capture(args: argparse.Namespace):
    source = parse_camera_source(args.camera)
    requested_fourcc = normalize_fourcc(args.fourcc)
    prefer_v4l2 = args.backend == "v4l2" or (
        args.backend == "auto" and is_linux_camera_source(args.camera, source)
    )

    fourcc_attempts: list[str | None] = [requested_fourcc]
    if requested_fourcc is None and (args.width is not None or args.height is not None) and prefer_v4l2:
        fourcc_attempts.extend(COMMON_CAMERA_FOURCCS)

    seen: set[str | None] = set()
    attempts: list[dict] = []
    fallback = None

    for fourcc in fourcc_attempts:
        if fourcc in seen:
            continue
        seen.add(fourcc)

        cap, backend = open_video_capture(source, prefer_v4l2)
        if not cap.isOpened():
            cap.release()
            continue

        apply_capture_settings(cap, args.width, args.height, args.fps, fourcc)
        actual_w, actual_h, actual_fps, actual_fourcc = capture_mode(cap)
        attempt = {
            "backend": backend,
            "fourcc": fourcc,
            "actual_w": actual_w,
            "actual_h": actual_h,
            "actual_fps": actual_fps,
            "actual_fourcc": actual_fourcc,
        }
        attempts.append(attempt)

        if fallback is None:
            fallback = (cap, attempt)
        elif requested_size_matches(actual_w, actual_h, args.width, args.height):
            fallback[0].release()
            return cap, attempt, attempts

        if requested_size_matches(actual_w, actual_h, args.width, args.height):
            return cap, attempt, attempts

        cap.release()

    if fallback is None:
        cap, backend = open_video_capture(source, False)
        if cap.isOpened():
            apply_capture_settings(cap, args.width, args.height, args.fps, requested_fourcc)
            actual_w, actual_h, actual_fps, actual_fourcc = capture_mode(cap)
            attempt = {
                "backend": backend,
                "fourcc": requested_fourcc,
                "actual_w": actual_w,
                "actual_h": actual_h,
                "actual_fps": actual_fps,
                "actual_fourcc": actual_fourcc,
            }
            return cap, attempt, attempts + [attempt]

    if fallback is None:
        raise RuntimeError(f"Could not open camera/video source: {args.camera}")

    return fallback[0], fallback[1], attempts


def next_index(output_dir: Path, prefix: str) -> int:
    max_idx = 0
    for p in output_dir.glob(f"{prefix}_*.png"):
        try:
            idx = int(p.stem.split("_")[-1])
            max_idx = max(max_idx, idx)
        except ValueError:
            pass
    return max_idx + 1


def sharpness_score(frame: np.ndarray) -> float:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def draw_overlay(frame: np.ndarray, captured: int, sharpness: float, warn_threshold: float):
    display = frame.copy()
    h, w = display.shape[:2]

    lines = [
        f"Captured: {captured}",
        f"Resolution: {w}x{h}",
        f"Sharpness: {sharpness:.1f}",
        "SPACE/C = capture   Q/ESC = quit",
    ]
    if sharpness < warn_threshold:
        lines.append("WARNING: frame may be blurry")

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.65
    thickness = 1
    padding = 12
    line_gap = 8

    metrics = [cv2.getTextSize(text, font, font_scale, thickness) for text in lines]
    max_text_w = max(size[0] for size, _ in metrics)
    max_text_h = max(size[1] for size, _ in metrics)
    max_baseline = max(baseline for _, baseline in metrics)
    line_step = max_text_h + max_baseline + line_gap

    box_x = 10
    box_y = 10
    box_w = min(w - (box_x * 2), max_text_w + (padding * 2))
    box_h = padding * 2 + len(lines) * line_step - line_gap

    overlay = display.copy()
    cv2.rectangle(
        overlay,
        (box_x, box_y),
        (box_x + box_w, box_y + box_h),
        (0, 0, 0),
        thickness=-1,
    )
    cv2.addWeighted(overlay, 0.55, display, 0.45, 0.0, display)

    y = box_y + padding + max_text_h
    for text in lines:
        color = (80, 220, 255) if text.startswith("WARNING:") else (255, 255, 255)
        cv2.putText(
            display,
            text,
            (box_x + padding, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
        y += line_step

    return display


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cap, selected_attempt, attempts = open_configured_capture(args)
    actual_w = selected_attempt["actual_w"]
    actual_h = selected_attempt["actual_h"]
    actual_fps = selected_attempt["actual_fps"]
    actual_fourcc = selected_attempt["actual_fourcc"] or "unknown"

    print(f"Opened source: {args.camera}")
    print(
        f"Actual capture mode: {actual_w}x{actual_h} @ {actual_fps:.2f} FPS "
        f"({actual_fourcc}, {selected_attempt['backend']})"
    )
    if not requested_size_matches(actual_w, actual_h, args.width, args.height):
        requested = "x".join(
            str(value) if value is not None else "any" for value in (args.width, args.height)
        )
        print(f"WARNING: requested {requested}, but the camera reported {actual_w}x{actual_h}.")
        if attempts:
            tried = ", ".join(attempt["fourcc"] or "default" for attempt in attempts)
            print(f"Tried pixel formats: {tried}")
        if args.camera.startswith("/dev/video"):
            print(f"Check supported modes with: v4l2-ctl --device {args.camera} --list-formats-ext")
        print("You can force a supported format with --fourcc, for example --fourcc MJPG or --fourcc YUYV.")
    print(f"Saving images to: {args.output_dir}")
    print("Controls: SPACE/C capture, Q/ESC quit")

    index = next_index(args.output_dir, args.prefix)
    captured = index - 1
    last_capture_time = 0.0

    window_name = "Calibration Capture"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Failed to read frame.")
                break

            sharpness = sharpness_score(frame)
            display = draw_overlay(frame, captured, sharpness, args.sharpness_warn)
            cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key in (ord("c"), ord(" ")):
                now = time.time()
                if now - last_capture_time < args.min_delay:
                    continue

                filename = args.output_dir / f"{args.prefix}_{index:03d}.png"
                if cv2.imwrite(str(filename), frame):
                    print(
                        f"Saved {filename} "
                        f"(sharpness={sharpness:.1f}, size={frame.shape[1]}x{frame.shape[0]})"
                    )
                    index += 1
                    captured += 1
                    last_capture_time = now
                else:
                    print(f"Failed to save {filename}")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"Done. Captured images: {captured}")


if __name__ == "__main__":
    main()
