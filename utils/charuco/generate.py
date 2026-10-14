#!/usr/bin/env python3
"""
generate.py

Generate an exact-size DIN A4 ChArUco calibration board PDF and a matching
board_config.json consumed by calibrate.py.

Default board:
  - 5 x 7 chessboard squares
  - 35 mm square length
  - 26 mm ArUco marker length
  - DICT_5X5_100
  - centered on DIN A4 (210 x 297 mm)

Install:
    python -m pip install opencv-contrib-python numpy pillow reportlab pyyaml

Usage:
    python generate.py
    python generate.py --output-dir calibration_board

IMPORTANT:
Print the generated PDF at 100% / "Actual size". Disable "Fit to page".
After printing, measure multiple 35 mm squares with a ruler/caliper.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

from calibrate import create_charuco_board


A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate an exact-size DIN A4 ChArUco board.")
    p.add_argument("--output-dir", type=Path, default=Path("."), help="Output directory.")
    p.add_argument("--squares-x", type=int, default=5, help="Number of chessboard squares in X.")
    p.add_argument("--squares-y", type=int, default=7, help="Number of chessboard squares in Y.")
    p.add_argument("--square-mm", type=float, default=35.0, help="Chessboard square size in mm.")
    p.add_argument("--marker-mm", type=float, default=26.0, help="ArUco marker size in mm.")
    p.add_argument(
        "--dictionary",
        default="DICT_5X5_100",
        help="OpenCV ArUco dictionary name, e.g. DICT_5X5_100.",
    )
    p.add_argument("--dpi", type=int, default=600, help="Rasterization DPI embedded in the PDF.")
    return p.parse_args()


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def mm_to_pt(mm: float) -> float:
    return mm * 72.0 / 25.4


def get_aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is unavailable. Install opencv-contrib-python.")
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def main() -> None:
    args = parse_args()

    if args.squares_x < 2 or args.squares_y < 2:
        raise ValueError("squares-x and squares-y must both be >= 2.")
    if args.square_mm <= 0 or args.marker_mm <= 0:
        raise ValueError("Square and marker sizes must be positive.")
    if args.marker_mm >= args.square_mm:
        raise ValueError("marker-mm must be smaller than square-mm.")

    board_w_mm = args.squares_x * args.square_mm
    board_h_mm = args.squares_y * args.square_mm

    if board_w_mm > A4_WIDTH_MM or board_h_mm > A4_HEIGHT_MM:
        raise ValueError(
            f"Board is {board_w_mm:.1f} x {board_h_mm:.1f} mm and does not fit A4 "
            f"({A4_WIDTH_MM:.1f} x {A4_HEIGHT_MM:.1f} mm)."
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    dictionary = get_aruco_dictionary(args.dictionary)
    board = create_charuco_board(
        args.squares_x, args.squares_y,
        args.square_mm / 1000.0,
        args.marker_mm / 1000.0,
        dictionary,
    )

    board_w_px = mm_to_px(board_w_mm, args.dpi)
    board_h_px = mm_to_px(board_h_mm, args.dpi)

    board_img = (
        board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)
        if hasattr(board, "generateImage")
        else board.draw((board_w_px, board_h_px), marginSize=0, borderBits=1)
    )

    # A4 PNG preview at the same nominal DPI.
    page_w_px = mm_to_px(A4_WIDTH_MM, args.dpi)
    page_h_px = mm_to_px(A4_HEIGHT_MM, args.dpi)
    page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    x0 = (page_w_px - board_w_px) // 2
    y0 = (page_h_px - board_h_px) // 2
    page[y0:y0 + board_h_px, x0:x0 + board_w_px] = board_img

    png_path = args.output_dir / "charuco_A4.png"
    Image.fromarray(page).save(png_path, dpi=(args.dpi, args.dpi))

    # Exact-size A4 PDF. The image is placed using physical dimensions in points.
    board_only_path = args.output_dir / "_charuco_board_only.png"
    Image.fromarray(board_img).save(board_only_path, dpi=(args.dpi, args.dpi))

    pdf_path = args.output_dir / "charuco_A4.pdf"
    pdf = canvas.Canvas(str(pdf_path), pagesize=A4)

    page_w_pt, page_h_pt = A4
    board_w_pt = mm_to_pt(board_w_mm)
    board_h_pt = mm_to_pt(board_h_mm)
    x_pt = (page_w_pt - board_w_pt) / 2.0
    y_pt = (page_h_pt - board_h_pt) / 2.0

    pdf.drawImage(
        ImageReader(str(board_only_path)),
        x_pt,
        y_pt,
        width=board_w_pt,
        height=board_h_pt,
        preserveAspectRatio=True,
        mask=None,
    )

    # 100 mm verification line below the board, where space permits.
    ref_len_mm = 100.0
    ref_x_mm = (A4_WIDTH_MM - ref_len_mm) / 2.0
    ref_y_mm = max(7.0, (A4_HEIGHT_MM - board_h_mm) / 4.0)

    ref_x = mm_to_pt(ref_x_mm)
    ref_y = mm_to_pt(ref_y_mm)
    ref_len = mm_to_pt(ref_len_mm)
    tick = mm_to_pt(2.0)

    pdf.setLineWidth(0.5)
    pdf.line(ref_x, ref_y, ref_x + ref_len, ref_y)
    pdf.line(ref_x, ref_y - tick, ref_x, ref_y + tick)
    pdf.line(ref_x + ref_len, ref_y - tick, ref_x + ref_len, ref_y + tick)
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(ref_x + ref_len / 2.0, ref_y + mm_to_pt(3.0), "100 mm print-size check")

    pdf.showPage()
    pdf.save()
    board_only_path.unlink(missing_ok=True)

    config = {
        "format_version": 1,
        "board_type": "charuco",
        "dictionary": args.dictionary,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_length_m": args.square_mm / 1000.0,
        "marker_length_m": args.marker_mm / 1000.0,
        "board_width_mm": board_w_mm,
        "board_height_mm": board_h_mm,
        "page": "DIN_A4",
        "page_width_mm": A4_WIDTH_MM,
        "page_height_mm": A4_HEIGHT_MM,
    }

    config_path = args.output_dir / "board_config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    print(f"Created: {pdf_path}")
    print(f"Created: {png_path}")
    print(f"Created: {config_path}")
    print()
    print(f"Board: {args.squares_x} x {args.squares_y} squares")
    print(f"Square length: {args.square_mm:.3f} mm")
    print(f"Marker length: {args.marker_mm:.3f} mm")
    print(f"Dictionary: {args.dictionary}")
    print(f"Physical board size: {board_w_mm:.1f} x {board_h_mm:.1f} mm")
    print()
    print('PRINT THE PDF AT 100% / "ACTUAL SIZE". DO NOT USE "FIT TO PAGE".')
    print(f"Verify several squares measure {args.square_mm:.3f} mm and the line measures 100 mm.")


if __name__ == "__main__":
    main()
