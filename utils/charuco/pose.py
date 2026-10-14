"""ChArUco target pose estimation using Leroy's existing calibration detector.

The functions load_board/detect_charuco are deliberately reused from
calibrate.py so the printed board and calibration geometry cannot diverge.
This module does not import ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from calibrate import load_board, detect_charuco


@dataclass(frozen=True)
class TargetPose:
    stamp_ns: int
    rotation_cam_board: np.ndarray
    camera_location_board: np.ndarray
    center_camera: np.ndarray
    center_uv: tuple[float, float]
    range_m: float
    corners: int
    rmse_px: float


class CharucoPoseEstimator:
    def __init__(self, config: Path, min_corners: int = 8, max_rmse_px: float = 2.0):
        self.config, self.dictionary, self.board = load_board(Path(config))
        self.object_points = np.asarray(self.board.getChessboardCorners(), dtype=np.float64)
        self.center_board = self.object_points.mean(axis=0)
        self.min_corners = min_corners
        self.max_rmse_px = max_rmse_px

    def estimate(self, gray, camera_info, stamp_ns: int) -> TargetPose | None:
        """Solve board-to-camera pose; reject weak or inconsistent detections."""
        if camera_info is None or gray is None or gray.ndim != 2:
            return None
        if camera_info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            return None
        if camera_info.width != gray.shape[1] or camera_info.height != gray.shape[0]:
            return None
        k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(camera_info.d, dtype=np.float64).reshape(-1)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            return None
        corners, charuco_ids, _, _ = detect_charuco(gray, self.board, self.dictionary)
        if charuco_ids is None or len(charuco_ids) < self.min_corners:
            return None
        ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
        if np.unique(ids).size != len(ids) or min(ids) < 0 or max(ids) >= len(self.object_points):
            return None
        pixel_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        object_points = self.object_points[ids]
        if np.ptp(pixel_points[:, 0]) < 40 or np.ptp(pixel_points[:, 1]) < 40:
            return None
        try:
            ok, rvec, tvec = cv2.solvePnP(
                object_points, pixel_points, k, d, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                return None
            r, _ = cv2.Rodrigues(rvec)
            center_camera = r @ self.center_board + tvec.reshape(3)
            if center_camera[2] <= 0:
                return None
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
            rmse = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(-1, 2) - pixel_points) ** 2, axis=1
            ))))
            if not np.isfinite(rmse) or rmse > self.max_rmse_px:
                return None
            center_px, _ = cv2.projectPoints(self.center_board[None, :], rvec, tvec, k, d)
            u, v = center_px.reshape(2)
            return TargetPose(
                stamp_ns=stamp_ns,
                rotation_cam_board=r,
                camera_location_board=-r.T @ tvec.reshape(3),
                center_camera=center_camera,
                center_uv=(float(u / camera_info.width), float(v / camera_info.height)),
                range_m=float(np.linalg.norm(center_camera)),
                corners=len(ids),
                rmse_px=rmse,
            )
        except cv2.error:
            return None
