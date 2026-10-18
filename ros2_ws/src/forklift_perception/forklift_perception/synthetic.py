"""Render ArUco markers seen by a pinhole camera at a known pose — ground truth for the host tests and a
quick way to check the detection pipeline without the real camera."""

from __future__ import annotations

import cv2
import numpy as np

from .aruco_core import marker_image


def camera_matrix(fx: float = 600.0, fy: float = 600.0, width: int = 640, height: int = 480) -> np.ndarray:
    return np.array([[fx, 0, width / 2.0], [0, fy, height / 2.0], [0, 0, 1.0]])


def look_at_rvec(yaw_deg: float = 0.0, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Rotation camera <- marker for a marker facing the camera (marker z towards the camera), then turned about
    its own vertical axis (yaw), horizontal axis (pitch) and normal (roll)."""
    face = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)  # marker y up = camera -y, z towards cam
    ry = cv2.Rodrigues(np.array([0.0, np.radians(yaw_deg), 0.0]))[0]
    rx = cv2.Rodrigues(np.array([np.radians(pitch_deg), 0.0, 0.0]))[0]
    rz = cv2.Rodrigues(np.array([0.0, 0.0, np.radians(roll_deg)]))[0]
    return cv2.Rodrigues(face @ ry @ rx @ rz)[0].reshape(3)


def render(marker_id: int, size_m: float, rvec: np.ndarray, tvec: np.ndarray, K: np.ndarray,
           width: int = 640, height: int = 480, supersample: int = 4, noise_std: float = 1.0,
           seed: int = 0) -> np.ndarray:
    """Gray image of one marker (with a white quiet zone of one module) on a gray background."""
    s = supersample
    Ks = K.copy()
    Ks[:2] *= s
    Ks[0, 2] = K[0, 2] * s + (s - 1) / 2.0  # pixel-centre convention survives the downscale
    Ks[1, 2] = K[1, 2] * s + (s - 1) / 2.0
    modules = 6 + 2  # 4x4 data + black border + 1 white quiet-zone module each side
    px = 800
    tex = np.full((px, px), 255, np.uint8)
    inner = px * 6 // modules
    off = (px - inner) // 2
    tex[off:off + inner, off:off + inner] = marker_image(marker_id, inner)
    h = size_m * modules / 6 / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj, np.asarray(rvec, float), np.asarray(tvec, float), Ks, np.zeros(5))
    src = np.array([[0, 0], [px, 0], [px, px], [0, px]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    big = np.full((height * s, width * s), 120, np.uint8)
    warped = cv2.warpPerspective(tex, H, (width * s, height * s), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full((px, px), 255, np.uint8), H, (width * s, height * s),
                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    alpha = mask.astype(np.float32) / 255.0
    big = (alpha * warped + (1 - alpha) * big).astype(np.float32)
    small = cv2.resize(big, (width, height), interpolation=cv2.INTER_AREA)
    if noise_std > 0:
        small += np.random.default_rng(abs(int(seed))).normal(0, noise_std, small.shape)
    return np.clip(small, 0, 255).astype(np.uint8)
