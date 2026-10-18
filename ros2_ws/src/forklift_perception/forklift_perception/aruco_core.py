"""ArUco detection and pose estimation (PC-03, PC-04, PC-05) — OpenCV + NumPy only, no ROS.

  * dictionary DICT_4X4_50; ids 1-20 = pallets (30 mm), 21-40 = locations: racks, docks, chargers (60 mm)
  * corners refined to sub-pixel, pose from solvePnP (IPPE_SQUARE, the better of its two solutions)
  * `PoseFilter`: moving average of the last 5 samples per marker, reset on a gap or a jump (PC-05)
  * `pallet_pose_in_frame`: marker pose -> pallet pose (x into the pallet, z up, planar yaw) in any frame

Works with the OpenCV of Ubuntu 22.04 (4.5.4, old `cv2.aruco.detectMarkers` API) and with OpenCV >= 4.7
(`cv2.aruco.ArucoDetector`).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

PALLET, LOCATION = "pallet", "location"


@dataclass(frozen=True)
class MarkerSpec:
    pallet_ids: tuple[int, int] = (1, 20)
    location_ids: tuple[int, int] = (21, 40)
    pallet_size_m: float = 0.030
    location_size_m: float = 0.060

    def classify(self, marker_id: int) -> tuple[str, float] | None:
        if self.pallet_ids[0] <= marker_id <= self.pallet_ids[1]:
            return PALLET, self.pallet_size_m
        if self.location_ids[0] <= marker_id <= self.location_ids[1]:
            return LOCATION, self.location_size_m
        return None


@dataclass
class MarkerObservation:
    marker_id: int
    kind: str
    size_m: float
    rvec: np.ndarray            # camera <- marker rotation (Rodrigues)
    tvec: np.ndarray            # marker centre in the camera optical frame, m
    corners: np.ndarray         # 4x2 px
    reproj_error_px: float
    confidence: float

    @property
    def R(self) -> np.ndarray:
        return cv2.Rodrigues(self.rvec)[0]

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.tvec))

    def T_cam_marker(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.tvec.reshape(3)
        return T


def get_dictionary(name: str = "DICT_4X4_50"):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(marker_id: int, pixels: int, name: str = "DICT_4X4_50") -> np.ndarray:
    d = get_dictionary(name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, marker_id, pixels)
    return cv2.aruco.drawMarker(d, marker_id, pixels)  # OpenCV < 4.7


def object_points(size_m: float) -> np.ndarray:
    """Marker corners in the marker frame (x right, y up, z out of the marker), OpenCV corner order."""
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


class ArucoEstimator:
    def __init__(self, spec: MarkerSpec | None = None, dictionary: str = "DICT_4X4_50", subpix: bool = True,
                 min_side_px: float = 12.0) -> None:
        self.spec = spec or MarkerSpec()
        self.dict = get_dictionary(dictionary)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
        else:
            params = cv2.aruco.DetectorParameters()
        # ArUco's own refinement is left OFF: in OpenCV 4.5 (Ubuntu 22.04) CORNER_REFINE_SUBPIX is ~5x less accurate
        # than in >= 4.7 (1.4 px vs 0.3 px on a 36 px marker → 10-24° orientation error). We refine with
        # cv2.cornerSubPix ourselves, which behaves the same on every version (see _refine).
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.subpix = subpix
        self.params = params
        self.detector = cv2.aruco.ArucoDetector(self.dict, params) if hasattr(cv2.aruco, "ArucoDetector") else None
        self.min_side_px = min_side_px

    def detect(self, image: np.ndarray) -> list[tuple[int, np.ndarray]]:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        if ids is None:
            return []
        out = [(int(i), c.reshape(4, 2).astype(np.float64)) for i, c in zip(ids.flatten(), corners)]
        return [(i, self._refine(gray, c)) for i, c in out] if self.subpix else out

    @staticmethod
    def _refine(gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
        """Sub-pixel corners. Window = 8 % of the marker side: big enough to average noise, small enough not to
        reach the marker's inner cells (≥ 12 % does and degrades the pose — measured on 24 synthetic cases)."""
        side = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
        w = max(2, int(round(side * 0.08)))
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
        pts = cv2.cornerSubPix(gray, corners.astype(np.float32).reshape(-1, 1, 2), (w, w), (-1, -1), crit)
        return pts.reshape(4, 2).astype(np.float64)

    def estimate(self, image: np.ndarray, K: np.ndarray, dist: np.ndarray | None = None) -> list[MarkerObservation]:
        dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
        out = []
        for mid, c in self.detect(image):
            cls = self.spec.classify(mid)
            if cls is None:
                continue
            kind, size = cls
            side = float(np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)))
            if side < self.min_side_px:
                continue
            obs = solve_marker_pose(c, size, K, dist)
            if obs is None:
                continue
            rvec, tvec, err = obs
            conf = confidence(err, side, rvec, tvec)
            out.append(MarkerObservation(mid, kind, size, rvec, tvec, c, err, conf))
        return out


def solve_marker_pose(corners: np.ndarray, size_m: float, K: np.ndarray, dist: np.ndarray):
    """(rvec, tvec, reprojection error px) — the better of the two IPPE_SQUARE solutions, LM-refined."""
    obj = object_points(size_m)
    img = corners.reshape(4, 1, 2)
    try:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if not n:
        return None
    best = int(np.argmin(np.asarray(errs).reshape(-1)))
    rvec, tvec = rvecs[best].reshape(3, 1), tvecs[best].reshape(3, 1)
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - corners) ** 2, axis=1))))
    if tvec[2, 0] <= 0:
        return None
    return rvec.reshape(3), tvec.reshape(3), err


def confidence(reproj_px: float, side_px: float, rvec: np.ndarray, tvec: np.ndarray) -> float:
    """0..1 heuristic: low reprojection error, a marker big enough in the image, not seen too obliquely."""
    c_err = math.exp(-((reproj_px / 1.0) ** 2))
    c_size = min(1.0, side_px / 30.0)
    R = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
    view = -np.asarray(tvec, dtype=np.float64).reshape(3)
    view /= np.linalg.norm(view)
    cos_angle = float(R[:, 2] @ view)   # marker normal vs direction to the camera
    c_angle = min(1.0, max(0.0, (cos_angle - 0.34) / (0.87 - 0.34)))  # 0 beyond 70 deg, 1 inside 30 deg
    return float(c_err * c_size * c_angle)


# ---------------------------------------------------------------------- frames
def pallet_pose_in_frame(T_frame_cam: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, float]:
    """Marker on a pallet / rack face -> (position xyz, yaw) in `frame`. The yaw is the insertion direction
    (into the pallet): the marker normal points out of the face, towards the forklift."""
    T_cm = np.eye(4)
    T_cm[:3, :3] = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
    T_cm[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    T = T_frame_cam @ T_cm
    into = -T[:3, 2]
    return T[:3, 3].copy(), math.atan2(into[1], into[0])


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def rotation_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return float(x), float(y), float(z), float(w)


def transform_from(translation, quaternion_xyzw) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(translation, dtype=np.float64)
    return T


# ---------------------------------------------------------------------- PC-05 filter
class PoseFilter:
    """Moving average of the last `window` (5) poses of one marker. Restarts after `max_gap_s` without a sample
    or a jump larger than `max_jump_m` (a different pallet / a bad detection)."""

    def __init__(self, window: int = 5, max_gap_s: float = 0.5, max_jump_m: float = 0.05) -> None:
        self.window = window
        self.max_gap = max_gap_s
        self.max_jump = max_jump_m
        self.samples: deque = deque(maxlen=window)
        self.t_last = -math.inf

    def add(self, t: float, position: np.ndarray, yaw: float, confidence: float = 1.0) -> None:
        position = np.asarray(position, dtype=np.float64).reshape(3)
        if t - self.t_last > self.max_gap:
            self.samples.clear()
        elif self.samples and np.linalg.norm(position - self.mean()[0]) > self.max_jump:
            self.samples.clear()
        self.samples.append((position, float(yaw), float(confidence)))
        self.t_last = t

    def mean(self) -> tuple[np.ndarray, float]:
        pos = np.mean([s[0] for s in self.samples], axis=0)
        yaw = math.atan2(sum(math.sin(s[1]) for s in self.samples), sum(math.cos(s[1]) for s in self.samples))
        return pos, yaw

    def confidence(self) -> float:
        """Mean sample confidence x agreement of the samples (spread of 1 cm -> 0.5)."""
        if not self.samples:
            return 0.0
        pos = np.array([s[0] for s in self.samples])
        spread = float(np.sqrt(np.mean(np.sum((pos - pos.mean(axis=0)) ** 2, axis=1))))
        agreement = 1.0 / (1.0 + (spread / 0.01) ** 2)
        fill = len(self.samples) / self.window
        return float(np.mean([s[2] for s in self.samples]) * agreement * (0.5 + 0.5 * fill))

    def __len__(self) -> int:
        return len(self.samples)
