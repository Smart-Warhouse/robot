"""Initial pose from the charger ArUco marker (NV-04) — replaces the manual "2D Pose Estimate".

    T_map_base = T_map_marker · inv(T_cam_marker) · inv(T_base_cam)

`T_map_marker` comes from the marker map: every location of the server's warehouse_map.yaml carries an ArUco id
(21-40); its marker hangs `offset_m` ahead of the location node along the location yaw (the direction the robot
faces when it stands there), at `height_m`, facing the robot. Measured marker poses (EN-08: ± 2 mm) override the
derived ones via `overrides`. Several consistent samples are averaged before the pose is published once.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MarkerPose:
    marker_id: int
    x: float
    y: float
    z: float
    normal_yaw: float  # direction the marker face points to (towards the robot standing in front of it)

    def T_map_marker(self) -> np.ndarray:
        """Marker frame: z = face normal, y = up, x = y × z (OpenCV ArUco convention)."""
        n = self.normal_yaw
        z = np.array([math.cos(n), math.sin(n), 0.0])
        y = np.array([0.0, 0.0, 1.0])
        x = np.cross(y, z)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
        T[:3, 3] = [self.x, self.y, self.z]
        return T


def marker_map(warehouse: dict, offsets_m: dict[str, float] | None = None,
               heights_m: dict[str, float] | None = None, overrides: list[dict] | None = None) -> dict[int, MarkerPose]:
    """Marker poses from warehouse_map.yaml locations (one marker per aruco id; both rack levels share it).

    Default heights: the camera rides on the fork carriage pitched 20 deg down, so at a charger (robot 0.2 m from
    the wall) only a marker mounted low is fully in view - its centre 2 cm above the floor."""
    offsets = {"rack": 0.15, "charge": 0.20, "dock_in": 0.15, "dock_out": 0.15, "staging": 0.15}
    offsets.update(offsets_m or {})
    heights = {"rack": 0.05, "charge": 0.02, "dock_in": 0.05, "dock_out": 0.05, "staging": 0.05}
    heights.update(heights_m or {})
    nodes = warehouse["nodes"]
    out: dict[int, MarkerPose] = {}
    for loc in warehouse.get("locations", []):
        mid = loc.get("aruco")
        if mid is None or mid in out:
            continue
        x, y = (float(v) for v in nodes[loc["node"]])
        yaw = float(loc.get("yaw", 0.0))
        kind = loc.get("type", "rack")
        d = offsets.get(kind, 0.15)
        out[int(mid)] = MarkerPose(int(mid), x + d * math.cos(yaw), y + d * math.sin(yaw), heights.get(kind, 0.05),
                                   _wrap(yaw + math.pi))
    for o in overrides or []:
        out[int(o["id"])] = MarkerPose(int(o["id"]), float(o["x"]), float(o["y"]), float(o.get("z", 0.05)),
                                       float(o["normal_yaw"]))
    return out


def robot_pose_from_marker(T_map_marker: np.ndarray, T_cam_marker: np.ndarray,
                           T_base_cam: np.ndarray) -> tuple[float, float, float]:
    """Planar (x, y, yaw) of base_link in map."""
    T = T_map_marker @ np.linalg.inv(T_cam_marker) @ np.linalg.inv(T_base_cam)
    return float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])


class InitialPoseEstimator:
    """Collects `samples` robot-pose estimates from allowed markers; `ready()` once they agree."""

    def __init__(self, markers: dict[int, MarkerPose], allowed_ids: set[int] | None = None, samples: int = 10,
                 max_spread_m: float = 0.02, max_spread_rad: float = math.radians(2.0),
                 max_distance_m: float = 1.0) -> None:
        self.markers = markers
        self.allowed = allowed_ids
        self.n = samples
        self.max_spread_m = max_spread_m
        self.max_spread_rad = max_spread_rad
        self.max_distance = max_distance_m
        self.poses: list[tuple[float, float, float]] = []
        self.marker_id: int | None = None

    def add(self, marker_id: int, T_cam_marker: np.ndarray, T_base_cam: np.ndarray) -> bool:
        if marker_id not in self.markers or (self.allowed is not None and marker_id not in self.allowed):
            return False
        if np.linalg.norm(T_cam_marker[:3, 3]) > self.max_distance:
            return False
        if self.marker_id not in (None, marker_id):
            self.poses.clear()
        self.marker_id = marker_id
        self.poses.append(robot_pose_from_marker(self.markers[marker_id].T_map_marker(), T_cam_marker, T_base_cam))
        self.poses = self.poses[-self.n:]
        return True

    def estimate(self) -> tuple[float, float, float] | None:
        if len(self.poses) < self.n:
            return None
        p = np.array(self.poses)
        yaw = math.atan2(np.sin(p[:, 2]).sum(), np.cos(p[:, 2]).sum())
        xy = p[:, :2].mean(axis=0)
        spread = float(np.sqrt(np.mean(np.sum((p[:, :2] - xy) ** 2, axis=1))))
        yaw_spread = float(np.sqrt(np.mean([_wrap(a - yaw) ** 2 for a in p[:, 2]])))
        if spread > self.max_spread_m or yaw_spread > self.max_spread_rad:
            return None
        return float(xy[0]), float(xy[1]), yaw


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
