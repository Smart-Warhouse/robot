"""The 18-dim docking observation, built from robot measurements exactly like the training simulator
(`code/training/forklift_rl/env.py`, `ForkliftDockingEnv._obs`) — pure NumPy, no ROS.

  0-1  pallet face centre in the fork-carriage frame (dx / 0.6, dy / 0.15), from the ArUco estimate (PC-04/05)
  2-3  sin / cos of the pallet yaw relative to the forklift
  4    fork height error to the pallet opening / 0.05 m
  5    marker visible this step (else the estimate is carried by odometry: the map-frame estimate is re-projected
       with the current carriage pose)
  6-8  measured speed / v_max, steering angle / steer_max, fork lift / lift_max
  9-11 front ToF ring: left corner, centre, right corner / 1.2 m
  12-13 TCRT5000 left / right fork tip
  14-16 previous action
  17   time left in the episode: 1 - steps / max_steps
Action (3, [-1, 1]): linear speed x v_max, rear steering angle x steer_max, lift rate x lift_rate.
Constants come from forklift_bringup/config/docking.yaml, which must equal code/training/forklift_rl/config.yaml.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

OBS_DIM, ACT_DIM = 18, 3
TOF_MAX_M = 1.2           # env: rays beyond 1.2 m read 1.2
NO_ESTIMATE_REL = (0.6, 0.0)  # env: before the first sighting the pallet is assumed 0.6 m straight ahead


@dataclass(frozen=True)
class DockingConfig:
    wheelbase: float = 0.20
    fork_length: float = 0.18
    fork_thickness: float = 0.012
    fork_clearance: float = 0.004
    lift_max: float = 0.12
    pallet_height: float = 0.035
    pallet_deck_thickness: float = 0.008
    rate_hz: float = 20.0
    v_max: float = 0.10
    steer_max_deg: float = 30.0
    lift_rate: float = 0.05
    max_steps: int = 300
    success_insertion: float = 0.80
    success_lateral: float = 0.01
    out_of_range: float = 1.0

    @property
    def steer_max(self) -> float:
        return math.radians(self.steer_max_deg)

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz

    @classmethod
    def from_training(cls, cfg: dict) -> "DockingConfig":
        """From the training config.yaml layout ({forklift, pallet, control, task})."""
        f, p, c, t = cfg["forklift"], cfg["pallet"], cfg["control"], cfg.get("task", {})
        return cls(wheelbase=f["wheelbase"], fork_length=f["fork_length"], fork_thickness=f["fork_thickness"],
                   fork_clearance=f["fork_clearance"], lift_max=f["lift_max"], pallet_height=p["height"],
                   pallet_deck_thickness=p["deck_thickness"], rate_hz=c["rate_hz"], v_max=c["v_max"],
                   steer_max_deg=c["steer_max_deg"], lift_rate=c["lift_rate"], max_steps=int(c["max_steps"]),
                   success_insertion=t.get("success_insertion", 0.8), success_lateral=t.get("success_lateral", 0.01),
                   out_of_range=t.get("out_of_range", 1.0))

    @classmethod
    def from_flat(cls, d: dict) -> "DockingConfig":
        base = cls()
        kw = {k: type(getattr(base, k))(v) for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**kw)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def relative_pallet(pallet_xy_yaw: tuple[float, float, float] | None, carriage_xy: tuple[float, float],
                    carriage_yaw: float) -> tuple[np.ndarray, float]:
    """Pallet face centre and yaw in the carriage frame (x forward)."""
    if pallet_xy_yaw is None:
        return np.array(NO_ESTIMATE_REL), 0.0
    px, py, pyaw = pallet_xy_yaw
    rel = rot(-carriage_yaw) @ (np.array([px, py]) - np.asarray(carriage_xy, dtype=np.float64))
    return rel, wrap(pyaw - carriage_yaw)


def build_observation(cfg: DockingConfig, pallet_xy_yaw: tuple[float, float, float] | None,
                      carriage_xy: tuple[float, float], carriage_yaw: float, visible: bool, lift_m: float,
                      support_elevation_m: float, speed: float, steer_rad: float,
                      tof_lcr: tuple[float, float, float], tcrt_lr: tuple[float, float],
                      prev_action: np.ndarray, steps: int) -> np.ndarray:
    rel, rel_yaw = relative_pallet(pallet_xy_yaw, carriage_xy, carriage_yaw)
    fork_z = cfg.fork_clearance + lift_m
    target_z = support_elevation_m + (cfg.pallet_height - cfg.pallet_deck_thickness - cfg.fork_thickness) / 2
    tof = [min(max(float(d), 0.0), TOF_MAX_M) if math.isfinite(d) else TOF_MAX_M for d in tof_lcr]
    obs = np.array([
        rel[0] / 0.6, rel[1] / 0.15, math.sin(rel_yaw), math.cos(rel_yaw), (target_z - fork_z) / 0.05,
        float(visible), speed / cfg.v_max, steer_rad / cfg.steer_max, lift_m / cfg.lift_max,
        tof[0] / TOF_MAX_M, tof[1] / TOF_MAX_M, tof[2] / TOF_MAX_M, float(tcrt_lr[0]), float(tcrt_lr[1]),
        *np.asarray(prev_action, dtype=np.float64).reshape(3), 1.0 - steps / cfg.max_steps,
    ], dtype=np.float32)
    return np.clip(obs, -5, 5)


def action_to_command(cfg: DockingConfig, action: np.ndarray, lift_target_m: float) -> tuple[float, float, float,
                                                                                               float]:
    """Policy action -> (v m/s, omega rad/s for cmd_vel, steering angle rad, new lift target m).

    The ESP32 takes a Twist (FW-01) and converts it back to a rear steering angle with the same Ackermann model the
    simulator uses: omega = -v * tan(steer) / wheelbase (rear-wheel steering: steer left -> yaw right).
    """
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(3), -1.0, 1.0)
    v = float(a[0]) * cfg.v_max
    steer = float(a[1]) * cfg.steer_max
    omega = -v * math.tan(steer) / cfg.wheelbase
    lift = float(np.clip(lift_target_m + a[2] * cfg.lift_rate * cfg.dt, 0.0, cfg.lift_max))
    return v, omega, steer, lift


def steering_from_twist(cfg: DockingConfig, v: float, omega: float, fallback: float) -> float:
    """Recover the steering angle from measured odometry (inverse of action_to_command); `fallback` when standing."""
    if abs(v) < 1e-3:
        return fallback
    return math.atan(-omega * cfg.wheelbase / v)


def docking_errors(cfg: DockingConfig, pallet_xy_yaw: tuple[float, float, float], carriage_xy: tuple[float, float],
                   carriage_yaw: float) -> dict:
    """Insertion depth and lateral errors in the pallet frame (env.errors(), from the estimate)."""
    px, py, psi = pallet_xy_yaw
    u = np.array([math.cos(psi), math.sin(psi)])
    n = np.array([-math.sin(psi), math.cos(psi)])
    c = np.asarray(carriage_xy, dtype=np.float64)
    tip = c + cfg.fork_length * np.array([math.cos(carriage_yaw), math.sin(carriage_yaw)])
    P = np.array([px, py])
    return {"depth": float(u @ (tip - P)), "lat_tip": float(n @ (tip - P)), "lat_carriage": float(n @ (c - P)),
            "heading": wrap(carriage_yaw - psi), "dist_face": float(np.linalg.norm(tip - P))}


def support_elevation(marker_z_m: float | None, marker_centre_above_support_m: float, fallback_m: float) -> float:
    """Height of the surface the pallet stands on: from the marker's 3-D pose when available (env docstring), else
    the task's height_mm."""
    if marker_z_m is None or not math.isfinite(marker_z_m):
        return fallback_m
    return max(0.0, marker_z_m - marker_centre_above_support_m)
