"""One docking episode on the real robot (requirements §8): observation -> policy -> command, with the success /
failure rules of the simulator (env.step) applied to the robot's estimates — pure Python, no ROS.

Every command produced here goes to `cmd_vel_dock` / `lift_cmd_dock` and through forklift_safety (AI-06: 0.1 m/s,
ToF, SF-03 front-sensor rule) before the ESP32.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .observation import (ACT_DIM, DockingConfig, action_to_command, build_observation, docking_errors,
                          support_elevation)

SAFETY_STOP = 2  # forklift_interfaces/SafetyState.STOP


@dataclass
class PalletEstimate:
    x: float
    y: float
    yaw: float      # insertion direction (x axis of PalletPose), map frame
    z: float        # marker centre height
    t: float        # time of the measurement


@dataclass
class DockInputs:
    t: float
    carriage_xy: tuple[float, float]   # TF map -> fork (the carriage), planar
    carriage_yaw: float
    pallet: PalletEstimate | None      # latest pallet_pose (5-sample average from forklift_perception)
    lift_m: float
    speed: float
    steer_rad: float
    tof_lcr: tuple[float, float, float]
    tcrt_lr: tuple[bool, bool]
    safety_state: int = 0


@dataclass
class StepResult:
    v: float
    omega: float
    lift_mm: float
    obs: np.ndarray
    action: np.ndarray
    done: bool = False
    success: bool = False
    message: str = ""
    errors: dict = field(default_factory=dict)


@dataclass
class EpisodeParams:
    marker_centre_above_support_m: float = 0.0175  # 30 mm marker centred on a 35 mm pallet face
    visible_timeout_s: float = 0.15                 # pallet_pose older than this = marker not visible this step
    marker_wait_s: float = 2.0                      # fail if the marker is never seen
    safety_stop_timeout_s: float = 3.0              # fail if the supervisor keeps us stopped this long
    success_lateral_m: float = 0.02                 # real-world check on the estimate (sim: 1 cm on ground truth)
    require_tcrt: bool = True                       # both fork-tip sensors must see the pallet


class DockingEpisode:
    def __init__(self, cfg: DockingConfig, policy, support_height_m: float, t0: float,
                 params: EpisodeParams | None = None) -> None:
        self.cfg = cfg
        self.policy = policy
        self.p = params or EpisodeParams()
        self.support_fallback = support_height_m
        self.t0 = t0
        self.steps = 0
        self.prev_action = np.zeros(ACT_DIM, dtype=np.float32)
        self.lift_target: float | None = None
        self.est: PalletEstimate | None = None
        self.elevation = support_height_m
        self.stopped_since: float | None = None
        if hasattr(policy, "reset"):
            policy.reset()

    def step(self, inp: DockInputs) -> StepResult:
        cfg, p = self.cfg, self.p
        if self.lift_target is None:
            self.lift_target = float(inp.lift_m)
        visible = inp.pallet is not None and inp.t - inp.pallet.t <= p.visible_timeout_s
        if visible:
            self.est = inp.pallet
            self.elevation = support_elevation(inp.pallet.z, p.marker_centre_above_support_m, self.support_fallback)
        est = None if self.est is None else (self.est.x, self.est.y, self.est.yaw)
        obs = build_observation(cfg, est, inp.carriage_xy, inp.carriage_yaw, visible, inp.lift_m, self.elevation,
                                inp.speed, inp.steer_rad, inp.tof_lcr,
                                (float(inp.tcrt_lr[0]), float(inp.tcrt_lr[1])), self.prev_action, self.steps)

        def finish(success: bool, message: str, errors: dict | None = None) -> StepResult:
            return StepResult(0.0, 0.0, self.lift_target * 1000.0, obs, np.zeros(ACT_DIM, np.float32), True, success,
                              message, errors or {})

        if est is None:
            if inp.t - self.t0 > p.marker_wait_s:
                return finish(False, "marker_lost: pallet marker never seen")
        errs = docking_errors(cfg, est, inp.carriage_xy, inp.carriage_yaw) if est else {}
        if errs:
            inserted = errs["depth"] >= cfg.success_insertion * cfg.fork_length
            aligned = abs(errs["lat_tip"]) <= p.success_lateral_m and abs(errs["lat_carriage"]) <= p.success_lateral_m
            tips = bool(inp.tcrt_lr[0] and inp.tcrt_lr[1]) or not p.require_tcrt
            if inserted and aligned and tips:
                return finish(True, "inserted", errs)
            if errs["dist_face"] > cfg.out_of_range or errs["depth"] < -0.9:
                return finish(False, "out_of_range", errs)
        if self.steps >= cfg.max_steps:
            return finish(False, f"timeout after {self.steps} steps", errs)
        if inp.safety_state >= SAFETY_STOP:
            self.stopped_since = inp.t if self.stopped_since is None else self.stopped_since
            if inp.t - self.stopped_since > p.safety_stop_timeout_s:
                return finish(False, "safety_stop", errs)
        else:
            self.stopped_since = None

        action = np.asarray(self.policy.act(obs), dtype=np.float32).reshape(ACT_DIM)
        v, omega, _, self.lift_target = action_to_command(cfg, action, self.lift_target)
        self.prev_action = np.clip(action, -1, 1)
        self.steps += 1
        return StepResult(v, omega, self.lift_target * 1000.0, obs, self.prev_action, errors=errs)
