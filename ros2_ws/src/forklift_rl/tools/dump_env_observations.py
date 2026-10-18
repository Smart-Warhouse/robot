"""Record (robot state -> observation) pairs from the training simulator so the ROS-side observation builder can be
checked against the exact 18-dim observation the policy was trained on.

Run with the *training* environment (it needs MuJoCo), from anywhere:
    ai/training/.venv/Scripts/python robot/ros2_ws/src/forklift_rl/tools/dump_env_observations.py
Writes robot/ros2_ws/src/forklift_rl/test/data/env_obs_samples.json (committed; regenerate after changing
ai/training/forklift_rl/env.py or config.yaml).
"""

from __future__ import annotations

import json
import math
import sys
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
TRAINING = (Path(os.environ["SW_WORKSPACE"]) if os.environ.get("SW_WORKSPACE") else HERE.parents[5]) / "ai" / "training"
OUT = HERE.parents[1] / "test" / "data" / "env_obs_samples.json"
sys.path.insert(0, str(TRAINING))

from forklift_rl.baseline import BaselineController  # noqa: E402  (the training package, not the ROS one)
from forklift_rl.env import ForkliftDockingEnv  # noqa: E402


def main() -> None:
    env = ForkliftDockingEnv(randomize=False)  # no sensor noise: the observation is a pure function of the state
    rng = np.random.default_rng(0)
    ctrl = BaselineController(env.cfg)
    samples = []
    for ep in range(10):
        obs, _ = env.reset(seed=ep, options={"elevation": 0.06 if ep % 2 else 0.0})
        ctrl.reset()
        for step in range(env.cfg["control"]["max_steps"]):
            pts = env._points()
            tof = env._tof()
            sample = {
                "carriage": [float(pts["carriage"][0]), float(pts["carriage"][1])],
                "yaw": float(pts["yaw"]),
                "pallet_estimate": None if env._est_world is None else [float(v) for v in env._est_world],
                "visible": bool(env._visible),
                "lift_m": float(env.data.qpos[env._qlift]),
                "elevation_m": float(env._elevation),
                "speed": float(env._v),
                "steer_rad": float(env._steer),
                "tof_lcr": [float(v) for v in tof],
                "tcrt_lr": [float(v) for v in env._tcrt()],
                "prev_action": [float(v) for v in env._prev_action],
                "steps": int(env._steps),
            }
            sample["obs"] = [float(v) for v in env._obs()]
            if step % 4 == 0 or (step % 2 == 0 and (sample["tcrt_lr"][0] or not sample["visible"])):
                samples.append(sample)
            # classical controller (reaches insertion: TCRT + marker out of view), with some random actions mixed in
            a = ctrl.act(obs) if ep < 7 else rng.uniform(-1, 1, size=3).astype(np.float32)
            obs, _, term, trunc, _ = env.step(a)
            if term or trunc:
                break
    cfg = env.cfg
    OUT.parent.mkdir(parents=True, exist_ok=True)
    def compact(x):
        if isinstance(x, float):
            return float(f"{x:.9g}")
        if isinstance(x, list):
            return [compact(v) for v in x]
        if isinstance(x, dict):
            return {k: compact(v) for k, v in x.items()}
        return x

    OUT.write_text(json.dumps({"config": {k: cfg[k] for k in ("forklift", "pallet", "control")},
                               "samples": compact(samples)}, separators=(",", ":")))
    print(f"{len(samples)} samples -> {OUT}")
    assert all(math.isfinite(v) for s in samples for v in s["obs"])


if __name__ == "__main__":
    main()
