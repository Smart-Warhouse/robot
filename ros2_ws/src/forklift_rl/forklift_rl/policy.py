"""Docking policies behind one interface `act(obs[18]) -> action[3]` (AI-05).

  * `OnnxPolicy`  — the exported actor (ai/training/scripts/export_onnx.py: input `obs` float32 [N, 18], output
    `action` float32 [N, 3] in [-1, 1]) on onnxruntime; TensorRT / CUDA execution providers are used when the
    Jetson build of onnxruntime-gpu offers them (TensorRT FP16 target <= 5 ms), else CPU.
  * `ClassicalPolicy` — the visual-servoing baseline (AI-07 reference), port of ai/training/forklift_rl/baseline.py.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from .observation import ACT_DIM, OBS_DIM, DockingConfig

DEFAULT_PROVIDERS = ("TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider")


class OnnxPolicy:
    def __init__(self, path: str | Path, providers: tuple[str, ...] = DEFAULT_PROVIDERS, fp16: bool = True,
                 trt_cache_dir: str | None = None) -> None:
        import onnxruntime as ort

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"policy model not found: {path}")
        available = set(ort.get_available_providers())
        chosen = []
        for p in providers:
            if p not in available:
                continue
            if p == "TensorrtExecutionProvider":
                opts = {"trt_fp16_enable": bool(fp16)}
                if trt_cache_dir:
                    opts.update({"trt_engine_cache_enable": True, "trt_engine_cache_path": trt_cache_dir})
                chosen.append((p, opts))
            else:
                chosen.append(p)
        if not chosen:
            chosen = ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(path), providers=chosen)
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError(f"expected one input, model has {[i.name for i in inputs]}")
        self.input_name = inputs[0].name
        shape = inputs[0].shape
        if shape and isinstance(shape[-1], int) and shape[-1] != OBS_DIM:
            raise ValueError(f"model input has {shape[-1]} features, the observation has {OBS_DIM}")
        outs = [o.name for o in self.session.get_outputs()]
        self.output_name = "action" if "action" in outs else outs[0]
        self.providers = self.session.get_providers()
        self.last_latency_ms = 0.0

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(1, OBS_DIM)
        t0 = time.perf_counter()
        y = self.session.run([self.output_name], {self.input_name: x})[0]
        self.last_latency_ms = (time.perf_counter() - t0) * 1000.0
        a = np.asarray(y, dtype=np.float32).reshape(-1)[:ACT_DIM]
        if a.shape[0] != ACT_DIM or not np.all(np.isfinite(a)):
            return np.zeros(ACT_DIM, dtype=np.float32)  # never pass garbage on to the motors
        return np.clip(a, -1.0, 1.0)

    def reset(self) -> None:
        pass


class ClassicalPolicy:
    """Visual servoing on the ArUco pallet pose; sees exactly the RL observation (fair comparison, AI-07)."""

    def __init__(self, cfg: DockingConfig, carriage_x: float = 0.03) -> None:
        self.L = cfg.wheelbase
        self.fork = cfg.fork_length
        self.steer_max = cfg.steer_max
        self.dt = cfg.dt
        self.carriage_x = carriage_x
        self.lam = 11.0
        self.reset()

    def reset(self) -> None:
        self.backoff = 0.0
        self.tries = 0

    def decode(self, obs: np.ndarray) -> dict:
        dx, dy = obs[0] * 0.6, obs[1] * 0.15
        phi = math.atan2(obs[2], obs[3])
        c, s = math.cos(-phi), math.sin(-phi)
        px, py = c * -dx - s * -dy, s * -dx + c * -dy
        heading = -phi
        return {"s": px, "e": py, "heading": heading, "tip_s": px + math.cos(heading) * self.fork,
                "tip_e": py + math.sin(heading) * self.fork, "z_err": obs[4] * 0.05}

    def act(self, obs: np.ndarray) -> np.ndarray:
        st = self.decode(np.asarray(obs, dtype=np.float64))
        lift = float(np.clip(st["z_err"] / 0.015, -1, 1))
        if self.backoff > 0:
            self.backoff -= self.dt
            return np.array([-1.0, 0.0, lift], dtype=np.float32)
        misaligned = (abs(st["tip_e"]) > 0.008 or abs(st["e"]) > 0.01 or abs(st["heading"]) > math.radians(3)
                      or abs(st["z_err"]) > 0.004)
        if -0.03 < st["tip_s"] < 0.005 and misaligned and self.tries < 4:
            self.tries += 1
            self.backoff = 2.0
            return np.array([0.0, 0.0, lift], dtype=np.float32)
        e_axle = st["e"] - self.carriage_x * math.sin(st["heading"])
        lam, zeta = self.lam, 0.9
        curvature = -(2 * zeta * lam * math.sin(st["heading"]) + lam * lam * e_axle)
        steer = math.atan(-curvature * self.L)
        if st["tip_s"] < -0.15:
            v = 1.0
        elif st["tip_s"] < -0.01:
            v = 0.55
        else:
            v = 0.4
        if abs(st["z_err"]) > 0.004 and st["tip_s"] > -0.06:
            v = 0.0
        return np.array([v, float(np.clip(steer / self.steer_max, -1, 1)), lift], dtype=np.float32)
