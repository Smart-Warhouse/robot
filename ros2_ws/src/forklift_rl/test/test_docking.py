"""forklift_rl: the observation equals the simulator's, the ONNX runner works on a generated model, and a docking
episode closes the loop on a kinematic model of the forklift."""

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_rl.docking_core import DockingEpisode, DockInputs, EpisodeParams, PalletEstimate
from forklift_rl.observation import (DockingConfig, action_to_command, build_observation, docking_errors,
                                     steering_from_twist, support_elevation)
from forklift_rl.policy import ClassicalPolicy, OnnxPolicy

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]
REPO = SRC.parents[2]
FIXTURE = json.loads((HERE / "data" / "env_obs_samples.json").read_text())
CFG = DockingConfig.from_training(FIXTURE["config"])


# ---------------------------------------------------------------------- observation == simulator
def test_observation_matches_training_env_exactly():
    """718 states recorded from code/training/forklift_rl/env.py (tools/dump_env_observations.py), including
    marker-not-visible and forks-inserted (TCRT) states."""
    assert len(FIXTURE["samples"]) > 500
    assert any(not s["visible"] for s in FIXTURE["samples"]) and any(s["tcrt_lr"][0] for s in FIXTURE["samples"])
    worst = 0.0
    for s in FIXTURE["samples"]:
        obs = build_observation(CFG, tuple(s["pallet_estimate"]), tuple(s["carriage"]), s["yaw"], s["visible"],
                                s["lift_m"], s["elevation_m"], s["speed"], s["steer_rad"], tuple(s["tof_lcr"]),
                                tuple(s["tcrt_lr"]), np.array(s["prev_action"]), s["steps"])
        assert obs.shape == (18,) and obs.dtype == np.float32
        worst = max(worst, float(np.max(np.abs(obs - np.array(s["obs"], dtype=np.float32)))))
    assert worst < 1e-5


def test_observation_before_first_sighting():
    obs = build_observation(CFG, None, (0.0, 0.0), 0.3, False, 0.0, 0.0, 0.0, 0.0, (1.2, 1.2, 1.2), (0, 0),
                            np.zeros(3), 0)
    assert obs[0] == pytest.approx(1.0) and obs[1] == 0.0 and obs[3] == pytest.approx(1.0) and obs[5] == 0.0
    assert obs[17] == pytest.approx(1.0)
    obs = build_observation(CFG, None, (0, 0), 0, False, 0, 0, 0, 0, (float("inf"), -1.0, 5.0), (1, 0),
                            np.zeros(3), 300)
    assert list(obs[9:12]) == pytest.approx([1.0, 0.0, 1.0]) and obs[17] == pytest.approx(0.0)


def test_bringup_docking_config_equals_training_config():
    """JS-05 keeps the robot's copy in forklift_bringup; it must not drift from what the policy was trained with."""
    train_file = REPO / "code" / "training" / "forklift_rl" / "config.yaml"
    if not train_file.exists():
        pytest.skip("training config not present")
    train = DockingConfig.from_training(yaml.safe_load(train_file.read_text(encoding="utf-8")))
    bringup = yaml.safe_load((SRC / "forklift_bringup" / "config" / "forklift.yaml").read_text())
    dock = bringup["/**/dock_server"]["ros__parameters"]["docking"]
    assert DockingConfig.from_flat(dock) == train


def test_action_mapping_rear_wheel_steering():
    v, omega, steer, lift = action_to_command(CFG, np.array([1.0, 0.5, 1.0]), 0.02)
    assert v == pytest.approx(0.10) and steer == pytest.approx(math.radians(15))
    assert omega == pytest.approx(-0.10 * math.tan(math.radians(15)) / 0.20)  # steer left -> yaw right
    assert lift == pytest.approx(0.02 + 0.05 / 20)
    _, _, _, lift = action_to_command(CFG, np.array([0, 0, 1.0]), 0.1199)
    assert lift == pytest.approx(0.12)
    _, _, _, lift = action_to_command(CFG, np.array([3.0, -7, -1.0]), 0.0)
    assert lift == 0.0
    assert steering_from_twist(CFG, v, omega, 0.0) == pytest.approx(steer)
    assert steering_from_twist(CFG, 0.0, 0.3, 0.1) == 0.1


def test_support_elevation_from_marker():
    assert support_elevation(0.0775, 0.0175, 0.0) == pytest.approx(0.06)
    assert support_elevation(None, 0.0175, 0.06) == 0.06
    assert support_elevation(0.005, 0.0175, 0.06) == 0.0


# ---------------------------------------------------------------------- ONNX
def tiny_policy_onnx(path: Path, W: np.ndarray, b: np.ndarray, in_dim: int = 18) -> Path:
    """Same interface as export_onnx.py: obs float32 [N, in_dim] -> action float32 [N, 3] = clip(obs W + b)."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    graph = helper.make_graph(
        [helper.make_node("MatMul", ["obs", "W"], ["h"]), helper.make_node("Add", ["h", "b"], ["z"]),
         helper.make_node("Clip", ["z", "lo", "hi"], ["action"])],
        "tiny_actor",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", in_dim])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", 3])],
        [numpy_helper.from_array(W.astype(np.float32), "W"), numpy_helper.from_array(b.astype(np.float32), "b"),
         numpy_helper.from_array(np.array(-1.0, np.float32), "lo"),
         numpy_helper.from_array(np.array(1.0, np.float32), "hi")],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, str(path))
    return path


def test_onnx_policy_runs_generated_model(tmp_path):
    rng = np.random.default_rng(1)
    W, b = rng.normal(0, 0.5, (18, 3)), rng.normal(0, 0.1, 3)
    pol = OnnxPolicy(tiny_policy_onnx(tmp_path / "policy.onnx", W, b))
    assert "CPUExecutionProvider" in pol.providers
    for s in FIXTURE["samples"][:50]:
        obs = np.array(s["obs"], np.float32)
        a = pol.act(obs)
        assert a.shape == (3,) and np.all(np.abs(a) <= 1.0)
        assert a == pytest.approx(np.clip(obs @ W.astype(np.float32) + b.astype(np.float32), -1, 1), abs=1e-5)
    assert pol.last_latency_ms < 50


def test_onnx_policy_rejects_wrong_input_size_and_missing_file(tmp_path):
    with pytest.raises(ValueError):
        OnnxPolicy(tiny_policy_onnx(tmp_path / "p.onnx", np.zeros((17, 3)), np.zeros(3), in_dim=17))
    with pytest.raises(FileNotFoundError):
        OnnxPolicy(tmp_path / "missing.onnx")


def test_real_exported_policy_if_present():
    """code/training/models/docking_v3/policy.onnx is produced by the training track; only checked when present."""
    model = REPO / "code" / "training" / "models" / "docking_v3" / "policy.onnx"
    if not model.exists():
        pytest.skip("docking_v3/policy.onnx not exported yet")
    pol = OnnxPolicy(model)
    for s in FIXTURE["samples"][:20]:
        a = pol.act(np.array(s["obs"], np.float32))
        assert a.shape == (3,) and np.all(np.abs(a) <= 1.0)


# ---------------------------------------------------------------------- closed loop on a kinematic forklift
class KinematicForklift:
    """Rear-steered bicycle like env.py (omega = -v tan(steer) / L), first-order actuator lag, pallet at the
    origin with its insertion axis along +x."""

    def __init__(self, cfg: DockingConfig, dist=0.45, lat=0.06, yaw_deg=12.0, carriage_x=0.03, support=0.0):
        self.cfg, self.cx = cfg, carriage_x
        self.yaw = math.radians(yaw_deg)
        tip = np.array([-dist, lat])
        self.base = tip - (carriage_x + cfg.fork_length) * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        self.v = self.steer = 0.0
        self.lift = 0.0
        self.lift_target = 0.0
        self.support = support

    def carriage(self):
        return self.base + self.cx * np.array([math.cos(self.yaw), math.sin(self.yaw)])

    def step(self, v_cmd, omega_cmd, lift_mm, dt):
        steer_cmd = 0.0 if abs(v_cmd) < 1e-6 else math.atan(-omega_cmd * self.cfg.wheelbase / v_cmd)
        a = min(1.0, dt / 0.08)
        self.v += a * (v_cmd - self.v)
        self.steer += a * (steer_cmd - self.steer)
        self.yaw += -self.v * math.tan(self.steer) / self.cfg.wheelbase * dt
        self.base = self.base + self.v * dt * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        self.lift += float(np.clip(lift_mm / 1000 - self.lift, -0.05 * dt, 0.05 * dt))

    def inputs(self, t):
        c = self.carriage()
        rel = np.array([[math.cos(-self.yaw), -math.sin(-self.yaw)], [math.sin(-self.yaw), math.cos(-self.yaw)]]) @ -c
        visible = rel[0] > 0.06 and abs(math.atan2(rel[1], rel[0])) < math.radians(35)
        pallet = PalletEstimate(0.0, 0.0, 0.0, self.support + 0.0175, t) if visible else None
        tips = []
        for side in (1, -1):
            tip = c + self.cfg.fork_length * np.array([math.cos(self.yaw), math.sin(self.yaw)]) \
                + side * 0.045 * np.array([-math.sin(self.yaw), math.cos(self.yaw)])
            tips.append(0.005 < tip[0] < 0.2 and abs(tip[1]) < 0.1)
        return DockInputs(t, (float(c[0]), float(c[1])), self.yaw, pallet, self.lift, self.v, self.steer,
                          (1.2, 1.2, 1.2), tuple(tips))


@pytest.mark.parametrize("dist,lat,yaw", [(0.45, 0.06, 12.0), (0.50, -0.05, -10.0), (0.55, 0.0, 0.0)])
def test_classical_controller_docks_in_closed_loop(dist, lat, yaw):
    robot = KinematicForklift(CFG, dist, lat, yaw)
    ep = DockingEpisode(CFG, ClassicalPolicy(CFG), 0.0, t0=0.0)
    t, res = 0.0, None
    for _ in range(CFG.max_steps + 5):
        res = ep.step(robot.inputs(t))
        if res.done:
            break
        robot.step(res.v, res.omega, res.lift_mm, CFG.dt)
        t += CFG.dt
        assert abs(res.v) <= CFG.v_max + 1e-9  # AI-06: never above the docking speed
    assert res.done and res.success, res.message
    e = docking_errors(CFG, (0.0, 0.0, 0.0), tuple(robot.carriage()), robot.yaw)
    assert e["depth"] >= 0.8 * CFG.fork_length and abs(e["lat_tip"]) <= 0.02


def test_episode_failures():
    robot = KinematicForklift(CFG)
    pol = ClassicalPolicy(CFG)
    ep = DockingEpisode(CFG, pol, 0.0, t0=0.0)
    blind = robot.inputs(0.0)
    blind.pallet = None
    assert not ep.step(blind).done
    blind.t = 2.5
    r = ep.step(blind)
    assert r.done and not r.success and r.message.startswith("marker_lost") and r.v == 0.0

    ep = DockingEpisode(CFG, pol, 0.0, t0=0.0, params=EpisodeParams(safety_stop_timeout_s=1.0))
    t = 0.0
    for _ in range(40):
        inp = robot.inputs(t)
        inp.safety_state = 2
        r = ep.step(inp)
        if r.done:
            break
        t += CFG.dt
    assert r.done and r.message == "safety_stop"

    class Still:
        def act(self, obs):
            return np.zeros(3, np.float32)
    ep = DockingEpisode(CFG, Still(), 0.0, t0=0.0)
    for k in range(CFG.max_steps + 1):
        r = ep.step(robot.inputs(k * CFG.dt))
        if r.done:
            break
    assert r.done and r.message.startswith("timeout")


def test_rl_policy_through_episode_with_generated_onnx(tmp_path):
    """The ONNX path end to end (generated stand-in model): commands stay inside the docking limits."""
    pol = OnnxPolicy(tiny_policy_onnx(tmp_path / "policy.onnx", np.full((18, 3), 0.05), np.array([0.8, 0.0, 0.0])))
    robot = KinematicForklift(CFG)
    ep = DockingEpisode(CFG, pol, 0.0, t0=0.0)
    for k in range(20):
        r = ep.step(robot.inputs(k * CFG.dt))
        robot.step(r.v, r.omega, r.lift_mm, CFG.dt)
        assert abs(r.v) <= CFG.v_max + 1e-9 and 0.0 <= r.lift_mm <= 120.0
    assert ep.steps == 20 and np.all(np.abs(ep.prev_action) <= 1.0)
