"""JS-05: every parameter in forklift_bringup/config is one a node actually declares (typos would be silently
ignored by ROS), JS-07 systemd unit, JS-08 record topics."""

import configparser
import re
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
CFG = yaml.safe_load((PKG / "config" / "forklift.yaml").read_text())

NODE_SOURCES = {
    "mission_node": ["forklift_mission/forklift_mission/mission_node.py"],
    "lift_server": ["forklift_mission/forklift_mission/lift_server_node.py"],
    "safety_supervisor": ["forklift_safety/forklift_safety/supervisor_node.py",
                          "forklift_safety/forklift_safety/safety_core.py"],
    "fork_joint_state_publisher": ["forklift_description/scripts/fork_joint_state_publisher.py"],
    "aruco_node": ["forklift_perception/forklift_perception/aruco_node.py"],
    "initial_pose_node": ["forklift_perception/forklift_perception/initial_pose_node.py"],
    "camera_stream": ["forklift_camera_stream/forklift_camera_stream/camera_stream_node.py",
                      "forklift_camera_stream/forklift_camera_stream/mjpeg_core.py"],
    "dock_server": ["forklift_rl/forklift_rl/dock_node.py", "forklift_rl/forklift_rl/observation.py",
                    "forklift_rl/forklift_rl/docking_core.py"],
}
EXTERNAL = {"camera"}  # v4l2_camera's own parameters


def flatten(d, prefix=""):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f"{prefix}{k}.")
        else:
            yield f"{prefix}{k}", v


def declared_names(files) -> set[str]:
    text = "\n".join((SRC / f).read_text(encoding="utf-8") for f in files)
    names = set(re.findall(r'(?:P|declare_parameter)\(\s*f?"([A-Za-z0-9_.{}]+)"', text))
    names |= set(re.findall(r"^\s{4}([a-z_][a-z0-9_]*)\s*:\s*[^=\n]+=", text, re.M))  # dataclass fields
    return names


@pytest.mark.parametrize("key", sorted(CFG))
def test_every_parameter_is_declared(key):
    node = key.split("/")[-1]
    assert key.startswith("/**/"), "use namespace wildcards so the file works for every robot"
    if node in EXTERNAL:
        return
    declared = declared_names(NODE_SOURCES[node])
    for name, _ in flatten(CFG[key]["ros__parameters"]):
        prefix, _, leaf = name.rpartition(".")
        if prefix in ("docking", "episode"):   # declared in a loop over the dataclass fields
            ok = leaf in declared and f"{prefix}.{{name}}" in {n.replace("{k}", "{name}") for n in declared}
        elif node == "safety_supervisor":      # declared in a loop over SafetyParams fields
            ok = name in declared
        else:
            ok = name in declared
        assert ok, f"{node}: {name} is not declared by the node"


def test_safety_values_match_requirements():
    s = CFG["/**/safety_supervisor"]["ros__parameters"]
    assert s["max_speed"] <= 0.3 and s["raised_speed"] <= 0.1 and s["docking_speed"] <= 0.1
    assert s["tof_slow_m"] == 0.4 and s["tof_stop_m"] == 0.15 and s["server_loss_timeout_s"] == 3.0
    assert s["docking_ignored_tof"] == ["front", "front_left", "front_right"]
    m = CFG["/**/mission_node"]["ros__parameters"]
    assert m["state_hz"] == 2.0 and m["grant_retry_s"] == 0.5 and m["nav_attempts"] == 3
    assert m["travel_loaded_mm"] < s["raised_fork_mm"]


def test_record_topics_cover_js08():
    t = yaml.safe_load((PKG / "config" / "record_topics.yaml").read_text())["topics"]
    for need in ("scan", "wheel_odom", "cmd_vel", "safety_state", "safety/status", "fleet_status"):
        assert need in t


def test_systemd_unit():
    unit = configparser.ConfigParser(strict=False, interpolation=None)
    unit.read_string((PKG / "systemd" / "forklift.service").read_text())
    assert unit["Service"]["ExecStart"].endswith("forklift-start.sh")
    assert unit["Service"]["EnvironmentFile"] == "/etc/forklift/forklift.env"
    assert unit["Install"]["WantedBy"] == "multi-user.target"
    env = (PKG / "systemd" / "forklift.env.example").read_text()
    assert "ROBOT_ID=" in env and "password" not in env.split("#")[0].lower()
    sh = (PKG / "scripts" / "forklift-start.sh").read_bytes()
    assert b"\r\n" not in sh and sh.startswith(b"#!/usr/bin/env bash")


def test_launch_declares_one_namespace_per_robot():
    text = (PKG / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
    assert "PushRosNamespace(ns)" in text and '"robot_id"' in text
    for exe in ("micro_ros_agent", "sllidar_node", "scan_to_scan_filter_chain", "safety_supervisor", "mission_node",
                "dock_server", "fork_state_bridge", "aruco_node", "initial_pose_node", "lift_server", "camera_stream"):
        assert f'"{exe}"' in text, exe
