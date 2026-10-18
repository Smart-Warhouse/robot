"""URDF/Xacro expands without ROS (pip `xacro`), parses, and forms the TF tree of JS-06."""

from __future__ import annotations

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def urdf() -> ET.Element:
    xacro = pytest.importorskip("xacro")
    doc = xacro.process_file(str(PKG / "urdf" / "forklift.urdf.xacro"),
                             mappings={"dims_file": (PKG / "config" / "dimensions.yaml").resolve().as_posix()})
    return ET.fromstring(doc.toxml())


def tree(root: ET.Element) -> dict[str, str]:
    return {j.find("child").get("link"): j.find("parent").get("link") for j in root.iter("joint")}


def test_single_root_is_base_link(urdf):
    links = {l.get("name") for l in urdf.iter("link")}
    parents = tree(urdf)
    roots = links - set(parents)
    assert roots == {"base_link"}
    assert set(parents.values()) <= links


def test_required_frames_and_chain(urdf):
    parents = tree(urdf)
    for frame in ("laser", "imu", "fork", "camera_link", "camera_optical_frame", "fork_tip_left", "fork_tip_right"):
        assert frame in parents, frame
    assert parents["laser"] == parents["imu"] == parents["fork"] == "base_link"
    assert parents["camera_link"] == "fork"  # ME-05: camera rides on the carriage
    assert parents["camera_optical_frame"] == "camera_link"
    dims = yaml.safe_load((PKG / "config" / "dimensions.yaml").read_text())
    for s in dims["tof"]:
        assert parents[f"tof_{s['name']}"] == "base_link"


def test_lift_joint_matches_dimensions(urdf):
    dims = yaml.safe_load((PKG / "config" / "dimensions.yaml").read_text())
    j = next(j for j in urdf.iter("joint") if j.get("name") == "fork_lift_joint")
    assert j.get("type") == "prismatic"
    assert float(j.find("limit").get("upper")) == pytest.approx(dims["lift_max"])
    x, _, z = (float(v) for v in j.find("origin").get("xyz").split())
    assert x == pytest.approx(dims["carriage_x"]) and z == pytest.approx(dims["fork_clearance"])
    cam = next(j for j in urdf.iter("joint") if j.get("name") == "camera_joint")
    pitch = float(cam.find("origin").get("rpy").split()[1])
    assert math.radians(15) <= pitch <= math.radians(25)  # ME-05


def test_dimensions_match_training_config():
    """The URDF and the RL simulator must describe the same forklift (else the policy sees a different robot)."""
    train = PKG.parents[2] / "training" / "forklift_rl" / "config.yaml"
    if not train.exists():
        pytest.skip("training config not present")
    t = yaml.safe_load(train.read_text(encoding="utf-8"))["forklift"]
    d = yaml.safe_load((PKG / "config" / "dimensions.yaml").read_text())
    for key in ("wheelbase", "body_length", "body_width", "carriage_x", "fork_length", "fork_thickness",
                "fork_spacing", "fork_clearance", "lift_max"):
        assert d[key] == pytest.approx(t[key]), key


def test_fork_joint_position_clamps():
    spec = importlib.util.spec_from_file_location("fjsp", PKG / "scripts" / "fork_joint_state_publisher.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.fork_joint_position(60, 0.12) == pytest.approx(0.06)
    assert mod.fork_joint_position(-5, 0.12) == 0.0
    assert mod.fork_joint_position(500, 0.12) == pytest.approx(0.12)
