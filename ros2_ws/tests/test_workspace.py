"""Workspace-wide checks that stand in for `colcon build` on a machine without ROS:
package manifests, every XML / YAML file parses, every Python file compiles, entry points exist, the pure cores
never import rclpy, and the interface package lists every .msg/.srv/.action file."""

from __future__ import annotations

import ast
import py_compile
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

WS = Path(__file__).resolve().parents[1]
SRC = WS / "src"
PACKAGES = sorted(p.parent for p in SRC.glob("*/package.xml"))
EXPECTED = {"forklift_description", "forklift_interfaces", "forklift_bringup", "forklift_navigation",
            "forklift_camera_stream",
            "forklift_perception", "forklift_safety", "forklift_mission", "forklift_rl", "forklift_sim"}
SKIP_DIRS = {"__pycache__", ".pytest_cache"}
ROS_MODULES = {"rclpy", "launch", "launch_ros", "ament_index_python", "tf2_ros", "cv_bridge", "nav2_common",
               "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "nav2_msgs", "action_msgs", "builtin_interfaces",
               "forklift_interfaces", "rclpy.qos"}
WRAPPER_SUFFIXES = ("_node.py", "_bridge.py")


def files(*patterns):
    out = []
    for pat in patterns:
        out += [p for p in SRC.rglob(pat) if not SKIP_DIRS & set(p.parts)]
    return sorted(out)


def test_all_packages_present():
    assert {p.name for p in PACKAGES} == EXPECTED


@pytest.mark.parametrize("pkg", PACKAGES, ids=lambda p: p.name)
def test_package_manifest(pkg):
    root = ET.parse(pkg / "package.xml").getroot()
    assert root.get("format") == "3"
    assert root.findtext("name") == pkg.name
    build = root.find("export/build_type").text
    assert build in ("ament_cmake", "ament_python")
    buildtool = [e.text for e in root.findall("buildtool_depend")]
    if build == "ament_cmake":
        assert "ament_cmake" in buildtool
    else:
        # `ament_python` is a build type, not a rosdep key: listing it makes `rosdep install` fail (CI run 1)
        assert "ament_python" not in buildtool
    if build == "ament_python":
        assert (pkg / "setup.py").exists() and (pkg / "setup.cfg").exists()
        assert (pkg / "resource" / pkg.name).exists()
        assert (pkg / pkg.name / "__init__.py").exists()
        setup = (pkg / "setup.py").read_text(encoding="utf-8")
        assert f'package_name = "{pkg.name}"' in setup
        for mod, func in re.findall(r'"\w+ = ([\w.]+):(\w+)"', setup):
            path = pkg / (mod.replace(".", "/") + ".py")
            assert path.exists(), mod
            assert f"def {func}(" in path.read_text(encoding="utf-8")
    else:
        cm = (pkg / "CMakeLists.txt").read_text(encoding="utf-8")
        assert f"project({pkg.name})" in cm and "ament_package()" in cm
        for d in re.findall(r"install\(DIRECTORY ([\w ]+) DESTINATION", cm):
            for sub in d.split():
                assert (pkg / sub).is_dir(), f"{pkg.name}: installs missing directory {sub}"


def test_interfaces_cmake_lists_every_definition():
    pkg = SRC / "forklift_interfaces"
    cm = (pkg / "CMakeLists.txt").read_text()
    defs = sorted(str(p.relative_to(pkg)).replace("\\", "/") for p in pkg.glob("*/*") if p.suffix in
                  (".msg", ".srv", ".action"))
    listed = sorted(re.findall(r'"((?:msg|srv|action)/\w+\.(?:msg|srv|action))"', cm))
    assert defs == listed and len(defs) == 8
    for d in defs:  # every field line is "<type> <name>" (or a constant / separator)
        for line in (pkg / d).read_text().splitlines():
            line = line.split("#")[0].strip()
            if line and line != "---":
                assert re.match(r"^[\w/]+(\[\])?\s+\w+(\s*=\s*\S+)?$", line), f"{d}: {line!r}"


@pytest.mark.parametrize("path", files("*.xml", "*.xacro", "*.sdf", "*.rviz.xml"), ids=lambda p: p.name)
def test_xml_parses(path):
    ET.parse(path)


@pytest.mark.parametrize("path", files("*.yaml", "*.yml", "*.rviz"), ids=lambda p: str(p.relative_to(SRC)))
def test_yaml_parses(path):
    docs = list(yaml.safe_load_all(path.read_text(encoding="utf-8")))
    assert docs and docs[0] is not None


@pytest.mark.parametrize("path", files("*.py"), ids=lambda p: str(p.relative_to(SRC)))
def test_python_compiles(path, tmp_path):
    py_compile.compile(str(path), cfile=str(tmp_path / "x.pyc"), doraise=True)


CORES = [p for p in files("*.py") if p.parent.name in EXPECTED and p.parent.parent.name == p.parent.name
         and not p.name.endswith(WRAPPER_SUFFIXES)]


@pytest.mark.parametrize("path", CORES, ids=lambda p: p.name)
def test_cores_have_no_ros_imports(path):
    """All logic lives in plain Python modules (host-testable); only *_node.py / *_bridge.py touch rclpy."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names = [node.module]
        for n in names:
            assert n.split(".")[0] not in ROS_MODULES, f"{path.name} imports {n} at module level"


def test_text_files_use_lf_and_utf8():
    bad = []
    for p in files("*.py", "*.yaml", "*.xml", "*.xacro", "*.sdf", "*.msg", "*.srv", "*.action", "*.txt", "*.md",
                   "*.cfg", "*.service", "*.example", "*.sh", "*.rviz"):
        raw = p.read_bytes()
        try:
            raw.decode("utf-8")
        except UnicodeDecodeError:
            bad.append(f"{p.relative_to(SRC)}: not utf-8")
        if b"\r\n" in raw:
            bad.append(f"{p.relative_to(SRC)}: CRLF")
    assert not bad, bad
