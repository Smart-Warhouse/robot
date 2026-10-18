import importlib.util
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("gen", PKG / "scripts" / "generate_world.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def test_world_is_generated_from_the_current_map(warehouse_map_path):
    wh = yaml.safe_load(warehouse_map_path.read_text(encoding="utf-8"))
    assert gen.world_sdf(wh).encode("utf-8") == (PKG / "worlds" / "warehouse.sdf").read_bytes(), \
        "worlds/warehouse.sdf is stale: run scripts/generate_world.py"
    pgm, meta = gen.occupancy(wh)
    assert pgm == (PKG / "maps" / "warehouse_sim.pgm").read_bytes(), "maps/warehouse_sim.pgm is stale"
    assert meta.encode("utf-8") == (PKG / "maps" / "warehouse_sim.yaml").read_bytes(), "maps/warehouse_sim.yaml is stale"


def test_world_contents(warehouse_map_path):
    root = ET.parse(PKG / "worlds" / "warehouse.sdf").getroot()
    assert root.tag == "sdf" and root.get("version") == "1.9"
    names = {m.get("name") for m in root.iter("model")}
    assert {"wall_south", "wall_north", "wall_west", "wall_east", "rack_R1", "rack_R2", "ground_plane"} <= names
    rack = next(m for m in root.iter("model") if m.get("name") == "rack_R1")
    x, y = (float(v) for v in rack.find("pose").text.split()[:2])
    assert (x, y) == (2.0, 2.0)  # centre of the R1 zone polygon
    # Fortress system names (ROS 2 Humble's ros_gz); the sensors system renders the LiDAR and the camera
    plugins = {p.get("name") for p in root.iter("plugin")}
    assert "ignition::gazebo::systems::Sensors" in plugins and "ignition::gazebo::systems::Imu" in plugins
    dims = yaml.safe_load((PKG.parent / "forklift_description" / "config" / "dimensions.yaml").read_text())
    wall = next(m for m in root.iter("model") if m.get("name") == "wall_north")
    assert float(wall.find("link/collision/geometry/box/size").text.split()[2]) >= dims["laser_z"] + 0.05  # EN-07


def test_occupancy_map(warehouse_map_path):
    meta = yaml.safe_load((PKG / "maps" / "warehouse_sim.yaml").read_text())
    data = (PKG / "maps" / "warehouse_sim.pgm").read_bytes()
    head = data.split(b"\n", 4)
    assert head[0] == b"P5"
    w, h = (int(v) for v in head[2].split())
    px = head[4]
    assert len(px) == w * h
    res, (ox, oy, _) = meta["resolution"], meta["origin"]

    def cell(x, y):
        i, j = int((x - ox) / res), int((y - oy) / res)
        return px[(h - 1 - j) * w + i]
    assert cell(1.0, 0.6) == 254          # J1 corridor: free
    assert cell(2.0, 2.0) == 0            # inside rack R1
    assert cell(-0.05, 2.0) == 0          # behind the west wall
    assert cell(5.9, 3.9) == 254


def test_sim_topics_match_the_bridge_and_the_stack():
    """Every Gazebo topic of the sim URDF is bridged, and the ROS names are those of the real robot (docs/13)."""
    xacro = (PKG / "urdf" / "forklift_gz.urdf.xacro").read_text(encoding="utf-8")
    gz_topics = set(re.findall(r"<(?:topic|odom_topic)>/\$\{ns\}/([\w/]+)</", xacro))
    bridge = yaml.safe_load((PKG / "config" / "bridge.yaml").read_text())
    bridged = {b["gz_topic_name"].replace("/ROBOT_ID/", "") for b in bridge}
    assert gz_topics <= bridged
    for b in bridge:
        assert b["ros_topic_name"] == b["gz_topic_name"] and b["ros_topic_name"].startswith("/ROBOT_ID/")
        assert b["gz_type_name"].startswith("ignition.msgs.")
    assert {"scan", "imu/data", "image_raw", "wheel_odom", "cmd_vel"} <= bridged   # firmware / sllidar / v4l2 names
    ekf = (PKG.parent / "forklift_navigation" / "config" / "ekf.yaml").read_text()
    assert "odom0: wheel_odom" in ekf and "imu0: imu/data" in ekf
