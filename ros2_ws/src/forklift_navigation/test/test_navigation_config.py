"""NV-05..NV-12 parameters and the NV-08 filter masks, checked without ROS."""

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("masks", PKG / "scripts" / "make_filter_masks.py")
masks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(masks)
NAV = yaml.safe_load((PKG / "config" / "nav2_params.yaml").read_text())


def test_planner_controller_and_tolerances():
    gb = NAV["planner_server"]["ros__parameters"]["GridBased"]
    assert gb["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"            # NV-05
    assert gb["motion_model_for_search"] == "REEDS_SHEPP" and gb["minimum_turning_radius"] > 0
    dims = yaml.safe_load((PKG.parent / "forklift_description" / "config" / "dimensions.yaml").read_text())
    assert gb["minimum_turning_radius"] >= dims["wheelbase"] / math.tan(math.radians(30)) - 0.01
    c = NAV["controller_server"]["ros__parameters"]
    assert c["controller_frequency"] == 20.0                                  # NV-06
    assert c["FollowPath"]["plugin"].endswith("RegulatedPurePursuitController")
    assert c["FollowPath"]["use_rotate_to_heading"] is False                   # Ackermann
    assert c["FollowPath"]["desired_linear_vel"] <= 0.3                        # NFR-03
    g = c["general_goal_checker"]
    assert g["xy_goal_tolerance"] == 0.05 and g["yaw_goal_tolerance"] == pytest.approx(math.radians(5), abs=1e-3)
    assert "spin" not in NAV["behavior_server"]["ros__parameters"]["behavior_plugins"]


def test_costmap_filters_configured():
    gc = NAV["global_costmap"]["global_costmap"]["ros__parameters"]
    assert gc["filters"] == ["keepout_filter", "speed_filter"]
    assert gc["speed_filter"]["speed_limit_topic"] == NAV["controller_server"]["ros__parameters"]["speed_limit_topic"]
    assert NAV["speed_costmap_filter_info_server"]["ros__parameters"]["type"] == 1
    assert NAV["keepout_costmap_filter_info_server"]["ros__parameters"]["type"] == 0
    for cm in ("local_costmap", "global_costmap"):
        assert NAV[cm][cm]["ros__parameters"]["resolution"] == 0.02


@pytest.mark.parametrize("xml, plan", [("navigate_lane.xml", "ComputePathToPose"),
                                       ("navigate_through_lanes.xml", "ComputePathThroughPoses")])
def test_behavior_trees_have_no_spin(xml, plan):
    # both bt_navigator trees are loaded at configure; Nav2's default through-poses tree has Spin, which is not
    # registered here, so the launch file must point default_nav_through_poses_bt_xml at ours
    root = ET.parse(PKG / "behavior_trees" / xml).getroot()
    tags = {e.tag for e in root.iter()}
    assert "Spin" not in tags and {plan, "FollowPath", "BackUp", "Wait"} <= tags
    assert xml in (PKG / "launch" / "navigation.launch.py").read_text(encoding="utf-8")


def test_masks_are_up_to_date_with_the_warehouse_map(warehouse_map_path, tmp_path):
    masks.generate(warehouse_map_path, tmp_path)
    for name in ("keepout_mask.pgm", "keepout_mask.yaml", "speed_mask.pgm", "speed_mask.yaml"):
        assert (tmp_path / name).read_bytes() == (PKG / "maps" / name).read_bytes(), \
            f"maps/{name} is stale: run scripts/make_filter_masks.py"


def pixel(path, x, y, res=0.02):
    w, h, data = masks.read_pgm(path)
    r, c = h - 1 - int(y / res), int(x / res)
    return data[r * w + c]


def test_mask_contents(warehouse_map_path):
    keep = PKG / "maps" / "keepout_mask.pgm"
    assert masks.read_pgm(keep)[:2] == (300, 200)
    assert pixel(keep, 5.75, 3.3) == 0        # KO1 keep-out
    assert pixel(keep, 0.3, 0.5) == 0         # walkway
    assert pixel(keep, 3.0, 0.6) == 255       # J2 is free
    wh = yaml.safe_load(warehouse_map_path.read_text(encoding="utf-8"))
    for n, (x, y) in wh["nodes"].items():     # no lane node may sit in a keep-out zone
        assert pixel(keep, x, y) == 255, n
    speed = PKG / "maps" / "speed_mask.pgm"
    occ = round(100 * (255 - pixel(speed, 1.0, 3.8)) / 255)
    assert occ == 50                          # charging area at 50 %
    assert pixel(speed, 3.0, 2.0) == 255      # aisles unrestricted
    yml = yaml.safe_load((PKG / "maps" / "speed_mask.yaml").read_text())
    assert yml["mode"] == "scale" and yml["free_thresh"] == 0.0 and yml["occupied_thresh"] == 1.0


def test_slam_and_ekf():
    m = yaml.safe_load((PKG / "config" / "slam_mapping.yaml").read_text())["slam_toolbox"]["ros__parameters"]
    assert m["mode"] == "mapping" and m["resolution"] == 0.02 and m["scan_topic"] == "scan_filtered"
    loc = yaml.safe_load((PKG / "config" / "slam_localization.yaml").read_text())["slam_toolbox"]["ros__parameters"]
    assert loc["mode"] == "localization"
    e = yaml.safe_load((PKG / "config" / "ekf.yaml").read_text())["ekf_filter_node"]["ros__parameters"]
    assert e["frequency"] == 50.0 and e["odom0"] == "wheel_odom" and e["imu0"] == "imu/data"   # NV-03
    assert len(e["odom0_config"]) == 15 and len(e["process_noise_covariance"]) == 225
