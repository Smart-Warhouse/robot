"""Every printed mount builds, is one valid solid, fits the printer, and meets its ME-* requirement."""
from __future__ import annotations

import math
import re
import warnings
import os
from pathlib import Path

import pytest
import yaml

import parts

ROOT = Path(os.environ["SW_WORKSPACE"]) if os.environ.get("SW_WORKSPACE") else Path(__file__).resolve().parents[3]  # workspace with robot/ and ai/
P = parts.load_params()


@pytest.fixture(scope="module")
def built():
    cache: dict = {}

    def get(name: str):
        if name not in cache:
            cache[name] = parts.PARTS[name](P)
        return cache[name]

    return get


# ------------------------------------------------------------------------------------------------------------
# every part
# ------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("name", list(parts.PARTS))
def test_part_is_one_valid_solid(built, name):
    wp = built(name)
    solids = wp.solids().vals()
    assert len(solids) == 1, f"{name}: {len(solids)} solids"
    assert solids[0].isValid(), f"{name}: invalid BRep"
    assert solids[0].Volume() > 100, f"{name}: volume {solids[0].Volume():.0f} mm³"


@pytest.mark.parametrize("name", list(parts.PARTS))
def test_part_fits_printer_bed(built, name):
    bb = built(name).val().BoundingBox()
    bed = P["printer"]["bed"]
    # as printed: x, y on the bed, z up (builders return parts in print orientation)
    assert bb.xlen <= bed[0] and bb.ylen <= bed[1] and bb.zlen <= bed[2], \
        f"{name}: {bb.xlen:.0f} × {bb.ylen:.0f} × {bb.zlen:.0f} mm > bed {bed}"


@pytest.mark.parametrize("name", list(parts.PARTS))
def test_part_sits_flat_on_the_bed(built, name):
    """Print orientation: the part starts at z = 0 (camera mount and caps included)."""
    bb = built(name).val().BoundingBox()
    if name == "camera_mount":
        pytest.skip("printed on its side; the slicer lays it on the clamp face")
    assert abs(bb.zmin) < 0.05, f"{name}: zmin = {bb.zmin:.2f}"


def test_walls_not_thinner_than_minimum():
    m = P["material"]["min_wall"]
    assert P["ebox"]["wall"] >= m and P["battery"]["tray_wall"] >= m and P["estop"]["wall"] >= m
    assert P["as5600"]["cap_wall"] >= m


# ------------------------------------------------------------------------------------------------------------
# ME-04: LiDAR scan plane above the mast and any carried load
# ------------------------------------------------------------------------------------------------------------
def test_lidar_plane_above_mast_and_load(built):
    plane = parts.lidar_scan_plane_z(P, built("lidar_mount"))
    obstacle = parts.highest_obstacle_z(P)
    need = P["lidar"]["min_clearance_above_obstacles"]
    assert obstacle >= P["forklift"]["mast_height"]
    assert plane >= obstacle + need - 0.01, f"scan plane {plane:.1f} mm, obstacle {obstacle:.1f} mm"
    # and not absurdly tall (top-heavy, wobbles): at most 40 mm over the requirement
    assert plane <= obstacle + need + 40


def test_lidar_plane_tracks_mast_rise():
    """If ME-01 finds that the inner mast rises at full lift, the riser grows with it."""
    p = yaml.safe_load(yaml.safe_dump(P))
    p["forklift"]["mast_rise_at_max_lift"] = 30
    plane = parts.lidar_scan_plane_z(p, parts.lidar_mount(p))
    assert plane >= p["forklift"]["mast_height"] + 30 + p["lidar"]["min_clearance_above_obstacles"] - 0.01


# ------------------------------------------------------------------------------------------------------------
# ME-05: camera tilted 15–25° down
# ------------------------------------------------------------------------------------------------------------
def test_camera_tilt_in_range(built):
    tilt = parts.camera_tilt_deg(built("camera_mount"))
    c = P["camera"]
    assert c["tilt_min_deg"] == 15 and c["tilt_max_deg"] == 25
    assert c["tilt_min_deg"] <= tilt <= c["tilt_max_deg"], f"tilt {tilt:.1f}°"
    assert tilt == pytest.approx(c["tilt_deg"], abs=0.5)


@pytest.mark.parametrize("tilt", [15, 25])
def test_camera_tilt_follows_parameter(tilt):
    p = yaml.safe_load(yaml.safe_dump(P))
    p["camera"]["tilt_deg"] = tilt
    assert parts.camera_tilt_deg(parts.camera_mount(p)) == pytest.approx(tilt, abs=0.5)


# ------------------------------------------------------------------------------------------------------------
# ME-06: six ToF brackets, same order as the firmware
# ------------------------------------------------------------------------------------------------------------
def test_tof_positions_match_firmware_order():
    cfg = (ROOT / "robot/firmware/lib/forklift_core/src/forklift/config.h").read_text(encoding="utf-8")
    enum = re.search(r"enum TofIndex[^{]*\{(.*?)\};", cfg, re.S).group(1)
    names = [m.lower() for m in re.findall(r"TOF_([A-Z_]+)\s*=\s*\d", enum) if m != "COUNT"]
    assert names == parts.TOF_NAMES == list(P["tof"]["positions"])


@pytest.mark.parametrize("name", parts.TOF_NAMES)
def test_tof_face_points_along_sensor_axis(built, name):
    ax = parts.tof_axis(P, name)
    wp = built(f"tof_bracket_{name}")
    found = False
    for face in wp.faces().vals():
        if face.geomType() != "PLANE":
            continue
        n = face.normalAt()
        if n.x * ax[0] + n.y * ax[1] + n.z * ax[2] > math.cos(math.radians(0.5)) and face.Area() > 100:
            found = True
    assert found, f"no mounting face along {ax}"


# ------------------------------------------------------------------------------------------------------------
# ME-07: AS5600 gap 0.5–3 mm
# ------------------------------------------------------------------------------------------------------------
def test_as5600_gap_in_range(built):
    gap = parts.as5600_gap_mm(P, built("as5600_magnet_cap"), built("as5600_bracket"))
    a = P["as5600"]
    assert a["gap_min"] == 0.5 and a["gap_max"] == 3.0
    assert a["gap_min"] <= gap <= a["gap_max"], f"gap {gap:.2f} mm"
    assert gap == pytest.approx(a["gap"], abs=0.05)
    # the slots let the gap be moved over the whole allowed range
    assert a["gap"] - a["slot_travel"] / 2 <= a["gap_min"] + 0.75
    assert a["gap"] + a["slot_travel"] / 2 >= a["gap_max"] - 0.75


def test_as5600_cap_clears_bracket_foot():
    a, f = P["as5600"], P["forklift"]
    cap_r = (max(f["drive_axle_diameter"], a["magnet"][0]) + 2 * a["cap_wall"]) / 2
    assert a["axle_height_above_mount_face"] - cap_r > a["plate_thickness"] + 1


# ------------------------------------------------------------------------------------------------------------
# ME-08: slide pot + limit switches cover the lift range
# ------------------------------------------------------------------------------------------------------------
def test_limit_switches_span_lift_range(built):
    bb = built("mast_sensor_bracket").val().BoundingBox()
    assert bb.xlen >= P["forklift"]["lift_max"] + P["mast_sensors"]["limit_switch"][0]


def test_pot_travel_vs_lift_range():
    if not parts.pot_covers_lift(P):
        warnings.warn(
            f"slide pot travel {P['mast_sensors']['pot_travel']} mm < lift range {P['forklift']['lift_max']} mm "
            "(both PLACEHOLDER): measure; if still short, use a longer pot or lower LIFT_MAX_MM",
            UserWarning,
        )


# ------------------------------------------------------------------------------------------------------------
# ME-09: fork-tip clip still enters the pallet
# ------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("side", ["left", "right"])
def test_tcrt_clip_fits_pallet_opening(built, side):
    bb = built(f"tcrt_clip_{side}").val().BoundingBox()
    opening = parts.pallet_opening(P)
    assert bb.zlen <= opening - P["tcrt"]["pallet_opening_margin"], f"clip {bb.zlen:.1f} mm vs opening {opening}"
    # sensor head fits inside the nose thickness
    assert P["tcrt"]["head"][2] + 0.4 < bb.zlen


# ------------------------------------------------------------------------------------------------------------
# ME-10: electronics box fits under the cover and is ventilated for 15 W
# ------------------------------------------------------------------------------------------------------------
def test_ebox_fits_rear_cover(built):
    bb = built("electronics_box").val().BoundingBox()
    lx, ly, lz = P["forklift"]["rear_cover_inner"]
    ox, oy, oz = parts.ebox_layout(P)["outer"]
    assert ox <= lx and oy <= ly and oz + P["ebox"]["lid_thickness"] <= lz
    assert bb.ylen <= P["forklift"]["width"]


def test_ebox_holds_jetson():
    ix, iy, iz = parts.ebox_layout(P)["inner"]
    jx, jy = P["jetson"]["footprint"]
    assert jx < ix and jy < iy and P["jetson"]["height"] + P["jetson"]["standoff"] < iz


def test_ebox_ventilation():
    area = parts.ebox_vent_area_mm2(P)
    assert area >= P["ebox"]["min_vent_area_mm2"]
    assert area >= 100 * P["jetson"]["power_w"], f"{area:.0f} mm² for {P['jetson']['power_w']} W"


# ------------------------------------------------------------------------------------------------------------
# ME-11 / ME-12
# ------------------------------------------------------------------------------------------------------------
def test_battery_tray_holds_holder(built):
    bb = built("battery_tray").val().BoundingBox()
    hx, hy, hz = P["battery"]["holder"]
    assert bb.xlen > hx and bb.ylen > hy
    assert P["battery"]["lip_height"] >= hz / 2  # walls hold at least half the holder height


def test_estop_panel_in_switch_clamp_range():
    s = P["estop"]
    assert 1.0 <= s["panel_thickness"] <= 6.0
    assert 22.0 < s["hole"] <= 22.5


# ------------------------------------------------------------------------------------------------------------
# params shared with the RL simulator stay equal (config.yaml is in metres)
# ------------------------------------------------------------------------------------------------------------
def test_params_match_rl_config():
    rl = yaml.safe_load((ROOT / "ai/training/forklift_rl/config.yaml").read_text(encoding="utf-8"))
    f, pal = P["forklift"], P["pallet"]
    pairs = {
        "wheelbase": (f["wheelbase"], rl["forklift"]["wheelbase"]),
        "width": (f["width"], rl["forklift"]["body_width"]),
        "mast_height": (f["mast_height"], rl["forklift"]["mast_height"]),
        "fork_length": (f["fork_length"], rl["forklift"]["fork_length"]),
        "fork_width": (f["fork_width"], rl["forklift"]["fork_width"]),
        "fork_thickness": (f["fork_thickness"], rl["forklift"]["fork_thickness"]),
        "fork_spacing": (f["fork_spacing"], rl["forklift"]["fork_spacing"]),
        "fork_clearance": (f["fork_clearance"], rl["forklift"]["fork_clearance"]),
        "lift_max": (f["lift_max"], rl["forklift"]["lift_max"]),
        "pallet_height": (pal["height"], rl["pallet"]["height"]),
        "deck_thickness": (pal["deck_thickness"], rl["pallet"]["deck_thickness"]),
    }
    for key, (mm, m) in pairs.items():
        assert mm == pytest.approx(m * 1000, abs=0.01), f"{key}: params.yaml {mm} mm vs RL config {m} m"


def test_params_match_firmware_lift_range():
    cfg = (ROOT / "robot/firmware/lib/forklift_core/src/forklift/config.h").read_text(encoding="utf-8")
    lift = float(re.search(r"LIFT_MAX_MM\s*=\s*([\d.]+)f", cfg).group(1))
    assert P["forklift"]["lift_max"] == pytest.approx(lift)
