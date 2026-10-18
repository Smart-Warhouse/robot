"""Parametric 3D-printed mounts for the autonomous forklift (requirements §2, ME-04 … ME-12).

Every dimension comes from params.yaml. Each builder takes the params dict and returns a cq.Workplane holding
ONE solid in its print orientation (flat side on the bed, z up). `PARTS` maps the exported file name to its
builder. The geometric helpers at the bottom (lidar_scan_plane_z, ebox_vent_area_mm2, …) are what the tests
check against the requirements.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

import cadquery as cq
import yaml

HERE = Path(__file__).resolve().parent


def load_params(path: Path = HERE / "params.yaml") -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


# ------------------------------------------------------------------------------------------------------------
# small helpers
# ------------------------------------------------------------------------------------------------------------
def box(x: float, y: float, z: float, at=(0.0, 0.0, 0.0), centered=(True, True, False)) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z, centered=centered).translate(at)


def cylinders_z(points, d: float, z0: float, h: float) -> cq.Workplane:
    """Vertical cylinders (hole tools or bosses) at (x, y) points, from z0 up by h."""
    return cq.Workplane("XY").workplane(offset=z0).pushPoints(list(points)).circle(d / 2).extrude(h)


def slot_z(x: float, y: float, w: float, length: float, z0: float, h: float, angle: float = 0) -> cq.Workplane:
    """Vertical stadium-shaped slot tool, long axis along x rotated by angle (deg)."""
    return (cq.Workplane("XY").workplane(offset=z0).center(x, y).transformed(rotate=(0, 0, angle))
            .slot2D(length, w).extrude(h))


def screw(p: dict, name: str) -> dict:
    return p["screws"][name]


def one_solid(wp: cq.Workplane) -> cq.Workplane:
    """Fuse everything into a single clean solid (the tests require exactly one)."""
    solids = wp.solids().vals()
    s = solids[0]
    for other in solids[1:]:
        s = s.fuse(other)
    return cq.Workplane("XY").newObject([s.clean()])


# ------------------------------------------------------------------------------------------------------------
# ME-04 LiDAR mount on the overhead guard
# ------------------------------------------------------------------------------------------------------------
def highest_obstacle_z(p: dict) -> float:
    """Highest point the LiDAR must look over: the mast (incl. its rise at full lift) or a carried load."""
    f, pal = p["forklift"], p["pallet"]
    mast = f["mast_height"] + f["mast_rise_at_max_lift"]
    fork_top = f["fork_clearance"] + f["lift_max"] + f["fork_thickness"]
    load_top = fork_top + pal["deck_thickness"] + pal["load_height"]
    return max(mast, load_top)


def lidar_riser_height(p: dict) -> float:
    li = p["lidar"]
    target = highest_obstacle_z(p) + li["min_clearance_above_obstacles"]
    h = target - p["forklift"]["overhead_guard_top"] - li["base_plate"][2] - li["top_plate"][2] - li["scan_plane_above_base"]
    return max(h, 5.0)


def lidar_mount(p: dict) -> cq.Workplane:
    li = p["lidar"]
    m3, m25 = screw(p, "m3"), screw(p, li["screw"])
    bx, by, bt = li["base_plate"]
    tx, ty, tt = li["top_plate"]
    ro, rw = li["riser_outer"], li["riser_wall"]
    h = lidar_riser_height(p)

    base = box(bx, by, bt)
    riser = (cq.Workplane("XY").workplane(offset=bt).rect(ro, ro).rect(ro - 2 * rw, ro - 2 * rw).extrude(h))
    top = box(tx, ty, tt, at=(0, 0, bt + h))
    part = base.union(riser).union(top)

    s = li["base_hole_spacing"] / 2
    part = part.cut(cylinders_z([(sx * s, sy * s) for sx in (-1, 1) for sy in (-1, 1)], m3["clear"], -1, bt + 2))
    # zip-tie slots on the centre lines, outside the riser, for clamping round the roof bars
    zt = (bx / 2 + ro / 2) / 2
    for x, y, a in ((zt, 0, 90), (-zt, 0, 90), (0, zt, 0), (0, -zt, 0)):
        part = part.cut(slot_z(x, y, 2.5, 6, -1, bt + 2, a))
    hx, hy = (v / 2 for v in li["hole_spacing"])
    part = part.cut(cylinders_z([(sx * hx, sy * hy) for sx in (-1, 1) for sy in (-1, 1)], m25["clear"],
                                bt + h - 1, tt + 2))
    part = part.cut(cylinders_z([(0, 0)], li["cable_hole"], bt + h - 1, tt + 2))
    cw, ch = li["cable_slot"]
    part = part.cut(box(rw * 3, cw, ch, at=(ro / 2 - rw / 2, 0, bt)))
    return one_solid(part)


def lidar_scan_plane_z(p: dict, part: cq.Workplane) -> float:
    """Scan plane height above the floor, measured from the built part's height."""
    bb = part.val().BoundingBox()
    return p["forklift"]["overhead_guard_top"] + bb.zlen + p["lidar"]["scan_plane_above_base"]


# ------------------------------------------------------------------------------------------------------------
# ME-05 camera mount on the fork carriage, tilted down
# ------------------------------------------------------------------------------------------------------------
def camera_mount(p: dict) -> cq.Workplane:
    """Printed lying on its side: x = forward (camera view), z = up on the forklift, y = across the carriage.

    A U-clamp hooks over the carriage cross bar (M3 set screw), an arm rises to the camera height and a
    plate tilted `tilt_deg` down carries the Camera Module 3 on four M2 bosses.
    """
    c, f, m2, m3 = p["camera"], p["forklift"], screw(p, "m2"), screw(p, "m3")
    cl = p["material"]["clearance_slide"]
    w, wall, arm_t, pt = c["clamp_width"], c["clamp_wall"], c["arm_thickness"], c["plate_thickness"]
    bt, bh = f["carriage_bar_thickness"] + 2 * cl, f["carriage_bar_height"] + cl

    # clamp: U opening downwards around the bar, bar centred at x = 0
    cx = bt + 2 * wall
    clamp = box(cx, w, bh + wall).cut(box(bt, w + 2, bh, at=(0, 0, -0.01)))
    clamp = clamp.cut(cq.Workplane("YZ").workplane(offset=-cx).center(0, bh / 2).circle(m3["pilot"] / 2)
                      .extrude(cx / 2))
    # arm rises from the clamp's front wall
    arm_top = bh + wall + c["height_above_fork_top"]
    arm_x0 = cx / 2 - arm_t
    arm = box(arm_t, w, arm_top - bh, at=(arm_x0 + arm_t / 2, 0, bh))

    # camera plate, built with its back face on x = 0, then tilted down about y and hung on the arm
    bw, bhh = c["board"][0] + 4, c["board"][1] + 4
    plate = box(pt, bw, bhh, centered=(False, True, False))
    hx, hy = c["hole_spacing"]
    z_row0 = 2 + c["hole_offset_from_bottom"]
    holes = [(-hx / 2, z_row0), (hx / 2, z_row0), (-hx / 2, z_row0 + hy), (hx / 2, z_row0 + hy)]
    for y, z in holes:
        boss = cq.Workplane("YZ").workplane(offset=pt).center(y, z).circle(2.5).extrude(c["standoff"])
        plate = plate.union(boss)
        plate = plate.cut(cq.Workplane("YZ").workplane(offset=-0.5).center(y, z).circle(m2["pilot"] / 2)
                          .extrude(pt + c["standoff"] + 1))
    plate = plate.rotate((0, 0, 0), (0, 1, 0), c["tilt_deg"])
    plate = plate.translate((arm_x0 + arm_t - 1.5, 0, arm_top - 10))
    return one_solid(clamp.union(arm).union(plate))


def camera_tilt_deg(part: cq.Workplane) -> float:
    """Downward tilt of the camera plate, from the biggest planar face that faces forward-and-down."""
    best, best_area = None, 0.0
    for face in part.faces().vals():
        if face.geomType() != "PLANE":
            continue
        n = face.normalAt()
        if abs(n.y) > 1e-3 or n.x < 0.2 or n.z > -0.05:
            continue
        if face.Area() > best_area:
            best, best_area = n, face.Area()
    if best is None:
        return 0.0
    return math.degrees(math.atan2(-best.z, best.x))


# ------------------------------------------------------------------------------------------------------------
# ME-06 ToF brackets (6 positions, order = firmware TofIndex)
# ------------------------------------------------------------------------------------------------------------
def tof_bracket(p: dict, name: str) -> cq.Workplane:
    """Base flange on the body face (z = 0), a wedge block whose top face points the sensor along
    (yaw about y, pitch about x), and two M2 standoffs for the VL53L1X board on that face."""
    t, m2 = p["tof"], screw(p, "m2")
    pos = t["positions"][name]
    bw, bh, _ = t["board"]
    bx, by = bw + 4, bh + 4
    base_t = t["base_thickness"]
    yaw, pitch = math.radians(pos["yaw"]), math.radians(pos["pitch"])
    rise = bx / 2 * abs(math.tan(yaw)) + by / 2 * abs(math.tan(pitch))
    z0 = base_t + 2 + rise  # centre of the tilted face

    flange_len = bx + 2 * t["flange"]
    part = box(flange_len, by, base_t)
    part = part.cut(cylinders_z([(-(bx / 2 + t["flange"] / 2), 0), (bx / 2 + t["flange"] / 2, 0)], m2["clear"],
                                -1, base_t + 2))
    block = box(bx, by, z0 + rise + 2)

    def place(wp: cq.Workplane) -> cq.Workplane:
        return (wp.rotate((0, 0, 0), (1, 0, 0), -pos["pitch"]).rotate((0, 0, 0), (0, 1, 0), pos["yaw"])
                .translate((0, 0, z0)))

    cutter = place(box(400, 400, 200))
    block = block.cut(cutter)
    standoffs = place(cylinders_z([tuple(h) for h in t["holes"]], 4.0, -0.5, t["standoff"] + 0.5))
    pilots = place(cylinders_z([tuple(h) for h in t["holes"]], m2["pilot"], -6, t["standoff"] + 7))
    part = part.union(block).union(standoffs).cut(pilots)
    return one_solid(part)


def tof_axis(p: dict, name: str) -> tuple[float, float, float]:
    """Sensor axis (unit vector, bracket frame) the bracket is designed for."""
    pos = p["tof"]["positions"][name]
    pitch, yaw = math.radians(-pos["pitch"]), math.radians(pos["yaw"])
    # +z rotated about x by -pitch, then about y by yaw (same order as tof_bracket.place)
    y1, z1 = -math.sin(pitch), math.cos(pitch)
    return (z1 * math.sin(yaw), y1, z1 * math.cos(yaw))


# ------------------------------------------------------------------------------------------------------------
# ME-10 electronics box behind the seat (Jetson + ESP32 perfboard) with ventilation, and its lid
# ------------------------------------------------------------------------------------------------------------
def ebox_layout(p: dict) -> dict:
    e, j, el = p["ebox"], p["jetson"], p["electronics"]
    g = e["gap"]
    jx, jy = j["footprint"]
    px, py = el["perfboard"][1], el["perfboard"][0]  # perfboard turned 90°: long side along x
    inner_x = g + max(jx, px) + g
    inner_y = g + jy + g + py + g
    inner_z = e["inner_height"]
    w = e["wall"]
    return {
        "inner": (inner_x, inner_y, inner_z),
        "outer": (inner_x + 2 * w, inner_y + 2 * w, inner_z + e["floor"]),
        "jetson_c": (0.0, -inner_y / 2 + g + jy / 2),
        "perf_c": (0.0, inner_y / 2 - g - py / 2),
        "perf_size": (px, py),
    }


def _ebox_wall_slots(p: dict) -> list[tuple[str, float, float, float]]:
    """Vent slots on the two long walls: (wall, along, z_centre, length)."""
    e = p["ebox"]
    lay = ebox_layout(p)
    ix, iy, iz = lay["inner"]
    sw, sl = e["vent_slot"]
    pitch = e["vent_pitch"]
    zc = e["floor"] + iz / 2
    out = []
    n_x = int((ix - 20) // pitch)
    for wall in ("y_min", "y_max"):
        for i in range(n_x):
            out.append((wall, -((n_x - 1) * pitch) / 2 + i * pitch, zc, sl))
    return out


def _ebox_lid_slots(p: dict) -> list[tuple[float, float, float]]:
    """Lid slots over the Jetson fan: (x, y, length) of slots running along y."""
    e, j = p["ebox"], p["jetson"]
    lay = ebox_layout(p)
    jx, jy = j["footprint"]
    pitch, sw = e["vent_pitch"], e["vent_slot"][0]
    n = int((jx - 20) // pitch)
    length = jy - 24
    _, cy = lay["jetson_c"]
    return [(-((n - 1) * pitch) / 2 + i * pitch, cy, length) for i in range(n)]


def ebox_vent_area_mm2(p: dict) -> float:
    sw, sl = p["ebox"]["vent_slot"]
    walls = sum(sw * (length - sw) + math.pi * (sw / 2) ** 2 for *_, length in _ebox_wall_slots(p))
    lid = sum(sw * (length - sw) + math.pi * (sw / 2) ** 2 for *_, length in _ebox_lid_slots(p))
    return walls + lid


def electronics_box(p: dict) -> cq.Workplane:
    e, j, el = p["ebox"], p["jetson"], p["electronics"]
    m3, m25, m2 = screw(p, "m3"), screw(p, j["screw"]), screw(p, el["perfboard_screw"])
    lay = ebox_layout(p)
    ix, iy, iz = lay["inner"]
    ox, oy, oz = lay["outer"]
    w, fl = e["wall"], e["floor"]

    part = box(ox, oy, oz).cut(box(ix, iy, iz + 1, at=(0, 0, fl)))
    # corner bosses with heat-set inserts for the lid
    bos = [(sx * (ix / 2 - 3), sy * (iy / 2 - 3)) for sx in (-1, 1) for sy in (-1, 1)]
    part = part.union(cylinders_z(bos, 8, fl - 0.1, iz + 0.1))
    part = part.cut(cylinders_z(bos, m3["insert"], oz - m3["insert_depth"] - 1, m3["insert_depth"] + 2))

    # Jetson standoffs (M2.5 heat-set or pilot) and perfboard standoffs (M2 pilot)
    jx, jy = j["footprint"]
    hx, hy = j["hole_spacing"]
    jcx, jcy = lay["jetson_c"]
    jpts = [(jcx + sx * hx / 2, jcy + sy * hy / 2) for sx in (-1, 1) for sy in (-1, 1)]
    part = part.union(cylinders_z(jpts, 6, fl - 0.1, j["standoff"] + 0.1))
    part = part.cut(cylinders_z(jpts, m25["pilot"], fl, j["standoff"] + 1))
    px, py = lay["perf_size"]
    pcx, pcy = lay["perf_c"]
    ins = el["perfboard_holes_inset"]
    ppts = [(pcx + sx * (px / 2 - ins), pcy + sy * (py / 2 - ins)) for sx in (-1, 1) for sy in (-1, 1)]
    part = part.union(cylinders_z(ppts, 5, fl - 0.1, el["standoff"] + 0.1))
    part = part.cut(cylinders_z(ppts, m2["pilot"], fl, el["standoff"] + 1))

    # Jetson I/O window on the x_min wall, cable port for battery / motors / sensors on the x_max wall
    pw, ph = e["port_window"]
    z_io = fl + j["standoff"] + 1.6
    part = part.cut(box(w * 3, pw, ph, at=(-ox / 2, jcy, z_io)))
    cw, ch = e["cable_port"]
    part = part.cut(box(w * 3, cw, ch, at=(ox / 2, pcy, fl + 4)))

    # vent slots on the long walls (intake low and high along the whole wall)
    sw, _ = e["vent_slot"]
    for wall, along, zc, length in _ebox_wall_slots(p):
        y = -oy / 2 if wall == "y_min" else oy / 2
        tool = (cq.Workplane("XZ").workplane(offset=-y - w * 1.5).center(along, zc)
                .slot2D(length, sw, angle=90).extrude(w * 3))
        part = part.cut(tool)

    # mounting ears with M3 clearance holes
    ear = 12
    for sx in (-1, 1):
        for sy in (-1, 1):
            ex, ey = sx * (ox / 2 + ear / 2 - 1), sy * (oy / 2 - ear)
            part = part.union(box(ear + 2, ear, 3, at=(ex - sx, ey, 0)))
            part = part.cut(cylinders_z([(ex + sx * 1, ey)], m3["clear"], -1, 5))
    return one_solid(part)


def electronics_box_lid(p: dict) -> cq.Workplane:
    e, m3 = p["ebox"], screw(p, "m3")
    lay = ebox_layout(p)
    ix, iy, _ = lay["inner"]
    ox, oy, _ = lay["outer"]
    t = e["lid_thickness"]
    cl = p["material"]["clearance_slide"]
    part = box(ox, oy, t)
    # locating lip inside the walls, notched round the corner bosses
    lip = box(ix - 2 * cl, iy - 2 * cl, 3, at=(0, 0, t)).cut(box(ix - 2 * cl - 4, iy - 2 * cl - 4, 4, at=(0, 0, t)))
    bos = [(sx * (ix / 2 - 3), sy * (iy / 2 - 3)) for sx in (-1, 1) for sy in (-1, 1)]
    lip = lip.cut(cylinders_z(bos, 9, t - 1, 6))
    part = part.union(lip)
    part = part.cut(cylinders_z(bos, m3["clear"], -1, t + 6))
    sw = e["vent_slot"][0]
    for x, y, length in _ebox_lid_slots(p):
        part = part.cut(slot_z(x, y, sw, length, -1, t + 2, 90))
    return one_solid(part)


# ------------------------------------------------------------------------------------------------------------
# ME-11 battery tray, low at the rear, holder pulls out for charging
# ------------------------------------------------------------------------------------------------------------
def battery_tray(p: dict) -> cq.Workplane:
    b, m3 = p["battery"], screw(p, "m3")
    cl = p["material"]["clearance_slide"]
    hx, hy, _ = b["holder"]
    w, fl, lip = b["tray_wall"], b["tray_floor"], b["lip_height"]
    ix, iy = hx + 2 * cl, hy + 2 * cl
    bl, bw, bt = b["bms"]
    pocket_y = bw + 2 * cl
    ox, oy = ix + 2 * w, iy + 2 * w + pocket_y + w
    y_hold = -oy / 2 + w + iy / 2
    y_bms = oy / 2 - w - pocket_y / 2

    part = box(ox, oy, fl + lip)
    part = part.cut(box(ix, iy, lip + 1, at=(0, y_hold, fl)))
    part = part.cut(box(bl + 2 * cl, pocket_y, lip + 1, at=(0, y_bms, fl)))
    # wire pass from the BMS pocket to the holder, and the battery lead exit
    part = part.cut(box(10, w + 2, lip, at=(-ix / 2 + 12, y_bms - pocket_y / 2 - w / 2, fl + 4)))
    part = part.cut(box(w * 3, 10, lip, at=(ox / 2, y_bms, fl + 3)))
    # finger notch in the x_min wall to lift the holder out
    part = part.cut(cq.Workplane("YZ").workplane(offset=-ox / 2 - 1).center(y_hold, fl + lip)
                    .circle(b["finger_notch"] / 2).extrude(w + 2))
    # Velcro strap slots through the floor, across the holder
    sl, sw = b["strap_slot"]
    for x in (-ix / 4, ix / 4):
        part = part.cut(box(sw, sl, fl + 2, at=(x, -oy / 2 + w + sw / 2 + 1, -1)))
        part = part.cut(box(sw, sl, fl + 2, at=(x, y_hold + iy / 2 - sw / 2 - 1, -1)))
    # 4 × M3 floor holes to the chassis
    pts = [(sx * (ix / 2 - 8), y_hold + sy * (iy / 2 - 18)) for sx in (-1, 1) for sy in (-1, 1)]
    part = part.cut(cylinders_z(pts, m3["clear"], -1, fl + 2))
    return one_solid(part)


# ------------------------------------------------------------------------------------------------------------
# ME-12 E-Stop holder at the rear top
# ------------------------------------------------------------------------------------------------------------
def estop_holder(p: dict) -> cq.Workplane:
    """Printed upside-down (panel on the bed): z = 0 is the top panel, the open side goes on the cover."""
    s, m3 = p["estop"], screw(p, "m3")
    bw, bd, bh = s["body_below_panel"]
    w, pt, fl = s["wall"], s["panel_thickness"], s["base_flange"]
    ox, oy = bw + 2 * w, bd + 2 * w
    oz = pt + bh
    part = box(ox, oy, oz).cut(box(bw, bd, bh + 1, at=(0, 0, pt)))
    flange = box(ox + 2 * fl, oy + 2 * fl, 3, at=(0, 0, oz - 3)).cut(box(bw, bd, 5, at=(0, 0, oz - 4)))
    part = part.union(flange)
    pts = [(sx * (ox / 2 + fl / 2), sy * (oy / 2 + fl / 2)) for sx in (-1, 1) for sy in (-1, 1)]
    part = part.cut(cylinders_z(pts, m3["clear"], oz - 4, 6))
    # 22 mm bushing hole with the anti-rotation key
    part = part.cut(cylinders_z([(0, 0)], s["hole"], -1, pt + 2))
    part = part.cut(box(s["key_width"], s["hole"] / 2 + s["key_depth"], pt + 2,
                        at=(0, (s["hole"] / 2 + s["key_depth"]) / 2, -1)))
    # wire exit in a side wall, near the open (cover) side
    part = part.cut(cq.Workplane("YZ").workplane(offset=ox / 2 - w - 1).center(0, oz - 10).circle(4)
                    .extrude(w + 2))
    return one_solid(part)


# ------------------------------------------------------------------------------------------------------------
# ME-07 AS5600 encoder: magnet cap on the drive axle + sensor bracket with gap-adjust slots
# ------------------------------------------------------------------------------------------------------------
def as5600_magnet_cap(p: dict) -> cq.Workplane:
    """Axle along z: z = 0 open end slides over the axle, magnet pocket flush at the top face."""
    a, f = p["as5600"], p["forklift"]
    cl = p["material"]["clearance_press"]
    md, mt = a["magnet"]
    ad = f["drive_axle_diameter"]
    od = max(ad, md) + 2 * a["cap_wall"]
    length = a["cap_depth_on_axle"] + a["cap_web"] + mt
    part = cq.Workplane("XY").circle(od / 2).extrude(length)
    bore = cq.Workplane("XY").workplane(offset=-1).circle((ad + cl) / 2).extrude(a["cap_depth_on_axle"] + 1)
    if f["drive_axle_flat"] > 0:  # D-flat: leave material where the flat is
        flat = box(ad + 2, ad, a["cap_depth_on_axle"] + 2, at=(0, ad / 2 + (ad + cl) / 2 - f["drive_axle_flat"], -1))
        bore = bore.cut(flat)
    part = part.cut(bore)
    part = part.cut(cq.Workplane("XY").workplane(offset=length - mt).circle((md + cl) / 2).extrude(mt + 1))
    return one_solid(part)


def as5600_plate_x(p: dict) -> float:
    """x of the bracket plate's magnet-side face (axle end at x = 0) for the nominal gap."""
    a = p["as5600"]
    magnet_face = a["cap_web"] + a["magnet"][1]
    return magnet_face + a["gap"] + a["chip_height"] + a["board"][2]


def as5600_bracket(p: dict) -> cq.Workplane:
    """Axle along +x at z = axle_height above the chassis face the foot screws to; foot slots along x let the
    gap be set to anywhere in gap_min … gap_max."""
    a, m2, m3 = p["as5600"], screw(p, "m2"), screw(p, "m3")
    bw, bh, _ = a["board"]
    zc = a["axle_height_above_mount_face"]
    pt = a["plate_thickness"]
    xp = as5600_plate_x(p)
    width = bw + 4
    foot = box(a["foot_length"] + pt, width, pt, at=(xp + pt - (a["foot_length"] + pt) / 2, 0, 0))
    plate = box(pt, width, zc + bh / 2 + 2, at=(xp + pt / 2, 0, 0))
    part = foot.union(plate)
    # gusset
    part = part.union(cq.Workplane("XZ").polyline([(xp - 8, pt), (xp, pt), (xp, pt + 8)]).close()
                      .extrude(1.5, both=True))
    for y, z in a["board_holes"]:
        part = part.cut(cq.Workplane("YZ").workplane(offset=xp - 1).center(y, zc + z).circle(m2["pilot"] / 2)
                        .extrude(pt + 2))
    for y in (-width / 2 + 5, width / 2 - 5):
        part = part.cut(slot_z(xp - a["foot_length"] / 2 - 2, y, m3["clear"], m3["clear"] + a["slot_travel"], -1, pt + 2))
    return one_solid(part)


def as5600_gap_mm(p: dict, cap: cq.Workplane, bracket: cq.Workplane) -> float:
    """Magnet → chip gap from the built parts: cap length past the axle end vs. the plate face position."""
    a = p["as5600"]
    cap_len = cap.val().BoundingBox().zlen
    magnet_face = cap_len - a["cap_depth_on_axle"]
    plate_face = None
    for face in bracket.faces().vals():
        n = face.normalAt()
        if face.geomType() == "PLANE" and n.x < -0.99 and face.Area() > a["board"][0] * a["board"][1] * 0.5:
            plate_face = face.Center().x
    if plate_face is None:
        raise ValueError("bracket plate face not found")
    return plate_face - a["board"][2] - a["chip_height"] - magnet_face


# ------------------------------------------------------------------------------------------------------------
# ME-08 slide pot + limit switches on the mast, and the carriage link
# ------------------------------------------------------------------------------------------------------------
def mast_sensor_bracket(p: dict) -> cq.Workplane:
    """Plate along the mast (x = up the mast), printed flat. Limit switches at x = 0 (bottom) and
    x = lift_max (top) so the carriage trigger tab hits them at the ends of the lift range; slide pot beside
    them; two cheek clamps on the back hug the mast upright with an M3 cross bolt."""
    ms, f = p["mast_sensors"], p["forklift"]
    m2, m3 = screw(p, "m2"), screw(p, ms["clamp_screw"])
    cl = p["material"]["clearance_slide"]
    t = ms["plate_thickness"]
    pl, pw, _ = ms["pot_body"]
    sl, sw, _ = ms["limit_switch"]
    lift = f["lift_max"]
    length = max(lift + sl, pl) + 16
    width = pw + sw + 3 * 4
    x0 = -length / 2 + 8 + sl / 2          # bottom switch centre
    y_sw = -width / 2 + 4 + sw / 2
    y_pot = width / 2 - 4 - pw / 2

    part = box(length, width, t)
    for xs in (x0, x0 + lift):
        s = ms["limit_holes_spacing"] / 2
        part = part.cut(cylinders_z([(xs - s, y_sw), (xs + s, y_sw)], m2["pilot"], -1, t + 2))
    xpc = x0 + lift / 2
    part = part.cut(cylinders_z([(xpc + hx, y_pot + hy) for hx, hy in ms["pot_holes"]], m2["pilot"], -1, t + 2))
    # wire slot under the pot pins
    part = part.cut(slot_z(xpc - pl / 2 + 4, y_pot, 4, 10, -1, t + 2, 90))

    rw, rd = f["mast_rail_width"] + 2 * cl, f["mast_rail_depth"]
    cheek = 3.0
    for xc in (-length / 2 + 6, length / 2 - 6):
        for sy in (-1, 1):
            part = part.union(box(10, cheek, rd + 6, at=(xc, sy * (rw / 2 + cheek / 2), t - 0.01)))
        bolt = (cq.Workplane("XZ").workplane(offset=-(rw / 2 + cheek + 1)).center(xc, t + rd + 3)
                .circle(m3["clear"] / 2).extrude(rw + 2 * cheek + 2))
        part = part.cut(bolt)
    return one_solid(part)


def mast_slider_link(p: dict) -> cq.Workplane:
    """Bolts to the fork carriage (M3), grips the pot knob and carries the tab that presses the limit
    switches."""
    ms, m3 = p["mast_sensors"], screw(p, "m3")
    cl = p["material"]["clearance_press"]
    kw, kh = ms["pot_knob"]
    length, width, t = 40, 12, 4
    part = box(length, width, t)
    part = part.cut(box(kw + cl, kh + cl, t + 2, at=(length / 2 - 8, 0, -1)))
    part = part.cut(cylinders_z([(-length / 2 + 7, 0)], m3["clear"], -1, t + 2))
    part = part.union(box(6, width, 10, at=(-2, 0, t - 0.01)))  # limit-switch trigger tab
    return one_solid(part)


def pot_covers_lift(p: dict) -> bool:
    return p["mast_sensors"]["pot_travel"] >= p["forklift"]["lift_max"]


# ------------------------------------------------------------------------------------------------------------
# ME-09 TCRT5000 nose clip on each fork tip, sensor facing up
# ------------------------------------------------------------------------------------------------------------
def tcrt_clip_size(p: dict) -> tuple[float, float]:
    """(outer width, outer thickness) of the sleeve around the fork."""
    t, f = p["tcrt"], p["forklift"]
    cl = p["material"]["clearance_slide"]
    side = p["material"]["min_wall"]
    return f["fork_width"] + 2 * cl + 2 * side, f["fork_thickness"] + 2 * cl + 2 * t["sleeve_wall"]


def tcrt_clip(p: dict, side: str = "left") -> cq.Workplane:
    """Fork along x, fork tip at x = 0. The sleeve slides over the last `sleeve_length` of the fork; the nose
    in front holds the bare TCRT5000 head in a pocket open to the top; its leads leave through the inner side
    (towards the other fork) so they run along the fork without touching the pallet."""
    t, f = p["tcrt"], p["forklift"]
    cl = p["material"]["clearance_slide"]
    ow, ot = tcrt_clip_size(p)
    sl, nl = t["sleeve_length"], t["nose_length"]
    part = box(sl + nl, ow, ot, at=((nl - sl) / 2, 0, 0))
    part = part.cut(box(sl + 1, f["fork_width"] + 2 * cl, f["fork_thickness"] + 2 * cl,
                        at=(-sl / 2 - 0.5, 0, t["sleeve_wall"])))
    # lead-in chamfer on the nose top front edge
    part = part.edges(">X and >Z").chamfer(min(4.0, ot / 3))
    hl, hw, hh = t["head"]
    pc = p["material"]["clearance_board"] / 2
    xh = nl / 2
    part = part.cut(box(hl + pc, hw + pc, hh + 0.4, at=(xh - 1, 0, ot - hh - 0.4)))
    cw, ch = t["wire_channel"]
    sign = -1 if side == "left" else 1  # left fork (+y) exits towards -y, the inner side; right the opposite
    part = part.cut(box(cw, ow / 2 + 1, ch, at=(xh - 1, sign * (ow / 4 + 0.5), ot - hh - 0.4)))
    return one_solid(part)


def pallet_opening(p: dict) -> float:
    pal = p["pallet"]
    return pal["height"] - 2 * pal["deck_thickness"]


# ------------------------------------------------------------------------------------------------------------
# registry: exported file name → builder
# ------------------------------------------------------------------------------------------------------------
TOF_NAMES = ["front", "front_left", "front_right", "left", "right", "rear"]

PARTS: dict[str, Callable[[dict], cq.Workplane]] = {
    "lidar_mount": lidar_mount,
    "camera_mount": camera_mount,
    **{f"tof_bracket_{n}": (lambda p, n=n: tof_bracket(p, n)) for n in TOF_NAMES},
    "electronics_box": electronics_box,
    "electronics_box_lid": electronics_box_lid,
    "battery_tray": battery_tray,
    "estop_holder": estop_holder,
    "as5600_magnet_cap": as5600_magnet_cap,
    "as5600_bracket": as5600_bracket,
    "mast_sensor_bracket": mast_sensor_bracket,
    "mast_slider_link": mast_slider_link,
    "tcrt_clip_left": lambda p: tcrt_clip(p, "left"),
    "tcrt_clip_right": lambda p: tcrt_clip(p, "right"),
}

# how many of each to print per forklift (README / build summary)
QUANTITY = {name: 1 for name in PARTS}
