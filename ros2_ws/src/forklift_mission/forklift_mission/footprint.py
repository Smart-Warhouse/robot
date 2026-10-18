"""Nav2 footprint polygon of the forklift, with and without a pallet on the forks (NV-12) — no ROS.

base_link = centre of the front (drive) axle; x forward. The polygon covers the chassis, the forks and (loaded)
the carried pallet, plus `padding` on every side. mission_node publishes it on
`local_costmap/footprint` and `global_costmap/footprint` when the load changes.
"""

from __future__ import annotations


def footprint(dims: dict, loaded: bool = False, padding: float = 0.0) -> list[tuple[float, float]]:
    rear = -float(dims["body_length"]) - padding
    front_body = float(dims["body_front"])
    half_w = float(dims["body_width"]) / 2 + padding
    carriage = float(dims["carriage_x"])
    if loaded:
        tip = carriage + float(dims["pallet_depth"]) + padding
        half_f = float(dims["pallet_width"]) / 2 + padding
    else:
        tip = carriage + float(dims["fork_length"]) + padding
        half_f = (float(dims["fork_spacing"]) + float(dims["fork_width"])) / 2 + padding
    x_step = max(front_body, carriage)
    pts = [(rear, -half_w), (x_step, -half_w), (x_step, -half_f), (tip, -half_f),
           (tip, half_f), (x_step, half_f), (x_step, half_w), (rear, half_w)]
    return [(round(x, 4), round(y, 4)) for x, y in pts]


def footprint_string(points: list[tuple[float, float]]) -> str:
    """Nav2 `footprint` parameter format: "[[x, y], [x, y], ...]"."""
    return "[" + ", ".join(f"[{x:g}, {y:g}]" for x, y in points) + "]"


def parse_footprint_string(s: str) -> list[tuple[float, float]]:
    import ast

    return [(float(x), float(y)) for x, y in ast.literal_eval(s)]


def half_width(points: list[tuple[float, float]]) -> float:
    return max(abs(y) for _, y in points)
