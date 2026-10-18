"""Decode the ESP32 `fork_state` std_msgs/Float32MultiArray (robot/firmware/src/microros_node.cpp) — no ROS.

    data = [height_mm (NaN = pot unreadable), limit_top, limit_bottom, tcrt_left, tcrt_right, at_target]
Flags are 0.0 / 1.0. Temporary until the firmware publishes forklift_interfaces/ForkState (see ForkState.msg).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

FIELDS = ("height_mm", "limit_top", "limit_bottom", "tcrt_left", "tcrt_right", "at_target")


@dataclass(frozen=True)
class ForkReading:
    height_mm: float
    limit_high: bool
    limit_low: bool
    tcrt_left: bool
    tcrt_right: bool
    at_target: bool

    @property
    def height_known(self) -> bool:
        return math.isfinite(self.height_mm)


def parse_fork_array(data) -> ForkReading | None:
    """None if the array is too short (wrong publisher / firmware mismatch)."""
    vals = [float(v) for v in data]
    if len(vals) < len(FIELDS):
        return None
    flag = lambda v: math.isfinite(v) and v > 0.5  # noqa: E731
    h = vals[0] if math.isfinite(vals[0]) else math.nan
    return ForkReading(h, flag(vals[1]), flag(vals[2]), flag(vals[3]), flag(vals[4]), flag(vals[5]))
