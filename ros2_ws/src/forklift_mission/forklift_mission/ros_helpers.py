"""Small pure helpers shared by the ROS wrappers of forklift_mission (host-testable, no rclpy)."""

from __future__ import annotations

import math

SAFETY_NAMES = ("normal", "slow", "stop", "safe", "estop")  # forklift_interfaces/SafetyState constants 0..4


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def safety_name(state: int) -> str:
    return SAFETY_NAMES[state] if 0 <= int(state) < len(SAFETY_NAMES) else "stop"


def battery_percent(percentage: float, voltage: float, v_empty: float = 6.4, v_full: float = 8.4) -> float:
    """sensor_msgs/BatteryState -> 0..100. Uses `percentage` (0..1) when the ESP32 fills it, else a linear 2S
    voltage estimate (6.4 V empty .. 8.4 V full)."""
    if percentage is not None and math.isfinite(percentage) and 0.0 <= percentage <= 1.0:
        return 100.0 * percentage
    if voltage is None or not math.isfinite(voltage) or voltage <= 0:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (voltage - v_empty) / (v_full - v_empty)))


def safety_event(prev: int, state: int, reason: str) -> tuple[str, str, str] | None:
    """(severity, code, message) for a supervisor transition worth an MQTT `event` (SF-13), else None.
    Reported when the state gets worse with a reason, and when it clears back to normal from stop or worse."""
    if state > prev and state >= 1 and reason:
        severity = "info" if state == 1 else ("critical" if state == 4 else "warning")
        return severity, reason, f"safety {safety_name(prev)} -> {safety_name(state)}: {reason}"
    return None


def lift_teleop_target(current_target_mm: float, lift_dir: int, rate_mm_s: float, dt: float,
                       lift_max_mm: float) -> float:
    """Integrate the gamepad lift button (-1/0/+1) into a lift height set-point."""
    return max(0.0, min(lift_max_mm, current_target_mm + lift_dir * rate_mm_s * dt))
