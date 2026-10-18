"""Lift action logic (FW-04: reach the height within ± 3 mm, stop at a limit switch) — no ROS.

The ESP32 runs the lift PID; the Jetson only sends the set-point (`lift_cmd`, mm) and watches `fork_state`.
`LiftTracker.update()` is called with every fork_state sample and says when the goal is reached, impossible
(limit switch hit before the target) or timed out.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LiftResult:
    done: bool
    success: bool = False
    message: str = ""


class LiftTracker:
    def __init__(self, target_mm: float, now: float, tolerance_mm: float = 3.0, timeout_s: float = 8.0,
                 settle_samples: int = 3, lift_max_mm: float = 120.0) -> None:
        self.requested = float(target_mm)
        self.target = min(max(float(target_mm), 0.0), float(lift_max_mm))
        self.tol = float(tolerance_mm)
        self.deadline = now + float(timeout_s)
        self.settle = max(1, int(settle_samples))
        self._in_band = 0
        self.last_height: float | None = None

    def update(self, height_mm: float, limit_low: bool, limit_high: bool, now: float) -> LiftResult:
        self.last_height = float(height_mm)
        err = self.target - self.last_height
        if abs(err) <= self.tol:
            self._in_band += 1
            if self._in_band >= self.settle:
                return LiftResult(True, True, f"reached {self.last_height:.1f} mm")
        else:
            self._in_band = 0
            if err > 0 and limit_high:
                return LiftResult(True, False, f"upper limit at {self.last_height:.1f} mm (target {self.target:.0f})")
            if err < 0 and limit_low and self.target > self.tol:
                return LiftResult(True, False, f"lower limit at {self.last_height:.1f} mm (target {self.target:.0f})")
            if err < 0 and limit_low:
                return LiftResult(True, True, "at the lower limit")
        if now > self.deadline:
            return LiftResult(True, False, f"timeout at {self.last_height:.1f} mm (target {self.target:.0f})")
        return LiftResult(False)
