#include "forklift/safety_limiter.h"

#include <cmath>

namespace fl {

int directionOf(float v, float eps) {
  if (v > eps) return 1;
  if (v < -eps) return -1;
  return 0;
}

TofEvaluation evaluateTof(int direction, float cmd_w, const ToFReading tof[cfg::TOF_COUNT],
                          uint32_t now_ms, bool docking, const SafetyConfig& c) {
  TofEvaluation e;
  if (direction == 0) return e;

  uint8_t leading = direction > 0 ? c.mask_front : c.mask_rear;
  if (docking) leading &= static_cast<uint8_t>(~c.mask_docking_ignore);
  const uint8_t sides = std::fabs(cmd_w) > c.turn_eps_radps ? c.mask_sides : 0;
  e.considered_mask = static_cast<uint8_t>(leading | sides);

  for (int i = 0; i < cfg::TOF_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1u << i);
    if (!(e.considered_mask & bit)) continue;
    const ToFReading& r = tof[i];
    const bool fresh = r.valid && (now_ms - r.stamp_ms) <= c.tof_stale_ms;
    const bool is_leading = (leading & bit) != 0;

    if (!fresh) {
      if (is_leading) {
        e.fault = true;
        e.fault_mask |= bit;
      } else {
        e.slow = true;  // blind side sensor while turning: be careful, do not block
        e.slow_mask |= bit;
      }
      continue;
    }
    if (is_leading && r.distance_m < c.tof_stop_m) {
      e.stop = true;
      e.stop_mask |= bit;
    } else if (r.distance_m < c.tof_slow_m) {
      e.slow = true;
      e.slow_mask |= bit;
    }
  }

  if (e.fault) {
    if (c.tof_fault_stops) e.stop = true;
    else e.slow = true;
  }
  return e;
}

SpeedCap speedCap(bool tof_slow, float fork_height_mm, bool fork_height_valid, bool docking,
                  const SafetyConfig& c) {
  SpeedCap cap{c.max_speed_mps, SafetyReason::NONE};
  auto apply = [&cap](float limit, SafetyReason why) {
    if (limit < cap.cap_mps) {
      cap.cap_mps = limit;
      cap.reason = why;
    }
  };
  if (tof_slow) apply(c.max_speed_mps * c.tof_slow_factor, SafetyReason::TOF_SLOW);
  // An unknown fork height is treated as raised (fail-safe).
  if (!fork_height_valid || fork_height_mm > c.fork_raised_mm)
    apply(c.raised_fork_max_mps, SafetyReason::FORK_RAISED);
  if (docking) apply(c.docking_max_mps, SafetyReason::DOCKING);
  return cap;
}

float batteryPercentFromVoltage(float v) {
  // 2S pack voltage → state of charge (typical Li-ion curve, 2 × single cell).
  static const float kV[] = {6.00f, 6.60f, 7.00f, 7.20f, 7.40f, 7.60f, 7.80f, 8.00f, 8.20f, 8.40f};
  static const float kP[] = {0.0f, 5.0f, 10.0f, 20.0f, 35.0f, 50.0f, 65.0f, 80.0f, 92.0f, 100.0f};
  const int n = sizeof(kV) / sizeof(kV[0]);
  if (v <= kV[0]) return 0.0f;
  if (v >= kV[n - 1]) return 100.0f;
  for (int i = 1; i < n; ++i) {
    if (v <= kV[i]) {
      const float t = (v - kV[i - 1]) / (kV[i] - kV[i - 1]);
      return kP[i - 1] + t * (kP[i] - kP[i - 1]);
    }
  }
  return 100.0f;
}

void BatteryMonitor::update(float percent, bool valid) {
  if (!valid) return;  // keep the last decision until a reading arrives
  if (critical_) {
    if (percent > c_.battery_critical_pct + c_.battery_hyst_pct) critical_ = false;
  } else if (percent < c_.battery_critical_pct) {
    critical_ = true;
  }
  if (low_) {
    if (percent > c_.battery_low_pct + c_.battery_hyst_pct) low_ = false;
  } else if (percent < c_.battery_low_pct) {
    low_ = true;
  }
  if (critical_) low_ = true;
}

float rampSpeed(float current, float target, float accel, float decel, float dt) {
  const bool slowing = (target * current < 0.0f) || std::fabs(target) < std::fabs(current);
  const float step = (slowing ? decel : accel) * dt;
  if (target > current) return (target - current > step) ? current + step : target;
  return (current - target > step) ? current - step : target;
}

}  // namespace fl
