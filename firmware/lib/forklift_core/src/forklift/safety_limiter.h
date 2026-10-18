// Safety limiter building blocks (requirements §9): ToF ring rule, speed caps, battery thresholds.
// Stateless functions + a small battery hysteresis object; the state machine combines them.
#pragma once

#include <cstdint>

#include "forklift/config.h"
#include "forklift/types.h"

namespace fl {

struct SafetyConfig {
  float max_speed_mps = cfg::MAX_SPEED_MPS;
  float tof_slow_m = cfg::TOF_SLOW_M;
  float tof_stop_m = cfg::TOF_STOP_M;
  float tof_slow_factor = cfg::TOF_SLOW_FACTOR;
  uint32_t tof_stale_ms = cfg::TOF_STALE_MS;
  bool tof_fault_stops = cfg::TOF_FAULT_STOPS;
  uint8_t mask_front = cfg::TOF_MASK_FRONT;
  uint8_t mask_rear = cfg::TOF_MASK_REAR;
  uint8_t mask_sides = cfg::TOF_MASK_SIDES;
  uint8_t mask_docking_ignore = cfg::TOF_MASK_DOCKING_IGNORE;
  uint32_t watchdog_ms = cfg::WATCHDOG_MS;
  uint32_t safe_recovery_hold_ms = cfg::SAFE_RECOVERY_HOLD_MS;
  float fork_raised_mm = cfg::FORK_RAISED_MM;
  float raised_fork_max_mps = cfg::RAISED_FORK_MAX_MPS;
  float docking_max_mps = cfg::DOCKING_MAX_MPS;
  float battery_low_pct = cfg::BATTERY_LOW_PCT;
  float battery_critical_pct = cfg::BATTERY_CRITICAL_PCT;
  float battery_hyst_pct = cfg::BATTERY_HYST_PCT;
  float accel_mps2 = cfg::ACCEL_MPS2;
  float decel_mps2 = cfg::DECEL_MPS2;
  float safe_decel_mps2 = cfg::SAFE_DECEL_MPS2;
  float cmd_eps_mps = cfg::CMD_EPS_MPS;
  float turn_eps_radps = cfg::TURN_EPS_RADPS;
};

// Direction of a speed: +1 forward (forks first), -1 reverse, 0 standing (|v| <= eps).
int directionOf(float v, float eps);

struct TofEvaluation {
  bool slow = false;
  bool stop = false;
  bool fault = false;          // a leading sensor is stale or reports an error
  uint8_t considered_mask = 0; // sensors that were looked at this tick
  uint8_t slow_mask = 0;
  uint8_t stop_mask = 0;
  uint8_t fault_mask = 0;
};

// SF-02 / SF-03. Only the sensors facing the direction of motion can slow or stop the robot:
//   forward → front + both front corners, reverse → rear. While turning the two side sensors may
//   slow (never stop) the robot, because the rear-steered tail swings out. Docking ignores the
//   sensors in mask_docking_ignore (they see the pallet itself).
TofEvaluation evaluateTof(int direction, float cmd_w, const ToFReading tof[cfg::TOF_COUNT],
                          uint32_t now_ms, bool docking, const SafetyConfig& c);

struct SpeedCap {
  float cap_mps = 0.0f;
  SafetyReason reason = SafetyReason::NONE;  // the tightest active cap (NONE = full speed)
};

// Combines SF-02 slow (30 %), SF-08 raised forks and SF-03 docking caps. The smallest cap wins.
SpeedCap speedCap(bool tof_slow, float fork_height_mm, bool fork_height_valid, bool docking,
                  const SafetyConfig& c);

// 2S Li-ion open-circuit curve → 0…100 %. Piecewise linear, good enough for 15 % / 8 % thresholds.
float batteryPercentFromVoltage(float volts);

// SF-09 with hysteresis so a sagging voltage under load does not chatter.
class BatteryMonitor {
 public:
  explicit BatteryMonitor(const SafetyConfig& c = SafetyConfig{}) : c_(c) {}
  void update(float percent, bool valid);
  bool low() const { return low_; }
  bool critical() const { return critical_; }

 private:
  SafetyConfig c_;
  bool low_ = false;
  bool critical_ = false;
};

// Moves `current` toward `target` with separate accel / decel limits (m/s²).
float rampSpeed(float current, float target, float accel, float decel, float dt);

}  // namespace fl
