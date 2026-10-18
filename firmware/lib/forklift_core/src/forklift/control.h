// FW-03 / FW-04: PID with feed-forward and anti-windup, and the fork-lift position controller.
#pragma once

#include "forklift/config.h"

namespace fl {

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float kff = 0.0f;  // output += kff · setpoint
};

class Pid {
 public:
  Pid() = default;
  Pid(const PidGains& g, float out_min, float out_max, float d_filter = 0.2f)
      : g_(g), out_min_(out_min), out_max_(out_max), d_filter_(d_filter) {}

  void setGains(const PidGains& g) { g_ = g; }
  const PidGains& gains() const { return g_; }
  void setLimits(float out_min, float out_max) {
    out_min_ = out_min;
    out_max_ = out_max;
  }
  void reset();

  // Derivative acts on the measurement (no kick on setpoint steps) through a first-order filter.
  // Anti-windup: conditional integration — the integrator only grows while the output is not saturated
  // in the direction of the error, and it is clamped to the output range.
  float update(float setpoint, float measurement, float dt);

  float integrator() const { return integ_; }
  bool saturated() const { return saturated_; }

 private:
  PidGains g_;
  float out_min_ = -1.0f;
  float out_max_ = 1.0f;
  float d_filter_ = 0.2f;
  float integ_ = 0.0f;
  float d_state_ = 0.0f;
  float last_meas_ = 0.0f;
  bool have_last_ = false;
  bool saturated_ = false;
};

// Drive-speed loop: setpoint m/s → motor duty −1 … 1.
inline PidGains defaultDriveGains() {
  return PidGains{6.0f, 80.0f, 0.0f, 1.0f / cfg::DRIVE_SPEED_AT_FULL_DUTY};  // PLACEHOLDER until T-03
}

// Lift loop: setpoint mm → duty, per mm of error.
inline PidGains defaultLiftGains() { return PidGains{0.06f, 0.02f, 0.0f, 0.0f}; }  // PLACEHOLDER (T-04)

struct LiftConfig {
  float max_mm = cfg::LIFT_MAX_MM;
  float stop_band_mm = cfg::LIFT_STOP_BAND_MM;
  float restart_band_mm = cfg::LIFT_RESTART_BAND_MM;
  float min_duty = cfg::LIFT_MIN_DUTY;
};

// FW-04: position control of the fork carriage from the slide pot. A hysteresis band switches the
// motor off on target (no hunting), the deadband of the geared motor is compensated, and a pressed
// limit switch blocks motion further in that direction immediately.
class LiftController {
 public:
  explicit LiftController(const PidGains& g = defaultLiftGains(), const LiftConfig& c = LiftConfig{});

  void setGains(const PidGains& g) { pid_.setGains(g); }
  void reset();
  float update(float target_mm, float height_mm, bool top_limit, bool bottom_limit, float dt);

  bool atTarget() const { return at_target_; }
  bool blockedByLimit() const { return blocked_; }

 private:
  LiftConfig c_;
  Pid pid_;
  bool at_target_ = false;
  bool blocked_ = false;
  float last_target_ = -1.0f;
};

}  // namespace fl
