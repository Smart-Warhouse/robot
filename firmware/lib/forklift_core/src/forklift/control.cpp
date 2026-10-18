#include "forklift/control.h"

#include <cmath>

namespace fl {

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void Pid::reset() {
  integ_ = 0.0f;
  d_state_ = 0.0f;
  have_last_ = false;
  saturated_ = false;
}

float Pid::update(float setpoint, float measurement, float dt) {
  if (dt <= 0.0f) dt = 1e-3f;
  const float error = setpoint - measurement;

  float d = 0.0f;
  if (g_.kd != 0.0f && have_last_) {
    const float raw = -(measurement - last_meas_) / dt;
    d_state_ += d_filter_ * (raw - d_state_);
    d = g_.kd * d_state_;
  }
  last_meas_ = measurement;
  have_last_ = true;

  const float ff = g_.kff * setpoint;
  const float p = g_.kp * error;
  const float candidate = integ_ + g_.ki * error * dt;
  const float u_try = ff + p + candidate + d;
  const bool pushing_high = u_try > out_max_ && error > 0.0f;
  const bool pushing_low = u_try < out_min_ && error < 0.0f;
  if (!pushing_high && !pushing_low) integ_ = clampf(candidate, out_min_, out_max_);

  const float u = ff + p + integ_ + d;
  saturated_ = u > out_max_ || u < out_min_;
  return clampf(u, out_min_, out_max_);
}

LiftController::LiftController(const PidGains& g, const LiftConfig& c)
    : c_(c), pid_(g, -1.0f, 1.0f) {}

void LiftController::reset() {
  pid_.reset();
  at_target_ = false;
  blocked_ = false;
}

float LiftController::update(float target_mm, float height_mm, bool top_limit, bool bottom_limit,
                             float dt) {
  target_mm = clampf(target_mm, 0.0f, c_.max_mm);
  if (std::fabs(target_mm - last_target_) > 1e-3f) {
    at_target_ = false;  // new goal: regulate even if we are inside the band of the old one
    pid_.reset();
    last_target_ = target_mm;
  }
  const float err = target_mm - height_mm;
  const float band = at_target_ ? c_.restart_band_mm : c_.stop_band_mm;
  if (std::fabs(err) <= band) {
    at_target_ = true;
    blocked_ = false;
    pid_.reset();
    return 0.0f;
  }
  at_target_ = false;

  float u = pid_.update(target_mm, height_mm, dt);
  // Deadband compensation: map (0, 1] onto (min_duty, 1].
  if (u > 0.0f) u = c_.min_duty + (1.0f - c_.min_duty) * u;
  else if (u < 0.0f) u = -c_.min_duty + (1.0f - c_.min_duty) * u;

  blocked_ = (u > 0.0f && top_limit) || (u < 0.0f && bottom_limit);
  if (blocked_) {
    pid_.reset();
    return 0.0f;
  }
  return clampf(u, -1.0f, 1.0f);
}

}  // namespace fl
