#include "forklift/kinematics.h"

#include <cmath>

namespace fl {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double wrapAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

DriveCommand RearAckermann::toDrive(float v, float w) {
  DriveCommand d;
  d.wheel_speed_mps = v;
  if (std::fabs(v) < min_speed_) {
    d.steer_rad = last_steer_;
    return d;
  }
  float steer = std::atan(-w * L_ / v);
  if (steer > max_steer_) {
    steer = max_steer_;
    d.steer_saturated = true;
  } else if (steer < -max_steer_) {
    steer = -max_steer_;
    d.steer_saturated = true;
  }
  d.steer_rad = steer;
  last_steer_ = steer;
  return d;
}

float RearAckermann::yawRate(float v, float steer_rad, float wheelbase_m) {
  return -v * std::tan(steer_rad) / wheelbase_m;
}

float RearAckermann::turningRadius(float steer_rad, float wheelbase_m) {
  const float t = std::tan(steer_rad);
  if (std::fabs(t) < 1e-6f) return 0.0f;
  return wheelbase_m / std::fabs(t);
}

EncoderSpeed::EncoderSpeed(float wheel_radius_m, float direction, float gear_ratio,
                           uint16_t counts_per_rev)
    : radius_(wheel_radius_m), dir_(direction < 0 ? -1.0f : 1.0f), gear_(gear_ratio),
      cpr_(counts_per_rev) {}

void EncoderSpeed::reset() {
  have_last_ = false;
  counts_ = 0;
  distance_ = 0.0;
  speed_ = 0.0f;
  head_ = 0;
  filled_ = 0;
}

void EncoderSpeed::update(uint16_t raw, uint32_t now_us) {
  raw = static_cast<uint16_t>(raw % cpr_);
  if (!have_last_) {
    have_last_ = true;
    last_raw_ = raw;
  }
  int delta = static_cast<int>(raw) - static_cast<int>(last_raw_);
  const int half = cpr_ / 2;
  if (delta > half) delta -= cpr_;  // wrapped backwards past 0
  else if (delta < -half) delta += cpr_;
  last_raw_ = raw;
  counts_ += delta;

  const double metres_per_count = 2.0 * kPi * radius_ / (static_cast<double>(cpr_) * gear_);
  distance_ = static_cast<double>(counts_) * metres_per_count * dir_;

  constexpr int N = kWindow + 1;
  hist_counts_[head_] = counts_;
  hist_us_[head_] = now_us;
  head_ = (head_ + 1) % N;
  if (filled_ < N) ++filled_;
  if (filled_ < 2) {
    speed_ = 0.0f;
    return;
  }
  const int newest = (head_ + N - 1) % N;
  const int oldest = (head_ + N - filled_) % N;
  const uint32_t dt_us = hist_us_[newest] - hist_us_[oldest];
  if (dt_us == 0) return;
  const int64_t dc = hist_counts_[newest] - hist_counts_[oldest];
  speed_ = static_cast<float>(static_cast<double>(dc) * metres_per_count * dir_ / (dt_us * 1e-6));
}

void Odometry::reset(const Pose2D& p) {
  pose_ = p;
  v_ = w_ = 0.0f;
  travelled_ = 0.0;
}

void Odometry::update(float v, float steer_rad, float dt) {
  v_ = v;
  w_ = RearAckermann::yawRate(v, steer_rad, L_);
  // Midpoint (second-order) integration: exact for straight lines, very small error on arcs at 1 kHz.
  const double th_mid = pose_.theta + 0.5 * w_ * dt;
  pose_.x += v * std::cos(th_mid) * dt;
  pose_.y += v * std::sin(th_mid) * dt;
  pose_.theta = wrapAngle(pose_.theta + static_cast<double>(w_) * dt);
  travelled_ += std::fabs(v) * dt;
}

}  // namespace fl
