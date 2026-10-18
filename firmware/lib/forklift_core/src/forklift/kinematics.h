// FW-01 / FW-02: rear-wheel-steered Ackermann kinematics, encoder speed and odometry.
//
// Reference point = centre of the FRONT (driven, non-steered) axle; the steered axle is behind it at a
// distance L (wheelbase). With the steering angle δ positive = rear wheels turned to the left:
//
//     ω = −v · tan δ / L          δ = atan(−ω · L / v)
//
// so to turn left (ω > 0) while driving forward the rear wheels turn right (δ < 0). The relation holds
// for v < 0 too (reversing with the same δ turns the other way). The AS5600 sits on the drive axle in
// front of the differential, so it measures the speed of the reference point directly.
#pragma once

#include <cstdint>

#include "forklift/config.h"
#include "forklift/types.h"

namespace fl {

struct DriveCommand {
  float wheel_speed_mps = 0.0f;  // speed of the front axle centre
  float steer_rad = 0.0f;        // rear wheel angle, + = left
  bool steer_saturated = false;
};

class RearAckermann {
 public:
  RearAckermann(float wheelbase_m = cfg::WHEELBASE_M, float max_steer_rad = cfg::MAX_STEER_RAD,
                float min_speed_mps = cfg::CMD_EPS_MPS)
      : L_(wheelbase_m), max_steer_(max_steer_rad), min_speed_(min_speed_mps) {}

  // cmd_vel (v, ω) → wheel speed + steering angle. When |v| is too small to define a curvature the
  // previous steering angle is held (no servo twitch while standing still).
  DriveCommand toDrive(float v, float w);

  // Inverse: what yaw rate does (v, δ) produce.
  static float yawRate(float v, float steer_rad, float wheelbase_m);
  // Turning radius of the reference point for a steering angle (infinite → returns 0 for straight).
  static float turningRadius(float steer_rad, float wheelbase_m);

  float wheelbase() const { return L_; }
  float lastSteer() const { return last_steer_; }

 private:
  float L_;
  float max_steer_;
  float min_speed_;
  float last_steer_ = 0.0f;
};

// AS5600 12-bit angle → unwrapped distance and speed over a short sliding window.
class EncoderSpeed {
 public:
  EncoderSpeed(float wheel_radius_m, float direction = 1.0f, float gear_ratio = cfg::DRIVE_GEAR_RATIO,
               uint16_t counts_per_rev = cfg::AS5600_COUNTS_PER_REV);

  void setWheelRadius(float r) { radius_ = r; }
  void setDirection(float d) { dir_ = d < 0 ? -1.0f : 1.0f; }

  // raw = 0 … counts_per_rev-1, now_us = monotonic microseconds.
  void update(uint16_t raw, uint32_t now_us);
  void reset();

  float speed() const { return speed_; }         // m/s, signed
  double distance() const { return distance_; }  // m since reset

 private:
  static constexpr int kWindow = cfg::ENCODER_SPEED_WINDOW;
  float radius_, dir_, gear_;
  uint16_t cpr_;
  bool have_last_ = false;
  uint16_t last_raw_ = 0;
  int64_t counts_ = 0;
  double distance_ = 0.0;
  float speed_ = 0.0f;
  int64_t hist_counts_[kWindow + 1] = {};
  uint32_t hist_us_[kWindow + 1] = {};
  int head_ = 0;
  int filled_ = 0;
};

// FW-02: dead-reckoning from the measured wheel speed and the commanded steering angle.
class Odometry {
 public:
  explicit Odometry(float wheelbase_m = cfg::WHEELBASE_M) : L_(wheelbase_m) {}

  void reset(const Pose2D& p = Pose2D{});
  void update(float v, float steer_rad, float dt);

  const Pose2D& pose() const { return pose_; }
  float v() const { return v_; }
  float w() const { return w_; }
  double travelled() const { return travelled_; }

 private:
  float L_;
  Pose2D pose_;
  float v_ = 0.0f;
  float w_ = 0.0f;
  double travelled_ = 0.0;
};

double wrapAngle(double a);

}  // namespace fl
