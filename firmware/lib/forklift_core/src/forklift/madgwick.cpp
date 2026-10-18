#include "forklift/madgwick.h"

#include <cmath>

namespace fl {

void Madgwick::updateImu(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

  // Rate of change of the quaternion from the gyroscope.
  float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qd1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qd2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qd3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  const float an = std::sqrt(ax * ax + ay * ay + az * az);
  if (an > 1e-6f) {
    ax /= an;
    ay /= an;
    az /= an;

    const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    const float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    const float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    const float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    // Gradient-descent corrective step (objective: measured gravity vs. predicted gravity).
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 +
               _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 +
               _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    const float sn = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f) {
      s0 /= sn;
      s1 /= sn;
      s2 /= sn;
      s3 /= sn;
      qd0 -= beta_ * s0;
      qd1 -= beta_ * s1;
      qd2 -= beta_ * s2;
      qd3 -= beta_ * s3;
    }
  }

  q0 += qd0 * dt;
  q1 += qd1 * dt;
  q2 += qd2 * dt;
  q3 += qd3 * dt;
  const float n = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 1e-9f) {
    q_.w = q0 / n;
    q_.x = q1 / n;
    q_.y = q2 / n;
    q_.z = q3 / n;
  }
}

float Madgwick::roll() const {
  return std::atan2(2.0f * (q_.w * q_.x + q_.y * q_.z), 1.0f - 2.0f * (q_.x * q_.x + q_.y * q_.y));
}

float Madgwick::pitch() const {
  float s = 2.0f * (q_.w * q_.y - q_.z * q_.x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return std::asin(s);
}

float Madgwick::yaw() const {
  return std::atan2(2.0f * (q_.w * q_.z + q_.x * q_.y), 1.0f - 2.0f * (q_.y * q_.y + q_.z * q_.z));
}

}  // namespace fl
