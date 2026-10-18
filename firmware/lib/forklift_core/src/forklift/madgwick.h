// Madgwick orientation filter (IMU / 6-DoF form) for the MPU9250 at 100 Hz.
//
// The magnetometer is deliberately not fused: indoors, a few centimetres from three DC motors and a
// 7 Ah battery, it mostly measures the robot itself. Yaw therefore comes from the gyro alone, which is why
// the gyro bias is calibrated at rest and stored in NVS (FW-05); long-term yaw is corrected on the Jetson
// by robot_localization with LiDAR / AMCL. T-02 asks yaw drift ≤ 2°/min.
#pragma once

namespace fl {

struct Quaternion {
  float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

class Madgwick {
 public:
  explicit Madgwick(float beta = 0.05f) : beta_(beta) {}

  void reset() { q_ = Quaternion{}; }
  void setBeta(float b) { beta_ = b; }

  // gyro in rad/s, accel in any unit (normalised internally), dt in seconds.
  void updateImu(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  const Quaternion& q() const { return q_; }
  float roll() const;
  float pitch() const;
  float yaw() const;

 private:
  float beta_;
  Quaternion q_;
};

}  // namespace fl
