// Host tests of the Madgwick filter and the FW-05 calibration blob.
#include <unity.h>

#include <cmath>
#include <cstring>

#include "forklift/calibration.h"
#include "forklift/madgwick.h"

using namespace fl;

void setUp() {}
void tearDown() {}

// ---- Madgwick ----

void test_madgwick_converges_to_static_tilt() {
  Madgwick f(0.1f);
  const float roll = 0.3f, pitch = -0.2f, g = 9.81f;
  // Gravity seen by a body rolled by φ and pitched by θ.
  const float ax = -std::sin(pitch) * g;
  const float ay = std::sin(roll) * std::cos(pitch) * g;
  const float az = std::cos(roll) * std::cos(pitch) * g;
  for (int i = 0; i < 3000; ++i) f.updateImu(0, 0, 0, ax, ay, az, 0.01f);  // 30 s at 100 Hz
  TEST_ASSERT_FLOAT_WITHIN(0.02f, roll, f.roll());
  TEST_ASSERT_FLOAT_WITHIN(0.02f, pitch, f.pitch());
}

void test_madgwick_integrates_yaw_rate() {
  Madgwick f(0.05f);
  for (int i = 0; i < 200; ++i) f.updateImu(0, 0, 0.5f, 0, 0, 9.81f, 0.01f);  // 2 s at 0.5 rad/s
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, f.yaw());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, f.roll());
}

void test_madgwick_quaternion_stays_unit() {
  Madgwick f(0.05f);
  for (int i = 0; i < 1000; ++i) f.updateImu(0.3f, -0.2f, 0.7f, 0.5f, 0.1f, 9.7f, 0.01f);
  const Quaternion& q = f.q();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z));
}

// ---- Calibration ----

void test_defaults_are_valid_and_sealed() {
  Calibration c = defaultCalibration();
  TEST_ASSERT_NULL(validateCalibration(c));
  TEST_ASSERT_TRUE(isSealed(c));
}

void test_roundtrip_through_blob() {
  Calibration c = defaultCalibration();
  c.servo_center_us = 1530.0f;
  c.gyro_bias[2] = 0.012f;
  c.wheel_radius_m = 0.0295f;
  sealCalibration(c);
  unsigned char blob[sizeof(Calibration)];
  std::memcpy(blob, &c, sizeof c);
  bool defaults = true;
  Calibration back = loadCalibration(blob, sizeof blob, &defaults);
  TEST_ASSERT_FALSE(defaults);
  TEST_ASSERT_EQUAL_FLOAT(1530.0f, back.servo_center_us);
  TEST_ASSERT_EQUAL_FLOAT(0.012f, back.gyro_bias[2]);
}

void test_corrupt_blob_falls_back_to_defaults() {
  Calibration c = defaultCalibration();
  c.servo_center_us = 1530.0f;
  sealCalibration(c);
  unsigned char blob[sizeof(Calibration)];
  std::memcpy(blob, &c, sizeof c);
  blob[20] ^= 0x40;  // one flipped bit
  bool defaults = false;
  Calibration back = loadCalibration(blob, sizeof blob, &defaults);
  TEST_ASSERT_TRUE(defaults);
  TEST_ASSERT_EQUAL_FLOAT(1500.0f, back.servo_center_us);
  // Wrong size (older firmware layout) and missing blob too.
  TEST_ASSERT_TRUE((loadCalibration(blob, sizeof blob - 4, &defaults), defaults));
  TEST_ASSERT_TRUE((loadCalibration(nullptr, 0, &defaults), defaults));
}

void test_sealed_but_invalid_values_are_rejected() {
  Calibration c = defaultCalibration();
  c.wheel_radius_m = 0.0f;
  sealCalibration(c);
  TEST_ASSERT_NOT_NULL(validateCalibration(c));
  bool defaults = false;
  loadCalibration(&c, sizeof c, &defaults);
  TEST_ASSERT_TRUE(defaults);
  c = defaultCalibration();
  c.drive.kp = NAN;
  TEST_ASSERT_NOT_NULL(validateCalibration(c));
}

void test_pot_and_servo_conversions() {
  Calibration c = defaultCalibration();
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, potToMm(c, c.pot_raw_bottom));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, c.pot_top_mm, potToMm(c, c.pot_raw_top));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 42.0f, potToMm(c, mmToPot(c, 42.0f)));
  TEST_ASSERT_EQUAL_FLOAT(1500.0f, steerToServoUs(c, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(c.servo_max_us, steerToServoUs(c, 5.0f));
  TEST_ASSERT_EQUAL_FLOAT(c.servo_min_us, steerToServoUs(c, -5.0f));
}

void test_crc32_known_value() {
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32("123456789", 9));  // standard check value
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_madgwick_converges_to_static_tilt);
  RUN_TEST(test_madgwick_integrates_yaw_rate);
  RUN_TEST(test_madgwick_quaternion_stays_unit);
  RUN_TEST(test_defaults_are_valid_and_sealed);
  RUN_TEST(test_roundtrip_through_blob);
  RUN_TEST(test_corrupt_blob_falls_back_to_defaults);
  RUN_TEST(test_sealed_but_invalid_values_are_rejected);
  RUN_TEST(test_pot_and_servo_conversions);
  RUN_TEST(test_crc32_known_value);
  return UNITY_END();
}
