// Host tests of FW-01 (rear-steer Ackermann) and FW-02 (encoder speed + odometry, T-05 on the host).
#include <unity.h>

#include <cmath>
#include <cstdint>

#include "forklift/kinematics.h"

using namespace fl;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr float L = 0.20f;
}  // namespace

void setUp() {}
void tearDown() {}

// ---- Ackermann ----

void test_straight_has_zero_steer() {
  RearAckermann a(L);
  DriveCommand d = a.toDrive(0.2f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, d.steer_rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, d.wheel_speed_mps);
}

void test_left_turn_forward_steers_rear_wheels_right() {
  RearAckermann a(L);
  DriveCommand d = a.toDrive(0.2f, 0.5f);  // ω > 0 = turn left (REP-103)
  TEST_ASSERT_TRUE(d.steer_rad < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, std::atan(-0.5f * L / 0.2f), d.steer_rad);
  DriveCommand r = a.toDrive(0.2f, -0.5f);
  TEST_ASSERT_TRUE(r.steer_rad > 0.0f);
}

void test_reversing_flips_steer_sign() {
  RearAckermann a(L);
  DriveCommand d = a.toDrive(-0.2f, 0.5f);
  TEST_ASSERT_TRUE(d.steer_rad > 0.0f);
}

void test_inverse_gives_back_the_yaw_rate() {
  RearAckermann a(L);
  const float vs[] = {0.3f, 0.1f, -0.1f, -0.25f};
  const float ws[] = {0.0f, 0.2f, -0.25f, 0.1f, -0.05f};  // inside ±30° at v ≥ 0.1
  for (float v : vs) {
    for (float w : ws) {
      DriveCommand d = a.toDrive(v, w);
      TEST_ASSERT_FALSE(d.steer_saturated);
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, w, RearAckermann::yawRate(v, d.steer_rad, L));
    }
  }
}

void test_formula_omega_equals_minus_v_tan_delta_over_L() {
  const float v = 0.25f, delta = 0.3f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -v * std::tan(delta) / L, RearAckermann::yawRate(v, delta, L));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, L / std::tan(delta), RearAckermann::turningRadius(delta, L));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, RearAckermann::turningRadius(0.0f, L));
}

void test_steer_saturates_at_max() {
  RearAckermann a(L, 0.5236f);
  DriveCommand d = a.toDrive(0.05f, 3.0f);
  TEST_ASSERT_TRUE(d.steer_saturated);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5236f, d.steer_rad);
}

void test_standing_still_holds_last_steer() {
  RearAckermann a(L);
  DriveCommand d1 = a.toDrive(0.2f, 0.4f);
  DriveCommand d2 = a.toDrive(0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, d1.steer_rad, d2.steer_rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, d2.wheel_speed_mps);
}

// ---- Encoder ----

void test_encoder_speed_and_wrap() {
  const float radius = 0.03f;
  EncoderSpeed enc(radius);
  const double v = 0.2;  // m/s
  const double counts_per_m = 4096.0 / (2.0 * kPi * radius);
  double pos_counts = 4000.0;  // start just before the wrap
  for (int ms = 0; ms <= 2000; ++ms) {
    enc.update(static_cast<uint16_t>(static_cast<int64_t>(std::floor(pos_counts)) % 4096),
               static_cast<uint32_t>(ms * 1000));
    pos_counts += v * counts_per_m * 1e-3;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.2f, enc.speed());
  TEST_ASSERT_FLOAT_WITHIN(0.002, 0.4, enc.distance());

  EncoderSpeed back(radius, -1.0f);  // mirrored mounting
  pos_counts = 50.0;
  for (int ms = 0; ms <= 1000; ++ms) {
    back.update(static_cast<uint16_t>(static_cast<int64_t>(std::floor(pos_counts) + 4096) % 4096),
                static_cast<uint32_t>(ms * 1000));
    pos_counts -= 0.1 * counts_per_m * 1e-3;  // magnet turns backwards
  }
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.1f, back.speed());  // direction −1 → forward again
}

// ---- Odometry ----

void test_odometry_straight_2m() {
  Odometry o(L);
  for (int i = 0; i < 10000; ++i) o.update(0.2f, 0.0f, 0.001f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0, o.pose().x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, o.pose().y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, o.pose().theta);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0, o.travelled());
}

void test_odometry_full_circle() {
  const float steer = -0.3f;  // rear wheels right → turn left
  const float v = 0.2f;
  const double R = L / std::tan(0.3);
  const double period = 2.0 * kPi * R / v;
  const int steps = static_cast<int>(std::lround(period / 0.001));
  Odometry o(L);
  double max_y = 0.0;
  for (int i = 0; i < steps; ++i) {
    o.update(v, steer, 0.001f);
    if (o.pose().y > max_y) max_y = o.pose().y;
  }
  TEST_ASSERT_TRUE(o.w() > 0.0f);                      // left = counter-clockwise
  TEST_ASSERT_FLOAT_WITHIN(2e-3, 2.0 * R, max_y);      // diameter
  TEST_ASSERT_FLOAT_WITHIN(2e-3, 0.0, o.pose().x);     // back at the start
  TEST_ASSERT_FLOAT_WITHIN(2e-3, 0.0, o.pose().y);
  TEST_ASSERT_FLOAT_WITHIN(5e-3, 0.0, o.pose().theta);           // heading back to 0
}

void test_odometry_reverse_arc_mirrors() {
  Odometry fwd(L), rev(L);
  for (int i = 0; i < 3000; ++i) {
    fwd.update(0.2f, -0.2f, 0.001f);
    rev.update(-0.2f, -0.2f, 0.001f);
  }
  // Reversing with the same steering retraces the forward arc backwards: x mirrored, y the same.
  TEST_ASSERT_FLOAT_WITHIN(1e-6, -fwd.pose().x, rev.pose().x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, fwd.pose().y, rev.pose().y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, -fwd.pose().theta, rev.pose().theta);
}

void test_encoder_to_odometry_2m_within_t05() {
  // T-05 on the host: 2 m straight with the quantised encoder in the loop → error ≤ 3 %.
  const float radius = 0.03f;
  EncoderSpeed enc(radius);
  Odometry o(L);
  const double counts_per_m = 4096.0 / (2.0 * kPi * radius);
  double pos = 0.0;
  for (int ms = 0; ms < 10000; ++ms) {
    pos += 0.2 * 1e-3;
    enc.update(static_cast<uint16_t>(static_cast<int64_t>(std::floor(pos * counts_per_m)) % 4096),
               static_cast<uint32_t>(ms * 1000));
    o.update(enc.speed(), 0.0f, 0.001f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.03 * 2.0, 2.0, o.pose().x);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, o.pose().x);  // in practice much better
}

void test_wrap_angle() {
  TEST_ASSERT_FLOAT_WITHIN(1e-9, -kPi + 0.1, wrapAngle(kPi + 0.1));
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.5, wrapAngle(0.5 + 4.0 * kPi));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_straight_has_zero_steer);
  RUN_TEST(test_left_turn_forward_steers_rear_wheels_right);
  RUN_TEST(test_reversing_flips_steer_sign);
  RUN_TEST(test_inverse_gives_back_the_yaw_rate);
  RUN_TEST(test_formula_omega_equals_minus_v_tan_delta_over_L);
  RUN_TEST(test_steer_saturates_at_max);
  RUN_TEST(test_standing_still_holds_last_steer);
  RUN_TEST(test_encoder_speed_and_wrap);
  RUN_TEST(test_odometry_straight_2m);
  RUN_TEST(test_odometry_full_circle);
  RUN_TEST(test_odometry_reverse_arc_mirrors);
  RUN_TEST(test_encoder_to_odometry_2m_within_t05);
  RUN_TEST(test_wrap_angle);
  return UNITY_END();
}
