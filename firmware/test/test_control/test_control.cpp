// Host tests of FW-03 (speed PID, T-03 criteria) and FW-04 (lift ± 3 mm, T-04 criteria) on plant models.
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "forklift/config.h"
#include "forklift/control.h"
#include "forklift/kinematics.h"

using namespace fl;

namespace {

constexpr float DT = 0.001f;  // motor_pid_task runs at 1 kHz

// Deterministic pseudo-random numbers (LCG) so the tests are repeatable.
struct Lcg {
  uint32_t s;
  explicit Lcg(uint32_t seed) : s(seed) {}
  float uniform() {  // [0, 1)
    s = s * 1664525u + 1013904223u;
    return static_cast<float>(s >> 8) / 16777216.0f;
  }
};

// First-order drive: v' = (K·u − v) / τ. τ = 0.08 s is the actuator lag used by the RL simulator.
struct DrivePlant {
  float K, tau, v = 0.0f;
  DrivePlant(float k, float t) : K(k), tau(t) {}
  void step(float u, float dt) { v += (K * u - v) / tau * dt; }
};

struct StepResult {
  float settle_s;          // last time |e| > 5 % of the step
  float steady_error_pct;  // mean |e| over the last 200 ms, % of the step
  float overshoot_pct;
};

// Speed step 0 → target with the measurement taken through the quantised AS5600 estimator.
StepResult speedStep(float K, float tau, float target, const PidGains& g, bool use_encoder = true) {
  DrivePlant plant(K, tau);
  Pid pid(g, -1.0f, 1.0f);
  EncoderSpeed enc(0.03f);
  const double counts_per_m = 4096.0 / (2.0 * 3.14159265358979 * 0.03);
  double pos = 0.0;
  float u_prev = 0.0f;  // one-sample actuation delay
  StepResult r{0.0f, 0.0f, 0.0f};
  const int n = 1500;
  double err_sum = 0.0;
  int err_n = 0;
  for (int i = 0; i < n; ++i) {
    pos += plant.v * DT;
    const int64_t c = static_cast<int64_t>(std::floor(pos * counts_per_m));
    enc.update(static_cast<uint16_t>(((c % 4096) + 4096) % 4096), static_cast<uint32_t>(i * 1000));
    const float meas = use_encoder ? enc.speed() : plant.v;
    const float u = pid.update(target, meas, DT);
    plant.step(u_prev, DT);
    u_prev = u;
    const float e = std::fabs(plant.v - target);
    if (e > 0.05f * std::fabs(target)) r.settle_s = (i + 1) * DT;
    const float os = (plant.v - target) / target * 100.0f;
    if (os > r.overshoot_pct) r.overshoot_pct = os;
    if (i >= n - 200) {
      err_sum += e;
      ++err_n;
    }
  }
  r.steady_error_pct = static_cast<float>(err_sum / err_n / std::fabs(target) * 100.0);
  return r;
}

// Fork carriage: geared DC motor with deadband and lag, slide pot with noise, EMA as in the firmware.
struct LiftPlant {
  float h = 0.0f;       // true height mm
  float vel = 0.0f;     // mm/s
  float rate = 50.0f;   // mm/s at full duty (lift_rate in the RL config)
  float deadband = 0.15f;
  float tau = 0.05f;
  float max_mm = cfg::LIFT_MAX_MM;
  void step(float u, float dt) {
    float eff = 0.0f;
    if (std::fabs(u) > deadband) eff = (std::fabs(u) - deadband) / (1.0f - deadband) * (u > 0 ? 1 : -1);
    vel += (rate * eff - vel) / tau * dt;
    h += vel * dt;
    if (h < 0.0f) { h = 0.0f; vel = 0.0f; }
    if (h > max_mm) { h = max_mm; vel = 0.0f; }
  }
  bool top() const { return h >= max_mm - 0.5f; }
  bool bottom() const { return h <= 0.5f; }
};

}  // namespace

void setUp() {}
void tearDown() {}

// ---- PID unit behaviour ----

void test_pid_proportional_and_clamp() {
  Pid p(PidGains{2.0f, 0.0f, 0.0f, 0.0f}, -1.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, p.update(0.2f, 0.0f, DT));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, p.update(5.0f, 0.0f, DT));
  TEST_ASSERT_TRUE(p.saturated());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, p.update(-5.0f, 0.0f, DT));
}

void test_pid_anti_windup_limits_integrator() {
  Pid p(PidGains{1.0f, 50.0f, 0.0f, 0.0f}, -1.0f, 1.0f);
  for (int i = 0; i < 5000; ++i) p.update(10.0f, 0.0f, DT);  // unreachable setpoint for 5 s
  TEST_ASSERT_TRUE(p.integrator() <= 1.0f);
  // As soon as the error flips sign the output leaves saturation quickly (no 5 s of wind-up to undo).
  int ms = 0;
  while (p.update(0.0f, 0.5f, DT) > 0.0f && ms < 1000) ++ms;
  TEST_ASSERT_TRUE(ms < 50);
}

void test_pid_derivative_has_no_setpoint_kick() {
  Pid p(PidGains{0.0f, 0.0f, 1.0f, 0.0f}, -10.0f, 10.0f, 1.0f);
  p.update(0.0f, 0.0f, DT);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p.update(1.0f, 0.0f, DT));  // setpoint step, meas. flat
  TEST_ASSERT_TRUE(p.update(1.0f, 0.01f, DT) < 0.0f);              // meas. rising → damping
}

// ---- FW-03 / T-03: speed step on a first-order plant ----

void test_speed_pid_meets_fw03_nominal() {
  StepResult r = speedStep(cfg::DRIVE_SPEED_AT_FULL_DUTY, 0.08f, 0.2f, defaultDriveGains());
  TEST_ASSERT_TRUE_MESSAGE(r.settle_s <= 0.300f, "settling time > 300 ms");
  TEST_ASSERT_TRUE_MESSAGE(r.steady_error_pct <= 5.0f, "steady-state error > 5 %");
}

void test_speed_pid_robust_to_plant_mismatch() {
  // Real motor unknown until ME-02: ±25 % gain and a slower lag must still pass.
  const float Ks[] = {0.75f * cfg::DRIVE_SPEED_AT_FULL_DUTY, 1.25f * cfg::DRIVE_SPEED_AT_FULL_DUTY};
  const float taus[] = {0.05f, 0.08f, 0.12f};
  const float targets[] = {0.1f, 0.2f, 0.3f, -0.2f};
  for (float K : Ks)
    for (float tau : taus)
      for (float t : targets) {
        StepResult r = speedStep(K, tau, t, defaultDriveGains());
        char msg[128];
        snprintf(msg, sizeof msg, "K=%.3f tau=%.2f target=%.2f: settle %.0f ms, error %.2f %%, overshoot %.1f %%",
                 K, tau, t, r.settle_s * 1000.0f, r.steady_error_pct, r.overshoot_pct);
        TEST_ASSERT_TRUE_MESSAGE(r.settle_s <= 0.300f, msg);
        TEST_ASSERT_TRUE_MESSAGE(r.steady_error_pct <= 5.0f, msg);
      }
}

void test_speed_pid_slow_targets() {
  StepResult r = speedStep(cfg::DRIVE_SPEED_AT_FULL_DUTY, 0.08f, 0.09f, defaultDriveGains());
  TEST_ASSERT_TRUE(r.settle_s <= 0.300f);
  TEST_ASSERT_TRUE(r.steady_error_pct <= 5.0f);
}

// ---- FW-04 / T-04: lift to 20 random heights, ± 3 mm ----

void test_lift_20_random_targets_within_3mm() {
  LiftController lift;
  LiftPlant plant;
  Lcg rng(12345);
  float filtered = 0.0f;
  for (int k = 0; k < 20; ++k) {
    const float target = 5.0f + rng.uniform() * (cfg::LIFT_MAX_MM - 10.0f);
    int reached_ms = -1;
    for (int ms = 0; ms < 4000; ++ms) {
      const float noisy = plant.h + (rng.uniform() - 0.5f) * 0.6f;  // ± 0.3 mm pot noise
      filtered += cfg::LIFT_HEIGHT_FILTER * (noisy - filtered);
      const float u = lift.update(target, filtered, plant.top(), plant.bottom(), DT);
      plant.step(u, DT);
      if (reached_ms < 0 && lift.atTarget()) reached_ms = ms;
    }
    char msg[96];
    snprintf(msg, sizeof msg, "target %.1f mm: true height %.2f mm", target, plant.h);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(3.0f, target, plant.h, msg);
    TEST_ASSERT_TRUE_MESSAGE(reached_ms >= 0 && reached_ms < 3500, msg);
  }
}

void test_lift_stops_on_limit_switch() {
  LiftController lift;
  // Asked to go up while the top limit is pressed → motor off at once (FW-04).
  TEST_ASSERT_EQUAL_FLOAT(0.0f, lift.update(120.0f, 60.0f, true, false, DT));
  TEST_ASSERT_TRUE(lift.blockedByLimit());
  // Going down is still allowed with the top limit pressed.
  TEST_ASSERT_TRUE(lift.update(10.0f, 60.0f, true, false, DT) < 0.0f);
  // Bottom limit blocks going down.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, lift.update(0.0f, 20.0f, false, true, DT));
}

void test_lift_target_clamped_to_range() {
  LiftController lift;
  LiftPlant plant;
  for (int ms = 0; ms < 5000; ++ms) plant.step(lift.update(500.0f, plant.h, plant.top(), plant.bottom(), DT), DT);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, cfg::LIFT_MAX_MM, plant.h);
}

void test_lift_holds_without_hunting() {
  LiftController lift;
  LiftPlant plant;
  plant.h = 50.0f;
  int nonzero = 0;
  for (int ms = 0; ms < 2000; ++ms) {
    const float u = lift.update(50.5f, plant.h, false, false, DT);
    if (u != 0.0f) ++nonzero;
    plant.step(u, DT);
  }
  TEST_ASSERT_EQUAL(0, nonzero);  // inside the band: motor stays off
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pid_proportional_and_clamp);
  RUN_TEST(test_pid_anti_windup_limits_integrator);
  RUN_TEST(test_pid_derivative_has_no_setpoint_kick);
  RUN_TEST(test_speed_pid_meets_fw03_nominal);
  RUN_TEST(test_speed_pid_robust_to_plant_mismatch);
  RUN_TEST(test_speed_pid_slow_targets);
  RUN_TEST(test_lift_20_random_targets_within_3mm);
  RUN_TEST(test_lift_stops_on_limit_switch);
  RUN_TEST(test_lift_target_clamped_to_range);
  RUN_TEST(test_lift_holds_without_hunting);
  return UNITY_END();
}
