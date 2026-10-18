// Host tests of the FW-08 state machine and the §9 safety limiter (SF-02, SF-03, SF-04, SF-08, SF-09).
#include <unity.h>

#include <cmath>

#include "forklift/safety_state_machine.h"

using namespace fl;

namespace {

constexpr float DT = 0.001f;

// A rig that ticks the state machine at 1 kHz like safety_task does.
struct Rig {
  SafetyStateMachine sm;
  SafetyInputs in;
  SafetyOutput out;
  bool link = true;       // Jetson sends heartbeat every tick
  bool send_cmd = true;   // Jetson sends cmd_vel every tick
  int frozen_tof = -1;    // this sensor stops producing new readings

  Rig() {
    in.boot_complete = true;
    in.fork_height_mm = 0.0f;
    in.fork_height_valid = true;
    in.battery_pct = 80.0f;
    in.battery_valid = true;
    for (auto& t : in.tof) t = ToFReading{4.0f, true, 0};
  }

  void cmd(float v, float w = 0.0f) {
    in.cmd_v = v;
    in.cmd_w = w;
  }

  void tick() {
    if (link) {
      in.heartbeat_seen = true;
      in.last_heartbeat_ms = in.now_ms;
    }
    if (send_cmd && link) {
      in.cmd_seen = true;
      in.last_cmd_ms = in.now_ms;
    }
    for (int i = 0; i < cfg::TOF_COUNT; ++i)  // ToF task keeps them fresh
      if (in.tof[i].valid && i != frozen_tof) in.tof[i].stamp_ms = in.now_ms;
    out = sm.update(in, DT);
    in.reset_requested = false;
    in.now_ms += 1;
  }

  void run(int ms) {
    for (int i = 0; i < ms; ++i) tick();
  }

  void tof(int i, float d) { in.tof[i].distance_m = d; }
};

}  // namespace

void setUp() {}
void tearDown() {}

// ---- BOOT / IDLE / ACTIVE ----

void test_boot_waits_for_drivers() {
  Rig r;
  r.in.boot_complete = false;
  r.cmd(0.2f);
  r.run(100);
  TEST_ASSERT_EQUAL(SafetyState::BOOT, r.out.state);
  TEST_ASSERT_FALSE(r.out.drive_enabled);
  TEST_ASSERT_FALSE(r.out.lift_enabled);
  r.in.boot_complete = true;
  r.cmd(0.0f);
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
}

void test_without_jetson_stays_idle_and_ignores_cmd() {  // FW-09
  Rig r;
  r.link = false;
  r.in.cmd_seen = true;
  r.cmd(0.2f);
  r.run(2000);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::NO_LINK, r.out.reason);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  TEST_ASSERT_FALSE(r.out.drive_enabled);
}

void test_idle_to_active_with_accel_ramp() {
  Rig r;
  r.run(10);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
  r.cmd(0.3f);
  r.run(100);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.06f, r.out.v);  // 0.6 m/s² × 0.1 s
  r.run(1000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, r.out.v);
  r.cmd(0.0f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
}

void test_speed_never_exceeds_max() {
  Rig r;
  r.cmd(2.0f);
  r.run(2000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, cfg::MAX_SPEED_MPS, r.out.v);
}

// ---- ESTOP (FW-08) ----

void test_estop_latches_and_needs_explicit_reset() {
  Rig r;
  r.cmd(0.3f);
  r.run(800);
  r.in.estop_pressed = true;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::ESTOP, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::ESTOP_PRESSED, r.out.reason);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  TEST_ASSERT_FALSE(r.out.drive_enabled);
  TEST_ASSERT_FALSE(r.out.lift_enabled);

  // Reset while still pressed is ignored.
  r.in.reset_requested = true;
  r.run(10);
  TEST_ASSERT_EQUAL(SafetyState::ESTOP, r.out.state);

  // Released: still latched, link and cmd_vel are alive, but no reset → stays in ESTOP.
  r.in.estop_pressed = false;
  r.run(5000);
  TEST_ASSERT_EQUAL(SafetyState::ESTOP, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::ESTOP_WAIT_RESET, r.out.reason);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);

  // Explicit reset → leaves ESTOP; the robot ramps up again from 0.
  r.cmd(0.0f);
  r.in.reset_requested = true;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
  r.cmd(0.3f);
  r.run(50);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_TRUE(r.out.v < 0.05f);
}

void test_estop_overrides_everything() {
  Rig r;
  r.in.boot_complete = false;
  r.in.estop_pressed = true;
  r.run(5);
  TEST_ASSERT_EQUAL(SafetyState::ESTOP, r.out.state);
  // Reset during boot returns to BOOT, not IDLE.
  r.in.estop_pressed = false;
  r.in.reset_requested = true;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::BOOT, r.out.state);
}

// ---- Watchdog (FW-06, SF-04, T-07) ----

void test_heartbeat_loss_goes_safe_and_stops_within_600ms() {
  Rig r;
  r.cmd(0.3f);
  r.run(1000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, r.out.v);

  r.link = false;  // USB unplugged: no heartbeat, no cmd_vel
  int stopped_at = -1;
  int safe_at = -1;
  for (int ms = 1; ms <= 1000; ++ms) {
    r.tick();
    if (safe_at < 0 && r.out.state == SafetyState::SAFE) safe_at = ms;
    if (stopped_at < 0 && r.out.v == 0.0f) stopped_at = ms;
  }
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::WATCHDOG_HEARTBEAT, r.out.reason);
  TEST_ASSERT_TRUE(safe_at >= 500 && safe_at <= 502);
  TEST_ASSERT_TRUE_MESSAGE(stopped_at > 0 && stopped_at <= 600, "T-07: stop within 600 ms");
  TEST_ASSERT_FALSE(r.out.drive_enabled);
}

void test_safe_recovers_after_link_returns() {
  Rig r;
  r.cmd(0.2f);
  r.run(500);
  r.link = false;
  r.run(700);
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  r.link = true;
  r.run(500);  // hold time not over yet
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  r.run(600);
  TEST_ASSERT_TRUE(r.out.state == SafetyState::ACTIVE);
  TEST_ASSERT_TRUE(r.out.v > 0.0f);
}

void test_stale_cmd_vel_while_moving_goes_safe() {
  Rig r;
  r.cmd(0.2f);
  r.run(500);
  r.send_cmd = false;  // heartbeat alive, Nav2 died with a non-zero last command
  r.run(499);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  r.run(3);
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::WATCHDOG_CMD, r.out.reason);
}

void test_stale_zero_cmd_is_just_idle() {
  Rig r;
  r.cmd(0.0f);
  r.run(100);
  r.send_cmd = false;
  r.run(2000);
  TEST_ASSERT_EQUAL(SafetyState::IDLE, r.out.state);
}

// ---- ToF ring (SF-02) ----

void test_tof_front_slow_below_40cm() {
  Rig r;
  r.cmd(0.3f);
  r.run(1000);
  r.tof(cfg::TOF_FRONT, 0.35f);
  r.run(500);
  TEST_ASSERT_EQUAL(SafetyState::SLOW, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::TOF_SLOW, r.out.reason);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f * cfg::MAX_SPEED_MPS, r.out.v);
  r.tof(cfg::TOF_FRONT, 0.45f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
}

void test_tof_front_corner_stop_below_15cm_is_immediate() {
  Rig r;
  r.cmd(0.3f);
  r.run(1000);
  r.tof(cfg::TOF_FRONT_LEFT, 0.12f);
  r.run(1);  // same tick: no ramp (NFR-04)
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::TOF_STOP, r.out.reason);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  TEST_ASSERT_FALSE(r.out.drive_enabled);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  // Obstacle gone → continue (FR-07).
  r.tof(cfg::TOF_FRONT_LEFT, 1.0f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
}

void test_reversing_ignores_front_sensors() {
  Rig r;
  r.tof(cfg::TOF_FRONT, 0.05f);
  r.tof(cfg::TOF_FRONT_LEFT, 0.05f);
  r.tof(cfg::TOF_FRONT_RIGHT, 0.05f);
  r.cmd(-0.2f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.2f, r.out.v);
  // …but forward is blocked.
  r.cmd(0.2f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
}

void test_forward_ignores_rear_sensor_and_reverse_uses_it() {
  Rig r;
  r.tof(cfg::TOF_REAR, 0.05f);
  r.cmd(0.2f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  r.cmd(-0.2f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  r.tof(cfg::TOF_REAR, 0.30f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::SLOW, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.09f, r.out.v);
}

void test_can_back_away_from_blocked_front() {
  Rig r;
  r.cmd(0.3f);
  r.run(1000);
  r.tof(cfg::TOF_FRONT, 0.10f);
  r.run(10);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  r.cmd(-0.1f);
  r.run(500);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.1f, r.out.v);
}

void test_side_sensors_only_slow_while_turning() {
  Rig r;
  r.tof(cfg::TOF_LEFT, 0.10f);  // wall close on the left
  r.cmd(0.3f, 0.0f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);  // straight: sides ignored
  r.cmd(0.3f, 0.5f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::SLOW, r.out.state);  // turning: slow, never stop
  TEST_ASSERT_TRUE(r.out.v > 0.0f);
}

void test_stale_leading_sensor_is_fail_safe_stop() {
  Rig r;
  r.cmd(0.2f);
  r.run(500);
  r.in.tof[cfg::TOF_FRONT_RIGHT].valid = false;  // I2C error / unplugged
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::TOF_FAULT, r.out.reason);
  // Reversing does not need the front sensor.
  r.cmd(-0.2f);
  r.run(500);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
}

void test_tof_stale_by_age() {
  Rig r;
  r.cmd(0.2f);
  r.run(200);
  // Sensor stops updating its stamp (valid flag stays true): stale after TOF_STALE_MS.
  r.frozen_tof = cfg::TOF_FRONT;
  r.run(cfg::TOF_STALE_MS);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  r.run(2);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::TOF_FAULT, r.out.reason);
}

// ---- SF-03 docking ----

void test_docking_ignores_front_sensors_and_caps_speed() {
  Rig r;
  r.in.docking = true;
  // the pallet itself: under the forks and, near the entrance, at both corners (outer stringers / rack shelf)
  r.tof(cfg::TOF_FRONT, 0.03f);
  r.tof(cfg::TOF_FRONT_LEFT, 0.04f);
  r.tof(cfg::TOF_FRONT_RIGHT, 0.04f);
  r.cmd(0.3f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::SLOW, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, cfg::DOCKING_MAX_MPS, r.out.v);
  // Leaving docking mode with something that close in front → full stop again.
  r.in.docking = false;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::STOP, r.out.state);
}

// ---- SF-08 raised forks ----

void test_raised_forks_cap_speed() {
  Rig r;
  r.in.fork_height_mm = 60.0f;
  r.cmd(0.3f);
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::SLOW, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::FORK_RAISED, r.out.reason);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, cfg::RAISED_FORK_MAX_MPS, r.out.v);
  r.in.fork_height_mm = 40.0f;
  r.run(1000);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, r.out.v);
  // Raising while moving fast: ramps down to the cap.
  r.in.fork_height_mm = 80.0f;
  r.run(300);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, cfg::RAISED_FORK_MAX_MPS, r.out.v);
}

void test_unknown_fork_height_counts_as_raised() {
  Rig r;
  r.in.fork_height_valid = false;
  r.cmd(-0.3f);
  r.run(1000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -cfg::RAISED_FORK_MAX_MPS, r.out.v);
}

void test_tightest_cap_wins() {
  SafetyConfig c;
  SpeedCap cap = speedCap(true, 80.0f, true, false, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.09f, cap.cap_mps);
  TEST_ASSERT_EQUAL(SafetyReason::TOF_SLOW, cap.reason);
  cap = speedCap(false, 0.0f, true, false, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, c.max_speed_mps, cap.cap_mps);
  TEST_ASSERT_EQUAL(SafetyReason::NONE, cap.reason);
}

// ---- SF-09 battery ----

void test_battery_low_flags_but_keeps_driving() {
  Rig r;
  r.in.battery_pct = 12.0f;
  r.cmd(0.2f);
  r.run(1000);
  TEST_ASSERT_TRUE(r.out.battery_low);
  TEST_ASSERT_EQUAL(SafetyState::ACTIVE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::BATTERY_LOW, r.out.reason);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, r.out.v);  // must still reach the charger
  // Hysteresis: 16 % is not enough to clear, 19 % is.
  r.in.battery_pct = 16.0f;
  r.run(10);
  TEST_ASSERT_TRUE(r.out.battery_low);
  r.in.battery_pct = 19.0f;
  r.run(10);
  TEST_ASSERT_FALSE(r.out.battery_low);
}

void test_battery_critical_is_safe_stop_with_hysteresis() {
  Rig r;
  r.cmd(0.3f);
  r.run(1000);
  r.in.battery_pct = 7.0f;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::BATTERY_CRITICAL, r.out.reason);
  r.run(100);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.v);
  r.in.battery_pct = 10.0f;  // voltage recovers a little when the load is gone: still critical
  r.run(3000);
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  r.in.battery_pct = 40.0f;  // on the charger
  r.run(1100);
  TEST_ASSERT_TRUE(r.out.state != SafetyState::SAFE);
}

void test_battery_curve() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, batteryPercentFromVoltage(5.5f));
  TEST_ASSERT_EQUAL_FLOAT(100.0f, batteryPercentFromVoltage(8.6f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, batteryPercentFromVoltage(7.6f));
  TEST_ASSERT_TRUE(batteryPercentFromVoltage(7.0f) < cfg::BATTERY_LOW_PCT);
  TEST_ASSERT_TRUE(batteryPercentFromVoltage(6.7f) < cfg::BATTERY_CRITICAL_PCT);
}

// ---- misc ----

void test_sensor_fault_goes_safe() {
  Rig r;
  r.cmd(0.2f);
  r.run(500);
  r.in.sensor_fault = true;
  r.run(1);
  TEST_ASSERT_EQUAL(SafetyState::SAFE, r.out.state);
  TEST_ASSERT_EQUAL(SafetyReason::SENSOR_FAULT, r.out.reason);
}

void test_wire_encoding_roundtrip() {
  const uint8_t w = encodeSafetyState(SafetyState::STOP, SafetyReason::TOF_STOP);
  TEST_ASSERT_EQUAL_HEX8(0x44, w);
  TEST_ASSERT_EQUAL(SafetyState::STOP, decodeState(w));
  TEST_ASSERT_EQUAL(SafetyReason::TOF_STOP, decodeReason(w));
  TEST_ASSERT_EQUAL_STRING("ESTOP", toString(SafetyState::ESTOP));
}

void test_curvature_preserved_under_cap() {
  Rig r;
  r.in.fork_height_mm = 80.0f;
  r.cmd(0.3f, 0.6f);  // κ = 2 m⁻¹
  r.run(1000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, r.out.v);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, r.out.w);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_waits_for_drivers);
  RUN_TEST(test_without_jetson_stays_idle_and_ignores_cmd);
  RUN_TEST(test_idle_to_active_with_accel_ramp);
  RUN_TEST(test_speed_never_exceeds_max);
  RUN_TEST(test_estop_latches_and_needs_explicit_reset);
  RUN_TEST(test_estop_overrides_everything);
  RUN_TEST(test_heartbeat_loss_goes_safe_and_stops_within_600ms);
  RUN_TEST(test_safe_recovers_after_link_returns);
  RUN_TEST(test_stale_cmd_vel_while_moving_goes_safe);
  RUN_TEST(test_stale_zero_cmd_is_just_idle);
  RUN_TEST(test_tof_front_slow_below_40cm);
  RUN_TEST(test_tof_front_corner_stop_below_15cm_is_immediate);
  RUN_TEST(test_reversing_ignores_front_sensors);
  RUN_TEST(test_forward_ignores_rear_sensor_and_reverse_uses_it);
  RUN_TEST(test_can_back_away_from_blocked_front);
  RUN_TEST(test_side_sensors_only_slow_while_turning);
  RUN_TEST(test_stale_leading_sensor_is_fail_safe_stop);
  RUN_TEST(test_tof_stale_by_age);
  RUN_TEST(test_docking_ignores_front_sensors_and_caps_speed);
  RUN_TEST(test_raised_forks_cap_speed);
  RUN_TEST(test_unknown_fork_height_counts_as_raised);
  RUN_TEST(test_tightest_cap_wins);
  RUN_TEST(test_battery_low_flags_but_keeps_driving);
  RUN_TEST(test_battery_critical_is_safe_stop_with_hysteresis);
  RUN_TEST(test_battery_curve);
  RUN_TEST(test_sensor_fault_goes_safe);
  RUN_TEST(test_wire_encoding_roundtrip);
  RUN_TEST(test_curvature_preserved_under_cap);
  return UNITY_END();
}
