// State shared between the FreeRTOS tasks. Every block is copied in and out under a spinlock
// (portMUX), so a reader never sees half of an update and no task ever blocks on another.
#pragma once

#include <Arduino.h>

#include <atomic>

#include "forklift_core.h"

namespace fl {

template <typename T>
class Shared {
 public:
  T get() const {
    portENTER_CRITICAL(&mux_);
    T copy = value_;
    portEXIT_CRITICAL(&mux_);
    return copy;
  }
  void set(const T& v) {
    portENTER_CRITICAL(&mux_);
    value_ = v;
    portEXIT_CRITICAL(&mux_);
  }
  template <typename F>
  void modify(F&& f) {
    portENTER_CRITICAL(&mux_);
    f(value_);
    portEXIT_CRITICAL(&mux_);
  }

 private:
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  T value_{};
};

// Written by microros_task (or the bench console), read by safety_task / motor_pid_task.
struct CommandState {
  float cmd_v = 0.0f, cmd_w = 0.0f;
  bool cmd_seen = false;
  uint32_t last_cmd_ms = 0;
  bool heartbeat_seen = false;
  uint32_t last_heartbeat_ms = 0;
  uint32_t heartbeat_value = 0;
  float lift_target_mm = 0.0f;
  bool lift_cmd_seen = false;  // until the first lift_cmd the forks hold where they are
  int8_t sideshift = 0;
  uint32_t sideshift_ms = 0;
  bool docking = false;
};

struct TofState {
  ToFReading r[cfg::TOF_COUNT];
  uint16_t errors[cfg::TOF_COUNT] = {};
  bool present[cfg::TOF_COUNT] = {};
};

// Written by motor_pid_task.
struct DriveState {
  float speed_mps = 0.0f;       // measured (AS5600)
  float steer_rad = 0.0f;       // commanded to the servo
  float fork_mm = 0.0f;
  bool fork_valid = false;
  float fork_raw = 0.0f;        // ADC counts, for calibration
  uint16_t encoder_raw = 0;
  bool encoder_ok = false;
  bool limit_top = false, limit_bottom = false;
  bool tcrt_left = false, tcrt_right = false;
  bool lift_at_target = false;
  Pose2D pose;
  float odom_v = 0.0f, odom_w = 0.0f;
  float drive_duty = 0.0f, lift_duty = 0.0f;
};

struct ImuState {
  Quaternion q;
  float gyro[3] = {};   // rad/s, bias removed
  float accel[3] = {};  // m/s²
  bool ok = false;
};

struct PowerState {
  float voltage = 0.0f, current = 0.0f, percent = 100.0f;
  bool valid = false;
};

// Loop timing per task (FW-10).
struct TaskStats {
  uint32_t last_us = 0, max_us = 0, overruns = 0, loops = 0;
  void record(uint32_t us, uint32_t budget_us) {
    last_us = us;
    if (us > max_us) max_us = us;
    if (us > budget_us) ++overruns;
    ++loops;
  }
};

enum TaskId { T_SAFETY, T_MOTOR, T_TOF, T_IMU, T_MICROROS, T_STATUS, T_COUNT };

struct App {
  Shared<CommandState> cmd;
  Shared<TofState> tof;
  Shared<DriveState> drive;
  Shared<ImuState> imu;
  Shared<PowerState> power;
  Shared<SafetyOutput> safety;
  Shared<Calibration> cal;
  TaskStats stats[T_COUNT];  // each written only by its own task

  std::atomic<uint32_t> safety_tick{0};
  std::atomic<bool> boot_complete{false};
  std::atomic<bool> reset_request{false};
  std::atomic<bool> estop_edge{false};
  std::atomic<bool> limit_top_edge{false};
  std::atomic<bool> limit_bottom_edge{false};
  std::atomic<bool> sensor_fault{false};
  std::atomic<bool> odom_reset{false};
  std::atomic<bool> gains_changed{false};
  std::atomic<bool> bench_mode{false};  // console drives cmd_vel + heartbeat (no Jetson)
  std::atomic<bool> imu_cal_request{false};
  std::atomic<uint32_t> microros_state{0};  // 0 off/disabled, 1 waiting agent, 2 connected

  char robot_id[16] = "fl0";
};

extern App app;

inline uint32_t nowMs() { return millis(); }

}  // namespace fl
