// FW-08 safety state machine, run by safety_task at 1 kHz on core 1.
//
//   BOOT → IDLE → ACTIVE ↔ SLOW → STOP → SAFE / ESTOP
//
//   BOOT   motors off until the drivers report ready
//   IDLE   no motion commanded (or no Jetson link yet — FW-09: alone, the firmware stays stopped)
//   ACTIVE moving at up to MAX_SPEED_MPS
//   SLOW   moving under a cap: ToF < 40 cm (30 %), forks > 5 cm (0.1 m/s), docking (0.1 m/s)
//   STOP   ToF < 15 cm in the direction of motion: speed forced to 0 at once; reversing away is allowed
//   SAFE   watchdog (no heartbeat / no cmd_vel while moving for 500 ms), critical battery, sensor fault:
//          ramp to 0, motors off. Returns to IDLE by itself once the cause has been gone for 1 s.
//   ESTOP  E-Stop contact open. Latched: needs the button released AND an explicit reset command.
#pragma once

#include <cstdint>

#include "forklift/safety_limiter.h"
#include "forklift/types.h"

namespace fl {

struct SafetyInputs {
  uint32_t now_ms = 0;
  bool boot_complete = false;
  bool estop_pressed = false;
  bool reset_requested = false;  // one-shot, consumed by this tick

  bool heartbeat_seen = false;
  uint32_t last_heartbeat_ms = 0;
  bool cmd_seen = false;
  uint32_t last_cmd_ms = 0;
  float cmd_v = 0.0f;  // last received cmd_vel.linear.x  (m/s)
  float cmd_w = 0.0f;  // last received cmd_vel.angular.z (rad/s)

  ToFReading tof[cfg::TOF_COUNT];
  float fork_height_mm = 0.0f;
  bool fork_height_valid = false;
  bool docking = false;

  float battery_pct = 100.0f;
  bool battery_valid = false;
  bool sensor_fault = false;  // drive encoder or fork pot dead
};

struct SafetyOutput {
  SafetyState state = SafetyState::BOOT;
  SafetyReason reason = SafetyReason::BOOTING;
  float v = 0.0f;          // ramped linear speed for the drive PID (m/s)
  float w = 0.0f;          // matching yaw rate (same curvature as the command)
  float v_target = 0.0f;   // limited target before the ramp
  float speed_cap = 0.0f;
  bool drive_enabled = false;  // false → brake the drive motor and reset its PID
  bool lift_enabled = false;
  bool link_alive = false;
  bool battery_low = false;
  TofEvaluation tof;

  uint8_t wire() const { return encodeSafetyState(state, reason); }
};

class SafetyStateMachine {
 public:
  explicit SafetyStateMachine(const SafetyConfig& c = SafetyConfig{});

  SafetyOutput update(const SafetyInputs& in, float dt_s);

  SafetyState state() const { return state_; }
  uint32_t transitions() const { return transitions_; }

 private:
  void setState(SafetyState s);
  SafetyOutput halted(SafetyReason why);

  SafetyConfig c_;
  BatteryMonitor battery_;
  SafetyState state_ = SafetyState::BOOT;
  SafetyReason safe_reason_ = SafetyReason::NONE;
  bool estop_latched_ = false;
  bool ever_linked_ = false;
  bool safe_clear_pending_ = false;
  uint32_t safe_clear_since_ms_ = 0;
  float v_out_ = 0.0f;
  float curvature_ = 0.0f;
  uint32_t transitions_ = 0;
};

}  // namespace fl
