// Shared value types of the forklift core.
#pragma once

#include <cstdint>

#include "forklift/config.h"

namespace fl {

// FW-08 state machine. Numeric values are part of the safety_state wire format — do not reorder.
enum class SafetyState : uint8_t {
  BOOT = 0,
  IDLE = 1,
  ACTIVE = 2,
  SLOW = 3,
  STOP = 4,
  SAFE = 5,
  ESTOP = 6,
};

// Main reason for the current state (one per tick, highest priority wins). 4 bits on the wire.
enum class SafetyReason : uint8_t {
  NONE = 0,
  BOOTING = 1,
  NO_LINK = 2,             // Jetson heartbeat never seen yet (FW-09)
  TOF_SLOW = 3,            // SF-02 < 40 cm
  TOF_STOP = 4,            // SF-02 < 15 cm
  TOF_FAULT = 5,           // leading ToF stale / error
  FORK_RAISED = 6,         // SF-08
  DOCKING = 7,             // SF-03
  WATCHDOG_HEARTBEAT = 8,  // FW-06 / SF-04: heartbeat lost
  WATCHDOG_CMD = 9,        // FW-06: moving and cmd_vel lost
  BATTERY_LOW = 10,        // SF-09 (flag only)
  BATTERY_CRITICAL = 11,   // SF-09 safe stop
  ESTOP_PRESSED = 12,
  ESTOP_WAIT_RESET = 13,   // released, waiting for an explicit reset (FW-08)
  SENSOR_FAULT = 14,       // encoder / pot failure
};

// std_msgs/UInt8 on `safety_state`: high nibble = state, low nibble = reason.
inline uint8_t encodeSafetyState(SafetyState s, SafetyReason r) {
  return static_cast<uint8_t>((static_cast<uint8_t>(s) << 4) | (static_cast<uint8_t>(r) & 0x0F));
}
inline SafetyState decodeState(uint8_t v) { return static_cast<SafetyState>(v >> 4); }
inline SafetyReason decodeReason(uint8_t v) { return static_cast<SafetyReason>(v & 0x0F); }

const char* toString(SafetyState s);
const char* toString(SafetyReason r);

struct ToFReading {
  float distance_m = cfg::TOF_NO_TARGET_M;
  bool valid = false;      // false = I2C error / timeout / not initialised
  uint32_t stamp_ms = 0;   // time of the last good reading
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

}  // namespace fl
