#include "forklift/types.h"

namespace fl {

const char* toString(SafetyState s) {
  switch (s) {
    case SafetyState::BOOT: return "BOOT";
    case SafetyState::IDLE: return "IDLE";
    case SafetyState::ACTIVE: return "ACTIVE";
    case SafetyState::SLOW: return "SLOW";
    case SafetyState::STOP: return "STOP";
    case SafetyState::SAFE: return "SAFE";
    case SafetyState::ESTOP: return "ESTOP";
  }
  return "?";
}

const char* toString(SafetyReason r) {
  switch (r) {
    case SafetyReason::NONE: return "none";
    case SafetyReason::BOOTING: return "booting";
    case SafetyReason::NO_LINK: return "no_link";
    case SafetyReason::TOF_SLOW: return "tof_slow";
    case SafetyReason::TOF_STOP: return "tof_stop";
    case SafetyReason::TOF_FAULT: return "tof_fault";
    case SafetyReason::FORK_RAISED: return "fork_raised";
    case SafetyReason::DOCKING: return "docking";
    case SafetyReason::WATCHDOG_HEARTBEAT: return "watchdog_heartbeat";
    case SafetyReason::WATCHDOG_CMD: return "watchdog_cmd";
    case SafetyReason::BATTERY_LOW: return "battery_low";
    case SafetyReason::BATTERY_CRITICAL: return "battery_critical";
    case SafetyReason::ESTOP_PRESSED: return "estop_pressed";
    case SafetyReason::ESTOP_WAIT_RESET: return "estop_wait_reset";
    case SafetyReason::SENSOR_FAULT: return "sensor_fault";
  }
  return "?";
}

}  // namespace fl
