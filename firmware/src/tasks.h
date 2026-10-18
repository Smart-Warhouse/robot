// FreeRTOS tasks — requirements §4 table.
//
//   task            core  rate     priority  job
//   safety_task      1    1 kHz    24        E-Stop, limits, ToF, watchdog → SafetyStateMachine
//   motor_pid_task   1    1 kHz    23        AS5600 speed PID, fork-height PID, steering servo, odometry
//   tof_task         1    50 Hz    22        6 × VL53L1X through the TCA9548A
//   imu_task         0    100 Hz   10        MPU9250 + Madgwick
//   microros_task    0    events    9        §10 topics (only with FL_ENABLE_MICROROS)
//   status_task      0    10 Hz     5        INA219, LED ring, buzzer
#pragma once

#include <Arduino.h>

namespace fl {

struct BootReport {
  bool encoder = false;
  bool imu = false;
  bool power = false;
  int tof_found = 0;
  bool calibration_from_nvs = false;
};

// Brings up drivers, loads NVS, starts every task. Never blocks on a missing sensor (FW-09).
BootReport startFirmware();

// Requests handled inside the tasks (set from the console or micro-ROS).
void requestImuCalibration();

// Text diagnostics (FW-10): JSON, one line.
size_t formatDiagnostics(char* buf, size_t len);

Stream& console();

}  // namespace fl
