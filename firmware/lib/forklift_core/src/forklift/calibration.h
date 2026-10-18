// FW-05: per-forklift calibration stored as one blob in NVS (key "cal"), sealed with a CRC32.
// A blob that is missing, from another version or fails validation is ignored and the defaults are used.
#pragma once

#include <cstddef>
#include <cstdint>

#include "forklift/config.h"
#include "forklift/control.h"

namespace fl {

struct Calibration {
  uint32_t magic;    // kMagic
  uint16_t version;  // kVersion — bump when the layout changes
  uint16_t size;     // sizeof(Calibration)

  // Fork slide pot (ADC raw 0 … 4095) → height in mm, linear between two points.
  float pot_raw_bottom;  // forks on the bottom limit
  float pot_raw_top;     // forks at pot_top_mm
  float pot_top_mm;

  // Steering servo (MG90S), pulse width in µs.
  float servo_center_us;
  float servo_us_per_rad;  // sign selects the direction: + means a longer pulse turns the rear wheels left
  float servo_min_us;
  float servo_max_us;

  // IMU biases (rad/s and m/s²), measured at rest.
  float gyro_bias[3];
  float accel_bias[3];

  // Drive.
  float wheel_radius_m;
  float encoder_direction;  // +1 / −1 so that forward reads positive

  // Loop gains (FW-03: also tunable at run time).
  PidGains drive;
  PidGains lift;

  uint32_t crc;  // CRC32 of every byte before this field
};

constexpr uint32_t kCalibrationMagic = 0x464B4C31;  // "FKL1"
constexpr uint16_t kCalibrationVersion = 1;

Calibration defaultCalibration();

// Returns nullptr when valid, else a short description of the first problem.
const char* validateCalibration(const Calibration& c);

uint32_t crc32(const void* data, size_t len);
void sealCalibration(Calibration& c);         // fills magic, version, size, crc
bool isSealed(const Calibration& c);          // magic, version, size and crc match

// Loads `blob` (len bytes, e.g. straight from NVS). Falls back to defaults; `used_defaults` tells which.
Calibration loadCalibration(const void* blob, size_t len, bool* used_defaults);

float potToMm(const Calibration& c, float raw);
float mmToPot(const Calibration& c, float mm);
float steerToServoUs(const Calibration& c, float steer_rad);

}  // namespace fl
