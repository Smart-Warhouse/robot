#include "forklift/calibration.h"

#include <cmath>
#include <cstring>
#include <type_traits>

namespace fl {

static_assert(std::is_trivially_copyable<Calibration>::value, "Calibration must be a POD blob");
static_assert(sizeof(Calibration) % 4 == 0, "no tail padding expected");
static_assert(offsetof(Calibration, crc) == sizeof(Calibration) - sizeof(uint32_t),
              "crc must be the last field");

Calibration defaultCalibration() {
  Calibration c{};  // every field zero (no padding: see static_asserts)
  c.pot_raw_bottom = 300.0f;   // PLACEHOLDER: run `cal pot bottom` / `cal pot top <mm>`
  c.pot_raw_top = 3800.0f;     // PLACEHOLDER
  c.pot_top_mm = cfg::LIFT_MAX_MM;
  c.servo_center_us = 1500.0f;
  c.servo_us_per_rad = 900.0f;  // MG90S ≈ 1000 µs per 90° → ≈ 640 µs/rad at the horn; PLACEHOLDER linkage
  c.servo_min_us = 1000.0f;
  c.servo_max_us = 2000.0f;
  c.wheel_radius_m = 0.030f;   // PLACEHOLDER: measure the drive wheel (or roll 2 m and fit)
  c.encoder_direction = 1.0f;
  c.drive = defaultDriveGains();
  c.lift = defaultLiftGains();
  sealCalibration(c);
  return c;
}

const char* validateCalibration(const Calibration& c) {
  auto finite = [](float v) { return std::isfinite(v); };
  if (!finite(c.pot_raw_bottom) || !finite(c.pot_raw_top) || !finite(c.pot_top_mm)) return "pot nan";
  if (std::fabs(c.pot_raw_top - c.pot_raw_bottom) < 200.0f) return "pot span < 200 counts";
  if (c.pot_top_mm < 10.0f || c.pot_top_mm > 500.0f) return "pot_top_mm out of range";
  if (c.servo_center_us < 1000.0f || c.servo_center_us > 2000.0f) return "servo centre out of range";
  if (std::fabs(c.servo_us_per_rad) < 100.0f || std::fabs(c.servo_us_per_rad) > 3000.0f)
    return "servo gain out of range";
  if (c.servo_min_us < 500.0f || c.servo_max_us > 2500.0f || c.servo_min_us >= c.servo_max_us)
    return "servo limits";
  for (int i = 0; i < 3; ++i) {
    if (!finite(c.gyro_bias[i]) || std::fabs(c.gyro_bias[i]) > 0.5f) return "gyro bias";
    if (!finite(c.accel_bias[i]) || std::fabs(c.accel_bias[i]) > 3.0f) return "accel bias";
  }
  if (c.wheel_radius_m < 0.005f || c.wheel_radius_m > 0.1f) return "wheel radius";
  if (c.encoder_direction != 1.0f && c.encoder_direction != -1.0f) return "encoder direction";
  const PidGains* gs[] = {&c.drive, &c.lift};
  for (const PidGains* g : gs) {
    if (!finite(g->kp) || !finite(g->ki) || !finite(g->kd) || !finite(g->kff)) return "gain nan";
    if (g->kp < 0 || g->ki < 0 || g->kd < 0 || g->kff < 0) return "negative gain";
  }
  return nullptr;
}

uint32_t crc32(const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

void sealCalibration(Calibration& c) {
  c.magic = kCalibrationMagic;
  c.version = kCalibrationVersion;
  c.size = sizeof(Calibration);
  c.crc = crc32(&c, offsetof(Calibration, crc));
}

bool isSealed(const Calibration& c) {
  return c.magic == kCalibrationMagic && c.version == kCalibrationVersion &&
         c.size == sizeof(Calibration) && c.crc == crc32(&c, offsetof(Calibration, crc));
}

Calibration loadCalibration(const void* blob, size_t len, bool* used_defaults) {
  if (blob && len == sizeof(Calibration)) {
    Calibration c;
    std::memcpy(&c, blob, sizeof(c));
    if (isSealed(c) && validateCalibration(c) == nullptr) {
      if (used_defaults) *used_defaults = false;
      return c;
    }
  }
  if (used_defaults) *used_defaults = true;
  return defaultCalibration();
}

float potToMm(const Calibration& c, float raw) {
  return (raw - c.pot_raw_bottom) * c.pot_top_mm / (c.pot_raw_top - c.pot_raw_bottom);
}

float mmToPot(const Calibration& c, float mm) {
  return c.pot_raw_bottom + mm * (c.pot_raw_top - c.pot_raw_bottom) / c.pot_top_mm;
}

float steerToServoUs(const Calibration& c, float steer_rad) {
  float us = c.servo_center_us + steer_rad * c.servo_us_per_rad;
  if (us < c.servo_min_us) us = c.servo_min_us;
  if (us > c.servo_max_us) us = c.servo_max_us;
  return us;
}

}  // namespace fl
