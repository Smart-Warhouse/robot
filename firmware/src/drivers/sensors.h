// Thin register-level drivers: AS5600 encoder, MPU9250 IMU, INA219 power monitor, TCA9548A mux,
// VL53L1X ToF ring, and the GPIO inputs (limits, TCRT5000, E-Stop sense, fork pot). Callers hold the bus
// lock (I2cLock) around every I2C call.
#pragma once

#include <Arduino.h>

#include "drivers/i2c_bus.h"
#include "forklift/config.h"
#include "forklift/types.h"

namespace fl {

// ---- AS5600 magnetic encoder on the drive axle ----
class As5600 {
 public:
  bool begin(I2cBus& bus);
  bool readRaw(uint16_t& raw);  // 0 … 4095
  bool magnetOk();              // MD set, neither too weak nor too strong
 private:
  I2cBus* bus_ = nullptr;
};

// ---- MPU9250 (accel + gyro; magnetometer not used, see madgwick.h) ----
class Mpu9250 {
 public:
  bool begin(I2cBus& bus);
  bool read(float gyro_radps[3], float accel_mps2[3]);
  uint8_t whoami() const { return whoami_; }

 private:
  I2cBus* bus_ = nullptr;
  uint8_t whoami_ = 0;
};

// ---- INA219 on the main battery line (EL-08) ----
class Ina219 {
 public:
  bool begin(I2cBus& bus);
  bool read(float& volts, float& amps);

 private:
  I2cBus* bus_ = nullptr;
};

// ---- VL53L1X × 6 behind a TCA9548A ----
class TofRing {
 public:
  // Initialises every sensor on its mux channel; returns the number found.
  int begin(I2cBus& bus);
  // Polls each sensor once (non-blocking) and updates `out`. Takes and releases the bus lock per sensor so
  // the 1 kHz encoder read never waits for more than one transfer.
  void poll(ToFReading out[cfg::TOF_COUNT], uint16_t errors[cfg::TOF_COUNT], uint32_t now_ms);
  bool present(int i) const { return present_[i]; }
  // Re-initialise sensors that dropped out (called at a low rate).
  void recover();

 private:
  bool select(uint8_t channel);
  bool initOne(int i);
  I2cBus* bus_ = nullptr;
  bool present_[cfg::TOF_COUNT] = {};
  uint8_t consecutive_errors_[cfg::TOF_COUNT] = {};
};

// ---- GPIO ----
void inputsBegin();  // pin modes + interrupts
bool estopPressedLevel();
bool limitTopLevel();
bool limitBottomLevel();
bool tcrtLeft();
bool tcrtRight();
float readForkPotRaw();  // averaged ADC counts 0 … 4095
void buzzer(bool on);

}  // namespace fl
