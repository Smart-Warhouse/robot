// One mutex per I2C bus (priority inheritance) — the bus is shared by tasks on both cores.
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace fl {

class I2cBus {
 public:
  void begin(TwoWire& wire, int sda, int scl, uint32_t freq);
  // Returns false if the bus could not be taken within `wait_ms` (0 = do not wait).
  bool lock(uint32_t wait_ms);
  void unlock();
  TwoWire& wire() { return *wire_; }

  bool writeReg8(uint8_t addr, uint8_t reg, uint8_t value);
  bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);
  bool probe(uint8_t addr);

  uint32_t errors = 0;

 private:
  TwoWire* wire_ = nullptr;
  SemaphoreHandle_t mutex_ = nullptr;
};

// RAII guard.
class I2cLock {
 public:
  I2cLock(I2cBus& b, uint32_t wait_ms) : bus_(b), ok_(b.lock(wait_ms)) {}
  ~I2cLock() {
    if (ok_) bus_.unlock();
  }
  explicit operator bool() const { return ok_; }

 private:
  I2cBus& bus_;
  bool ok_;
};

extern I2cBus i2c_main;     // TCA9548A + ToFs, MPU9250, INA219 (+ AS5600 by default)
extern I2cBus i2c_encoder;  // AS5600 when cfg::AS5600_ON_SECOND_BUS
I2cBus& encoderBus();

}  // namespace fl
