#include "drivers/i2c_bus.h"

#include "forklift/config.h"

namespace fl {

I2cBus i2c_main;
I2cBus i2c_encoder;

I2cBus& encoderBus() { return cfg::AS5600_ON_SECOND_BUS ? i2c_encoder : i2c_main; }

void I2cBus::begin(TwoWire& wire, int sda, int scl, uint32_t freq) {
  wire_ = &wire;
  if (!mutex_) mutex_ = xSemaphoreCreateMutex();
  wire_->begin(sda, scl, freq);
  wire_->setTimeOut(10);
}

bool I2cBus::lock(uint32_t wait_ms) {
  return xSemaphoreTake(mutex_, wait_ms ? pdMS_TO_TICKS(wait_ms) : 0) == pdTRUE;
}

void I2cBus::unlock() { xSemaphoreGive(mutex_); }

bool I2cBus::writeReg8(uint8_t addr, uint8_t reg, uint8_t value) {
  wire_->beginTransmission(addr);
  wire_->write(reg);
  wire_->write(value);
  const bool ok = wire_->endTransmission() == 0;
  if (!ok) ++errors;
  return ok;
}

bool I2cBus::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
  wire_->beginTransmission(addr);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    ++errors;
    return false;
  }
  const size_t got = wire_->requestFrom(addr, static_cast<uint8_t>(len));
  if (got != len) {
    ++errors;
    return false;
  }
  for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(wire_->read());
  return true;
}

bool I2cBus::probe(uint8_t addr) {
  wire_->beginTransmission(addr);
  return wire_->endTransmission() == 0;
}

}  // namespace fl
