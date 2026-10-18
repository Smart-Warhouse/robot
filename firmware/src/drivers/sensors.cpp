#include "drivers/sensors.h"

#include <VL53L1X.h>

#include "app_state.h"

namespace fl {

// =============================================== AS5600 ===============================================
namespace {
constexpr uint8_t AS5600_STATUS = 0x0B;
constexpr uint8_t AS5600_RAW_ANGLE = 0x0C;
}  // namespace

bool As5600::begin(I2cBus& bus) {
  bus_ = &bus;
  return bus_->probe(cfg::ADDR_AS5600);
}

bool As5600::readRaw(uint16_t& raw) {
  uint8_t b[2];
  if (!bus_->readRegs(cfg::ADDR_AS5600, AS5600_RAW_ANGLE, b, 2)) return false;
  raw = static_cast<uint16_t>(((b[0] & 0x0F) << 8) | b[1]);
  return true;
}

bool As5600::magnetOk() {
  uint8_t s = 0;
  if (!bus_->readRegs(cfg::ADDR_AS5600, AS5600_STATUS, &s, 1)) return false;
  const bool md = s & 0x20, ml = s & 0x10, mh = s & 0x08;
  return md && !ml && !mh;
}

// =============================================== MPU9250 ==============================================
namespace {
constexpr uint8_t MPU_SMPLRT_DIV = 0x19, MPU_CONFIG = 0x1A, MPU_GYRO_CONFIG = 0x1B,
                  MPU_ACCEL_CONFIG = 0x1C, MPU_ACCEL_CONFIG2 = 0x1D, MPU_ACCEL_XOUT_H = 0x3B,
                  MPU_PWR_MGMT_1 = 0x6B, MPU_WHO_AM_I = 0x75;
constexpr float GYRO_LSB_PER_DPS = 65.5f;   // ±500 °/s
constexpr float ACCEL_LSB_PER_G = 8192.0f;  // ±4 g
constexpr float DEG2RAD = 0.017453292519943f;
constexpr float G = 9.80665f;
}  // namespace

bool Mpu9250::begin(I2cBus& bus) {
  bus_ = &bus;
  if (!bus_->readRegs(cfg::ADDR_MPU9250, MPU_WHO_AM_I, &whoami_, 1)) return false;
  // 0x71 MPU9250, 0x73 MPU9255, 0x70 MPU6500 (common "MPU9250" boards ship a 6500 — fine, no mag needed)
  if (whoami_ != 0x71 && whoami_ != 0x73 && whoami_ != 0x70) return false;
  bool ok = bus_->writeReg8(cfg::ADDR_MPU9250, MPU_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_PWR_MGMT_1, 0x01);  // PLL clock
  delay(10);
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_CONFIG, 0x03);        // gyro DLPF 41 Hz
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_SMPLRT_DIV, 0x04);    // 1 kHz / 5 = 200 Hz
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_GYRO_CONFIG, 0x08);   // ±500 °/s
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_ACCEL_CONFIG, 0x08);  // ±4 g
  ok &= bus_->writeReg8(cfg::ADDR_MPU9250, MPU_ACCEL_CONFIG2, 0x03); // accel DLPF 41 Hz
  return ok;
}

bool Mpu9250::read(float gyro[3], float accel[3]) {
  uint8_t b[14];
  if (!bus_->readRegs(cfg::ADDR_MPU9250, MPU_ACCEL_XOUT_H, b, sizeof b)) return false;
  auto s16 = [&b](int i) { return static_cast<int16_t>((b[i] << 8) | b[i + 1]); };
  for (int k = 0; k < 3; ++k) {
    accel[k] = s16(2 * k) / ACCEL_LSB_PER_G * G;
    gyro[k] = s16(8 + 2 * k) / GYRO_LSB_PER_DPS * DEG2RAD;
  }
  return true;
}

// =============================================== INA219 ===============================================
namespace {
constexpr uint8_t INA_CONFIG = 0x00, INA_SHUNT = 0x01, INA_BUS = 0x02;
}

bool Ina219::begin(I2cBus& bus) {
  bus_ = &bus;
  // 32 V range, PGA /8 (±320 mV), 12-bit, shunt + bus continuous.
  TwoWire& w = bus_->wire();
  w.beginTransmission(cfg::ADDR_INA219);
  w.write(INA_CONFIG);
  w.write(0x39);
  w.write(0x9F);
  return w.endTransmission() == 0;
}

bool Ina219::read(float& volts, float& amps) {
  uint8_t b[2];
  if (!bus_->readRegs(cfg::ADDR_INA219, INA_BUS, b, 2)) return false;
  const uint16_t bus_reg = static_cast<uint16_t>((b[0] << 8) | b[1]);
  volts = static_cast<float>(bus_reg >> 3) * 0.004f;
  if (!bus_->readRegs(cfg::ADDR_INA219, INA_SHUNT, b, 2)) return false;
  const int16_t shunt = static_cast<int16_t>((b[0] << 8) | b[1]);
  amps = shunt * 10e-6f / cfg::INA219_SHUNT_OHM;
  return true;
}

// ============================================ VL53L1X ring ============================================
namespace {
VL53L1X tof_dev[cfg::TOF_COUNT];
constexpr uint8_t TOF_MAX_ERRORS = 5;
}  // namespace

bool TofRing::select(uint8_t channel) {
  TwoWire& w = bus_->wire();
  w.beginTransmission(cfg::ADDR_TCA9548A);
  w.write(static_cast<uint8_t>(1u << channel));
  return w.endTransmission() == 0;
}

bool TofRing::initOne(int i) {
  I2cLock lock(*bus_, 50);
  if (!lock || !select(cfg::TOF_MUX_CHANNEL[i])) return false;
  VL53L1X& s = tof_dev[i];
  s.setBus(&bus_->wire());
  s.setTimeout(50);
  if (!s.init()) return false;
  s.setDistanceMode(VL53L1X::Short);  // ≤ 1.3 m, best in ambient light; we only care about < 40 cm
  s.setMeasurementTimingBudget(cfg::TOF_TIMING_BUDGET_US);
  s.startContinuous(1000 / cfg::TOF_HZ);
  return true;
}

int TofRing::begin(I2cBus& bus) {
  bus_ = &bus;
  int n = 0;
  for (int i = 0; i < cfg::TOF_COUNT; ++i) {
    present_[i] = initOne(i);
    consecutive_errors_[i] = 0;
    if (present_[i]) ++n;
  }
  return n;
}

void TofRing::recover() {
  for (int i = 0; i < cfg::TOF_COUNT; ++i)
    if (!present_[i]) present_[i] = initOne(i);
}

void TofRing::poll(ToFReading out[cfg::TOF_COUNT], uint16_t errors[cfg::TOF_COUNT], uint32_t now_ms) {
  for (int i = 0; i < cfg::TOF_COUNT; ++i) {
    if (!present_[i]) {
      out[i].valid = false;
      continue;
    }
    I2cLock lock(*bus_, 2);
    if (!lock) continue;  // reading stays as it was; the safety task notices if it gets stale
    VL53L1X& s = tof_dev[i];
    if (!select(cfg::TOF_MUX_CHANNEL[i])) {
      ++errors[i];
      if (++consecutive_errors_[i] >= TOF_MAX_ERRORS) present_[i] = false;
      out[i].valid = false;
      continue;
    }
    if (!s.dataReady()) continue;
    const uint16_t mm = s.read(false);
    if (s.timeoutOccurred()) {
      ++errors[i];
      if (++consecutive_errors_[i] >= TOF_MAX_ERRORS) present_[i] = false;
      out[i].valid = false;
      continue;
    }
    consecutive_errors_[i] = 0;
    float d;
    switch (s.ranging_data.range_status) {
      case VL53L1X::RangeValid:
      case VL53L1X::RangeValidMinRangeClipped:
      case VL53L1X::RangeValidNoWrapCheckFail:
      case VL53L1X::SigmaFail:        // noisy but an object is there: use it (conservative)
      case VL53L1X::WrapTargetFail:
        d = mm * 0.001f;
        break;
      case VL53L1X::SignalFail:
      case VL53L1X::OutOfBoundsFail:
      case VL53L1X::None:
        d = cfg::TOF_NO_TARGET_M;     // nothing in range
        break;
      default:                        // HardwareFail, MinRangeFail, ProcessingFail, …
        ++errors[i];
        out[i].valid = false;
        continue;
    }
    out[i].distance_m = d;
    out[i].valid = true;
    out[i].stamp_ms = now_ms;
  }
}

// ================================================ GPIO ================================================
namespace {
void IRAM_ATTR isrEstop() { app.estop_edge.store(true); }
void IRAM_ATTR isrLimitTop() { app.limit_top_edge.store(true); }
void IRAM_ATTR isrLimitBottom() { app.limit_bottom_edge.store(true); }
}  // namespace

void inputsBegin() {
  pinMode(cfg::PIN_LIMIT_TOP, INPUT_PULLUP);
  pinMode(cfg::PIN_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(cfg::PIN_TCRT_LEFT, INPUT_PULLUP);
  pinMode(cfg::PIN_TCRT_RIGHT, INPUT_PULLUP);
  pinMode(cfg::PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(cfg::PIN_BUZZER, OUTPUT);
  digitalWrite(cfg::PIN_BUZZER, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(cfg::PIN_FORK_POT, ADC_11db);
  // Edges into the "pressed" direction are latched so even a sub-millisecond contact is seen.
  attachInterrupt(cfg::PIN_ESTOP_SENSE, isrEstop, cfg::ESTOP_PRESSED_HIGH ? RISING : FALLING);
  attachInterrupt(cfg::PIN_LIMIT_TOP, isrLimitTop, cfg::LIMIT_ACTIVE_LOW ? FALLING : RISING);
  attachInterrupt(cfg::PIN_LIMIT_BOTTOM, isrLimitBottom, cfg::LIMIT_ACTIVE_LOW ? FALLING : RISING);
}

bool estopPressedLevel() { return digitalRead(cfg::PIN_ESTOP_SENSE) == (cfg::ESTOP_PRESSED_HIGH ? HIGH : LOW); }
bool limitTopLevel() { return digitalRead(cfg::PIN_LIMIT_TOP) == (cfg::LIMIT_ACTIVE_LOW ? LOW : HIGH); }
bool limitBottomLevel() { return digitalRead(cfg::PIN_LIMIT_BOTTOM) == (cfg::LIMIT_ACTIVE_LOW ? LOW : HIGH); }
bool tcrtLeft() { return digitalRead(cfg::PIN_TCRT_LEFT) == (cfg::TCRT_ACTIVE_LOW ? LOW : HIGH); }
bool tcrtRight() { return digitalRead(cfg::PIN_TCRT_RIGHT) == (cfg::TCRT_ACTIVE_LOW ? LOW : HIGH); }

float readForkPotRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < 4; ++i) sum += analogRead(cfg::PIN_FORK_POT);
  return sum / 4.0f;
}

void buzzer(bool on) { digitalWrite(cfg::PIN_BUZZER, on ? HIGH : LOW); }

}  // namespace fl
