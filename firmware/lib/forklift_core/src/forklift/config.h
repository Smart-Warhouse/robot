// All firmware tunables in one place.
//
// Pins come from requirements §3 (ESP32-S3 pin table). Values marked PLACEHOLDER are design guesses that
// must be replaced after the forklift is opened and measured (ME-01 / ME-02) or after the bench tests
// (T-02 … T-07). Calibration values that differ per forklift (pot, servo centre, IMU bias, wheel radius,
// PID gains) live in calibration.h and are stored in NVS (FW-05); the numbers here are only their defaults.
//
// Pure C++17: no Arduino headers, so the same file is used by the board build and the host unit tests.
#pragma once

#include <cstdint>

namespace fl {
namespace cfg {

// ---------------------------------------------------------------------------------------------------
// Pins — requirements §3. Verify against the Waveshare ESP32-S3 Zero pinout before soldering (A4).
// Do not use strapping pins 0, 45, 46 (GPIO 3 is also a strapping pin on the S3, but only for the JTAG
// source when an eFuse is burnt — harmless on a stock chip). GPIO 19/20 are the native USB (micro-ROS).
// GPIO 21 is the Zero's on-board RGB LED.
// ---------------------------------------------------------------------------------------------------
constexpr int PIN_DRIVE_IN1 = 2;
constexpr int PIN_DRIVE_IN2 = 3;
constexpr int PIN_LIFT_IN1 = 4;
constexpr int PIN_LIFT_IN2 = 5;
constexpr int PIN_SIDESHIFT_IN1 = 6;
constexpr int PIN_SIDESHIFT_IN2 = 7;
constexpr int PIN_STEER_SERVO = 11;
constexpr int PIN_FORK_POT = 1;  // ADC1_CH0 (ADC1 keeps working with Wi-Fi on)
constexpr int PIN_I2C_SDA = 12;
constexpr int PIN_I2C_SCL = 13;
constexpr int PIN_LIMIT_TOP = 14;
constexpr int PIN_LIMIT_BOTTOM = 15;
constexpr int PIN_TCRT_LEFT = 16;
constexpr int PIN_TCRT_RIGHT = 17;
constexpr int PIN_ESTOP_SENSE = 18;  // second (NC) contact of the E-Stop button
constexpr int PIN_LED_RING = 38;
constexpr int PIN_BUZZER = 39;

// Optional second I2C bus for the AS5600 so the 1 kHz encoder read never waits behind a ToF transfer.
// Off by default (the §3 table has one bus). PLACEHOLDER pins: GPIO 8 / 9 are free on the Zero.
constexpr bool AS5600_ON_SECOND_BUS = false;
constexpr int PIN_I2C2_SDA = 8;
constexpr int PIN_I2C2_SCL = 9;

// Input polarity. Limit switches and TCRT5000 modules pull low when active (internal pull-ups on).
// The E-Stop second contact is wired NC to GND: pressed (or a broken wire) reads HIGH — fail-safe.
constexpr bool LIMIT_ACTIVE_LOW = true;
constexpr bool TCRT_ACTIVE_LOW = true;
constexpr bool ESTOP_PRESSED_HIGH = true;

// ---------------------------------------------------------------------------------------------------
// Buses and addresses
// ---------------------------------------------------------------------------------------------------
constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr uint8_t ADDR_TCA9548A = 0x70;
constexpr uint8_t ADDR_AS5600 = 0x36;
constexpr uint8_t ADDR_MPU9250 = 0x68;
constexpr uint8_t ADDR_INA219 = 0x40;
constexpr uint8_t ADDR_VL53L1X = 0x29;  // every ToF has this address; each sits on its own mux channel

// ---------------------------------------------------------------------------------------------------
// PWM
// ---------------------------------------------------------------------------------------------------
constexpr uint32_t MOTOR_PWM_HZ = 20000;  // 20 kHz: above hearing
constexpr uint8_t MOTOR_PWM_BITS = 10;
constexpr uint32_t SERVO_PWM_HZ = 50;
constexpr uint8_t SERVO_PWM_BITS = 14;

// ---------------------------------------------------------------------------------------------------
// FreeRTOS tasks — requirements §4 table (core, rate, priority)
// ---------------------------------------------------------------------------------------------------
constexpr int SAFETY_CORE = 1, SAFETY_PRIO = 24, SAFETY_HZ = 1000;
constexpr int MOTOR_CORE = 1, MOTOR_PRIO = 23, MOTOR_HZ = 1000;
constexpr int TOF_CORE = 1, TOF_PRIO = 22, TOF_HZ = 50;
constexpr int IMU_CORE = 0, IMU_PRIO = 10, IMU_HZ = 100;
constexpr int MICROROS_CORE = 0, MICROROS_PRIO = 9;
constexpr int STATUS_CORE = 0, STATUS_PRIO = 5, STATUS_HZ = 10;
constexpr uint32_t SAFETY_STALE_MS = 5;  // motor task brakes if the safety output is older than this

// ---------------------------------------------------------------------------------------------------
// Geometry and drive — PLACEHOLDER until ME-01 (same placeholders as code/training/forklift_rl/config.yaml)
// ---------------------------------------------------------------------------------------------------
constexpr float WHEELBASE_M = 0.20f;          // PLACEHOLDER front (drive) axle → rear (steer) axle
constexpr float MAX_STEER_RAD = 0.5236f;      // PLACEHOLDER 30°
constexpr float MAX_SPEED_MPS = 0.30f;        // NFR-03
constexpr float DRIVE_SPEED_AT_FULL_DUTY = 0.45f;  // PLACEHOLDER m/s at 100 % PWM, for feed-forward
constexpr uint16_t AS5600_COUNTS_PER_REV = 4096;
constexpr float DRIVE_GEAR_RATIO = 1.0f;      // PLACEHOLDER magnet revolutions per wheel revolution
constexpr int ENCODER_SPEED_WINDOW = 10;      // samples (10 ms at 1 kHz) for the speed estimate
constexpr float ACCEL_MPS2 = 0.6f;            // normal acceleration limit
constexpr float DECEL_MPS2 = 1.2f;            // normal deceleration (e.g. entering SLOW)
constexpr float SAFE_DECEL_MPS2 = 5.0f;       // watchdog ramp: 0.3 m/s → 0 in 60 ms (T-07 ≤ 600 ms total)
constexpr float CMD_EPS_MPS = 0.005f;         // |v| below this is "no motion"
constexpr float TURN_EPS_RADPS = 0.05f;       // |ω| above this is "turning" (side ToF sensors count)

// ---------------------------------------------------------------------------------------------------
// Safety — requirements §9
// ---------------------------------------------------------------------------------------------------
constexpr float TOF_SLOW_M = 0.40f;           // SF-02: closer than this in the direction of motion → 30 %
constexpr float TOF_STOP_M = 0.15f;           // SF-02: closer than this → full stop
constexpr float TOF_SLOW_FACTOR = 0.30f;      // SF-02: 30 % of MAX_SPEED_MPS
constexpr uint32_t TOF_STALE_MS = 100;        // a ToF reading older than this counts as a sensor fault
constexpr bool TOF_FAULT_STOPS = true;        // fault on a leading sensor → STOP (fail-safe); false → SLOW
constexpr float TOF_NO_TARGET_M = 4.0f;       // reported distance when the sensor sees nothing
constexpr uint32_t WATCHDOG_MS = 500;         // FW-06 / SF-04
constexpr uint32_t SAFE_RECOVERY_HOLD_MS = 1000;  // cause must be gone this long before SAFE → IDLE
constexpr float FORK_RAISED_MM = 50.0f;       // SF-08: forks above 5 cm …
constexpr float RAISED_FORK_MAX_MPS = 0.10f;  // … → at most 0.1 m/s
constexpr float DOCKING_MAX_MPS = 0.10f;      // SF-03
constexpr float BATTERY_LOW_PCT = 15.0f;      // SF-09: flag, server sends the robot to charge
constexpr float BATTERY_CRITICAL_PCT = 8.0f;  // SF-09: safe stop
constexpr float BATTERY_HYST_PCT = 3.0f;

// ToF ring layout (ME-06) and TCA9548A channel of each sensor.
enum TofIndex : uint8_t {
  TOF_FRONT = 0,        // under the forks
  TOF_FRONT_LEFT = 1,
  TOF_FRONT_RIGHT = 2,
  TOF_LEFT = 3,
  TOF_RIGHT = 4,
  TOF_REAR = 5,
  TOF_COUNT = 6
};
constexpr uint8_t TOF_MUX_CHANNEL[TOF_COUNT] = {0, 1, 2, 3, 4, 5};  // PLACEHOLDER wiring
constexpr uint8_t TOF_MASK_FRONT = (1u << TOF_FRONT) | (1u << TOF_FRONT_LEFT) | (1u << TOF_FRONT_RIGHT);
constexpr uint8_t TOF_MASK_REAR = (1u << TOF_REAR);
constexpr uint8_t TOF_MASK_SIDES = (1u << TOF_LEFT) | (1u << TOF_RIGHT);
// SF-03: while docking, the front sensors that see the pallet itself are ignored. In the docking simulator the
// two corner sensors also see the pallet's outer stringers (or the rack shelf) once the fork tips are ~4 cm in, so
// ignoring only the centre one stopped 99.5 % of dockings (code/training/models/docking_v3/safety_eval.json). All
// three are ignored; the 0.1 m/s docking cap, the side/rear sensors and person detection stay active.
// Re-check on the real ToF ring (T-06/T-12): if a corner never sees the pallet, drop it from this mask.
constexpr uint8_t TOF_MASK_DOCKING_IGNORE = (1u << TOF_FRONT) | (1u << TOF_FRONT_LEFT) | (1u << TOF_FRONT_RIGHT);
constexpr uint32_t TOF_TIMING_BUDGET_US = 20000;  // 50 Hz per sensor (§4)

// ---------------------------------------------------------------------------------------------------
// Lift and side shift
// ---------------------------------------------------------------------------------------------------
constexpr float LIFT_MAX_MM = 120.0f;         // PLACEHOLDER lift range
constexpr float LIFT_STOP_BAND_MM = 1.5f;     // inside → motor off (FW-04 asks ± 3 mm)
constexpr float LIFT_RESTART_BAND_MM = 2.5f;  // outside → regulate again
constexpr float LIFT_MIN_DUTY = 0.20f;        // PLACEHOLDER duty that just overcomes the lift deadband
constexpr float LIFT_HEIGHT_FILTER = 0.05f;   // EMA factor on the pot reading per 1 ms sample
constexpr float SIDESHIFT_DUTY = 0.6f;
constexpr uint32_t SIDESHIFT_TIMEOUT_MS = 3000;  // no position feedback → stop after this

// ---------------------------------------------------------------------------------------------------
// Power — INA219 on the main line after the BMS (EL-08); 2S Li-ion 6.0–8.4 V
// ---------------------------------------------------------------------------------------------------
constexpr float INA219_SHUNT_OHM = 0.1f;   // PLACEHOLDER: stock 0.1 Ω reads only ±3.2 A; peak is ≈ 5 A →
                                           // fit 0.01 Ω (R010) and change this value
constexpr float BATTERY_EMPTY_V = 6.0f;
constexpr float BATTERY_FULL_V = 8.4f;
constexpr float BATTERY_CAPACITY_AH = 7.0f;
constexpr float BATTERY_FILTER = 0.1f;     // EMA per 10 Hz sample

// ---------------------------------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------------------------------
constexpr float MADGWICK_BETA = 0.05f;
constexpr int GYRO_CAL_SAMPLES = 500;      // 5 s at 100 Hz, robot stationary

// ---------------------------------------------------------------------------------------------------
// Identity (FW-12)
// ---------------------------------------------------------------------------------------------------
constexpr const char* DEFAULT_ROBOT_ID = "fl0";  // set per forklift with the console: `id fl1`

}  // namespace cfg
}  // namespace fl
