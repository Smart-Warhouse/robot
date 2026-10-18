// Motor drivers and steering servo on the ESP32 LEDC peripheral.
//
// H-bridge: MX1508 (two PWM inputs per motor). A TB6612FNG works with the same code if its PWMx and STBY
// pins are tied high and AINx/BINx are driven as IN1/IN2 — the §3 pin table has only two pins per motor.
//   forward  IN1 = PWM, IN2 = 0      reverse  IN1 = 0, IN2 = PWM
//   brake    IN1 = IN2 = 1           coast    IN1 = IN2 = 0
#pragma once

#include <Arduino.h>

namespace fl {

void pwmAttach(int pin, int channel, uint32_t freq, uint8_t bits);
void pwmWrite(int pin, int channel, uint32_t duty);

class HBridge {
 public:
  void begin(int pin_in1, int pin_in2, int ch1, int ch2, bool invert = false);
  void set(float duty);  // −1 … 1
  void brake();
  void coast();
  float duty() const { return duty_; }

 private:
  int in1_ = -1, in2_ = -1, ch1_ = 0, ch2_ = 0;
  bool invert_ = false;
  uint32_t max_ = 0;
  float duty_ = 0.0f;
};

class SteeringServo {
 public:
  void begin(int pin, int channel);
  void writeMicroseconds(float us);
  float microseconds() const { return us_; }

 private:
  int pin_ = -1, ch_ = 0;
  float us_ = 0.0f;
};

// LEDC channel plan (8 channels on the S3; channel pairs share a timer).
constexpr int LEDC_DRIVE_A = 0, LEDC_DRIVE_B = 1;       // timer 0, 20 kHz
constexpr int LEDC_LIFT_A = 2, LEDC_LIFT_B = 3;         // timer 1, 20 kHz
constexpr int LEDC_SHIFT_A = 4, LEDC_SHIFT_B = 5;       // timer 2, 20 kHz
constexpr int LEDC_SERVO = 6;                           // timer 3, 50 Hz

extern HBridge drive_motor, lift_motor, sideshift_motor;
extern SteeringServo steer_servo;

void actuatorsBegin();
void actuatorsAllStop();

}  // namespace fl
