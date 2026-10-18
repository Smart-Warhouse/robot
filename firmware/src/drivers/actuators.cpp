#include "drivers/actuators.h"

#include "forklift/config.h"

namespace fl {

HBridge drive_motor, lift_motor, sideshift_motor;
SteeringServo steer_servo;

void pwmAttach(int pin, int channel, uint32_t freq, uint8_t bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, freq, bits, channel);
#else
  ledcSetup(channel, freq, bits);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(int pin, int channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void HBridge::begin(int pin_in1, int pin_in2, int ch1, int ch2, bool invert) {
  in1_ = pin_in1;
  in2_ = pin_in2;
  ch1_ = ch1;
  ch2_ = ch2;
  invert_ = invert;
  max_ = (1u << cfg::MOTOR_PWM_BITS) - 1;
  pwmAttach(in1_, ch1_, cfg::MOTOR_PWM_HZ, cfg::MOTOR_PWM_BITS);
  pwmAttach(in2_, ch2_, cfg::MOTOR_PWM_HZ, cfg::MOTOR_PWM_BITS);
  coast();
}

void HBridge::set(float duty) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  duty_ = duty;
  if (invert_) duty = -duty;
  const uint32_t d = static_cast<uint32_t>(fabsf(duty) * max_ + 0.5f);
  if (duty >= 0.0f) {
    pwmWrite(in2_, ch2_, 0);
    pwmWrite(in1_, ch1_, d);
  } else {
    pwmWrite(in1_, ch1_, 0);
    pwmWrite(in2_, ch2_, d);
  }
}

void HBridge::brake() {
  duty_ = 0.0f;
  pwmWrite(in1_, ch1_, max_);
  pwmWrite(in2_, ch2_, max_);
}

void HBridge::coast() {
  duty_ = 0.0f;
  pwmWrite(in1_, ch1_, 0);
  pwmWrite(in2_, ch2_, 0);
}

void SteeringServo::begin(int pin, int channel) {
  pin_ = pin;
  ch_ = channel;
  pwmAttach(pin_, ch_, cfg::SERVO_PWM_HZ, cfg::SERVO_PWM_BITS);
}

void SteeringServo::writeMicroseconds(float us) {
  if (us == us_) return;
  us_ = us;
  const float period_us = 1e6f / cfg::SERVO_PWM_HZ;
  const uint32_t max = (1u << cfg::SERVO_PWM_BITS) - 1;
  pwmWrite(pin_, ch_, static_cast<uint32_t>(us / period_us * max + 0.5f));
}

void actuatorsBegin() {
  drive_motor.begin(cfg::PIN_DRIVE_IN1, cfg::PIN_DRIVE_IN2, LEDC_DRIVE_A, LEDC_DRIVE_B);
  lift_motor.begin(cfg::PIN_LIFT_IN1, cfg::PIN_LIFT_IN2, LEDC_LIFT_A, LEDC_LIFT_B);
  sideshift_motor.begin(cfg::PIN_SIDESHIFT_IN1, cfg::PIN_SIDESHIFT_IN2, LEDC_SHIFT_A, LEDC_SHIFT_B);
  steer_servo.begin(cfg::PIN_STEER_SERVO, LEDC_SERVO);
}

void actuatorsAllStop() {
  drive_motor.brake();
  lift_motor.coast();
  sideshift_motor.coast();
}

}  // namespace fl
