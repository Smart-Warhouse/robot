#include "drivers/indicators.h"

#include <Adafruit_NeoPixel.h>

#include "drivers/sensors.h"
#include "forklift/config.h"

namespace fl {

namespace {
constexpr int kLeds = 7;
Adafruit_NeoPixel ring(kLeds, cfg::PIN_LED_RING, NEO_GRB + NEO_KHZ800);
SafetyState last_state = SafetyState::BOOT;
uint32_t state_since_ms = 0;

void fill(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < kLeds; ++i) ring.setPixelColor(i, ring.Color(r, g, b));
}
}  // namespace

void indicatorsBegin() {
  ring.begin();
  ring.setBrightness(60);  // ≈ 25 %: visible, keeps the 5 V rail load low
  fill(0, 0, 40);
  ring.show();
}

void indicatorsUpdate(const SafetyOutput& s, uint32_t now) {
  if (s.state != last_state) {
    last_state = s.state;
    state_since_ms = now;
  }
  const uint32_t t = now - state_since_ms;
  const bool blink = (now / 500) % 2;  // 1 Hz
  bool beep = false;

  switch (s.state) {
    case SafetyState::BOOT:
      ring.clear();
      ring.setPixelColor((now / 100) % kLeds, ring.Color(0, 0, 255));
      break;
    case SafetyState::IDLE:
      if (s.reason == SafetyReason::NO_LINK) fill(blink ? 255 : 60, blink ? 180 : 40, 0);  // waiting
      else if (s.battery_low) fill(255, 180, 0);
      else fill(0, 80, 0);
      break;
    case SafetyState::ACTIVE:
      if (s.battery_low) fill(blink ? 255 : 0, blink ? 180 : 255, 0);
      else fill(0, 255, 0);
      break;
    case SafetyState::SLOW:
      fill(255, 180, 0);
      break;
    case SafetyState::STOP:
      fill(255, 0, 0);
      beep = t < 150 || (t > 300 && t < 450);  // two short beeps on entry
      break;
    case SafetyState::SAFE:
      fill(blink ? 255 : 40, 0, 0);
      beep = (now % 1000) < 200;
      break;
    case SafetyState::ESTOP:
      fill(255, 0, 0);
      beep = (now % 1000) < 500;
      break;
  }
  ring.show();
  buzzer(beep);
}

}  // namespace fl
