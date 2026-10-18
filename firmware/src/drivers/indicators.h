// SF-12 indicators: WS2812 7-LED ring + active buzzer.
//   green  = normal (IDLE dim, ACTIVE bright)        yellow = slowing / waiting (SLOW, no link, battery low)
//   red + buzzer = stop / error (STOP, SAFE, ESTOP)  blue = booting
#pragma once

#include <Arduino.h>

#include "forklift/safety_state_machine.h"

namespace fl {

void indicatorsBegin();
// Called by status_task at 10 Hz.
void indicatorsUpdate(const SafetyOutput& s, uint32_t now_ms);

}  // namespace fl
