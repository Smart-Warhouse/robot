// Line console for calibration (FW-05), robot_id (FW-12) and bench tests (T-02 … T-04, T-07).
// USB CDC when micro-ROS is off; UART0 (GPIO 43/44, 115200) when micro-ROS owns the USB port.
#pragma once

namespace fl {

void consoleBegin();
void consolePoll();  // called from loop()

}  // namespace fl
