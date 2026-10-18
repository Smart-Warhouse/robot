// micro-ROS node (requirements §10), compiled only with -DFL_ENABLE_MICROROS=1 (env esp32s3_microros).
// Without it microrosStart() does nothing: no heartbeat ever arrives, so the robot stays in IDLE / no_link
// and never moves (FW-09) — unless the bench console is put in `bench on` mode.
#pragma once

#ifndef FL_ENABLE_MICROROS
#define FL_ENABLE_MICROROS 0
#endif

namespace fl {

void microrosStart();

}  // namespace fl
