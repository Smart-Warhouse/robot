// FW-05 calibration and FW-12 robot_id in NVS (namespace "forklift").
#pragma once

#include <cstddef>

#include "forklift/calibration.h"

namespace fl {

// Returns true if a valid blob was found (false → defaults are in `out`).
bool storageLoadCalibration(Calibration& out);
bool storageSaveCalibration(Calibration c);  // seals before writing
bool storageEraseCalibration();

void storageLoadRobotId(char* out, size_t len);  // default cfg::DEFAULT_ROBOT_ID
bool storageSaveRobotId(const char* id);
bool isValidRobotId(const char* id);             // [a-z0-9_], 1…15 chars, starts with a letter

}  // namespace fl
