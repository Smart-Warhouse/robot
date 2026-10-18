#include "storage.h"

#include <Preferences.h>

#include <cstring>

namespace fl {

namespace {
constexpr const char* kNs = "forklift";
constexpr const char* kCalKey = "cal";
constexpr const char* kIdKey = "robot_id";
}  // namespace

bool storageLoadCalibration(Calibration& out) {
  Preferences p;
  if (!p.begin(kNs, true)) {
    out = defaultCalibration();
    return false;
  }
  Calibration blob;
  const size_t n = p.getBytesLength(kCalKey) == sizeof(Calibration) ? p.getBytes(kCalKey, &blob, sizeof blob) : 0;
  p.end();
  bool defaults = true;
  out = loadCalibration(n ? &blob : nullptr, n, &defaults);
  return !defaults;
}

bool storageSaveCalibration(Calibration c) {
  if (validateCalibration(c) != nullptr) return false;
  sealCalibration(c);
  Preferences p;
  if (!p.begin(kNs, false)) return false;
  const bool ok = p.putBytes(kCalKey, &c, sizeof c) == sizeof c;
  p.end();
  return ok;
}

bool storageEraseCalibration() {
  Preferences p;
  if (!p.begin(kNs, false)) return false;
  const bool ok = p.remove(kCalKey);
  p.end();
  return ok;
}

bool isValidRobotId(const char* id) {
  const size_t n = strlen(id);
  if (n == 0 || n > 15 || !(id[0] >= 'a' && id[0] <= 'z')) return false;
  for (size_t i = 0; i < n; ++i) {
    const char c = id[i];
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return true;
}

void storageLoadRobotId(char* out, size_t len) {
  strncpy(out, cfg::DEFAULT_ROBOT_ID, len - 1);
  out[len - 1] = '\0';
  Preferences p;
  if (!p.begin(kNs, true)) return;
  char buf[16] = {};
  if (p.getString(kIdKey, buf, sizeof buf) > 0 && isValidRobotId(buf)) {
    strncpy(out, buf, len - 1);
    out[len - 1] = '\0';
  }
  p.end();
}

bool storageSaveRobotId(const char* id) {
  if (!isValidRobotId(id)) return false;
  Preferences p;
  if (!p.begin(kNs, false)) return false;
  const bool ok = p.putString(kIdKey, id) > 0;
  p.end();
  return ok;
}

}  // namespace fl
