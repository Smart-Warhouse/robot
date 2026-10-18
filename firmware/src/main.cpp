// ESP32-S3 firmware — autonomous 1:14 forklift (Huina 1577 Pro), smart-warehouse fleet.
// Same binary on every forklift; the identity (robot_id) and calibration live in NVS (FW-05, FW-12).
// Hardware-independent logic: lib/forklift_core. Task layout: tasks.h.
#include <Arduino.h>

#include "app_state.h"
#include "console.h"
#include "microros_node.h"
#include "tasks.h"

void setup() {
  fl::consoleBegin();
  const fl::BootReport r = fl::startFirmware();
  fl::console().printf(
      "\nforklift firmware  id=%s  micro-ROS=%s\n  encoder=%d imu=%d ina219=%d tof=%d/6 calibration=%s\n"
      "type `help`\n",
      fl::app.robot_id, FL_ENABLE_MICROROS ? "on" : "off", r.encoder, r.imu, r.power, r.tof_found,
      r.calibration_from_nvs ? "nvs" : "defaults");
}

void loop() {
  fl::consolePoll();  // lowest priority (Arduino loop task)
  delay(5);
}
