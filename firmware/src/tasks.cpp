#include "tasks.h"

#include <esp_task_wdt.h>

#include <cmath>
#include <cstdio>

#include "app_state.h"
#include "drivers/actuators.h"
#include "drivers/i2c_bus.h"
#include "drivers/indicators.h"
#include "drivers/sensors.h"
#include "microros_node.h"
#include "storage.h"

static_assert(configTICK_RATE_HZ == 1000, "1 kHz tasks need a 1 ms FreeRTOS tick");

namespace fl {

App app;

namespace {

As5600 encoder_dev;
Mpu9250 imu_dev;
Ina219 power_dev;
TofRing tof_ring;
SafetyStateMachine safety_sm;
BootReport boot_report;

constexpr uint32_t ENCODER_FAULT_MS = 50;  // no encoder sample for this long → SENSOR_FAULT

uint32_t elapsedUs(uint32_t t0) { return micros() - t0; }

// =========================================== safety_task ===========================================
void safetyTask(void*) {
  esp_task_wdt_add(nullptr);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1);
    const uint32_t t0 = micros();

    SafetyInputs in;
    in.now_ms = nowMs();
    in.boot_complete = app.boot_complete.load();
    in.estop_pressed = estopPressedLevel() || app.estop_edge.exchange(false);
    in.reset_requested = app.reset_request.exchange(false);

    const CommandState c = app.cmd.get();
    in.heartbeat_seen = c.heartbeat_seen;
    in.last_heartbeat_ms = c.last_heartbeat_ms;
    in.cmd_seen = c.cmd_seen;
    in.last_cmd_ms = c.last_cmd_ms;
    in.cmd_v = c.cmd_v;
    in.cmd_w = c.cmd_w;
    in.docking = c.docking;

    const TofState t = app.tof.get();
    for (int i = 0; i < cfg::TOF_COUNT; ++i) in.tof[i] = t.r[i];

    const DriveState d = app.drive.get();
    in.fork_height_mm = d.fork_mm;
    in.fork_height_valid = d.fork_valid;

    const PowerState p = app.power.get();
    in.battery_pct = p.percent;
    in.battery_valid = p.valid;
    in.sensor_fault = app.sensor_fault.load();

    app.safety.set(safety_sm.update(in, 0.001f));
    app.safety_tick.fetch_add(1);
    esp_task_wdt_reset();
    app.stats[T_SAFETY].record(elapsedUs(t0), 1000);
  }
}

// ========================================== motor_pid_task ==========================================
void motorTask(void*) {
  Calibration cal = app.cal.get();
  EncoderSpeed enc(cal.wheel_radius_m, cal.encoder_direction);
  Pid drive_pid(cal.drive, -1.0f, 1.0f);
  LiftController lift(cal.lift);
  RearAckermann ackermann;
  Odometry odom;

  float fork_mm = NAN;
  uint32_t last_safety_tick = 0, safety_tick_ms = nowMs();
  uint32_t last_encoder_ms = nowMs();
  uint32_t last_us = micros();
  SafetyState prev_state = SafetyState::BOOT;

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1);
    const uint32_t t0 = micros();
    const uint32_t now = nowMs();
    float dt = (t0 - last_us) * 1e-6f;
    last_us = t0;
    if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;

    if (app.gains_changed.exchange(false)) {
      cal = app.cal.get();
      drive_pid.setGains(cal.drive);
      lift.setGains(cal.lift);
      enc.setWheelRadius(cal.wheel_radius_m);
      enc.setDirection(cal.encoder_direction);
    }

    // ---- sensors ----
    uint16_t raw = 0;
    bool enc_ok = false;
    {
      I2cLock lock(encoderBus(), 0);  // never wait: skip this sample if the bus is busy
      if (lock) enc_ok = encoder_dev.readRaw(raw);
    }
    if (enc_ok) {
      enc.update(raw, t0);
      last_encoder_ms = now;
    }
    const bool encoder_fault = now - last_encoder_ms > ENCODER_FAULT_MS;
    app.sensor_fault.store(encoder_fault);

    const float pot = readForkPotRaw();
    const bool fork_valid = pot > 20.0f && pot < 4075.0f;  // a broken wire reads 0 or 4095
    const float mm = potToMm(cal, pot);
    fork_mm = std::isnan(fork_mm) ? mm : fork_mm + cfg::LIFT_HEIGHT_FILTER * (mm - fork_mm);

    const bool top = limitTopLevel() || app.limit_top_edge.exchange(false);
    const bool bottom = limitBottomLevel() || app.limit_bottom_edge.exchange(false);

    // ---- safety output (brake if safety_task stopped producing it) ----
    const SafetyOutput s = app.safety.get();
    const uint32_t tick = app.safety_tick.load();
    if (tick != last_safety_tick) {
      last_safety_tick = tick;
      safety_tick_ms = now;
    }
    const bool stale = now - safety_tick_ms > cfg::SAFETY_STALE_MS;

    // ---- drive ----
    float drive_duty = 0.0f;
    if (stale || !s.drive_enabled) {
      drive_pid.reset();
      drive_motor.brake();
    } else {
      drive_duty = drive_pid.update(s.v, enc.speed(), dt);
      drive_motor.set(drive_duty);
    }

    // ---- steering (FW-01) ----
    const DriveCommand dc = ackermann.toDrive(s.v, s.w);
    steer_servo.writeMicroseconds(steerToServoUs(cal, dc.steer_rad));

    // ---- odometry (FW-02) ----
    if (app.odom_reset.exchange(false)) odom.reset();
    odom.update(enc.speed(), dc.steer_rad, dt);

    // ---- lift (FW-04) and side shift ----
    // Hold the current height until the first lift_cmd, and again after leaving ESTOP / SAFE.
    const bool resumed = (prev_state == SafetyState::ESTOP || prev_state == SafetyState::SAFE ||
                          prev_state == SafetyState::BOOT) && s.lift_enabled;
    prev_state = s.state;
    CommandState c = app.cmd.get();
    if (fork_valid && (resumed || !c.lift_cmd_seen)) {
      app.cmd.modify([&](CommandState& cs) {
        cs.lift_target_mm = fork_mm < 0.0f ? 0.0f : fork_mm;
        if (resumed) cs.sideshift = 0;
      });
      c = app.cmd.get();
    }
    float lift_duty = 0.0f;
    if (!stale && s.lift_enabled && fork_valid) {
      lift_duty = lift.update(c.lift_target_mm, fork_mm, top, bottom, dt);
      lift_motor.set(lift_duty);
    } else {
      lift.reset();
      lift_motor.coast();
    }
    if (!stale && s.lift_enabled && c.sideshift != 0 && now - c.sideshift_ms < cfg::SIDESHIFT_TIMEOUT_MS)
      sideshift_motor.set(c.sideshift * cfg::SIDESHIFT_DUTY);
    else
      sideshift_motor.coast();

    // ---- publish ----
    app.drive.modify([&](DriveState& d) {
      d.speed_mps = enc.speed();
      d.steer_rad = dc.steer_rad;
      d.fork_mm = fork_mm;
      d.fork_valid = fork_valid;
      d.fork_raw = pot;
      d.encoder_raw = raw;
      d.encoder_ok = !encoder_fault;
      d.limit_top = top;
      d.limit_bottom = bottom;
      d.tcrt_left = tcrtLeft();
      d.tcrt_right = tcrtRight();
      d.lift_at_target = lift.atTarget();
      d.pose = odom.pose();
      d.odom_v = odom.v();
      d.odom_w = odom.w();
      d.drive_duty = drive_duty;
      d.lift_duty = lift_duty;
    });
    app.stats[T_MOTOR].record(elapsedUs(t0), 1000);
  }
}

// ============================================= tof_task =============================================
void tofTask(void*) {
  ToFReading local[cfg::TOF_COUNT];
  uint16_t errors[cfg::TOF_COUNT] = {};
  uint32_t last_recover = nowMs();
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1000 / cfg::TOF_HZ);
    const uint32_t t0 = micros();
    tof_ring.poll(local, errors, nowMs());
    app.tof.modify([&](TofState& t) {
      for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        t.r[i] = local[i];
        t.errors[i] = errors[i];
        t.present[i] = tof_ring.present(i);
      }
    });
    if (nowMs() - last_recover > 2000) {  // try to bring back a sensor that dropped out
      last_recover = nowMs();
      tof_ring.recover();
    }
    app.stats[T_TOF].record(elapsedUs(t0), 1000000 / cfg::TOF_HZ);
  }
}

// ============================================= imu_task =============================================
void imuTask(void*) {
  Madgwick filter(cfg::MADGWICK_BETA);
  float bias_sum[3] = {}, accel_sum[3] = {};
  int cal_n = -1;  // < 0: not calibrating
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1000 / cfg::IMU_HZ);
    const uint32_t t0 = micros();
    if (!boot_report.imu) {
      app.stats[T_IMU].record(elapsedUs(t0), 10000);
      continue;
    }
    float g[3], a[3];
    bool ok;
    {
      I2cLock lock(i2c_main, 5);
      ok = lock && imu_dev.read(g, a);
    }
    if (ok) {
      const Calibration cal = app.cal.get();
      if (app.imu_cal_request.exchange(false)) {
        cal_n = 0;
        for (int k = 0; k < 3; ++k) bias_sum[k] = accel_sum[k] = 0.0f;
      }
      if (cal_n >= 0) {  // FW-05: average at rest
        for (int k = 0; k < 3; ++k) {
          bias_sum[k] += g[k];
          accel_sum[k] += a[k];
        }
        if (++cal_n >= cfg::GYRO_CAL_SAMPLES) {
          app.cal.modify([&](Calibration& c) {
            for (int k = 0; k < 3; ++k) c.gyro_bias[k] = bias_sum[k] / cal_n;
            // Accel bias: x, y should read 0 and z +g when level.
            c.accel_bias[0] = accel_sum[0] / cal_n;
            c.accel_bias[1] = accel_sum[1] / cal_n;
            c.accel_bias[2] = accel_sum[2] / cal_n - 9.80665f;
          });
          console().printf("imu calibration done: gyro bias %.5f %.5f %.5f rad/s (use `cal save`)\n",
                           bias_sum[0] / cal_n, bias_sum[1] / cal_n, bias_sum[2] / cal_n);
          cal_n = -1;
          filter.reset();
        }
      }
      for (int k = 0; k < 3; ++k) {
        g[k] -= cal.gyro_bias[k];
        a[k] -= cal.accel_bias[k];
      }
      filter.updateImu(g[0], g[1], g[2], a[0], a[1], a[2], 1.0f / cfg::IMU_HZ);
      app.imu.modify([&](ImuState& s) {
        s.q = filter.q();
        for (int k = 0; k < 3; ++k) {
          s.gyro[k] = g[k];
          s.accel[k] = a[k];
        }
        s.ok = true;
      });
    } else {
      app.imu.modify([](ImuState& s) { s.ok = false; });
    }
    app.stats[T_IMU].record(elapsedUs(t0), 10000);
  }
}

// ============================================ status_task ===========================================
void statusTask(void*) {
  bool have = false;
  float pct_f = 100.0f;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1000 / cfg::STATUS_HZ);
    const uint32_t t0 = micros();
    if (boot_report.power) {
      float v = 0, a = 0;
      bool ok;
      {
        I2cLock lock(i2c_main, 5);
        ok = lock && power_dev.read(v, a);
      }
      if (ok && v > 1.0f) {
        const float pct = batteryPercentFromVoltage(v);
        pct_f = have ? pct_f + cfg::BATTERY_FILTER * (pct - pct_f) : pct;
        have = true;
        app.power.set(PowerState{v, a, pct_f, true});
      }
    }
    indicatorsUpdate(app.safety.get(), nowMs());
    app.stats[T_STATUS].record(elapsedUs(t0), 100000);
  }
}

void initTaskWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = 1000;
  wdt.idle_core_mask = 0;
  wdt.trigger_panic = true;
  esp_task_wdt_reconfigure(&wdt);
#else
  esp_task_wdt_init(1, true);  // safety_task stuck for 1 s → reboot (motors are off during BOOT)
  // Only safety_task is watched: the micro-ROS serial transport busy-waits in Stream::readBytes and may
  // keep IDLE0 from running for a while, which must not reboot the robot.
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
#endif
  // Faster than the hardware watchdog: motor_pid_task brakes on its own if the safety output is older
  // than SAFETY_STALE_MS (5 ms).
}

}  // namespace

void requestImuCalibration() { app.imu_cal_request.store(true); }

Stream& console() {
#if FL_ENABLE_MICROROS
  return Serial0;  // USB CDC belongs to micro-ROS; console on UART0 (GPIO 43 TX / 44 RX)
#else
  return Serial;
#endif
}

BootReport startFirmware() {
  actuatorsBegin();
  actuatorsAllStop();
  inputsBegin();

  storageLoadRobotId(app.robot_id, sizeof app.robot_id);
  Calibration cal;
  boot_report.calibration_from_nvs = storageLoadCalibration(cal);
  app.cal.set(cal);
  steer_servo.writeMicroseconds(cal.servo_center_us);

  i2c_main.begin(Wire, cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL, cfg::I2C_FREQ_HZ);
  if (cfg::AS5600_ON_SECOND_BUS) i2c_encoder.begin(Wire1, cfg::PIN_I2C2_SDA, cfg::PIN_I2C2_SCL, 1000000);

  indicatorsBegin();
  {
    I2cLock lock(encoderBus(), 100);
    boot_report.encoder = encoder_dev.begin(encoderBus()) && encoder_dev.magnetOk();
  }
  {
    I2cLock lock(i2c_main, 100);
    boot_report.imu = imu_dev.begin(i2c_main);
    boot_report.power = power_dev.begin(i2c_main);
  }
  boot_report.tof_found = tof_ring.begin(i2c_main);  // locks per sensor

  initTaskWatchdog();
  xTaskCreatePinnedToCore(safetyTask, "safety_task", 4096, nullptr, cfg::SAFETY_PRIO, nullptr, cfg::SAFETY_CORE);
  xTaskCreatePinnedToCore(motorTask, "motor_pid_task", 6144, nullptr, cfg::MOTOR_PRIO, nullptr, cfg::MOTOR_CORE);
  xTaskCreatePinnedToCore(tofTask, "tof_task", 4096, nullptr, cfg::TOF_PRIO, nullptr, cfg::TOF_CORE);
  xTaskCreatePinnedToCore(imuTask, "imu_task", 4096, nullptr, cfg::IMU_PRIO, nullptr, cfg::IMU_CORE);
  xTaskCreatePinnedToCore(statusTask, "status_task", 4096, nullptr, cfg::STATUS_PRIO, nullptr, cfg::STATUS_CORE);
  microrosStart();  // no-op without FL_ENABLE_MICROROS

  // A missing encoder is reported as SENSOR_FAULT (SAFE) by motor_pid_task; missing ToF sensors are stale
  // and block motion in their direction. Boot always completes so the state is visible.
  app.boot_complete.store(true);
  return boot_report;
}

size_t formatDiagnostics(char* buf, size_t len) {
  const SafetyOutput s = app.safety.get();
  const TofState t = app.tof.get();
  const DriveState d = app.drive.get();
  const PowerState p = app.power.get();
  const ImuState imu = app.imu.get();
  int n = snprintf(buf, len,
                   "{\"id\":\"%s\",\"state\":\"%s\",\"reason\":\"%s\",\"temp_c\":%.1f,\"heap\":%u,"
                   "\"encoder_ok\":%d,\"imu_ok\":%d,\"fork_ok\":%d,\"battery_ok\":%d,\"i2c_err\":%u,\"tof\":[",
                   app.robot_id, toString(s.state), toString(s.reason), temperatureRead(),
                   static_cast<unsigned>(ESP.getFreeHeap()), d.encoder_ok, imu.ok, d.fork_valid, p.valid,
                   static_cast<unsigned>(i2c_main.errors));
  for (int i = 0; i < cfg::TOF_COUNT && n > 0 && static_cast<size_t>(n) < len; ++i)
    n += snprintf(buf + n, len - n, "%s{\"ok\":%d,\"err\":%u}", i ? "," : "", t.present[i] && t.r[i].valid,
                  t.errors[i]);
  static const char* names[T_COUNT] = {"safety", "motor", "tof", "imu", "microros", "status"};
  if (n > 0 && static_cast<size_t>(n) < len) n += snprintf(buf + n, len - n, "],\"loop_us\":{");
  for (int i = 0; i < T_COUNT && n > 0 && static_cast<size_t>(n) < len; ++i)
    n += snprintf(buf + n, len - n, "%s\"%s\":[%u,%u,%u]", i ? "," : "", names[i],
                  static_cast<unsigned>(app.stats[i].last_us), static_cast<unsigned>(app.stats[i].max_us),
                  static_cast<unsigned>(app.stats[i].overruns));
  if (n > 0 && static_cast<size_t>(n) < len) n += snprintf(buf + n, len - n, "}}");
  return n > 0 ? static_cast<size_t>(n) : 0;
}

}  // namespace fl
