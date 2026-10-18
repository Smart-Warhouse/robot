#include "console.h"

#include <Arduino.h>

#include <cstdlib>
#include <cstring>

#include "app_state.h"
#include "microros_node.h"
#include "storage.h"
#include "tasks.h"

namespace fl {

namespace {

char line[96];
size_t line_len = 0;

void help() {
  Stream& o = console();
  o.println(F("commands:"));
  o.println(F("  status | diag | tof               show state / diagnostics JSON / ToF distances"));
  o.println(F("  id [name]                         show or set robot_id (FW-12, e.g. fl1)"));
  o.println(F("  reset                             leave ESTOP after the button is released (FW-08)"));
  o.println(F("  bench on|off                      console acts as the Jetson (heartbeat) — wheels off!"));
  o.println(F("  cmd <v m/s> <w rad/s>             cmd_vel in bench mode"));
  o.println(F("  step <v m/s> <ms>                 speed step, CSV log at 100 Hz (T-03)"));
  o.println(F("  lift <mm> | shift <-1|0|1> | dock on|off"));
  o.println(F("  odom reset"));
  o.println(F("  cal show | cal save | cal defaults | cal erase"));
  o.println(F("  cal pot bottom | cal pot top <mm>  fork pot two-point calibration"));
  o.println(F("  cal servo <us> | cal servo_gain <us/rad> | cal servo_limits <min> <max>"));
  o.println(F("  cal wheel <radius m> | cal encdir <1|-1> | cal imu"));
  o.println(F("  cal drive <kp> <ki> <kd> <kff> | cal lift <kp> <ki>"));
}

void showStatus() {
  Stream& o = console();
  const SafetyOutput s = app.safety.get();
  const DriveState d = app.drive.get();
  const PowerState p = app.power.get();
  o.printf("id=%s state=%s reason=%s link=%d bench=%d microros=%u\n", app.robot_id, toString(s.state),
           toString(s.reason), s.link_alive, app.bench_mode.load(), app.microros_state.load());
  o.printf("v_cmd=%.3f v_meas=%.3f cap=%.2f steer=%.3f rad duty=%.2f\n", s.v, d.speed_mps, s.speed_cap,
           d.steer_rad, d.drive_duty);
  o.printf("fork=%.1f mm (raw %.0f, valid %d) top=%d bottom=%d tcrt=%d/%d lift_duty=%.2f\n", d.fork_mm,
           d.fork_raw, d.fork_valid, d.limit_top, d.limit_bottom, d.tcrt_left, d.tcrt_right, d.lift_duty);
  o.printf("odom x=%.3f y=%.3f th=%.3f  enc_raw=%u ok=%d\n", d.pose.x, d.pose.y, d.pose.theta, d.encoder_raw,
           d.encoder_ok);
  o.printf("battery %.2f V %.2f A %.0f %% valid=%d low=%d\n", p.voltage, p.current, p.percent, p.valid,
           s.battery_low);
}

void showTof() {
  const TofState t = app.tof.get();
  static const char* names[] = {"front", "front_left", "front_right", "left", "right", "rear"};
  for (int i = 0; i < cfg::TOF_COUNT; ++i)
    console().printf("%-12s %6.3f m  valid=%d present=%d age=%lu ms err=%u\n", names[i], t.r[i].distance_m,
                     t.r[i].valid, t.present[i], static_cast<unsigned long>(nowMs() - t.r[i].stamp_ms),
                     t.errors[i]);
}

void showCal() {
  const Calibration c = app.cal.get();
  Stream& o = console();
  o.printf("pot raw bottom=%.0f top=%.0f top_mm=%.1f\n", c.pot_raw_bottom, c.pot_raw_top, c.pot_top_mm);
  o.printf("servo centre=%.0f us gain=%.0f us/rad limits=%.0f..%.0f\n", c.servo_center_us, c.servo_us_per_rad,
           c.servo_min_us, c.servo_max_us);
  o.printf("gyro bias %.5f %.5f %.5f  accel bias %.3f %.3f %.3f\n", c.gyro_bias[0], c.gyro_bias[1],
           c.gyro_bias[2], c.accel_bias[0], c.accel_bias[1], c.accel_bias[2]);
  o.printf("wheel radius %.4f m  encoder dir %.0f\n", c.wheel_radius_m, c.encoder_direction);
  o.printf("drive kp=%.3f ki=%.3f kd=%.3f kff=%.3f  lift kp=%.3f ki=%.3f\n", c.drive.kp, c.drive.ki, c.drive.kd,
           c.drive.kff, c.lift.kp, c.lift.ki);
  const char* why = validateCalibration(c);
  o.printf("valid: %s\n", why ? why : "yes");
}

void setCmd(float v, float w) {
  const uint32_t now = nowMs();
  app.cmd.modify([&](CommandState& c) {
    c.cmd_v = v;
    c.cmd_w = w;
    c.cmd_seen = true;
    c.last_cmd_ms = now;
  });
}

// T-03 bench step: drive at v for ms, then 0; prints t_ms, target, ramped, measured, duty.
void speedStep(float v, uint32_t ms) {
  if (!app.bench_mode.load()) {
    console().println(F("step needs `bench on` (wheels off the ground)"));
    return;
  }
  Stream& o = console();
  o.println(F("t_ms,v_target,v_ramped,v_measured,duty,state"));
  const uint32_t t0 = nowMs();
  setCmd(v, 0.0f);
  while (nowMs() - t0 < ms + 500) {
    const uint32_t t = nowMs() - t0;
    if (t >= ms) setCmd(0.0f, 0.0f);
    else setCmd(v, 0.0f);
    app.cmd.modify([&](CommandState& c) {
      c.heartbeat_seen = true;
      c.last_heartbeat_ms = nowMs();
    });
    const SafetyOutput s = app.safety.get();
    const DriveState d = app.drive.get();
    o.printf("%lu,%.3f,%.3f,%.4f,%.3f,%s\n", static_cast<unsigned long>(t), t < ms ? v : 0.0f, s.v, d.speed_mps,
             d.drive_duty, toString(s.state));
    delay(10);
  }
}

bool parseFloat(const char* s, float& out) {
  if (!s) return false;
  char* end = nullptr;
  out = strtof(s, &end);
  return end != s;
}

void handle(char* l) {
  Stream& o = console();
  char* argv[8] = {};
  int argc = 0;
  for (char* tok = strtok(l, " \t"); tok && argc < 8; tok = strtok(nullptr, " \t")) argv[argc++] = tok;
  if (argc == 0) return;
  const char* c0 = argv[0];
  float a = 0, b = 0;

  if (!strcmp(c0, "help") || !strcmp(c0, "?")) help();
  else if (!strcmp(c0, "status")) showStatus();
  else if (!strcmp(c0, "tof")) showTof();
  else if (!strcmp(c0, "diag")) {
    static char buf[512];
    formatDiagnostics(buf, sizeof buf);
    o.println(buf);
  } else if (!strcmp(c0, "id")) {
    if (argc >= 2) {
      if (storageSaveRobotId(argv[1])) o.printf("robot_id saved: %s (reboot to apply)\n", argv[1]);
      else o.println(F("invalid id: [a-z][a-z0-9_]{0,14}"));
    } else {
      o.printf("robot_id=%s\n", app.robot_id);
    }
  } else if (!strcmp(c0, "reset")) {
    app.reset_request.store(true);
    o.println(F("reset requested"));
  } else if (!strcmp(c0, "bench") && argc >= 2) {
    const bool on = !strcmp(argv[1], "on");
    app.bench_mode.store(on);
    if (!on) setCmd(0.0f, 0.0f);
    o.printf("bench %s — %s\n", on ? "on" : "off",
             on ? "console heartbeat active" : "heartbeat stops: watchdog trips in 500 ms (T-07)");
  } else if (!strcmp(c0, "cmd") && argc >= 3 && parseFloat(argv[1], a) && parseFloat(argv[2], b)) {
    if (!app.bench_mode.load()) o.println(F("cmd needs `bench on`"));
    else setCmd(a, b);
  } else if (!strcmp(c0, "step") && argc >= 3 && parseFloat(argv[1], a) && parseFloat(argv[2], b)) {
    speedStep(a, static_cast<uint32_t>(b));
  } else if (!strcmp(c0, "lift") && argc >= 2 && parseFloat(argv[1], a)) {
    app.cmd.modify([&](CommandState& c) {
      c.lift_target_mm = constrain(a, 0.0f, cfg::LIFT_MAX_MM);
      c.lift_cmd_seen = true;
    });
  } else if (!strcmp(c0, "shift") && argc >= 2) {
    const int d = atoi(argv[1]);
    const uint32_t now = nowMs();
    app.cmd.modify([&](CommandState& c) {
      c.sideshift = static_cast<int8_t>(d > 0 ? 1 : (d < 0 ? -1 : 0));
      c.sideshift_ms = now;
    });
  } else if (!strcmp(c0, "dock") && argc >= 2) {
    const bool on = !strcmp(argv[1], "on");
    app.cmd.modify([&](CommandState& c) { c.docking = on; });
  } else if (!strcmp(c0, "odom") && argc >= 2 && !strcmp(argv[1], "reset")) {
    app.odom_reset.store(true);
  } else if (!strcmp(c0, "cal") && argc >= 2) {
    const char* c1 = argv[1];
    const DriveState d = app.drive.get();
    bool changed = true;
    if (!strcmp(c1, "show")) { showCal(); changed = false; }
    else if (!strcmp(c1, "save")) {
      o.println(storageSaveCalibration(app.cal.get()) ? F("calibration saved to NVS") : F("save FAILED (invalid?)"));
      changed = false;
    } else if (!strcmp(c1, "defaults")) app.cal.set(defaultCalibration());
    else if (!strcmp(c1, "erase")) { o.println(storageEraseCalibration() ? F("erased") : F("erase failed")); changed = false; }
    else if (!strcmp(c1, "pot") && argc >= 3 && !strcmp(argv[2], "bottom"))
      app.cal.modify([&](Calibration& c) { c.pot_raw_bottom = d.fork_raw; });
    else if (!strcmp(c1, "pot") && argc >= 4 && !strcmp(argv[2], "top") && parseFloat(argv[3], a))
      app.cal.modify([&](Calibration& c) { c.pot_raw_top = d.fork_raw; c.pot_top_mm = a; });
    else if (!strcmp(c1, "servo") && argc >= 3 && parseFloat(argv[2], a))
      app.cal.modify([&](Calibration& c) { c.servo_center_us = a; });
    else if (!strcmp(c1, "servo_gain") && argc >= 3 && parseFloat(argv[2], a))
      app.cal.modify([&](Calibration& c) { c.servo_us_per_rad = a; });
    else if (!strcmp(c1, "servo_limits") && argc >= 4 && parseFloat(argv[2], a) && parseFloat(argv[3], b))
      app.cal.modify([&](Calibration& c) { c.servo_min_us = a; c.servo_max_us = b; });
    else if (!strcmp(c1, "wheel") && argc >= 3 && parseFloat(argv[2], a))
      app.cal.modify([&](Calibration& c) { c.wheel_radius_m = a; });
    else if (!strcmp(c1, "encdir") && argc >= 3 && parseFloat(argv[2], a))
      app.cal.modify([&](Calibration& c) { c.encoder_direction = a < 0 ? -1.0f : 1.0f; });
    else if (!strcmp(c1, "imu")) {
      requestImuCalibration();
      o.println(F("keep the forklift still for 5 s…"));
      changed = false;
    } else if (!strcmp(c1, "drive") && argc >= 6) {
      float kp, ki, kd, kff;
      if (parseFloat(argv[2], kp) && parseFloat(argv[3], ki) && parseFloat(argv[4], kd) && parseFloat(argv[5], kff))
        app.cal.modify([&](Calibration& c) { c.drive = PidGains{kp, ki, kd, kff}; });
    } else if (!strcmp(c1, "lift") && argc >= 4 && parseFloat(argv[2], a) && parseFloat(argv[3], b))
      app.cal.modify([&](Calibration& c) { c.lift.kp = a; c.lift.ki = b; });
    else {
      o.println(F("unknown cal command (help)"));
      changed = false;
    }
    if (changed) {
      app.gains_changed.store(true);
      showCal();
      o.println(F("(not saved yet: `cal save`)"));
    }
  } else {
    o.println(F("unknown command (help)"));
  }
}

}  // namespace

void consoleBegin() {
#if FL_ENABLE_MICROROS
  Serial0.begin(115200);
#else
  Serial.begin(115200);
#endif
}

void consolePoll() {
  Stream& in = console();
  while (in.available()) {
    const int ch = in.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[line_len] = '\0';
      handle(line);
      line_len = 0;
    } else if (line_len < sizeof line - 1) {
      line[line_len++] = static_cast<char>(ch);
    }
  }
  // Bench mode: the console stands in for the Jetson so T-03/T-04/T-07 can run without ROS.
  static uint32_t last_hb = 0;
  if (app.bench_mode.load() && nowMs() - last_hb >= 50) {
    last_hb = nowMs();
    app.cmd.modify([](CommandState& c) {
      c.heartbeat_seen = true;
      c.last_heartbeat_ms = nowMs();
      if (c.cmd_seen) c.last_cmd_ms = nowMs();  // keep the last `cmd` alive
    });
  }
}

}  // namespace fl
