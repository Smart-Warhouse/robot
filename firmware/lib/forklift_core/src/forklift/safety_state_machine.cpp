#include "forklift/safety_state_machine.h"

#include <cmath>

namespace fl {

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void merge(TofEvaluation& into, const TofEvaluation& e) {
  into.slow = into.slow || e.slow;
  into.stop = into.stop || e.stop;
  into.fault = into.fault || e.fault;
  into.considered_mask |= e.considered_mask;
  into.slow_mask |= e.slow_mask;
  into.stop_mask |= e.stop_mask;
  into.fault_mask |= e.fault_mask;
}

}  // namespace

SafetyStateMachine::SafetyStateMachine(const SafetyConfig& c) : c_(c), battery_(c) {}

void SafetyStateMachine::setState(SafetyState s) {
  if (s != state_) {
    state_ = s;
    ++transitions_;
  }
}

SafetyOutput SafetyStateMachine::halted(SafetyReason why) {
  SafetyOutput out;
  out.state = state_;
  out.reason = why;
  out.v = out.w = out.v_target = out.speed_cap = 0.0f;
  out.drive_enabled = false;
  out.lift_enabled = false;
  out.battery_low = battery_.low();
  return out;
}

SafetyOutput SafetyStateMachine::update(const SafetyInputs& in, float dt) {
  const uint32_t now = in.now_ms;
  battery_.update(in.battery_pct, in.battery_valid);
  const bool link_alive = in.heartbeat_seen && (now - in.last_heartbeat_ms) <= c_.watchdog_ms;

  // ---- 1. E-Stop: highest priority, latched until released + explicit reset (FW-08) ----
  if (in.estop_pressed) estop_latched_ = true;
  if (estop_latched_) {
    if (!in.estop_pressed && in.reset_requested) {
      estop_latched_ = false;
      safe_clear_pending_ = false;
      v_out_ = 0.0f;
      setState(SafetyState::BOOT);  // re-enters IDLE below once the drivers are ready
    } else {
      setState(SafetyState::ESTOP);
      v_out_ = 0.0f;
      SafetyOutput out = halted(in.estop_pressed ? SafetyReason::ESTOP_PRESSED
                                                 : SafetyReason::ESTOP_WAIT_RESET);
      out.link_alive = link_alive;
      return out;
    }
  }

  // ---- 2. Boot ----
  if (state_ == SafetyState::BOOT) {
    if (!in.boot_complete) {
      SafetyOutput out = halted(SafetyReason::BOOTING);
      out.link_alive = link_alive;
      return out;
    }
    setState(SafetyState::IDLE);
  }
  if (link_alive) ever_linked_ = true;

  // ---- 3. SAFE: watchdog, critical battery, sensor fault ----
  const bool cmd_fresh = in.cmd_seen && (now - in.last_cmd_ms) <= c_.watchdog_ms;
  const bool cmd_nonzero =
      std::fabs(in.cmd_v) > c_.cmd_eps_mps || std::fabs(in.cmd_w) > c_.turn_eps_radps;
  SafetyReason cause = SafetyReason::NONE;
  if (in.sensor_fault) cause = SafetyReason::SENSOR_FAULT;
  else if (battery_.critical()) cause = SafetyReason::BATTERY_CRITICAL;
  else if (ever_linked_ && !link_alive) cause = SafetyReason::WATCHDOG_HEARTBEAT;
  else if (link_alive && in.cmd_seen && !cmd_fresh && cmd_nonzero) cause = SafetyReason::WATCHDOG_CMD;

  if (cause != SafetyReason::NONE) {
    safe_reason_ = cause;
    safe_clear_pending_ = false;
    setState(SafetyState::SAFE);
  } else if (state_ == SafetyState::SAFE) {
    if (!safe_clear_pending_) {
      safe_clear_pending_ = true;
      safe_clear_since_ms_ = now;
    }
    if (now - safe_clear_since_ms_ >= c_.safe_recovery_hold_ms) {
      safe_clear_pending_ = false;
      setState(SafetyState::IDLE);
    }
  }

  if (state_ == SafetyState::SAFE) {
    v_out_ = rampSpeed(v_out_, 0.0f, c_.safe_decel_mps2, c_.safe_decel_mps2, dt);
    SafetyOutput out = halted(safe_reason_);
    out.v = v_out_;
    out.w = v_out_ * curvature_;
    out.drive_enabled = v_out_ != 0.0f;  // keep regulating the ramp, then brake
    out.link_alive = link_alive;
    return out;
  }

  // ---- 4. Normal driving: ToF ring + speed caps ----
  float cmd_v = 0.0f, cmd_w = 0.0f;
  if (link_alive && cmd_fresh) {
    cmd_v = in.cmd_v;
    cmd_w = in.cmd_w;
  }
  if (std::fabs(cmd_v) > c_.cmd_eps_mps) curvature_ = cmd_w / cmd_v;

  // Watch both the way we are rolling now and the way we are asked to go (they differ while reversing
  // out of a blocked spot). Each only looks at the sensors facing its own direction (SF-02).
  const int dir_now = directionOf(v_out_, c_.cmd_eps_mps);
  const int dir_cmd = directionOf(cmd_v, c_.cmd_eps_mps);
  const TofEvaluation eval_now = evaluateTof(dir_now, cmd_w, in.tof, now, in.docking, c_);
  const TofEvaluation eval_cmd =
      dir_cmd == dir_now ? eval_now : evaluateTof(dir_cmd, cmd_w, in.tof, now, in.docking, c_);

  SafetyOutput out;
  merge(out.tof, eval_now);
  merge(out.tof, eval_cmd);

  const SpeedCap cap =
      speedCap(out.tof.slow, in.fork_height_mm, in.fork_height_valid, in.docking, c_);
  float target = clampf(cmd_v, -cap.cap_mps, cap.cap_mps);
  if (eval_cmd.stop) target = 0.0f;
  if (eval_now.stop) v_out_ = 0.0f;  // NFR-04: cut at once, no ramp
  v_out_ = rampSpeed(v_out_, target, c_.accel_mps2, c_.decel_mps2, dt);

  out.v = v_out_;
  out.w = v_out_ * curvature_;
  out.v_target = target;
  out.speed_cap = eval_cmd.stop ? 0.0f : cap.cap_mps;
  out.drive_enabled = v_out_ != 0.0f || target != 0.0f;
  out.lift_enabled = true;
  out.link_alive = link_alive;
  out.battery_low = battery_.low();

  const bool moving = dir_cmd != 0 || v_out_ != 0.0f;
  const bool stopped = eval_cmd.stop || eval_now.stop;
  const SafetyReason idle_reason = !ever_linked_ ? SafetyReason::NO_LINK
                                   : battery_.low() ? SafetyReason::BATTERY_LOW
                                                    : SafetyReason::NONE;
  if (!moving) {
    setState(SafetyState::IDLE);
    out.reason = idle_reason;
  } else if (stopped) {
    setState(SafetyState::STOP);
    out.reason = out.tof.stop_mask ? SafetyReason::TOF_STOP : SafetyReason::TOF_FAULT;
  } else if (cap.reason != SafetyReason::NONE) {
    setState(SafetyState::SLOW);
    out.reason = cap.reason;
  } else {
    setState(SafetyState::ACTIVE);
    out.reason = battery_.low() ? SafetyReason::BATTERY_LOW : SafetyReason::NONE;
  }
  out.state = state_;
  return out;
}

}  // namespace fl
