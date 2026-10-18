"""Safety supervisor core (requirements §9) — pure Python, no ROS.

Every motion command source goes through `SafetySupervisor.step()` before the ESP32 (SF-07, AI-06):

    nav (Nav2 controller / behaviors: cmd_vel_nav) | dock (forklift_rl: cmd_vel_dock) | teleop (gamepad over MQTT)
        -> source select (mode) -> stops (E-stop, pause, SF-05 server loss, ToF, person) -> speed caps
        (max speed, raised forks SF-08, docking SF-03, ToF slow SF-02, person slow SF-06)
        -> curvature-preserving clamp (Ackermann: the path stays the same when v is limited)
        -> acceleration ramp (hard stops bypass it: NFR-04 <= 100 ms) -> cmd_vel

This is the Jetson layer; the ESP32 enforces ToF / watchdog / raised-fork limits again on its own (FW-07, SF-08).
State values match forklift_interfaces/SafetyState and the MQTT `safety_state` enum.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

NORMAL, SLOW, STOP, SAFE, ESTOP = range(5)
STATE_NAMES = ("normal", "slow", "stop", "safe", "estop")

# ESP32 `safety_state` (std_msgs/UInt8) exactly as code/firmware publishes it (docs/12-firmware.md §3):
#   byte = state << 4 | reason; states 0 BOOT 1 IDLE 2 ACTIVE 3 SLOW 4 STOP 5 SAFE 6 ESTOP.
ESP_BOOT, ESP_IDLE, ESP_ACTIVE, ESP_SLOW, ESP_STOP, ESP_SAFE, ESP_ESTOP = range(7)
ESP_STATE_MAP = {ESP_BOOT: STOP, ESP_IDLE: NORMAL, ESP_ACTIVE: NORMAL, ESP_SLOW: SLOW, ESP_STOP: STOP,
                 ESP_SAFE: SAFE, ESP_ESTOP: ESTOP}
# firmware reason -> event code of docs/02-api-contract.md §5 ("" = not an event)
ESP_REASON_NAMES = ("none", "booting", "no_link", "tof_slow", "tof_stop", "tof_fault", "fork_raised", "docking",
                    "watchdog_heartbeat", "watchdog_cmd", "battery_low", "battery_critical", "estop_pressed",
                    "estop_wait_reset", "sensor_fault")
ESP_REASONS = {0: "", 1: "", 2: "", 3: "tof_slow", 4: "tof_stop", 5: "tof_stop", 6: "", 7: "",
               8: "watchdog_timeout", 9: "watchdog_timeout", 10: "battery_low", 11: "battery_critical",
               12: "estop_pressed", 13: "estop_pressed", 14: "watchdog_timeout"}
ESP_CAP_ONLY = (6, 7)  # SLOW because of raised forks / docking: a speed cap, not a hazard

SOURCES = ("nav", "dock", "teleop")


def decode_esp32(byte: int) -> tuple[int, str]:
    """(supervisor state, event reason) of the raw ESP32 safety byte."""
    st, why = (int(byte) >> 4) & 0x0F, int(byte) & 0x0F
    state = ESP_STATE_MAP.get(st, STOP)
    if state == SLOW and why in ESP_CAP_ONLY:
        state = NORMAL
    return state, ESP_REASONS.get(why, "")


def encode_esp32(state: int, reason: int = 0) -> int:
    return (state & 0x0F) << 4 | (reason & 0x0F)


@dataclass
class SafetyParams:
    max_speed: float = 0.30            # NFR-03
    max_reverse_speed: float = 0.15
    max_omega: float = 1.0             # rad/s
    max_accel: float = 0.5             # m/s^2 speeding up
    max_decel: float = 1.0             # m/s^2 normal slowing down (hard stops are immediate)
    raised_fork_mm: float = 50.0       # SF-08
    raised_speed: float = 0.10
    docking_speed: float = 0.10        # SF-03, AI-06
    tof_names: list[str] = field(default_factory=lambda: ["front", "front_left", "front_right", "left", "right",
                                                          "rear"])
    tof_forward: list[str] = field(default_factory=lambda: ["front", "front_left", "front_right"])
    tof_reverse: list[str] = field(default_factory=lambda: ["rear"])
    tof_side: list[str] = field(default_factory=lambda: ["left", "right"])
    docking_ignored_tof: list[str] = field(default_factory=lambda: ["front"])  # = firmware TOF_MASK_DOCKING_IGNORE
    tof_slow_m: float = 0.40           # SF-02
    tof_stop_m: float = 0.15
    tof_slow_factor: float = 0.30
    side_stop_m: float = 0.05
    tof_timeout_s: float = 0.3
    person_slow_m: float = 1.0         # SF-06
    person_stop_m: float = 0.5
    person_speed: float = 0.10
    person_timeout_s: float = 0.5
    person_detector_required: bool = False  # True once YOLO runs: a silent detector then limits the speed
    cmd_timeout_s: float = 0.5
    fleet_status_timeout_s: float = 1.0
    server_loss_timeout_s: float = 3.0  # SF-05
    lift_max_mm: float = 120.0

    @classmethod
    def from_dict(cls, d: dict) -> "SafetyParams":
        p = cls()
        for k, v in d.items():
            if hasattr(p, k):
                cur = getattr(p, k)
                setattr(p, k, list(v) if isinstance(cur, list) else type(cur)(v))
        return p


@dataclass
class Output:
    v: float
    w: float
    lift_mm: float | None
    state: int
    reason: str
    source: str
    speed_limit: float
    docking: bool
    forks_raised: bool
    min_tof_m: float

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state]


class SafetySupervisor:
    def __init__(self, params: SafetyParams | None = None) -> None:
        self.p = params or SafetyParams()
        self.cmd: dict[str, tuple[float, float, float]] = {}       # source -> (v, w, t)
        self.lift_cmd: dict[str, tuple[float, float]] = {}        # source -> (mm, t)
        self._lift_sent: tuple[str, float, float] | None = None   # (source, mm, t) already forwarded
        self.tof: tuple[list[float], float] | None = None
        self.fork_mm = math.inf  # unknown until the first fork_state: capped like raised forks
        self.fleet: dict | None = None
        self.fleet_t = -math.inf
        self.esp = (NORMAL, "", 0)
        self.person: tuple[float, float] | None = None
        self.v_out = 0.0
        self.w_out = 0.0
        self.latched_safe = ""   # reason; cleared by reset()

    # ------------------------------------------------------------------ inputs
    def set_cmd(self, source: str, v: float, w: float, t: float) -> None:
        assert source in SOURCES, source
        self.cmd[source] = (_finite(v), _finite(w), t)

    def set_lift(self, source: str, height_mm: float, t: float) -> None:
        self.lift_cmd[source] = (_finite(height_mm), t)

    def set_tof(self, distances: list[float], t: float) -> None:
        # NaN = sensor fault (firmware) -> 0 m: a faulted sensor in the direction of travel stops (TOF_FAULT_STOPS)
        self.tof = ([0.0 if not _isnum(d) or math.isnan(d) else (float(d) if d > 0 else math.inf) for d in distances], t)

    def set_fork(self, height_mm: float) -> None:
        self.fork_mm = _finite(height_mm, math.inf)  # unknown height = assume raised (SF-08, like the ESP32)

    def set_fleet(self, t: float, mode: str, fleet_estop: bool, server_connected: bool, server_silence_s: float,
                  step_active: bool) -> None:
        self.fleet = {"mode": mode, "estop": fleet_estop, "connected": server_connected,
                      "silence": server_silence_s, "step_active": step_active}
        self.fleet_t = t

    def set_esp32(self, byte: int) -> None:
        st, reason = decode_esp32(byte)
        self.esp = (st, reason, int(byte))

    def set_person(self, distance_m: float, t: float) -> None:
        self.person = (_finite(distance_m, math.inf), t)

    def reset(self) -> bool:
        """Operator reset (FW-08). The node also publishes `safety_reset` (std_msgs/Empty) so the ESP32 leaves
        ESTOP once the button is released. Clears a supervisor latch if one was set."""
        self.latched_safe = ""
        return True

    # ------------------------------------------------------------------ helpers
    def _tof(self, now: float) -> dict[str, float] | None:
        if self.tof is None or now - self.tof[1] > self.p.tof_timeout_s:
            return None
        vals, _ = self.tof
        return {n: (vals[i] if i < len(vals) else math.inf) for i, n in enumerate(self.p.tof_names)}

    def _fresh(self, source: str, now: float) -> tuple[float, float] | None:
        c = self.cmd.get(source)
        if c is None or now - c[2] > self.p.cmd_timeout_s:
            return None
        return c[0], c[1]

    def _select(self, mode: str, now: float) -> tuple[str, float, float]:
        if mode == "manual":
            order = ("teleop",)
        else:
            order = ("dock", "nav")
        for s in order:
            c = self._fresh(s, now)
            if c is not None:
                return s, c[0], c[1]
        return "none", 0.0, 0.0

    # ------------------------------------------------------------------ main
    def step(self, now: float, dt: float) -> Output:
        p = self.p
        fleet = self.fleet if now - self.fleet_t <= p.fleet_status_timeout_s else None
        mode = fleet["mode"] if fleet else "unknown"
        source, v, w = self._select(mode, now)
        docking = source == "dock"
        raised = self.fork_mm > p.raised_fork_mm

        state, reason = NORMAL, ""

        def worse(s: int, r: str) -> None:
            nonlocal state, reason
            if s > state:
                state, reason = s, r

        # ---- hard stops (no ramp)
        esp_state, esp_reason, esp_raw = self.esp
        if fleet and fleet["estop"]:
            worse(ESTOP, "fleet_estop")
        if esp_state == ESTOP:
            worse(ESTOP, esp_reason or "estop_pressed")
        if self.latched_safe:
            worse(SAFE, self.latched_safe)
        if esp_state in (SLOW, STOP, SAFE):
            worse(esp_state, esp_reason)
        if fleet is None:
            worse(STOP, "watchdog_timeout")       # mission node silent: we do not know the mode
        elif mode in ("paused",):
            worse(STOP, "paused")
        elif mode in ("safe", "error", "estop"):
            worse(SAFE if mode != "estop" else ESTOP, mode if mode != "estop" else "fleet_estop")
        elif (mode != "manual" and not fleet["connected"] and fleet["silence"] > p.server_loss_timeout_s
              and not fleet["step_active"]):
            worse(STOP, "wifi_lost")              # SF-05: the step in progress may finish, nothing new

        # ---- speed caps
        limit = p.max_speed if v >= 0 else p.max_reverse_speed
        if raised:
            limit = min(limit, p.raised_speed)
        if docking:
            limit = min(limit, p.docking_speed)
        if esp_state == SLOW:
            limit = min(limit, p.max_speed * p.tof_slow_factor)
        tof = self._tof(now)
        min_tof = math.inf
        if tof is None:
            if source != "none":
                limit = min(limit, p.max_speed * p.tof_slow_factor)
                worse(SLOW, "tof_slow")
        else:
            ahead = p.tof_forward if v > 0 else (p.tof_reverse if v < 0 else [])
            if docking:
                ahead = [n for n in ahead if n not in p.docking_ignored_tof]
            if ahead:
                min_tof = min(tof[n] for n in ahead)
                if min_tof < p.tof_stop_m:
                    worse(STOP, "tof_stop")
                elif min_tof < p.tof_slow_m:
                    limit = min(limit, p.max_speed * p.tof_slow_factor)
                    worse(SLOW, "tof_slow")
            if abs(w) > 1e-3 and any(tof[n] < p.side_stop_m for n in p.tof_side):
                worse(STOP, "tof_stop")
        if self.person is not None and now - self.person[1] <= p.person_timeout_s:
            d = self.person[0]
            if d < p.person_stop_m and v != 0.0:
                worse(STOP, "person_detected")
            elif d < p.person_slow_m:
                limit = min(limit, p.person_speed)
                worse(SLOW, "person_detected")
        elif p.person_detector_required and source != "none":
            limit = min(limit, p.person_speed)
            worse(SLOW, "person_detected")

        # ---- output
        if state >= STOP or source == "none":
            self.v_out, self.w_out = 0.0, 0.0
            v_out, w_out = 0.0, 0.0
        else:
            v_t, w_t = clamp_curvature(v, w, limit, p.max_omega)
            v_out = ramp(self.v_out, v_t, p.max_accel * dt, p.max_decel * dt)
            if abs(v_t) > 1e-6:
                w_out = w_t * (v_out / v_t)  # keep the curvature during the ramp
            else:
                w_out = w_t
            self.v_out, self.w_out = v_out, w_out

        lift = self._lift(mode, docking, state, now)
        return Output(v_out, w_out, lift, state, reason, source, 0.0 if state >= STOP else limit, docking, raised,
                      min_tof)

    def _lift(self, mode: str, docking: bool, state: int, now: float) -> float | None:
        """Forward the newest lift set-point of the allowed source (once); none while E-stopped / SAFE."""
        if state >= SAFE:
            return None
        allowed = ("teleop",) if mode == "manual" else (("dock", "mission") if docking else ("mission", "dock"))
        best = None
        for s in allowed:
            c = self.lift_cmd.get(s)
            if c is not None and now - c[1] <= self.p.cmd_timeout_s * 4 and (best is None or c[1] > best[2]):
                best = (s, c[0], c[1])
        if best is None or best == self._lift_sent:
            return None
        self._lift_sent = best
        return min(max(best[1], 0.0), self.p.lift_max_mm)


def clamp_curvature(v: float, w: float, v_max: float, w_max: float) -> tuple[float, float]:
    """Limit |v| <= v_max and |w| <= w_max while keeping the path curvature w/v (Ackermann)."""
    v_max = max(0.0, v_max)
    if abs(v) < 1e-6:
        return 0.0, max(-w_max, min(w_max, w))
    k = w / v
    v2 = max(-v_max, min(v_max, v))
    w2 = k * v2
    if abs(w2) > w_max:
        w2 = math.copysign(w_max, w2)
        v2 = w2 / k
    return v2, w2


def ramp(current: float, target: float, accel_step: float, decel_step: float) -> float:
    """Rate-limit a speed: speeding up by <= accel_step, slowing down (towards 0 / reversing) by <= decel_step."""
    if current * target < 0:  # direction change: brake to zero first
        return current - math.copysign(min(abs(current), decel_step), current)
    step = accel_step if abs(target) > abs(current) else decel_step
    delta = max(-step, min(step, target - current))
    return current + delta


def _isnum(x) -> bool:
    try:
        float(x)
    except (TypeError, ValueError):
        return False
    return True


def _finite(x, default: float = 0.0) -> float:
    try:
        f = float(x)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default
