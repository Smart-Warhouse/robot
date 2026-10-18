"""Robot side of the robot <-> server MQTT contract (docs/03-mqtt-contract.md, docs/schemas/*.json) — no ROS.

Ported from the proven reference client `code/ros2_ws/tools/mqtt_robot_client/robot_client.py`, restructured as a
single-threaded, tick-driven core with an injectable `Transport` and clock so it can be unit-tested against
`fake_broker.FakeBroker`:

  * Last Will `{"online": false}` on `wh/fl/{id}/state`, QoS 1, not retained (CM-03); the same message is sent on
    a clean shutdown (the will only fires on a drop).
  * every outbound message carries `seq` (per-sender counter) and `ts` (UTC ISO-8601) (CM-04).
  * `wh/fl/{id}/state` at `state_hz` (2 Hz), QoS 0.
  * `wh/fl/{id}/task` / `cmd` / `teleop` handed to the executor; `task_status` and `event` QoS 1.
  * `wh/traffic/<from>--<to>`: `request`, wait for *our* `grant` (answers for another robot_id are ignored),
    re-request every 500 ms (on `deny` or no answer), `release` on arrival.
  * retained `wh/fleet/estop`: `estop` is True while `engaged`.
  * QoS-1 reports produced while the broker is unreachable (task_status, events, lane releases) are queued and
    flushed on reconnect; state and lane requests are not (they would be stale).
"""

from __future__ import annotations

import itertools
import json
import logging
import math
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone

from .lane_graph import segment_id
from .transport import Message, Transport

log = logging.getLogger("fleet_client")

TASK_STATUSES = ("accepted", "to_pickup", "picking", "to_drop", "dropping", "done", "failed")
MODES = ("idle", "busy", "charging", "paused", "manual", "safe", "estop", "error")
SAFETY_STATES = ("normal", "slow", "stop", "safe", "estop")
EVENT_TYPES = ("safety", "error", "info")
SEVERITIES = ("info", "warning", "critical")
CMDS = ("pause", "resume", "cancel", "go_charge", "reset", "mode")


def utcnow() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class FleetConfig:
    robot_id: str
    host: str = "localhost"
    port: int = 1883
    username: str | None = None       # default: the robot id (one MQTT user per robot, CM-01)
    password: str | None = None
    keepalive_s: int = 2               # broker loss detected in ~1.5 x keepalive (SF-05 budget: 3 s)
    state_hz: float = 2.0
    grant_retry_s: float = 0.5
    server_loss_timeout_s: float = 3.0  # SF-05
    teleop_timeout_s: float = 0.5       # dead-man for manual drive
    outbox_max: int = 200

    @property
    def base(self) -> str:
        return f"wh/fl/{self.robot_id}"


class FleetClient:
    def __init__(self, cfg: FleetConfig, transport: Transport, clock: Callable[[], float] = time.monotonic,
                 utc: Callable[[], str] = utcnow) -> None:
        self.cfg = cfg
        self.t = transport
        self.clock = clock
        self.utc = utc
        self._seq = itertools.count(1)
        self.connected = False
        self.ever_connected = False
        self.disconnected_at: float = clock()
        self.last_outage_s = 0.0          # length of the last broker outage (set on reconnect)
        self.estop = False
        self.estop_known = False
        self.estop_by: str | None = None
        self.grants: dict[str, str] = {}  # segment -> "grant" | "deny" (answers addressed to us)
        self.outbox: deque[Message] = deque(maxlen=cfg.outbox_max)
        self.teleop = (0.0, 0.0, 0, -math.inf)  # v, w, lift, clock time
        self._next_state = 0.0
        # executor hooks
        self.on_task: Callable[[dict], None] = lambda body: None
        self.on_cmd: Callable[[dict], None] = lambda body: None
        self.on_estop: Callable[[bool], None] = lambda engaged: None
        self.on_reconnect: Callable[[float], None] = lambda outage_s: None
        self.state_provider: Callable[[], dict] | None = None
        transport.on_connect = self._on_connect
        transport.on_disconnect = self._on_disconnect
        transport.on_message = self._on_message

    # ------------------------------------------------------------------ lifecycle
    def will(self) -> Message:
        return Message(f"{self.cfg.base}/state", json.dumps({"online": False}), qos=1, retain=False)

    def start(self) -> None:
        user = self.cfg.username or (self.cfg.robot_id if self.cfg.password else None)
        self.t.configure(f"robot-{self.cfg.robot_id}", user, self.cfg.password, self.will())
        self.t.connect(self.cfg.host, self.cfg.port, self.cfg.keepalive_s)

    def stop(self) -> None:
        """Clean shutdown: tell the server ourselves (the broker only sends the will on a drop)."""
        if self.connected:
            self.t.publish(self.will())
        self.t.disconnect()
        self.connected = False

    def _on_connect(self) -> None:
        outage = self.clock() - self.disconnected_at if self.ever_connected else 0.0
        self.connected = True
        self.ever_connected = True
        b = self.cfg.base
        self.t.subscribe([(f"{b}/task", 1), (f"{b}/cmd", 1), (f"{b}/teleop", 0),
                          ("wh/traffic/+", 1), ("wh/fleet/estop", 1)])
        while self.outbox:
            self.t.publish(self.outbox.popleft())
        self.last_outage_s = outage
        self._next_state = 0.0  # publish a state at once (the server marks us online on the first one)
        log.info("connected as %s (outage %.1f s)", self.cfg.robot_id, outage)
        self.on_reconnect(outage)

    def _on_disconnect(self) -> None:
        if self.connected:
            self.disconnected_at = self.clock()
        self.connected = False
        log.warning("broker connection lost")

    # ------------------------------------------------------------------ link health (SF-05)
    def server_silence(self) -> float:
        return 0.0 if self.connected else max(0.0, self.clock() - self.disconnected_at)

    def server_lost(self) -> bool:
        return self.server_silence() > self.cfg.server_loss_timeout_s

    # ------------------------------------------------------------------ inbound
    def _on_message(self, topic: str, payload: bytes) -> None:
        try:
            body = json.loads(payload or b"{}")
        except (json.JSONDecodeError, UnicodeDecodeError):
            log.warning("bad json on %s", topic)
            return
        if not isinstance(body, dict):
            log.warning("payload on %s is not an object", topic)
            return
        try:
            self._dispatch(topic, body)
        except Exception:  # one bad message must not kill the client
            log.exception("handling %s failed", topic)

    def _dispatch(self, topic: str, body: dict) -> None:
        parts = topic.split("/")
        if topic == "wh/fleet/estop":
            engaged = bool(body.get("engaged"))
            changed = engaged != self.estop or not self.estop_known
            self.estop, self.estop_known, self.estop_by = engaged, True, body.get("by")
            if changed:
                log.warning("fleet E-stop %s", "ENGAGED" if engaged else "released")
                self.on_estop(engaged)
        elif parts[:2] == ["wh", "traffic"] and len(parts) == 3:
            if body.get("robot_id") == self.cfg.robot_id and body.get("action") in ("grant", "deny"):
                self.grants[parts[2]] = body["action"]
        elif parts[:2] == ["wh", "fl"] and len(parts) == 4 and parts[2] == self.cfg.robot_id:
            kind = parts[3]
            if kind == "task":
                self.on_task(body)
            elif kind == "cmd":
                if body.get("cmd") in CMDS:
                    self.on_cmd(body)
                else:
                    log.warning("unknown cmd %r", body.get("cmd"))
            elif kind == "teleop":
                v = _clamp(_num(body.get("v")), -0.3, 0.3)
                w = _clamp(_num(body.get("w")), -1.0, 1.0)
                lift = int(body.get("lift", 0)) if body.get("lift") in (-1, 0, 1) else 0
                self.teleop = (v, w, lift, self.clock())

    def teleop_command(self) -> tuple[float, float, int]:
        """Manual drive set-point with a dead-man timeout; zero while the fleet E-stop is engaged."""
        v, w, lift, t = self.teleop
        if self.estop or self.clock() - t > self.cfg.teleop_timeout_s:
            return 0.0, 0.0, 0
        return v, w, lift

    # ------------------------------------------------------------------ outbound
    def _send(self, topic: str, body: dict, qos: int, queue_if_offline: bool = False) -> bool:
        msg = Message(topic, json.dumps({**body, "seq": next(self._seq), "ts": self.utc()}), qos=qos)
        if self.connected and self.t.publish(msg):
            return True
        if queue_if_offline:
            self.outbox.append(msg)
        return False

    def publish_state(self, fields: dict) -> bool:
        if not self.connected:
            return False
        return self._send(f"{self.cfg.base}/state", {"online": True, **fields}, qos=0)

    def task_status(self, task_id: str, status: str, reason: str | None = None) -> None:
        assert status in TASK_STATUSES, status
        body = {"task_id": task_id, "status": status}
        if reason:
            body["reason"] = reason
        self._send(f"{self.cfg.base}/task_status", body, qos=1, queue_if_offline=True)

    def event(self, type_: str, severity: str, code: str, message: str, **extra) -> None:
        assert type_ in EVENT_TYPES and severity in SEVERITIES
        self._send(f"{self.cfg.base}/event",
                   {"type": type_, "severity": severity, "code": code, "message": message, **extra},
                   qos=1, queue_if_offline=True)

    # ------------------------------------------------------------------ traffic (SV-03, SF-10)
    def request_lane(self, a: str, b: str) -> bool:
        seg = segment_id(a, b)
        self.grants.pop(seg, None)
        return self._send(f"wh/traffic/{seg}", {"robot_id": self.cfg.robot_id, "action": "request"}, qos=1)

    def lane_answer(self, a: str, b: str) -> str | None:
        return self.grants.get(segment_id(a, b))

    def release_lane(self, a: str, b: str) -> None:
        seg = segment_id(a, b)
        self.grants.pop(seg, None)
        self._send(f"wh/traffic/{seg}", {"robot_id": self.cfg.robot_id, "action": "release"}, qos=1,
                   queue_if_offline=True)

    # ------------------------------------------------------------------ periodic
    def tick(self) -> None:
        now = self.clock()
        if self.connected and self.state_provider is not None and now >= self._next_state:
            self._next_state = now + 1.0 / self.cfg.state_hz
            self.publish_state(self.state_provider())


class LaneRequest:
    """Handle for one lane request: re-requests every `grant_retry_s` until our `grant` arrives (docs/03).

    Never fails on its own — a denied lane just waits (the server re-routes deadlocks, SV-03). While the broker is
    unreachable nothing is sent and the handle stays RUNNING (SF-05).
    """

    def __init__(self, client: FleetClient, a: str, b: str) -> None:
        from .bt import Status

        self._S = Status
        self.c, self.a, self.b = client, a, b
        self.message = ""
        self.sent_at: float | None = None
        self.requests = 0
        self._send()

    def _send(self) -> None:
        if self.c.connected:
            self.c.request_lane(self.a, self.b)
            self.sent_at = self.c.clock()
            self.requests += 1
        else:
            self.sent_at = None

    def status(self):
        ans = self.c.lane_answer(self.a, self.b)
        if ans == "grant":
            return self._S.SUCCESS
        now = self.c.clock()
        if self.sent_at is None or now - self.sent_at >= self.c.cfg.grant_retry_s:
            self._send()
        return self._S.RUNNING

    def cancel(self) -> None:
        pass


def _num(v) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    return f if math.isfinite(f) else 0.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
