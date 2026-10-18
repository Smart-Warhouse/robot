"""Test doubles for the mission core: a fake clock, scripted robot capabilities and a fake fleet server that
speaks the server side of docs/03 (lane locks like code/backend/app/fleet/traffic.py). Used by the host tests
and handy for dry runs without hardware."""

from __future__ import annotations

import json
import math
from collections import deque

from .bt import Status
from .fake_broker import FakeBroker, FakeTransport
from .transport import Message


class FakeClock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class TimedHandle:
    """Finishes after `ticks` status() polls with the given result."""

    def __init__(self, ticks: int, ok: bool, message: str = "", on_success=None) -> None:
        self.left = ticks
        self.ok = ok
        self.message = message
        self.cancelled = False
        self._on_success = on_success
        self._done: Status | None = None

    def status(self) -> Status:
        if self._done is not None:
            return self._done
        if self.cancelled:
            return Status.FAILURE
        if self.left > 0:
            self.left -= 1
            return Status.RUNNING
        self._done = Status.SUCCESS if self.ok else Status.FAILURE
        if self.ok and self._on_success:
            self._on_success()
        return self._done

    def cancel(self) -> None:
        self.cancelled = True


class FakeCapabilities:
    """Robot that reaches every goal after `ticks` polls. `fail[kind]` = list of booleans consumed per call
    (False = that call fails)."""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, ticks: int = 2) -> None:
        self.pose = [x, y, yaw]
        self.ticks = ticks
        self.fork = 0.0
        self.loaded = False
        self.calls: list[tuple] = []
        self.fail: dict[str, deque] = {}
        self.handles: list[TimedHandle] = []
        self.stops = 0
        self.resets = 0
        self.safety_state = "normal"

    def _result(self, kind: str) -> bool:
        q = self.fail.get(kind)
        return q.popleft() if q else True

    def _handle(self, kind: str, on_success=None) -> TimedHandle:
        ok = self._result(kind)
        h = TimedHandle(self.ticks, ok, "" if ok else f"{kind} failed", on_success)
        self.handles.append(h)
        return h

    def navigate(self, x, y, yaw):
        self.calls.append(("navigate", round(x, 3), round(y, 3), round(yaw, 3)))

        def arrive():
            self.pose = [x, y, yaw]
        return self._handle("navigate", arrive)

    def dock(self, pallet_id, support_height_mm):
        self.calls.append(("dock", pallet_id, support_height_mm))
        return self._handle("dock")

    def lift(self, height_mm):
        self.calls.append(("lift", round(height_mm, 1)))

        def done():
            self.fork = height_mm
        return self._handle("lift", done)

    def drive_straight(self, distance_m):
        self.calls.append(("straight", round(distance_m, 3)))
        return self._handle("straight")

    def stop(self):
        self.stops += 1
        for h in self.handles:
            if h._done is None:
                h.cancel()

    def set_loaded(self, loaded):
        self.loaded = loaded
        self.calls.append(("loaded", loaded))

    def reset_safety(self):
        self.resets += 1

    def telemetry(self):
        return {"x": self.pose[0], "y": self.pose[1], "yaw": self.pose[2], "speed": 0.0, "battery_pct": 87.5,
                "fork_height_mm": self.fork, "safety_state": self.safety_state}

    def kinds(self) -> list[str]:
        return [c[0] for c in self.calls]


class FakeFleetServer:
    """Server side of docs/03 over the FakeBroker: grants a lane + far node to one robot at a time."""

    def __init__(self, broker: FakeBroker, deny_first: int = 0) -> None:
        self.b = broker
        self.t = FakeTransport(broker)
        self.t.configure("fleet-server", "server", "x", Message("server/none", "{}"))
        self.t.on_message = self._on
        self.t.connect("localhost", 1883, 5)
        self.t.subscribe([("wh/fl/+/state", 0), ("wh/fl/+/task_status", 1), ("wh/fl/+/event", 1),
                          ("wh/traffic/+", 1)])
        self.locks: dict[str, str] = {}
        self.seq = 0
        self.deny_first = deny_first
        self.requests: list[tuple[str, str]] = []
        self.states: dict[str, list[dict]] = {}
        self.task_status: list[dict] = []
        self.events: list[dict] = []

    def send(self, topic: str, body: dict, retain: bool = False) -> None:
        self.seq += 1
        self.t.publish(Message(topic, json.dumps({**body, "seq": self.seq, "ts": "2026-10-18T12:00:00+00:00"}), 1,
                               retain))

    def _res(self, a: str, b: str) -> list[str]:
        lo, hi = sorted((a, b))
        return [f"edge:{lo}--{hi}", f"node:{b}"]

    def _on(self, topic: str, payload: bytes) -> None:
        body = json.loads(payload)
        parts = topic.split("/")
        if parts[1] == "traffic":
            seg, action, rid = parts[2], body.get("action"), body.get("robot_id")
            a, _, b = seg.partition("--")
            if action == "request":
                self.requests.append((rid, seg))
                res = self._res(a, b)
                free = all(self.locks.get(r) in (None, rid) for r in res)
                if self.deny_first > 0:
                    self.deny_first -= 1
                    free = False
                if free:
                    for r in res:
                        self.locks[r] = rid
                self.send(topic, {"robot_id": rid, "action": "grant" if free else "deny"})
            elif action == "release":
                lo, hi = sorted((a, b))
                for r in (f"edge:{lo}--{hi}", f"node:{a}"):
                    if self.locks.get(r) == rid:
                        del self.locks[r]
        elif parts[3] == "state":
            self.states.setdefault(parts[2], []).append(body)
        elif parts[3] == "task_status":
            self.task_status.append(body)
        elif parts[3] == "event":
            self.events.append(body)

    def statuses(self, task_id: str) -> list[str]:
        return [s["status"] for s in self.task_status if s.get("task_id") == task_id]


def make_task(graph, task_id: str, pick_loc: str, drop_loc: str, start_node: str, pallet_id: int = 7,
              with_route: bool = True) -> dict:
    """A task message as code/backend/app/fleet/manager.py builds it (from/to carry the extension `node`)."""
    f, t = graph.locations[pick_loc], graph.locations[drop_loc]
    route = (graph.plan(start_node, f.node) + graph.plan(f.node, t.node)[1:]) if with_route else []

    def place(loc):
        x, y = graph.xy(loc.node)
        return {"location": loc.code, "node": loc.node, "x": x, "y": y, "yaw": loc.yaw, "height_mm": loc.height_mm}

    return {"task_id": task_id, "type": "move_pallet", "pallet_id": pallet_id, "from": place(f), "to": place(t),
            "route": route, "priority": 2, "issued_at": "2026-10-18T12:00:00+00:00", "seq": 1,
            "ts": "2026-10-18T12:00:00+00:00"}


def angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
