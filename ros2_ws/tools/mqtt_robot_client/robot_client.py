"""Reference forklift fleet client — the robot side of docs/03-mqtt-contract.md, without ROS.

Pretends to be one real forklift: it drives along the lane graph from warehouse_map.yaml at a fixed speed,
asks the server for every lane before entering it and does the pick/drop as timed dwells. Use it to exercise
the server's MQTT bridge (app/fleet/mqtt_bridge.py) and as the template for the forklift_mission ROS node.

    python robot_client.py --id fl1 --password <fl1-password> [--host localhost] [--start CH1]

Contract points implemented:
  * Last Will {"online": false} on wh/fl/{id}/state (CM-03); state at 2 Hz with seq + ts (CM-04)
  * wh/fl/{id}/task → task_status accepted, to_pickup, picking, to_drop, dropping, done | failed
  * wh/traffic/{from}--{to}: request, wait for grant (re-request every 500 ms on deny), release on arrival
  * wh/fl/{id}/cmd pause/resume/cancel/go_charge/reset/mode, wh/fl/{id}/teleop in manual mode
  * retained wh/fleet/estop: hold still while engaged
  * losing the broker = stop and go to SAFE; an operator `reset` is needed to continue (FW-08)
"""

from __future__ import annotations

import argparse
import itertools
import json
import logging
import math
import signal
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt
import yaml

log = logging.getLogger("robot")
DEFAULT_MAP = Path(__file__).resolve().parents[3] / "backend" / "app" / "warehouse_map.yaml"
STATE_HZ = 2.0
STEP_S = 0.05
GRANT_RETRY_S = 0.5
CARRY_HEIGHT_MM = 20.0


class Cancelled(Exception):
    """The current mission was cancelled (server cmd or a newer task)."""


def utcnow() -> str:
    return datetime.now(timezone.utc).isoformat()


class Forklift:
    def __init__(self, args: argparse.Namespace) -> None:
        raw = yaml.safe_load(Path(args.map).read_text(encoding="utf-8"))
        self.nodes = {k: (float(v[0]), float(v[1])) for k, v in raw["nodes"].items()}
        self.adj: dict[str, list[str]] = {n: [] for n in self.nodes}
        for a, b in raw["edges"]:
            self.adj[a].append(b)
            self.adj[b].append(a)
        cfg = next((r for r in raw.get("robots", []) if r["id"] == args.id), {})
        self.id = args.id
        self.home = args.start or cfg.get("home") or next(iter(self.nodes))
        self.speed_mps = args.speed
        self.action_s = args.action_s
        self.fail_pick = args.fail_pick

        self.lock = threading.RLock()
        self.x, self.y = self.nodes[self.home]
        self.yaw = math.pi / 2
        self.node: str | None = self.home
        self.speed = 0.0
        self.battery = 100.0
        self.fork_mm = 0.0
        self.carrying: int | None = None
        self.mode = "idle"  # idle busy charging paused manual safe error (estop is reported on top)
        self.paused_from = "idle"
        self.task_id: str | None = None
        self.estop = False
        self.connected = threading.Event()
        self.stop = threading.Event()
        self.cancel = threading.Event()
        self.teleop = (0.0, 0.0, 0, 0.0)  # v, w, lift, monotonic time
        self.grants: dict[str, str] = {}  # segment -> last grant/deny answer for us
        self.grant_cv = threading.Condition(self.lock)
        self.mission: threading.Thread | None = None
        self.pending_task: dict | None = None
        self.seq = itertools.count(1)

        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"robot-{self.id}", clean_session=True)
        if args.username or args.password:
            self.c.username_pw_set(args.username or self.id, args.password)
        self.c.will_set(f"wh/fl/{self.id}/state", json.dumps({"online": False}), qos=1, retain=False)
        self.c.reconnect_delay_set(1, 5)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.on_message = self._on_message
        self.host, self.port = args.host, args.port

    # ------------------------------------------------------------------ MQTT plumbing
    def pub(self, topic: str, body: dict, qos: int = 1) -> None:
        self.c.publish(topic, json.dumps({**body, "seq": next(self.seq), "ts": utcnow()}), qos=qos)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            log.error("broker refused connection: %s", reason_code)
            return
        t = f"wh/fl/{self.id}"
        client.subscribe([(f"{t}/task", 1), (f"{t}/cmd", 1), (f"{t}/teleop", 0),
                          ("wh/traffic/+", 1), ("wh/fleet/estop", 1)])
        self.connected.set()
        log.info("connected to %s:%s as %s", self.host, self.port, self.id)
        self.publish_state()

    def _on_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        self.connected.clear()
        with self.lock:
            self.speed = 0.0
            if not self.stop.is_set() and self.mode not in ("safe", "error"):
                self.mode = "safe"  # FW-08: keep still until an operator resets us
        log.warning("lost the broker (%s) — SAFE stop", reason_code)

    def _on_message(self, client, userdata, msg) -> None:
        try:
            body = json.loads(msg.payload or b"{}")
        except json.JSONDecodeError:
            log.warning("bad json on %s", msg.topic)
            return
        try:
            self._dispatch(msg.topic, body)
        except Exception:
            log.exception("handling %s failed", msg.topic)

    def _dispatch(self, topic: str, body: dict) -> None:
        parts = topic.split("/")
        if topic == "wh/fleet/estop":
            with self.lock:
                self.estop = bool(body.get("engaged"))
                if self.estop:
                    self.speed = 0.0
            log.warning("fleet E-stop %s", "ENGAGED" if self.estop else "released")
        elif parts[:2] == ["wh", "traffic"] and len(parts) == 3:
            if body.get("robot_id") == self.id and body.get("action") in ("grant", "deny"):
                with self.grant_cv:
                    self.grants[parts[2]] = body["action"]
                    self.grant_cv.notify_all()
        elif parts[-1] == "task":
            self._on_task(body)
        elif parts[-1] == "cmd":
            self._on_cmd(body)
        elif parts[-1] == "teleop":
            with self.lock:
                self.teleop = (float(body.get("v", 0)), float(body.get("w", 0)), int(body.get("lift", 0)),
                               time.monotonic())

    # ------------------------------------------------------------------ inbound task / cmd
    def _on_task(self, task: dict) -> None:
        log.info("task %s: %s -> %s via %s", task.get("task_id"), task["from"]["location"], task["to"]["location"],
                 task.get("route"))
        with self.lock:
            if self.mission and self.mission.is_alive():
                self.cancel.set()  # the server only sends a new task when it considers us free
            self.pending_task = task
        threading.Thread(target=self._start_pending, daemon=True).start()

    def _start_pending(self) -> None:
        if self.mission and self.mission is not threading.current_thread():
            self.mission.join()
        with self.lock:
            task, self.pending_task = self.pending_task, None
            if task is None:
                return
            self.cancel.clear()
            self.task_id = task["task_id"]
            if self.mode == "idle":
                self.mode = "busy"
            self.mission = threading.Thread(target=self._run_task, args=(task,), daemon=True)
        self.pub(f"wh/fl/{self.id}/task_status", {"task_id": task["task_id"], "status": "accepted"})
        self.mission.start()

    def _on_cmd(self, body: dict) -> None:
        cmd = body.get("cmd")
        log.info("cmd %s", body)
        with self.lock:
            if cmd == "pause" and self.mode not in ("paused", "safe", "error"):
                self.paused_from, self.mode, self.speed = self.mode, "paused", 0.0
            elif cmd == "resume" and self.mode == "paused":
                self.mode = self.paused_from
            elif cmd == "reset" and self.mode in ("safe", "error", "paused"):
                self.mode = "busy" if self.task_id else "idle"
            elif cmd == "cancel":
                if body.get("task_id") in (None, self.task_id):
                    self.cancel.set()
            elif cmd == "go_charge":
                self.cancel.set()
                threading.Thread(target=self._go_charge, daemon=True).start()
            elif cmd == "mode":
                if body.get("mode") == "manual":
                    self.mode, self.speed, self.node = "manual", 0.0, None
                elif body.get("mode") == "auto" and self.mode == "manual":
                    self.mode = "idle"
                    self.node = self._nearest_node()
                    self.x, self.y = self.nodes[self.node]
        self.publish_state()

    def _go_charge(self) -> None:
        if self.mission:
            self.mission.join()
        with self.lock:
            self.mode = "busy"
            self.mission = threading.current_thread()
        try:
            self._drive_to(self.home)
            with self.lock:
                self.mode = "charging"
        except Cancelled:
            pass
        finally:
            with self.lock:
                if self.mode == "busy":
                    self.mode = "idle"

    # ------------------------------------------------------------------ mission
    def _run_task(self, task: dict) -> None:
        tid = task["task_id"]
        status = lambda s, **kw: self.pub(f"wh/fl/{self.id}/task_status", {"task_id": tid, "status": s, **kw})
        pick = task["from"].get("node") or self._nearest_node(task["from"]["x"], task["from"]["y"])
        drop = task["to"].get("node") or self._nearest_node(task["to"]["x"], task["to"]["y"])
        route = list(task.get("route") or [])
        try:
            if pick in route:
                to_pick, to_drop = route[:route.index(pick) + 1], route[route.index(pick):]
            else:
                to_pick, to_drop = self._plan(self.node, pick), self._plan(pick, drop)
            status("to_pickup")
            self._follow(to_pick)
            status("picking")
            self._dwell(self.action_s, task["from"].get("height_mm", 0), task["from"].get("yaw"))
            if self.fail_pick:
                self.fork_mm = 0.0
                raise RuntimeError("fork insertion failed (--fail-pick)")
            with self.lock:
                self.carrying, self.fork_mm = int(task["pallet_id"]), CARRY_HEIGHT_MM
            status("to_drop")
            self._follow(to_drop if to_drop and to_drop[-1] == drop else self._plan(self.node, drop))
            status("dropping")
            self._dwell(self.action_s, float(task["to"].get("height_mm", 0)) + CARRY_HEIGHT_MM, task["to"].get("yaw"))
            with self.lock:
                self.carrying, self.fork_mm = None, 0.0
            status("done")
            log.info("task %s done", tid)
        except Cancelled:
            log.warning("task %s cancelled", tid)
        except Exception as exc:
            log.error("task %s failed: %s", tid, exc)
            status("failed", reason=str(exc))
        finally:
            with self.lock:
                if self.task_id == tid:
                    self.task_id = None
                if self.mode == "busy":
                    self.mode = "idle"
            self.publish_state()

    def _follow(self, route: list[str]) -> None:
        for nxt in route:
            if nxt == self.node:
                continue
            if self.node is None or nxt not in self.adj[self.node]:
                raise RuntimeError(f"route step {self.node} -> {nxt} is not a lane")
            self._drive_lane(self.node, nxt)

    def _drive_to(self, goal: str) -> None:
        self._follow(self._plan(self.node, goal))

    def _drive_lane(self, a: str, b: str) -> None:
        seg = f"{a}--{b}"
        self._wait_grant(seg)
        (ax, ay), (bx, by) = self.nodes[a], self.nodes[b]
        length = math.hypot(bx - ax, by - ay)
        done = 0.0
        with self.lock:
            self.yaw = math.atan2(by - ay, bx - ax)
            self.node = None  # between nodes
        try:
            while done < length:
                self._hold()
                done = min(length, done + self.speed_mps * STEP_S)
                with self.lock:
                    f = done / length if length else 1.0
                    self.x, self.y, self.speed = ax + (bx - ax) * f, ay + (by - ay) * f, self.speed_mps
                    self.battery = max(0.0, self.battery - 0.002)
                time.sleep(STEP_S)
        finally:
            with self.lock:
                self.speed = 0.0
                if done >= length:
                    self.node = b
                else:
                    self.node = a  # cancelled mid-lane: snap back logically (sim only)
                    self.x, self.y = ax, ay
        if done >= length:
            self.pub(f"wh/traffic/{seg}", {"robot_id": self.id, "action": "release"})
        else:
            self.pub(f"wh/traffic/{b}--{a}", {"robot_id": self.id, "action": "release"})

    def _wait_grant(self, seg: str) -> None:
        while True:
            self._hold()
            with self.grant_cv:
                self.grants.pop(seg, None)
            if self.connected.is_set():
                self.pub(f"wh/traffic/{seg}", {"robot_id": self.id, "action": "request"})
            deadline = time.monotonic() + GRANT_RETRY_S
            with self.grant_cv:
                while seg not in self.grants and time.monotonic() < deadline:
                    self.grant_cv.wait(deadline - time.monotonic())
                answer = self.grants.pop(seg, None)
            if answer == "grant":
                return
            if answer == "deny":
                log.debug("lane %s denied — waiting", seg)
                time.sleep(max(0.0, deadline - time.monotonic()))

    def _dwell(self, seconds: float, fork_mm: float, yaw: float | None) -> None:
        with self.lock:
            self.fork_mm = float(fork_mm)
            if yaw is not None:
                self.yaw = float(yaw)
        left = seconds
        while left > 0:
            self._hold()
            time.sleep(STEP_S)
            left -= STEP_S

    def _hold(self) -> None:
        """Block while we must not move (E-stop, pause, SAFE, broker lost); raise if cancelled."""
        while True:
            if self.cancel.is_set() or self.stop.is_set():
                raise Cancelled()
            with self.lock:
                blocked = self.estop or self.mode in ("paused", "safe", "error", "manual") or not self.connected.is_set()
                if blocked:
                    self.speed = 0.0
            if not blocked:
                return
            time.sleep(STEP_S)

    # ------------------------------------------------------------------ geometry
    def _nearest_node(self, x: float | None = None, y: float | None = None) -> str:
        x = self.x if x is None else x
        y = self.y if y is None else y
        return min(self.nodes, key=lambda n: math.hypot(self.nodes[n][0] - x, self.nodes[n][1] - y))

    def _plan(self, start: str | None, goal: str) -> list[str]:
        """Plain BFS-by-distance (Dijkstra) fallback when the task route does not cover a leg."""
        import heapq

        start = start or self._nearest_node()
        dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
        while pq:
            d, n = heapq.heappop(pq)
            if n == goal:
                break
            for nb in self.adj[n]:
                nd = d + math.dist(self.nodes[n], self.nodes[nb])
                if nd < dist.get(nb, math.inf):
                    dist[nb], prev[nb] = nd, n
                    heapq.heappush(pq, (nd, nb))
        if goal not in dist:
            raise RuntimeError(f"no route {start} -> {goal}")
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1]

    # ------------------------------------------------------------------ state + manual drive
    def publish_state(self) -> None:
        if not self.connected.is_set():
            return
        with self.lock:
            mode = "estop" if self.estop else self.mode
            body = {
                "online": True, "x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 3),
                "speed": round(self.speed, 3), "battery_pct": round(self.battery, 1), "mode": mode,
                "safety_state": {"estop": "estop", "safe": "safe"}.get(mode, "normal"),
                "fork_height_mm": round(self.fork_mm), "carrying_pallet_id": self.carrying, "node": self.node,
                "task_id": self.task_id,
            }
        self.pub(f"wh/fl/{self.id}/state", body, qos=0)

    def _manual_step(self) -> None:
        with self.lock:
            if self.mode != "manual":
                return
            v, w, lift, t = self.teleop
            if time.monotonic() - t > 0.5 or self.estop:  # dead-man timeout
                v, w, lift = 0.0, 0.0, 0
            self.yaw = (self.yaw + w * STEP_S + math.pi) % (2 * math.pi) - math.pi
            self.x += v * math.cos(self.yaw) * STEP_S
            self.y += v * math.sin(self.yaw) * STEP_S
            self.fork_mm = min(120.0, max(0.0, self.fork_mm + lift * 30.0 * STEP_S))
            self.speed = v

    def run(self) -> None:
        self.c.connect_async(self.host, self.port, keepalive=5)
        self.c.loop_start()
        next_state = 0.0
        try:
            while not self.stop.is_set():
                self._manual_step()
                if time.monotonic() >= next_state:
                    next_state = time.monotonic() + 1.0 / STATE_HZ
                    self.publish_state()
                time.sleep(STEP_S)
        finally:
            self.cancel.set()
            if self.connected.is_set():  # clean exit: tell the server ourselves (the will fires only on a drop)
                self.c.publish(f"wh/fl/{self.id}/state", json.dumps({"online": False}), qos=1).wait_for_publish(2)
            self.c.disconnect()
            self.c.loop_stop()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", default="fl1", help="robot id = MQTT username (default fl1)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--username", help="MQTT username (default: the robot id when a password is given)")
    ap.add_argument("--password")
    ap.add_argument("--map", default=str(DEFAULT_MAP), help="warehouse_map.yaml with nodes/edges")
    ap.add_argument("--start", help="start node (default: the robot's home in the map)")
    ap.add_argument("--speed", type=float, default=0.5, help="driving speed, m/s")
    ap.add_argument("--action-s", type=float, default=2.0, help="seconds per pick / drop")
    ap.add_argument("--fail-pick", action="store_true", help="report every pickup as failed")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    bot = Forklift(args)
    signal.signal(signal.SIGINT, lambda *_: bot.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: bot.stop.set())
    bot.run()


if __name__ == "__main__":
    main()
