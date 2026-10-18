"""Mission executor: modes, server commands and the Behavior Tree of one move_pallet task — no ROS.

    task -> accepted -> to_pickup (lane by lane: request -> navigate -> release)
         -> picking (pre-lift, Dock, lift off, back out, travel height)
         -> to_drop (lanes) -> dropping (lift, drive in, lower, back out) -> done | failed (+ reason)

Robot I/O goes through `Capabilities` (implemented with Nav2 / Dock / Lift action clients in mission_node.py, with
fakes in the tests). Mode handling follows tools/mqtt_robot_client:
  pause/resume/reset/cancel/go_charge/mode, E-stop (retained) holds still, a new task replaces the current one.
Requirements: NV-09 (lane graph), NV-10 (recovery: wait + back up + re-plan, report after 3 failures),
SF-05 (server loss > 3 s: finish the step in progress, then stop and wait), NV-12 (loaded footprint).
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from . import bt
from .bt import Action, Context, Do, Handle, Retry, Sequence, Status, Wait
from .fleet_client_core import FleetClient, LaneRequest
from .lane_graph import LaneGraph

log = logging.getLogger("mission")

REQUIRED_TASK_FIELDS = ("task_id", "type", "pallet_id", "from", "to", "route")
REQUIRED_PLACE_FIELDS = ("location", "x", "y", "yaw", "height_mm")


class Capabilities(Protocol):
    """What the executor needs from the robot. Every motion returns a bt.Handle (asynchronous)."""

    def navigate(self, x: float, y: float, yaw: float) -> Handle: ...          # Nav2 NavigateToPose
    def dock(self, pallet_id: int, support_height_mm: float) -> Handle: ...    # forklift_rl Dock action
    def lift(self, height_mm: float) -> Handle: ...                            # Lift action
    def drive_straight(self, distance_m: float) -> Handle: ...                 # + DriveOnHeading / - BackUp
    def stop(self) -> None: ...                                                # cancel every running goal
    def set_loaded(self, loaded: bool) -> None: ...                            # footprint switch (NV-12)
    def reset_safety(self) -> None: ...                                        # forklift_safety Reset service
    def telemetry(self) -> dict: ...  # x, y, yaw, speed, battery_pct, fork_height_mm, safety_state


@dataclass
class MissionParams:
    robot_id: str
    home_node: str
    nav_attempts: int = 3            # NV-10
    recovery_wait_s: float = 2.0
    recovery_backup_m: float = 0.05
    dock_attempts: int = 2
    entry_offset_mm: float = 8.0     # fork height above the support surface to enter the pallet opening
    lift_off_mm: float = 20.0        # raise the pallet this much off its support
    travel_loaded_mm: float = 20.0   # fork height while driving with a pallet (SF-08: keep below 50 mm)
    travel_empty_mm: float = 0.0
    backout_m: float = 0.25          # reverse after picking (back to the spur node)
    place_m: float = 0.25            # drive in to place a pallet at the drop location
    require_reset_after_server_loss: bool = False  # True = tools/mqtt_robot_client behaviour (operator reset)


class MissionExecutor:
    def __init__(self, params: MissionParams, graph: LaneGraph, client: FleetClient, caps: Capabilities,
                 clock: Callable[[], float]) -> None:
        self.p = params
        self.g = graph
        self.c = client
        self.caps = caps
        self.clock = clock
        self.ctx = Context(clock)

        self.mode = "idle"            # idle busy charging paused manual safe error (estop is reported on top)
        self.paused_from = "idle"
        self.node: str | None = params.home_node if params.home_node in graph.nodes else None
        self.in_lane: tuple[str, str] | None = None
        self.carrying: int | None = None
        self.task: dict | None = None
        self.task_id: str | None = None
        self.phase = ""
        self.tree: bt.Node | None = None
        self.tree_kind = ""           # task | charge | cleanup
        self.pending_task: dict | None = None
        self.after_cleanup: Callable[[], None] | None = None
        self.inserted = False         # forks inside a pallet that is not lifted yet (cleanup must back out)
        self._halted = False
        self.events: list[tuple[str, str]] = []  # (code, message) local log for the node / tests

        client.on_task = self.on_task
        client.on_cmd = self.on_cmd
        client.on_reconnect = self._on_reconnect
        client.state_provider = self.state_fields

    # ================================================================== status for the server and ROS
    def hold_reason(self) -> str:
        if self.c.estop:
            return "fleet_estop"
        if self.mode in ("paused", "safe", "error", "manual"):
            return self.mode
        if self.c.server_lost():
            return "server_lost"
        return ""

    def reported_mode(self) -> str:
        return "estop" if self.c.estop else self.mode

    def step_active(self) -> bool:
        if self.tree is None:
            return False
        leaf = self.tree.running_leaf()
        return isinstance(leaf, Action) and leaf.is_motion and leaf.handle is not None

    def state_fields(self) -> dict:
        tel = self.caps.telemetry()
        mode = self.reported_mode()
        safety = tel.get("safety_state", "normal")
        if mode == "estop":
            safety = "estop"
        elif mode == "safe" and safety == "normal":
            safety = "safe"
        return {
            "x": round(float(tel.get("x", 0.0)), 3), "y": round(float(tel.get("y", 0.0)), 3),
            "yaw": round(float(tel.get("yaw", 0.0)), 3), "speed": round(float(tel.get("speed", 0.0)), 3),
            "battery_pct": round(min(100.0, max(0.0, float(tel.get("battery_pct", 0.0)))), 1),
            "mode": mode, "safety_state": safety, "fork_height_mm": round(float(tel.get("fork_height_mm", 0.0))),
            "carrying_pallet_id": self.carrying, "node": self.node, "task_id": self.task_id,
        }

    def status(self) -> dict:
        """Fields of forklift_interfaces/FleetStatus."""
        return {
            "robot_id": self.p.robot_id, "mode": self.reported_mode(), "server_connected": self.c.connected,
            "server_silence_s": self.c.server_silence(), "fleet_estop": self.c.estop,
            "step_active": self.step_active(), "hold": bool(self.hold_reason()), "hold_reason": self.hold_reason(),
            "node": self.node or "", "task_id": self.task_id or "", "phase": self.phase,
            "carrying_pallet_id": -1 if self.carrying is None else int(self.carrying),
        }

    def teleop_output(self) -> tuple[float, float, int]:
        if self.mode != "manual":
            return 0.0, 0.0, 0
        return self.c.teleop_command()

    def _event(self, type_: str, severity: str, code: str, message: str) -> None:
        self.events.append((code, message))
        log.info("event %s: %s", code, message)
        self.c.event(type_, severity, code, message)

    # ================================================================== main loop
    def tick(self) -> None:
        self.c.tick()
        if self.hold_reason() and self.hold_reason() != "server_lost":
            if not self._halted:
                self._halted = True
                if self.tree is not None:
                    self.tree.halt()
                self.caps.stop()
            return
        self._halted = False
        self.ctx.soft_hold = self.c.server_lost()  # SF-05: finish the running step, start nothing new
        if self.tree is None:
            return
        st = self.tree.tick(self.ctx)
        if st is Status.RUNNING:
            return
        kind, tree = self.tree_kind, self.tree
        self.tree, self.tree_kind = None, ""
        if kind == "task":
            self._task_finished(st is Status.SUCCESS)
        elif kind == "charge":
            if self.mode == "busy":
                self.mode = "charging" if st is Status.SUCCESS else "idle"
            if st is Status.FAILURE:
                self._event("error", "warning", "nav_failed", f"go_charge failed: {self.ctx.last_failure}")
        elif kind == "cleanup":
            then, self.after_cleanup = self.after_cleanup, None
            if self.mode == "busy":
                self.mode = "idle"
            if then:
                then()
        del tree

    def _on_reconnect(self, outage_s: float) -> None:
        if outage_s > self.c.cfg.server_loss_timeout_s:
            self._event("safety", "warning", "wifi_lost", f"server link lost for {outage_s:.1f} s (SF-05)")
            if self.p.require_reset_after_server_loss and self.mode not in ("manual", "error"):
                if self.mode != "safe":
                    self.paused_from = self.mode
                self.mode = "safe"

    # ================================================================== inbound task
    def on_task(self, task: dict) -> None:
        problem = self._validate(task)
        tid = str(task.get("task_id", "?"))
        if problem:
            log.warning("rejecting task %s: %s", tid, problem)
            self.c.task_status(tid, "failed", f"invalid task: {problem}")
            return
        if tid == self.task_id or (self.pending_task and self.pending_task.get("task_id") == tid):
            return  # QoS-1 redelivery
        if self.mode == "manual":
            self.c.task_status(tid, "failed", "robot is in manual mode")
            return
        if self.tree is not None:
            # the server only sends a task when it considers us free: drop what we were doing
            self.pending_task = task
            self._abort(then=self._start_pending)
            return
        self._start_task(task)

    @staticmethod
    def _validate(task: dict) -> str:
        for k in REQUIRED_TASK_FIELDS:
            if k not in task:
                return f"missing {k}"
        if task["type"] != "move_pallet":
            return f"unsupported type {task['type']!r}"
        for side in ("from", "to"):
            place = task[side]
            if not isinstance(place, dict) or any(k not in place for k in REQUIRED_PLACE_FIELDS):
                return f"incomplete {side}"
        if not isinstance(task["route"], list):
            return "route is not a list"
        return ""

    def _start_pending(self) -> None:
        task, self.pending_task = self.pending_task, None
        if task is not None:
            self._start_task(task)

    def _place_node(self, place: dict) -> str:
        node = place.get("node")
        if node in self.g.nodes:
            return node
        loc = self.g.locations.get(place.get("location", ""))
        if loc is not None:
            return loc.node
        return self.g.nearest_node(float(place["x"]), float(place["y"]))

    def _start_task(self, task: dict) -> None:
        tid = str(task["task_id"])
        pick, drop = self._place_node(task["from"]), self._place_node(task["to"])
        start = self.node or self._nearest_node()
        try:
            to_pick, to_drop = self.g.split_route(list(task.get("route") or []), start, pick, drop)
        except (ValueError, KeyError) as exc:
            self.c.task_status(tid, "failed", f"no route: {exc}")
            self._event("error", "warning", "nav_failed", f"task {tid}: no route ({exc})")
            return
        self.task, self.task_id, self.phase = task, tid, "accepted"
        if self.mode in ("idle", "charging"):
            self.mode = "busy"
        self.c.task_status(tid, "accepted")
        self.tree = self._task_tree(task, to_pick, to_drop)
        self.tree_kind = "task"
        log.info("task %s: %s -> %s, legs %s | %s", tid, pick, drop, to_pick, to_drop)

    def _task_finished(self, ok: bool) -> None:
        tid = self.task_id
        if not ok:
            code = self.ctx.last_failure_code or "task_failed"
            reason = self.ctx.last_failure or "failed"
            self.c.task_status(tid, "failed", f"{code}: {reason}")
            self._event("error", "warning", code, f"task {tid} failed: {reason}")
            self.phase = "failed"
            if code == "nav_failed" and self.in_lane is not None:
                self.mode = "error"  # stuck inside a lane after NV-10 recovery: needs the operator
            elif self.mode == "busy":
                self.mode = "idle"
        else:
            self.phase = "done"
            if self.mode == "busy":
                self.mode = "idle"
        self.task, self.task_id = None, None
        self.ctx.last_failure, self.ctx.last_failure_code = "", ""
        if not ok and self.inserted:
            # last pickup attempt failed with the forks still in the pallet: back out and lower them
            self.tree = Sequence(
                "back out after failure",
                self._motion("back out", lambda c: self.caps.drive_straight(-self.p.backout_m)),
                Do("not inserted", lambda c: setattr(self, "inserted", False)),
                self._lift("lower forks", self.p.travel_empty_mm))
            self.tree_kind = "cleanup"

    # ================================================================== commands (wh/fl/{id}/cmd)
    def on_cmd(self, body: dict) -> None:
        cmd = body.get("cmd")
        if cmd == "pause":
            if self.mode not in ("paused", "safe", "error"):
                self.paused_from, self.mode = self.mode, "paused"
        elif cmd == "resume":
            if self.mode == "paused":
                self.mode = self.paused_from
        elif cmd == "reset":
            # always forwarded: the ESP32 leaves a (hardware) ESTOP only on `safety_reset` (FW-08), whatever our mode
            self.caps.reset_safety()
            if self.mode in ("safe", "error", "paused"):
                self.mode = "busy" if self.tree is not None else "idle"
        elif cmd == "cancel":
            if self.task_id is not None and body.get("task_id") in (None, self.task_id):
                self.pending_task = None
                self._abort()
        elif cmd == "go_charge":
            self.pending_task = None
            if self.tree is not None:
                self._abort(then=self._start_charge)
            else:
                self._start_charge()
        elif cmd == "mode":
            if body.get("mode") == "manual":
                if self.tree is not None:
                    self.tree.halt()
                self.tree, self.tree_kind, self.task, self.task_id = None, "", None, None
                self.caps.stop()
                self.mode, self.node, self.in_lane = "manual", None, None  # the server releases our locks
            elif body.get("mode") == "auto" and self.mode == "manual":
                self.mode = "idle"
                self.node = self._nearest_node()

    def _nearest_node(self) -> str:
        tel = self.caps.telemetry()
        return self.g.nearest_node(float(tel.get("x", 0.0)), float(tel.get("y", 0.0)))

    def _abort(self, then: Callable[[], None] | None = None) -> None:
        """Stop the current tree and run a cleanup: finish the lane we are in (we hold its lock), back out of a
        pallet we have not lifted, lower empty forks. Then `then()` (next task / go charge)."""
        if self.tree is not None:
            self.tree.halt()
        self.caps.stop()
        if self.tree_kind == "task":
            log.warning("task %s cancelled", self.task_id)
            self.task, self.task_id, self.phase = None, None, ""
        steps: list[bt.Node] = []
        if self.in_lane is not None:
            a, b = self.in_lane
            steps += self._lane_steps(a, b, self.g.heading(a, b), request=False)
        if self.inserted:
            steps.append(self._motion("back out", lambda c: self.caps.drive_straight(-self.p.backout_m)))
            steps.append(Do("not inserted", lambda c: setattr(self, "inserted", False)))
        if self.carrying is None:
            steps.append(self._lift("lower forks", self.p.travel_empty_mm))
        self.tree = Sequence("cleanup", *steps)
        self.tree_kind = "cleanup"
        self.after_cleanup = then

    def _start_charge(self) -> None:
        start = self.node or self._nearest_node()
        home = self.p.home_node
        loc = self.g.charger_location(home)
        try:
            path = self.g.plan(start, home)
        except (ValueError, KeyError) as exc:
            self._event("error", "warning", "nav_failed", f"go_charge: no route ({exc})")
            return
        self.mode = "busy"
        self.tree = Sequence("go_charge", self._lanes(path, loc.yaw if loc else None))
        self.tree_kind = "charge"

    # ================================================================== tree building
    def _motion(self, name: str, start: Callable[[Context], Handle], code: str = "") -> Action:
        return Action(name, start, failure_code=code, is_motion=True)

    def _lift(self, name: str, height_mm: float) -> Action:
        h = max(0.0, float(height_mm))
        return Action(f"{name} ({h:.0f} mm)", lambda c: self.caps.lift(h), failure_code="lift_failed")

    def _report(self, status: str) -> Do:
        def fn(ctx: Context) -> None:
            self.phase = status
            self.c.task_status(self.task_id, status)
        return Do(f"report {status}", fn)

    def _lane_steps(self, a: str, b: str, yaw: float, request: bool = True) -> list[bt.Node]:
        x, y = self.g.xy(b)
        steps: list[bt.Node] = []
        if request:
            steps.append(Action(f"request {a}--{b}", lambda c: LaneRequest(self.c, a, b)))

        def enter(ctx: Context) -> None:
            self.in_lane, self.node = (a, b), None

        def arrive(ctx: Context) -> None:
            self.in_lane, self.node = None, b
            self.c.release_lane(a, b)

        steps.append(Do(f"enter {a}--{b}", enter))
        nav = self._motion(f"navigate {a}->{b}", lambda c: self.caps.navigate(x, y, yaw), code="nav_failed")
        recovery = Sequence("recovery", Wait("stop and wait", self.p.recovery_wait_s),
                            self._motion("back up", lambda c: self.caps.drive_straight(-self.p.recovery_backup_m)))

        def failed_attempt(ctx: Context, n: int) -> None:
            log.warning("lane %s--%s: navigation attempt %d/%d failed", a, b, n, self.p.nav_attempts)

        steps.append(Retry(f"lane {a}--{b}", nav, self.p.nav_attempts, recovery, failed_attempt))
        steps.append(Do(f"release {a}--{b}", arrive))
        return steps

    def _lanes(self, path: list[str], final_yaw: float | None) -> bt.Node:
        lanes = self.g.lanes(path)
        yaws = self.g.goal_yaws([lanes[0][0]] + [b for _, b in lanes], final_yaw) if lanes else []
        steps: list[bt.Node] = []
        for (a, b), yaw in zip(lanes, yaws):
            steps += self._lane_steps(a, b, yaw)
        return Sequence(f"lanes {'>'.join(path)}", *steps)

    def _task_tree(self, task: dict, to_pick: list[str], to_drop: list[str]) -> bt.Node:
        p = self.p
        src, dst = task["from"], task["to"]
        pallet = int(task["pallet_id"])
        h_src, h_dst = float(src["height_mm"]), float(dst["height_mm"])

        def set_inserted(v: bool):
            return Do(f"inserted={v}", lambda c: setattr(self, "inserted", v))

        def loaded(ctx: Context) -> None:
            self.carrying = pallet
            self.inserted = False
            self.caps.set_loaded(True)

        def unloaded(ctx: Context) -> None:
            self.carrying = None
            self.caps.set_loaded(False)

        pick_attempt = Sequence(
            "pick attempt",
            self._lift("pre-lift", h_src + p.entry_offset_mm),
            set_inserted(True),
            self._motion("dock", lambda c: self.caps.dock(pallet, h_src), code="pickup_failed"),
            self._lift("lift off", h_src + p.entry_offset_mm + p.lift_off_mm),
        )
        pick_recovery = Sequence(
            "pick recovery",
            self._lift("lower to entry", h_src + p.entry_offset_mm),
            self._motion("back off", lambda c: self.caps.drive_straight(-p.backout_m)),
            set_inserted(False),
        )

        def pick_failed(ctx: Context, n: int) -> None:
            ctx.last_failure_code = "pickup_failed"
            log.warning("pickup attempt %d/%d failed: %s", n, p.dock_attempts, ctx.last_failure)

        return Sequence(
            f"task {self.task_id}",
            self._report("to_pickup"),
            self._lanes(to_pick, float(src["yaw"])),
            self._report("picking"),
            Retry("pick", pick_attempt, p.dock_attempts, pick_recovery, pick_failed),
            Do("loaded", loaded),
            self._motion("back out", lambda c: self.caps.drive_straight(-p.backout_m)),
            self._lift("travel height", p.travel_loaded_mm),
            self._report("to_drop"),
            self._lanes(to_drop, float(dst["yaw"])),
            self._report("dropping"),
            self._lift("place height", h_dst + p.entry_offset_mm + p.lift_off_mm),
            self._motion("drive in", lambda c: self.caps.drive_straight(p.place_m), code="drop_failed"),
            self._lift("set down", h_dst),
            Do("unloaded", unloaded),
            self._motion("back out", lambda c: self.caps.drive_straight(-p.place_m), code="drop_failed"),
            self._lift("travel height", p.travel_empty_mm),
            self._report("done"),
        )


def angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi
