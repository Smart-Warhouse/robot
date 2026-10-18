"""Mission executor + fleet client + fake fleet server over the fake broker: whole tasks, failures, commands."""

from collections import deque

import pytest

from contract import validate
from forklift_mission.fake_broker import FakeBroker, FakeTransport
from forklift_mission.fleet_client_core import FleetClient, FleetConfig
from forklift_mission.lane_graph import LaneGraph
from forklift_mission.mission_core import MissionExecutor, MissionParams
from forklift_mission.testing import FakeCapabilities, FakeClock, FakeFleetServer, make_task

PHASES = ["accepted", "to_pickup", "picking", "to_drop", "dropping", "done"]


class World:
    def __init__(self, graph, **params):
        self.g = graph
        self.broker = FakeBroker()
        self.clock = FakeClock()
        self.server = FakeFleetServer(self.broker)
        self.tr = FakeTransport(self.broker)
        self.client = FleetClient(FleetConfig(robot_id="fl1"), self.tr, clock=self.clock)
        x, y = graph.xy("CH1")
        self.caps = FakeCapabilities(x, y, 1.5708)
        self.ex = MissionExecutor(MissionParams(robot_id="fl1", home_node="CH1", **params), graph, self.client,
                                  self.caps, self.clock)
        self.client.start()
        self.broker.pump()

    def step(self, n=1, dt=0.1):
        for _ in range(n):
            self.broker.pump()
            self.ex.tick()
            self.broker.pump()
            self.clock.advance(dt)

    def run_until(self, cond, limit=3000):
        for _ in range(limit):
            if cond():
                return True
            self.step()
        return cond()

    def send_task(self, task):
        self.server.send("wh/fl/fl1/task", task)
        self.broker.pump()

    def cmd(self, **body):
        self.server.send("wh/fl/fl1/cmd", body)
        self.broker.pump()

    def robot_messages(self):
        return [(m.topic, m.payload) for cid, m in self.broker.log if cid == "robot-fl1"]


@pytest.fixture(scope="module")
def graph(warehouse_map_path):
    return LaneGraph.load(warehouse_map_path)


@pytest.fixture
def w(graph):
    return World(graph)


def done_or_failed(w, tid):
    return lambda: any(s in ("done", "failed") for s in w.server.statuses(tid))


def test_full_task_follows_the_contract(w):
    task = make_task(w.g, "T-0042", "R1-E1-L2", "DOCK-OUT-1", "CH1")
    w.send_task(task)
    assert w.run_until(done_or_failed(w, "T-0042"))
    assert w.server.statuses("T-0042") == PHASES
    w.step(5)
    # every lane of the route was requested, in order, and released after arrival
    route = task["route"]
    lanes = [f"{a}--{b}" for a, b in zip(route, route[1:])]
    assert [seg for rid, seg in w.server.requests] == lanes
    assert w.server.locks == {"node:DO1": "fl1"}  # only the node it stands on
    # pick: pre-lift to 60 + 8 mm, dock, lift off +20 mm, back out, travel height; drop mirrors it
    k = w.caps.calls
    i = k.index(("dock", 7, 60.0))
    assert k[i - 1] == ("lift", 68.0) and k[i + 1] == ("lift", 88.0)
    assert ("loaded", True) in k and ("loaded", False) in k
    assert k[-3:] == [("loaded", False), ("straight", -0.25), ("lift", 0.0)]
    # the last navigate goal is the drop node, facing the location yaw
    nav = [c for c in k if c[0] == "navigate"]
    assert nav[-1] == ("navigate", 5.65, 1.6, 0.0)
    assert w.ex.mode == "idle" and w.ex.carrying is None and w.ex.task_id is None and w.ex.node == "DO1"
    for topic, payload in w.robot_messages():
        import json
        validate(topic, json.loads(payload))


def test_state_reports_task_and_load(w):
    w.send_task(make_task(w.g, "T-1", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    assert w.run_until(lambda: "to_drop" in w.server.statuses("T-1"))
    w.step(6)
    s = w.server.states["fl1"][-1]
    assert s["mode"] == "busy" and s["carrying_pallet_id"] == 7 and s["task_id"] == "T-1"
    assert s["online"] is True and s["battery_pct"] == 87.5


def test_nav_failure_three_times_reports_failed(w):
    w.caps.fail["navigate"] = deque([True, False, False, False])  # first lane ok, second lane fails 3x
    w.send_task(make_task(w.g, "T-2", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    assert w.run_until(done_or_failed(w, "T-2"))
    st = [s for s in w.server.task_status if s["task_id"] == "T-2"]
    assert st[-1]["status"] == "failed" and st[-1]["reason"].startswith("nav_failed")
    assert [c[0] for c in w.caps.calls].count("navigate") == 4
    assert w.caps.calls.count(("straight", -0.05)) == 2  # NV-10 recovery: back up between attempts
    assert any(e["code"] == "nav_failed" for e in w.server.events)
    assert w.ex.mode == "error"  # stuck in a lane: an operator reset is needed
    w.step(5)
    assert w.server.states["fl1"][-1]["mode"] == "error"
    w.cmd(cmd="reset")
    w.step()
    assert w.ex.mode == "idle" and w.caps.resets == 1


def test_nav_recovers_after_two_failures(w):
    w.caps.fail["navigate"] = deque([False, False])
    w.send_task(make_task(w.g, "T-3", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    assert w.run_until(done_or_failed(w, "T-3"))
    assert w.server.statuses("T-3")[-1] == "done"


def test_pickup_failure_backs_out(w):
    w.caps.fail["dock"] = deque([False, False])
    w.send_task(make_task(w.g, "T-4", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    assert w.run_until(done_or_failed(w, "T-4"))
    st = [s for s in w.server.task_status if s["task_id"] == "T-4"]
    assert st[-1]["status"] == "failed" and st[-1]["reason"].startswith("pickup_failed")
    assert "to_drop" not in w.server.statuses("T-4")
    w.step(30)
    assert w.caps.kinds().count("dock") == 2
    assert w.caps.calls[-2:] == [("straight", -0.25), ("lift", 0.0)]  # backed out of the pallet, forks down
    assert w.ex.mode == "idle" and w.ex.inserted is False and w.ex.carrying is None


def test_pause_resume_restarts_the_running_lane(w):
    w.send_task(make_task(w.g, "T-5", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.run_until(lambda: w.ex.step_active())
    navs = w.caps.kinds().count("navigate")
    w.cmd(cmd="pause")
    w.step(20)
    assert w.ex.mode == "paused" and w.caps.stops >= 1
    assert w.caps.kinds().count("navigate") == navs  # nothing new while paused
    assert w.server.states["fl1"][-1]["mode"] == "paused"
    w.cmd(cmd="resume")
    assert w.run_until(done_or_failed(w, "T-5"))
    assert w.server.statuses("T-5")[-1] == "done"
    assert w.caps.kinds().count("navigate") == len(w.g.lanes(make_task(w.g, "x", "R1-W1-L1", "DOCK-OUT-1", "CH1")["route"])) + 1


def test_fleet_estop_holds_until_released(w):
    w.send_task(make_task(w.g, "T-6", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.run_until(lambda: w.ex.step_active())
    w.server.send("wh/fleet/estop", {"engaged": True, "by": "admin"}, retain=True)
    w.step(20)
    assert w.ex.status()["hold_reason"] == "fleet_estop"
    s = w.server.states["fl1"][-1]
    assert s["mode"] == "estop" and s["safety_state"] == "estop"
    calls = len(w.caps.calls)
    w.step(20)
    assert len(w.caps.calls) == calls
    w.server.send("wh/fleet/estop", {"engaged": False, "by": "admin"}, retain=True)
    assert w.run_until(done_or_failed(w, "T-6"))
    assert w.server.statuses("T-6")[-1] == "done"


def test_server_loss_finishes_step_then_waits(graph):
    w = World(graph)
    w.caps.ticks = 40  # a lane takes ~4 s
    w.send_task(make_task(w.g, "T-7", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.run_until(lambda: w.ex.step_active())
    lane_requests = len(w.server.requests)
    w.broker.drop(w.tr)
    w.step(35)     # 3.5 s: lost > 3 s while the lane is still being driven
    assert w.client.server_lost()
    assert w.ex.ctx.soft_hold
    w.step(40)     # the lane in progress completes, nothing new starts
    assert not w.ex.step_active()
    navs = w.caps.kinds().count("navigate")
    w.step(50)
    assert w.caps.kinds().count("navigate") == navs
    assert w.ex.status()["hold_reason"] == "server_lost"
    w.broker.restore(w.tr)
    assert w.run_until(done_or_failed(w, "T-7"))
    assert w.server.statuses("T-7")[-1] == "done"
    assert len(w.server.requests) > lane_requests
    assert any(e["code"] == "wifi_lost" for e in w.server.events)


def test_server_loss_with_reset_policy_goes_safe(graph):
    w = World(graph, require_reset_after_server_loss=True)
    w.send_task(make_task(w.g, "T-8", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.run_until(lambda: w.ex.step_active())
    w.broker.drop(w.tr)
    w.step(40)
    w.broker.restore(w.tr)
    w.step(20)
    assert w.ex.mode == "safe"
    assert "done" not in w.server.statuses("T-8")
    w.cmd(cmd="reset")
    assert w.run_until(done_or_failed(w, "T-8"))
    assert w.server.statuses("T-8")[-1] == "done"


def test_cancel_mid_lane_finishes_the_lane(w):
    w.send_task(make_task(w.g, "T-9", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.run_until(lambda: w.ex.in_lane is not None and w.ex.step_active())
    lane = w.ex.in_lane
    w.cmd(cmd="cancel", task_id="T-other")   # not ours: ignored
    w.step()
    assert w.ex.task_id == "T-9"
    w.cmd(cmd="cancel", task_id="T-9")
    w.run_until(lambda: w.ex.tree is None)
    assert w.ex.task_id is None and w.ex.mode == "idle" and w.ex.node == lane[1]
    assert f"node:{lane[1]}" in w.server.locks and f"edge:{'--'.join(sorted(lane))}" not in w.server.locks
    assert w.server.statuses("T-9")[-1] != "done"


def test_new_task_replaces_current_and_duplicates_are_ignored(w):
    t1 = make_task(w.g, "T-10", "R1-W1-L1", "DOCK-OUT-1", "CH1")
    w.send_task(t1)
    w.send_task(t1)  # QoS-1 redelivery
    w.step(3)
    assert w.server.statuses("T-10").count("accepted") == 1
    w.run_until(lambda: w.ex.step_active())
    w.send_task(make_task(w.g, "T-11", "R2-W1-L1", "DOCK-OUT-2", w.g.nearest_node(*w.caps.pose[:2])))
    assert w.run_until(done_or_failed(w, "T-11"))
    assert w.server.statuses("T-11")[-1] == "done"
    assert "done" not in w.server.statuses("T-10")


def test_invalid_and_unsupported_tasks_fail_fast(w):
    w.send_task({"task_id": "T-12", "type": "move_pallet"})
    bad = make_task(w.g, "T-13", "R1-W1-L1", "DOCK-OUT-1", "CH1")
    bad["type"] = "teleport"
    w.send_task(bad)
    w.step()
    for tid in ("T-12", "T-13"):
        st = [s for s in w.server.task_status if s["task_id"] == tid]
        assert st and st[-1]["status"] == "failed" and st[-1]["reason"].startswith("invalid task")
    assert w.ex.mode == "idle"


def test_task_without_route_is_planned_locally(w):
    w.send_task(make_task(w.g, "T-14", "R2-E2-L2", "DOCK-IN-1", "CH1", with_route=False))
    assert w.run_until(done_or_failed(w, "T-14"))
    assert w.server.statuses("T-14") == PHASES
    assert w.ex.node == "DI1"


def test_go_charge_drives_home(w):
    w.send_task(make_task(w.g, "T-15", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    assert w.run_until(done_or_failed(w, "T-15"))
    w.cmd(cmd="go_charge")
    assert w.run_until(lambda: w.ex.mode == "charging")
    assert w.ex.node == "CH1"
    nav = [c for c in w.caps.calls if c[0] == "navigate"]
    assert nav[-1] == ("navigate", 1.0, 3.8, pytest.approx(1.5708, abs=1e-3))
    w.send_task(make_task(w.g, "T-16", "R1-W1-L1", "DOCK-IN-1", "CH1"))
    w.step(3)
    assert w.ex.mode == "busy"


def test_manual_mode_teleop_and_back_to_auto(w):
    w.cmd(cmd="mode", mode="manual")
    w.step()
    assert w.ex.mode == "manual" and w.ex.node is None
    w.server.send("wh/fl/fl1/teleop", {"v": 0.2, "w": 0.5, "lift": -1})
    w.broker.pump()
    assert w.ex.teleop_output() == (0.2, 0.5, -1)
    w.send_task(make_task(w.g, "T-17", "R1-W1-L1", "DOCK-OUT-1", "CH1"))
    w.step()
    assert w.server.statuses("T-17") == ["failed"]
    w.caps.pose = [2.95, 0.65, 0.0]
    w.cmd(cmd="mode", mode="auto")
    w.step()
    assert w.ex.mode == "idle" and w.ex.node == "J2"
    assert w.ex.teleop_output() == (0.0, 0.0, 0)


def test_status_fields_for_fleet_status_msg(w):
    st = w.ex.status()
    assert set(st) == {"robot_id", "mode", "server_connected", "server_silence_s", "fleet_estop", "step_active",
                       "hold", "hold_reason", "node", "task_id", "phase", "carrying_pallet_id"}
    assert st["robot_id"] == "fl1" and st["carrying_pallet_id"] == -1 and st["node"] == "CH1"
