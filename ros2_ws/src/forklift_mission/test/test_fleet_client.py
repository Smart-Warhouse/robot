"""FleetClient against the in-process FakeBroker: docs/03-mqtt-contract.md, message by message."""

import json

import pytest

from contract import validate
from forklift_mission.bt import Status
from forklift_mission.fake_broker import FakeBroker, FakeTransport
from forklift_mission.fleet_client_core import FleetClient, FleetConfig, LaneRequest
from forklift_mission.testing import FakeClock, FakeFleetServer
from forklift_mission.transport import topic_matches


@pytest.fixture
def env():
    broker = FakeBroker()
    clock = FakeClock()
    server = FakeFleetServer(broker)
    tr = FakeTransport(broker)
    client = FleetClient(FleetConfig(robot_id="fl1", password="pw"), tr, clock=clock)
    client.state_provider = lambda: {"x": 1.0, "y": 3.8, "yaw": 1.571, "speed": 0.0, "battery_pct": 90.0,
                                     "mode": "idle", "safety_state": "normal", "fork_height_mm": 0,
                                     "carrying_pallet_id": None, "node": "CH1", "task_id": None}
    client.start()
    broker.pump()
    return broker, clock, server, tr, client


def robot_msgs(broker, flt="#"):
    return [(m.topic, json.loads(m.payload)) for cid, m in broker.log if cid == "robot-fl1" and topic_matches(flt, m.topic)]


def test_topic_matching():
    assert topic_matches("wh/traffic/+", "wh/traffic/A21--R1-E1")
    assert not topic_matches("wh/traffic/+", "wh/traffic/a/b")
    assert topic_matches("wh/#", "wh/fl/fl1/state")
    assert not topic_matches("wh/fl/+/state", "wh/fl/fl1/task")


def test_connect_will_credentials_and_subscriptions(env):
    broker, clock, server, tr, client = env
    assert tr.client_id == "robot-fl1" and tr.username == "fl1" and tr.password == "pw"
    assert tr.will.topic == "wh/fl/fl1/state" and json.loads(tr.will.payload) == {"online": False}
    assert tr.will.qos == 1 and tr.will.retain is False
    assert tr.subs == {"wh/fl/fl1/task", "wh/fl/fl1/cmd", "wh/fl/fl1/teleop", "wh/traffic/+", "wh/fleet/estop"}
    assert tr.keepalive <= 3  # SF-05: the broker loss must be noticed within the 3 s budget


def test_state_at_2hz_with_seq_and_ts(env):
    broker, clock, server, tr, client = env
    for _ in range(40):  # 4 s at 10 Hz ticks
        client.tick()
        clock.advance(0.1)
    broker.pump()
    states = server.states["fl1"]
    assert 8 <= len(states) <= 9
    seqs = [s["seq"] for s in states]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    for s in states:
        assert s["online"] is True
        validate("wh/fl/fl1/state", s)
    assert all(m.qos == 0 for cid, m in broker.log if m.topic == "wh/fl/fl1/state" and cid == "robot-fl1")


def test_seq_is_one_counter_for_all_topics(env):
    broker, clock, server, tr, client = env
    client.tick()
    client.task_status("T-1", "accepted")
    client.event("info", "info", "robot_online", "hello")
    client.request_lane("CH1", "J4")
    seqs = [b["seq"] for _, b in robot_msgs(broker)]
    assert seqs == list(range(1, len(seqs) + 1))
    for topic, body in robot_msgs(broker):
        validate(topic, body)


def test_last_will_on_drop_and_clean_offline_on_stop(env):
    broker, clock, server, tr, client = env
    client.tick()
    broker.pump()
    broker.drop(tr)  # Wi-Fi gone: the broker sends the will
    broker.pump()
    assert server.states["fl1"][-1] == {"online": False}
    broker.restore(tr)
    broker.pump()
    client.stop()     # clean exit: the client sends {"online": false} itself
    broker.pump()
    assert server.states["fl1"][-1] == {"online": False}
    assert tr.connected is False


def test_traffic_request_grant_release(env):
    broker, clock, server, tr, client = env
    h = LaneRequest(client, "A21", "R1-E1")
    assert h.status() is Status.RUNNING
    broker.pump()   # server sees the request and answers
    broker.pump()   # robot receives the grant
    assert h.status() is Status.SUCCESS
    assert server.locks == {"edge:A21--R1-E1": "fl1", "node:R1-E1": "fl1"}
    client.release_lane("A21", "R1-E1")
    broker.pump()
    assert "edge:A21--R1-E1" not in server.locks
    msgs = robot_msgs(broker, "wh/traffic/+")
    assert [(t, b["action"]) for t, b in msgs] == [("wh/traffic/A21--R1-E1", "request"),
                                                    ("wh/traffic/A21--R1-E1", "release")]
    for t, b in msgs:
        validate(t, b)
        assert b["robot_id"] == "fl1"


def test_deny_rerequests_every_500ms(env):
    broker, clock, server, tr, client = env
    server.deny_first = 2
    h = LaneRequest(client, "J1", "J2")
    granted_at = None
    for i in range(30):
        broker.pump()
        if h.status() is Status.SUCCESS:
            granted_at = i
            break
        clock.advance(0.1)
    assert granted_at is not None
    reqs = [r for r in server.requests if r == ("fl1", "J1--J2")]
    assert len(reqs) == 3
    assert 10 <= granted_at <= 12  # two denials x 500 ms


def test_answers_for_other_robots_are_ignored(env):
    broker, clock, server, tr, client = env
    h = LaneRequest(client, "J1", "J2")
    broker.inject("wh/traffic/J1--J2", {"robot_id": "fl2", "action": "grant", "seq": 1, "ts": "x"})
    # the fake server also answers our request; drop its answer to isolate the foreign grant
    broker.queue.clear()
    broker.inject("wh/traffic/J1--J2", {"robot_id": "fl2", "action": "grant", "seq": 2, "ts": "x"})
    broker.pump()
    assert h.status() is Status.RUNNING


def test_retained_estop_is_learned_on_connect():
    broker = FakeBroker()
    broker.inject("wh/fleet/estop", {"engaged": True, "by": "admin", "seq": 5, "ts": "2026-10-18T12:00:00Z"},
                  retain=True)
    seen = []
    client = FleetClient(FleetConfig(robot_id="fl2"), FakeTransport(broker), clock=FakeClock())
    client.on_estop = seen.append
    client.start()
    broker.pump()
    assert client.estop is True and seen == [True]
    broker.inject("wh/fleet/estop", {"engaged": False, "by": "admin", "seq": 6, "ts": "x"}, retain=True)
    broker.pump()
    assert client.estop is False and seen == [True, False]


def test_task_cmd_teleop_routing(env):
    broker, clock, server, tr, client = env
    tasks, cmds = [], []
    client.on_task, client.on_cmd = tasks.append, cmds.append
    server.send("wh/fl/fl1/task", {"task_id": "T-1", "type": "move_pallet"})
    server.send("wh/fl/fl2/task", {"task_id": "T-2", "type": "move_pallet"})  # not subscribed / not ours
    server.send("wh/fl/fl1/cmd", {"cmd": "pause"})
    server.send("wh/fl/fl1/cmd", {"cmd": "selfdestruct"})
    server.send("wh/fl/fl1/teleop", {"v": 0.9, "w": -3, "lift": 1})
    broker.pump()
    assert [t["task_id"] for t in tasks] == ["T-1"]
    assert [c["cmd"] for c in cmds] == ["pause"]
    assert client.teleop_command() == (0.3, -1.0, 1)  # clamped to teleop.schema.json ranges
    clock.advance(0.6)
    assert client.teleop_command() == (0.0, 0.0, 0)   # dead-man timeout


def test_bad_payloads_do_not_crash(env):
    broker, clock, server, tr, client = env
    tr.on_message("wh/fl/fl1/cmd", b"{not json")
    tr.on_message("wh/fl/fl1/cmd", b"[1,2]")
    tr.on_message("wh/fl/fl1/teleop", b'{"v": "fast", "w": null, "lift": 7}')
    assert client.teleop_command() == (0.0, 0.0, 0)


def test_reports_queued_while_offline_and_flushed(env):
    broker, clock, server, tr, client = env
    broker.drop(tr)
    clock.advance(1.0)
    assert client.server_silence() == pytest.approx(1.0) and not client.server_lost()
    clock.advance(2.5)
    assert client.server_lost()  # SF-05: > 3 s
    client.task_status("T-9", "to_drop")
    client.release_lane("J1", "J2")
    assert client.request_lane("J2", "J3") is False  # requests are not queued
    outages = []
    client.on_reconnect = outages.append
    broker.restore(tr)
    broker.pump()
    assert outages == [pytest.approx(3.5)]
    assert server.statuses("T-9") == ["to_drop"]
    assert not client.server_lost()


@pytest.mark.skipif(not __import__("os").environ.get("MQTT_TEST_HOST"),
                    reason="set MQTT_TEST_HOST (and MQTT_TEST_USER / MQTT_TEST_PASSWORD) to test against a real broker")
def test_paho_transport_against_a_real_broker():
    import os
    import time

    from forklift_mission.transport import PahoTransport, ThreadSafeTransport

    ts = ThreadSafeTransport(PahoTransport())
    cfg = FleetConfig(robot_id=os.environ.get("MQTT_TEST_USER", "fl1"), host=os.environ["MQTT_TEST_HOST"],
                      password=os.environ.get("MQTT_TEST_PASSWORD"))
    client = FleetClient(cfg, ts)
    client.state_provider = lambda: {"mode": "idle", "node": None}
    client.start()
    t0 = time.monotonic()
    while not client.connected and time.monotonic() - t0 < 5:
        ts.drain()
        time.sleep(0.05)
    assert client.connected
    for _ in range(5):
        ts.drain()
        client.tick()
        time.sleep(0.5)
    client.stop()
