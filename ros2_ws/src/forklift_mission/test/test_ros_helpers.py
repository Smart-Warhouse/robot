import math

import pytest

from forklift_mission.fake_broker import FakeBroker, FakeTransport
from forklift_mission.fleet_client_core import FleetClient, FleetConfig
from forklift_mission.ros_helpers import (battery_percent, lift_teleop_target, quaternion_to_yaw, safety_event,
                                          yaw_to_quaternion)
from forklift_mission.testing import FakeClock
from forklift_mission.transport import ThreadSafeTransport


@pytest.mark.parametrize("yaw", [0.0, 1.0, -2.5, math.pi - 1e-6])
def test_quaternion_roundtrip(yaw):
    assert quaternion_to_yaw(*yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_battery_percent():
    assert battery_percent(0.5, 7.4) == 50.0
    assert battery_percent(float("nan"), 7.4) == pytest.approx(50.0)
    assert battery_percent(float("nan"), 9.0) == 100.0
    assert battery_percent(float("nan"), float("nan")) == 0.0


def test_safety_events_on_worsening_only():
    assert safety_event(0, 2, "tof_stop") == ("warning", "tof_stop", "safety normal -> stop: tof_stop")
    assert safety_event(0, 4, "fleet_estop")[0] == "critical"
    assert safety_event(2, 0, "") is None
    assert safety_event(1, 1, "tof_slow") is None


def test_lift_teleop_integration():
    assert lift_teleop_target(10, 1, 30, 0.1, 120) == pytest.approx(13)
    assert lift_teleop_target(1, -1, 30, 0.1, 120) == 0.0
    assert lift_teleop_target(119, 1, 30, 0.1, 120) == 120.0


def test_threadsafe_transport_defers_callbacks():
    broker = FakeBroker()
    ts = ThreadSafeTransport(FakeTransport(broker))
    client = FleetClient(FleetConfig(robot_id="fl3"), ts, clock=FakeClock())
    client.start()
    assert client.connected is False   # connect callback queued, not yet delivered
    assert ts.drain() == 1
    assert client.connected is True
    broker.inject("wh/fleet/estop", {"engaged": True, "seq": 1, "ts": "x"}, retain=True)
    broker.pump()
    assert client.estop is False
    ts.drain()
    assert client.estop is True


def test_paho_transport_configures_will_and_credentials():
    from forklift_mission.transport import Message, PahoTransport

    t = PahoTransport()
    t.configure("robot-fl1", "fl1", "pw", Message("wh/fl/fl1/state", '{"online": false}', 1, False))
    assert t._c is not None
    t.disconnect()
