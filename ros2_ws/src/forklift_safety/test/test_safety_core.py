import math

import pytest

from forklift_safety.safety_core import (ESP_ESTOP, ESP_SAFE, ESP_SLOW, ESTOP, NORMAL, SAFE, SLOW, STOP,
                                         SafetyParams, SafetySupervisor, clamp_curvature, decode_esp32,
                                         encode_esp32, ramp)

DT = 0.05
CLEAR = [2.0] * 6


def sup(**kw) -> SafetySupervisor:
    s = SafetySupervisor(SafetyParams(**kw))
    s.set_fork(0.0)
    return s


def feed(s, t, mode="busy", estop=False, connected=True, silence=0.0, step_active=True, tof=CLEAR, fork=None):
    s.set_fleet(t, mode, estop, connected, silence, step_active)
    if fork is not None:
        s.set_fork(fork)
    if tof is not None:
        s.set_tof(tof, t)


def run(s, t0, seconds, source="nav", v=0.3, w=0.0, **fleet):
    out, t = None, t0
    for _ in range(int(round(seconds / DT))):
        feed(s, t, **fleet)
        s.set_cmd(source, v, w, t)
        out = s.step(t, DT)
        t += DT
    return out, t


def test_nominal_speed_cap_and_accel_ramp():
    s = sup()
    feed(s, 0.0)
    s.set_cmd("nav", 0.5, 0.0, 0.0)
    o = s.step(0.0, DT)
    assert o.v == pytest.approx(0.5 * DT)  # 0.5 m/s^2 ramp
    o, _ = run(s, DT, 2.0, v=0.5)
    assert o.v == pytest.approx(0.30) and o.state == NORMAL and o.source == "nav"


def test_curvature_is_preserved_when_clamping():
    v, w = clamp_curvature(0.4, 0.8, 0.1, 1.0)
    assert v == pytest.approx(0.1) and w == pytest.approx(0.2)
    v, w = clamp_curvature(0.2, 2.0, 0.3, 1.0)
    assert w == pytest.approx(1.0) and v == pytest.approx(0.1)
    assert clamp_curvature(0.0, 0.5, 0.3, 1.0) == (0.0, 0.5)


def test_ramp_brakes_through_zero():
    assert ramp(0.2, -0.2, 0.025, 0.05) == pytest.approx(0.15)
    assert ramp(0.02, -0.2, 0.025, 0.05) == 0.0
    assert ramp(0.0, 0.1, 0.025, 0.05) == pytest.approx(0.025)


def test_raised_forks_limit_speed_sf08():
    s = sup()
    s.set_fork(80.0)
    o, _ = run(s, 0.0, 2.0, v=0.3)
    assert o.v == pytest.approx(0.10) and o.forks_raised


def test_docking_source_wins_and_is_limited_sf03():
    s = sup()
    t = 0.0
    for _ in range(60):
        feed(s, t)
        s.set_cmd("nav", 0.3, 0.0, t)
        s.set_cmd("dock", 0.1, 0.05, t)
        o = s.step(t, DT)
        t += DT
    assert o.source == "dock" and o.docking and o.v == pytest.approx(0.10)
    # the pallet right under the forks does not stop docking (centre sensor ignored, like the firmware) ...
    for _ in range(5):
        feed(s, t, tof=[0.02, 2, 2, 2, 2, 2])
        s.set_cmd("dock", 0.1, 0.0, t)
        o = s.step(t, DT)
        t += DT
    assert o.state != STOP and o.v > 0
    # ... the corners still stop while docking ...
    feed(s, t, tof=[0.02, 0.05, 2, 2, 2, 2])
    s.set_cmd("dock", 0.1, 0.0, t)
    assert s.step(t, DT).state == STOP
    # ... and the centre reading stops normal driving
    s2 = sup()
    o, _ = run(s2, 0.0, 0.5, tof=[0.10] + [2.0] * 5)
    assert o.state == STOP and o.reason == "tof_stop" and o.v == 0.0


def test_tof_slow_and_stop_only_in_direction_of_travel_sf02():
    s = sup()
    o, t = run(s, 0.0, 2.0, v=0.3, tof=[0.3, 2, 2, 2, 2, 2])
    assert o.state == SLOW and o.reason == "tof_slow" and o.v == pytest.approx(0.09)
    o, t = run(s, t, 0.2, v=0.3, tof=[0.1, 2, 2, 2, 2, 2])
    assert o.state == STOP and o.v == 0.0  # immediate, no ramp (NFR-04)
    o, t = run(s, t, 1.0, v=-0.1, tof=[0.1, 2, 2, 2, 2, 2])
    assert o.state == NORMAL and o.v < 0  # reversing away from the obstacle is allowed
    o, t = run(s, t, 0.2, v=-0.1, tof=[2, 2, 2, 2, 2, 0.1])
    assert o.state == STOP


def test_stale_tof_limits_speed():
    s = sup()
    o, _ = run(s, 0.0, 2.0, v=0.3, tof=None)
    assert o.state == SLOW and o.v <= 0.09 + 1e-9


def test_person_slow_and_stop_sf06():
    s = sup()
    t = 0.0
    for _ in range(40):
        feed(s, t)
        s.set_person(0.8, t)
        s.set_cmd("nav", 0.3, 0.0, t)
        o = s.step(t, DT)
        t += DT
    assert o.state == SLOW and o.reason == "person_detected" and o.v == pytest.approx(0.10)
    feed(s, t)
    s.set_person(0.4, t)
    s.set_cmd("nav", 0.3, 0.0, t)
    o = s.step(t, DT)
    assert o.state == STOP and o.v == 0.0


def test_person_detector_required_but_silent():
    s = sup(person_detector_required=True)
    o, _ = run(s, 0.0, 2.0)
    assert o.state == SLOW and o.v == pytest.approx(0.10)


def test_estop_pause_and_mode_gating():
    s = sup()
    o, t = run(s, 0.0, 1.0, estop=True)
    assert o.state == ESTOP and o.reason == "fleet_estop" and o.v == 0.0
    o, t = run(s, t, 1.0, mode="paused")
    assert o.state == STOP and o.reason == "paused"
    # in manual mode only the gamepad drives
    o, t = run(s, t, 1.0, mode="manual", source="nav")
    assert o.source == "none" and o.v == 0.0
    o, t = run(s, t, 1.0, mode="manual", source="teleop", v=0.2)
    assert o.source == "teleop" and o.v == pytest.approx(0.2)
    # and in auto the gamepad is ignored
    o, t = run(s, t, 1.0, mode="busy", source="teleop", v=0.2)
    assert o.source == "none" and o.v == 0.0


def test_server_loss_finishes_step_then_stops_sf05():
    s = sup()
    o, t = run(s, 0.0, 1.0, connected=False, silence=5.0, step_active=True)
    assert o.state == NORMAL and o.v > 0           # the lane in progress may finish
    o, t = run(s, t, 0.1, connected=False, silence=5.0, step_active=False)
    assert o.state == STOP and o.reason == "wifi_lost" and o.v == 0.0
    o, t = run(s, t, 0.1, connected=False, silence=2.0, step_active=False)
    assert o.state == NORMAL                        # < 3 s is tolerated


def test_command_and_fleet_watchdogs():
    s = sup()
    feed(s, 0.0)
    s.set_cmd("nav", 0.3, 0.0, 0.0)
    s.step(0.0, DT)
    o = s.step(0.6, DT)                             # command older than 0.5 s
    assert o.source == "none" and o.v == 0.0
    s.set_cmd("nav", 0.3, 0.0, 2.0)
    o = s.step(2.0, DT)                             # fleet_status older than 1 s
    assert o.state == STOP and o.reason == "watchdog_timeout"


def test_esp32_state_byte_matches_firmware():
    """docs/12-firmware.md §3: byte = state << 4 | reason."""
    assert encode_esp32(ESP_ESTOP, 13) == 0x6D
    assert decode_esp32(0x6D) == (ESTOP, "estop_pressed")          # ESTOP waiting for reset
    assert decode_esp32(encode_esp32(ESP_SLOW, 3)) == (SLOW, "tof_slow")
    assert decode_esp32(encode_esp32(ESP_SLOW, 7)) == (NORMAL, "")  # docking cap is not a hazard
    assert decode_esp32(encode_esp32(4, 5)) == (STOP, "tof_stop")    # tof_fault
    assert decode_esp32(encode_esp32(1, 2)) == (NORMAL, "")         # IDLE, no_link
    assert decode_esp32(encode_esp32(0, 1)) == (STOP, "")           # BOOT
    s = sup()
    s.set_esp32(encode_esp32(ESP_SAFE, 8))
    o, t = run(s, 0.0, 0.5)
    assert o.state == SAFE and o.reason == "watchdog_timeout" and o.v == 0.0
    s.set_esp32(encode_esp32(2, 0))                 # ESP32 recovers by itself after 1 s (FW-08): follow it
    o, t = run(s, t, 0.5)
    assert o.state == NORMAL
    assert s.reset()


def test_unknown_fork_height_and_faulted_tof():
    s = SafetySupervisor()
    o, t = run(s, 0.0, 2.0)                         # no fork_state yet = unknown height
    assert o.v == pytest.approx(0.10) and o.forks_raised
    s.set_fork(float("nan"))
    o, t = run(s, t, 0.5)
    assert o.forks_raised
    s = sup()
    o, t = run(s, 0.0, 0.5, tof=[float("nan"), 2, 2, 2, 2, 2])
    assert o.state == STOP and o.reason == "tof_stop"  # faulted leading sensor
    o, t = run(s, t, 1.0, v=-0.1, tof=[float("nan"), 2, 2, 2, 2, 2])
    assert o.state == NORMAL and o.v < 0


def test_lift_forwarding_by_mode():
    s = sup()
    feed(s, 0.0)
    s.set_lift("mission", 60.0, 0.0)
    assert s.step(0.0, DT).lift_mm == 60.0
    assert s.step(0.05, DT).lift_mm is None          # forwarded once
    s.set_lift("mission", 500.0, 0.1)
    assert s.step(0.1, DT).lift_mm == 120.0          # clamped to the lift range
    s.set_lift("teleop", 30.0, 0.15)
    assert s.step(0.15, DT).lift_mm is None          # gamepad ignored in auto
    feed(s, 0.2, mode="manual")
    s.set_lift("teleop", 30.0, 0.2)
    assert s.step(0.2, DT).lift_mm == 30.0
    feed(s, 0.25, estop=True)
    s.set_lift("mission", 10.0, 0.25)
    assert s.step(0.25, DT).lift_mm is None          # nothing moves under E-stop


def test_params_from_dict():
    p = SafetyParams.from_dict({"max_speed": 0.25, "tof_forward": ["front"], "unknown": 1})
    assert p.max_speed == 0.25 and p.tof_forward == ["front"]
    assert math.isclose(p.docking_speed, 0.1)


def test_fork_state_array_from_firmware():
    from forklift_safety.fork_state import parse_fork_array
    r = parse_fork_array([62.5, 0.0, 1.0, 1.0, 0.0, 1.0])
    assert r.height_mm == 62.5 and not r.limit_high and r.limit_low and r.tcrt_left and not r.tcrt_right
    assert r.at_target and r.height_known
    r = parse_fork_array([float("nan"), 1, 0, 0, 0, 0])
    assert not r.height_known and r.limit_high
    assert parse_fork_array([1.0, 0.0]) is None
