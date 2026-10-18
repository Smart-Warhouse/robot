import math

import pytest

from forklift_mission.lane_graph import LaneGraph, parse_segment, segment_id


@pytest.fixture(scope="module")
def g(warehouse_map_path):
    return LaneGraph.load(warehouse_map_path)


def test_loads_server_map(g):
    assert {"J1", "A21", "R1-E1", "DO1", "CH1"} <= set(g.nodes)
    assert g.is_lane("A21", "R1-E1") and g.is_lane("R1-E1", "A21")
    assert g.locations["R1-E1-L2"].node == "R1-E1" and g.locations["R1-E1-L2"].height_mm == 60
    assert g.robots["fl1"]["home"] == "CH1"
    assert g.charger_location("CH1").code == "CHARGE-1"


def test_segment_ids_keep_dashes_in_node_ids():
    assert segment_id("A21", "R1-E1") == "A21--R1-E1"
    assert parse_segment("A21--R1-E1") == ("A21", "R1-E1")
    assert parse_segment("R1-W1--A11") == ("R1-W1", "A11")
    with pytest.raises(ValueError):
        parse_segment("A21-R1")


def test_plan_shortest(g):
    path = g.plan("CH1", "R1-W1")
    assert path[0] == "CH1" and path[-1] == "R1-W1"
    g.lanes(path)  # every step is a lane
    assert path == ["CH1", "J4", "A12", "A11", "R1-W1"]


def test_split_route_like_the_reference_client(g):
    # the docs/03 example route
    route = ["J5", "A22", "A21", "R1-E1", "A21", "J2", "J3", "A31", "DO1"]
    to_pick, to_drop = g.split_route(route, "J5", "R1-E1", "DO1")
    assert to_pick == ["J5", "A22", "A21", "R1-E1"]
    assert to_drop == ["R1-E1", "A21", "J2", "J3", "A31", "DO1"]


def test_split_route_plans_missing_legs(g):
    to_pick, to_drop = g.split_route([], "CH1", "R1-E1", "DO1")
    assert to_pick[0] == "CH1" and to_pick[-1] == "R1-E1"
    assert to_drop[0] == "R1-E1" and to_drop[-1] == "DO1"
    # robot is not where the route starts: a planned leg is prefixed
    route = ["J5", "A22", "A21", "R1-E1", "A21", "J2", "J3", "A31", "DO1"]
    to_pick, _ = g.split_route(route, "CH2", "R1-E1", "DO1")
    assert to_pick[:3] == ["CH2", "J5", "A22"]


def test_split_route_rejects_non_lane(g):
    with pytest.raises(ValueError):
        g.split_route(["J5", "J1", "R1-E1"], "J5", "R1-E1", "DO1")


def test_goal_yaws_face_next_lane_and_location(g):
    path = ["CH1", "J4", "A12", "A11", "R1-W1"]
    yaws = g.goal_yaws(path, 0.0)
    assert len(yaws) == 4
    assert yaws[0] == pytest.approx(-math.pi / 2)  # at J4: face down the aisle towards A12
    assert yaws[-1] == pytest.approx(0.0)           # at the spur: location yaw (facing the rack)
