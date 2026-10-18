from pathlib import Path

import pytest
import yaml

from forklift_mission.footprint import footprint, footprint_string, half_width, parse_footprint_string
from forklift_mission.lift_core import LiftTracker

SRC = Path(__file__).resolve().parents[2]
DIMS = yaml.safe_load((SRC / "forklift_description" / "config" / "dimensions.yaml").read_text())


def test_footprint_includes_forks_and_pallet():
    empty = footprint(DIMS)
    loaded = footprint(DIMS, loaded=True)
    assert max(x for x, _ in empty) == pytest.approx(DIMS["carriage_x"] + DIMS["fork_length"])
    assert max(x for x, _ in loaded) == pytest.approx(DIMS["carriage_x"] + DIMS["pallet_depth"])
    assert min(x for x, _ in empty) == pytest.approx(-DIMS["body_length"])
    assert half_width(empty) == pytest.approx(DIMS["body_width"] / 2)
    assert half_width(loaded) >= DIMS["pallet_width"] / 2
    assert parse_footprint_string(footprint_string(empty)) == empty


def test_nav2_params_use_this_footprint_and_inflation():
    """NV-07 / NV-12: the static footprint in nav2_params.yaml is the unloaded polygon of dimensions.yaml and the
    inflation radius is >= half the robot width + 3 cm."""
    nav = yaml.safe_load((SRC / "forklift_navigation" / "config" / "nav2_params.yaml").read_text())
    for cm in ("local_costmap", "global_costmap"):
        p = nav[cm][cm]["ros__parameters"]
        assert parse_footprint_string(p["footprint"]) == footprint(DIMS)
        assert p["inflation_layer"]["inflation_radius"] >= DIMS["body_width"] / 2 + 0.03


def test_lift_tracker_reaches_target_with_settling():
    t = LiftTracker(60, now=0.0, tolerance_mm=3, settle_samples=3)
    assert not t.update(30, False, False, 0.1).done
    assert not t.update(58.5, False, False, 0.2).done
    assert not t.update(59.0, False, False, 0.3).done
    r = t.update(60.5, False, False, 0.4)
    assert r.done and r.success


def test_lift_tracker_limit_and_timeout():
    t = LiftTracker(200, now=0.0, lift_max_mm=120)
    assert t.target == 120
    r = t.update(100, False, True, 0.5)
    assert r.done and not r.success and "upper limit" in r.message
    t = LiftTracker(0, now=0.0)
    r = t.update(5, True, False, 0.1)
    assert r.done and r.success
    t = LiftTracker(50, now=0.0, timeout_s=1.0)
    r = t.update(10, False, False, 1.5)
    assert r.done and not r.success and "timeout" in r.message
