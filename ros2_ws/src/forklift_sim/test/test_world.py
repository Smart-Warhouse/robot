import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("gen", PKG / "scripts" / "generate_world.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def test_world_is_generated_from_the_current_map(warehouse_map_path):
    wh = yaml.safe_load(warehouse_map_path.read_text(encoding="utf-8"))
    assert gen.world_sdf(wh).encode("utf-8") == (PKG / "worlds" / "warehouse.sdf").read_bytes(), \
        "worlds/warehouse.sdf is stale: run scripts/generate_world.py"


def test_world_contents(warehouse_map_path):
    root = ET.parse(PKG / "worlds" / "warehouse.sdf").getroot()
    assert root.tag == "sdf" and root.get("version") == "1.9"
    names = {m.get("name") for m in root.iter("model")}
    assert {"wall_south", "wall_north", "wall_west", "wall_east", "rack_R1", "rack_R2", "ground_plane"} <= names
    rack = next(m for m in root.iter("model") if m.get("name") == "rack_R1")
    x, y = (float(v) for v in rack.find("pose").text.split()[:2])
    assert (x, y) == (2.0, 2.0)  # centre of the R1 zone polygon
