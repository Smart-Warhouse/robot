"""Validate published MQTT messages against docs/schemas (the robot <-> server contract, CM-05)."""

from __future__ import annotations

import json
from datetime import datetime
from functools import lru_cache
from pathlib import Path

SCHEMAS = Path(__file__).resolve().parents[5] / "docs" / "schemas"


@lru_cache(maxsize=None)
def schema(name: str) -> dict:
    return json.loads((SCHEMAS / f"{name}.schema.json").read_text(encoding="utf-8"))


def schema_for(topic: str) -> str | None:
    parts = topic.split("/")
    if topic == "wh/fleet/estop":
        return "estop"
    if parts[:2] == ["wh", "traffic"]:
        return "traffic"
    if parts[:2] == ["wh", "fl"] and len(parts) == 4:
        return {"state": "state", "task_status": "task_status", "event": "event", "task": "task", "cmd": "cmd",
                "teleop": "teleop"}.get(parts[3])
    return None


def validate(topic: str, body: dict) -> None:
    import jsonschema

    name = schema_for(topic)
    assert name, f"no schema for {topic}"
    if name == "state" and body == {"online": False}:
        return  # Last Will: the only state message without seq/ts (docs/03)
    jsonschema.validate(body, schema(name))
    datetime.fromisoformat(body["ts"].replace("Z", "+00:00"))  # format: date-time
