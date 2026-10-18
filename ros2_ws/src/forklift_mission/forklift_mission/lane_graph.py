"""Lane graph from the server's warehouse_map.yaml (SV-02, NV-09) — pure Python, no ROS.

The fleet server owns the map (`backend/app/warehouse_map.yaml`); forklift_bringup installs a copy of that
file at build time so robot and server always plan on the same graph. Segment ids on `wh/traffic/{segment}` are
`<from>--<to>` in the direction of travel (docs/03-mqtt-contract.md); node ids may contain a single dash (R1-W1).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


def segment_id(a: str, b: str) -> str:
    """Directed traffic segment id: `wh/traffic/<a>--<b>` while driving from a to b."""
    return f"{a}--{b}"


def parse_segment(seg: str) -> tuple[str, str]:
    a, sep, b = seg.partition("--")
    if not sep or not a or not b:
        raise ValueError(f"not a segment id: {seg!r}")
    return a, b


@dataclass(frozen=True)
class Location:
    code: str
    node: str
    yaw: float
    height_mm: int
    aruco: int | None
    type: str


@dataclass
class LaneGraph:
    nodes: dict[str, tuple[float, float]]
    adj: dict[str, list[str]] = field(default_factory=dict)
    locations: dict[str, Location] = field(default_factory=dict)
    robots: dict[str, dict] = field(default_factory=dict)
    zones: list[dict] = field(default_factory=list)
    width_m: float = 0.0
    height_m: float = 0.0
    resolution_m: float = 0.02

    # ------------------------------------------------------------------ loading
    @classmethod
    def from_dict(cls, raw: dict) -> "LaneGraph":
        nodes = {str(k): (float(v[0]), float(v[1])) for k, v in raw["nodes"].items()}
        adj: dict[str, list[str]] = {n: [] for n in nodes}
        for a, b in raw.get("edges", []):
            a, b = str(a), str(b)
            if a not in nodes or b not in nodes:
                raise ValueError(f"edge {a}-{b} uses an unknown node")
            adj[a].append(b)
            adj[b].append(a)
        locs = {}
        for loc in raw.get("locations", []):
            locs[loc["code"]] = Location(code=loc["code"], node=str(loc["node"]), yaw=float(loc.get("yaw", 0.0)),
                                         height_mm=int(loc.get("height_mm", 0)), aruco=loc.get("aruco"),
                                         type=str(loc.get("type", "")))
        robots = {r["id"]: dict(r) for r in raw.get("robots", [])}
        return cls(nodes=nodes, adj=adj, locations=locs, robots=robots, zones=list(raw.get("zones", [])),
                   width_m=float(raw.get("width_m", 0.0)), height_m=float(raw.get("height_m", 0.0)),
                   resolution_m=float(raw.get("resolution_m", 0.02)))

    @classmethod
    def load(cls, path: str | Path) -> "LaneGraph":
        return cls.from_dict(yaml.safe_load(Path(path).read_text(encoding="utf-8")))

    # ------------------------------------------------------------------ geometry
    def xy(self, node: str) -> tuple[float, float]:
        return self.nodes[node]

    def dist(self, a: str, b: str) -> float:
        return math.dist(self.nodes[a], self.nodes[b])

    def heading(self, a: str, b: str) -> float:
        (ax, ay), (bx, by) = self.nodes[a], self.nodes[b]
        return math.atan2(by - ay, bx - ax)

    def is_lane(self, a: str, b: str) -> bool:
        return a in self.adj and b in self.adj[a]

    def nearest_node(self, x: float, y: float) -> str:
        return min(self.nodes, key=lambda n: math.hypot(self.nodes[n][0] - x, self.nodes[n][1] - y))

    def charger_location(self, home_node: str) -> Location | None:
        return next((l for l in self.locations.values() if l.node == home_node and l.type == "charge"), None)

    # ------------------------------------------------------------------ planning
    def plan(self, start: str, goal: str) -> list[str]:
        """Shortest path (Dijkstra on lane length) — fallback when the server route does not cover a leg."""
        if start not in self.nodes or goal not in self.nodes:
            raise KeyError(f"unknown node in plan {start} -> {goal}")
        dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
        while pq:
            d, n = heapq.heappop(pq)
            if n == goal:
                break
            if d > dist.get(n, math.inf):
                continue
            for nb in self.adj[n]:
                nd = d + self.dist(n, nb)
                if nd < dist.get(nb, math.inf):
                    dist[nb], prev[nb] = nd, n
                    heapq.heappush(pq, (nd, nb))
        if goal not in dist:
            raise ValueError(f"no route {start} -> {goal}")
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1]

    def lanes(self, path: list[str]) -> list[tuple[str, str]]:
        """Consecutive (from, to) pairs of a node path, skipping repeated nodes; raises on a non-lane step."""
        out = []
        for a, b in zip(path, path[1:]):
            if a == b:
                continue
            if not self.is_lane(a, b):
                raise ValueError(f"route step {a} -> {b} is not a lane")
            out.append((a, b))
        return out

    def split_route(self, route: list[str], start: str | None, pick: str, drop: str) -> tuple[list[str], list[str]]:
        """Split the server's task route at the pick node into (to_pick, to_drop) node paths.

        Mirrors tools/mqtt_robot_client: route = [current node, ..., pick, ..., drop]. A leg that the route does not
        cover (empty route, robot not at the route start, route not ending at the drop) is planned locally.
        """
        route = [n for n in route if n in self.nodes]
        if pick in route:
            i = route.index(pick)
            to_pick, to_drop = route[: i + 1], route[i:]
        else:
            to_pick, to_drop = [], []
        if start is not None:
            if not to_pick:
                to_pick = self.plan(start, pick)
            elif to_pick[0] != start:
                to_pick = self.plan(start, to_pick[0])[:-1] + to_pick
        elif not to_pick:
            to_pick = [pick]
        if not to_drop or to_drop[-1] != drop:
            to_drop = self.plan(pick, drop)
        self.lanes(to_pick)
        self.lanes(to_drop)
        return to_pick, to_drop

    def goal_yaws(self, path: list[str], final_yaw: float | None) -> list[float]:
        """Heading to request at each node of `path[1:]`: the direction of the *next* lane, so an Ackermann robot
        arrives already lined up (no turning on the spot), and `final_yaw` (location yaw) at the last node."""
        yaws = []
        for i in range(1, len(path)):
            if i + 1 < len(path):
                yaws.append(self.heading(path[i], path[i + 1]))
            elif final_yaw is not None:
                yaws.append(final_yaw)
            else:
                yaws.append(self.heading(path[i - 1], path[i]))
        return yaws
