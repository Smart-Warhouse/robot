"""A small tick-based Behavior Tree (no ROS, no threads) for the forklift mission (requirements §5 forklift_mission).

Semantics (BehaviorTree.CPP-like, with memory):
  * `tick(ctx)` returns SUCCESS, FAILURE or RUNNING. Long operations (Nav2 goal, Dock, Lift) are `Action` leaves
    that start an asynchronous *handle* on the first tick and poll it on later ticks.
  * `halt()` stops whatever is running (cancels the handle) but keeps the progress of Sequences, so a paused
    mission resumes at the step it was in — that step restarts (a Nav2 goal is simply re-sent).
  * `reset()` forgets all progress.
  * A soft hold (`ctx.soft_hold`, used for SF-05 server loss) lets the step in progress finish but does not
    *start* a new Action: it returns RUNNING until the hold clears.
"""

from __future__ import annotations

import enum
from collections.abc import Callable
from typing import Any, Protocol


class Status(enum.Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class Handle(Protocol):
    """An asynchronous operation started by an Action leaf (an action-client goal, a timer...)."""

    def status(self) -> Status: ...

    def cancel(self) -> None: ...

    @property
    def message(self) -> str: ...


class DoneHandle:
    """A handle that is already finished (synchronous operations, tests)."""

    def __init__(self, ok: bool = True, message: str = "") -> None:
        self._s = Status.SUCCESS if ok else Status.FAILURE
        self.message = message

    def status(self) -> Status:
        return self._s

    def cancel(self) -> None:
        pass


class Context:
    """Blackboard passed to every tick. `soft_hold` gates the start of new actions (SF-05)."""

    def __init__(self, now: Callable[[], float]) -> None:
        self.now = now
        self.soft_hold = False
        self.data: dict[str, Any] = {}
        self.last_failure = ""        # message of the most recent failing leaf
        self.last_failure_code = ""   # event code (nav_failed, pickup_failed, ...)


class Node:
    def __init__(self, name: str) -> None:
        self.name = name
        self.status: Status | None = None

    def tick(self, ctx: Context) -> Status:
        self.status = self._tick(ctx)
        return self.status

    def _tick(self, ctx: Context) -> Status:  # pragma: no cover - abstract
        raise NotImplementedError

    def halt(self) -> None:
        """Stop a running operation; keep progress."""

    def reset(self) -> None:
        self.halt()
        self.status = None

    def running_leaf(self) -> "Node | None":
        return self if self.status is Status.RUNNING else None

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.name!r})"


class Action(Node):
    """Leaf that starts `start(ctx) -> Handle` once and polls the handle. `on_done(ctx, ok, msg)` is optional."""

    def __init__(self, name: str, start: Callable[[Context], Handle],
                 on_done: Callable[[Context, bool, str], None] | None = None, failure_code: str = "",
                 is_motion: bool = False) -> None:
        super().__init__(name)
        self._start = start
        self._on_done = on_done
        self.failure_code = failure_code
        self.is_motion = is_motion  # a motion step (lane, dock, straight move) — reported as FleetStatus.step_active
        self.handle: Handle | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.handle is None:
            if ctx.soft_hold:
                return Status.RUNNING
            self.handle = self._start(ctx)
        st = self.handle.status()
        if st is Status.RUNNING:
            return st
        msg = getattr(self.handle, "message", "") or ""
        self.handle = None
        if self._on_done:
            self._on_done(ctx, st is Status.SUCCESS, msg)
        if st is Status.FAILURE:
            ctx.last_failure = f"{self.name}: {msg}" if msg else self.name
            if self.failure_code:
                ctx.last_failure_code = self.failure_code
        return st

    def halt(self) -> None:
        if self.handle is not None:
            self.handle.cancel()
            self.handle = None
        if self.status is Status.RUNNING:
            self.status = None


class Do(Node):
    """Instant leaf: runs `fn(ctx)`; returns SUCCESS unless fn returns False (or a Status)."""

    def __init__(self, name: str, fn: Callable[[Context], Any]) -> None:
        super().__init__(name)
        self.fn = fn

    def _tick(self, ctx: Context) -> Status:
        r = self.fn(ctx)
        if isinstance(r, Status):
            return r
        return Status.FAILURE if r is False else Status.SUCCESS


class Condition(Node):
    def __init__(self, name: str, fn: Callable[[Context], bool]) -> None:
        super().__init__(name)
        self.fn = fn

    def _tick(self, ctx: Context) -> Status:
        return Status.SUCCESS if self.fn(ctx) else Status.FAILURE


class Composite(Node):
    def __init__(self, name: str, *children: Node) -> None:
        super().__init__(name)
        self.children = list(children)
        self.index = 0

    def halt(self) -> None:
        for c in self.children:
            c.halt()
        if self.status is Status.RUNNING:
            self.status = None

    def reset(self) -> None:
        for c in self.children:
            c.reset()
        self.index = 0
        self.status = None

    def running_leaf(self) -> Node | None:
        if self.index < len(self.children):
            return self.children[self.index].running_leaf()
        return None


class Sequence(Composite):
    """Ticks children in order; remembers the running child; fails on the first failure."""

    def _tick(self, ctx: Context) -> Status:
        while self.index < len(self.children):
            st = self.children[self.index].tick(ctx)
            if st is Status.RUNNING:
                return st
            if st is Status.FAILURE:
                self.index = 0
                return st
            self.index += 1
        self.index = 0
        return Status.SUCCESS


class Fallback(Composite):
    """Ticks children in order until one succeeds."""

    def _tick(self, ctx: Context) -> Status:
        while self.index < len(self.children):
            st = self.children[self.index].tick(ctx)
            if st is Status.RUNNING:
                return st
            if st is Status.SUCCESS:
                self.index = 0
                return st
            self.index += 1
        self.index = 0
        return Status.FAILURE


class Retry(Node):
    """Run `child` up to `attempts` times; between attempts run `recovery` (if given) — NV-10.

    `on_attempt_failed(ctx, attempt)` is called after each failed attempt (for logging / events).
    """

    def __init__(self, name: str, child: Node, attempts: int, recovery: Node | None = None,
                 on_attempt_failed: Callable[[Context, int], None] | None = None) -> None:
        super().__init__(name)
        self.child = child
        self.attempts = max(1, int(attempts))
        self.recovery = recovery
        self.on_attempt_failed = on_attempt_failed
        self.failures = 0
        self.recovering = False

    def _tick(self, ctx: Context) -> Status:
        while True:
            if self.recovering:
                st = self.recovery.tick(ctx)
                if st is Status.RUNNING:
                    return st
                self.recovering = False  # recovery result does not matter: try again either way
                self.recovery.reset()
            st = self.child.tick(ctx)
            if st is not Status.FAILURE:
                if st is Status.SUCCESS:
                    self.failures = 0
                return st
            self.failures += 1
            if self.on_attempt_failed:
                self.on_attempt_failed(ctx, self.failures)
            self.child.reset()
            if self.failures >= self.attempts:
                self.failures = 0
                return Status.FAILURE
            if self.recovery is not None:
                self.recovering = True

    def halt(self) -> None:
        self.child.halt()
        if self.recovery is not None:
            self.recovery.halt()
        if self.status is Status.RUNNING:
            self.status = None

    def reset(self) -> None:
        self.child.reset()
        if self.recovery is not None:
            self.recovery.reset()
        self.failures = 0
        self.recovering = False
        self.status = None

    def running_leaf(self) -> Node | None:
        if self.recovering and self.recovery is not None:
            return self.recovery.running_leaf()
        return self.child.running_leaf()


class Wait(Node):
    """Succeeds after `seconds` (measured with ctx.now)."""

    def __init__(self, name: str, seconds: float) -> None:
        super().__init__(name)
        self.seconds = float(seconds)
        self.t0: float | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.t0 is None:
            self.t0 = ctx.now()
        if ctx.now() - self.t0 >= self.seconds:
            self.t0 = None
            return Status.SUCCESS
        return Status.RUNNING

    def halt(self) -> None:
        self.t0 = None
        if self.status is Status.RUNNING:
            self.status = None


class UntilSuccess(Node):
    """Re-tick `child` (reset on failure) until it succeeds, waiting `period` s between tries — e.g. a lane request
    that the server denies is re-sent every 500 ms (docs/03)."""

    def __init__(self, name: str, child: Node, period: float) -> None:
        super().__init__(name)
        self.child = child
        self.period = float(period)
        self.retry_at: float | None = None

    def _tick(self, ctx: Context) -> Status:
        if self.retry_at is not None:
            if ctx.now() < self.retry_at:
                return Status.RUNNING
            self.retry_at = None
        st = self.child.tick(ctx)
        if st is Status.FAILURE:
            self.child.reset()
            self.retry_at = ctx.now() + self.period
            return Status.RUNNING
        return st

    def halt(self) -> None:
        self.child.halt()
        self.retry_at = None
        if self.status is Status.RUNNING:
            self.status = None

    def reset(self) -> None:
        self.child.reset()
        self.retry_at = None
        self.status = None

    def running_leaf(self) -> Node | None:
        return self.child.running_leaf()


def walk(node: Node):
    """Depth-first iteration over a tree (for tests and introspection)."""
    yield node
    for attr in ("children",):
        for c in getattr(node, attr, []):
            yield from walk(c)
    for attr in ("child", "recovery"):
        c = getattr(node, attr, None)
        if isinstance(c, Node):
            yield from walk(c)
