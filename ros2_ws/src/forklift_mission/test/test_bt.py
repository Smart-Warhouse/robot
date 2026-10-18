from forklift_mission.bt import Action, Context, Do, DoneHandle, Retry, Sequence, Status, UntilSuccess, Wait
from forklift_mission.testing import FakeClock, TimedHandle


def act(name, results, log):
    it = iter(results)

    def start(ctx):
        log.append(name)
        ok = next(it)
        return TimedHandle(1, ok, "" if ok else "boom")
    return Action(name, start, failure_code=f"{name}_failed")


def run(node, ctx, n=50):
    for _ in range(n):
        st = node.tick(ctx)
        if st is not Status.RUNNING:
            return st
    return Status.RUNNING


def test_sequence_runs_in_order():
    log, ctx = [], Context(FakeClock())
    seq = Sequence("s", act("a", [True], log), act("b", [True], log))
    assert run(seq, ctx) is Status.SUCCESS
    assert log == ["a", "b"]


def test_sequence_failure_records_reason():
    log, ctx = [], Context(FakeClock())
    seq = Sequence("s", act("a", [False], log), act("b", [True], log))
    assert run(seq, ctx) is Status.FAILURE
    assert log == ["a"] and ctx.last_failure_code == "a_failed" and "boom" in ctx.last_failure


def test_retry_with_recovery_three_attempts():
    log, ctx = [], Context(FakeClock())
    rec = Do("recover", lambda c: log.append("recover"))
    r = Retry("r", act("nav", [False, False, True], log), 3, rec)
    assert run(r, ctx) is Status.SUCCESS
    assert log == ["nav", "recover", "nav", "recover", "nav"]


def test_retry_gives_up_after_attempts():
    log, ctx = [], Context(FakeClock())
    fails = []
    r = Retry("r", act("nav", [False] * 3, log), 3, None, lambda c, n: fails.append(n))
    assert run(r, ctx) is Status.FAILURE
    assert fails == [1, 2, 3] and log.count("nav") == 3


def test_halt_keeps_progress_and_restarts_running_step():
    log, ctx = [], Context(FakeClock())
    seq = Sequence("s", act("a", [True], log), act("b", [True, True], log))
    seq.tick(ctx)  # a running
    seq.tick(ctx)  # a done -> b started
    assert log == ["a", "b"]
    seq.halt()
    assert run(seq, ctx) is Status.SUCCESS
    assert log == ["a", "b", "b"]  # b restarted, a not repeated


def test_soft_hold_blocks_new_actions_only():
    log, ctx = [], Context(FakeClock())
    seq = Sequence("s", act("a", [True], log), act("b", [True], log))
    seq.tick(ctx)           # a started
    ctx.soft_hold = True
    for _ in range(5):
        assert seq.tick(ctx) is Status.RUNNING
    assert log == ["a"]     # a finished, b not started
    ctx.soft_hold = False
    assert run(seq, ctx) is Status.SUCCESS and log == ["a", "b"]


def test_wait_and_until_success():
    clock = FakeClock()
    ctx = Context(clock)
    w = Wait("w", 1.0)
    assert w.tick(ctx) is Status.RUNNING
    clock.advance(1.0)
    assert w.tick(ctx) is Status.SUCCESS
    calls = []
    results = iter([False, False, True])
    u = UntilSuccess("u", Action("x", lambda c: (calls.append(1), DoneHandle(next(results)))[1]), 0.5)
    st = Status.RUNNING
    for _ in range(10):
        st = u.tick(ctx)
        clock.advance(0.25)
        if st is Status.SUCCESS:
            break
    assert st is Status.SUCCESS and len(calls) == 3
