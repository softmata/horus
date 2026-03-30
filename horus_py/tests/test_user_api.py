"""
User-facing Python API tests.

Tests the EXACT code a new user writes on day 1:
  import horus
  node = horus.Node('my_node', tick=my_function)
  horus.run(node, duration=1.0)

Run: cd horus_py && maturin develop && pytest tests/test_user_api.py -v
"""
import time
import threading


def test_basic_node_tick_fires():
    """The most basic test: create a node, run it, verify tick was called."""
    import horus

    ticks = [0]

    def my_tick(node=None):
        ticks[0] += 1

    node = horus.Node("basic_counter", tick=my_tick, rate=100)
    horus.run(node, duration=0.5, tick_rate=100)
    print(f"  Basic node: {ticks[0]} ticks in 0.5s")
    assert ticks[0] > 10, f"Expected >10 ticks, got {ticks[0]}"


def test_node_send_recv():
    """Two nodes: publisher sends, subscriber receives via topic."""
    import horus

    received = []

    def pub_tick(node):
        node.send("test_sr_topic", {"x": 1.5, "y": 2.5})

    def sub_tick(node):
        msg = node.recv("test_sr_topic")
        if msg is not None:
            received.append(msg)

    pub = horus.Node("sr_pub", tick=pub_tick, pubs="test_sr_topic", rate=50)
    sub = horus.Node("sr_sub", tick=sub_tick, subs="test_sr_topic", rate=50)
    horus.run(pub, sub, duration=1.0, tick_rate=50)

    print(f"  Send/recv: {len(received)} messages received")
    assert len(received) > 5, f"Expected >5 messages, got {len(received)}"


def test_scheduler_api():
    """Test the Scheduler class directly (not the run() shortcut)."""
    import horus

    ticks = [0]

    def my_tick(node=None):
        ticks[0] += 1

    node = horus.Node("sched_test", tick=my_tick, rate=100)
    sched = horus.Scheduler(tick_rate=100, name="test_sched")
    sched.add(node)
    sched.run(duration=0.5)

    print(f"  Scheduler API: {ticks[0]} ticks")
    assert ticks[0] > 10, f"Expected >10 ticks, got {ticks[0]}"


def test_multiple_nodes_in_scheduler():
    """3 nodes in one scheduler — all should tick."""
    import horus

    counters = [0, 0, 0]

    def tick_a(node=None):
        counters[0] += 1

    def tick_b(node=None):
        counters[1] += 1

    def tick_c(node=None):
        counters[2] += 1

    a = horus.Node("multi_a", tick=tick_a, rate=100)
    b = horus.Node("multi_b", tick=tick_b, rate=100)
    c = horus.Node("multi_c", tick=tick_c, rate=100)
    horus.run(a, b, c, duration=0.5, tick_rate=100)

    print(f"  Multiple nodes: A={counters[0]}, B={counters[1]}, C={counters[2]}")
    assert all(c > 10 for c in counters), f"All nodes should tick >10 times: {counters}"


def test_node_exception_doesnt_crash():
    """A node that raises an exception shouldn't crash the scheduler."""
    import horus

    healthy_ticks = [0]
    exceptions = [0]

    def bad_tick(node=None):
        exceptions[0] += 1
        if exceptions[0] == 3:
            raise ValueError("intentional error at tick 3")

    def good_tick(node=None):
        healthy_ticks[0] += 1

    bad = horus.Node("bad_node", tick=bad_tick, rate=50)
    good = horus.Node("good_node", tick=good_tick, rate=50)
    horus.run(bad, good, duration=1.0, tick_rate=50)

    print(f"  Exception test: healthy={healthy_ticks[0]}, bad ticked {exceptions[0]} times")
    assert healthy_ticks[0] > 10, f"Healthy node should keep running, got {healthy_ticks[0]}"


def test_node_with_init_and_shutdown():
    """Test Node lifecycle: init → tick → shutdown."""
    import horus

    lifecycle = []

    def my_init(node=None):
        lifecycle.append("init")

    def my_tick(node=None):
        lifecycle.append("tick")

    def my_shutdown(node=None):
        lifecycle.append("shutdown")

    node = horus.Node("lifecycle", tick=my_tick, init=my_init, shutdown=my_shutdown, rate=100)
    horus.run(node, duration=0.3, tick_rate=100)

    print(f"  Lifecycle: {lifecycle[:5]}...{lifecycle[-3:]}")
    assert lifecycle[0] == "init", "init should be first"
    assert lifecycle[-1] == "shutdown", "shutdown should be last"
    assert "tick" in lifecycle, "tick should be called"


def test_node_with_rate_and_order():
    """Test rate and order builder options."""
    import horus

    order_log = []

    def tick_a(node=None):
        order_log.append("A")

    def tick_b(node=None):
        order_log.append("B")

    a = horus.Node("ordered_a", tick=tick_a, rate=100, order=0)
    b = horus.Node("ordered_b", tick=tick_b, rate=100, order=1)
    horus.run(a, b, duration=0.3, tick_rate=100, deterministic=True)

    # In deterministic mode, order 0 should always run before order 1
    violations = 0
    for i in range(0, len(order_log) - 1, 2):
        if i + 1 < len(order_log):
            if order_log[i] != "A" or order_log[i + 1] != "B":
                violations += 1

    print(f"  Order test: {len(order_log)} entries, {violations} violations")
    assert violations == 0, f"Order(0) should run before Order(1): {violations} violations"


def test_deterministic_produces_same_output():
    """Two deterministic runs should produce identical tick sequences."""
    import horus

    def run_once():
        values = []
        counter = [0]

        def tick(node=None):
            counter[0] += 1
            values.append(counter[0])

        node = horus.Node("det_test", tick=tick, rate=100)
        horus.run(node, duration=0.2, tick_rate=100, deterministic=True)
        return values

    run1 = run_once()
    run2 = run_once()

    print(f"  Determinism: run1={len(run1)} ticks, run2={len(run2)} ticks")
    assert run1 == run2, f"Deterministic runs differ: {len(run1)} vs {len(run2)}"
