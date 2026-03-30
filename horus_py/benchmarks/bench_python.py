#!/usr/bin/env python3
"""
HORUS Python Benchmarks — Measure what matters for robotics.

Run: python3 benchmarks/bench_python.py

Benchmarks:
1. Message roundtrip latency (dict, typed)
2. Node tick overhead (empty, with send/recv)
3. Image zero-copy (to_numpy, from_numpy, to_torch)
4. Multi-node scheduler throughput
5. Generic message serialization sizes
"""

import time
import statistics
import sys

# ── Helpers ──────────────────────────────────────────────────────────────────

def bench(name, fn, iterations=10000, warmup=100):
    """Run fn() iterations times, report median/p99/min latency."""
    # Warmup
    for _ in range(warmup):
        fn()

    times = []
    for _ in range(iterations):
        start = time.perf_counter_ns()
        fn()
        end = time.perf_counter_ns()
        times.append(end - start)

    times.sort()
    median = times[len(times) // 2]
    p99 = times[int(len(times) * 0.99)]
    min_t = times[0]
    avg = statistics.mean(times)

    print(f"  {name:45s}  median={median:>8,}ns  p99={p99:>8,}ns  min={min_t:>8,}ns  avg={avg:>8,.0f}ns  ({iterations} iters)")
    return median


def section(title):
    print(f"\n{'=' * 80}")
    print(f"  {title}")
    print(f"{'=' * 80}")


# ── 1. Message Send/Recv Latency ────────────────────────────────────────────

def bench_message_latency():
    section("1. Message Send/Recv Latency (single-process Topic roundtrip)")

    from horus._horus import Topic, CmdVel, Pose2D, Imu

    # Dict (GenericMessage path)
    topic_dict = Topic("bench_dict")
    msg_dict = {"x": 1.0, "y": 2.0, "z": 3.0}
    bench("dict {x, y, z}", lambda: (topic_dict.send(msg_dict), topic_dict.recv()))

    # Small dict
    topic_small = Topic("bench_small")
    msg_small = {"v": 1.0}
    bench("dict {v}", lambda: (topic_small.send(msg_small), topic_small.recv()))

    # Large dict (~1KB)
    topic_large = Topic("bench_large")
    msg_large = {"data": list(range(100)), "label": "sensor_001"}
    bench("dict ~1KB", lambda: (topic_large.send(msg_large), topic_large.recv()), iterations=5000)

    # Typed CmdVel (Pod zero-copy path)
    topic_cmd = Topic(CmdVel)
    msg_cmd = CmdVel(1.0, 0.5)
    bench("CmdVel (typed, zero-copy)", lambda: (topic_cmd.send(msg_cmd), topic_cmd.recv()))

    # Typed Pose2D
    topic_pose = Topic(Pose2D)
    msg_pose = Pose2D(1.0, 2.0, 0.5)
    bench("Pose2D (typed, zero-copy)", lambda: (topic_pose.send(msg_pose), topic_pose.recv()))

    # Typed Imu (larger struct)
    topic_imu = Topic(Imu)
    msg_imu = Imu(0.1, 0.2, 9.8, 0.01, -0.02, 0.0)
    bench("Imu (typed, zero-copy)", lambda: (topic_imu.send(msg_imu), topic_imu.recv()))


# ── 2. Node Tick Overhead ───────────────────────────────────────────────────

def bench_node_overhead():
    section("2. Node Tick Overhead (scheduler → Python tick → return)")

    import horus

    # Empty tick
    ticks = [0]
    def empty_tick(node):
        ticks[0] += 1

    node = horus.Node(name="bench_empty", tick=empty_tick, rate=1000)
    sched = horus.Scheduler(tick_rate=1000)
    sched.add(node)

    start = time.perf_counter()
    sched.run(duration=1.0)
    elapsed = time.perf_counter() - start

    tick_count = ticks[0]
    if tick_count > 0:
        avg_us = (elapsed / tick_count) * 1_000_000
        print(f"  {'Empty tick (Rust→Python→Rust)':45s}  {tick_count:,} ticks in {elapsed:.2f}s  avg={avg_us:.1f}μs/tick  ({tick_count/elapsed:.0f} Hz)")

    # Tick with send
    sends = [0]
    def send_tick(node):
        node.send("bench_out", {"v": sends[0]})
        sends[0] += 1

    node2 = horus.Node(name="bench_send", tick=send_tick, rate=1000, pubs=["bench_out"])
    sched2 = horus.Scheduler(tick_rate=1000)
    sched2.add(node2)

    start = time.perf_counter()
    sched2.run(duration=1.0)
    elapsed = time.perf_counter() - start

    send_count = sends[0]
    if send_count > 0:
        avg_us = (elapsed / send_count) * 1_000_000
        print(f"  {'Tick + send(dict)':45s}  {send_count:,} ticks in {elapsed:.2f}s  avg={avg_us:.1f}μs/tick  ({send_count/elapsed:.0f} Hz)")

    # Tick with send + recv
    pairs = [0]
    def sendrecv_tick(node):
        node.send("bench_loop", {"v": pairs[0]})
        node.recv("bench_loop")
        pairs[0] += 1

    node3 = horus.Node(name="bench_sr", tick=sendrecv_tick, rate=1000, pubs=["bench_loop"], subs=["bench_loop"])
    sched3 = horus.Scheduler(tick_rate=1000)
    sched3.add(node3)

    start = time.perf_counter()
    sched3.run(duration=1.0)
    elapsed = time.perf_counter() - start

    pair_count = pairs[0]
    if pair_count > 0:
        avg_us = (elapsed / pair_count) * 1_000_000
        print(f"  {'Tick + send(dict) + recv(dict)':45s}  {pair_count:,} ticks in {elapsed:.2f}s  avg={avg_us:.1f}μs/tick  ({pair_count/elapsed:.0f} Hz)")


# ── 3. Image Zero-Copy ──────────────────────────────────────────────────────

def bench_image_zerocopy():
    section("3. Image/PointCloud Zero-Copy vs Naive Copy")
    print("  (horus uses shared memory pool + DLPack — to_numpy returns a pointer, no pixel copy)")
    print()

    try:
        import numpy as np
        from horus import Image, PointCloud, DepthImage
    except ImportError:
        print("  SKIP — numpy not available")
        return

    # ── Image: zero-copy vs naive copy across sizes ──
    sizes = [
        ("320x240",  (240, 320, 3)),
        ("640x480",  (480, 640, 3)),
        ("1280x720", (720, 1280, 3)),
        ("1920x1080", (1080, 1920, 3)),
    ]

    for name, shape in sizes:
        data = np.random.randint(0, 255, shape, dtype=np.uint8)
        kb = data.nbytes / 1024

        img = Image.from_numpy(data)

        # Warmup
        for _ in range(50):
            img.to_numpy()
            data.copy()

        # Zero-copy: to_numpy (returns view into shared memory)
        times_zc = []
        for _ in range(1000):
            s = time.perf_counter_ns()
            img.to_numpy()
            times_zc.append(time.perf_counter_ns() - s)
        times_zc.sort()
        zc = times_zc[len(times_zc) // 2]

        # Naive: np.copy (full pixel data copy)
        times_cp = []
        for _ in range(1000):
            s = time.perf_counter_ns()
            data.copy()
            times_cp.append(time.perf_counter_ns() - s)
        times_cp.sort()
        cp = times_cp[len(times_cp) // 2]

        speedup = cp / zc if zc > 0 else 0
        print(f"  Image {name:10s} ({kb:>5.0f}KB)  zero-copy={zc:>8,}ns  np.copy={cp:>8,}ns  → {speedup:.0f}x faster")

    # ── DLPack protocol (fastest path) ──
    print()
    img_med = Image.from_numpy(np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8))
    bench("np.from_dlpack(Image 640x480)", lambda: np.from_dlpack(img_med), iterations=5000)

    # ── PointCloud zero-copy ──
    print()
    pc_data = np.random.randn(10000, 3).astype(np.float32)
    pc = PointCloud.from_numpy(pc_data)
    bench("PointCloud.to_numpy(10K points)", lambda: pc.to_numpy(), iterations=5000)

    pc_large = PointCloud.from_numpy(np.random.randn(100000, 3).astype(np.float32))
    bench("PointCloud.to_numpy(100K points)", lambda: pc_large.to_numpy(), iterations=2000)

    # ── DepthImage zero-copy ──
    print()
    depth_data = np.random.rand(480, 640).astype(np.float32)
    depth = DepthImage.from_numpy(depth_data)
    bench("DepthImage.to_numpy(640x480)", lambda: depth.to_numpy(), iterations=5000)

    # ── PyTorch interop ──
    print()
    try:
        import torch
        bench("Image.to_torch(640x480 RGB)", lambda: img_med.to_torch(), iterations=2000)
    except ImportError:
        print(f"  {'SKIP — torch not available':45s}")


# ── 4. Multi-Node Throughput ────────────────────────────────────────────────

def bench_multi_node():
    section("4. Multi-Node Scheduler Throughput")

    import horus

    # 3 nodes at different rates
    counts = {"fast": [0], "medium": [0], "slow": [0]}

    def fast_tick(node):
        counts["fast"][0] += 1

    def medium_tick(node):
        counts["medium"][0] += 1

    def slow_tick(node):
        counts["slow"][0] += 1

    fast = horus.Node(name="fast", tick=fast_tick, rate=1000, order=0)
    medium = horus.Node(name="medium", tick=medium_tick, rate=100, order=1)
    slow = horus.Node(name="slow", tick=slow_tick, rate=10, order=2)

    sched = horus.Scheduler(tick_rate=1000)
    sched.add(fast)
    sched.add(medium)
    sched.add(slow)

    start = time.perf_counter()
    sched.run(duration=2.0)
    elapsed = time.perf_counter() - start

    print(f"  {'3-node pipeline (1000/100/10 Hz)':45s}  fast={counts['fast'][0]:,}  medium={counts['medium'][0]:,}  slow={counts['slow'][0]:,}  in {elapsed:.2f}s")

    # 10 nodes all at 100Hz
    node_counts = {}
    nodes = []
    for i in range(10):
        name = f"node_{i}"
        node_counts[name] = [0]
        def make_tick(n=name):
            def tick(node):
                node_counts[n][0] += 1
            return tick
        nodes.append(horus.Node(name=name, tick=make_tick(), rate=100, order=i))

    sched2 = horus.Scheduler(tick_rate=1000)
    for n in nodes:
        sched2.add(n)

    start = time.perf_counter()
    sched2.run(duration=2.0)
    elapsed = time.perf_counter() - start

    total = sum(v[0] for v in node_counts.values())
    print(f"  {'10 nodes × 100Hz':45s}  total={total:,} ticks in {elapsed:.2f}s  ({total/elapsed:.0f} ticks/s)")


# ── 5. Serialization Size ───────────────────────────────────────────────────

def bench_serialization_size():
    section("5. Generic Message Serialization Overhead")

    import msgpack

    messages = {
        "empty dict": {},
        "CmdVel-like": {"linear": 1.0, "angular": 0.5},
        "Pose2D-like": {"x": 1.0, "y": 2.0, "theta": 0.5},
        "IMU-like": {"accel": [0.1, 0.2, 9.8], "gyro": [0.01, -0.02, 0.0], "mag": [0.3, 0.1, 0.5]},
        "LaserScan-like (360 rays)": {"ranges": [1.0 + 0.01*i for i in range(360)]},
        "Detection-like": {"class": "person", "conf": 0.95, "bbox": [100, 200, 50, 80]},
        "10 detections": [{"class": f"obj_{i}", "conf": 0.9, "bbox": [i*10, i*20, 50, 80]} for i in range(10)],
    }

    for name, msg in messages.items():
        packed = msgpack.packb(msg)
        print(f"  {name:45s}  {len(packed):>6} bytes (msgpack)")


# ── Main ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("HORUS Python Benchmarks")
    print(f"Python {sys.version.split()[0]}")

    try:
        from horus import get_version
        print(f"horus {get_version()}")
    except Exception:
        print("horus version unknown")

    bench_message_latency()
    bench_node_overhead()
    bench_image_zerocopy()
    bench_multi_node()
    bench_serialization_size()

    print(f"\n{'=' * 80}")
    print("  Done.")
    print(f"{'=' * 80}")
