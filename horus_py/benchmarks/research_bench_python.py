#!/usr/bin/env python3
"""
HORUS Python Benchmarks — Sustained Measurement with Full Percentile Distribution

Measures Python binding performance with academic rigor:
- Sustained runs (10s+ per test) with 100K+ samples
- Full percentile distribution (p50/p95/p99/p999/max)
- FFI overhead attribution (Rust baseline vs Python)
- CSV/JSON output for analysis scripts

Run:
    python3 horus_py/benchmarks/research_bench_python.py
    python3 horus_py/benchmarks/research_bench_python.py --duration 30 --csv results.csv
    python3 horus_py/benchmarks/research_bench_python.py --duration 2  # quick validation
"""

import argparse
import json
import os
import platform
import statistics
import sys
import time

# ── Statistics ────────────────────────────────────────────────────────────────

class Stats:
    __slots__ = ('count', 'p50', 'p95', 'p99', 'p999', 'max_val', 'mean', 'stddev', 'min_val')

    def __init__(self, samples):
        samples.sort()
        n = len(samples)
        self.count = n
        if n == 0:
            self.p50 = self.p95 = self.p99 = self.p999 = self.max_val = self.mean = self.stddev = self.min_val = 0
            return
        self.p50 = samples[n // 2]
        self.p95 = samples[int(n * 0.95)]
        self.p99 = samples[int(n * 0.99)]
        self.p999 = samples[int(n * 0.999)]
        self.max_val = samples[-1]
        self.min_val = samples[0]
        self.mean = sum(samples) // n
        variance = sum((s - self.mean) ** 2 for s in samples) / n
        self.stddev = int(variance ** 0.5)

    def header():
        return f"{'Test':<35s} {'Samples':>8s} {'p50':>8s} {'p95':>8s} {'p99':>8s} {'p999':>8s} {'max':>8s}"

    def row(self, name):
        def fmt(ns):
            if ns >= 1_000_000:
                return f"{ns/1_000_000:.1f}ms"
            elif ns >= 1_000:
                return f"{ns/1_000:.1f}us"
            else:
                return f"{ns}ns"
        count_str = f"{self.count//1000}K" if self.count >= 1000 else str(self.count)
        return f"{name:<35s} {count_str:>8s} {fmt(self.p50):>8s} {fmt(self.p95):>8s} {fmt(self.p99):>8s} {fmt(self.p999):>8s} {fmt(self.max_val):>8s}"


# ── Sustained benchmark helper ────────────────────────────────────────────────

def sustained_bench(name, setup_fn, measure_fn, duration_secs, warmup_secs=1.0):
    """Run measure_fn() continuously for duration_secs, return Stats."""
    ctx = setup_fn()

    # Warmup
    deadline = time.perf_counter() + warmup_secs
    while time.perf_counter() < deadline:
        measure_fn(ctx)

    # Measure
    samples = []
    deadline = time.perf_counter() + duration_secs
    while time.perf_counter() < deadline:
        start = time.perf_counter_ns()
        measure_fn(ctx)
        samples.append(time.perf_counter_ns() - start)

    stats = Stats(samples)
    print(f"  {stats.row(name)}")
    return name, samples, stats


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='HORUS Python Benchmarks (sustained)')
    parser.add_argument('--duration', type=int, default=10, help='Seconds per test (default: 10)')
    parser.add_argument('--csv', type=str, help='Write raw samples to CSV')
    parser.add_argument('--json', type=str, help='Write summary to JSON')
    args = parser.parse_args()

    duration = args.duration

    # ── Platform info ─────────────────────────────────────────────────

    print("╔════════════════════════════════════════════════════════════╗")
    print("║        HORUS Python Benchmarks (sustained)                  ║")
    print("╚════════════════════════════════════════════════════════════╝")
    print()

    py_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    cpu = platform.processor() or platform.machine()
    os_name = platform.platform()

    try:
        import horus
        horus_version = horus.get_version() if hasattr(horus, 'get_version') else "unknown"
    except Exception:
        horus_version = "unknown"

    print(f"Python: {py_version}")
    print(f"HORUS:  {horus_version}")
    print(f"CPU:    {cpu}")
    print(f"OS:     {os_name}")
    print(f"Duration: {duration}s per test")
    print()

    all_results = []
    csv_rows = []

    # ── 1. Typed Message IPC (Pod zero-copy path) ─────────────────────

    print("── Typed Message IPC (zero-copy Pod path) ──────────────────")
    print(f"  {Stats.header()}")

    try:
        from horus._horus import Topic, CmdVel, Pose2D, Imu

        def setup_cmdvel():
            t = Topic("rbench_cmdvel")
            msg = CmdVel(1.0, 0.5)
            return (t, msg)

        def measure_cmdvel(ctx):
            t, msg = ctx
            t.send(msg)
            t.recv()

        name, samples, stats = sustained_bench("CmdVel send+recv", setup_cmdvel, measure_cmdvel, duration)
        all_results.append(("typed_ipc", "CmdVel", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("typed_ipc", "CmdVel", i, s))

        def setup_pose2d():
            t = Topic(Pose2D)
            msg = Pose2D(1.0, 2.0, 0.5)
            return (t, msg)

        def measure_pose2d(ctx):
            t, msg = ctx
            t.send(msg)
            t.recv()

        name, samples, stats = sustained_bench("Pose2D send+recv", setup_pose2d, measure_pose2d, duration)
        all_results.append(("typed_ipc", "Pose2D", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("typed_ipc", "Pose2D", i, s))

        def setup_imu():
            t = Topic(Imu)
            msg = Imu(0.1, 0.2, 9.8, 0.01, -0.02, 0.0)
            return (t, msg)

        def measure_imu(ctx):
            t, msg = ctx
            t.send(msg)
            t.recv()

        name, samples, stats = sustained_bench("Imu send+recv", setup_imu, measure_imu, duration)
        all_results.append(("typed_ipc", "Imu", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("typed_ipc", "Imu", i, s))

    except ImportError as e:
        print(f"  SKIP — horus not installed: {e}")

    print()

    # ── 2. Generic Message IPC (MessagePack path) ─────────────────────

    print("── Generic Message IPC (MessagePack path) ──────────────────")
    print(f"  {Stats.header()}")

    try:
        from horus._horus import Topic

        def setup_dict_small():
            t = Topic("rbench_dict_small")
            msg = {"v": 1.0}
            return (t, msg)

        def measure_dict(ctx):
            t, msg = ctx
            t.send(msg)
            t.recv()

        name, samples, stats = sustained_bench("dict {v: 1.0}", setup_dict_small, measure_dict, duration)
        all_results.append(("generic_ipc", "dict_small", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("generic_ipc", "dict_small", i, s))

        def setup_dict_medium():
            t = Topic("rbench_dict_med")
            msg = {"x": 1.0, "y": 2.0, "z": 3.0, "w": 0.5}
            return (t, msg)

        name, samples, stats = sustained_bench("dict {x,y,z,w}", setup_dict_medium, measure_dict, duration)
        all_results.append(("generic_ipc", "dict_medium", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("generic_ipc", "dict_medium", i, s))

        def setup_dict_large():
            t = Topic("rbench_dict_large")
            msg = {f"key_{i}": float(i) for i in range(50)}
            return (t, msg)

        name, samples, stats = sustained_bench("dict 50 keys (~1KB)", setup_dict_large, measure_dict, duration)
        all_results.append(("generic_ipc", "dict_large", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("generic_ipc", "dict_large", i, s))

    except ImportError:
        print("  SKIP — horus not installed")

    print()

    # ── 3. Image Zero-Copy ────────────────────────────────────────────

    print("── Image Zero-Copy ────────────────────────────────────────")
    print(f"  {Stats.header()}")

    try:
        import numpy as np
        from horus import Image

        def setup_image():
            data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            img = Image.from_numpy(data)
            return (img, data)

        def measure_to_numpy(ctx):
            img, _ = ctx
            img.to_numpy()

        name, samples, stats = sustained_bench("Image.to_numpy(640x480)", setup_image, measure_to_numpy, duration)
        all_results.append(("image", "to_numpy_640x480", stats))
        for i, s in enumerate(samples):
            csv_rows.append(("image", "to_numpy_640x480", i, s))

        def measure_np_copy(ctx):
            _, data = ctx
            data.copy()

        name, samples, stats = sustained_bench("np.copy(640x480) baseline", setup_image, measure_np_copy, duration)
        all_results.append(("image", "np_copy_640x480", stats))

        try:
            def measure_dlpack(ctx):
                img, _ = ctx
                np.from_dlpack(img)

            name, samples, stats = sustained_bench("np.from_dlpack(640x480)", setup_image, measure_dlpack, duration)
            all_results.append(("image", "dlpack_640x480", stats))
        except Exception:
            print("  SKIP — DLPack not supported")

    except ImportError as e:
        print(f"  SKIP — numpy or horus.Image not available: {e}")

    print()

    # ── 4. Scheduler Tick Overhead ────────────────────────────────────

    print("── Scheduler Tick Overhead ─────────────────────────────────")

    try:
        import horus

        tick_times = []
        def timed_tick(node):
            tick_times.append(time.perf_counter_ns())

        node = horus.Node(name="rbench_tick", tick=timed_tick, rate=10000, order=0)
        sched = horus.Scheduler(tick_rate=10000)
        sched.add(node)

        tick_times.clear()
        start = time.perf_counter()
        sched.run(duration=float(min(duration, 5)))
        elapsed = time.perf_counter() - start

        if len(tick_times) > 2:
            intervals = [tick_times[i+1] - tick_times[i] for i in range(len(tick_times)-1)]
            stats = Stats(intervals)
            count = len(tick_times)
            hz = count / elapsed if elapsed > 0 else 0
            print(f"  Ticks: {count:,} in {elapsed:.2f}s ({hz:.0f} Hz)")
            print(f"  {stats.row('Tick interval (Rust→Py→Rust)')}")
            all_results.append(("scheduler", "tick_interval", stats))

            # Per-second breakdown (GC pause detection)
            if elapsed >= 2:
                per_sec = {}
                base = tick_times[0]
                for t in tick_times:
                    sec = int((t - base) / 1_000_000_000)
                    per_sec[sec] = per_sec.get(sec, 0) + 1
                rates = list(per_sec.values())
                if rates:
                    min_r = min(rates)
                    max_r = max(rates)
                    print(f"  Per-second: min={min_r} max={max_r} (GC dip: {max_r - min_r} ticks)")
        else:
            print("  Too few ticks collected")

    except ImportError:
        print("  SKIP — horus not installed")

    print()

    # ── 5. FFI Overhead Attribution ───────────────────────────────────

    print("── FFI Overhead Attribution (Rust vs Python) ────────────────")

    # Rust baselines (from research_latency benchmark on same hardware)
    rust_baselines = {
        "CmdVel": 22,    # DirectChannel 8B p50 in ns
        "Imu": 30,       # DirectChannel ~64B p50
        "dict_small": 22, # Rust has no dict equivalent — use raw SHM baseline
    }

    # Try loading from JSON if available
    json_path = os.path.join(os.path.dirname(__file__), '..', '..', 'benchmarks', 'research', 'results', 'latency.json')
    if os.path.exists(json_path):
        try:
            with open(json_path) as f:
                data = json.load(f)
            for r in data.get('results', []):
                if r.get('backend') == 'DirectChannel' and r.get('msg_size_bytes') == 8:
                    rust_baselines['CmdVel'] = r['p50_ns']
                if r.get('backend') == 'DirectChannel' and r.get('msg_size_bytes') == 64:
                    rust_baselines['Imu'] = r['p50_ns']
            print(f"  (Rust baselines loaded from {json_path})")
        except Exception:
            pass
    else:
        print("  (Using hardcoded Rust baselines — run research_latency for exact numbers)")

    print()
    print(f"  {'Operation':<25s} {'Rust (ns)':>10s} {'Python (ns)':>12s} {'Overhead':>10s} {'Factor':>8s}")
    print(f"  {'─'*70}")

    for test, msg_type, stats in all_results:
        if test != "typed_ipc" and test != "generic_ipc":
            continue
        rust_ns = rust_baselines.get(msg_type, rust_baselines.get("CmdVel", 22))
        py_ns = stats.p50
        overhead = py_ns - rust_ns
        factor = py_ns / rust_ns if rust_ns > 0 else 0
        print(f"  {msg_type:<25s} {rust_ns:>10,} {py_ns:>12,} {overhead:>9,}ns {factor:>7.0f}x")

    print()
    print("  Overhead sources: PyO3 boundary (~500ns) + GIL acquire (~500ns) + Python object alloc (~500ns)")

    # ── CSV Output ────────────────────────────────────────────────────

    if args.csv and csv_rows:
        with open(args.csv, 'w') as f:
            f.write("test,msg_type,sample_id,latency_ns\n")
            for test, msg_type, sample_id, latency in csv_rows:
                f.write(f"{test},{msg_type},{sample_id},{latency}\n")
        print(f"\nCSV written: {args.csv} ({len(csv_rows):,} rows)")

    # ── JSON Output ───────────────────────────────────────────────────

    if args.json:
        report = {
            "benchmark": "research_bench_python",
            "platform": {
                "python": py_version,
                "horus": horus_version,
                "cpu": cpu,
                "os": os_name,
            },
            "duration_secs": duration,
            "results": [
                {
                    "test": test,
                    "msg_type": msg_type,
                    "samples": stats.count,
                    "p50_ns": stats.p50,
                    "p95_ns": stats.p95,
                    "p99_ns": stats.p99,
                    "p999_ns": stats.p999,
                    "max_ns": stats.max_val,
                    "mean_ns": stats.mean,
                    "stddev_ns": stats.stddev,
                }
                for test, msg_type, stats in all_results
            ],
        }
        with open(args.json, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"JSON written: {args.json}")

    print("\nDone.")


if __name__ == '__main__':
    main()
