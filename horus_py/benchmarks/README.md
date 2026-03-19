# HORUS Python Benchmarks

## Quick Run

```bash
cd horus_py
maturin develop --no-default-features  # build extension
PYTHONPATH=. python3 benchmarks/bench_python.py
```

## Extended Benchmark

Sustained runs with full percentile distributions, FFI overhead attribution, and CSV/JSON output:

```bash
# Quick validation (2s per test)
python3 benchmarks/research_bench_python.py --duration 2

# Full run (10s per test, CSV + JSON output)
python3 benchmarks/research_bench_python.py --duration 10 --csv results.csv --json results.json

# Reports: p50/p95/p99/p999/max for each test
# Produces: FFI overhead table (Rust baseline vs Python)
# Detects: GC pauses via per-second tick count analysis
```

## Results (Python 3.12, Linux x86_64, WSL2)

### Message Send/Recv Latency

| Message type | Median | Path |
|---|---|---|
| CmdVel (typed) | 1.5μs | Zero-copy Pod memcpy |
| Pose2D (typed) | 1.6μs | Zero-copy Pod memcpy |
| Imu (typed) | 1.6μs | Zero-copy Pod memcpy |
| dict `{"v": 1.0}` | 5.4μs | GenericMessage + MessagePack |
| dict `{"x", "y", "z"}` | 9.1μs | GenericMessage + MessagePack |
| dict ~1KB | 52μs | GenericMessage + MessagePack |

### Zero-Copy: to_numpy() is O(1)

| Data | to_numpy() | np.copy() | Speedup |
|---|---|---|---|
| Image 320×240 (225KB) | 3.0μs | 3.0μs | 1x |
| Image 640×480 (900KB) | 3.0μs | 13μs | 4x |
| Image 1280×720 (2.7MB) | 3.0μs | 75μs | 25x |
| Image 1920×1080 (6MB) | 3.0μs | 178μs | 59x |
| PointCloud 10K pts | 2.8μs | — | — |
| PointCloud 100K pts | 2.8μs | — | — |
| DepthImage 640×480 | 2.8μs | — | — |
| np.from_dlpack() | 979ns | — | — |

### Node Tick Overhead

| Scenario | Throughput | Per-tick |
|---|---|---|
| Empty tick | ~530 Hz | 1.9ms |
| Tick + send(dict) | ~525 Hz | 1.9ms |
| Tick + send + recv | ~525 Hz | 1.9ms |

Bottleneck is Python GIL (~1.8ms), not the Rust binding (~30μs).

### GenericMessage Sizes

| Payload | Bytes |
|---|---|
| Empty dict | 1 |
| CmdVel-like | 34 |
| IMU-like | 100 |
| LaserScan 360 rays | 3,251 |
| 10 detections | 374 |
| Max GenericMessage | 4,096 |
