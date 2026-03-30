# HORUS Benchmark Suite

Benchmarks for the HORUS robotics IPC framework with sustained runs, full percentile distributions, and CSV output.

## Quick Start

```bash
# 1. Configure environment (requires root)
sudo ./research/setup.sh

# 2. Run raw hardware baselines
cargo run --release -p horus_benchmarks --bin raw_baselines -- --json research/results/baselines.json

# 3. Run all-paths latency (standard mode)
cargo run --release -p horus_benchmarks --bin all_paths_latency -- --json research/results/latency.json

# 4. Restore system settings
sudo ./research/setup.sh --teardown
```

## Methodology

### Timing Precision

All measurements use **RDTSC cycle counting** (x86_64) with:
- Pre-calibrated CPU frequency (100-500ms calibration phase)
- Timing overhead subtraction (measured per-run via `measure_rdtsc_overhead()`)
- Serializing fences (`lfence` + `mfence`) around measured code
- Fallback to `Instant::now()` on non-x86_64 platforms

### Statistical Rigor

Each benchmark reports:
- **Percentiles**: p1, p5, p50, p95, p99, p99.9, p99.99, max
- **Central tendency**: mean, median, trimmed mean
- **Dispersion**: standard deviation, coefficient of variation, IQR
- **Confidence**: 95% bootstrap confidence intervals (10,000 resamples)
- **Normality**: Jarque-Bera test, skewness, excess kurtosis
- **Outlier filtering**: Tukey IQR method (1.5x IQR fence)

### Environment Control

For reproducible results:
1. **CPU Governor**: Set to `performance` (prevents frequency scaling)
2. **Turbo Boost**: Disabled (prevents frequency spikes between iterations)
3. **CPU Isolation**: Recommended `isolcpus=2,3` kernel boot param
4. **CPU Affinity**: Benchmarks pin to specific cores via `sched_setaffinity`
5. **Memory Locking**: `mlockall()` prevents page faults during measurement

### Message Sizes

Standard sizes matching real robotics data:
| Size | Robotics Equivalent |
|------|-------------------|
| 8B | CmdVel (linear + angular) |
| 64B | IMU (orientation + angular_vel + linear_acc) |
| 256B | JointState (16 joints) |
| 1KB | Odometry with covariance |
| 8KB | LaserScan (360 ranges) |
| 64KB | Small point cloud |

### Backend Paths

HORUS auto-detects the optimal IPC backend:
| Backend | Topology | Latency |
|---------|----------|---------|
| DirectChannel | Same thread | ~3ns |
| SpscIntra | Same process, 1P:1C | ~18ns |
| SpmcIntra | Same process, 1P:MC | ~24ns |
| MpscIntra | Same process, MP:1C | ~26ns |
| MpmcIntra | Same process, MP:MC | ~36ns |
| PodShm | Cross-process, POD type | ~50ns |
| SpscShm | Cross-process, 1P:1C | ~85ns |
| MpscShm | Cross-process, MP:1C | ~65ns |
| SpmcShm | Cross-process, 1P:MC | ~70ns |
| MpmcShm | Cross-process, MP:MC | ~167ns |

## Benchmark Binaries

| Binary | Purpose |
|--------|---------|
| `raw_baselines` | Hardware floor: memcpy, atomic, mmap (no HORUS) |
| `all_paths_latency` | All 10 backend paths with RDTSC precision |
| `cross_process_benchmark` | True inter-process IPC latency |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan) |
| `determinism_benchmark` | Bit-identical replay verification |
| `scalability_benchmark` | Node and topic count scaling |
| **Extended Suite** | |
| `raw_baselines` | Hardware floor (memcpy, atomic, mmap) |
| `research_latency` | Sustained measurement + size sweep, all topologies |
| `research_throughput` | Per-second throughput timeseries |
| `research_jitter` | RT tick jitter + IPC under CPU contention |
| `research_scalability` | Node scaling (1-100) + topic scaling (1-1000) |
| `competitor_comparison` | HORUS vs raw UDP (+ Zenoh with `--features zenoh`) |
| **Python** | |
| `research_bench_python.py` | Python IPC latency, FFI overhead, image zero-copy |

## Reproducing Results

### Hardware Requirements
- x86_64 CPU with `constant_tsc` flag (Intel Core/Xeon, AMD Ryzen/EPYC)
- Linux kernel 5.15+ (for io_uring benchmarks, optional)
- 4+ CPU cores (2 for benchmark, 2 for OS)
- 2GB+ /dev/shm (shared memory)

### Software Requirements
- Rust 1.75+ (`rustup install stable`)
- Linux tools: `cpupower` (`sudo apt install linux-tools-common`)
- Python 3.9+ with matplotlib, numpy, pandas (for analysis)

### Steps
1. Clone: `git clone https://github.com/softmata/horus.git && cd horus`
2. Setup: `sudo benchmarks/research/setup.sh`
3. Build: `cargo build --release -p horus_benchmarks`
4. Run: `benchmarks/research/run_all.sh`
5. Results: `benchmarks/research/results/`
6. Teardown: `sudo benchmarks/research/setup.sh --teardown`

## Output Formats

- **Human-readable**: Default (tables and boxes to stdout)
- **JSON**: `--json <path>` (machine-readable, includes platform info)
- **CSV**: `--csv <path>` (raw samples, one row per measurement)
