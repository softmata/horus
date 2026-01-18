# HORUS Benchmark Suite

Industry-grade benchmark suite for the HORUS robotics IPC framework. Designed for academic publication and production validation.

## Quick Start

```bash
# Run all benchmarks
./run_benchmarks.sh

# Quick validation run
./run_benchmarks.sh --quick

# Run only performance benchmarks
./run_benchmarks.sh --perf

# Run only Criterion benchmarks
./run_benchmarks.sh --criterion
```

## Benchmark Categories

### Performance Benchmarks (`src/bin/performance/`)

| Benchmark | Description | Output |
|-----------|-------------|--------|
| `ipc_benchmark` | Core IPC latency and throughput | Console |
| `pod_benchmark` | Zero-copy POD message performance | Console |
| `determinism_benchmark` | Jitter and real-time analysis | JSON |
| `scalability_benchmark` | Multi-thread scaling curves | JSON |
| `cross_process_benchmark` | TRUE inter-process IPC (separate processes) | JSON |
| `robotics_messages_benchmark` | Real robotics message types (CmdVel, Imu, etc.) | JSON |
| `dds_comparison_benchmark` | Comparison vs CycloneDDS/FastDDS | JSON |

### Criterion Benchmarks (`benches/`)

| Benchmark | Description |
|-----------|-------------|
| `latency_rigorous` | Statistical latency analysis with bootstrap CI |
| `competitive_comparison` | vs crossbeam, flume, std::sync::mpsc |
| `latency_matrix` | Message size × backend matrix |
| `topic_performance` | Topic API performance |
| `production_messages` | Real-world message patterns |

### Functional Tests (`src/bin/functional/`)

API compatibility, memory safety, and correctness validation.

### Stress Tests (`src/bin/stress/`)

High-load reliability testing: multi-process, many topics, chaos engineering.

### Production Tests (`src/bin/production/`)

Long-running qualification tests including 24-hour soak testing.

## Statistical Methodology

Per REP 2014 and Hoefler's "12 Rules for Benchmarking":

- **Bootstrap CI**: 10,000 iterations for confidence intervals
- **Tukey IQR Filtering**: Removes measurement artifacts
- **Normality Testing**: Jarque-Bera, Anderson-Darling, D'Agostino tests
- **Percentiles**: p1, p5, p25, p50, p75, p95, p99, p99.9, p99.99
- **Determinism Metrics**: CV, jitter, deadline miss rate
- **Cross-Process Testing**: Separate producer/consumer processes

## Running Individual Benchmarks

```bash
# Build all benchmarks
cargo build --release -p horus_benchmarks

# Performance benchmarks
cargo run --release --bin determinism_benchmark -- --json results.json
cargo run --release --bin cross_process_benchmark -- --iterations 100000
cargo run --release --bin robotics_messages_benchmark
cargo run --release --bin dds_comparison_benchmark

# Criterion benchmarks
cargo bench --bench latency_rigorous
cargo bench --bench competitive_comparison
```

## Output Formats

### JSON Reports

All performance benchmarks output JSON for CI integration:

```json
{
  "name": "SpscIntra",
  "statistics": {
    "median": 45.0,
    "p99": 120,
    "ci_low": 44.2,
    "ci_high": 45.8
  },
  "determinism": {
    "cv": 0.12,
    "max_jitter_ns": 500,
    "deadline_misses": 0
  }
}
```

### Chrome Trace Format

Enable detailed tracing for visualization in `chrome://tracing`:

```rust
use horus_benchmarks::tracing::BenchmarkTracer;
let mut tracer = BenchmarkTracer::new("my_benchmark");
// ... run benchmark ...
tracer.write_chrome_trace("trace.json")?;
```

## Real-Time Suitability

The benchmarks validate suitability for robotics control loops:

| Control Rate | p99 Requirement | Typical Result |
|--------------|-----------------|----------------|
| 1 kHz | < 1 ms | ✓ PASS |
| 10 kHz | < 100 µs | ✓ PASS |
| 100 kHz | < 10 µs | Depends on backend |

## Platform Optimization

For accurate results:

```bash
# Set CPU governor to performance
sudo cpupower frequency-set -g performance

# Disable CPU frequency scaling
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Pin benchmark to specific cores (automatic in benchmarks)
taskset -c 0,1 cargo run --release --bin determinism_benchmark
```

## Directory Structure

```
benchmarks/
├── Cargo.toml              # Package manifest
├── README.md               # This file
├── run_benchmarks.sh       # Master benchmark runner
├── src/
│   ├── lib.rs              # Library exports
│   ├── stats.rs            # Statistical analysis
│   ├── platform.rs         # Platform detection
│   ├── timing.rs           # High-precision timing
│   ├── output.rs           # JSON/CSV output
│   ├── tracing.rs          # Instrumentation
│   └── bin/
│       ├── performance/    # Performance benchmarks
│       ├── functional/     # Functional tests
│       ├── stress/         # Stress tests
│       └── production/     # Production qualification
└── benches/                # Criterion benchmarks
```

## CI Integration

The benchmark suite outputs machine-readable JSON for regression detection:

```yaml
# Example GitHub Actions
- name: Run benchmarks
  run: |
    cargo run --release --bin determinism_benchmark -- --json results.json

- name: Check for regressions
  run: |
    python scripts/check_regression.py results.json baseline.json
```

## License

Apache-2.0
