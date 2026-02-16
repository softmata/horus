# HORUS Benchmarks

Performance benchmarks for the HORUS robotics framework.

## Quick Start

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Available Benchmarks

| Benchmark | What It Measures |
|-----------|------------------|
| `all_paths_latency` | All communication backends end-to-end |
| `cross_process_benchmark` | True IPC between separate processes |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan) |
| `determinism_benchmark` | Jitter and real-time suitability |
| `scalability_benchmark` | Multi-thread scaling |

Run any benchmark with:

```bash
cargo run --release -p horus_benchmarks --bin <name>
```

## Expected Results

| Scenario | Expected Latency |
|----------|------------------|
| Same thread | ~3 ns |
| Same process, single consumer | ~18 ns |
| Same process, multiple consumers | ~24-36 ns |
| Cross process | ~50-167 ns |

### Real-Time Suitability

| Control Rate | Requirement | Result |
|--------------|-------------|--------|
| 1 kHz | < 1 ms | PASS |
| 10 kHz | < 100 us | PASS |
| 100 kHz | < 10 us | PASS |

## For Accurate Results

```bash
# Set CPU governor to performance mode
sudo cpupower frequency-set -g performance
```

## License

Apache-2.0
