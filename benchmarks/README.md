# HORUS Benchmarks

Performance benchmarks for the HORUS robotics IPC framework.

## Quick Start

```bash
# Main benchmark - shows all Topic backend latencies
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Benchmarks

| Benchmark | What It Measures | Run Command |
|-----------|------------------|-------------|
| `all_paths_latency` | **All 10 Topic backends** - DirectChannel to MpmcShm | `cargo run --release -p horus_benchmarks --bin all_paths_latency` |
| `cross_process_benchmark` | True IPC between separate processes | `cargo run --release -p horus_benchmarks --bin cross_process_benchmark` |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan) | `cargo run --release -p horus_benchmarks --bin robotics_messages_benchmark` |
| `determinism_benchmark` | Jitter and real-time suitability | `cargo run --release -p horus_benchmarks --bin determinism_benchmark` |
| `scalability_benchmark` | Multi-thread scaling | `cargo run --release -p horus_benchmarks --bin scalability_benchmark` |

## Expected Results

### Topic Backend Latencies (`all_paths_latency`)

| Backend | Type | Expected Latency |
|---------|------|------------------|
| DirectChannel | Same thread | ~3ns |
| SpscIntra | Cross-thread 1P-1C | ~18ns |
| MpscIntra | Cross-thread MP-1C | ~26ns |
| MpmcIntra | Cross-thread MPMC | ~36ns |
| SpscShm | Cross-process 1P-1C | ~85ns |
| MpscShm | Cross-process MP-1C | ~65ns |
| MpmcShm | Cross-process MPMC | ~167ns |

### Real-Time Suitability

| Control Rate | Requirement | HORUS Result |
|--------------|-------------|--------------|
| 1 kHz | < 1 ms | ✓ PASS |
| 10 kHz | < 100 µs | ✓ PASS |
| 100 kHz | < 10 µs | ✓ PASS |

## For Accurate Results

```bash
# Set CPU governor to performance mode
sudo cpupower frequency-set -g performance

# Or manually
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## License

Apache-2.0
