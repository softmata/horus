# HORUS Benchmarks

Performance benchmarks for the HORUS robotics IPC framework.

## Quick Start

```bash
# Main benchmark - shows all 10 Topic backend latencies
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Benchmarks

| Benchmark | What It Measures | Run Command |
|-----------|------------------|-------------|
| `all_paths_latency` | **All 10 Topic backends** — DirectChannel to MpmcShm | `cargo run --release -p horus_benchmarks --bin all_paths_latency` |
| `cross_process_benchmark` | True IPC between separate processes | `cargo run --release -p horus_benchmarks --bin cross_process_benchmark` |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan) | `cargo run --release -p horus_benchmarks --bin robotics_messages_benchmark` |
| `determinism_benchmark` | Jitter and real-time suitability | `cargo run --release -p horus_benchmarks --bin determinism_benchmark` |
| `scalability_benchmark` | Multi-thread scaling | `cargo run --release -p horus_benchmarks --bin scalability_benchmark` |

## Expected Results

### Topic Backend Latencies (`all_paths_latency`)

All 10 backends benchmarked with function-pointer dispatch (zero-branch hot path):

| Scenario | Backend | Topology | Expected Latency |
|----------|---------|----------|------------------|
| SameThread | DirectChannel | same thread, 1P-1C, POD | ~3ns |
| CrossThread-1P1C | SpscIntra | same process, 1P-1C | ~18ns |
| CrossThread-1PMC | SpmcIntra | same process, 1P-2C | ~24ns |
| CrossThread-MP1C | MpscIntra | same process, 2P-1C | ~26ns |
| CrossThread-MPMC | MpmcIntra | same process, 2P-2C | ~36ns |
| CrossProc-PodShm | PodShm | cross process, 2P-2C, POD | ~50ns |
| CrossProc-2P1C | MpscShm | cross process, 2P-1C | ~65ns |
| CrossProc-1PMC | SpmcShm | cross process, 1P-2C | ~70ns |
| CrossProc-1P1C | SpscShm | cross process, 1P-1C | ~85ns |
| CrossProc-MPMC | MpmcShm | cross process, MPMC | ~167ns |

### Real-Time Suitability

| Control Rate | Requirement | HORUS Result |
|--------------|-------------|--------------|
| 1 kHz | < 1 ms | PASS |
| 10 kHz | < 100 us | PASS |
| 100 kHz | < 10 us | PASS |

## For Accurate Results

```bash
# Set CPU governor to performance mode
sudo cpupower frequency-set -g performance

# Or manually
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## License

Apache-2.0
