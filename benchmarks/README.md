# HORUS Benchmarks

Performance benchmarks for the HORUS robotics framework.

## Quick Start

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Available Benchmarks

| Benchmark | What It Measures |
|-----------|------------------|
| `all_paths_latency` | All 10 communication backends end-to-end with full statistical analysis |
| `cross_process_benchmark` | True IPC between separate processes |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan, JointCommand) |
| `determinism_benchmark` | Jitter and real-time suitability |
| `scalability_benchmark` | Multi-thread/multi-process scaling |
| `dds_comparison_benchmark` | Comparison against DDS middleware |

Run any benchmark with:

```bash
cargo run --release -p horus_benchmarks --bin <name>
```

Criterion microbenchmarks:

```bash
cargo bench -p horus_benchmarks
```

## Measured Results

Benchmarked on Intel Core i7-10750H @ 2.60 GHz (6C/12T), `powersave` governor, 100K iterations per scenario.

### Intra-Process (same process, heap ring buffers)

| Scenario | Backend | p50 | p99 | p99.9 | Max |
|----------|---------|-----|-----|-------|-----|
| Same thread | DirectChannel | **23 ns** | 31 ns | 32 ns | 32 ns |
| 1 pub, 1 sub | SpscIntra | **10 ns** | 19 ns | 19 ns | 19 ns |
| 1 pub, N sub | SpmcIntra | **3 ns** | 10 ns | 11 ns | 11 ns |
| N pub, 1 sub (contended) | MpscIntra | **110 ns** | 426 ns | 528 ns | 578 ns |
| N pub, N sub (contended) | MpmcIntra | **100 ns** | 643 ns | 757 ns | 885 ns |

### Cross-Process (shared memory, RDTSC-in-payload one-way)

| Scenario | Backend | p50 | p99 | p99.9 | Notes |
|----------|---------|-----|-----|-------|-------|
| 1 pub, 2 sub | SpmcShm | **198 ns** | 298 ns | 308 ns | 31 ns framework overhead |
| 1 pub, 4 sub | SpmcShm | **236 ns** | 417 ns | 449 ns | Linear scaling |
| 1 pub, 8 sub | SpmcShm | **276 ns** | 510 ns | 549 ns | 8 consumers |
| 4 pub, 4 sub | PodShm | **304 ns** | 1.5 us | 6.5 us | Broadcast/latest-value |
| Raw SHM atomic | — | **167 ns** | 319 ns | 363 ns | Hardware floor |

### Robotics Messages (intra-process)

| Message | Size | Median | p99 |
|---------|------|--------|-----|
| CmdVel | 16 bytes | **20 ns** | 29 ns |

### Real-Time Suitability

All intra-process and SpmcShm cross-process paths are suitable for:

| Control Rate | Budget | Worst-Case Measured | Result |
|--------------|--------|---------------------|--------|
| 1 kHz | < 1 ms | 885 ns (MpmcIntra) | **PASS** |
| 10 kHz | < 100 us | 885 ns (MpmcIntra) | **PASS** |
| 100 kHz | < 10 us | 885 ns (MpmcIntra) | **PASS** |

## Methodology

- **Timing**: RDTSC with calibrated overhead subtraction (~29 ns overhead per measurement)
- **Outlier removal**: Tukey IQR fences (1.5x)
- **Confidence intervals**: Bootstrap 95% CI (10K resamples)
- **CPU pinning**: Producer and consumer pinned to separate physical cores
- **Warmup**: 5,000 iterations discarded before measurement
- **Cross-process**: RDTSC timestamp embedded in message payload for true one-way measurement

## For Accurate Results

```bash
# Set CPU governor to performance mode
sudo cpupower frequency-set -g performance

# Disable turbo boost for consistent results (optional)
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo
```

> Cross-process paced benchmarks (SpscShm, MpscShm, PodShm) are heavily affected by OS scheduling under `powersave` governor. The SpmcShm path (seqlock broadcast) is least affected since readers don't block.

## License

Apache-2.0
