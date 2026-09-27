# HORUS Benchmarks

Performance benchmarks for the HORUS robotics framework.

## Quick Start

```bash
cargo run --release -p horus_benchmarks --bin all_paths_latency
```

## Available Benchmarks

| Benchmark                     | What It Measures                                                        |
|-------------------------------|-------------------------------------------------------------------------|
| `all_paths_latency`           | Every intra- and cross-process topic path, with full statistical analysis |
| `cross_process_benchmark`     | True IPC between separate processes                                     |
| `robotics_messages_benchmark` | Real message types (CmdVel, Imu, LaserScan, JointCommand)               |
| `determinism_benchmark`       | Jitter and real-time suitability                                        |
| `scalability_benchmark`       | Multi-thread/multi-process scaling                                      |
| `competitor_comparison`       | HORUS vs raw UDP loopback (single-thread; see the file header)          |
| `topic_probe`                 | One POD path, ~2s — the fast loop for hot-path work                     |

Run any benchmark with:

```bash
cargo run --release -p horus_benchmarks --bin <name>
```

Criterion microbenchmarks:

```bash
cargo bench -p horus_benchmarks
```

## Measured Results

> **The previously published tables have been withdrawn, not updated.** They were
> wrong in two independent ways and no run on this machine can restore them:
>
> 1. **They named backends that do not exist.** The intra-process table listed
>    `DirectChannel`, `SpscIntra`, `SpmcIntra`, `MpscIntra` and `MpmcIntra` under
>    the heading "heap ring buffers". Those backends were removed. Every topic is
>    SHM-backed today — `Topic::backend_name()` can only return `PodShm`,
>    `SpscShm`, `SpmcShm`, `MpscShm`, `FanoutShm` or `Unknown` — so a real
>    `all_paths_latency` run prints SHM backends for those same scenarios and
>    nothing named `*Intra` exists to reproduce.
> 2. **Their tails were truncated by construction.** `Statistics::from_samples`
>    used to apply a Tukey IQR filter *before* computing `max`, `p99`, `p99.9` and
>    `p99.99`, so the reported worst case could not exceed `Q3 + 1.5*IQR` however
>    badly the machine stalled. The "Worst-Case Measured 885 ns" figure behind the
>    1 kHz / 10 kHz / 100 kHz **PASS** verdicts was a number the code guaranteed
>    could not be large. The filter now applies only to the mean and the
>    confidence interval; every order statistic comes from the full sample set.
>
> Regenerate with `cargo run --release -p horus_benchmarks --bin all_paths_latency`
> on a machine configured as described under *For Accurate Results* below, and
> publish what that run prints — including the scenario, backend and measurement
> columns it already emits.

### What `all_paths_latency` reports

The binary prints one row per scenario. The **measurement** column is the part
the withdrawn tables dropped, and it is not comparable across kinds:

| Scenario           | Backend   | Measurement | What the number is                                    |
|--------------------|-----------|-------------|-------------------------------------------------------|
| `SameThread`       | `SpscShm` | `send`      | Producer-side `Topic::send()` cost, RDTSC overhead subtracted |
| `CrossThread-1P1C` | `SpscShm` | `send`      | Producer-side `send()` cost                           |
| `CrossThread-MP1C` | `MpscShm` | `send`      | Producer-side `send()` cost under producer contention |
| `CrossThread-1PMC` | `PodShm`  | `send`      | Producer-side `send()` cost                           |
| `CrossThread-MPMC` | `PodShm`  | `send`      | Producer-side `send()` cost under contention          |
| `CrossProc-1P1C`   | `SpscShm` | `one-way`   | Pub-to-sub delivery, RDTSC timestamp in payload       |
| `CrossProc-2P1C`   | `MpscShm` | `one-way`   | Pub-to-sub delivery                                   |
| `CrossProc-1PMC`   | `PodShm`  | `one-way`   | Pub-to-sub delivery; one-to-many POD routes to broadcast |
| `CrossProc-PodShm` | `PodShm`  | `broadcast` | Latest-value broadcast; readers may skip ahead        |
| `RawAtomic`        | —         | `one-way`   | Raw SHM atomic store/load — the hardware floor        |

A `send`-measurement row is *not* an IPC latency: it is the cost of the store
into the shared-memory slot, with no consumer in the path. Only the `one-way`
rows are comparable to a DDS end-to-end latency figure. `all_paths_latency`
prints `MISMATCH` if the live backend differs from the expected one above, which
is the check that would have caught the withdrawn table.

## Methodology

- **Timing**: RDTSC with calibrated overhead subtraction (~29 ns overhead per measurement)
- **Outlier removal**: Tukey IQR fences (1.5x), applied to the **mean and
  confidence interval only**. `min`, `max`, the median and every percentile are
  computed from the full, unfiltered sample set — on a real-time path the tail
  *is* the measurement, and an OS preemption or page fault is the event that
  misses the deadline, not an artifact to discard.
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

> Cross-process paced benchmarks (SpscShm, MpscShm) are heavily affected by OS scheduling under `powersave` governor. The `PodShm` broadcast path is least affected, since its readers do not block — a one-to-many POD topology resolves to `PodShm`, not `SpmcShm`, which is why no row above names the latter.

### Host quietness: when a far tail is a transport figure

The `p99.9`/`p99.99` numbers in a run are only comparable when the process that
produced them could not be preempted. One involuntary context switch becomes a
burst of samples that waited for the scheduler: on 2026-09-23, three
repetitions of one trunk run measured `p99.9` at 27 µs / 1.9 µs / 16 µs with
*identical* medians and `p99`s. `cross_process_benchmark` records the evidence
next to the numbers (`measurement_quality` in the JSON: `rt_granted`,
`isolated_cores`, voluntary/involuntary context switches), and
`regression_gate` reports a far-tail excursion from a run without that evidence
as host-limited instead of failing a build on it. The medians and `p99`s gate
everywhere, because a preemption does not move them.

For numbers that are going to be published:

```bash
# 1. Isolate the cores you will pin to on the kernel command line, e.g. cores
#    2 and 3: `isolcpus=2,3` ... and reboot.

# 2. Ask for real-time priority (needs CAP_SYS_NICE or an rtprio limit;
#    `horus setup-rt` grants one).
cargo run --release -p horus_benchmarks --bin cross_process_benchmark -- \
  --cpus 2,3 --rt --require-quiet --json measured.json
```

`--require-quiet` exits non-zero unless every measured window was quiet, so a
run that publishes numbers has proven the host did not produce them. Without
it, a run on a shared machine is still a useful *delay* measurement — the
medians and `p99`s are unaffected by preemption — but its far tails describe
the host.

## License

Apache-2.0
