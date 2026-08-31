//! Cross-process measurement for the HORUS `Topic` shared-memory transport.
//!
//! This is the **only** cross-process benchmark in the tree. It replaces two
//! binaries that used to exist side by side and disagreed by a factor of twenty
//! (`cross_process_benchmark` measuring saturated-queue delay under the name
//! "latency", and `cross_process_oneway` measuring one-way latency correctly).
//!
//! # What changed, and what a downstream consumer has to do about it
//!
//! The old binary published a JSON metric named **`Topic_cross_process`** and
//! headlined it "TRUE inter-process communication measurement". It was not a
//! latency measurement. Its producer flooded in a tight loop (`yield_now()` once
//! every 1000 sends) while its consumer busy-polled one message per iteration.
//! `TimestampedMsg` was 64 B, so `auto_capacity` gave `PAGE_SIZE / 64 = 64`
//! slots, and every sample therefore sat behind a *full ring*. Its CI figure was
//! median 5914 ns / p99 12914 / CV 0.231, and `5914 / 64 = 92.4 ns` per message
//! — which is exactly the in-process per-message cost this tree measures
//! elsewhere (80 ns at 16 B, 160 ns at 1480 B). The low CV was not determinism;
//! it is the signature of a stable queue. It was **queueing delay**, and quoting
//! it as one-way latency overstated the transport's cost by ~20x.
//!
//! `Topic_cross_process` is therefore **retired, not redefined**. Nothing in
//! this binary emits that key any more. Three new keys take its place, each
//! named for what it actually measures:
//!
//! | JSON `name`                                | what it is |
//! |--------------------------------------------|------------|
//! | `Topic_cross_process_oneway_pingpong`      | one-way latency, offset-free (**the headline**) |
//! | `Topic_cross_process_oneway_paced_stream`  | one-way latency, direct, assumes TSC coherence |
//! | `Topic_cross_process_unpaced_queue_delay`  | publish-to-receive delay with an unthrottled publisher, queue time included. **NOT a latency.** |
//!
//! The third key says `unpaced`, not `saturated`, because whether the ring fills
//! is an outcome of the two loops' relative speeds and not something a benchmark
//! can assert — see [`KEY_UNPACED`]. The run measures the achieved backlog and
//! warns in both directions.
//!
//! Reusing `Topic_cross_process` for the corrected figure was the one option
//! ruled out from the start: every consumer of that key — the CI gate's rolling
//! baseline, `benchmarks/README.md`, any chart built from an uploaded artifact —
//! would have kept reading it and silently started comparing ~300 ns against a
//! window of ~5900 ns baselines. Retiring the key makes the change *loud*
//! instead: `RegressionPolicy::fail_on_missing_benchmark` is `true`, so the
//! first gate run after this lands reports `Topic_cross_process@64B` under
//! "benchmarks in the baseline that this run did not produce" and fails, with
//! instructions to refresh the window. That failure is the feature. A silent
//! meaning-swap is the thing it prevents.
//!
//! # Methodology
//!
//! Two real OS processes, pinned to two **distinct physical cores**, exchanging
//! POD messages over the same `Topic` SHM ring a robot node graph would use,
//! with every handle holding exactly **one role** so the `role == Both`
//! same-thread fast path is unreachable. That fast path requires one handle to
//! both publish and subscribe on one thread — a shape that exists in benchmarks
//! and in no real node graph — and it is where this repo's headline send-cost
//! figure comes from. `benchmarks/README.md` says of that row that it "is *not*
//! an IPC latency: it is the cost of the store into the shared-memory slot, with
//! no consumer in the path", which is why nothing here goes near it.
//!
//! ## Clock
//!
//! Both processes read the same hardware TSC. Rather than assume the two cores'
//! TSCs are offset-free, the ping-pong scenario records four timestamps per
//! sample and decomposes them the way NTP does:
//!
//! ```text
//!   t0       (clock A) parent stamps, then publishes on `ping`
//!   tb_recv  (clock B) child's recv() returns the message
//!   tb_send  (clock B) child stamps, then publishes on `pong`
//!   t1       (clock A) parent's recv() returns the reply
//!
//!   fwd = tb_recv - t0      = D_fwd + theta       (theta = clock_B - clock_A)
//!   rev = t1      - tb_send = D_rev - theta
//!   rtt = t1      - t0      = D_fwd + turnaround + D_rev   (one clock, exact)
//!
//!   one-way (offset-free) = (fwd + rev) / 2 = (D_fwd + D_rev) / 2
//!   theta_hat             = (median(fwd) - median(rev)) / 2
//! ```
//!
//! `(fwd + rev) / 2` cancels `theta` exactly and needs no assumption about TSC
//! synchronisation across cores. It is the headline one-way figure. `rtt / 2` is
//! printed alongside it, explicitly labelled as a halved round trip; it is the
//! larger of the two because it also carries the responder's recv->send
//! turnaround. Which one to compare against a competitor depends on that
//! competitor's methodology, so both are printed and neither is hidden.
//!
//! The stream scenarios measure one way directly (publisher stamps, subscriber
//! stamps on arrival) and therefore *do* depend on cross-core TSC coherence; the
//! `theta_hat` measured above bounds that bias and is printed with them.
//!
//! The old binary used `SystemTime::now()` — `CLOCK_REALTIME`, which NTP is free
//! to slew or step underneath a running measurement, and which produced samples
//! floored at exactly 0 ns through `saturating_sub` whenever it stepped
//! backwards. Everything here is TSC.
//!
//! ## What is included and never subtracted
//!
//! No clock-instrumentation overhead is subtracted from any sample. The measured
//! cost of the `serialize(); rdtsc()` / `rdtscp()` pattern is printed instead, so
//! a reader can see how much of the figure is instrumentation. Subtracting a
//! once-captured minimum from every later sample (which `all_paths_latency`
//! does) is what made these binaries' numbers non-comparable, and it biases the
//! result in the flattering direction: when the core drops to a lower P-state
//! the subtraction under-corrects, and when a sample beats the calibrated
//! minimum it over-corrects and floors the sample at zero.
//!
//! No outlier filtering anywhere, either. Tukey's fence deletes exactly the
//! preemptions and page faults that constitute the tail, and **the tail is this
//! project's figure of merit**; a "worst case" that cannot exceed `Q3 + 1.5·IQR`
//! is not a worst case.
//!
//! # Running
//!
//! ```bash
//! cargo build --release -p horus_benchmarks --bin cross_process_benchmark
//! ./target/release/cross_process_benchmark
//! ./target/release/cross_process_benchmark --iterations 4000000 --json out.json
//! ./target/release/cross_process_benchmark --iterations 20000 --warmup 2000   # smoke test
//! ```

use horus_benchmarks::timing::{calibrate_rdtsc, rdtsc, rdtscp, serialize, RdtscCalibration};
use horus_benchmarks::{
    calculate_percentile, coefficient_of_variation, detect_platform, set_cpu_affinity,
    write_json_report, BenchmarkConfig, BenchmarkReport, BenchmarkResult, DeterminismMetrics,
    PlatformInfo, Provenance, Statistics, ThroughputMetrics,
};
use horus_core::communication::{Topic, TopicMessage};
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;
use std::process::{Child, Command};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

// ============================================================================
// JSON metric keys
//
// The CI gate lines results up across runs by `format!("{name}@{message_size}B")`
// (`BaselineEntry::key`). These three constants ARE that contract. Changing one
// retires the old key and starts a new baseline window; see the module header
// for why that is the intended behaviour rather than a hazard to be avoided.
// ============================================================================

/// One-way latency, offset-free. The figure to compare against a competitor's
/// published pub->sub number (iceoryx2: ~240 ns at 64 B on a tuned i7-10875H).
const KEY_ONEWAY_PINGPONG: &str = "Topic_cross_process_oneway_pingpong";

/// One-way latency measured directly on a paced stream. Depends on cross-core
/// TSC coherence, which the ping-pong row does not.
const KEY_ONEWAY_STREAM: &str = "Topic_cross_process_oneway_paced_stream";

/// Publish-to-receive delay with the publisher running **unpaced**, so any time
/// the message spent waiting in the ring is included. A queueing/backpressure
/// figure, **not** a latency.
///
/// Named `unpaced` rather than `saturated` on purpose. Whether the ring actually
/// fills is an *outcome* of the two loops' relative speeds, not something the
/// benchmark can assert: on the box this was developed on the subscriber keeps
/// up and the measured backlog is ~1 message, while the retired
/// `Topic_cross_process` (64-slot ring, `SystemTime` clock on both ends) ran ~64
/// messages deep. A key that says "saturated" while reporting an unsaturated run
/// is the same species of lie this file exists to remove, so the name states the
/// *input* (the publisher is unpaced) and the run reports the *outcome* (the
/// measured backlog, plus a warning when it is below 2 messages).
const KEY_UNPACED: &str = "Topic_cross_process_unpaced_queue_delay";

// ============================================================================
// Configuration
// ============================================================================

/// Measured samples for the ping-pong scenarios. p99.99 rests on `n / 10_000`
/// observations, and that count is printed on every row. +/-5% on p99.99 needs
/// ~400 exceedances, i.e. `--iterations 4000000`.
const DEFAULT_ITERATIONS: usize = 1_000_000;

/// Discarded iterations before measurement. Must exceed 256: `resolve_owner`
/// retries node-name resolution on every send until it has failed 256 times, so
/// a benchmark that warms up for less than that measures the retry too.
const DEFAULT_WARMUP: usize = 200_000;

/// Measured samples for the two stream scenarios.
///
/// Lower than `DEFAULT_ITERATIONS` because the paced stream costs one publish
/// *period* per sample, not one round trip: at the default 10 us pace, a million
/// samples is ten seconds of wall clock per payload size, per repetition, and CI
/// runs three repetitions. 200k keeps p99.9 well supported (200 exceedances) and
/// clears the gate's 100k floor for p99.99 — where it rests on 20 observations,
/// which the run warns about rather than hides. Raise it with
/// `--stream-iterations` when the stream tail is the question being asked.
const DEFAULT_STREAM_ITERATIONS: usize = 200_000;

/// Ring slots, for every scenario. Power of two (validated by
/// `Topic::with_capacity`).
///
/// Stated explicitly rather than left to `auto_capacity`, which derives depth
/// from payload size (`PAGE_SIZE / size_of::<T>()`, clamped to 16..1024). Under
/// `auto_capacity` the 64 B and 1024 B rows would run on 64- and 16-slot rings
/// respectively, so the saturated row's headline number would silently be a
/// function of the payload size rather than of the transport. Ring depth is a
/// parameter of the experiment and is printed with every result.
const RING_CAPACITY: u32 = 256;

/// Publish interval for the paced-stream scenario. Flooding measures queueing
/// delay, not wire latency — `all_paths_latency.rs:905` documents the same
/// effect as "~3 us instead of true wire latency ~300 ns", and the binary this
/// file replaces shipped that mistake as its headline for months.
const DEFAULT_PACE_NS: u64 = 10_000;

/// Consecutive empty polls before the loop consults a wall clock.
///
/// The poll loop must not read a clock on its fast path: an `Instant::now()`
/// there lands squarely in the p99 band of the very distribution being measured.
/// But a bare spin *count* is not a timeout — on a machine where the peer never
/// got scheduled it is minutes of wedged CI. So the count is the fast path and
/// the clock is consulted only once every `IDLE_CLOCK_CHECK` *consecutive* empty
/// polls, i.e. after ~0.5 ms of unbroken starvation that no healthy sample ever
/// reaches. A successful poll resets the counter, so a measured sample never
/// pays for this.
const IDLE_CLOCK_CHECK: u64 = 1 << 16;

/// Wall-clock ceiling on a single blocked poll in the measuring process.
const IDLE_TIMEOUT: Duration = Duration::from_secs(20);

/// Wall-clock ceiling on a single blocked poll in a child. Longer than
/// [`IDLE_TIMEOUT`]: a child is spawned before the parent has attached to the
/// shared memory and must outlive the handshake.
const CHILD_IDLE_TIMEOUT: Duration = Duration::from_secs(90);

/// Confidence level for the interval published in `Statistics`.
const CONFIDENCE_LEVEL: f64 = 95.0;

/// Two-sided standard-normal quantile at [`CONFIDENCE_LEVEL`].
const Z_95: f64 = 1.959_963_985;

/// Deadline used for the `deadline_misses` counter.
///
/// 10 us is a real-time control-loop budget and is the threshold the retired
/// binary used, kept so that number stays comparable. For the saturated row it
/// counts messages that waited more than 10 us *in the queue*, which is a
/// backlog statistic and not a missed deadline; the row is labelled accordingly.
const DEADLINE_NS: u64 = 10_000;

const SEQ_STOP: u64 = u64::MAX;
const SEQ_HELLO: u64 = u64::MAX - 1;

// ============================================================================
// Probe messages
// ============================================================================

/// A fixed-size POD probe message.
///
/// `Copy`, `!Drop`, `size_of > 1`, so `communication::pod::is_pod` classifies it
/// POD and the topic takes the raw shared-ring path rather than the serde path.
/// The serde derives exist only to satisfy the `TopicMessage` blanket impl's
/// bounds; they are not on the data path for these types.
trait Probe: Copy + Clone + Send + Sync + Serialize + DeserializeOwned + 'static {
    /// Wire size in bytes.
    const BYTES: usize;
    /// Type name, printed with every row.
    const TYPE_NAME: &'static str;

    fn new_probe() -> Self;
    fn seq(&self) -> u64;
    fn set_seq(&mut self, v: u64);
    fn t0(&self) -> u64;
    fn set_t0(&mut self, v: u64);
    fn tb_recv(&self) -> u64;
    fn set_tb_recv(&mut self, v: u64);
    fn tb_send(&self) -> u64;
    fn set_tb_send(&mut self, v: u64);
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct Probe64 {
    seq: u64,
    t0: u64,
    tb_recv: u64,
    tb_send: u64,
    payload: [u8; 32],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct Probe1k {
    seq: u64,
    t0: u64,
    tb_recv: u64,
    tb_send: u64,
    #[serde(with = "serde_arrays")]
    payload: [u8; 992],
}

macro_rules! impl_probe {
    ($t:ty, $bytes:expr, $name:expr, $pad:expr) => {
        impl Probe for $t {
            const BYTES: usize = $bytes;
            const TYPE_NAME: &'static str = $name;

            #[inline(always)]
            fn new_probe() -> Self {
                Self {
                    seq: 0,
                    t0: 0,
                    tb_recv: 0,
                    tb_send: 0,
                    payload: [0u8; $pad],
                }
            }
            #[inline(always)]
            fn seq(&self) -> u64 {
                self.seq
            }
            #[inline(always)]
            fn set_seq(&mut self, v: u64) {
                self.seq = v;
            }
            #[inline(always)]
            fn t0(&self) -> u64 {
                self.t0
            }
            #[inline(always)]
            fn set_t0(&mut self, v: u64) {
                self.t0 = v;
            }
            #[inline(always)]
            fn tb_recv(&self) -> u64 {
                self.tb_recv
            }
            #[inline(always)]
            fn set_tb_recv(&mut self, v: u64) {
                self.tb_recv = v;
            }
            #[inline(always)]
            fn tb_send(&self) -> u64 {
                self.tb_send
            }
            #[inline(always)]
            fn set_tb_send(&mut self, v: u64) {
                self.tb_send = v;
            }
        }
    };
}

impl_probe!(Probe64, 64, "Probe64", 32);
impl_probe!(Probe1k, 1024, "Probe1k", 992);

/// Mirrors `communication::pod::is_pod`, which decides whether the topic takes
/// the raw-ring path or the serde path.
fn is_pod_like<M: 'static>() -> bool {
    !std::mem::needs_drop::<M>() && std::mem::size_of::<M>() > 1
}

// ============================================================================
// Machine facts — the settings every reported nanosecond is conditional on
// ============================================================================

fn read_trim(path: &str) -> Option<String> {
    fs::read_to_string(path).ok().map(|s| s.trim().to_string())
}

fn read_u64(path: &str) -> Option<u64> {
    read_trim(path).and_then(|s| s.parse().ok())
}

fn cpufreq(cpu: usize, leaf: &str) -> String {
    format!("/sys/devices/system/cpu/cpu{cpu}/cpufreq/{leaf}")
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct MachineConfig {
    governor_a: Option<String>,
    governor_b: Option<String>,
    scaling_driver: Option<String>,
    scaling_min_khz: Option<u64>,
    scaling_max_khz: Option<u64>,
    cpuinfo_max_khz: Option<u64>,
    turbo_disabled: Option<bool>,
    clocksource: Option<String>,
    tsc_flags: Vec<String>,
    transparent_hugepage: Option<String>,
    sched_rt_runtime_us: Option<i64>,
    sched_rt_period_us: Option<i64>,
    perf_event_paranoid: Option<i64>,
    isolcpus: Option<String>,
    nohz_full: Option<String>,
    smt_active: Option<String>,
}

fn detect_machine_config(cpu_a: usize, cpu_b: usize) -> MachineConfig {
    let cmdline = read_trim("/proc/cmdline").unwrap_or_default();
    let extract = |key: &str| -> Option<String> {
        cmdline
            .split_whitespace()
            .find_map(|t| t.strip_prefix(key).map(|v| v.to_string()))
    };
    let tsc_flags: Vec<String> = fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| s.lines().find(|l| l.starts_with("flags")).map(String::from))
        .map(|l| {
            l.split_whitespace()
                .filter(|f| {
                    matches!(
                        *f,
                        "tsc" | "constant_tsc" | "nonstop_tsc" | "tsc_reliable" | "rdtscp"
                    )
                })
                .map(String::from)
                .collect()
        })
        .unwrap_or_default();

    // `intel_pstate/no_turbo` on Intel, `cpufreq/boost` on the generic driver.
    let turbo_disabled = read_u64("/sys/devices/system/cpu/intel_pstate/no_turbo")
        .map(|v| v == 1)
        .or_else(|| read_u64("/sys/devices/system/cpu/cpufreq/boost").map(|v| v == 0));

    MachineConfig {
        governor_a: read_trim(&cpufreq(cpu_a, "scaling_governor")),
        governor_b: read_trim(&cpufreq(cpu_b, "scaling_governor")),
        scaling_driver: read_trim(&cpufreq(cpu_a, "scaling_driver")),
        scaling_min_khz: read_u64(&cpufreq(cpu_a, "scaling_min_freq")),
        scaling_max_khz: read_u64(&cpufreq(cpu_a, "scaling_max_freq")),
        cpuinfo_max_khz: read_u64(&cpufreq(cpu_a, "cpuinfo_max_freq")),
        turbo_disabled,
        clocksource: read_trim("/sys/devices/system/clocksource/clocksource0/current_clocksource"),
        tsc_flags,
        transparent_hugepage: read_trim("/sys/kernel/mm/transparent_hugepage/enabled"),
        sched_rt_runtime_us: read_trim("/proc/sys/kernel/sched_rt_runtime_us")
            .and_then(|s| s.parse().ok()),
        sched_rt_period_us: read_trim("/proc/sys/kernel/sched_rt_period_us")
            .and_then(|s| s.parse().ok()),
        perf_event_paranoid: read_trim("/proc/sys/kernel/perf_event_paranoid")
            .and_then(|s| s.parse().ok()),
        isolcpus: extract("isolcpus="),
        nohz_full: extract("nohz_full="),
        smt_active: read_trim("/sys/devices/system/cpu/smt/active"),
    }
}

// ============================================================================
// Core frequency: sampled throughout, not assumed
// ============================================================================
//
// RDTSC counts at the nominal TSC rate on `constant_tsc` regardless of P-state,
// so a TSC-derived nanosecond is a correct WALL-CLOCK nanosecond — but the work
// costs a fixed number of CORE cycles, so every reported ns is
// `core_cycles / f_core`, and `f_core` is never recorded anywhere else in this
// harness: `platform.rs` declares `measured_freq_mhz: None, // Set later by
// calibration` and nothing sets it. On both intel_pstate and acpi-cpufreq,
// `scaling_cur_freq` is derived from APERF/MPERF and does track the real core
// frequency, so it is sampled throughout each measured window here rather than
// read once, and the sampled record is printed with the results.

#[derive(Debug, Clone, Serialize, Deserialize)]
struct FreqRecord {
    cpu: usize,
    start_khz: Option<u64>,
    end_khz: Option<u64>,
    min_khz: u64,
    max_khz: u64,
    mean_khz: f64,
    samples: usize,
}

impl FreqRecord {
    fn spread_frac(&self) -> f64 {
        if self.mean_khz <= 0.0 {
            return 0.0;
        }
        self.max_khz.saturating_sub(self.min_khz) as f64 / self.mean_khz
    }
    /// Two samples is the minimum that can show movement at all; below that the
    /// record says nothing and must not raise an alarm.
    fn moved(&self) -> bool {
        self.samples >= 2 && self.spread_frac() > 0.02
    }
}

struct FreqSampler {
    stop: Arc<AtomicBool>,
    gate: Arc<AtomicBool>,
    handle: std::thread::JoinHandle<(Vec<u64>, Vec<u64>)>,
    cpu_a: usize,
    cpu_b: usize,
}

impl FreqSampler {
    /// Sample both benchmark cores every 5 ms from a third core, so the
    /// sampler's own `read(2)`s never land on a core under measurement.
    ///
    /// Samples are recorded only while the gate is open. The gate is opened by
    /// the measuring process immediately before its measured loop and closed
    /// immediately after, so process spawn, SHM attach, the handshake and warmup
    /// — during which a core legitimately sits at its idle P-state — cannot
    /// masquerade as a frequency excursion during measurement.
    fn start(cpu_a: usize, cpu_b: usize, sampler_cpu: usize) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let gate = Arc::new(AtomicBool::new(false));
        let s = stop.clone();
        let g = gate.clone();
        let pa = cpufreq(cpu_a, "scaling_cur_freq");
        let pb = cpufreq(cpu_b, "scaling_cur_freq");
        let handle = std::thread::spawn(move || {
            let _ = set_cpu_affinity(sampler_cpu);
            let mut va = Vec::with_capacity(8192);
            let mut vb = Vec::with_capacity(8192);
            while !s.load(Ordering::Relaxed) {
                if g.load(Ordering::Relaxed) {
                    if let Some(v) = read_u64(&pa) {
                        va.push(v);
                    }
                    if let Some(v) = read_u64(&pb) {
                        vb.push(v);
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            (va, vb)
        });
        Self {
            stop,
            gate,
            handle,
            cpu_a,
            cpu_b,
        }
    }

    fn gate(&self) -> Arc<AtomicBool> {
        self.gate.clone()
    }

    fn finish(self) -> (FreqRecord, FreqRecord) {
        self.gate.store(false, Ordering::Relaxed);
        self.stop.store(true, Ordering::Relaxed);
        let (va, vb) = self.handle.join().unwrap_or_default();
        let build = |cpu: usize, v: &[u64]| FreqRecord {
            cpu,
            start_khz: v.first().copied(),
            end_khz: v.last().copied(),
            min_khz: v.iter().copied().min().unwrap_or(0),
            max_khz: v.iter().copied().max().unwrap_or(0),
            mean_khz: if v.is_empty() {
                0.0
            } else {
                v.iter().map(|&x| x as f64).sum::<f64>() / v.len() as f64
            },
            samples: v.len(),
        };
        (build(self.cpu_a, &va), build(self.cpu_b, &vb))
    }
}

// ============================================================================
// Kernel-visible perturbation counters
// ============================================================================

/// Minor faults, major faults, and voluntary/involuntary context switches,
/// reported as a delta across the measured window.
///
/// A pre-faulted buffer that still takes minor faults during measurement is a
/// benchmark bug, not a property of the transport, and this is how a reader
/// finds out. Involuntary context switches are the mechanism behind most of the
/// far tail on a box without CPU isolation; a `max` quoted without them is not
/// interpretable.
///
/// SCOPE CAVEAT: `RUSAGE_SELF` is process-wide, so this also counts the
/// frequency-sampler thread, whose two `read_to_string` calls every 5 ms
/// allocate and therefore contribute a handful of minor faults per measured
/// window (observed: 1-6 per million samples). Those are NOT faults on the
/// measured path. Narrowing this to the measuring thread needs `RUSAGE_THREAD`,
/// which the `libc` crate does not define for `linux-gnu` — it would have to be
/// declared here as the raw value 1. Until then, read a single-digit `minflt`
/// as "the sampler", and anything larger as a genuine missed pre-fault.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
struct Perturbation {
    minflt: u64,
    majflt: u64,
    nvcsw: u64,
    nivcsw: u64,
}

impl Perturbation {
    /// Whether these counters mean anything on this platform. Off Unix there is
    /// no `getrusage`, so every field is zero — which without this flag reads as
    /// a perfectly quiet machine instead of an unmeasured one.
    const fn available() -> bool {
        cfg!(unix)
    }
}

#[cfg(unix)]
fn rusage_now() -> Perturbation {
    // SAFETY: `ru` is a zeroed, correctly sized `rusage` on this thread's stack
    // and `getrusage` only writes into it; RUSAGE_SELF is a valid resource id.
    unsafe {
        let mut ru: libc::rusage = std::mem::zeroed();
        if libc::getrusage(libc::RUSAGE_SELF, &mut ru) != 0 {
            return Perturbation::default();
        }
        Perturbation {
            minflt: ru.ru_minflt as u64,
            majflt: ru.ru_majflt as u64,
            nvcsw: ru.ru_nvcsw as u64,
            nivcsw: ru.ru_nivcsw as u64,
        }
    }
}

/// `getrusage` is Unix-only; `libc` does not define it, `rusage`, or
/// `RUSAGE_SELF` for the MSVC target, which is why this binary did not compile
/// on Windows at all. There is no Windows equivalent that counts the same
/// events, so the counters are reported as unavailable rather than as zero —
/// see [`Perturbation::available`], which the printer consults so a Windows run
/// cannot be read as "no faults, no preemptions".
#[cfg(not(unix))]
fn rusage_now() -> Perturbation {
    Perturbation::default()
}

fn perturbation_delta(a: Perturbation, b: Perturbation) -> Perturbation {
    Perturbation {
        minflt: b.minflt.saturating_sub(a.minflt),
        majflt: b.majflt.saturating_sub(a.majflt),
        nvcsw: b.nvcsw.saturating_sub(a.nvcsw),
        nivcsw: b.nivcsw.saturating_sub(a.nivcsw),
    }
}

/// Lock currently-mapped pages. `MCL_CURRENT` only: `MCL_FUTURE` converts a swap
/// event into an OOM kill, which is a trade a benchmark has no business making
/// on the user's behalf.
#[cfg(unix)]
fn try_mlock_current() -> bool {
    // SAFETY: `mlockall` takes only a flags word and has no memory-safety
    // preconditions; failure is reported through the return value.
    unsafe { libc::mlockall(libc::MCL_CURRENT) == 0 }
}

/// No `mlockall` off Unix. `false` is the honest answer to "are the pages
/// locked": they are not.
#[cfg(not(unix))]
fn try_mlock_current() -> bool {
    false
}

/// Allocate and touch every element, so every page is resident before the first
/// timestamp is taken.
///
/// `vec![0u64; n]` goes through `calloc`, which for a large allocation hands
/// back lazily-zeroed pages: the faults then land inside the measured window.
/// The research traced ~196 minor faults from an 800 KB lazily-faulted `Vec`
/// straight into the cross-process p99.9 band.
fn prefaulted(n: usize) -> Vec<u64> {
    let mut v = vec![0u64; n];
    for (i, slot) in v.iter_mut().enumerate() {
        *slot = i as u64 | 1;
    }
    std::hint::black_box(&v);
    for slot in v.iter_mut() {
        *slot = 0;
    }
    v
}

/// A wall-clock deadline that is only *created* once a poll loop has already
/// spun [`IDLE_CLOCK_CHECK`] times without receiving anything.
///
/// Keeps `Instant::now()` off the measured fast path (see [`IDLE_CLOCK_CHECK`]).
struct IdleGuard {
    idle: u64,
    deadline: Option<Instant>,
    timeout: Duration,
}

impl IdleGuard {
    fn new(timeout: Duration) -> Self {
        Self {
            idle: 0,
            deadline: None,
            timeout,
        }
    }

    /// Call on a successful receive.
    #[inline(always)]
    fn reset(&mut self) {
        self.idle = 0;
        self.deadline = None;
    }

    /// Call on an empty poll. Returns `true` when the loop should give up.
    #[inline(always)]
    fn empty_poll(&mut self) -> bool {
        self.idle += 1;
        if self.idle.is_multiple_of(IDLE_CLOCK_CHECK) {
            let now = Instant::now();
            match self.deadline {
                None => self.deadline = Some(now + self.timeout),
                Some(d) => return now > d,
            }
        }
        false
    }
}

// ============================================================================
// Distributions
// ============================================================================

/// Order statistics over the FULL sample set, with the exceedance count behind
/// each tail figure.
///
/// No outlier filtering — see the module header.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Dist {
    n: usize,
    min_c: u64,
    median_c: u64,
    p95_c: u64,
    p99_c: u64,
    p999_c: u64,
    p9999_c: u64,
    max_c: u64,
    mean_c: f64,
    stdev_c: f64,
    n_gt_p99: usize,
    n_gt_p999: usize,
    n_gt_p9999: usize,
}

/// `sorted` must be ascending.
fn dist_from_sorted(sorted: &[u64]) -> Dist {
    if sorted.is_empty() {
        return Dist {
            n: 0,
            min_c: 0,
            median_c: 0,
            p95_c: 0,
            p99_c: 0,
            p999_c: 0,
            p9999_c: 0,
            max_c: 0,
            mean_c: 0.0,
            stdev_c: 0.0,
            n_gt_p99: 0,
            n_gt_p999: 0,
            n_gt_p9999: 0,
        };
    }
    let n = sorted.len();
    let mean = sorted.iter().map(|&x| x as f64).sum::<f64>() / n as f64;
    let var = sorted
        .iter()
        .map(|&x| {
            let d = x as f64 - mean;
            d * d
        })
        .sum::<f64>()
        / n as f64;
    let p99 = calculate_percentile(sorted, 99.0);
    let p999 = calculate_percentile(sorted, 99.9);
    let p9999 = calculate_percentile(sorted, 99.99);
    let gt = |v: u64| n - sorted.partition_point(|&x| x <= v);
    Dist {
        n,
        min_c: sorted[0],
        median_c: calculate_percentile(sorted, 50.0),
        p95_c: calculate_percentile(sorted, 95.0),
        p99_c: p99,
        p999_c: p999,
        p9999_c: p9999,
        max_c: sorted[n - 1],
        mean_c: mean,
        stdev_c: var.sqrt(),
        n_gt_p99: gt(p99),
        n_gt_p999: gt(p999),
        n_gt_p9999: gt(p9999),
    }
}

/// Materialise `f(i)` for `i in 0..n` into `scratch`, sort it, and summarise.
///
/// Leaves `scratch[..n]` holding the ascending sample set, which callers use to
/// capture the gate projection without re-sorting.
fn dist_of<F: FnMut(usize) -> u64>(n: usize, scratch: &mut [u64], mut f: F) -> Dist {
    for (i, slot) in scratch.iter_mut().enumerate().take(n) {
        *slot = f(i);
    }
    let s = &mut scratch[..n];
    s.sort_unstable();
    dist_from_sorted(s)
}

/// Cycles to nanoseconds. Strictly monotone, so applying it elementwise to an
/// already-sorted array preserves every order statistic exactly — which is why
/// the gate projection can sort in cycles and convert afterwards.
#[inline]
fn cyc_to_ns(c: u64, ns_per_cycle: f64) -> u64 {
    (c as f64 * ns_per_cycle).round() as u64
}

/// Dump raw per-sample cycle counts so a percentile can be re-derived later.
///
/// The primary harness writes `raw_latencies_ns: Vec::new()` with the comment
/// "Exclude raw latencies to keep JSON compact", which makes every result in it
/// unfalsifiable from the artefact it produced. `--raw-dir` is this binary's
/// answer, and it is lossless: one little-endian `u64` per sample, in TSC
/// cycles, before any conversion or rounding.
fn dump_raw(dir: Option<&str>, name: &str, v: &[u64]) {
    let Some(dir) = dir else { return };
    let _ = fs::create_dir_all(dir);
    let path = PathBuf::from(dir).join(format!("{name}.u64le"));
    if let Err(e) = fs::write(&path, bytemuck::cast_slice::<u64, u8>(v)) {
        eprintln!("warning: could not write {}: {e}", path.display());
    }
}

// ============================================================================
// Scenario results
// ============================================================================

/// What a scenario's headline row *is*, so nothing downstream has to infer it
/// from a name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum Kind {
    /// Lock-step round trip, decomposed into an offset-free one-way figure.
    PingPong,
    /// Paced publisher, direct one-way measurement.
    PacedStream,
    /// Unpaced publisher: publish-to-receive delay including queue time. NOT a
    /// latency.
    Unpaced,
}

impl Kind {
    fn slug(self) -> &'static str {
        match self {
            Kind::PingPong => "pingpong",
            Kind::PacedStream => "paced_stream",
            Kind::Unpaced => "unpaced_queue",
        }
    }

    /// The JSON metric key this scenario publishes.
    fn metric_key(self) -> &'static str {
        match self {
            Kind::PingPong => KEY_ONEWAY_PINGPONG,
            Kind::PacedStream => KEY_ONEWAY_STREAM,
            Kind::Unpaced => KEY_UNPACED,
        }
    }

    /// `BenchmarkResult::subject`. Written so that a row lifted out of the JSON
    /// and pasted into a slide still says what it is.
    fn subject(self) -> &'static str {
        match self {
            Kind::PingPong => {
                "HORUS Topic, cross-process ONE-WAY latency (2 processes, 2 physical cores, \
                 1 role per handle, offset-free NTP decomposition of a lock-step round trip)"
            }
            Kind::PacedStream => {
                "HORUS Topic, cross-process ONE-WAY latency (2 processes, 2 physical cores, \
                 paced publisher, direct publish->receive; assumes cross-core TSC coherence)"
            }
            Kind::Unpaced => {
                "HORUS Topic, cross-process PUBLISH-TO-RECEIVE DELAY with an UNPACED publisher \
                 (backpressure/throughput measurement -- THIS IS NOT A LATENCY FIGURE: each \
                 sample includes however long the message waited in the ring, so when the \
                 publisher outruns the subscriber it scales with ring depth rather than with \
                 the transport; read `measured backlog` in the run output to see which \
                 regime this run was in)"
            }
        }
    }

    fn is_latency(self) -> bool {
        !matches!(self, Kind::Unpaced)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ScenarioResult {
    /// Human-readable one-line description, printed above the table.
    scenario: String,
    kind: Kind,
    /// JSON metric key, duplicated here so the detail report and the gate report
    /// can be lined up without knowing the mapping.
    metric_key: String,
    payload_type: String,
    message_bytes: usize,
    is_pod: bool,
    ring_capacity: u32,
    topology: String,
    cpu_a: usize,
    cpu_b: usize,
    parent_pid: u32,
    child_pid: u32,
    iterations: usize,
    warmup: usize,
    /// Metric name -> distribution, in TSC cycles. The FIRST entry is the
    /// headline and is the one projected into the gate report.
    metrics: Vec<(String, Dist)>,
    /// The headline metric's samples, ascending, in nanoseconds.
    #[serde(skip)]
    headline_ns_sorted: Vec<u64>,
    /// Wall-clock duration of the measured window.
    window_secs: f64,
    /// Messages that actually crossed the transport and were accounted for:
    /// both directions for the ping-pong, delivered + detected-lost for a
    /// stream. NOT the publisher's total send count, which for an unpaced run
    /// keeps going after the subscriber has its samples.
    messages_transferred: u64,
    messages_lost: u64,
    invalid_samples: usize,
    theta_hat_cycles: f64,
    perturbation: Perturbation,
    freq_a: FreqRecord,
    freq_b: FreqRecord,
    empty_poll_cycles: u64,
}

impl ScenarioResult {
    /// How many messages deep the ring was, on the median sample.
    ///
    /// `median_delay / mean_inter_arrival`. This is the number that says whether
    /// an unpaced run actually built a backlog: ~1 means the subscriber kept up
    /// and the row is a delay measurement with no queueing in it, while ~`ring
    /// capacity` means the ring was full and the row is the backpressure figure
    /// the retired `Topic_cross_process` metric was accidentally reporting.
    ///
    /// `None` when the window or the sample count cannot support the ratio.
    fn measured_backlog_msgs(&self, ns_per_cycle: f64) -> Option<f64> {
        let (_, d) = self.metrics.first()?;
        if d.n == 0 || self.window_secs <= 0.0 {
            return None;
        }
        let inter_arrival_ns = self.window_secs * 1e9 / d.n as f64;
        if inter_arrival_ns <= 0.0 {
            return None;
        }
        Some(d.median_c as f64 * ns_per_cycle / inter_arrival_ns)
    }
}

// ============================================================================
// Scenario 1: cross-process ping-pong
// ============================================================================

struct PingPongRaw {
    fwd: Vec<u64>,
    turn: Vec<u64>,
    rev: Vec<u64>,
    invalid: usize,
    perturbation: Perturbation,
    empty_poll_cycles: u64,
    window_secs: f64,
}

/// Parent side: publishes on `ping`, subscribes to `pong`.
///
/// Two distinct topics, so this handle publishes on one and subscribes on the
/// other, and the `role == Both` inlined fast path is unreachable. That path is
/// where the repo's headline send-cost figure comes from, and it is why that
/// figure is not an IPC latency.
fn ping_pong_parent<M>(
    ping_name: &str,
    pong_name: &str,
    n: usize,
    warmup: usize,
    gate: &AtomicBool,
) -> Result<(PingPongRaw, u32, u32), String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let ping: Topic<M> = Topic::with_capacity(ping_name, RING_CAPACITY, None)
        .map_err(|e| format!("ping topic: {e}"))?;
    let pong: Topic<M> = Topic::with_capacity(pong_name, RING_CAPACITY, None)
        .map_err(|e| format!("pong topic: {e}"))?;

    // Sample buffers are faulted in before the first timestamp is taken.
    let mut fwd = prefaulted(n);
    let mut turn = prefaulted(n);
    let mut rev = prefaulted(n);

    let mut msg = M::new_probe();

    // Handshake, so the measured loop never includes the child's process
    // start-up or its SHM attach.
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut connected = false;
    while Instant::now() < deadline && !connected {
        msg.set_seq(SEQ_HELLO);
        msg.set_t0(rdtsc());
        ping.send(msg);
        let spin_end = Instant::now() + Duration::from_millis(50);
        while Instant::now() < spin_end {
            if let Some(r) = pong.recv() {
                if r.seq() == SEQ_HELLO {
                    connected = true;
                    break;
                }
            }
            std::hint::spin_loop();
        }
    }
    if !connected {
        return Err("echo child never responded".to_string());
    }
    while pong.recv().is_some() {}

    // The subscriber's empty-poll cost is the detection granularity of a
    // spin-polling consumer, and it sits inside every one-way figure below.
    let empty_poll_cycles = {
        const PROBES: u64 = 20_000;
        serialize();
        let s = rdtsc();
        for _ in 0..PROBES {
            std::hint::black_box(pong.recv());
        }
        let e = rdtscp();
        e.wrapping_sub(s) / PROBES
    };

    // One closure for warmup and measurement, so the measured code path is
    // byte-identical to the warmed one.
    let mut round = |seq: u64| -> Option<(u64, u64, u64)> {
        msg.set_seq(seq);
        msg.set_tb_recv(0);
        msg.set_tb_send(0);
        serialize();
        let t0 = rdtsc();
        msg.set_t0(t0);
        ping.send(msg);

        let mut idle = IdleGuard::new(IDLE_TIMEOUT);
        loop {
            if let Some(r) = pong.recv() {
                if r.seq() == seq {
                    let t1 = rdtscp();
                    return Some((
                        r.tb_recv().wrapping_sub(t0),
                        r.tb_send().wrapping_sub(r.tb_recv()),
                        t1.wrapping_sub(r.tb_send()),
                    ));
                }
                idle.reset();
                continue;
            }
            if idle.empty_poll() {
                return None;
            }
            std::hint::spin_loop();
        }
    };

    for i in 0..warmup {
        if round(i as u64).is_none() {
            return Err("echo child stopped responding during warmup".to_string());
        }
    }

    gate.store(true, Ordering::Relaxed);
    let before = rusage_now();
    let window_start = Instant::now();
    for i in 0..n {
        match round((warmup + i) as u64) {
            Some((f, tn, v)) => {
                fwd[i] = f;
                turn[i] = tn;
                rev[i] = v;
            }
            None => {
                gate.store(false, Ordering::Relaxed);
                return Err(format!("echo child stopped responding at sample {i}"));
            }
        }
    }
    let window_secs = window_start.elapsed().as_secs_f64();
    let after = rusage_now();
    gate.store(false, Ordering::Relaxed);

    // A cross-core TSC offset large enough to make the decomposition
    // meaningless shows up as a leg longer than the whole round trip.
    let mut invalid = 0usize;
    for i in 0..n {
        let rtt = fwd[i].wrapping_add(turn[i]).wrapping_add(rev[i]);
        if fwd[i] > rtt || rev[i] > rtt {
            invalid += 1;
        }
    }

    let (pubs, subs) = (ping.pub_count(), ping.sub_count());

    msg.set_seq(SEQ_STOP);
    for _ in 0..8 {
        ping.send(msg);
    }

    Ok((
        PingPongRaw {
            fwd,
            turn,
            rev,
            invalid,
            perturbation: perturbation_delta(before, after),
            empty_poll_cycles,
            window_secs,
        },
        pubs,
        subs,
    ))
}

/// Child side: subscribes to `ping`, publishes on `pong`. One role per handle.
fn echo_child<M>(ping_name: &str, pong_name: &str) -> Result<(), String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let ping: Topic<M> = Topic::with_capacity(ping_name, RING_CAPACITY, None)
        .map_err(|e| format!("ping topic: {e}"))?;
    let pong: Topic<M> = Topic::with_capacity(pong_name, RING_CAPACITY, None)
        .map_err(|e| format!("pong topic: {e}"))?;
    try_mlock_current();

    let mut idle = IdleGuard::new(CHILD_IDLE_TIMEOUT);
    loop {
        match ping.recv() {
            Some(mut m) => {
                let tb_recv = rdtscp();
                if m.seq() == SEQ_STOP {
                    return Ok(());
                }
                m.set_tb_recv(tb_recv);
                serialize();
                m.set_tb_send(rdtsc());
                pong.send(m);
                idle.reset();
            }
            None => {
                if idle.empty_poll() {
                    return Ok(());
                }
                std::hint::spin_loop();
            }
        }
    }
}

// ============================================================================
// Scenarios 2 and 3: one-way stream, paced or saturated
// ============================================================================

/// How the publisher child drives the stream.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Pacing {
    /// One publish every `pace_ns`, so the ring never systematically fills and
    /// the subscriber measures wire latency.
    Paced,
    /// Publish as fast as the transport accepts. The subscriber then measures
    /// publish-to-receive delay *including* queue time. Whether a backlog
    /// actually forms depends on the two loops' relative speeds and is reported,
    /// not assumed — see [`KEY_UNPACED`].
    Unpaced,
}

struct StreamRaw {
    lat: Vec<u64>,
    received: usize,
    lost: u64,
    perturbation: Perturbation,
    empty_poll_cycles: u64,
    window_secs: f64,
}

/// Parent side: subscriber only.
fn stream_parent<M>(
    topic_name: &str,
    n: usize,
    warmup: usize,
    gate: &AtomicBool,
) -> Result<(StreamRaw, u32, u32), String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let sub: Topic<M> = Topic::with_capacity(topic_name, RING_CAPACITY, None)
        .map_err(|e| format!("stream topic: {e}"))?;
    let mut lat = prefaulted(n);

    let deadline = Instant::now() + Duration::from_secs(30);
    let mut seen_any = false;
    while Instant::now() < deadline && !seen_any {
        if let Some(m) = sub.recv() {
            if m.seq() != SEQ_STOP {
                seen_any = true;
            }
        }
        std::hint::spin_loop();
    }
    if !seen_any {
        return Err("publisher child never produced a message".to_string());
    }

    let empty_poll_cycles = {
        const PROBES: u64 = 2_000;
        serialize();
        let s = rdtsc();
        for _ in 0..PROBES {
            std::hint::black_box(sub.try_recv());
        }
        let e = rdtscp();
        e.wrapping_sub(s) / PROBES
    };

    let mut received = 0usize;
    let mut lost = 0u64;
    let mut last: Option<u64> = None;
    let mut idle = IdleGuard::new(IDLE_TIMEOUT);
    let mut measuring = false;
    let mut before = rusage_now();
    let mut window_start = Instant::now();

    loop {
        match sub.recv() {
            Some(m) => {
                let t = rdtscp();
                idle.reset();
                let seq = m.seq();
                if seq == SEQ_STOP {
                    break;
                }
                if seq >= warmup as u64 {
                    if !measuring {
                        measuring = true;
                        gate.store(true, Ordering::Relaxed);
                        before = rusage_now();
                        window_start = Instant::now();
                        last = None;
                    }
                    if let Some(l) = last {
                        if seq > l + 1 {
                            lost += seq - l - 1;
                        }
                    }
                    last = Some(seq);
                    lat[received] = t.wrapping_sub(m.t0());
                    received += 1;
                    if received == n {
                        break;
                    }
                }
            }
            None => {
                if idle.empty_poll() {
                    break;
                }
                std::hint::spin_loop();
            }
        }
    }
    let window_secs = window_start.elapsed().as_secs_f64();
    let after = rusage_now();
    gate.store(false, Ordering::Relaxed);
    let (pubs, subs) = (sub.pub_count(), sub.sub_count());

    Ok((
        StreamRaw {
            lat,
            received,
            lost,
            perturbation: perturbation_delta(before, after),
            empty_poll_cycles,
            window_secs,
        },
        pubs,
        subs,
    ))
}

/// Child side: publisher only.
///
/// `pace_cycles == 0` floods; anything else paces in TSC cycles.
///
/// Pacing removes the *systematic* queueing, but not the episodic kind: if the
/// subscriber is descheduled for longer than `ring_capacity * pace`, the
/// publisher keeps filling the ring, and every message the subscriber then
/// drains reports how long it sat there rather than how long the wire took.
/// A measured run showed exactly this — median 201 ns, p99 1.3 us, then a cliff
/// to p99.9 = 2.7 ms with 3,225 messages lost, against a backlog ceiling of
/// `256 slots * 10 us = 2.56 ms`. `messages_lost` and the involuntary
/// context-switch count are printed so a reader can tell that shape apart from
/// a transport tail. The lock-step ping-pong cannot queue and is therefore the
/// more trustworthy tail estimator on a box without CPU isolation.
///
/// The flooding mode is the *same* mechanism run deliberately: it is what the
/// retired `Topic_cross_process` metric measured by accident.
fn pub_child<M>(topic_name: &str, total: usize, pace_cycles: u64) -> Result<(), String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let tx: Topic<M> = Topic::with_capacity(topic_name, RING_CAPACITY, None)
        .map_err(|e| format!("stream topic: {e}"))?;
    try_mlock_current();

    // Let the subscriber attach before the first message.
    std::thread::sleep(Duration::from_millis(300));

    let mut msg = M::new_probe();
    let mut next = rdtsc();
    for i in 0..total {
        if pace_cycles > 0 {
            while rdtsc() < next {
                std::hint::spin_loop();
            }
            next = next.wrapping_add(pace_cycles);
        }
        msg.set_seq(i as u64);
        serialize();
        msg.set_t0(rdtsc());
        tx.send(msg);
    }
    msg.set_seq(SEQ_STOP);
    for _ in 0..8 {
        tx.send(msg);
        std::thread::sleep(Duration::from_micros(200));
    }
    Ok(())
}

// ============================================================================
// Scenario drivers
// ============================================================================

fn spawn_child(args: &[String]) -> std::io::Result<Child> {
    let exe = std::env::current_exe()?;
    Command::new(exe).args(args).spawn()
}

fn reap(mut child: Child, secs: u64) {
    let deadline = Instant::now() + Duration::from_secs(secs);
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {
                if Instant::now() > deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    return;
                }
                std::thread::sleep(Duration::from_millis(2));
            }
            Err(_) => return,
        }
    }
}

struct RunCtx<'a> {
    run_id: &'a str,
    cpu_a: usize,
    cpu_b: usize,
    sampler_cpu: usize,
    iterations: usize,
    stream_iterations: usize,
    warmup: usize,
    stream_warmup: usize,
    ns_per_cycle: f64,
    raw_dir: Option<&'a str>,
}

fn run_pingpong<M>(ctx: &RunCtx<'_>, scratch: &mut [u64]) -> Result<ScenarioResult, String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let bytes = M::BYTES;
    let ping = format!("xpb.ping.{}.{}", bytes, ctx.run_id);
    let pong = format!("xpb.pong.{}.{}", bytes, ctx.run_id);
    let child = spawn_child(&[
        "--child".to_string(),
        "echo".to_string(),
        ctx.cpu_b.to_string(),
        bytes.to_string(),
        ping.clone(),
        pong.clone(),
    ])
    .map_err(|e| format!("spawn echo child: {e}"))?;
    let child_pid = child.id();

    let sampler = FreqSampler::start(ctx.cpu_a, ctx.cpu_b, ctx.sampler_cpu);
    let gate = sampler.gate();
    let outcome = ping_pong_parent::<M>(&ping, &pong, ctx.iterations, ctx.warmup, &gate);
    let (freq_a, freq_b) = sampler.finish();
    reap(child, 15);

    let (raw, pubs, subs) = outcome?;
    let n = ctx.iterations;

    dump_raw(ctx.raw_dir, &format!("pingpong.{bytes}.fwd"), &raw.fwd);
    dump_raw(
        ctx.raw_dir,
        &format!("pingpong.{bytes}.turnaround"),
        &raw.turn,
    );
    dump_raw(ctx.raw_dir, &format!("pingpong.{bytes}.rev"), &raw.rev);

    let fwd = &raw.fwd;
    let turn = &raw.turn;
    let rev = &raw.rev;

    // Headline first: `dist_of` leaves `scratch[..n]` sorted ascending, and the
    // cycles->ns map is monotone, so the gate projection needs no second sort.
    let d_oneway = dist_of(n, scratch, |i| fwd[i].wrapping_add(rev[i]) / 2);
    let headline_ns_sorted: Vec<u64> = scratch[..n]
        .iter()
        .map(|&c| cyc_to_ns(c, ctx.ns_per_cycle))
        .collect();

    let d_rtt = dist_of(n, scratch, |i| {
        fwd[i].wrapping_add(turn[i]).wrapping_add(rev[i])
    });
    // Halving a round trip is a monotone transform, so the same distribution
    // scaled by 0.5 at print time gives exact percentiles.
    let d_rtt_half = d_rtt.clone();
    let d_fwd = dist_of(n, scratch, |i| fwd[i]);
    let d_rev = dist_of(n, scratch, |i| rev[i]);
    let d_turn = dist_of(n, scratch, |i| turn[i]);

    let theta = (d_fwd.median_c as f64 - d_rev.median_c as f64) / 2.0;

    Ok(ScenarioResult {
        scenario:
            "cross-process ping-pong -- ONE-WAY LATENCY (2 processes, 2 topics, 1 role per handle)"
                .to_string(),
        kind: Kind::PingPong,
        metric_key: KEY_ONEWAY_PINGPONG.to_string(),
        payload_type: M::TYPE_NAME.to_string(),
        message_bytes: bytes,
        is_pod: is_pod_like::<M>(),
        ring_capacity: RING_CAPACITY,
        topology: format!("{pubs}P/{subs}C"),
        cpu_a: ctx.cpu_a,
        cpu_b: ctx.cpu_b,
        parent_pid: std::process::id(),
        child_pid,
        iterations: n,
        warmup: ctx.warmup,
        metrics: vec![
            ("one-way, offset-free".to_string(), d_oneway),
            ("round trip (RTT)".to_string(), d_rtt),
            ("RTT halved (one-way)".to_string(), d_rtt_half),
            ("leg A->B (apparent)".to_string(), d_fwd),
            ("leg B->A (apparent)".to_string(), d_rev),
            ("responder turnaround".to_string(), d_turn),
        ],
        headline_ns_sorted,
        window_secs: raw.window_secs,
        // Each round trip puts one message on `ping` and one on `pong`; both
        // crossed the transport. Lock-step, so this is NOT a throughput
        // capability figure — it is bounded by the round trip, not by the ring.
        messages_transferred: 2 * n as u64,
        messages_lost: 0,
        invalid_samples: raw.invalid,
        theta_hat_cycles: theta,
        perturbation: raw.perturbation,
        freq_a,
        freq_b,
        empty_poll_cycles: raw.empty_poll_cycles,
    })
}

fn run_stream<M>(
    ctx: &RunCtx<'_>,
    pacing: Pacing,
    pace_ns: u64,
    pace_cycles: u64,
    theta_hat: f64,
    scratch: &mut [u64],
) -> Result<ScenarioResult, String>
where
    M: Probe
        + TopicMessage<Wire = M>
        + Clone
        + Send
        + Sync
        + Serialize
        + DeserializeOwned
        + 'static,
{
    let kind = match pacing {
        Pacing::Paced => Kind::PacedStream,
        Pacing::Unpaced => Kind::Unpaced,
    };
    let bytes = M::BYTES;
    let topic = format!("xpb.{}.{}.{}", kind.slug(), bytes, ctx.run_id);
    let n_req = ctx.stream_iterations;
    // Head-room over the requested sample count: when an unpaced publisher does
    // outrun the subscriber the subscriber loses messages, so `total` must
    // exceed `n` or the run ends before `n` samples have been delivered.
    let total = match pacing {
        Pacing::Paced => n_req + ctx.stream_warmup + 1024,
        Pacing::Unpaced => (n_req + ctx.stream_warmup) * 4 + 1024,
    };
    let effective_pace = match pacing {
        Pacing::Paced => pace_cycles.max(1),
        Pacing::Unpaced => 0,
    };
    let child = spawn_child(&[
        "--child".to_string(),
        "pub".to_string(),
        ctx.cpu_b.to_string(),
        bytes.to_string(),
        topic.clone(),
        total.to_string(),
        effective_pace.to_string(),
    ])
    .map_err(|e| format!("spawn publisher child: {e}"))?;
    let child_pid = child.id();

    let sampler = FreqSampler::start(ctx.cpu_a, ctx.cpu_b, ctx.sampler_cpu);
    let gate = sampler.gate();
    let outcome = stream_parent::<M>(&topic, n_req, ctx.stream_warmup, &gate);
    let (freq_a, freq_b) = sampler.finish();
    reap(child, 30);

    let (raw, pubs, subs) = outcome?;
    let n = raw.received;
    dump_raw(
        ctx.raw_dir,
        &format!("{}.{bytes}.oneway", kind.slug()),
        &raw.lat[..n],
    );

    let lat = &raw.lat;
    let d = dist_of(n, scratch, |i| lat[i]);
    let headline_ns_sorted: Vec<u64> = scratch[..n]
        .iter()
        .map(|&c| cyc_to_ns(c, ctx.ns_per_cycle))
        .collect();

    let (scenario, metric_label) = match pacing {
        Pacing::Paced => (
            format!(
                "cross-process paced stream -- ONE-WAY LATENCY ({pace_ns} ns publish period, \
                 publisher -> subscriber, direct)"
            ),
            "one-way (assumes TSC coherence)".to_string(),
        ),
        Pacing::Unpaced => (
            "cross-process unpaced stream -- QUEUE DELAY / BACKPRESSURE, NOT LATENCY \
             (publisher unthrottled; each sample includes its wait in the ring)"
                .to_string(),
            "publish->receive incl. queue".to_string(),
        ),
    };

    Ok(ScenarioResult {
        scenario,
        kind,
        metric_key: kind.metric_key().to_string(),
        payload_type: M::TYPE_NAME.to_string(),
        message_bytes: bytes,
        is_pod: is_pod_like::<M>(),
        ring_capacity: RING_CAPACITY,
        topology: format!("{pubs}P/{subs}C"),
        cpu_a: ctx.cpu_a,
        cpu_b: ctx.cpu_b,
        parent_pid: std::process::id(),
        child_pid,
        iterations: n,
        warmup: ctx.stream_warmup,
        metrics: vec![(metric_label, d)],
        headline_ns_sorted,
        window_secs: raw.window_secs,
        messages_transferred: n as u64 + raw.lost,
        messages_lost: raw.lost,
        invalid_samples: 0,
        theta_hat_cycles: theta_hat,
        perturbation: raw.perturbation,
        freq_a,
        freq_b,
        empty_poll_cycles: raw.empty_poll_cycles,
    })
}

// ============================================================================
// CPU selection
// ============================================================================

/// Online CPU -> physical core id.
fn cpu_core_ids() -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for cpu in 0..512usize {
        let p = format!("/sys/devices/system/cpu/cpu{cpu}/topology/core_id");
        if let Some(core) = read_u64(&p) {
            out.push((cpu, core as usize));
        } else if let Some(&(last, _)) = out.last() {
            if cpu > last + 8 {
                break;
            }
        } else if cpu > 8 {
            break;
        }
    }
    out
}

/// How the benchmark CPUs were chosen, so a degraded placement is stated rather
/// than silently measured.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum CpuChoice {
    /// Two distinct physical cores, neither of them CPU 0. What the figures assume.
    DistinctPhysical,
    /// Two distinct physical cores, one of which is CPU 0 (the default IRQ target).
    DistinctPhysicalIncludingCpu0,
    /// Two logical CPUs on the SAME physical core. SMT siblings share the whole
    /// pipeline; the figures are not a cross-core measurement.
    SmtSiblings,
    /// One CPU. Producer and consumer time-slice against each other.
    SingleCpu,
    /// The operator passed `--cpus`; nothing was inferred.
    Explicit,
}

/// Two CPUs on *different* physical cores, plus a third for the frequency
/// sampler.
///
/// SMT siblings share the whole core pipeline, so a pair pinned to one physical
/// core measures something else entirely. CPU 0 is skipped where possible: it is
/// the default IRQ target on a box with no explicit affinity mask.
fn pick_cpus() -> (usize, usize, usize, CpuChoice) {
    let map = cpu_core_ids();
    if map.len() < 2 {
        return (0, 0, 0, CpuChoice::SingleCpu);
    }

    // Pass 1: distinct physical cores, avoiding CPU 0.
    let distinct = |skip_cpu0: bool| -> Vec<(usize, usize)> {
        let mut chosen: Vec<(usize, usize)> = Vec::new();
        for &(cpu, core) in map.iter().rev() {
            if (skip_cpu0 && cpu == 0) || chosen.iter().any(|&(_, c)| c == core) {
                continue;
            }
            chosen.push((cpu, core));
            if chosen.len() == 3 {
                break;
            }
        }
        chosen
    };

    let sampler_of = |chosen: &[(usize, usize)]| -> usize {
        if chosen.len() >= 3 {
            chosen[2].0
        } else {
            chosen[1].0
        }
    };

    let chosen = distinct(true);
    if chosen.len() >= 2 {
        return (
            chosen[0].0,
            chosen[1].0,
            sampler_of(&chosen),
            CpuChoice::DistinctPhysical,
        );
    }
    let chosen = distinct(false);
    if chosen.len() >= 2 {
        return (
            chosen[0].0,
            chosen[1].0,
            sampler_of(&chosen),
            CpuChoice::DistinctPhysicalIncludingCpu0,
        );
    }

    // Everything reports one physical core (common under a hypervisor that does
    // not expose topology). Two distinct logical CPUs still beat one, and the
    // degradation is named rather than hidden.
    let a = map[0].0;
    let b = map[1].0;
    let sampler = map.get(2).map(|&(c, _)| c).unwrap_or(b);
    (a, b, sampler, CpuChoice::SmtSiblings)
}

// ============================================================================
// SHM housekeeping
// ============================================================================

fn topics_dir() -> PathBuf {
    let ns = std::env::var("HORUS_NAMESPACE").unwrap_or_else(|_| "default".to_string());
    PathBuf::from("/dev/shm")
        .join(format!("horus_{ns}"))
        .join("topics")
}

/// Remove only files whose name carries this run's unique token. Other agents
/// and other benchmarks share this directory; a blanket wipe would break them.
fn cleanup_run(run_id: &str) {
    if let Ok(rd) = fs::read_dir(topics_dir()) {
        for e in rd.flatten() {
            if e.file_name().to_string_lossy().contains(run_id) {
                let _ = fs::remove_file(e.path());
            }
        }
    }
}

// ============================================================================
// Gate projection
// ============================================================================

/// Build the harness `Statistics` from an ALREADY-SORTED nanosecond sample set.
///
/// Deliberately not `Statistics::from_samples`. That computes `ci_low`/`ci_high`
/// with `bootstrap_ci(samples, level, 10_000)`, which is `O(resamples * n)` —
/// 10^10 dependent random-access loads at the 10^6 sample counts this binary
/// needs to put ~100 observations behind a p99.99, times one call per scenario.
/// The interval below is the normal approximation to the *same estimand*: a 95%
/// confidence interval on the MEAN. The central limit theorem applies to the
/// mean regardless of how heavy-tailed the underlying distribution is, and at
/// `n >= 10^4` the two agree far below this harness's own resolution (~25 ns).
///
/// Every other field is computed exactly as `from_samples` computes it, through
/// the same `calculate_percentile`, over the FULL unfiltered sample set.
/// `outliers_removed` is 0 for the same reason `filter_outliers` is `false` in
/// the config: nothing here is fenced, and reporting a Tukey count would invite
/// the reader to subtract it.
fn statistics_from_sorted(sorted_ns: &[u64]) -> Statistics {
    if sorted_ns.is_empty() {
        // Mirrors `Statistics::empty`: NaN, not zero. A zero here reads as "0 ns
        // latency, 0 ns jitter" — the best possible result — in every table
        // downstream.
        return Statistics {
            count: 0,
            mean: f64::NAN,
            median: f64::NAN,
            std_dev: f64::NAN,
            min: 0,
            max: 0,
            p1: 0,
            p5: 0,
            p25: 0,
            p75: 0,
            p95: 0,
            p99: 0,
            p999: 0,
            p9999: 0,
            ci_low: f64::NAN,
            ci_high: f64::NAN,
            confidence_level: CONFIDENCE_LEVEL,
            outliers_removed: 0,
        };
    }
    let n = sorted_ns.len();
    let mean = sorted_ns.iter().map(|&x| x as f64).sum::<f64>() / n as f64;
    let var = sorted_ns
        .iter()
        .map(|&x| {
            let d = x as f64 - mean;
            d * d
        })
        .sum::<f64>()
        / n as f64;
    let std_dev = var.sqrt();
    // A confidence interval cannot be estimated from one observation.
    let (ci_low, ci_high) = if n < 2 {
        (f64::NAN, f64::NAN)
    } else {
        let se = std_dev / (n as f64).sqrt();
        (mean - Z_95 * se, mean + Z_95 * se)
    };
    Statistics {
        count: n,
        mean,
        median: calculate_percentile(sorted_ns, 50.0) as f64,
        std_dev,
        min: sorted_ns[0],
        max: sorted_ns[n - 1],
        p1: calculate_percentile(sorted_ns, 1.0),
        p5: calculate_percentile(sorted_ns, 5.0),
        p25: calculate_percentile(sorted_ns, 25.0),
        p75: calculate_percentile(sorted_ns, 75.0),
        p95: calculate_percentile(sorted_ns, 95.0),
        p99: calculate_percentile(sorted_ns, 99.0),
        p999: calculate_percentile(sorted_ns, 99.9),
        p9999: calculate_percentile(sorted_ns, 99.99),
        ci_low,
        ci_high,
        confidence_level: CONFIDENCE_LEVEL,
        outliers_removed: 0,
    }
}

/// Project one scenario's headline distribution into the shape the CI gate
/// reads (`BenchmarkReport` -> `BenchmarkResult` -> `BaselineEntry`).
///
/// The gate's identity for a row is `format!("{name}@{message_size}B")`, so
/// `name` here is the published contract; see the module header.
fn to_benchmark_result(
    r: &ScenarioResult,
    platform: &PlatformInfo,
    include_raw: bool,
) -> BenchmarkResult {
    let ns = &r.headline_ns_sorted;
    let statistics = statistics_from_sorted(ns);
    // `coefficient_of_variation` is order-independent and is computed over the
    // full unfiltered set, which is how the other binaries in this tree compute
    // the field of the same name. The gate never gates it.
    let cv = coefficient_of_variation(ns);
    let determinism = DeterminismMetrics {
        cv,
        max_jitter_ns: statistics.max.saturating_sub(statistics.min),
        p999: statistics.p999,
        p9999: statistics.p9999,
        deadline_misses: (ns.len() - ns.partition_point(|&x| x <= DEADLINE_NS)) as u64,
        deadline_threshold_ns: DEADLINE_NS,
        run_variance: 0.0,
    };

    let secs = r.window_secs.max(f64::MIN_POSITIVE);
    let throughput = ThroughputMetrics {
        messages_per_sec: r.messages_transferred as f64 / secs,
        bytes_per_sec: (r.messages_transferred as usize * r.message_bytes) as f64 / secs,
        total_messages: r.messages_transferred,
        total_bytes: (r.messages_transferred as usize * r.message_bytes) as u64,
        duration_secs: r.window_secs,
    };

    let config = BenchmarkConfig {
        warmup_iterations: r.warmup,
        iterations: r.iterations,
        runs: 1,
        // The retired binary recorded `None` here while actually pinning to
        // CPUs 0 and 1, so its JSON claimed an unpinned run it never did.
        cpu_affinity: Some((r.cpu_a, r.cpu_b)),
        // Nothing here is fenced. See the module header.
        filter_outliers: false,
        confidence_level: CONFIDENCE_LEVEL,
    };

    BenchmarkResult {
        provenance: Provenance::Measured,
        name: r.metric_key.clone(),
        subject: r.kind.subject().to_string(),
        message_size: r.message_bytes,
        config,
        platform: platform.clone(),
        timestamp: chrono::Utc::now().to_rfc3339(),
        // Off by default: 10^6 samples per scenario is ~10 MB of JSON per row,
        // and CI keeps three repetitions as an artifact. `--raw-dir` dumps every
        // sample losslessly in cycles instead, and `--raw-in-json` opts back in.
        raw_latencies_ns: if include_raw { ns.clone() } else { Vec::new() },
        statistics,
        throughput,
        determinism,
    }
}

// ============================================================================
// Printing
// ============================================================================

/// Rule width. Matches the widest table below (the SUMMARY row), so no table
/// runs past its own rule.
const RULE_WIDTH: usize = 138;

fn hr() {
    println!("{}", "-".repeat(RULE_WIDTH));
}

fn fmt_khz(k: Option<u64>) -> String {
    k.map(|v| format!("{:.0}MHz", v as f64 / 1000.0))
        .unwrap_or_else(|| "n/a".to_string())
}

fn print_machine(
    plat: &PlatformInfo,
    cfg: &MachineConfig,
    cpu_a: usize,
    cpu_b: usize,
    choice: CpuChoice,
    cal: &RdtscCalibration,
) {
    println!("MACHINE — every nanosecond below is conditional on this block");
    hr();
    println!(
        "  cpu model              : {} ({} physical / {} logical cores)",
        plat.cpu.model, plat.cpu.physical_cores, plat.cpu.logical_cores
    );
    println!("  kernel                 : {} ({})", plat.kernel, plat.arch);
    println!("  benchmark cores        : cpu{cpu_a} = process A (pinger / subscriber), cpu{cpu_b} = process B (responder / publisher)  [{choice:?}]");
    println!(
        "  governor               : cpu{cpu_a}={} cpu{cpu_b}={}  driver={}",
        cfg.governor_a.as_deref().unwrap_or("n/a"),
        cfg.governor_b.as_deref().unwrap_or("n/a"),
        cfg.scaling_driver.as_deref().unwrap_or("n/a")
    );
    println!(
        "  scaling range          : {} .. {}  (hw max {})  turbo_disabled={}",
        fmt_khz(cfg.scaling_min_khz),
        fmt_khz(cfg.scaling_max_khz),
        fmt_khz(cfg.cpuinfo_max_khz),
        cfg.turbo_disabled
            .map(|b| b.to_string())
            .unwrap_or_else(|| "unknown".to_string())
    );
    println!(
        "  clocksource            : {}   tsc flags: [{}]",
        cfg.clocksource.as_deref().unwrap_or("n/a"),
        cfg.tsc_flags.join(", ")
    );
    println!(
        "  TSC rate (calibrated)  : {:.3} MHz ({:.4} ns/cycle) — a WALL clock; NOT the core clock",
        cal.freq_hz / 1e6,
        cal.ns_per_cycle
    );
    println!(
        "  clock instrumentation  : {} cycles = {:.1} ns for serialize()+rdtsc()+rdtscp(), INCLUDED in every sample and never subtracted",
        cal.overhead_cycles,
        cal.overhead_cycles as f64 * cal.ns_per_cycle
    );
    println!(
        "  transparent_hugepage   : {}",
        cfg.transparent_hugepage.as_deref().unwrap_or("n/a")
    );
    println!(
        "  sched_rt_runtime/period: {} / {} us    perf_event_paranoid={}",
        cfg.sched_rt_runtime_us
            .map(|v| v.to_string())
            .unwrap_or_else(|| "n/a".into()),
        cfg.sched_rt_period_us
            .map(|v| v.to_string())
            .unwrap_or_else(|| "n/a".into()),
        cfg.perf_event_paranoid
            .map(|v| v.to_string())
            .unwrap_or_else(|| "n/a".into())
    );
    println!(
        "  isolcpus={}  nohz_full={}  smt_active={}  mlockall(MCL_CURRENT)={}",
        cfg.isolcpus.as_deref().unwrap_or("(none)"),
        cfg.nohz_full.as_deref().unwrap_or("(none)"),
        cfg.smt_active.as_deref().unwrap_or("n/a"),
        try_mlock_current()
    );
    hr();
}

fn print_warnings(warnings: &[String], header: &str) {
    if warnings.is_empty() {
        return;
    }
    println!();
    println!("{}", "!".repeat(RULE_WIDTH));
    println!("!! {header}");
    println!("{}", "!".repeat(RULE_WIDTH));
    for w in warnings {
        println!("!!  {w}");
    }
    println!("{}", "!".repeat(RULE_WIDTH));
}

fn print_metric_header() {
    println!(
        "  {:<30} {:>9} {:>9} {:>9} {:>9} {:>9} {:>10} {:>11} | {:>8} {:>9}",
        "metric (ns)", "n", "min", "median", "p99", "p99.9", "p99.99", "max", "n>p99.9", "n>p99.99"
    );
}

/// One metric row in nanoseconds, with the exceedance count behind each tail
/// figure so nobody quotes a p99.99 that rests on two observations.
fn print_metric_row(label: &str, d: &Dist, ns_per_cycle: f64, scale: f64) {
    let c = |v: u64| v as f64 * ns_per_cycle * scale;
    println!(
        "  {:<30} {:>9} {:>9.1} {:>9.1} {:>9.1} {:>9.1} {:>10.1} {:>11.1} | {:>8} {:>9}",
        label,
        d.n,
        c(d.min_c),
        c(d.median_c),
        c(d.p99_c),
        c(d.p999_c),
        c(d.p9999_c),
        c(d.max_c),
        d.n_gt_p999,
        d.n_gt_p9999,
    );
}

fn print_scenario(r: &ScenarioResult, ns_per_cycle: f64) {
    println!();
    println!("== {} ==", r.scenario);
    if r.kind == Kind::Unpaced {
        println!("   !! Every number in this block is a PUBLISH-TO-RECEIVE DELAY that INCLUDES");
        println!("   !! time spent waiting in the ring. It is a backpressure/throughput figure.");
        println!("   !! Do not quote it as an IPC latency; that is the exact mistake the retired");
        println!("   !! `Topic_cross_process` metric shipped for months. Read `measured backlog`");
        println!("   !! below: ~1 message means the subscriber kept up and no queue formed.");
    }
    println!(
        "   json metric key: `{}` (identity used by the CI gate: `{}@{}B`)",
        r.metric_key, r.metric_key, r.message_bytes
    );
    println!(
        "   payload {} = {} B (POD={}) | ring capacity {} slots | topology {} | cpu{} <-> cpu{} | pid {} <-> pid {}",
        r.payload_type,
        r.message_bytes,
        r.is_pod,
        r.ring_capacity,
        r.topology,
        r.cpu_a,
        r.cpu_b,
        r.parent_pid,
        r.child_pid
    );
    hr();
    print_metric_header();
    hr();
    for (name, d) in &r.metrics {
        let scale = if name.contains("halved") { 0.5 } else { 1.0 };
        print_metric_row(name, d, ns_per_cycle, scale);
    }
    hr();
    // `poll cost`, not `empty-poll cost`: the probe runs while the publisher is
    // already live, so under an unpaced publisher most of those polls return a
    // message and the figure is a populated-poll cost, not an empty one.
    println!(
        "  samples={}  warmup={}  transferred={}  lost={}  invalid(TSC-skew)={}  poll cost={:.1} ns  window={:.3} s",
        r.iterations,
        r.warmup,
        r.messages_transferred,
        r.messages_lost,
        r.invalid_samples,
        r.empty_poll_cycles as f64 * ns_per_cycle,
        r.window_secs
    );
    if r.kind != Kind::PingPong {
        let delivered_frac = if r.messages_transferred > 0 {
            100.0 * r.iterations as f64 / r.messages_transferred as f64
        } else {
            0.0
        };
        match r.measured_backlog_msgs(ns_per_cycle) {
            Some(b) => println!(
                "  measured backlog: {b:.2} messages deep on the median sample (ring holds {}); delivered {delivered_frac:.1}% of transferred",
                r.ring_capacity
            ),
            None => println!(
                "  measured backlog: n/a (window or sample count too small); delivered {delivered_frac:.1}% of transferred"
            ),
        }
    }
    if Perturbation::available() {
        println!(
            "  measured window: minor_faults={} major_faults={} ctx_switches vol={} invol={}",
            r.perturbation.minflt, r.perturbation.majflt, r.perturbation.nvcsw, r.perturbation.nivcsw
        );
    } else {
        // Printing the zeroed struct here would claim a flawlessly quiet
        // machine on the one platform where nothing was measured.
        println!(
            "  measured window: fault and context-switch counters unavailable on this platform"
        );
    }
    for f in [&r.freq_a, &r.freq_b] {
        println!(
            "  core freq cpu{:<3} (measured window only): start={} end={} min={} max={} mean={:.0}MHz spread={:.2}% (n={})",
            f.cpu,
            fmt_khz(f.start_khz),
            fmt_khz(f.end_khz),
            fmt_khz(Some(f.min_khz)),
            fmt_khz(Some(f.max_khz)),
            f.mean_khz / 1000.0,
            f.spread_frac() * 100.0,
            f.samples
        );
    }
    if let Some((name, d)) = r.metrics.first() {
        let ghz = r.freq_a.mean_khz / 1_000_000.0;
        if ghz > 0.0 {
            println!(
                "  frequency-normalised `{}`: median {:.0} core cycles, p99.9 {:.0} core cycles at f_core={:.2} GHz",
                name,
                d.median_c as f64 * ns_per_cycle * ghz,
                d.p999_c as f64 * ns_per_cycle * ghz,
                ghz
            );
        }
    }
    match r.kind {
        Kind::PingPong => println!(
            "  cross-core TSC offset theta_hat = {:+.1} ns (clock B - clock A). The `one-way, offset-free` row does not depend on it.",
            r.theta_hat_cycles * ns_per_cycle
        ),
        _ => println!(
            "  this row assumes cross-core TSC coherence; theta_hat measured in the ping-pong at this size = {:+.1} ns (add to correct).",
            r.theta_hat_cycles * ns_per_cycle
        ),
    }
}

// ============================================================================
// Detail report (everything; the gate report is a lossy projection of it)
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
struct DetailReport {
    generated_utc: String,
    tsc_freq_hz: f64,
    ns_per_cycle: f64,
    clock_overhead_cycles: u64,
    cpu_choice: CpuChoice,
    machine: MachineConfig,
    warnings: Vec<String>,
    scenarios: Vec<ScenarioResult>,
}

fn usage() {
    println!("cross_process_benchmark — the single cross-process benchmark for the HORUS Topic SHM transport");
    println!();
    println!("USAGE:");
    println!("  cross_process_benchmark [--iterations N] [--stream-iterations N] [--warmup N]");
    println!("                          [--pace-ns N] [--cpus A,B] [--no-unpaced]");
    println!("                          [--json PATH] [--detail-json PATH] [--raw-dir DIR]");
    println!("                          [--raw-in-json]");
    println!();
    println!(
        "  --iterations N        ping-pong samples per payload size (default {DEFAULT_ITERATIONS})."
    );
    println!("                        p99.99 rests on N/10000 observations; +/-5% on p99.99 needs");
    println!("                        ~400, i.e. 4000000.");
    println!(
        "  --stream-iterations N samples for each stream scenario (default {DEFAULT_STREAM_ITERATIONS});"
    );
    println!("                        a paced sample costs one publish period of wall clock.");
    println!("  --warmup N            discarded ping-pong iterations (default {DEFAULT_WARMUP}; must exceed 256).");
    println!("  --pace-ns N          publish interval for the paced-stream scenario (default {DEFAULT_PACE_NS}).");
    println!("  --cpus A,B           pin process A to A and process B to B (default: two distinct");
    println!("                        physical cores).");
    println!("  --no-unpaced         skip the unpaced backpressure/queue-delay scenario.");
    println!("  --json PATH          write the gate-schema `BenchmarkReport` (what CI consumes).");
    println!("  --detail-json PATH   write the full report: machine block, every leg of every");
    println!("                        decomposition, warnings, frequency and fault records.");
    println!(
        "  --raw-dir DIR        dump raw per-sample cycle counts as little-endian u64 arrays."
    );
    println!("  --raw-in-json        also embed the headline samples in --json (large).");
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    // ---- child dispatch ----------------------------------------------------
    // --child <echo|pub> <cpu> <bytes> <topic> <topic2|count> [pace_cycles]
    if args.first().map(String::as_str) == Some("--child") {
        let mode = args.get(1).cloned().unwrap_or_default();
        let cpu: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
        let bytes: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(64);
        if set_cpu_affinity(cpu).is_err() {
            eprintln!("[child {mode}] warning: could not pin to cpu{cpu}; this run is not a two-core measurement");
        }
        // No `unwrap_or` on the pace: 0 means FLOOD, so a silently-defaulted
        // parse failure would turn a paced latency run into a saturated-queue
        // run reported as latency — precisely the bug this binary exists to
        // undo. A malformed argument kills the child instead.
        let pub_args = |idx_total: usize, idx_pace: usize| -> Result<(usize, u64), String> {
            let total = args
                .get(idx_total)
                .ok_or("publisher child: missing message count")?
                .parse::<usize>()
                .map_err(|e| format!("publisher child: bad message count: {e}"))?;
            let pace = args
                .get(idx_pace)
                .ok_or("publisher child: missing pace")?
                .parse::<u64>()
                .map_err(|e| format!("publisher child: bad pace: {e}"))?;
            Ok((total, pace))
        };
        let r = match (mode.as_str(), bytes) {
            ("echo", 64) => echo_child::<Probe64>(&args[4], &args[5]),
            ("echo", _) => echo_child::<Probe1k>(&args[4], &args[5]),
            ("pub", 64) => pub_args(5, 6).and_then(|(t, p)| pub_child::<Probe64>(&args[4], t, p)),
            ("pub", _) => pub_args(5, 6).and_then(|(t, p)| pub_child::<Probe1k>(&args[4], t, p)),
            _ => Err(format!("unknown child mode {mode}")),
        };
        if let Err(e) = r {
            eprintln!("[child {mode}] {e}");
            std::process::exit(1);
        }
        return;
    }

    // ---- arguments ---------------------------------------------------------
    let mut iterations = DEFAULT_ITERATIONS;
    let mut stream_iterations = DEFAULT_STREAM_ITERATIONS;
    let mut warmup = DEFAULT_WARMUP;
    let mut pace_ns = DEFAULT_PACE_NS;
    let mut json_path: Option<String> = None;
    let mut detail_json_path: Option<String> = None;
    let mut raw_dir: Option<String> = None;
    let mut raw_in_json = false;
    let mut run_unpaced = true;
    let mut cpu_override: Option<(usize, usize)> = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--iterations" => {
                iterations = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_ITERATIONS);
                i += 2;
            }
            "--stream-iterations" => {
                stream_iterations = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_STREAM_ITERATIONS);
                i += 2;
            }
            "--warmup" => {
                warmup = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_WARMUP);
                i += 2;
            }
            "--pace-ns" => {
                pace_ns = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(DEFAULT_PACE_NS);
                i += 2;
            }
            "--json" => {
                json_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--detail-json" => {
                detail_json_path = args.get(i + 1).cloned();
                i += 2;
            }
            "--raw-dir" => {
                raw_dir = args.get(i + 1).cloned();
                i += 2;
            }
            "--raw-in-json" => {
                raw_in_json = true;
                i += 1;
            }
            "--no-unpaced" => {
                run_unpaced = false;
                i += 1;
            }
            "--cpus" => {
                if let Some(v) = args.get(i + 1) {
                    let parts: Vec<&str> = v.split(',').collect();
                    if parts.len() == 2 {
                        if let (Ok(a), Ok(b)) = (parts[0].trim().parse(), parts[1].trim().parse()) {
                            cpu_override = Some((a, b));
                        }
                    }
                }
                i += 2;
            }
            "--help" | "-h" => {
                usage();
                return;
            }
            _ => i += 1,
        }
    }
    if iterations == 0 {
        iterations = DEFAULT_ITERATIONS;
    }
    if stream_iterations == 0 {
        stream_iterations = DEFAULT_STREAM_ITERATIONS;
    }
    // A stream sample costs a publish *period*, so the ping-pong warmup would
    // dominate the run. Both floors stay above the 256 `resolve_owner` retries
    // that a shorter warmup would otherwise measure.
    let stream_warmup = warmup.clamp(1_024, 20_000).min(stream_iterations);

    let (mut cpu_a, mut cpu_b, sampler_cpu, mut cpu_choice) = pick_cpus();
    if let Some((a, b)) = cpu_override {
        cpu_a = a;
        cpu_b = b;
        cpu_choice = CpuChoice::Explicit;
    }

    println!();
    println!("HORUS cross-process benchmark — ONE-WAY LATENCY (headline) + unpaced queue delay");
    println!("Two OS processes on two distinct physical cores over the real Topic SHM transport.");
    println!(
        "Every handle holds exactly one role, so the `role == Both` same-thread fast path that"
    );
    println!("produces the repo's headline send-cost figure is never taken. Round-trip rows are");
    println!("labelled where they are a halved round trip. See the module header for the clock");
    println!("decomposition and for why the `Topic_cross_process` JSON key was retired.");
    println!();

    let cal = calibrate_rdtsc(300);
    let cfg = detect_machine_config(cpu_a, cpu_b);

    // Captured BEFORE this process is pinned, and reused for the JSON report.
    // `detect_platform()` derives `logical_cores` from the calling thread's
    // affinity mask, so calling it after `set_cpu_affinity` publishes
    // `logical_cores: 1` on a 12-core box — a machine description that is a
    // side effect of the benchmark rather than a fact about the machine.
    let platform = detect_platform();
    print_machine(&platform, &cfg, cpu_a, cpu_b, cpu_choice, &cal);

    let affinity_ok = set_cpu_affinity(cpu_a).is_ok();
    try_mlock_current();

    let run_id = format!(
        "{}x{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.subsec_nanos())
            .unwrap_or(0)
    );

    let ctx = RunCtx {
        run_id: &run_id,
        cpu_a,
        cpu_b,
        sampler_cpu,
        iterations,
        stream_iterations,
        warmup,
        stream_warmup,
        ns_per_cycle: cal.ns_per_cycle,
        raw_dir: raw_dir.as_deref(),
    };

    let mut scratch = prefaulted(iterations.max(stream_iterations));
    let mut scenarios: Vec<ScenarioResult> = Vec::new();
    let mut warnings: Vec<String> = Vec::new();

    println!();
    println!("running ping-pong (one-way latency), 64 B payload ...");
    match run_pingpong::<Probe64>(&ctx, &mut scratch) {
        Ok(r) => scenarios.push(r),
        Err(e) => warnings.push(format!("ping-pong 64 B failed: {e}")),
    }
    println!("running ping-pong (one-way latency), 1024 B payload ...");
    match run_pingpong::<Probe1k>(&ctx, &mut scratch) {
        Ok(r) => scenarios.push(r),
        Err(e) => warnings.push(format!("ping-pong 1024 B failed: {e}")),
    }

    let theta_for = |bytes: usize, s: &[ScenarioResult]| -> f64 {
        s.iter()
            .find(|r| r.kind == Kind::PingPong && r.message_bytes == bytes)
            .map(|r| r.theta_hat_cycles)
            .unwrap_or(0.0)
    };
    let pace_cycles = (pace_ns as f64 * cal.cycles_per_ns) as u64;

    println!("running paced stream (one-way latency), 64 B payload ...");
    let t64 = theta_for(64, &scenarios);
    match run_stream::<Probe64>(&ctx, Pacing::Paced, pace_ns, pace_cycles, t64, &mut scratch) {
        Ok(r) => scenarios.push(r),
        Err(e) => warnings.push(format!("paced stream 64 B failed: {e}")),
    }
    println!("running paced stream (one-way latency), 1024 B payload ...");
    let t1k = theta_for(1024, &scenarios);
    match run_stream::<Probe1k>(&ctx, Pacing::Paced, pace_ns, pace_cycles, t1k, &mut scratch) {
        Ok(r) => scenarios.push(r),
        Err(e) => warnings.push(format!("paced stream 1024 B failed: {e}")),
    }

    if run_unpaced {
        println!("running unpaced stream (QUEUE DELAY, not latency), 64 B payload ...");
        match run_stream::<Probe64>(&ctx, Pacing::Unpaced, 0, 0, t64, &mut scratch) {
            Ok(r) => scenarios.push(r),
            Err(e) => warnings.push(format!("unpaced stream 64 B failed: {e}")),
        }
    }

    cleanup_run(&run_id);

    // ---- integrity checks ---------------------------------------------------
    if !affinity_ok {
        warnings.push(format!(
            "sched_setaffinity FAILED for the measuring process (cpu{cpu_a}). Nothing below is a pinned two-core measurement."
        ));
    }
    match cpu_choice {
        CpuChoice::SingleCpu => warnings.push(
            "Only ONE usable CPU was found: both processes are pinned to the same core and time-slice against each other. These are not cross-core numbers."
                .to_string(),
        ),
        CpuChoice::SmtSiblings => warnings.push(format!(
            "cpu{cpu_a} and cpu{cpu_b} report the SAME physical core id. SMT siblings share the whole core pipeline, so this is not a cross-core measurement."
        )),
        CpuChoice::DistinctPhysicalIncludingCpu0 => warnings.push(
            "One benchmark core is CPU 0, the default IRQ target on a box with no explicit affinity mask. Interrupt service time is in the tail."
                .to_string(),
        ),
        CpuChoice::Explicit => {
            if cpu_a == cpu_b {
                warnings.push(format!(
                    "--cpus named the same CPU twice (cpu{cpu_a}). Both processes share one core; these are not cross-core numbers."
                ));
            }
        }
        CpuChoice::DistinctPhysical => {}
    }
    if cfg.governor_a.as_deref() != Some("performance")
        || cfg.governor_b.as_deref() != Some("performance")
    {
        warnings.push(format!(
            "CPU governor is {}/{}, not `performance`. RDTSC is a wall clock while the work costs a fixed number of CORE cycles, so every nanosecond below scales with a frequency the governor is free to change. iceoryx2's maintainers measured ~300 ns of difference from the governor alone.",
            cfg.governor_a.as_deref().unwrap_or("?"),
            cfg.governor_b.as_deref().unwrap_or("?")
        ));
    }
    if cfg.scaling_min_khz != cfg.scaling_max_khz {
        warnings.push(format!(
            "DVFS is live: scaling_min_freq ({}) != scaling_max_freq ({}). Pin them equal before quoting any absolute figure.",
            fmt_khz(cfg.scaling_min_khz),
            fmt_khz(cfg.scaling_max_khz)
        ));
    }
    if cfg.turbo_disabled == Some(false) {
        warnings.push(
            "Turbo/boost is ENABLED; core frequency varies with thermals and neighbour load."
                .to_string(),
        );
    }
    if cfg.clocksource.as_deref() != Some("tsc") {
        warnings.push(format!(
            "clocksource is {:?}, not `tsc`; the calibration and the cross-process decomposition both assume an invariant TSC.",
            cfg.clocksource
        ));
    }
    if cfg.isolcpus.is_none() {
        warnings.push("No isolcpus= on the kernel command line: the benchmark cores are shared with everything else on this box. The involuntary context-switch count in each scenario block is the mechanism behind the far tail.".to_string());
    }
    for r in &scenarios {
        if r.freq_a.moved() || r.freq_b.moved() {
            warnings.push(format!(
                "FREQUENCY MOVED during {} ({} B): cpu{} spread {:.1}%, cpu{} spread {:.1}%. These nanoseconds are not comparable to another run, including another run on this same box.",
                r.kind.slug(),
                r.message_bytes,
                r.freq_a.cpu,
                r.freq_a.spread_frac() * 100.0,
                r.freq_b.cpu,
                r.freq_b.spread_frac() * 100.0
            ));
        }
        if r.perturbation.minflt > 0 {
            warnings.push(format!(
                "{} ({} B): {} MINOR PAGE FAULTS inside the measured window — a pre-fault was missed, and those faults are in the tail figures.",
                r.kind.slug(), r.message_bytes, r.perturbation.minflt
            ));
        }
        if r.perturbation.majflt > 0 {
            warnings.push(format!(
                "{} ({} B): {} MAJOR page faults inside the measured window.",
                r.kind.slug(),
                r.message_bytes,
                r.perturbation.majflt
            ));
        }
        // An unpaced run that DID build a backlog is measuring the thing it is
        // named for, but a reader must not mistake it for a latency, and an
        // unpaced run that did NOT build one is not a backpressure measurement
        // at all. Both directions get said out loud.
        if r.kind == Kind::Unpaced {
            match r.measured_backlog_msgs(cal.ns_per_cycle) {
                Some(b) if b < 2.0 => warnings.push(format!(
                    "unpaced_queue ({} B): measured backlog is only {:.2} messages — the subscriber kept up, so the ring never filled and `{}` is NOT a backpressure figure on this run. It is a publish-to-receive delay with no queueing in it, and it is still not a latency (see the paced and ping-pong rows for that).",
                    r.message_bytes, b, KEY_UNPACED
                )),
                Some(b) => warnings.push(format!(
                    "unpaced_queue ({} B): measured backlog {:.1} messages deep of {} ring slots. `{}` is QUEUE DELAY. Quoting it as a latency overstates the transport by roughly that factor — the mistake the retired `Topic_cross_process` metric shipped.",
                    r.message_bytes, b, r.ring_capacity, KEY_UNPACED
                )),
                None => warnings.push(format!(
                    "unpaced_queue ({} B): backlog could not be measured, so nothing distinguishes this row from a latency row. Do not quote it.",
                    r.message_bytes
                )),
            }
        }
        // Loss is a *defect* in a paced run and an expected consequence of an
        // unpaced one, so only the former is a warning.
        if r.messages_lost > 0 && r.kind != Kind::Unpaced {
            warnings.push(format!(
                "{} ({} B): {} messages LOST. The distribution is over delivered messages only and is biased by the loss.",
                r.kind.slug(), r.message_bytes, r.messages_lost
            ));
        }
        if r.invalid_samples > 0 {
            warnings.push(format!(
                "{} ({} B): {} samples had a one-way leg longer than the whole round trip — cross-core TSC skew large enough to invalidate the decomposition.",
                r.kind.slug(), r.message_bytes, r.invalid_samples
            ));
        }
        if let Some((name, d)) = r.metrics.first() {
            if d.n_gt_p9999 < 100 {
                warnings.push(format!(
                    "{} ({} B) `{}`: p99.99 rests on {} observations. Below ~100 exceedances it is a max wearing a percentile's name; +/-5% needs ~400 (--iterations 4000000).",
                    r.kind.slug(), r.message_bytes, name, d.n_gt_p9999
                ));
            }
        }
    }
    if scenarios.is_empty() {
        warnings.push("NO SCENARIO PRODUCED A RESULT.".to_string());
    }

    // ---- output -------------------------------------------------------------
    print_warnings(&warnings, "READ THIS BEFORE QUOTING ANY NUMBER BELOW");

    for r in &scenarios {
        print_scenario(r, cal.ns_per_cycle);
    }

    println!();
    println!("SUMMARY — one row per published JSON metric (nanoseconds, wall clock)");
    hr();
    println!(
        "  {:<46} {:<26} {:>8} {:>9} {:>9} {:>9} {:>10} {:>11}",
        "json metric key", "what it measures", "n", "median", "p99", "p99.9", "p99.99", "max"
    );
    hr();
    for r in &scenarios {
        if let Some((_, d)) = r.metrics.first() {
            let c = |v: u64| v as f64 * cal.ns_per_cycle;
            let what = if r.kind.is_latency() {
                "ONE-WAY latency"
            } else {
                "QUEUE DELAY (not latency)"
            };
            println!(
                "  {:<46} {:<26} {:>8} {:>9.1} {:>9.1} {:>9.1} {:>10.1} {:>11.1}",
                format!("{}@{}B", r.metric_key, r.message_bytes),
                what,
                d.n,
                c(d.median_c),
                c(d.p99_c),
                c(d.p999_c),
                c(d.p9999_c),
                c(d.max_c)
            );
        }
    }
    hr();
    println!("  Instrumentation is INCLUDED in every figure and never subtracted.");
    println!("  `{KEY_ONEWAY_PINGPONG}` is the true one-way figure: it assumes");
    println!(
        "  nothing about cross-core TSC synchronisation and excludes the responder's turnaround."
    );
    println!(
        "  `RTT halved (one-way)` in the per-scenario tables is a ROUND TRIP DIVIDED BY TWO; it is"
    );
    println!("  the row to compare against any competitor publishing a halved ping-pong.");
    println!("  `{KEY_UNPACED}` is a BACKPRESSURE figure: it includes");
    println!(
        "  however long the message waited in the ring, so it scales with the achieved backlog"
    );
    println!("  (printed in its scenario block) and is NOT a latency.");
    println!("  A figure without its payload size, ring depth and topology is not comparable to anything.");

    print_warnings(
        &warnings,
        "THE SAME WARNINGS AGAIN — they apply to every number above",
    );

    // Gate-schema report: one BenchmarkResult per scenario, keyed by
    // `{name}@{message_size}B`. This is what CI's regression_gate consumes.
    if let Some(path) = &json_path {
        let mut report = BenchmarkReport::new(platform.clone());
        for r in &scenarios {
            report.add_result(to_benchmark_result(r, &platform, raw_in_json));
        }
        match write_json_report(&report, path) {
            Ok(()) => {
                println!("\nGate report written to {path}");
                println!(
                    "  keys: {}",
                    report
                        .results
                        .iter()
                        .map(|r| format!("{}@{}B", r.name, r.message_size))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                println!(
                    "  NOTE: `Topic_cross_process` is retired. A baseline window that still holds it"
                );
                println!(
                    "  will report it as missing and fail the gate until the window is refreshed —"
                );
                println!("  that is intended, and is why the key was not silently reused.");
                if !raw_in_json {
                    println!("  Raw samples are omitted (pass --raw-in-json, or --raw-dir for a lossless cycle dump).");
                }
                if !warnings.is_empty() {
                    println!("  {} warning(s) apply to these numbers and are NOT in this file; use --detail-json.", warnings.len());
                }
            }
            Err(e) => eprintln!("\nfailed to write {path}: {e}"),
        }
    }

    // Full report: everything the gate schema cannot carry.
    if let Some(path) = &detail_json_path {
        let report = DetailReport {
            generated_utc: chrono::Utc::now().to_rfc3339(),
            tsc_freq_hz: cal.freq_hz,
            ns_per_cycle: cal.ns_per_cycle,
            clock_overhead_cycles: cal.overhead_cycles,
            cpu_choice,
            machine: cfg,
            warnings,
            scenarios,
        };
        match serde_json::to_string_pretty(&report) {
            Ok(s) => match fs::write(path, s) {
                Ok(()) => println!("\nDetail report written to {path}"),
                Err(e) => eprintln!("\nfailed to write {path}: {e}"),
            },
            Err(e) => eprintln!("\nfailed to serialise detail report: {e}"),
        }
    }
}
