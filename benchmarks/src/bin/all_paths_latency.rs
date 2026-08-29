//! HORUS IPC Latency Benchmark — All Backend Paths
//!
//! Measures true per-message latency across every Topic backend route.
//!
//! ## Methodology
//!
//! **Same-process** (SpscShm self-loop, then SpscShm/MpscShm/PodShm cross-thread):
//! - Measures `send()` latency via RDTSC with overhead subtraction
//! - Producer and consumer in same process, consumer on separate core. Every
//!   topic is SHM-backed, so these hit the same backends as cross-process —
//!   latency is set by core locality (a same-thread self-loop stays L1-hot; the
//!   moment the consumer runs on another core you pay cross-core cache coherence)
//!
//! **Cross-process** (SpscShm, MpscShm, PodShm):
//! - One-way latency via RDTSC cycle timestamps embedded in `CmdVel.timestamp_ns`
//! - Producer writes `rdtsc()` → `send()`, consumer reads `recv()` → `rdtscp()`
//! - Requires `constant_tsc` for cross-core TSC synchronization
//! - No `yield_now()` or `sleep()` in measurement hot loops
//!
//! **Note**: non-POD many-to-many resolves to FanoutShm. Since CmdVel is POD,
//! multi-pub/sub here resolves to PodShm broadcast, so FanoutShm is not directly
//! benchmarked (it shares the SHM SPSC matrix with added serialization overhead).
//!
//! ## Statistical Analysis
//!
//! Each scenario: 100K samples, bootstrap 95% CI, CPU-pinned processes on
//! verified-distinct physical cores.
//!
//! **No statistic published here is outlier-filtered.** Tukey's fence is
//! computed and its count reported as a diagnostic, and nothing else. The fence
//! deletes exactly the preemptions and page faults that constitute jitter, so a
//! filtered mean, standard deviation or CV is bounded by construction rather
//! than measured.
//!
//! **A percentile is printed only when at least 100 observations lie beyond
//! it.** At 100K samples that permits p99 and p99.9 and forbids p99.99, which
//! would rest on ten observations and move by tens of percent between identical
//! runs. Unsupported figures print as `n/a` with the exceedance count.
//!
//! ## Usage
//!
//! ```sh
//! # Standard run (human-readable, includes stress tests)
//! cargo run --release --bin all_paths_latency
//!
//! # Skip stress tests (faster, standard latency suite only)
//! cargo run --release --bin all_paths_latency -- --skip-stress
//!
//! # Machine-readable JSON output for CI/regression tracking
//! cargo run --release --bin all_paths_latency -- --json results.json
//!
//! # Best results: set performance governor first
//! sudo cpupower frequency-set -g performance
//! ```

use horus::prelude::Topic;
use horus_benchmarks::output::{write_json_report, BenchmarkReport};
use horus_benchmarks::platform::{
    detect_cpu_frequency, detect_platform, distinct_physical_cores, has_constant_tsc,
    share_physical_core, PlatformInfo,
};
use horus_benchmarks::set_cpu_affinity;
use horus_benchmarks::stats::{count_zero_samples, Statistics, MIN_TAIL_EXCEEDANCES};
use horus_benchmarks::timing::{rdtsc, rdtscp, serialize, PrecisionTimer, RdtscCalibration};
use horus_benchmarks::{
    BenchmarkConfig, BenchmarkResult, DeterminismMetrics, Provenance, ThroughputMetrics,
};
use horus_core::core::DurationExt;
use horus_robotics::CmdVel;
use std::hint::spin_loop;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// Configuration
// ============================================================================

const ITERATIONS: u64 = 100_000;
const WARMUP: u64 = 5_000;
const MIGRATION_BOOT: u64 = 200;
const TIMEOUT: Duration = Duration::from_secs(30);

/// PodShm publishers need far more messages than other scenarios because:
/// 1. Setup (spawn consumer, wait for topology, trigger migration) takes ~350ms
/// 2. At ~100ns/msg SHM speed, 105K msgs finishes in ~10ms
/// 3. Publishers must still be running during the measurement phase
///
/// 10M msgs × ~100ns = ~1s — enough headroom for setup + warmup + measurement.
const PODSHM_MSGS_PER_PUB: u64 = 10_000_000;

/// Default deadline budget, in nanoseconds, against which `deadline_misses` is
/// counted.
///
/// This is a *declared budget for this benchmark*, not a HORUS guarantee. 10 us
/// is two orders of magnitude above every path measured here and an order of
/// magnitude above the ~1 us producer pacing, so a sample that exceeds it did
/// not lose a cache-coherence race — it was preempted, faulted, or migrated.
/// That is the event a control loop cares about, and the count of them is a
/// number this harness can actually produce. Override with `--deadline-ns`.
const DEFAULT_DEADLINE_NS: u64 = 10_000;

/// Deadline budget for this run. Set once from the CLI before any scenario runs.
static DEADLINE_NS: AtomicU64 = AtomicU64::new(DEFAULT_DEADLINE_NS);

fn deadline_ns() -> u64 {
    DEADLINE_NS.load(Ordering::Relaxed)
}

/// Threads whose CPU pin did not take, in this process.
///
/// Every pin used to be `let _ = set_cpu_affinity(..)`. On a runner with fewer
/// logical CPUs than the benchmark asks for, every one of those calls fails and
/// the whole suite silently measures whatever the scheduler felt like doing —
/// producing numbers that look exactly like pinned ones. A benchmark that could
/// not place its threads has not measured what it claims to measure.
static PIN_FAILURES: AtomicU64 = AtomicU64::new(0);

/// Pin the calling thread to `core`, recording and reporting any failure.
fn pin_to(core: usize, role: &str) {
    if let Err(e) = set_cpu_affinity(core) {
        PIN_FAILURES.fetch_add(1, Ordering::Relaxed);
        eprintln!(
            "  ERROR: could not pin {} to CPU {}: {} -- this run is NOT pinned",
            role, core, e
        );
    }
}

/// CPU cores this benchmark pins to, one per distinct physical core.
///
/// The old assignment was five `const`s, `0, 2, 4, 6, 8`, commented "spaced by 2
/// to avoid hyperthreading siblings on most Intel/AMD layouts". No layout is
/// avoided by arithmetic. On a 6-core/12-thread Intel part `cpu0`'s sibling list
/// is `0,6`, so `core_main() = 0` and `core_child_cons() = 6` were the two threads of
/// one physical core: the cross-process scenarios pinned the measuring consumer
/// onto the same core as a child and reported the result as a cross-core
/// number. On AMD Zen, siblings are enumerated adjacently and the *opposite*
/// convention (`0, 1`) collides instead. At most one of the two conventions in
/// this repository can be right on any given machine, and neither ever read
/// `thread_siblings_list`.
struct CoreAssignment {
    main: usize,
    aux: usize,
    pub2: usize,
    child_cons: usize,
    cons2: usize,
    /// True when these came from `thread_siblings_list` and are known pairwise
    /// distinct. False means the fallback list is in use and core locality is
    /// unverified.
    topology_verified: bool,
}

/// Used only when the kernel will not tell us the topology.
const FALLBACK_CORES: [usize; 5] = [0, 2, 4, 6, 8];

impl CoreAssignment {
    fn detect() -> Self {
        match distinct_physical_cores(5) {
            Some(c) => Self {
                main: c[0],
                aux: c[1],
                pub2: c[2],
                child_cons: c[3],
                cons2: c[4],
                topology_verified: true,
            },
            None => Self {
                main: FALLBACK_CORES[0],
                aux: FALLBACK_CORES[1],
                pub2: FALLBACK_CORES[2],
                child_cons: FALLBACK_CORES[3],
                cons2: FALLBACK_CORES[4],
                topology_verified: false,
            },
        }
    }

    fn all(&self) -> [usize; 5] {
        [self.main, self.aux, self.pub2, self.child_cons, self.cons2]
    }
}

static CORES: OnceLock<CoreAssignment> = OnceLock::new();

fn cores() -> &'static CoreAssignment {
    CORES.get_or_init(CoreAssignment::detect)
}

fn core_main() -> usize {
    cores().main
}
fn core_aux() -> usize {
    cores().aux
}
fn core_pub2() -> usize {
    cores().pub2
}
fn core_child_cons() -> usize {
    cores().child_cons
}
fn core_cons2() -> usize {
    cores().cons2
}

/// Width of the output box (interior, excluding border characters)
const BOX_W: usize = 72;

// ============================================================================
// Result
// ============================================================================

struct ScenarioResult {
    name: &'static str,
    backend: String,
    expected_backend: &'static str,
    measurement: &'static str,
    latencies_ns: Vec<u64>,
    total_sent: u64,
    total_received: u64,
    note: Option<&'static str>,
    /// For broadcast backends: freshness (how many messages producer advanced between reads)
    freshness_samples: Option<Vec<u64>>,
    /// For broadcast backends: fraction of produced messages that were actually read
    delivery_ratio: Option<f64>,
    /// For broadcast backends: number of consumer skip-aheads
    skip_count: Option<u64>,
}

impl ScenarioResult {
    fn stats(&self) -> Statistics {
        if self.latencies_ns.is_empty() {
            return Statistics::from_samples(&[], 95.0, false);
        }
        Statistics::from_samples(&self.latencies_ns, 95.0, true)
    }

    fn backend_ok(&self) -> bool {
        self.backend.contains(self.expected_backend)
    }

    fn loss_pct(&self) -> f64 {
        if self.total_sent == 0 {
            return 0.0;
        }
        let loss = self.total_sent.saturating_sub(self.total_received);
        loss as f64 / self.total_sent as f64 * 100.0
    }

    /// Samples that exceeded the declared deadline budget.
    ///
    /// This field used to be the literal `0`, written straight into the JSON
    /// report under `Provenance::Measured`, beside a `deadline_threshold_ns` of
    /// `0`. Nothing was measured and nothing could ever have made it non-zero;
    /// any consumer of the artefact read "zero deadline misses observed" from a
    /// framework whose entire selling point is safety-critical real time. An
    /// unmeasured zero stamped `Measured` is worse than an absent field.
    fn deadline_misses(&self) -> u64 {
        let budget = deadline_ns();
        self.latencies_ns.iter().filter(|&&l| l > budget).count() as u64
    }

    /// Samples that came out as exactly 0 ns.
    ///
    /// Not a fast operation: the signature of an overhead subtraction that
    /// over-corrected. See [`horus_benchmarks::stats::count_zero_samples`].
    fn zero_samples(&self) -> usize {
        count_zero_samples(&self.latencies_ns)
    }

    /// Short backend name for summary table (e.g. "SpscShm" from "SpscShm (Adaptive)")
    fn backend_short(&self) -> &str {
        self.backend
            .split_once(" (")
            .map(|(name, _)| name)
            .unwrap_or(&self.backend)
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // --- Child process entry points ---
    if args.len() >= 5 && args[1] == "--child-publisher" {
        let paced = args.get(5).map(|s| s == "--paced").unwrap_or(false);
        run_child_publisher(
            &args[2],
            args[3].parse().unwrap(),
            args[4].parse().unwrap(),
            paced,
        );
        return;
    }
    if args.len() >= 5 && args[1] == "--child-consumer" {
        run_child_consumer(&args[2], args[3].parse().unwrap(), args[4].parse().unwrap());
        return;
    }
    if args.len() >= 4 && args[1] == "--child-atomic-writer" {
        run_child_atomic_writer(&args[2], args[3].parse().unwrap());
        return;
    }

    // --- Parse CLI flags ---
    let json_path = args
        .windows(2)
        .find(|w| w[0] == "--json")
        .map(|w| w[1].clone());
    let skip_stress = args.iter().any(|a| a == "--skip-stress");
    let with_raw = args.iter().any(|a| a == "--raw-samples");
    let allow_unpinned = args.iter().any(|a| a == "--allow-unpinned");
    if let Some(v) = args.windows(2).find(|w| w[0] == "--deadline-ns") {
        match v[1].parse::<u64>() {
            Ok(ns) if ns > 0 => DEADLINE_NS.store(ns, Ordering::Relaxed),
            _ => {
                eprintln!(
                    "  ERROR: --deadline-ns needs a positive integer, got '{}'",
                    v[1]
                );
                std::process::exit(2);
            }
        }
    }

    // --- Initialization ---
    let mut platform = detect_platform();
    let timer = PrecisionTimer::with_calibration(500);
    let cal = timer.calibration();

    print_header(&platform, cal);
    let platform_ok = validate_platform(&platform);
    if !platform_ok && !allow_unpinned {
        eprintln!();
        eprintln!("  Refusing to run: the conditions above make the numbers this binary");
        eprintln!("  would print unattributable to the code under test. Publishing an");
        eprintln!("  unpinned or sibling-colliding latency figure is worse than publishing");
        eprintln!("  none. Re-run with --allow-unpinned to measure anyway.");
        std::process::exit(2);
    }
    pin_to(core_main(), "main measurement thread");

    let mut results: Vec<ScenarioResult> = Vec::new();

    // === Same-process (5 scenarios, all SHM) ===
    println!("─── Same-Process {}", "─".repeat(BOX_W - 18));
    println!();

    let r = bench_same_thread_selfloop(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spsc_same_proc(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpsc_same_proc(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spmc_same_proc(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpmc_same_proc(&timer);
    print_detail(&r);
    results.push(r);

    // === Cross-process (4 scenarios) ===
    println!(
        "─── Cross-Process (RDTSC-in-payload) {}",
        "─".repeat(BOX_W - 37)
    );
    println!();

    let r = bench_spsc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_mpsc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_spmc_shm(&timer);
    print_detail(&r);
    results.push(r);

    let r = bench_pod_shm(&timer);
    print_detail(&r);
    print_pod_shm_detail(&r);
    results.push(r);

    // === Raw atomic probe (hardware floor) ===
    println!();
    println!(
        "─── Hardware Floor (raw SHM atomic) {}",
        "─".repeat(BOX_W - 38)
    );
    println!();

    let r = bench_raw_atomic(&timer);
    print_detail(&r);
    results.push(r);

    // === Scalability Stress Tests ===
    if !skip_stress {
        println!();
        println!(
            "─── Scalability Stress (PodShm N-pub x M-sub) {}",
            "─".repeat(BOX_W - 47)
        );
        println!();

        let stress_scenarios: Vec<(&str, usize, usize)> = vec![
            ("Stress-4P1C", 4, 1),
            ("Stress-8P1C", 8, 1),
            ("Stress-1P4C", 1, 4),
            ("Stress-1P8C", 1, 8),
            ("Stress-4P4C", 4, 4),
        ];

        for (name, pubs, cons) in stress_scenarios {
            let r = bench_stress(name, pubs, cons, &timer);
            print_detail(&r);
            results.push(r);
        }
    } else {
        println!();
        println!("  (Stress tests skipped via --skip-stress)");
        println!();
    }

    // === Summary ===
    print_summary(&results);
    print_overhead_analysis(&results);
    print_methodology(cal);

    // === Did the machine hold still? ===
    // Every reported nanosecond is core_cycles / f_core. If f_core moved during
    // the run, the rows above are not comparable to each other, let alone to a
    // previous run — and nothing else in the output would say so.
    platform.cpu.measured_freq_mhz_end = detect_cpu_frequency();
    print_frequency_check(&platform);

    // === Was anything actually pinned? ===
    let pin_failures = PIN_FAILURES.load(Ordering::Relaxed);
    if pin_failures > 0 {
        eprintln!();
        eprintln!("  {} thread pin(s) FAILED in this process.", pin_failures);
        eprintln!("  The affected threads ran wherever the scheduler put them, which may");
        eprintln!("  include sharing a physical core with the thread they were measured");
        eprintln!("  against. These numbers do not mean what the table says they mean.");
        if !allow_unpinned {
            eprintln!("  JSON output suppressed. Re-run with --allow-unpinned to write it anyway.");
            std::process::exit(2);
        }
    }

    // === JSON output ===
    if let Some(path) = json_path {
        write_json_output(&path, &platform, &results, with_raw);
    }
}

/// Report the TSC rate at both ends of the run.
fn print_frequency_check(platform: &PlatformInfo) {
    let (Some(start), Some(end)) = (
        platform.cpu.measured_freq_mhz,
        platform.cpu.measured_freq_mhz_end,
    ) else {
        return;
    };
    let drift = platform.cpu.freq_drift_pct().unwrap_or(f64::NAN);

    println!("─── Clock Check {}", "─".repeat(BOX_W - 18));
    println!();
    println!(
        "  TSC rate: {:.1} MHz at start, {:.1} MHz at end ({:+.2}%)",
        start, end, drift
    );
    println!("  Every nanosecond above is core_cycles / core_frequency. RDTSC ticks at");
    println!("  the nominal rate regardless of P-state, so a core that clocked down");
    println!("  makes identical code report a larger latency, with nothing else in this");
    println!("  output changing.");
    if drift.abs() > 2.0 {
        println!();
        println!(
            "  WARNING: the clock moved {:+.2}% across this run. Rows measured early and",
            drift
        );
        println!("  rows measured late are not comparable to each other.");
    }
    println!();
}

// ============================================================================
// Platform header & validation
// ============================================================================

fn print_header(platform: &PlatformInfo, cal: &RdtscCalibration) {
    let now = chrono::Local::now().format("%Y-%m-%d %H:%M:%S %Z");
    let commit = detect_git_commit_short();

    println!();
    println!("{}", box_top());
    println!("{}", box_center("HORUS IPC Latency Benchmark v2.0"));
    println!("{}", box_center("Per-Message RDTSC-Instrumented Latency"));
    println!("{}", box_sep());

    let model = truncate(&platform.cpu.model, BOX_W - 8);
    println!("{}", box_left(&format!("CPU: {}", model)));
    println!(
        "{}",
        box_left(&format!(
            "Cores: {} physical / {} logical, NUMA: {} node(s)",
            platform.cpu.physical_cores, platform.cpu.logical_cores, platform.numa_nodes,
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "RDTSC: {:.2} GHz, ~{}ns overhead, constant_tsc: {}",
            cal.freq_hz / 1e9,
            cal.cycles_to_ns(cal.overhead_cycles),
            if has_constant_tsc() { "YES" } else { "NO" },
        ))
    );

    let gov = platform.cpu_governor.as_deref().unwrap_or("unknown");
    println!(
        "{}",
        box_left(&format!(
            "Governor: {}, VM: {}",
            gov,
            if platform.virtualized { "Yes" } else { "No" },
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "Pinning: main={}, aux={}, pub2={}, cons={}, cons2={} ({})",
            core_main(),
            core_aux(),
            core_pub2(),
            core_child_cons(),
            core_cons2(),
            if cores().topology_verified {
                "distinct physical cores"
            } else {
                "UNVERIFIED -- topology unreadable"
            },
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "Config: {} iterations, {} warmup, {} boot msgs",
            ITERATIONS, WARMUP, MIGRATION_BOOT,
        ))
    );
    println!(
        "{}",
        box_left(&format!(
            "Deadline budget: {} ns (declared, not a HORUS guarantee)",
            deadline_ns(),
        ))
    );

    println!("{}", box_sep());
    println!("{}", box_left(&format!("Date: {}", now)));
    if let Some(ref c) = commit {
        println!("{}", box_left(&format!("Commit: {}", c)));
    }
    println!("{}", box_bot());
    println!();
}

/// Check the machine can support the measurement this binary claims to make.
///
/// Returns false for conditions that make the numbers unattributable to the code
/// under test — too few CPUs to place the threads, or a core assignment that
/// puts two measured threads on one physical core. Those are not warnings: a
/// benchmark that could not place its threads has measured the scheduler.
fn validate_platform(platform: &PlatformInfo) -> bool {
    let mut warnings = Vec::new();
    let mut blockers = Vec::new();

    if !has_constant_tsc() {
        warnings
            .push("constant_tsc not detected -- cross-process RDTSC may be inaccurate".to_string());
    }
    if let Some(ref gov) = platform.cpu_governor {
        if gov != "performance" {
            warnings.push(format!(
                "CPU governor is '{}' (want 'performance'). Run: sudo cpupower frequency-set -g performance",
                gov
            ));
        }
    }
    if platform.virtualized {
        warnings.push("Running in VM -- RDTSC timing may be unreliable".to_string());
    }

    // The suite pins five threads plus up to eleven stress children. Below the
    // five it needs, every pin fails and every row is an unpinned number wearing
    // a pinned number's label.
    let assignment = cores();
    let needed = assignment.all().iter().copied().max().unwrap_or(0) + 1;
    if platform.cpu.logical_cores < needed {
        blockers.push(format!(
            "{} logical CPUs available, {} needed for the core assignment {:?}",
            platform.cpu.logical_cores,
            needed,
            assignment.all()
        ));
    }

    if !assignment.topology_verified {
        blockers.push(format!(
            "could not read thread_siblings_list; falling back to {:?}, whose core locality \
             is unverified -- two of these may be SMT siblings, which moves cross-core \
             latency by roughly 2x",
            FALLBACK_CORES
        ));
    } else {
        // Belt and braces: prove the assignment pairwise.
        let all = assignment.all();
        for (i, &a) in all.iter().enumerate() {
            for &b in &all[i + 1..] {
                if share_physical_core(a, b) == Some(true) {
                    blockers.push(format!("CPUs {} and {} are SMT siblings", a, b));
                }
            }
        }
    }

    for w in &warnings {
        eprintln!("  WARNING: {}", w);
    }
    for b in &blockers {
        eprintln!("  BLOCKER: {}", b);
    }
    if !warnings.is_empty() || !blockers.is_empty() {
        println!();
    }
    blockers.is_empty()
}

// ============================================================================
// Same-Process Benchmarks
// ============================================================================

/// Same-thread self-loop (role=Both) -- one Topic instance sends and receives.
/// Resolves to SpscShm and takes the role=Both inlined fast path; measures
/// send() latency with RDTSC overhead subtracted. This is the L1-hot case.
///
/// **Batched sends**: We send in batches of 128, then drain outside the
/// measurement window. This avoids icache/branch-predictor pollution from
/// alternating send/recv calls, which inflated the self-loop send latency.
fn bench_same_thread_selfloop(timer: &PrecisionTimer) -> ScenarioResult {
    let topic: Topic<CmdVel> = Topic::new("bench_dc_v2").unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    // Warmup: batch send + drain to warm caches without polluting measurement
    for _ in 0..WARMUP {
        topic.send(msg);
    }
    while topic.recv().is_some() {}

    let backend = topic.backend_name().to_string();

    // Measure send-only latency in batches.
    // Ring capacity is 256 for CmdVel (16 bytes). Use batch of 128 (half capacity).
    // Each batch: measure 128 sends, then drain outside measurement window.
    const BATCH: u64 = 128;
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    let mut sent = 0u64;
    while sent < ITERATIONS {
        let n = std::cmp::min(BATCH, ITERATIONS - sent);
        for _ in 0..n {
            serialize();
            let start = rdtsc();
            topic.send(std::hint::black_box(msg));
            let end = rdtscp();
            latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
        }
        sent += n;
        // Drain outside measurement window — icache stays warm for sends
        while topic.recv().is_some() {}
    }

    ScenarioResult {
        name: "SameThread",
        backend,
        expected_backend: "SpscShm",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Same-process cross-thread 1P-1C (resolves to SpscShm).
/// Measures send() latency on producer thread while consumer spins on another core.
fn bench_spsc_same_proc(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_si_v2_{}", std::process::id());
    let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let done_c = done.clone();
    let backend_out = Arc::new(std::sync::Mutex::new(String::new()));
    let backend_c = backend_out.clone();
    let ready = Arc::new(AtomicBool::new(false));
    let ready_c = ready.clone();

    // Pre-fill ring to establish SpscShm before consumer starts
    for _ in 0..2000 {
        producer.send(msg);
    }

    // Consumer thread pinned to separate core
    let handle = thread::spawn(move || {
        pin_to(core_aux(), "aux worker thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 1000 && Instant::now() < deadline {
            if consumer.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        *backend_c.lock().unwrap() = consumer.backend_name().to_string();
        ready_c.store(true, Ordering::Release);

        while !done_c.load(Ordering::Relaxed) {
            if consumer.recv().is_none() {
                spin_loop();
            }
        }
        while consumer.recv().is_some() {}
    });

    // Wait for consumer ready
    while !ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Extra warmup after consumer is ready
    for _ in 0..WARMUP {
        producer.send(msg);
    }
    thread::sleep(5_u64.ms());

    // Measure send latency
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer.send(msg);
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(50_u64.ms());
    done.store(true, Ordering::Relaxed);
    handle.join().unwrap();

    let backend = backend_out.lock().unwrap().clone();

    ScenarioResult {
        name: "CrossThread-1P1C",
        backend,
        expected_backend: "SpscShm",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Same-process cross-thread 2P-1C (resolves to MpscShm).
/// Measures send() latency on main thread while second producer runs on another core.
/// Multi-producer contention via CAS loop.
fn bench_mpsc_same_proc(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_mi_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let cons_ready = Arc::new(AtomicBool::new(false));
    let p2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer1 only (registers it on main thread)
    for _ in 0..2000 {
        producer1.send(msg);
    }

    // Consumer thread pinned to core_aux()
    let done_c = done.clone();
    let cons_ready_c = cons_ready.clone();
    let cons_handle = thread::spawn(move || {
        pin_to(core_aux(), "aux worker thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 1000 && Instant::now() < deadline {
            if consumer.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        cons_ready_c.store(true, Ordering::Release);
        while !done_c.load(Ordering::Relaxed) {
            if consumer.recv().is_none() {
                spin_loop();
            }
        }
        while consumer.recv().is_some() {}
    });

    // Wait for consumer to register
    while !cons_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Producer2 thread pinned to core_pub2()
    let done_p2 = done.clone();
    let p2_ready_c = p2_ready.clone();
    let p2_handle = thread::spawn(move || {
        pin_to(core_pub2(), "second producer thread");
        // Send to register as 2nd publisher, triggers MpscShm migration
        for _ in 0..2000 {
            producer2.send(msg);
        }
        p2_ready_c.store(true, Ordering::Release);
        // Keep sending to create realistic multi-producer contention
        while !done_p2.load(Ordering::Relaxed) {
            producer2.send(msg);
        }
    });

    // Wait for producer2 to register
    while !p2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Extra warmup after all participants registered
    for _ in 0..WARMUP {
        producer1.send(msg);
    }
    thread::sleep(5_u64.ms());

    let backend = producer1.backend_name().to_string();

    // Measure send() latency on main thread (contended with producer2)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer1.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(50_u64.ms());
    done.store(true, Ordering::Relaxed);
    cons_handle.join().unwrap();
    p2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-MP1C",
        backend,
        expected_backend: "MpscShm",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: Some("contended: 2 producers active"),
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Same-process 1-producer / 2-consumer, POD (resolves to PodShm broadcast).
/// Each consumer receives every message (broadcast semantics, no tail contention).
fn bench_spmc_same_proc(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_smi_{}", std::process::id());
    let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let c1_ready = Arc::new(AtomicBool::new(false));
    let c2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer (registers on main thread)
    for _ in 0..2000 {
        producer.send(msg);
    }

    // Consumer 1 thread pinned to core_aux()
    let done_c1 = done.clone();
    let c1_ready_c = c1_ready.clone();
    let c1_handle = thread::spawn(move || {
        pin_to(core_aux(), "aux worker thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 500 && Instant::now() < deadline {
            if consumer1.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c1_ready_c.store(true, Ordering::Release);
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_none() {
                spin_loop();
            }
        }
        while consumer1.recv().is_some() {}
    });

    // Wait for consumer1 to register
    while !c1_ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Consumer 2 thread pinned to core_cons2()
    let done_c2 = done.clone();
    let c2_ready_c = c2_ready.clone();
    let c2_handle = thread::spawn(move || {
        pin_to(core_cons2(), "second consumer thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 500 && Instant::now() < deadline {
            if consumer2.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c2_ready_c.store(true, Ordering::Release);
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_none() {
                spin_loop();
            }
        }
        while consumer2.recv().is_some() {}
    });

    // Wait for consumer2 to register
    while !c2_ready.load(Ordering::Acquire) {
        producer.send(msg);
        spin_loop();
    }

    // Extra warmup after all participants registered
    for _ in 0..WARMUP {
        producer.send(msg);
    }
    thread::sleep(5_u64.ms());

    let backend = producer.backend_name().to_string();

    // Measure send() latency (single producer, 2 consumers draining)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(50_u64.ms());
    done.store(true, Ordering::Relaxed);
    c1_handle.join().unwrap();
    c2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-1PMC",
        backend,
        expected_backend: "PodShm",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Same-process 2-producer / 2-consumer, POD (resolves to PodShm broadcast).
/// Measures send() latency with multi-producer contention and multi-consumer drain.
fn bench_mpmc_same_proc(timer: &PrecisionTimer) -> ScenarioResult {
    let topic_name = format!("bench_mmi_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    let done = Arc::new(AtomicBool::new(false));
    let c1_ready = Arc::new(AtomicBool::new(false));
    let c2_ready = Arc::new(AtomicBool::new(false));
    let p2_ready = Arc::new(AtomicBool::new(false));

    // Pre-fill from producer1 (registers on main thread)
    for _ in 0..2000 {
        producer1.send(msg);
    }

    // Consumer 1 thread pinned to core_aux()
    let done_c1 = done.clone();
    let c1_ready_c = c1_ready.clone();
    let c1_handle = thread::spawn(move || {
        pin_to(core_aux(), "aux worker thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 500 && Instant::now() < deadline {
            if consumer1.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c1_ready_c.store(true, Ordering::Release);
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_none() {
                spin_loop();
            }
        }
        while consumer1.recv().is_some() {}
    });

    // Wait for consumer1
    while !c1_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Consumer 2 thread pinned to core_cons2()
    let done_c2 = done.clone();
    let c2_ready_c = c2_ready.clone();
    let c2_handle = thread::spawn(move || {
        pin_to(core_cons2(), "second consumer thread");
        let mut count = 0u64;
        let deadline = Instant::now() + 5_u64.secs();
        while count < 500 && Instant::now() < deadline {
            if consumer2.recv().is_some() {
                count += 1;
            } else {
                spin_loop();
            }
        }
        c2_ready_c.store(true, Ordering::Release);
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_none() {
                spin_loop();
            }
        }
        while consumer2.recv().is_some() {}
    });

    // Wait for consumer2
    while !c2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Producer2 thread pinned to core_pub2()
    let done_p2 = done.clone();
    let p2_ready_c = p2_ready.clone();
    let p2_handle = thread::spawn(move || {
        pin_to(core_pub2(), "second producer thread");
        for _ in 0..2000 {
            producer2.send(msg);
        }
        p2_ready_c.store(true, Ordering::Release);
        while !done_p2.load(Ordering::Relaxed) {
            producer2.send(msg);
        }
    });

    // Wait for producer2
    while !p2_ready.load(Ordering::Acquire) {
        producer1.send(msg);
        spin_loop();
    }

    // Extra warmup after all registered
    for _ in 0..WARMUP {
        producer1.send(msg);
    }
    thread::sleep(5_u64.ms());

    let backend = producer1.backend_name().to_string();

    // Measure send() latency (contended: 2 producers, 2 consumers)
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    for _ in 0..ITERATIONS {
        serialize();
        let start = rdtsc();
        producer1.send(std::hint::black_box(msg));
        let end = rdtscp();
        latencies.push(cal.cycles_to_ns(end.wrapping_sub(start).saturating_sub(overhead)));
    }

    thread::sleep(50_u64.ms());
    done.store(true, Ordering::Relaxed);
    c1_handle.join().unwrap();
    c2_handle.join().unwrap();
    p2_handle.join().unwrap();

    ScenarioResult {
        name: "CrossThread-MPMC",
        backend,
        expected_backend: "PodShm",
        measurement: "send",
        latencies_ns: latencies,
        total_sent: ITERATIONS,
        total_received: ITERATIONS,
        note: Some("contended: 2 producers, 2 consumers"),
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

// ============================================================================
// Cross-Process Benchmarks
// ============================================================================

/// SpscShm -- cross-process 1 publisher, 1 consumer.
/// Parent = consumer, child = publisher.
/// Measures one-way latency via RDTSC timestamps in message payload.
fn bench_spsc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_spsc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register as consumer

    let child_count = PODSHM_MSGS_PER_PUB;
    // Paced publisher: prevents ring overflow that causes queuing delay.
    // Without pacing, producer outruns consumer → ring fills → measured latency
    // shows queuing delay (~3µs) instead of true wire latency (~300ns).
    let mut child = spawn_paced_publisher(&topic_name, child_count, core_aux());

    // Wait for publisher to register and migration to occur
    wait_for_topology(&consumer, 1, 1, 5_u64.secs());
    let migration_recv = wait_for_messages(&consumer, 100, 10_u64.secs());

    // Force migration detection
    consumer.check_migration_now();
    let backend = consumer.backend_name().to_string();

    // Collect: first WARMUP discarded, then ITERATIONS measured
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);
    child.wait().ok();

    let total_received = migration_recv + measure_recv;
    ScenarioResult {
        name: "CrossProc-1P1C",
        backend,
        expected_backend: "SpscShm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT + child_count,
        total_received,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// MpscShm -- cross-process 2 publishers, 1 consumer.
/// Parent = consumer, 2 children = publishers.
///
/// Previous bugs fixed:
/// - Used only 52.5K msgs/pub (publishers ran dry during measurement)
/// - No wait_for_topology() → parent never detected MpscShm migration
/// - No check_migration_now() → measured SpscShm latency, not MpscShm
fn bench_mpsc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_mpsc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv();

    // Use same large message count as PodShm — publishers must outlast setup + measurement.
    // Previous count (52.5K) caused publishers to finish before measurement started.
    let msgs_per_pub = PODSHM_MSGS_PER_PUB;

    // Spawn pub1 → SpscShm migration (1P, 1C, cross-process)
    let mut pub1 = spawn_paced_publisher(&topic_name, msgs_per_pub, core_aux());
    let migration1 = wait_for_messages(&consumer, 100, 10_u64.secs());

    // Spawn pub2 → MpscShm migration (2P, 1C, cross-process)
    let mut pub2 = spawn_paced_publisher(&topic_name, msgs_per_pub, core_pub2());

    // Wait for pub2 to actually register in the topology
    wait_for_topology(&consumer, 2, 1, 5_u64.secs());

    // Drain migration-era messages
    let migration2 = wait_for_messages(&consumer, 200, 5_u64.secs());

    // Force migration detection so parent sees MpscShm before measurement
    consumer.check_migration_now();

    let backend = consumer.backend_name().to_string();

    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);

    // Kill publishers (don't wait for 10M msgs to finish)
    let _ = pub1.kill();
    let _ = pub2.kill();
    pub1.wait().ok();
    pub2.wait().ok();

    let total_received = migration1 + migration2 + measure_recv;

    ScenarioResult {
        name: "CrossProc-2P1C",
        backend,
        expected_backend: "MpscShm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: total_received, // publishers killed mid-stream; actual sent is unknown
        total_received,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Cross-process 1 publisher, 2 consumers.
/// Parent = consumer 1, child = consumer 2, child = publisher.
/// Measures one-way latency via RDTSC timestamps in message payload.
///
/// The backend is `PodShm`, not `SpmcShm`, and that is the policy rather than a
/// defect. `detect_optimal_backend` routes 1-to-many POD to broadcast on
/// purpose: `SpmcShm`'s consumers share one tail and COMPETE for messages, so a
/// fast subscriber would starve the others — wrong for pub/sub, where every
/// subscriber is supposed to see every message.
///
/// This scenario asserted `SpmcShm` and printed `MISMATCH` on every run,
/// reporting the implementation as broken when the expectation was. A false
/// alarm in benchmark output is worse than no check: it trains the reader to
/// ignore the line that would matter if the backend really did change.
fn bench_spmc_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_spmc_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register parent as consumer

    // Spawn child consumer first (registers as 2nd subscriber)
    let child_count = PODSHM_MSGS_PER_PUB;
    let mut child_cons = spawn_consumer(&topic_name, child_count, core_child_cons());
    thread::sleep(200_u64.ms());

    // Paced publisher: even with 2 consumers, the parent consumer (doing measurement
    // work: RDTSC + Vec::push) is slower, so CAS contention causes bursty delivery.
    // Pacing prevents ring overflow that inflates measured latency.
    let mut child_pub = spawn_paced_publisher(&topic_name, child_count, core_aux());

    // Wait for publisher to register and topology to settle (1P, 2S → PodShm)
    wait_for_topology(&consumer, 1, 2, 5_u64.secs());
    let migration_recv = wait_for_messages(&consumer, 100, 10_u64.secs());

    // Force migration detection
    consumer.check_migration_now();
    let backend = consumer.backend_name().to_string();

    // Collect one-way latencies
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);
    child_pub.wait().ok();
    child_cons.wait().ok();

    let total_received = migration_recv + measure_recv;
    ScenarioResult {
        name: "CrossProc-1PMC",
        backend,
        expected_backend: "PodShm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: MIGRATION_BOOT + child_count,
        total_received,
        // Without this the row prints "98.9% loss" beside a 196ns median and
        // reads as though HORUS drops 99 messages in 100. It does not: 1-to-many
        // POD resolves to PodShm, which is latest-value broadcast with no
        // backpressure, so an unthrottled publisher overwrites slots a consumer
        // has not read yet — by design, and the same design the
        // CrossProc-PodShm row already annotates. The delivered messages are
        // what the latency figure is computed from.
        note: Some(
            "PodShm broadcast: unread slots are overwritten by design, so the \
             loss figure is producer pacing, not dropped delivery",
        ),
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// PodShm -- cross-process 2 publishers, 2 consumers, POD type.
/// Parent + child = consumers, 2 children = publishers.
///
/// For POD types (like CmdVel), multi-pub/multi-sub cross-process always selects
/// PodShm (zero-copy atomic slot). MpmcShm only activates for non-POD types
/// (which require serialization). Both share the same dispatch paths.
///
/// Spawn order: consumers first, then publishers. This ensures PodShm is the
/// final topology and publishers are still running during measurement (previous
/// design spawned publishers first, which caused them to finish before measurement).
fn bench_pod_shm(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!("bench_pod_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register as consumer

    // PodShm needs many more messages than other scenarios — publishers must
    // still be running during measurement (setup takes ~350ms, hot loop at
    // ~100ns/msg means 105K msgs finishes in ~10ms, way too fast).
    let msgs_per_pub = PODSHM_MSGS_PER_PUB;

    // Step 1: Spawn child consumer FIRST so both consumers are present before publishers.
    // Child consumer receives from both publishers (broadcast), so count = msgs_per_pub * 2.
    let mut child_cons = spawn_consumer(&topic_name, msgs_per_pub * 2, core_child_cons());
    thread::sleep(200_u64.ms()); // Wait for child to register

    // Step 2: Spawn pub1 (paced) → topology: 1P, 2S, cross-proc → PodShm
    // Paced publishers prevent ring overflow that causes the consumer to read
    // stale messages with old timestamps, inflating measured latency.
    let mut pub1 = spawn_paced_publisher(&topic_name, msgs_per_pub, core_aux());
    let migration1 = wait_for_messages(&consumer, 100, 10_u64.secs());

    // Step 3: Spawn pub2 (paced) → topology: 2P, 2S, cross-proc, POD → PodShm migration
    let mut pub2 = spawn_paced_publisher(&topic_name, msgs_per_pub, core_pub2());

    // Wait for pub2 to actually register (it sleeps 100ms at startup).
    // We need pubs=2 visible in the header before migration detection works.
    wait_for_topology(&consumer, 2, 2, 5_u64.secs());

    // Drain any queued messages from before the topology settled
    let migration2 = wait_for_messages(&consumer, 200, 5_u64.secs());

    // Step 4: Force migration check so parent detects PodShm before measurement
    consumer.check_migration_now();

    let backend = consumer.backend_name().to_string();

    // Step 5: Measure — publishers are still running their 10M-message hot loops.
    // Use broadcast-aware collector that tracks freshness and skip-aheads.
    let (latencies, freshness, measure_recv, skips) =
        collect_pod_shm(&consumer, WARMUP, ITERATIONS, cal);

    // Don't wait for publishers to finish all 10M msgs — just kill them
    let _ = pub1.kill();
    let _ = pub2.kill();
    let _ = child_cons.kill();
    pub1.wait().ok();
    pub2.wait().ok();
    child_cons.wait().ok();

    let total_received = migration1 + migration2 + measure_recv;

    // Delivery ratio: messages consumed vs messages produced between reads
    let total_produced_during_meas: u64 = freshness.iter().sum();
    let delivery_ratio = if total_produced_during_meas > 0 {
        Some(freshness.len() as f64 / total_produced_during_meas as f64)
    } else {
        None
    };

    ScenarioResult {
        name: "CrossProc-PodShm",
        backend,
        expected_backend: "PodShm",
        measurement: "broadcast",
        latencies_ns: latencies,
        total_sent: total_received, // publishers killed mid-stream; actual sent is unknown
        total_received,
        note: Some("broadcast/latest-value — skipped messages are by design"),
        freshness_samples: Some(freshness),
        delivery_ratio,
        skip_count: Some(skips),
    }
}

// ============================================================================
// Cross-Process Helpers
// ============================================================================

/// Spawn a paced publisher that inserts spin_loops between sends.
/// Prevents ring overflow and queuing delay in 1P1C scenarios where
/// the producer would otherwise outrun the consumer.
fn spawn_paced_publisher(topic: &str, count: u64, core: usize) -> std::process::Child {
    Command::new(std::env::current_exe().unwrap())
        .args([
            "--child-publisher",
            topic,
            &count.to_string(),
            &core.to_string(),
            "--paced",
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn paced child publisher")
}

fn spawn_consumer(topic: &str, count: u64, core: usize) -> std::process::Child {
    Command::new(std::env::current_exe().unwrap())
        .args([
            "--child-consumer",
            topic,
            &count.to_string(),
            &core.to_string(),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn child consumer")
}

/// Wait until the topic header shows at least `min_pubs` publishers and `min_subs` subscribers.
fn wait_for_topology(topic: &Topic<CmdVel>, min_pubs: u32, min_subs: u32, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    while topic.pub_count() < min_pubs || topic.sub_count() < min_subs {
        // Drain any messages while waiting (keeps lease alive)
        let _ = topic.recv();
        spin_loop();
        if Instant::now() > deadline {
            eprintln!(
                "  [warn] topology timeout: wanted pubs>={} subs>={}, got pubs={} subs={}",
                min_pubs,
                min_subs,
                topic.pub_count(),
                topic.sub_count()
            );
            break;
        }
    }
}

/// Spin-receive until `count` messages arrive or timeout.
///
/// Deadline check is amortized (every 4096 polls) to avoid ~108ns Instant::now()
/// overhead on every iteration, which would dominate cross-process wire latency.
fn wait_for_messages(consumer: &Topic<CmdVel>, count: u64, timeout: Duration) -> u64 {
    let mut received = 0u64;
    let deadline = Instant::now() + timeout;
    let mut polls = 0u64;
    while received < count {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if consumer.recv().is_some() {
            received += 1;
        } else {
            spin_loop();
        }
    }
    received
}

/// Collect cross-process latencies from RDTSC timestamps in message payload.
///
/// Phase 1: Discard `warmup` messages (cache/TLB warming).
/// Phase 1.5: Drain all stale messages from the ring buffer.
/// Phase 2: Collect up to `iterations` per-message one-way latencies.
///
/// **Critical**: The drain phase between warmup and measurement eliminates stale
/// messages that accumulated in the ring while the consumer was processing warmup.
/// Without this drain, measurement would start with old-timestamp messages, inflating
/// latency by hundreds of nanoseconds.
///
/// **Critical**: Deadline and idle checks are amortized (every 4096 polls) to avoid
/// injecting ~108ns Instant::now() overhead into the measurement hot loop. This is
/// essential for accurate sub-microsecond latency measurement.
///
/// Returns (latencies_ns, total_messages_received_in_both_phases).
fn collect_cross_proc(
    consumer: &Topic<CmdVel>,
    warmup: u64,
    iterations: u64,
    cal: &RdtscCalibration,
) -> (Vec<u64>, u64) {
    let mut total = 0u64;
    let deadline = Instant::now() + TIMEOUT;
    let mut last_recv_cycles = rdtsc(); // Use RDTSC for idle detection too
    let idle_threshold = cal.ns_to_cycles(2_000_000_000); // 2 seconds in cycles (warmup)
    let idle_threshold_meas = cal.ns_to_cycles(500_000_000); // 500ms in cycles (measurement)
    let overhead = cal.overhead_cycles;

    // Phase 1: Warmup -- receive and discard (cache/TLB warming)
    let mut polls = 0u64;
    while total < warmup {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if consumer.recv().is_some() {
            total += 1;
            last_recv_cycles = rdtsc();
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold {
                break;
            }
            spin_loop();
        }
    }

    // No drain phase needed: producer pacing (~1µs/msg via 256 spin_loops) is
    // slow enough that the consumer processes each message before the next one
    // arrives. There is no queue buildup, so every message reflects true wire
    // latency (cache coherency + dispatch overhead), not queuing delay.

    // Phase 2: Measurement -- ZERO overhead hot loop
    // No Instant::now(), no unnecessary branches. Pure spin on recv().
    // RDTSC overhead (serialize+rdtsc on producer + rdtscp on consumer) is subtracted
    // from each sample, same as the same-process measurements.
    let mut latencies = Vec::with_capacity(iterations as usize);
    last_recv_cycles = rdtsc();
    polls = 0;
    while (latencies.len() as u64) < iterations {
        polls += 1;
        // Amortized deadline check: every 4096 polls (~1µs at full speed)
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if let Some(msg) = consumer.recv() {
            let recv_cycles = rdtscp();
            let send_cycles = msg.timestamp_ns;
            let delta = recv_cycles
                .wrapping_sub(send_cycles)
                .saturating_sub(overhead);
            latencies.push(cal.cycles_to_ns(delta));
            total += 1;
            last_recv_cycles = recv_cycles;
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold_meas {
                break; // Publisher likely finished
            }
            // No spin_loop() here — tight poll for minimum latency measurement.
            // PAUSE on Comet Lake costs ~140 cycles (~40ns), adding ~20ns avg
            // polling delay. For latency benchmarks, we want the tightest loop.
        }
    }

    (latencies, total)
}

/// Collect cross-process latencies with PodShm broadcast-aware freshness tracking.
///
/// Like `collect_cross_proc`, but additionally reads `header.sequence_or_head` after
/// each recv() to measure how far the producer advanced between consumer reads.
/// This captures PodShm's broadcast/overwrite semantics where "loss" is by design.
///
/// Returns (latencies_ns, freshness_samples, total_messages_received, skip_count).
/// - `freshness_samples[i]` = how many messages the producer wrote between read i-1 and read i
/// - skip_count = number of times freshness > 1 (consumer fell behind, skipped messages)
fn collect_pod_shm(
    consumer: &Topic<CmdVel>,
    warmup: u64,
    iterations: u64,
    cal: &RdtscCalibration,
) -> (Vec<u64>, Vec<u64>, u64, u64) {
    let seq_ptr = consumer.sequence_head_ptr();
    let overhead = cal.overhead_cycles;
    let deadline = Instant::now() + TIMEOUT;
    let idle_threshold = cal.ns_to_cycles(2_000_000_000); // 2s for warmup
    let idle_threshold_meas = cal.ns_to_cycles(500_000_000); // 500ms for measurement

    let mut total = 0u64;
    let mut last_recv_cycles = rdtsc();

    // Phase 1: Warmup -- receive and discard (cache/TLB warming)
    let mut polls = 0u64;
    while total < warmup {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if consumer.recv().is_some() {
            total += 1;
            last_recv_cycles = rdtsc();
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold {
                break;
            }
            spin_loop();
        }
    }

    // Snapshot producer head after warmup for freshness baseline
    let mut prev_head = if !seq_ptr.is_null() {
        // SAFETY: seq_ptr is non-null (checked above) and points to an AtomicU64 inside
        // the SHM-mapped TopicHeader. The SHM mapping remains live for the lifetime of
        // `consumer`, which is borrowed for the duration of this function.
        unsafe { (*seq_ptr).load(Ordering::Acquire) }
    } else {
        0
    };

    // Phase 2: Measurement with freshness tracking
    let mut latencies = Vec::with_capacity(iterations as usize);
    let mut freshness = Vec::with_capacity(iterations as usize);
    let mut skip_count = 0u64;
    last_recv_cycles = rdtsc();
    polls = 0;

    while (latencies.len() as u64) < iterations {
        polls += 1;
        if polls & 4095 == 0 && Instant::now() > deadline {
            break;
        }
        if let Some(msg) = consumer.recv() {
            let recv_cycles = rdtscp();
            let send_cycles = msg.timestamp_ns;
            let delta = recv_cycles
                .wrapping_sub(send_cycles)
                .saturating_sub(overhead);
            latencies.push(cal.cycles_to_ns(delta));
            total += 1;
            last_recv_cycles = recv_cycles;

            // Read current producer head for freshness calculation
            if !seq_ptr.is_null() {
                // SAFETY: seq_ptr is non-null (checked above) and points to an AtomicU64
                // inside the SHM-mapped TopicHeader. The SHM mapping remains live for the
                // lifetime of `consumer`, which is borrowed for the duration of this function.
                let head = unsafe { (*seq_ptr).load(Ordering::Acquire) };
                let advance = head.saturating_sub(prev_head);
                if advance > 0 {
                    freshness.push(advance);
                    if advance > 1 {
                        skip_count += 1;
                    }
                }
                prev_head = head;
            }
        } else {
            let now = rdtsc();
            if now.wrapping_sub(last_recv_cycles) > idle_threshold_meas {
                break;
            }
        }
    }

    (latencies, freshness, total, skip_count)
}

// ============================================================================
// Child Process Entry Points
// ============================================================================

fn run_child_publisher(topic_name: &str, count: u64, core: usize, paced: bool) {
    pin_to(core, "child publisher");
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();

    // Wait for parent consumer to register
    thread::sleep(100_u64.ms());

    // Boot messages: trigger cross-process backend migration
    for _ in 0..MIGRATION_BOOT {
        serialize();
        let t = rdtsc();
        topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
    }
    thread::sleep(50_u64.ms());

    eprintln!(
        "  [pub] PID={} core={} backend={} pubs={} subs={}{}",
        std::process::id(),
        core,
        topic.backend_name(),
        topic.pub_count(),
        topic.sub_count(),
        if paced { " (paced)" } else { "" },
    );

    // === MEASUREMENT HOT LOOP ===
    // RDTSC timestamp embedded in payload. No yield, no sleep.
    if paced {
        // Paced mode: insert spin_loops between sends so the consumer ALWAYS
        // processes each message before the next one arrives. This ensures zero
        // queue buildup, so measured one-way latency reflects true wire latency
        // (cache coherency + dispatch overhead), not queuing delay.
        //
        // 256 spin_loops ≈ 1024ns (~1µs). Consumer processes each message in
        // ~150-300ns (SHM reads + RDTSC + Vec::push), so ~700ns of idle time
        // between messages — more than enough headroom.
        for _ in 0..count {
            serialize();
            let t = rdtsc();
            topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
            for _ in 0..256 {
                spin_loop();
            }
        }
    } else {
        for _ in 0..count {
            serialize();
            let t = rdtsc();
            topic.send(CmdVel::with_timestamp(1.5, 0.8, t));
        }
    }
}

fn run_child_consumer(topic_name: &str, count: u64, core: usize) {
    pin_to(core, "child consumer");
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();
    let _ = topic.recv(); // Register as subscriber
    thread::sleep(50_u64.ms());

    let mut received = 0u64;
    let deadline = Instant::now() + TIMEOUT;
    while received < count && Instant::now() < deadline {
        if topic.recv().is_some() {
            received += 1;
        } else {
            spin_loop();
        }
    }
}

// ============================================================================
// Raw Atomic Probe: Hardware floor for cross-process latency
// ============================================================================

/// Measure the raw hardware floor for cross-process cache line transfer.
///
/// Uses a single AtomicU64 in shared memory (via HORUS Topic header's user field).
/// Writer: serialize() → rdtsc() → atomic store(ts) → paced wait
/// Reader: tight poll → rdtscp() → compute delta
///
/// This bypasses ALL framework overhead (fn ptrs, epoch guards, ring buffer logic)
/// to measure the pure cross-core cache line transfer latency.
fn bench_raw_atomic(timer: &PrecisionTimer) -> ScenarioResult {
    let cal = timer.calibration();
    let overhead = cal.overhead_cycles;

    // Create a topic just to get shared memory. We'll use a raw atomic in the
    // topic's SHM region to communicate between processes.
    let topic_name = format!("bench_raw_atom_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register to get SHM allocated

    // Spawn the atomic writer child
    let exe = std::env::current_exe().unwrap();
    let total_writes = WARMUP + ITERATIONS;
    let mut child = Command::new(&exe)
        .args([
            "--child-atomic-writer",
            &topic_name,
            &total_writes.to_string(),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("Failed to spawn atomic writer");

    // Wait for child to start writing
    thread::sleep(200_u64.ms());

    // Get pointer to sequence_or_head atomic — we'll use this as our raw atomic.
    // Both processes access the same SHM-mapped header.
    let seq_ptr = consumer.sequence_head_ptr();
    if seq_ptr.is_null() {
        child.wait().ok();
        return ScenarioResult {
            name: "RawAtomic",
            backend: "shm".to_string(),
            expected_backend: "shm",
            measurement: "one-way",
            latencies_ns: vec![],
            total_sent: total_writes,
            total_received: 0,
            note: Some("seq_ptr null"),
            freshness_samples: None,
            delivery_ratio: None,
            skip_count: None,
        };
    }
    // SAFETY: seq_ptr is non-null (null case returned early above) and points to an
    // AtomicU64 inside the SHM-mapped TopicHeader. The SHM mapping remains live for
    // the lifetime of `consumer`, which is in scope for the remainder of this function.
    // AtomicU64 has the same layout as u64, so the pointer is correctly aligned.
    let atom = unsafe { &*seq_ptr };

    // Phase 1: Warmup
    let mut last_val = 0u64;
    for _ in 0..WARMUP {
        loop {
            let v = atom.load(Ordering::Acquire);
            if v != last_val && v != 0 {
                last_val = v;
                break;
            }
        }
    }

    // Phase 2: Measurement — tight poll, no PAUSE, no framework code
    let mut latencies = Vec::with_capacity(ITERATIONS as usize);
    let deadline = Instant::now() + TIMEOUT;
    let mut polls = 0u64;
    while (latencies.len() as u64) < ITERATIONS {
        polls += 1;
        if polls & 16383 == 0 && Instant::now() > deadline {
            break;
        }
        let v = atom.load(Ordering::Acquire);
        if v != last_val && v != 0 {
            let end = rdtscp();
            let delta = end.wrapping_sub(v).saturating_sub(overhead);
            latencies.push(cal.cycles_to_ns(delta));
            last_val = v;
        }
    }

    child.wait().ok();

    // Not `WARMUP + ITERATIONS`: the measurement loop breaks on the deadline or
    // when the writer dies, so a literal made `loss_pct()` structurally 0 even
    // on a run that collected nothing.
    let received = WARMUP + latencies.len() as u64;

    ScenarioResult {
        name: "RawAtomic",
        backend: "shm".to_string(),
        expected_backend: "shm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: total_writes,
        total_received: received,
        note: None,
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

/// Child process: write RDTSC timestamps to a raw atomic in SHM.
fn run_child_atomic_writer(topic_name: &str, count: u64) {
    pin_to(core_aux(), "raw-atomic writer child");
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();
    // Send a dummy message to register as publisher and trigger SHM creation
    topic.send(CmdVel::with_timestamp(0.0, 0.0, 0));
    thread::sleep(100_u64.ms());

    let seq_ptr = topic.sequence_head_ptr();
    if seq_ptr.is_null() {
        eprintln!("  [raw-atomic] seq_ptr null, aborting");
        return;
    }
    // SAFETY: seq_ptr is non-null (null case returned early above) and points to an
    // AtomicU64 inside the SHM-mapped TopicHeader. The SHM mapping remains live for
    // the lifetime of `topic`, which is in scope for the remainder of this function.
    // AtomicU64 has the same layout as u64, so the pointer is correctly aligned.
    let atom = unsafe { &*seq_ptr };

    eprintln!(
        "  [raw-atomic] PID={} core={} writing {} timestamps",
        std::process::id(),
        core_aux(),
        count
    );

    for _ in 0..count {
        serialize();
        let t = rdtsc();
        atom.store(t, Ordering::Release);
        // Same pacing as other cross-process benchmarks
        for _ in 0..256 {
            spin_loop();
        }
    }
}

// ============================================================================
// Scalability Stress Tests
// ============================================================================

/// Assign CPU cores for stress test child processes.
/// Core 0 is reserved for the parent/main thread (measurement consumer).
fn assign_stress_cores(total_children: usize) -> Vec<usize> {
    // Ask the kernel which logical CPUs are on distinct physical cores, and take
    // the ones the measuring thread is not already using. The previous version
    // guessed -- `(i + 1) * 2` under six children, `i + 1` above -- which on a
    // 6C/12T part whose sibling lists are (0,6), (1,7), ... puts child 2 on
    // cpu 6, the sibling of the parent's cpu 0.
    let main = core_main();
    if let Some(distinct) = distinct_physical_cores(total_children + 1) {
        let chosen: Vec<usize> = distinct.into_iter().filter(|&c| c != main).collect();
        if chosen.len() >= total_children {
            return chosen[..total_children].to_vec();
        }
    }

    // Over-subscribed, or topology unreadable. Say so rather than silently
    // producing a number that looks like the others.
    eprintln!(
        "  [warn] {} stress children exceed the distinct physical cores available; \
         siblings will be shared and these rows are not comparable to the pinned ones",
        total_children
    );
    (0..total_children)
        .map(|i| (i + 1) % num_cpus_or(1).max(1))
        .collect()
}

fn num_cpus_or(default: usize) -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(default)
}

/// Generic stress benchmark: N publishers x M consumers, cross-process.
///
/// Parent process is the measurement consumer. Child consumers are spawned
/// to create the desired topology, then publishers are spawned with enough
/// messages to outlast setup + measurement.
fn bench_stress(
    name: &'static str,
    num_pubs: usize,
    num_cons: usize,
    timer: &PrecisionTimer,
) -> ScenarioResult {
    let cal = timer.calibration();
    let topic_name = format!(
        "bench_stress_{}_{}",
        name.to_lowercase().replace('-', "_"),
        std::process::id()
    );
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let _ = consumer.recv(); // Register parent as consumer

    let cores = assign_stress_cores(num_pubs + num_cons.saturating_sub(1));
    let msgs_per_pub = PODSHM_MSGS_PER_PUB;

    // Spawn child consumers (num_cons - 1 since parent is one)
    let mut child_consumers: Vec<std::process::Child> = Vec::new();
    for i in 0..num_cons.saturating_sub(1) {
        let core_idx = num_pubs + i;
        let core = cores.get(core_idx).copied().unwrap_or(0);
        let child = spawn_consumer(&topic_name, msgs_per_pub * num_pubs as u64, core);
        child_consumers.push(child);
    }
    if !child_consumers.is_empty() {
        thread::sleep(200_u64.ms()); // Wait for child consumers to register
    }

    // Spawn paced publishers
    let mut publishers: Vec<std::process::Child> = Vec::new();
    for i in 0..num_pubs {
        let core = cores.get(i).copied().unwrap_or(0);
        let child = spawn_paced_publisher(&topic_name, msgs_per_pub, core);
        publishers.push(child);
        // Stagger spawns slightly so each publisher registers before the next
        if i < num_pubs - 1 {
            thread::sleep(50_u64.ms());
        }
    }

    // Wait for full topology
    wait_for_topology(&consumer, num_pubs as u32, num_cons as u32, 10_u64.secs());

    // Drain migration-era messages
    let migration_recv = wait_for_messages(&consumer, 500, 5_u64.secs());

    // Force migration check so parent detects the final backend
    consumer.check_migration_now();

    let backend = consumer.backend_name().to_string();

    eprintln!(
        "  [stress] {} pubs={} subs={} backend={}",
        name,
        consumer.pub_count(),
        consumer.sub_count(),
        backend
    );

    // Collect measurements
    let (latencies, measure_recv) = collect_cross_proc(&consumer, WARMUP, ITERATIONS, cal);

    // Kill all children
    for mut p in publishers {
        let _ = p.kill();
        p.wait().ok();
    }
    for mut c in child_consumers {
        let _ = c.kill();
        c.wait().ok();
    }

    let total_received = migration_recv + measure_recv;

    // total_sent is unknown (publishers are killed mid-stream), so use
    // total_received. That makes `loss_pct()` structurally 0 for these rows,
    // which is why the note below says loss is not measured rather than letting
    // a reader take the 0 at face value.
    ScenarioResult {
        name,
        backend,
        expected_backend: "Shm",
        measurement: "one-way",
        latencies_ns: latencies,
        total_sent: total_received,
        total_received,
        note: Some("publishers killed mid-stream: messages sent is unknown, so loss is not measured for this row"),
        freshness_samples: None,
        delivery_ratio: None,
        skip_count: None,
    }
}

// ============================================================================
// Reporting: Detail
// ============================================================================

fn print_detail(r: &ScenarioResult) {
    let check = if r.backend_ok() { "ok" } else { "MISMATCH" };

    println!("  {} [{}]", r.name, r.measurement);
    println!(
        "  Backend: {} ({}, expected {})",
        r.backend_short(),
        check,
        r.expected_backend
    );

    if r.latencies_ns.is_empty() {
        println!("  NO SAMPLES -- topology did not route messages to parent consumer");
        println!("  Messages: {}/{} received", r.total_received, r.total_sent);
        println!();
        return;
    }

    let s = r.stats();
    // `outliers_removed` is a diagnostic and nothing more: no statistic printed
    // below excludes a single sample. Tukey's fence deletes the preemptions and
    // faults that jitter consists of, so a fenced mean, std dev or CV is bounded
    // by construction rather than measured.
    println!(
        "  Samples: {} ({} outside Tukey's fence; none excluded from any figure below)",
        s.count, s.outliers_removed
    );
    println!(
        "  Tail support: p99 <- {} obs, p99.9 <- {} obs, p99.99 <- {} obs (need {})",
        s.quantile_exceedances(99.0),
        s.quantile_exceedances(99.9),
        s.quantile_exceedances(99.99),
        MIN_TAIL_EXCEEDANCES,
    );

    if r.total_sent != r.total_received && r.measurement != "broadcast" {
        println!(
            "  Messages: {}/{} received ({:.1}% loss)",
            r.total_received,
            r.total_sent,
            r.loss_pct()
        );
    }

    if let Some(note) = r.note {
        println!("  Note: {}", note);
    }

    println!();
    println!(
        "    {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "p50", "p95", "p99", "p99.9", "p99.99", "max"
    );
    println!(
        "    {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        fmt_ns(s.median as u64),
        fmt_ns(s.p95),
        fmt_quantile(&s, 99.0, s.p99),
        fmt_quantile(&s, 99.9, s.p999),
        fmt_quantile(&s, 99.99, s.p9999),
        fmt_ns(s.max),
    );
    println!();

    println!(
        "    Mean: {}  CI95: [{:.1}, {:.1}]ns  StdDev: {:.1}ns  CV: {:.3} (unfiltered)",
        fmt_ns(s.mean as u64),
        s.ci_low,
        s.ci_high,
        s.std_dev,
        s.cv(),
    );
    println!(
        "    Jitter: {} (max-min)  IQR: [{}, {}]",
        fmt_ns(s.max - s.min),
        fmt_ns(s.p25),
        fmt_ns(s.p75),
    );

    let misses = r.deadline_misses();
    println!(
        "    Deadline: {} of {} samples over {} ({:.4}%)",
        misses,
        s.count,
        fmt_ns(deadline_ns()),
        if s.count > 0 {
            misses as f64 / s.count as f64 * 100.0
        } else {
            f64::NAN
        },
    );

    // Free self-check on the overhead subtraction: a latency benchmark cannot
    // observe a genuine 0 ns operation, so any zero is the once-calibrated
    // overhead MIN being subtracted from a sample it exceeded.
    let zeros = r.zero_samples();
    if zeros > 0 {
        println!(
            "    WARNING: {} sample(s) floored at 0 ns -- the overhead subtraction \
             over-corrected;\n             this run's median and mean are biased low.",
            zeros
        );
    }
    println!();
}

/// Print broadcast-specific metrics for PodShm scenarios.
/// Shows freshness distribution, delivery ratio, and skip-ahead count.
fn print_pod_shm_detail(r: &ScenarioResult) {
    println!("  Broadcast Metrics:");

    if let Some(ref freshness) = r.freshness_samples {
        if !freshness.is_empty() {
            let fresh_stats = Statistics::from_samples(freshness, 95.0, false);
            println!(
                "    Freshness (msgs between reads):  p50: {}  p95: {}  p99: {}  max: {}",
                fresh_stats.median as u64, fresh_stats.p95, fresh_stats.p99, fresh_stats.max,
            );
        }
    }

    if let Some(ratio) = r.delivery_ratio {
        println!("    Delivery ratio: {:.1}%", ratio * 100.0);
    }

    if let Some(skips) = r.skip_count {
        let total = r.freshness_samples.as_ref().map(|f| f.len()).unwrap_or(0);
        let skip_pct = if total > 0 {
            skips as f64 / total as f64 * 100.0
        } else {
            0.0
        };
        println!("    Skip-aheads: {} ({:.1}% of reads)", skips, skip_pct);
    }

    println!("    Note: PodShm is broadcast/latest-value — skipped messages are by design.");
    println!("    Latency above measures read freshness (time since last producer write),");
    println!("    not queuing delay. A \"fast\" consumer sees freshness=1 on every read.");
    println!();
}

// ============================================================================
// Reporting: Summary Table
// ============================================================================

fn print_summary(results: &[ScenarioResult]) {
    let w = BOX_W + 4;
    println!("{}", "=".repeat(w));
    println!(
        "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  Backend",
        "Scenario", "Type", "p50", "p95", "p99", "p99.9", "max"
    );
    println!("{}", "-".repeat(w));

    for r in results {
        if r.latencies_ns.is_empty() {
            println!(
                "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {} {}",
                r.name,
                r.measurement,
                "--",
                "--",
                "--",
                "--",
                "--",
                if r.backend_ok() { "ok" } else { "!!" },
                r.backend_short(),
            );
            continue;
        }

        let s = r.stats();
        let mark = if r.backend_ok() { "ok" } else { "!!" };

        println!(
            "  {:<18} {:>7}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {} {}",
            r.name,
            r.measurement,
            fmt_ns(s.median as u64),
            fmt_ns(s.p95),
            fmt_quantile(&s, 99.0, s.p99),
            fmt_quantile(&s, 99.9, s.p999),
            fmt_ns(s.max),
            mark,
            r.backend_short(),
        );
    }

    println!("{}", "=".repeat(w));
    println!(
        "  n/a[k] = too few observations (k) beyond that quantile to report it; need {}.",
        MIN_TAIL_EXCEEDANCES
    );
    println!("  max is a single observation and is reported, never compared.");
    println!();
}

// ============================================================================
// Reporting: Framework Overhead Analysis
// ============================================================================

fn print_overhead_analysis(results: &[ScenarioResult]) {
    // Find the RawAtomic hardware floor. `stats()` re-runs a 10,000-resample
    // bootstrap over every sample, so take it once and keep both figures.
    let floor = results
        .iter()
        .find(|r| r.name == "RawAtomic")
        .filter(|r| !r.latencies_ns.is_empty())
        .map(|r| r.stats());

    let Some(floor) = floor else { return };
    let floor_ns = floor.median as u64;
    let floor_p99 = floor.p99;

    println!(
        "─── Framework Overhead (vs hardware floor: {}ns) {}",
        floor_ns,
        "─".repeat(BOX_W - 55)
    );
    println!();

    let cross_proc = [
        ("CrossProc-1P1C", "SpscShm"),
        ("CrossProc-2P1C", "MpscShm"),
        ("CrossProc-1PMC", "PodShm"),
        ("CrossProc-PodShm", "PodShm"),
    ];

    for (name, label) in &cross_proc {
        if let Some(r) = results.iter().find(|r| r.name == *name) {
            if r.latencies_ns.is_empty() {
                continue;
            }
            let s = r.stats();
            let p50 = s.median as u64;
            // Signed. `saturating_sub` clamped this at 0, so a path measuring
            // *faster* than the floor -- which means the floor measurement is
            // wrong, or the two rows ran at different core frequencies -- read
            // as "0 ns of framework overhead", the best possible result, from
            // what is actually a broken comparison.
            let overhead = p50 as i64 - floor_ns as i64;
            let overhead_p99 = s.p99 as i64 - floor_p99 as i64;
            println!(
                "  {:<18} p50: {:>5}ns  over floor: {:>+6}ns   p99: {:>6}ns  over floor: {:>+7}ns  {}",
                name, p50, overhead, s.p99, overhead_p99, label
            );
        }
    }
    println!();
    println!("  A negative figure is not a fast path: it means the floor and the row were");
    println!("  not measured under the same conditions, and neither number is usable.");
    println!("  The floor itself moves by tens of ns between runs with core frequency and");
    println!("  load, so this whole block is an order-of-magnitude guide, not a measurement.");
    println!();
}

// ============================================================================
// Reporting: Methodology
// ============================================================================

fn print_methodology(cal: &RdtscCalibration) {
    println!("─── Methodology {}", "─".repeat(BOX_W - 18));
    println!();
    println!("  Measurement types:");
    println!("    send      = producer-side send() latency (RDTSC, overhead subtracted)");
    println!("    one-way   = producer-to-consumer via RDTSC timestamp in CmdVel.timestamp_ns");
    println!("    broadcast = like one-way, but tracks freshness/skip-aheads (PodShm)");
    println!();
    println!("  Statistical processing:");
    println!("    - NO outlier removal. Every figure above -- mean, std dev, CV, CI and");
    println!(
        "      every percentile -- is computed from all {} samples.",
        ITERATIONS
    );
    println!("      Tukey's 1.5x fence is computed and its count reported as a diagnostic");
    println!("      only. The fence deletes the preemptions and page faults that jitter");
    println!("      consists of; a CV taken after it cannot exceed the fence, which is why");
    println!("      the older tables that used one were withdrawn.");
    println!("    - Bootstrap 95% CI on the mean (10K resamples, fixed-seed LCG)");
    println!(
        "    - A percentile is printed only when >= {} observations lie beyond it.",
        MIN_TAIL_EXCEEDANCES
    );
    println!(
        "      At {} samples that permits p99.9 and forbids p99.99.",
        ITERATIONS
    );
    println!("    - max is one observation. It is reported, never compared between runs.");
    println!(
        "    - deadline_misses counts samples over the declared {} ns budget.",
        deadline_ns()
    );
    println!();
    println!("  Timing infrastructure:");
    println!(
        "    RDTSC overhead: ~{}ns (serialize + rdtsc/rdtscp pair)",
        cal.cycles_to_ns(cal.overhead_cycles)
    );

    // Measure Instant::now() for comparison
    let mut times: Vec<u64> = Vec::with_capacity(10_000);
    for _ in 0..10_000 {
        let start = Instant::now();
        std::hint::black_box(());
        times.push(start.elapsed().as_nanos() as u64);
    }
    times.sort_unstable();
    println!(
        "    Instant::now(): ~{}ns (for comparison)",
        times[times.len() / 2]
    );
    println!("    Same-process: RDTSC overhead subtracted from each sample");
    println!(
        "    Cross-process: RDTSC overhead subtracted (rdtsc on producer + rdtscp on consumer)"
    );
    println!();

    println!("  Known limitations:");
    println!("    - Topic selects backends based on topology and type (POD vs non-POD)");
    println!("    - MpmcShm only activates for non-POD types; POD multi-pub/sub uses PodShm");
    println!("    - Cross-process POD: zero-copy (memcpy + atomics). Non-POD: +bincode ser/deser");
    println!("    - RDTSC ticks at the nominal rate; the work costs a fixed number of CORE");
    println!("      cycles. Every nanosecond here is core_cycles / f_core, so a governor");
    println!("      that clocks the core down inflates every figure without changing any");
    println!("      other output. See the Clock Check block for this run's drift.");
    println!("    - The RDTSC overhead subtracted from each sample is a MIN captured once,");
    println!("      at whatever P-state held during calibration. When a later sample beats");
    println!("      it the sample is floored at 0 ns; those are counted and warned about");
    println!("      per scenario, and they bias the median and mean low.");
    println!("    - runs = 1. Nothing here measures run-to-run spread, which on this");
    println!("      harness has been observed at 2x for an unchanged binary. A single run");
    println!("      cannot support a claim about a change of less than that.");
    println!();
}

// ============================================================================
// JSON Output
// ============================================================================

/// Write the machine-readable report.
///
/// Fields that this binary does not measure are written as `NaN`, which
/// `serde_json` emits as `null`. They used to be written as `0.0` and `0` under
/// `Provenance::Measured`: `deadline_misses: 0` beside `deadline_threshold_ns: 0`,
/// and `run_variance: 0.0` beside `runs: 1` — an assertion of perfect
/// reproducibility from a single run. A null is a fact about the measurement; a
/// zero in a field named for violations is a false safety claim.
fn write_json_output(
    path: &str,
    platform: &PlatformInfo,
    results: &[ScenarioResult],
    with_raw: bool,
) {
    let mut report = BenchmarkReport::new(platform.clone());

    for r in results {
        let s = r.stats();

        let result = BenchmarkResult {
            provenance: Provenance::Measured,
            name: format!("all_paths_latency/{}", r.name),
            subject: format!("{} ({})", r.backend_short(), r.measurement),
            message_size: std::mem::size_of::<CmdVel>(),
            config: BenchmarkConfig {
                warmup_iterations: WARMUP as usize,
                iterations: ITERATIONS as usize,
                runs: 1,
                cpu_affinity: Some((core_main(), core_aux())),
                // No statistic in this report excludes a sample.
                // `statistics.outliers_removed` is a count of samples outside
                // Tukey's fence, kept as a diagnostic; nothing is removed.
                filter_outliers: false,
                confidence_level: 95.0,
            },
            platform: platform.clone(),
            timestamp: chrono::Utc::now().to_rfc3339(),
            // Off by default (~800 KB per scenario), on with --raw-samples.
            // Without them no percentile can be re-derived after a bug is fixed,
            // no paired test can be run, and the bimodality that a mid-run
            // frequency step leaves behind is undetectable. That is a size
            // trade-off, stated here; it is not a claim that the samples do not
            // matter.
            raw_latencies_ns: if with_raw {
                r.latencies_ns.clone()
            } else {
                Vec::new()
            },
            statistics: s.clone(),
            throughput: ThroughputMetrics {
                // This binary measures latency. `1e9 / mean` is not a measured
                // throughput: the cross-process producers are deliberately paced
                // at ~1 us/message, so the reciprocal of a one-way latency is a
                // number about nothing. It was stamped `Measured` all the same.
                messages_per_sec: f64::NAN,
                bytes_per_sec: f64::NAN,
                total_messages: r.total_received,
                total_bytes: r.total_received * std::mem::size_of::<CmdVel>() as u64,
                // Wall-clock duration of a scenario is dominated by process
                // spawn and topology settling, neither of which is timed here.
                duration_secs: f64::NAN,
            },
            determinism: DeterminismMetrics {
                // Unfiltered, from `Statistics::cv()`, so this field means the
                // same thing as the `cv` the other benchmark binaries write.
                cv: s.cv(),
                max_jitter_ns: s.max - s.min,
                p999: s.p999,
                // Reported, but at 100K samples this rests on ~10 observations;
                // `statistics.count` is the divisor a consumer needs to decide
                // whether it can be compared. See `fmt_quantile`.
                p9999: s.p9999,
                deadline_misses: r.deadline_misses(),
                deadline_threshold_ns: deadline_ns(),
                // runs = 1: there is no run-to-run variance to report, and 0.0
                // asserted that there was none.
                run_variance: f64::NAN,
            },
        };
        report.add_result(result);
    }

    match write_json_report(&report, path) {
        Ok(()) => println!("  JSON report written to: {}", path),
        Err(e) => eprintln!("  ERROR: Failed to write JSON report: {}", e),
    }
}

// ============================================================================
// Formatting Helpers
// ============================================================================

/// Format a tail quantile, or refuse to print one the sample count cannot
/// support.
///
/// A percentile is an estimate whose relative standard error is roughly
/// `1/sqrt(k)`, where `k` is the number of observations beyond it. At this
/// binary's 100,000 samples, `p99.9` rests on 100 observations (about +/-10%)
/// and `p99.99` rests on ten (about +/-32%) — the latter is the second largest
/// sample wearing a percentile's name, and "p99.99 improved 20%" is well inside
/// its own noise. Rather than print a number that cannot be compared, print the
/// exceedance count that disqualifies it.
fn fmt_quantile(s: &Statistics, p: f64, value: u64) -> String {
    let k = s.quantile_exceedances(p);
    if k >= MIN_TAIL_EXCEEDANCES {
        fmt_ns(value)
    } else {
        format!("n/a[{}]", k)
    }
}

fn fmt_ns(ns: u64) -> String {
    if ns < 10_000 {
        format!("{}ns", ns)
    } else if ns < 1_000_000 {
        format!("{:.1}us", ns as f64 / 1_000.0)
    } else if ns < 1_000_000_000 {
        format!("{:.1}ms", ns as f64 / 1_000_000.0)
    } else {
        format!("{:.2}s", ns as f64 / 1_000_000_000.0)
    }
}

fn truncate(s: &str, max: usize) -> &str {
    if s.len() > max {
        &s[..max]
    } else {
        s
    }
}

// Box-drawing helpers (fixed-width interior = BOX_W)
fn box_top() -> String {
    format!("╔{}╗", "═".repeat(BOX_W + 2))
}
fn box_bot() -> String {
    format!("╚{}╝", "═".repeat(BOX_W + 2))
}
fn box_sep() -> String {
    format!("╠{}╣", "═".repeat(BOX_W + 2))
}
fn box_center(text: &str) -> String {
    let pad = BOX_W.saturating_sub(text.len());
    let left = pad / 2;
    let right = pad - left;
    format!("║ {}{}{} ║", " ".repeat(left), text, " ".repeat(right))
}
fn box_left(text: &str) -> String {
    let content = if text.len() > BOX_W {
        &text[..BOX_W]
    } else {
        text
    };
    let pad = BOX_W - content.len();
    format!("║ {}{} ║", content, " ".repeat(pad))
}

fn detect_git_commit_short() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|output| {
            if output.status.success() {
                String::from_utf8(output.stdout)
                    .ok()
                    .map(|s| s.trim().to_string())
            } else {
                None
            }
        })
}
