//! Benchmark output formats, and the CI performance gate.
//!
//! Supports:
//! - JSON output for machine parsing and regression detection
//! - CSV output for spreadsheet analysis
//! - Comparison against a rolling baseline, tail-aware
//!
//! # The gate
//!
//! HORUS is safety-critical robotics middleware. Its figure of merit is
//! **worst-case latency and jitter first, median second**: a change that
//! improves the median and widens the tail is a regression here.
//!
//! The gate this module used to implement was, in full,
//! `regression = median > baseline.median * 1.05`. `p99_change_percent` was
//! computed two lines above it, stored in the struct, and never consulted. A
//! change that left the median untouched and added 50 µs to p99 reported `[OK]`
//! and printed "All benchmarks within acceptable range" — precisely the trade
//! this project must never accept, waved through by construction.
//!
//! It was also statistically empty on the axis it did check. 5% of an 80 ns
//! median is 4 ns, and repeated runs of *identical code* on this tree's harness
//! have produced medians of 63 ns and 105 ns, and 99 ns and 193 ns. A 5%
//! single-run-vs-single-run comparison against that spread is a coin flip.
//!
//! ## What replaced it
//!
//! 1. **A rolling baseline window**, not a single run. The baseline carries the
//!    last `max_runs` trunk runs and the gate derives both a robust center
//!    (median across runs) and a robust spread (1.4826·MAD) from it.
//! 2. **Self-calibrating bands.** Each metric's threshold is
//!    `max(relative, center + k·σ_observed, center + absolute_floor)`. When the
//!    harness is noisy the band widens on its own; as the harness is fixed
//!    (see the research items on RDTSC-vs-core-frequency, `codegen-units`, and
//!    P-state pinning) the window tightens and the band tightens with it, down
//!    to the relative floor. No one has to re-tune a constant for the gate to
//!    get sharper.
//! 3. **A three-valued verdict.** "Not detected" is not "not there".
//!    [`Verdict::Inconclusive`] is reported whenever the band is wider than the
//!    change the project actually cares about, so a green run never claims more
//!    than it measured.
//! 4. **Tail-shape metrics.** `p99/median` and `p999/median` are gated as
//!    first-class metrics. The dominant *characterized* noise source in this
//!    harness — RDTSC counts at the nominal rate while the work costs a fixed
//!    number of core cycles, so every sample is `core_cycles / f_core` and
//!    `f_core` is never recorded — is **multiplicative and common-mode across
//!    every percentile of a run**, and therefore cancels exactly in a ratio of
//!    two latencies from the same run. That makes tail *shape* the one tail
//!    statistic with a defensible noise model on a shared, frequency-scaling
//!    runner.
//! 5. **Report-only where the measurement cannot support a verdict.**
//!    `max`, `p99.99` and `max_jitter` are single-sample (or near-single-sample)
//!    order statistics; on a shared CI runner one host preemption or VM steal
//!    event puts an entire millisecond in `max`, and that is a property of the
//!    runner, not of the code. They are reported, never blocking, until the
//!    only excursion no runner can explain — see `gross_multiplier`.
//!
//! ## What is honestly not gated yet
//!
//! Absolute `p99`/`p999` are **report-only**. Nothing in this tree has ever
//! measured their run-to-run spread, and gating them at a percentage would be
//! inventing a number. They become blocking either by a gross multiplier (an
//! excursion no plausible combination of P-state and core placement explains)
//! or, on trunk only, by regressing for `consecutive_to_block` runs in a row —
//! the probability of which under noise is the per-run false-positive rate
//! cubed. See [`RegressionPolicy`] for the full reasoning and the TODOs that
//! would let them be promoted to blocking.

use crate::{BenchmarkResult, PlatformInfo};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Complete benchmark report containing multiple results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkReport {
    /// Report version for forward compatibility
    pub version: String,
    /// Timestamp when report was generated
    pub generated_at: String,
    /// Platform the benchmarks ran on
    pub platform: PlatformInfo,
    /// Individual benchmark results
    pub results: Vec<BenchmarkResult>,
    /// Git commit hash (if available)
    pub git_commit: Option<String>,
    /// Git branch (if available)
    pub git_branch: Option<String>,
    /// CI run ID (if available)
    pub ci_run_id: Option<String>,
}

impl BenchmarkReport {
    /// Create a new report with current timestamp
    pub fn new(platform: PlatformInfo) -> Self {
        Self {
            version: "1.0.0".to_string(),
            generated_at: chrono::Utc::now().to_rfc3339(),
            platform,
            results: Vec::new(),
            git_commit: detect_git_commit(),
            git_branch: detect_git_branch(),
            ci_run_id: std::env::var("CI_RUN_ID")
                .or_else(|_| std::env::var("GITHUB_RUN_ID"))
                .ok(),
        }
    }

    /// Add a benchmark result
    pub fn add_result(&mut self, result: BenchmarkResult) {
        self.results.push(result);
    }

    /// Compare against a single baseline report.
    ///
    /// Kept for callers that only have one baseline run. **A one-run baseline
    /// cannot estimate the harness's run-to-run spread**, so this path applies
    /// [`RegressionPolicy::no_noise_fallback_rel`] to everything and can only
    /// block on a gross regression. Every other verdict comes back
    /// [`Verdict::Inconclusive`] — which is the honest description of a
    /// one-run-vs-one-run comparison on a harness that has produced 63 ns and
    /// 105 ns medians from the same binary.
    ///
    /// Prefer [`BaselineHistory::compare`] with a real window.
    pub fn compare(&self, baseline: &BenchmarkReport) -> ComparisonReport {
        let mut history = BaselineHistory::new(1);
        history.push(baseline);
        history.compare(&[self], &RegressionPolicy::default())
    }

    /// Reduce this report to the small per-benchmark summary the baseline
    /// stores. Deliberately drops `raw_latencies_ns`: a baseline window of 20
    /// runs must stay small enough to live in a CI cache entry.
    pub fn summarize(&self) -> Vec<BaselineEntry> {
        self.results
            .iter()
            .map(BaselineEntry::from_result)
            .collect()
    }
}

// =============================================================================
// Metrics
// =============================================================================

/// A statistic the gate tracks.
///
/// The ordering here is the order the report prints them in: central estimate
/// first, then out along the tail, then the dimensionless tail-shape ratios.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Metric {
    /// Median latency (ns).
    Median,
    /// 95th percentile latency (ns).
    P95,
    /// 99th percentile latency (ns).
    P99,
    /// 99.9th percentile latency (ns).
    P999,
    /// 99.99th percentile latency (ns).
    P9999,
    /// Worst observed latency (ns).
    Max,
    /// Worst observed jitter, `max - min` (ns).
    MaxJitter,
    /// `p99 / median` — dimensionless tail shape.
    TailRatioP99,
    /// `p99.9 / median` — dimensionless tail shape.
    TailRatioP999,
}

impl Metric {
    /// Every metric the gate knows about, in report order.
    pub const ALL: [Metric; 9] = [
        Metric::Median,
        Metric::P95,
        Metric::P99,
        Metric::P999,
        Metric::P9999,
        Metric::Max,
        Metric::MaxJitter,
        Metric::TailRatioP99,
        Metric::TailRatioP999,
    ];

    /// Short label used in tables.
    pub fn label(self) -> &'static str {
        match self {
            Metric::Median => "median",
            Metric::P95 => "p95",
            Metric::P99 => "p99",
            Metric::P999 => "p99.9",
            Metric::P9999 => "p99.99",
            Metric::Max => "max",
            Metric::MaxJitter => "max_jitter",
            Metric::TailRatioP99 => "p99/median",
            Metric::TailRatioP999 => "p99.9/median",
        }
    }

    /// Whether this metric is a dimensionless ratio rather than a duration.
    pub fn is_ratio(self) -> bool {
        matches!(self, Metric::TailRatioP99 | Metric::TailRatioP999)
    }

    /// Render a value of this metric for a human.
    pub fn format(self, value: f64) -> String {
        if self.is_ratio() {
            format!("{:.2}x", value)
        } else {
            format_duration_ns(value)
        }
    }

    /// Minimum sample count for this metric to mean anything.
    ///
    /// A percentile `p` estimated from `n` samples is the
    /// `n·(1 - p/100)`-th worst observation. Below ~10 observations beyond the
    /// percentile it is a single draw from the tail, not an estimate of it:
    /// p99.99 of the harness's default 50,000 iterations is the **5th worst
    /// sample**, and comparing that between two runs compares two lottery
    /// tickets. Metrics below their minimum are reported
    /// [`Verdict::InsufficientSamples`] rather than silently compared.
    pub fn min_samples(self) -> usize {
        match self {
            Metric::Median => 100,
            Metric::P95 => 200,
            Metric::P99 | Metric::TailRatioP99 => 1_000,
            Metric::P999 | Metric::TailRatioP999 => 10_000,
            Metric::P9999 => 100_000,
            // `max` and `max_jitter` are single samples at any n; the floor
            // here only asserts the run was long enough to have seen anything.
            Metric::Max | Metric::MaxJitter => 1_000,
        }
    }

    /// Extract this metric from a baseline entry, if it is defined.
    pub fn value(self, entry: &BaselineEntry) -> Option<f64> {
        let ratio = |num: f64| {
            if entry.median > 0.0 {
                Some(num / entry.median)
            } else {
                None
            }
        };
        match self {
            Metric::Median => Some(entry.median),
            Metric::P95 => Some(entry.p95 as f64),
            Metric::P99 => Some(entry.p99 as f64),
            Metric::P999 => Some(entry.p999 as f64),
            Metric::P9999 => Some(entry.p9999 as f64),
            Metric::Max => Some(entry.max as f64),
            Metric::MaxJitter => Some(entry.max_jitter_ns as f64),
            Metric::TailRatioP99 => ratio(entry.p99 as f64),
            Metric::TailRatioP999 => ratio(entry.p999 as f64),
        }
    }

    /// For a tail-shape ratio, the absolute metric it is built from.
    ///
    /// A real improvement to the median that leaves the tail alone *raises*
    /// `p99/median` and would otherwise be reported as a tail regression. The
    /// gate suppresses that false positive by requiring the underlying absolute
    /// tail metric to have not improved — which leaves the true positive
    /// (something added time to the tail) fully intact, because that raises
    /// both.
    pub fn ratio_numerator(self) -> Option<Metric> {
        match self {
            Metric::TailRatioP99 => Some(Metric::P99),
            Metric::TailRatioP999 => Some(Metric::P999),
            _ => None,
        }
    }
}

/// Whether a metric's verdict can fail the job.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Enforcement {
    /// A regression on this metric fails CI.
    Blocking,
    /// A regression on this metric is reported but does not fail CI — unless it
    /// is gross, or (on trunk) sustained.
    ReportOnly,
}

/// Per-metric thresholds.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetricPolicy {
    /// Which statistic this governs.
    pub metric: Metric,
    /// Whether a plain regression here fails the job.
    pub enforcement: Enforcement,
    /// Relative band floor, as a fraction (0.10 = 10%). The band never gets
    /// *tighter* than this even if the baseline window looks noiseless.
    pub rel_threshold: f64,
    /// How many robust sigmas of the baseline window's own observed spread to
    /// allow before calling a regression.
    pub noise_sigmas: f64,
    /// Absolute band floor, in the metric's units. Guards against a percentage
    /// of a small number being below what the harness can resolve at all.
    pub abs_floor: f64,
    /// Multiple of the baseline center beyond which the excursion is blocking
    /// regardless of `enforcement` or of how little baseline history exists.
    pub gross_multiplier: f64,
}

/// The gate's configuration, with the reasoning for every number.
///
/// # Where the numbers come from
///
/// The only run-to-run noise this tree has actually measured is on the
/// **median**: repeated runs of identical code produced 63 ns / 105 ns and
/// 99 ns / 193 ns, i.e. between-run bias of `σ/µ ≈ 0.25–0.35` driven by P-state
/// history, core placement, code layout and neighbours. A single run sitting at
/// 1.95× the window center is ~3.2σ under that model, so a 3σ band would fire
/// on identical code roughly as often as not. `noise_sigmas = 4.0` on the
/// median puts the worst observed identical-code excursion inside the band with
/// margin, and — this is the point — **shrinks automatically** as the harness's
/// σ is driven down.
///
/// Everything past the median is *not* measured here, and the policy says so by
/// keeping it report-only rather than by inventing a σ for it.
///
/// # TODO: what would let the tail be gated directly
///
/// Promote `P99` (then `P999`) to [`Enforcement::Blocking`] once **all** of:
///
/// - the runner records `measured_freq_mhz` and pins a P-state, so the
///   multiplicative frequency term stops moving between runs;
/// - the job runs the benchmark `R > 1` times and the gate is fed all `R`
///   (already supported: pass several current reports);
/// - a baseline window of ≥ 20 same-CPU-model runs shows the *observed* σ/µ of
///   `p99` is under ~0.15, which the printed band width reports on every run.
///
/// Until then, promoting them would be a gate that lies. `max`, `p99.99` and
/// `max_jitter` need a pinned, isolated, non-shared runner before they can be
/// anything but advisory, and that is not a threshold problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegressionPolicy {
    /// Per-metric thresholds.
    pub metrics: Vec<MetricPolicy>,
    /// Baseline runs required before the window's spread is trusted. Below
    /// this, every band falls back to `no_noise_fallback_rel` and nothing but a
    /// gross regression can block.
    pub min_baseline_runs: usize,
    /// Band used when the window is too short to estimate spread. `1.0` (a 2x
    /// band) is the observed identical-code median swing, not a guess.
    pub no_noise_fallback_rel: f64,
    /// The smallest change the project actually wants to catch. Whenever the
    /// effective band is wider than this, a passing result is reported
    /// [`Verdict::Inconclusive`] instead of [`Verdict::Ok`]: the run did not
    /// show a regression, and also could not have.
    pub resolvable_rel: f64,
    /// Refuse to compare across CPU models. GitHub-hosted runners are not one
    /// machine; a baseline gathered on a different CPU is a different
    /// experiment. When the current model is absent from the window the whole
    /// comparison goes advisory (gross regressions still block).
    pub require_same_cpu_model: bool,
    /// Consecutive above-band runs that escalate a report-only metric to
    /// blocking. Only applied on trunk runs (see
    /// [`ComparisonInput::is_trunk_run`]): the point is to catch sustained tail
    /// creep that no single run can distinguish from noise, without making
    /// pull-request CI flaky. Under independent noise with per-run false-alarm
    /// rate `p`, this fires at `p^n`.
    pub consecutive_to_block: usize,
    /// A benchmark that the baseline knows about but the run did not produce is
    /// an ungated critical path, not a pass. Same rule the sibling
    /// `check_regression.py` settled on after the same bug.
    pub fail_on_missing_benchmark: bool,
}

impl Default for RegressionPolicy {
    fn default() -> Self {
        Self {
            metrics: vec![
                // The one statistic whose run-to-run spread this tree has
                // actually measured. 10% is the number humans care about and is
                // the floor the band converges to; 25 ns is roughly one
                // cross-core cache-line transfer and is the smallest delta the
                // harness has any business claiming to see (the research
                // concluded it cannot resolve 5-10 ns at any feasible run
                // count).
                MetricPolicy {
                    metric: Metric::Median,
                    enforcement: Enforcement::Blocking,
                    rel_threshold: 0.10,
                    noise_sigmas: 4.0,
                    abs_floor: 25.0,
                    gross_multiplier: 3.0,
                },
                MetricPolicy {
                    metric: Metric::P95,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 0.15,
                    noise_sigmas: 4.0,
                    abs_floor: 50.0,
                    gross_multiplier: 4.0,
                },
                // Report-only: spread unmeasured. Gross at 4x, because no
                // plausible combination of P-state (base-vs-turbo is under ~2x)
                // and core placement (SMT sibling ~20 ns vs cross-socket
                // ~200 ns of coherence cost) reaches 4x on the same runner
                // class. The failure this gate exists to catch -- 50 µs added
                // to a ~400 ns p99 -- is 125x, caught with 30x of margin.
                MetricPolicy {
                    metric: Metric::P99,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 0.20,
                    noise_sigmas: 4.0,
                    abs_floor: 100.0,
                    gross_multiplier: 4.0,
                },
                MetricPolicy {
                    metric: Metric::P999,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 0.25,
                    noise_sigmas: 5.0,
                    abs_floor: 250.0,
                    gross_multiplier: 6.0,
                },
                // Order statistics thin enough that one host preemption moves
                // them. Advisory at any threshold on a shared runner.
                MetricPolicy {
                    metric: Metric::P9999,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 0.50,
                    noise_sigmas: 6.0,
                    abs_floor: 1_000.0,
                    gross_multiplier: 10.0,
                },
                MetricPolicy {
                    metric: Metric::Max,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 1.00,
                    noise_sigmas: 6.0,
                    abs_floor: 5_000.0,
                    gross_multiplier: 10.0,
                },
                MetricPolicy {
                    metric: Metric::MaxJitter,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 1.00,
                    noise_sigmas: 6.0,
                    abs_floor: 5_000.0,
                    gross_multiplier: 10.0,
                },
                // The blocking tail metric. Dimensionless, so the dominant
                // characterized noise term -- every sample being
                // `core_cycles / f_core` with an unrecorded, drifting `f_core`
                // -- cancels between numerator and denominator of the same run.
                // Blocking is only sound because `Median` is gated separately:
                // on its own this ratio could be "improved" by making the
                // median worse.
                MetricPolicy {
                    metric: Metric::TailRatioP99,
                    enforcement: Enforcement::Blocking,
                    rel_threshold: 0.25,
                    noise_sigmas: 4.0,
                    abs_floor: 0.5,
                    gross_multiplier: 4.0,
                },
                // Same construction one percentile further out, where the
                // numerator is the 100th-worst of 100k rather than the
                // 1000th-worst. Kept report-only until a window shows its
                // spread; the gross multiplier and the trunk streak rule still
                // apply.
                MetricPolicy {
                    metric: Metric::TailRatioP999,
                    enforcement: Enforcement::ReportOnly,
                    rel_threshold: 0.35,
                    noise_sigmas: 5.0,
                    abs_floor: 1.0,
                    gross_multiplier: 6.0,
                },
            ],
            min_baseline_runs: 5,
            no_noise_fallback_rel: 1.0,
            resolvable_rel: 0.10,
            require_same_cpu_model: true,
            consecutive_to_block: 3,
            fail_on_missing_benchmark: true,
        }
    }
}

impl RegressionPolicy {
    /// Look up the policy for a metric.
    pub fn for_metric(&self, metric: Metric) -> Option<&MetricPolicy> {
        self.metrics.iter().find(|m| m.metric == metric)
    }
}

// =============================================================================
// Rolling baseline
// =============================================================================

/// One benchmark's summary within one run.
///
/// Everything the gate needs and nothing it does not — in particular not
/// `raw_latencies_ns`, so a 20-run window stays a few kilobytes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaselineEntry {
    /// Benchmark name.
    pub name: String,
    /// Message size tested (bytes).
    pub message_size: usize,
    /// Number of samples the statistics were computed from.
    pub count: usize,
    /// Median latency (ns).
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub median: f64,
    /// 95th percentile (ns).
    pub p95: u64,
    /// 99th percentile (ns).
    pub p99: u64,
    /// 99.9th percentile (ns).
    pub p999: u64,
    /// 99.99th percentile (ns).
    pub p9999: u64,
    /// Worst observed latency (ns).
    pub max: u64,
    /// Worst observed jitter, `max - min` (ns).
    pub max_jitter_ns: u64,
    /// Coefficient of variation as the producing binary computed it. Carried
    /// for the report only: two binaries in this tree publish a field named
    /// `cv` computed from different sample populations (one filtered, one not),
    /// so it is never gated.
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub cv: f64,
}

impl BaselineEntry {
    /// Summarize a single benchmark result.
    pub fn from_result(result: &BenchmarkResult) -> Self {
        Self {
            name: result.name.clone(),
            message_size: result.message_size,
            count: result.statistics.count,
            median: result.statistics.median,
            p95: result.statistics.p95,
            p99: result.statistics.p99,
            p999: result.statistics.p999,
            p9999: result.statistics.p9999,
            max: result.statistics.max,
            max_jitter_ns: result.determinism.max_jitter_ns,
            cv: result.determinism.cv,
        }
    }

    /// Identity used to line results up across runs.
    pub fn key(&self) -> String {
        format!("{}@{}B", self.name, self.message_size)
    }
}

/// One recorded run of the whole suite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaselineRun {
    /// Commit the run measured.
    pub git_commit: Option<String>,
    /// Branch the run measured.
    pub git_branch: Option<String>,
    /// CI run id, for chasing the logs.
    pub ci_run_id: Option<String>,
    /// When the run was recorded.
    pub recorded_at: String,
    /// CPU model the run executed on. Hosted CI runners are not one machine.
    pub cpu_model: String,
    /// Governor in force, when the platform reports one. A window mixing
    /// `powersave` and `performance` entries is a window whose spread is
    /// measuring the governor.
    pub cpu_governor: Option<String>,
    /// Per-benchmark summaries.
    pub entries: Vec<BaselineEntry>,
}

impl BaselineRun {
    /// Summarize a report into a recordable run.
    pub fn from_report(report: &BenchmarkReport) -> Self {
        Self {
            git_commit: report.git_commit.clone(),
            git_branch: report.git_branch.clone(),
            ci_run_id: report.ci_run_id.clone(),
            recorded_at: report.generated_at.clone(),
            cpu_model: report.platform.cpu.model.clone(),
            cpu_governor: report.platform.cpu_governor.clone(),
            entries: report.summarize(),
        }
    }

    /// Summarize several repetitions of the same code into **one** recordable
    /// run, field by field, by median.
    ///
    /// One run, not `R` runs. Repetitions inside a single CI job share a
    /// runner, a P-state history and a set of neighbours, so recording them
    /// separately would fill the window with correlated samples, understate the
    /// between-job spread, and make every band too tight — a gate that fires on
    /// noise. The window must hold one point per job.
    pub fn from_reports(reports: &[&BenchmarkReport]) -> Option<Self> {
        let first = reports.first()?;
        let mut grouped: BTreeMap<String, Vec<BaselineEntry>> = BTreeMap::new();
        for report in reports {
            for entry in report.summarize() {
                grouped.entry(entry.key()).or_default().push(entry);
            }
        }

        let med_f = |mut v: Vec<f64>| robust_center(&mut v).unwrap_or(0.0);
        let med_u = |v: Vec<u64>| {
            let mut v: Vec<f64> = v.into_iter().map(|x| x as f64).collect();
            robust_center(&mut v).unwrap_or(0.0).round() as u64
        };

        let entries = grouped
            .into_values()
            .map(|reps| BaselineEntry {
                name: reps[0].name.clone(),
                message_size: reps[0].message_size,
                count: reps.iter().map(|e| e.count).min().unwrap_or(0),
                median: med_f(reps.iter().map(|e| e.median).collect()),
                p95: med_u(reps.iter().map(|e| e.p95).collect()),
                p99: med_u(reps.iter().map(|e| e.p99).collect()),
                p999: med_u(reps.iter().map(|e| e.p999).collect()),
                p9999: med_u(reps.iter().map(|e| e.p9999).collect()),
                max: med_u(reps.iter().map(|e| e.max).collect()),
                max_jitter_ns: med_u(reps.iter().map(|e| e.max_jitter_ns).collect()),
                cv: med_f(reps.iter().map(|e| e.cv).collect()),
            })
            .collect();

        Some(Self {
            git_commit: first.git_commit.clone(),
            git_branch: first.git_branch.clone(),
            ci_run_id: first.ci_run_id.clone(),
            recorded_at: first.generated_at.clone(),
            cpu_model: first.platform.cpu.model.clone(),
            cpu_governor: first.platform.cpu_governor.clone(),
            entries,
        })
    }

    /// Find one benchmark's summary in this run.
    pub fn entry(&self, key: &str) -> Option<&BaselineEntry> {
        self.entries.iter().find(|e| e.key() == key)
    }
}

/// A rolling window of recent trunk runs.
///
/// `runs[0]` is the most recent. A window rather than a single run because the
/// single-run comparison this replaced could not distinguish a 10% regression
/// from the harness's own 60-95% run-to-run swing on identical code, and no
/// choice of threshold fixes that — only more runs do.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaselineHistory {
    /// Format version for forward compatibility.
    pub version: String,
    /// How many runs to keep.
    pub max_runs: usize,
    /// Recent runs, newest first.
    pub runs: Vec<BaselineRun>,
}

impl BaselineHistory {
    /// Create an empty history with the given window length.
    pub fn new(max_runs: usize) -> Self {
        Self {
            version: Self::FORMAT.to_string(),
            max_runs: max_runs.max(1),
            runs: Vec::new(),
        }
    }

    /// Format version this build writes and reads.
    pub const FORMAT: &'static str = "2.0.0";

    /// Load a history file.
    ///
    /// A missing, unreadable, unparseable or wrong-version file yields an
    /// **empty** history with a warning on stderr, not an error. The window
    /// lives in a CI cache: making a damaged or outdated cache entry a hard
    /// failure would leave the pipeline red until someone found the cache purge
    /// button, and a gate people cannot get past is a gate people delete. An
    /// empty window degrades to "only gross regressions can fail" and refills
    /// itself over the next few trunk runs, which is visible in every report —
    /// it says how many runs it has.
    pub fn load<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let path = path.as_ref();
        if !path.exists() {
            return Ok(Self::new(20));
        }
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) => {
                eprintln!(
                    "warning: baseline history {} could not be read ({e}); starting an empty \
                     window. Only gross regressions can fail until it refills.",
                    path.display()
                );
                return Ok(Self::new(20));
            }
        };
        match serde_json::from_str::<Self>(&text) {
            Ok(history) if history.version == Self::FORMAT => Ok(history),
            Ok(history) => {
                eprintln!(
                    "warning: baseline history {} is format {} but this build writes {}; \
                     starting an empty window.",
                    path.display(),
                    history.version,
                    Self::FORMAT
                );
                Ok(Self::new(20))
            }
            Err(e) => {
                eprintln!(
                    "warning: baseline history {} did not parse ({e}); starting an empty \
                     window. Only gross regressions can fail until it refills.",
                    path.display()
                );
                Ok(Self::new(20))
            }
        }
    }

    /// Write the history file.
    pub fn save<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let file = File::create(path)?;
        let writer = BufWriter::new(file);
        serde_json::to_writer_pretty(writer, self)?;
        Ok(())
    }

    /// Record a run, evicting the oldest once the window is full.
    pub fn push(&mut self, report: &BenchmarkReport) {
        self.push_run(BaselineRun::from_report(report));
    }

    /// Record several repetitions of one job as a single window point.
    pub fn push_reports(&mut self, reports: &[&BenchmarkReport]) {
        if let Some(run) = BaselineRun::from_reports(reports) {
            self.push_run(run);
        }
    }

    /// Record an already-summarized run.
    pub fn push_run(&mut self, run: BaselineRun) {
        self.runs.insert(0, run);
        self.runs.truncate(self.max_runs);
    }

    /// Runs usable as a baseline for a run on `cpu_model`.
    fn applicable<'a>(
        &'a self,
        cpu_model: &str,
        policy: &RegressionPolicy,
    ) -> Vec<&'a BaselineRun> {
        if !policy.require_same_cpu_model || cpu_model.is_empty() {
            return self.runs.iter().collect();
        }
        self.runs
            .iter()
            .filter(|r| r.cpu_model == cpu_model)
            .collect()
    }

    /// Compare one or more reports of the current code against this window.
    ///
    /// Passing several reports is the supported way to run the benchmark `R`
    /// times in one job: the current arm becomes the **median across
    /// repetitions**, which is quieter than any single historical run. Since
    /// the band is derived from the spread of single historical runs, that
    /// makes the comparison conservative in the safe direction — it is harder,
    /// not easier, to fire falsely.
    pub fn compare(
        &self,
        current: &[&BenchmarkReport],
        policy: &RegressionPolicy,
    ) -> ComparisonReport {
        self.evaluate(&ComparisonInput {
            current,
            policy,
            is_trunk_run: false,
        })
    }

    /// Compare with full control over gate context.
    pub fn evaluate(&self, input: &ComparisonInput<'_>) -> ComparisonReport {
        let policy = input.policy;
        let first = input.current.first();
        let cpu_model = first
            .map(|r| r.platform.cpu.model.clone())
            .unwrap_or_default();
        let cpu_governor = first.and_then(|r| r.platform.cpu_governor.clone());

        // Group the repetitions of the current arm by benchmark.
        let mut current_entries: BTreeMap<String, Vec<BaselineEntry>> = BTreeMap::new();
        for report in input.current {
            for entry in report.summarize() {
                current_entries.entry(entry.key()).or_default().push(entry);
            }
        }

        let mut applicable = self.applicable(&cpu_model, policy);
        let dropped_for_cpu = self.runs.len() - applicable.len();
        let cpu_model_mismatch = policy.require_same_cpu_model
            && !self.runs.is_empty()
            && applicable.is_empty()
            && !cpu_model.is_empty();
        if cpu_model_mismatch {
            // Still compare — a report with no numbers in it teaches nobody
            // anything — but every verdict below is advisory, and only a gross
            // regression can fail the job.
            applicable = self.runs.iter().collect();
        }

        let mut comparisons = Vec::new();
        for (key, reps) in &current_entries {
            comparisons.push(self.compare_one(
                key,
                reps,
                &applicable,
                policy,
                input.is_trunk_run,
                cpu_model_mismatch,
            ));
        }

        // A benchmark the baseline knows about but this run did not produce is
        // a hole in the gate: renamed, removed, or crashed, and silently no
        // longer checked.
        let mut baselined: BTreeSet<String> = BTreeSet::new();
        for run in applicable.iter().take(1) {
            for entry in &run.entries {
                baselined.insert(entry.key());
            }
        }
        let missing_from_run: Vec<String> = baselined
            .iter()
            .filter(|k| !current_entries.contains_key(*k))
            .cloned()
            .collect();

        ComparisonReport {
            current_commit: first.and_then(|r| r.git_commit.clone()),
            current_branch: first.and_then(|r| r.git_branch.clone()),
            baseline_commit: applicable.first().and_then(|r| r.git_commit.clone()),
            repetitions: input.current.len(),
            baseline_runs: applicable.len(),
            baseline_runs_dropped_for_cpu: dropped_for_cpu,
            cpu_model,
            cpu_governor,
            cpu_model_mismatch,
            is_trunk_run: input.is_trunk_run,
            missing_from_run,
            comparisons,
            policy: policy.clone(),
        }
    }

    fn compare_one(
        &self,
        key: &str,
        reps: &[BaselineEntry],
        applicable: &[&BaselineRun],
        policy: &RegressionPolicy,
        is_trunk_run: bool,
        cpu_model_mismatch: bool,
    ) -> ResultComparison {
        let sample_count = reps.iter().map(|e| e.count).min().unwrap_or(0);
        let mut metrics = Vec::new();

        for metric in Metric::ALL {
            let Some(mp) = policy.for_metric(metric) else {
                continue;
            };

            let mut rep_values: Vec<f64> = reps.iter().filter_map(|e| metric.value(e)).collect();
            let Some(current) = robust_center(&mut rep_values) else {
                continue;
            };

            let hist: Vec<f64> = applicable
                .iter()
                .filter_map(|run| run.entry(key))
                .filter(|e| e.count >= metric.min_samples())
                .filter_map(|e| metric.value(e))
                .collect();

            let mut cmp = MetricComparison {
                metric,
                current,
                baseline_center: 0.0,
                baseline_spread: 0.0,
                baseline_min: 0.0,
                baseline_max: 0.0,
                baseline_runs: hist.len(),
                upper_band: f64::INFINITY,
                band_source: BandSource::NoBaseline,
                change_percent: 0.0,
                streak: 0,
                verdict: Verdict::NoBaseline,
                blocking: false,
                note: None,
            };

            if sample_count < metric.min_samples() {
                cmp.verdict = Verdict::InsufficientSamples;
                cmp.note = Some(format!(
                    "needs >= {} samples to estimate; run had {}",
                    metric.min_samples(),
                    sample_count
                ));
                metrics.push(cmp);
                continue;
            }

            let mut hist_sorted = hist.clone();
            let Some(center) = robust_center(&mut hist_sorted) else {
                cmp.note = Some("no baseline run recorded this benchmark".to_string());
                metrics.push(cmp);
                continue;
            };
            if center <= 0.0 {
                cmp.note = Some("baseline center is zero; nothing to compare".to_string());
                metrics.push(cmp);
                continue;
            }

            cmp.baseline_center = center;
            cmp.baseline_spread = robust_sigma(&hist_sorted, center);
            cmp.baseline_min = hist_sorted.first().copied().unwrap_or(center);
            cmp.baseline_max = hist_sorted.last().copied().unwrap_or(center);
            cmp.change_percent = (current - center) / center * 100.0;

            let (band, source) = band_for(center, cmp.baseline_spread, hist.len(), mp, policy);
            cmp.upper_band = band;
            cmp.band_source = source;

            let gross = current > center * mp.gross_multiplier;
            let over = current > band;
            let band_rel = (band - center) / center;

            if gross {
                cmp.verdict = Verdict::GrossRegression;
                cmp.blocking = true;
                cmp.note = Some(format!(
                    "{:.1}x the baseline center; past {:.1}x no runner-side effect explains it",
                    current / center,
                    mp.gross_multiplier
                ));
            } else if over {
                cmp.verdict = Verdict::Regressed;
                cmp.blocking = mp.enforcement == Enforcement::Blocking
                    && source != BandSource::Fallback
                    && !cpu_model_mismatch;
                if is_trunk_run && policy.consecutive_to_block > 0 {
                    cmp.streak = 1 + self.above_band_streak(key, metric, applicable, mp, policy);
                    if !cmp.blocking && cmp.streak >= policy.consecutive_to_block {
                        cmp.blocking = !cpu_model_mismatch;
                        cmp.note = Some(format!(
                            "above band on {} consecutive trunk runs; sustained drift, not a draw",
                            cmp.streak
                        ));
                    }
                }
            } else if current < center * (1.0 - mp.rel_threshold)
                && source != BandSource::Fallback
                && current < center - cmp.baseline_spread
            {
                cmp.verdict = Verdict::Improved;
            } else if source == BandSource::Fallback || band_rel > policy.resolvable_rel {
                cmp.verdict = Verdict::Inconclusive;
                cmp.note = Some(format!(
                    "band is +{:.0}%, wider than the +{:.0}% this gate wants to detect",
                    band_rel * 100.0,
                    policy.resolvable_rel * 100.0
                ));
            } else {
                cmp.verdict = Verdict::Ok;
            }

            metrics.push(cmp);
        }

        // A genuine median win leaves the tail alone and so *raises*
        // p99/median. That is the one false positive the tail-shape metrics
        // have, and it is suppressed narrowly: only when the median actually
        // improved by more than the gate claims to resolve, AND the absolute
        // tail percentile the ratio is built from did not itself regress. A
        // change that adds time to the tail raises both, so the true positive
        // is untouched.
        let median_change = metrics
            .iter()
            .find(|m| m.metric == Metric::Median)
            .map(|m| m.change_percent)
            .unwrap_or(0.0);
        let numerator_within_band: BTreeSet<Metric> = metrics
            .iter()
            .filter(|m| !m.verdict.is_regression())
            .map(|m| m.metric)
            .collect();
        let median_improved = median_change < -policy.resolvable_rel * 100.0;
        for cmp in &mut metrics {
            let Some(num) = cmp.metric.ratio_numerator() else {
                continue;
            };
            if cmp.verdict.is_regression()
                && median_improved
                && numerator_within_band.contains(&num)
            {
                cmp.verdict = Verdict::Inconclusive;
                cmp.blocking = false;
                cmp.note = Some(format!(
                    "tail shape widened, but absolute {} stayed within its band while the \
                     median improved {:.1}% — this is the median getting better, not the \
                     tail getting worse",
                    num.label(),
                    -median_change
                ));
            }
        }

        let regression = metrics.iter().any(|m| m.blocking);
        ResultComparison {
            name: reps[0].name.clone(),
            message_size: reps[0].message_size,
            sample_count,
            baseline_runs: metrics.iter().map(|m| m.baseline_runs).max().unwrap_or(0),
            current_median_ns: reps[0].median,
            baseline_median_ns: metrics
                .iter()
                .find(|m| m.metric == Metric::Median)
                .map(|m| m.baseline_center)
                .unwrap_or(0.0),
            change_percent: metrics
                .iter()
                .find(|m| m.metric == Metric::Median)
                .map(|m| m.change_percent)
                .unwrap_or(0.0),
            current_p99_ns: reps[0].p99,
            baseline_p99_ns: metrics
                .iter()
                .find(|m| m.metric == Metric::P99)
                .map(|m| m.baseline_center.round() as u64)
                .unwrap_or(0),
            p99_change_percent: metrics
                .iter()
                .find(|m| m.metric == Metric::P99)
                .map(|m| m.change_percent)
                .unwrap_or(0.0),
            metrics,
            regression,
            advisory_only: cpu_model_mismatch,
        }
    }

    /// How many of the most recent baseline runs were themselves above the band
    /// derived from the runs older than them.
    ///
    /// Derived from the window rather than stored as a counter, so it cannot go
    /// stale or be wrong about history that was later evicted.
    fn above_band_streak(
        &self,
        key: &str,
        metric: Metric,
        applicable: &[&BaselineRun],
        mp: &MetricPolicy,
        policy: &RegressionPolicy,
    ) -> usize {
        let mut streak = 0usize;
        for i in 0..applicable.len() {
            let Some(entry) = applicable[i].entry(key) else {
                break;
            };
            if entry.count < metric.min_samples() {
                break;
            }
            let Some(value) = metric.value(entry) else {
                break;
            };
            let prior: Vec<f64> = applicable[i + 1..]
                .iter()
                .filter_map(|run| run.entry(key))
                .filter(|e| e.count >= metric.min_samples())
                .filter_map(|e| metric.value(e))
                .collect();
            if prior.len() < policy.min_baseline_runs {
                break;
            }
            let mut prior_sorted = prior.clone();
            let Some(center) = robust_center(&mut prior_sorted) else {
                break;
            };
            if center <= 0.0 {
                break;
            }
            let sigma = robust_sigma(&prior_sorted, center);
            let (band, _) = band_for(center, sigma, prior.len(), mp, policy);
            if value > band {
                streak += 1;
            } else {
                break;
            }
        }
        streak
    }
}

/// Everything the gate needs to know beyond the numbers themselves.
pub struct ComparisonInput<'a> {
    /// One report per repetition of the current code.
    pub current: &'a [&'a BenchmarkReport],
    /// Thresholds and enforcement.
    pub policy: &'a RegressionPolicy,
    /// True for a push to the trunk branch. Enables the consecutive-runs
    /// escalation, which must not apply to pull requests: a PR author is not
    /// responsible for drift that landed on trunk before their branch.
    pub is_trunk_run: bool,
}

/// Which term set the band, so the report can say why the threshold is what it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BandSource {
    /// The relative threshold dominated: the window is tight.
    Relative,
    /// The window's own observed spread dominated.
    ObservedNoise,
    /// The absolute floor dominated: the numbers are too small to gate by percentage.
    AbsoluteFloor,
    /// Too few baseline runs to estimate spread at all.
    Fallback,
    /// No baseline for this benchmark.
    NoBaseline,
}

impl BandSource {
    /// One-line explanation for the report.
    pub fn describe(self) -> &'static str {
        match self {
            BandSource::Relative => "relative threshold (window is tight enough)",
            BandSource::ObservedNoise => "baseline window's own measured spread",
            BandSource::AbsoluteFloor => "absolute floor (percentage would be below resolution)",
            BandSource::Fallback => "fallback: too few baseline runs to estimate spread",
            BandSource::NoBaseline => "no baseline",
        }
    }
}

fn band_for(
    center: f64,
    sigma: f64,
    runs: usize,
    mp: &MetricPolicy,
    policy: &RegressionPolicy,
) -> (f64, BandSource) {
    if runs < policy.min_baseline_runs {
        return (
            center * (1.0 + policy.no_noise_fallback_rel).max(1.0 + mp.rel_threshold),
            BandSource::Fallback,
        );
    }
    let by_rel = center * (1.0 + mp.rel_threshold);
    let by_noise = center + mp.noise_sigmas * sigma;
    let by_floor = center + mp.abs_floor;

    let mut best = (by_rel, BandSource::Relative);
    if by_noise > best.0 {
        best = (by_noise, BandSource::ObservedNoise);
    }
    if by_floor > best.0 {
        best = (by_floor, BandSource::AbsoluteFloor);
    }
    best
}

/// Median of a slice, sorting it in place. `None` for an empty slice.
fn robust_center(values: &mut [f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = values.len();
    Some(if n.is_multiple_of(2) {
        (values[n / 2 - 1] + values[n / 2]) / 2.0
    } else {
        values[n / 2]
    })
}

/// Robust standard deviation: `1.4826 * median(|x - center|)`.
///
/// The median absolute deviation rather than the standard deviation because the
/// window is short and a single pathological run must widen the band a little,
/// not blow it open.
fn robust_sigma(values: &[f64], center: f64) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let mut dev: Vec<f64> = values.iter().map(|v| (v - center).abs()).collect();
    robust_center(&mut dev).unwrap_or(0.0) * 1.4826
}

// =============================================================================
// Verdicts and comparison results
// =============================================================================

/// What the gate concluded about one metric.
///
/// The important addition over a boolean is [`Verdict::Inconclusive`]. A gate
/// that reports only pass/fail turns "we could not have seen it" into "it is
/// not there", which is how a suite ends up printing "All benchmarks within
/// acceptable range" over a measurement that could not have detected anything
/// smaller than a doubling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    /// Inside the band, and the band is tight enough to have caught the change
    /// the project cares about.
    Ok,
    /// Inside the band, but the band is wider than the change the project cares
    /// about. Nothing was detected and nothing could have been.
    Inconclusive,
    /// Below the band by more than noise.
    Improved,
    /// Above the band.
    Regressed,
    /// So far above the baseline that no runner-side effect explains it.
    GrossRegression,
    /// The run was too short for this percentile to be an estimate rather than
    /// a single draw from the tail.
    InsufficientSamples,
    /// No comparable baseline for this benchmark.
    NoBaseline,
}

impl Verdict {
    /// Fixed-width label for tables.
    pub fn label(self) -> &'static str {
        match self {
            Verdict::Ok => "OK",
            Verdict::Inconclusive => "INCONCLUSIVE",
            Verdict::Improved => "IMPROVED",
            Verdict::Regressed => "REGRESSED",
            Verdict::GrossRegression => "GROSS REGRESSION",
            Verdict::InsufficientSamples => "N/A (samples)",
            Verdict::NoBaseline => "N/A (no baseline)",
        }
    }

    /// Whether this verdict describes a regression, blocking or not.
    pub fn is_regression(self) -> bool {
        matches!(self, Verdict::Regressed | Verdict::GrossRegression)
    }
}

/// The gate's conclusion about one metric of one benchmark.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetricComparison {
    /// Which statistic.
    pub metric: Metric,
    /// Current value (median across repetitions).
    pub current: f64,
    /// Robust center of the baseline window.
    pub baseline_center: f64,
    /// Robust spread (1.4826·MAD) of the baseline window.
    pub baseline_spread: f64,
    /// Smallest value in the baseline window.
    pub baseline_min: f64,
    /// Largest value in the baseline window.
    pub baseline_max: f64,
    /// How many baseline runs contributed.
    pub baseline_runs: usize,
    /// The threshold. Above this is a regression.
    pub upper_band: f64,
    /// Which term set the threshold.
    pub band_source: BandSource,
    /// Percent change against the baseline center (positive = slower).
    pub change_percent: f64,
    /// Consecutive runs above band, current run included. Only computed on
    /// trunk runs.
    pub streak: usize,
    /// What the gate concluded.
    pub verdict: Verdict,
    /// Whether this conclusion fails the job.
    pub blocking: bool,
    /// Human explanation, when there is something to explain.
    pub note: Option<String>,
}

impl MetricComparison {
    /// Band width as a fraction of the baseline center.
    pub fn band_rel(&self) -> f64 {
        if self.baseline_center > 0.0 && self.upper_band.is_finite() {
            (self.upper_band - self.baseline_center) / self.baseline_center
        } else {
            f64::INFINITY
        }
    }
}

/// Comparison of a single benchmark against the baseline window.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResultComparison {
    /// Benchmark name.
    pub name: String,
    /// Message size tested.
    pub message_size: usize,
    /// Samples behind the current statistics (minimum across repetitions).
    pub sample_count: usize,
    /// Baseline runs available for this benchmark.
    pub baseline_runs: usize,
    /// Current median latency (ns), first repetition.
    pub current_median_ns: f64,
    /// Baseline window's median latency (ns).
    pub baseline_median_ns: f64,
    /// Percent change in median (positive = slower).
    pub change_percent: f64,
    /// Current p99 latency (ns), first repetition.
    pub current_p99_ns: u64,
    /// Baseline window's p99 latency (ns).
    pub baseline_p99_ns: u64,
    /// Percent change in p99.
    pub p99_change_percent: f64,
    /// Per-metric conclusions.
    pub metrics: Vec<MetricComparison>,
    /// Whether anything here fails the job.
    pub regression: bool,
    /// Whether every conclusion here is advisory because the baseline was
    /// gathered on different hardware.
    pub advisory_only: bool,
}

impl ResultComparison {
    /// Look up one metric's conclusion.
    pub fn metric(&self, metric: Metric) -> Option<&MetricComparison> {
        self.metrics.iter().find(|m| m.metric == metric)
    }

    /// Metrics that fail the job.
    pub fn blocking(&self) -> Vec<&MetricComparison> {
        self.metrics.iter().filter(|m| m.blocking).collect()
    }

    /// Metrics that regressed but do not fail the job.
    pub fn advisory_regressions(&self) -> Vec<&MetricComparison> {
        self.metrics
            .iter()
            .filter(|m| m.verdict.is_regression() && !m.blocking)
            .collect()
    }

    /// Identity used in reports.
    pub fn key(&self) -> String {
        format!("{}@{}B", self.name, self.message_size)
    }
}

/// The gate's full conclusion.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComparisonReport {
    /// Commit under test.
    pub current_commit: Option<String>,
    /// Branch under test.
    pub current_branch: Option<String>,
    /// Newest commit in the baseline window.
    pub baseline_commit: Option<String>,
    /// How many times the current code was benchmarked.
    pub repetitions: usize,
    /// How many baseline runs were usable.
    pub baseline_runs: usize,
    /// Baseline runs discarded for being on different hardware.
    pub baseline_runs_dropped_for_cpu: usize,
    /// CPU model the current run executed on.
    pub cpu_model: String,
    /// Governor in force for the current run, if reported.
    pub cpu_governor: Option<String>,
    /// True when no baseline run matched the current CPU model.
    pub cpu_model_mismatch: bool,
    /// Whether the consecutive-runs escalation was in play.
    pub is_trunk_run: bool,
    /// Benchmarks the baseline knows about that this run did not produce.
    pub missing_from_run: Vec<String>,
    /// Per-benchmark conclusions.
    pub comparisons: Vec<ResultComparison>,
    /// The policy that produced these conclusions.
    pub policy: RegressionPolicy,
}

impl ComparisonReport {
    /// Whether anything regressed at all, blocking or advisory.
    pub fn has_regressions(&self) -> bool {
        self.comparisons
            .iter()
            .any(|c| c.metrics.iter().any(|m| m.verdict.is_regression()))
    }

    /// Whether the job should fail.
    pub fn has_blocking_regressions(&self) -> bool {
        self.comparisons.iter().any(|c| c.regression) || self.missing_is_blocking()
    }

    fn missing_is_blocking(&self) -> bool {
        self.policy.fail_on_missing_benchmark && !self.missing_from_run.is_empty()
    }

    /// Benchmarks with at least one regressed metric.
    pub fn regressions(&self) -> Vec<&ResultComparison> {
        self.comparisons
            .iter()
            .filter(|c| c.metrics.iter().any(|m| m.verdict.is_regression()))
            .collect()
    }

    /// Process exit code for the gate: 0 pass, 1 fail.
    pub fn exit_code(&self) -> i32 {
        if self.has_blocking_regressions() {
            1
        } else {
            0
        }
    }

    /// Metrics that fail the job, paired with their benchmark.
    pub fn blocking_findings(&self) -> Vec<(&ResultComparison, &MetricComparison)> {
        self.comparisons
            .iter()
            .flat_map(|c| c.blocking().into_iter().map(move |m| (c, m)))
            .collect()
    }

    /// True when nothing failed but the measurement could not have detected the
    /// change the project cares about either — no benchmark produced a median
    /// verdict the gate is willing to stand behind.
    pub fn is_inconclusive(&self) -> bool {
        !self.has_blocking_regressions()
            && !self.comparisons.iter().any(|c| {
                matches!(
                    c.metric(Metric::Median),
                    Some(m) if matches!(m.verdict, Verdict::Ok | Verdict::Improved)
                )
            })
    }

    /// Print the full report to stdout.
    ///
    /// This is what a contributor reads when the job goes red, so it prints the
    /// whole distribution on both sides, the threshold, **and where the
    /// threshold came from** — a gate that fails without saying what number it
    /// compared, against what, and why that number, gets switched off.
    pub fn print_summary(&self) {
        println!("\n=== HORUS benchmark gate ===\n");
        println!(
            "current   : {} ({})",
            self.current_commit.as_deref().unwrap_or("unknown"),
            self.current_branch.as_deref().unwrap_or("unknown branch"),
        );
        println!(
            "baseline  : {} — rolling window of {} run(s){}",
            self.baseline_commit.as_deref().unwrap_or("none"),
            self.baseline_runs,
            if self.baseline_runs_dropped_for_cpu > 0 {
                format!(
                    " ({} dropped: different CPU model)",
                    self.baseline_runs_dropped_for_cpu
                )
            } else {
                String::new()
            }
        );
        println!(
            "runner    : {}{}",
            if self.cpu_model.is_empty() {
                "unknown CPU"
            } else {
                &self.cpu_model
            },
            self.cpu_governor
                .as_deref()
                .map(|g| format!(", governor={}", g))
                .unwrap_or_default()
        );
        println!(
            "current arm: median of {} repetition(s){}",
            self.repetitions,
            if self.is_trunk_run {
                format!(
                    ", trunk run (report-only metrics escalate after {} consecutive)",
                    self.policy.consecutive_to_block
                )
            } else {
                String::new()
            }
        );

        if self.cpu_model_mismatch {
            println!(
                "\nNOTE: no baseline run was recorded on this CPU model. Every verdict below is\n\
                 advisory — a baseline from different hardware is a different experiment.\n\
                 Only gross regressions can fail the job in this state."
            );
        }

        for comp in &self.comparisons {
            println!(
                "\n{} — {} samples, {} baseline run(s)",
                comp.key(),
                comp.sample_count,
                comp.baseline_runs
            );
            println!(
                "  {:<13} {:>12} {:>12} {:>10} {:>13}  {:<11} verdict",
                "metric", "baseline", "current", "change", "band (<=)", "source"
            );
            for m in &comp.metrics {
                let (baseline, current, band, change) = match m.verdict {
                    Verdict::NoBaseline => (
                        "—".to_string(),
                        m.metric.format(m.current),
                        "—".to_string(),
                        "—".to_string(),
                    ),
                    Verdict::InsufficientSamples => (
                        "—".to_string(),
                        m.metric.format(m.current),
                        "—".to_string(),
                        "—".to_string(),
                    ),
                    _ => (
                        m.metric.format(m.baseline_center),
                        m.metric.format(m.current),
                        m.metric.format(m.upper_band),
                        format_change(m.change_percent),
                    ),
                };
                let source = match m.verdict {
                    Verdict::NoBaseline | Verdict::InsufficientSamples => "—",
                    _ => match m.band_source {
                        BandSource::Relative => "relative",
                        BandSource::ObservedNoise => "noise",
                        BandSource::AbsoluteFloor => "floor",
                        BandSource::Fallback => "fallback",
                        BandSource::NoBaseline => "—",
                    },
                };
                println!(
                    "  {:<13} {:>12} {:>12} {:>10} {:>13}  {:<11} {}{}",
                    m.metric.label(),
                    baseline,
                    current,
                    change,
                    band,
                    source,
                    m.verdict.label(),
                    if m.blocking { "  [BLOCKING]" } else { "" }
                );
            }
        }

        if !self.missing_from_run.is_empty() {
            println!("\n--- benchmarks in the baseline that this run did not produce ---");
            for key in &self.missing_from_run {
                println!("  {}", key);
            }
            println!(
                "  An ungated critical path is not a pass. If these were intentionally renamed\n\
                 or removed, land the change on the trunk branch so the baseline refreshes, or\n\
                 delete the baseline history file to start a new window."
            );
        }

        self.print_failure_detail();

        println!();
        if self.has_blocking_regressions() {
            println!(
                "FAIL: {} blocking finding(s). See the detail above for the distribution\n\
                 on both sides and the threshold that was crossed.",
                self.blocking_findings().len() + usize::from(self.missing_is_blocking())
            );
        } else if self.has_regressions() {
            println!(
                "PASS with advisories: {} metric(s) regressed but are report-only on this\n\
                 measurement. They are not noise-free enough to fail a job on — see the\n\
                 per-metric bands above — but they are the numbers this project cares most\n\
                 about, so read them.",
                self.comparisons
                    .iter()
                    .map(|c| c.advisory_regressions().len())
                    .sum::<usize>()
            );
        } else if self.baseline_runs < self.policy.min_baseline_runs {
            println!(
                "PASS (inconclusive): the baseline window holds {} run(s); {} are needed before\n\
                 its spread can be estimated. Until then only gross regressions can fail.",
                self.baseline_runs, self.policy.min_baseline_runs
            );
        } else {
            println!(
                "PASS: no metric crossed its band. Note the band widths printed above — where a\n\
                 band is wider than {:.0}%, this run did not detect a regression and could not\n\
                 have.",
                self.policy.resolvable_rel * 100.0
            );
        }
    }

    fn print_failure_detail(&self) {
        if self.blocking_findings().is_empty() {
            return;
        }
        println!("\n--- why this failed ---");
        for comp in &self.comparisons {
            let blocking = comp.blocking();
            if blocking.is_empty() {
                continue;
            }

            // The distribution on both sides, once per benchmark: this is the
            // evidence, and a contributor should not have to read it five times
            // because five metrics tripped on the same change.
            println!("\n{}", comp.key());
            println!(
                "  {:<13} {:>12} {:>12} {:>12}  verdict",
                "metric", "baseline", "current", "change"
            );
            for m in &comp.metrics {
                if matches!(
                    m.verdict,
                    Verdict::NoBaseline | Verdict::InsufficientSamples
                ) {
                    continue;
                }
                println!(
                    "  {:<13} {:>12} {:>12} {:>12}  {}{}",
                    m.metric.label(),
                    m.metric.format(m.baseline_center),
                    m.metric.format(m.current),
                    format_change(m.change_percent),
                    m.verdict.label(),
                    if m.blocking { "  [BLOCKING]" } else { "" }
                );
            }

            for m in blocking {
                println!("\n  BLOCKING: {}", m.metric.label());
                println!(
                    "    {} -> {} ({})",
                    m.metric.format(m.baseline_center),
                    m.metric.format(m.current),
                    format_change(m.change_percent)
                );
                println!(
                    "    threshold {} — {}",
                    m.metric.format(m.upper_band),
                    m.band_source.describe()
                );
                if m.baseline_runs > 0 {
                    println!(
                        "    baseline: {} run(s), center {}, robust sigma {}, observed {} .. {}",
                        m.baseline_runs,
                        m.metric.format(m.baseline_center),
                        m.metric.format(m.baseline_spread),
                        m.metric.format(m.baseline_min),
                        m.metric.format(m.baseline_max),
                    );
                }
                if let Some(note) = &m.note {
                    println!("    note: {}", note);
                }
                if m.streak > 1 {
                    println!("    consecutive trunk runs above band: {}", m.streak);
                }
                println!("    what to do: {}", remedy_for(m.metric));
            }
        }
    }

    /// Render the report as Markdown for a pull-request comment.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        out.push_str("## Benchmark gate\n\n");
        out.push_str(&format!(
            "Baseline: rolling window of **{} run(s)** on `{}`. Current arm: median of **{} repetition(s)**.\n\n",
            self.baseline_runs,
            if self.cpu_model.is_empty() { "unknown CPU" } else { &self.cpu_model },
            self.repetitions,
        ));

        if self.cpu_model_mismatch {
            out.push_str(
                "> No baseline run matched this runner's CPU model. All verdicts are advisory.\n\n",
            );
        }

        if self.has_blocking_regressions() {
            out.push_str("### Result: FAIL\n\n");
            for (comp, m) in self.blocking_findings() {
                out.push_str(&format!(
                    "- **`{}` — {}**: {} -> {} ({}{:.1}%), threshold {} ({}).\n",
                    comp.key(),
                    m.metric.label(),
                    m.metric.format(m.baseline_center),
                    m.metric.format(m.current),
                    if m.change_percent >= 0.0 { "+" } else { "" },
                    m.change_percent,
                    m.metric.format(m.upper_band),
                    m.band_source.describe(),
                ));
            }
            if self.missing_is_blocking() {
                out.push_str(&format!(
                    "- **Missing from this run**: {}. These paths are no longer gated.\n",
                    self.missing_from_run.join(", ")
                ));
            }
            out.push('\n');
        } else if self.has_regressions() {
            out.push_str("### Result: PASS (with tail advisories)\n\n");
        } else {
            out.push_str("### Result: PASS\n\n");
        }

        for comp in &self.comparisons {
            out.push_str(&format!(
                "<details><summary><code>{}</code> — {} samples</summary>\n\n",
                comp.key(),
                comp.sample_count
            ));
            out.push_str("| metric | baseline | current | change | band (<=) | verdict |\n");
            out.push_str("|---|---:|---:|---:|---:|---|\n");
            for m in &comp.metrics {
                if matches!(
                    m.verdict,
                    Verdict::NoBaseline | Verdict::InsufficientSamples
                ) {
                    out.push_str(&format!(
                        "| `{}` | — | {} | — | — | {} |\n",
                        m.metric.label(),
                        m.metric.format(m.current),
                        m.verdict.label()
                    ));
                    continue;
                }
                out.push_str(&format!(
                    "| `{}` | {} | {} | {}{:.1}% | {} | {}{} |\n",
                    m.metric.label(),
                    m.metric.format(m.baseline_center),
                    m.metric.format(m.current),
                    if m.change_percent >= 0.0 { "+" } else { "" },
                    m.change_percent,
                    m.metric.format(m.upper_band),
                    m.verdict.label(),
                    if m.blocking { " **(blocking)**" } else { "" },
                ));
            }
            out.push_str("\n</details>\n\n");
        }

        out.push_str(&format!(
            "*Blocking metrics: {}. Report-only: {}. Tail metrics past p99.9 are advisory on a \
             shared runner — one host preemption owns `max`.*\n",
            self.policy
                .metrics
                .iter()
                .filter(|m| m.enforcement == Enforcement::Blocking)
                .map(|m| m.metric.label())
                .collect::<Vec<_>>()
                .join(", "),
            self.policy
                .metrics
                .iter()
                .filter(|m| m.enforcement == Enforcement::ReportOnly)
                .map(|m| m.metric.label())
                .collect::<Vec<_>>()
                .join(", "),
        ));
        out
    }
}

/// Signed percentage, pre-formatted so tables can right-align it.
fn format_change(percent: f64) -> String {
    format!("{}{:.1}%", if percent >= 0.0 { "+" } else { "" }, percent)
}

fn remedy_for(metric: Metric) -> &'static str {
    match metric {
        Metric::Median => {
            "the typical-case cost of the measured path grew. Profile the send/recv path and \
             compare against the previous commit with an interleaved A/B in one process."
        }
        Metric::TailRatioP99 | Metric::TailRatioP999 => {
            "the distribution got heavier relative to its own median, which is this project's \
             primary figure of merit. Look for a newly-added lock, allocation, syscall or \
             contended cache line on the hot path — something that is cheap most of the time \
             and expensive sometimes."
        }
        Metric::Max | Metric::MaxJitter | Metric::P9999 => {
            "a worst-case excursion grew far past anything the runner explains. Check for an \
             unbounded loop, a blocking wait, or a fault path newly reachable from the hot path."
        }
        _ => {
            "a tail percentile grew far past anything the runner explains. Check for a slow \
             path that is now taken occasionally on the measured route."
        }
    }
}

// =============================================================================
// Serialization helpers
// =============================================================================

/// Write benchmark report as JSON
pub fn write_json_report<P: AsRef<Path>>(report: &BenchmarkReport, path: P) -> std::io::Result<()> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    serde_json::to_writer_pretty(writer, report)?;
    Ok(())
}

/// Write benchmark results as CSV for spreadsheet analysis
pub fn write_csv_report<P: AsRef<Path>>(report: &BenchmarkReport, path: P) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    // Header
    writeln!(
        writer,
        "name,subject,message_size,mean_ns,median_ns,std_dev_ns,min_ns,max_ns,p95_ns,p99_ns,p999_ns,ci_low_ns,ci_high_ns,throughput_msg_sec,cv,deadline_misses"
    )?;

    // Data rows
    for result in &report.results {
        writeln!(
            writer,
            "{},{},{},{:.1},{:.1},{:.1},{},{},{},{},{},{:.1},{:.1},{:.1},{:.4},{}",
            result.name,
            result.subject,
            result.message_size,
            result.statistics.mean,
            result.statistics.median,
            result.statistics.std_dev,
            result.statistics.min,
            result.statistics.max,
            result.statistics.p95,
            result.statistics.p99,
            result.statistics.p999,
            result.statistics.ci_low,
            result.statistics.ci_high,
            result.throughput.messages_per_sec,
            result.determinism.cv,
            result.determinism.deadline_misses,
        )?;
    }

    Ok(())
}

/// Detect current git commit hash
fn detect_git_commit() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
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

/// Detect current git branch
fn detect_git_branch() -> Option<String> {
    std::process::Command::new("git")
        .args(["rev-parse", "--abbrev-ref", "HEAD"])
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

/// Format duration in human-readable form
pub fn format_duration_ns(ns: f64) -> String {
    if ns < 1000.0 {
        format!("{:.1} ns", ns)
    } else if ns < 1_000_000.0 {
        format!("{:.2} µs", ns / 1000.0)
    } else if ns < 1_000_000_000.0 {
        format!("{:.2} ms", ns / 1_000_000.0)
    } else {
        format!("{:.2} s", ns / 1_000_000_000.0)
    }
}

/// Format throughput in human-readable form
pub fn format_throughput(msgs_per_sec: f64) -> String {
    if msgs_per_sec < 1000.0 {
        format!("{:.1} msg/s", msgs_per_sec)
    } else if msgs_per_sec < 1_000_000.0 {
        format!("{:.2} K msg/s", msgs_per_sec / 1000.0)
    } else {
        format!("{:.2} M msg/s", msgs_per_sec / 1_000_000.0)
    }
}

/// Format bytes in human-readable form
pub fn format_bytes(bytes: f64) -> String {
    if bytes < 1024.0 {
        format!("{:.0} B", bytes)
    } else if bytes < 1024.0 * 1024.0 {
        format!("{:.2} KB", bytes / 1024.0)
    } else if bytes < 1024.0 * 1024.0 * 1024.0 {
        format!("{:.2} MB", bytes / (1024.0 * 1024.0))
    } else {
        format!("{:.2} GB", bytes / (1024.0 * 1024.0 * 1024.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        detect_platform, BenchmarkConfig, BenchmarkResult, DeterminismMetrics, Provenance,
        Statistics, ThroughputMetrics,
    };

    /// Shape of one benchmark result, parameterised on the numbers the gate
    /// actually reads.
    struct Shape {
        median: f64,
        p95: u64,
        p99: u64,
        p999: u64,
        p9999: u64,
        max: u64,
        count: usize,
    }

    impl Shape {
        /// A plausible healthy SHM topic: ~100 ns median with a tail an order
        /// of magnitude out, measured over enough samples for p99.9.
        fn healthy(median: f64) -> Self {
            Self {
                median,
                p95: (median * 1.8) as u64,
                p99: (median * 4.3) as u64,
                p999: (median * 12.0) as u64,
                p9999: (median * 40.0) as u64,
                max: (median * 90.0) as u64,
                count: 100_000,
            }
        }
    }

    fn make_result(name: &str, shape: &Shape) -> BenchmarkResult {
        BenchmarkResult {
            provenance: Provenance::Measured,
            name: name.to_string(),
            subject: "test".to_string(),
            message_size: 64,
            config: BenchmarkConfig::default(),
            platform: detect_platform(),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
            raw_latencies_ns: vec![],
            statistics: Statistics {
                count: shape.count,
                mean: shape.median,
                median: shape.median,
                std_dev: 10.0,
                min: (shape.median * 0.8) as u64,
                max: shape.max,
                p1: (shape.median * 0.85) as u64,
                p5: (shape.median * 0.9) as u64,
                p25: (shape.median * 0.95) as u64,
                p75: (shape.median * 1.2) as u64,
                p95: shape.p95,
                p99: shape.p99,
                p999: shape.p999,
                p9999: shape.p9999,
                ci_low: shape.median - 5.0,
                ci_high: shape.median + 5.0,
                confidence_level: 95.0,
                outliers_removed: 0,
            },
            throughput: ThroughputMetrics {
                messages_per_sec: 1_000_000.0,
                bytes_per_sec: 64_000_000.0,
                total_messages: 100_000,
                total_bytes: 6_400_000,
                duration_secs: 0.1,
            },
            determinism: DeterminismMetrics {
                cv: 0.1,
                max_jitter_ns: shape.max - (shape.median * 0.8) as u64,
                p999: shape.p999,
                p9999: shape.p9999,
                deadline_misses: 0,
                deadline_threshold_ns: 1000,
                run_variance: 0.05,
            },
        }
    }

    fn report_with(shape: &Shape) -> BenchmarkReport {
        let mut report = BenchmarkReport::new(detect_platform());
        report.add_result(make_result("test_bench", shape));
        report
    }

    /// A window of `n` runs whose medians wobble by the amount this harness
    /// actually wobbles on identical code.
    fn window(n: usize, medians: &[f64]) -> BaselineHistory {
        let mut history = BaselineHistory::new(20);
        for i in 0..n {
            let median = medians[i % medians.len()];
            history.push(&report_with(&Shape::healthy(median)));
        }
        history
    }

    #[test]
    fn noise_sized_wobble_against_a_real_window_is_not_a_regression() {
        // Ten runs of identical code with the spread this harness has actually
        // shown, then an eleventh inside that spread. A gate that fires here is
        // a gate that gets switched off.
        let history = window(10, &[95.0, 105.0, 99.0, 110.0, 92.0]);
        let current = report_with(&Shape::healthy(112.0));
        let report = history.compare(&[&current], &RegressionPolicy::default());
        assert!(
            !report.has_blocking_regressions(),
            "noise-sized wobble must not fail the job: {:?}",
            report
                .blocking_findings()
                .iter()
                .map(|(c, m)| (c.key(), m.metric.label(), m.change_percent))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn tail_regression_with_untouched_median_fails() {
        // The exact trade this project must never accept and the old gate could
        // not see: median unchanged, 50 µs added to p99.
        let history = window(10, &[100.0, 98.0, 103.0, 101.0, 99.0]);
        let mut shape = Shape::healthy(100.0);
        shape.p99 = 50_430;
        shape.p999 = 51_200;
        shape.p9999 = 52_000;
        shape.max = 60_000;
        let current = report_with(&shape);
        let report = history.compare(&[&current], &RegressionPolicy::default());

        assert!(
            report.has_blocking_regressions(),
            "a 50 µs tail regression with an untouched median must fail the job"
        );
        let flagged: Vec<&'static str> = report
            .blocking_findings()
            .iter()
            .map(|(_, m)| m.metric.label())
            .collect();
        assert!(
            flagged.contains(&"p99/median"),
            "tail shape must be among the blocking findings, got {:?}",
            flagged
        );
        // And the old median-only gate would have been perfectly happy.
        let median = report.comparisons[0].metric(Metric::Median).unwrap();
        assert!(median.change_percent.abs() < 5.0);
        assert!(!median.blocking);
    }

    #[test]
    fn gross_tail_blowup_blocks_even_on_a_report_only_metric() {
        let history = window(10, &[100.0, 98.0, 103.0, 101.0, 99.0]);
        let mut shape = Shape::healthy(100.0);
        // Median and p99 shape held constant on purpose: only the far tail moves.
        shape.p9999 = 4_000_000;
        shape.max = 8_000_000;
        let current = report_with(&shape);
        let report = history.compare(&[&current], &RegressionPolicy::default());
        assert!(
            report.has_blocking_regressions(),
            "a 100x far-tail excursion is past anything a runner explains and must block"
        );
    }

    #[test]
    fn one_run_baseline_cannot_resolve_a_large_jump_and_says_so() {
        // The old gate called this a regression at 5%. On a harness that has
        // produced 63 ns and 105 ns, and 99 ns and 193 ns, from the same
        // binary, a single-run comparison genuinely cannot tell an 80%
        // regression from a bad draw.
        let baseline = report_with(&Shape::healthy(100.0));
        let current = report_with(&Shape::healthy(180.0));
        let report = current.compare(&baseline);

        let median = report.comparisons[0].metric(Metric::Median).unwrap();
        assert_eq!(median.band_source, BandSource::Fallback);
        assert_eq!(median.verdict, Verdict::Inconclusive);
        assert!(!report.has_blocking_regressions());
    }

    #[test]
    fn one_run_baseline_still_blocks_a_gross_regression() {
        let baseline = report_with(&Shape::healthy(100.0));
        let current = report_with(&Shape::healthy(1_000.0));
        let report = current.compare(&baseline);
        assert!(report.has_blocking_regressions());
        assert_eq!(
            report.comparisons[0]
                .metric(Metric::Median)
                .unwrap()
                .verdict,
            Verdict::GrossRegression
        );
    }

    #[test]
    fn a_short_run_cannot_estimate_the_far_tail_and_refuses_to_try() {
        // 50_000 samples puts p99.99 at the 5th worst observation. Comparing
        // that across runs compares two lottery tickets.
        let mut history = BaselineHistory::new(20);
        for _ in 0..10 {
            let mut shape = Shape::healthy(100.0);
            shape.count = 50_000;
            history.push(&report_with(&shape));
        }
        let mut shape = Shape::healthy(100.0);
        shape.count = 50_000;
        let current = report_with(&shape);
        let report = history.compare(&[&current], &RegressionPolicy::default());
        assert_eq!(
            report.comparisons[0].metric(Metric::P9999).unwrap().verdict,
            Verdict::InsufficientSamples
        );
        // ...while p99, which 50k samples does support, is still evaluated.
        assert_ne!(
            report.comparisons[0].metric(Metric::P99).unwrap().verdict,
            Verdict::InsufficientSamples
        );
    }

    #[test]
    fn a_median_win_is_not_reported_as_a_tail_regression() {
        // Halving the median while leaving the tail alone doubles p99/median.
        // That is a win, not a regression, and the gate must not punish it.
        let history = window(10, &[200.0, 198.0, 203.0, 201.0, 199.0]);
        let mut shape = Shape::healthy(200.0);
        shape.median = 100.0;
        let current = report_with(&shape);
        let report = history.compare(&[&current], &RegressionPolicy::default());
        let ratio = report.comparisons[0].metric(Metric::TailRatioP99).unwrap();
        assert!(ratio.change_percent > 50.0, "the ratio did widen");
        assert!(!ratio.blocking, "but it must not block: {:?}", ratio.note);
        assert!(!report.has_blocking_regressions());
    }

    #[test]
    fn a_benchmark_that_vanished_from_the_run_is_a_hole_not_a_pass() {
        let history = window(10, &[100.0]);
        let empty = BenchmarkReport::new(detect_platform());
        let report = history.compare(&[&empty], &RegressionPolicy::default());
        assert_eq!(report.missing_from_run.len(), 1);
        assert!(report.has_blocking_regressions());
    }

    #[test]
    fn a_baseline_from_different_hardware_is_advisory_only() {
        let mut history = window(10, &[100.0]);
        for run in &mut history.runs {
            run.cpu_model = "Some Other CPU".to_string();
        }
        let current = report_with(&Shape::healthy(160.0));
        let report = history.compare(&[&current], &RegressionPolicy::default());
        assert!(report.cpu_model_mismatch);
        assert!(!report.has_blocking_regressions());
    }

    #[test]
    fn repetitions_are_reduced_by_median_not_by_worst_case() {
        // Three repetitions, one of them a bad draw. The current arm is the
        // median of the three, so the single bad run does not fail the job.
        let history = window(10, &[100.0, 98.0, 103.0, 101.0, 99.0]);
        let a = report_with(&Shape::healthy(100.0));
        let b = report_with(&Shape::healthy(101.0));
        let c = report_with(&Shape::healthy(240.0));
        let report = history.compare(&[&a, &b, &c], &RegressionPolicy::default());
        assert_eq!(report.repetitions, 3);
        let median = report.comparisons[0].metric(Metric::Median).unwrap();
        assert!((median.current - 101.0).abs() < 0.001);
        assert!(!report.has_blocking_regressions());
    }

    #[test]
    fn the_band_reports_where_it_came_from() {
        let history = window(10, &[100.0, 100.0, 100.0, 100.0, 100.0]);
        let current = report_with(&Shape::healthy(100.0));
        let report = history.compare(&[&current], &RegressionPolicy::default());
        let median = report.comparisons[0].metric(Metric::Median).unwrap();
        // A perfectly stable window has zero spread, so the absolute floor
        // (25 ns on a 100 ns median) is wider than 10% and must win.
        assert_eq!(median.band_source, BandSource::AbsoluteFloor);
        assert!((median.upper_band - 125.0).abs() < 0.001);
    }

    #[test]
    fn history_window_evicts_oldest() {
        let mut history = BaselineHistory::new(3);
        for m in [100.0, 101.0, 102.0, 103.0, 104.0] {
            history.push(&report_with(&Shape::healthy(m)));
        }
        assert_eq!(history.runs.len(), 3);
        assert!((history.runs[0].entries[0].median - 104.0).abs() < 0.001);
    }

    #[test]
    fn a_damaged_baseline_degrades_to_an_empty_window_not_a_red_pipeline() {
        let dir = std::env::temp_dir().join(format!("horus_gate_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("history.json");

        std::fs::write(&path, "{ this is not json").unwrap();
        let loaded = BaselineHistory::load(&path).expect("a damaged cache must not be an error");
        assert!(loaded.runs.is_empty());

        // Wrong format version: same treatment.
        let mut old = window(3, &[100.0]);
        old.version = "1.0.0".to_string();
        std::fs::write(&path, serde_json::to_string(&old).unwrap()).unwrap();
        let loaded = BaselineHistory::load(&path).expect("an old format must not be an error");
        assert!(loaded.runs.is_empty());

        // And a current-format file still loads.
        let good = window(3, &[100.0]);
        good.save(&path).unwrap();
        assert_eq!(BaselineHistory::load(&path).unwrap().runs.len(), 3);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn history_round_trips_through_json() {
        let history = window(4, &[100.0, 110.0]);
        let text = serde_json::to_string(&history).unwrap();
        let back: BaselineHistory = serde_json::from_str(&text).unwrap();
        assert_eq!(back.runs.len(), history.runs.len());
        assert_eq!(back.runs[0].entries[0].key(), "test_bench@64B");
    }

    #[test]
    fn markdown_names_the_failing_metric() {
        let history = window(10, &[100.0]);
        let mut shape = Shape::healthy(100.0);
        shape.p99 = 50_430;
        shape.p999 = 51_200;
        let current = report_with(&shape);
        let report = history.compare(&[&current], &RegressionPolicy::default());
        let md = report.to_markdown();
        assert!(md.contains("FAIL"));
        assert!(md.contains("p99/median"));
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration_ns(500.0), "500.0 ns");
        assert_eq!(format_duration_ns(1500.0), "1.50 µs");
        assert_eq!(format_duration_ns(1_500_000.0), "1.50 ms");
    }

    #[test]
    fn test_format_throughput() {
        assert_eq!(format_throughput(500.0), "500.0 msg/s");
        assert_eq!(format_throughput(50_000.0), "50.00 K msg/s");
        assert_eq!(format_throughput(5_000_000.0), "5.00 M msg/s");
    }
}
