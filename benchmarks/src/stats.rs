//! Statistical analysis functions for benchmarking.
//!
//! Provides:
//! - Bootstrap confidence intervals
//! - Percentile calculations
//! - Tukey outlier accounting (IQR-based, diagnostic only)
//! - Standard deviation and variance
//! - Coefficient of variation for determinism analysis
//! - Quantile support accounting (how many observations back a tail figure)
//! - Normality testing (Shapiro-Wilk, Jarque-Bera, Anderson-Darling)

use serde::{Deserialize, Deserializer, Serialize};

/// Deserialize an `f64` that may appear as `null`.
///
/// `Statistics::empty` stores `NaN` on purpose — see its doc — and
/// `serde_json` writes `NaN` as `null`. Without this, that report could be
/// written and never read back: the regression gate died on a real one with
/// `invalid type: null, expected f64`, so a single zero-sample benchmark took
/// down the whole gate rather than being reported as the empty run it was.
/// Reading `null` back as `NaN` closes the round trip and keeps the "no
/// samples is not zero latency" property on both sides of the wire.
pub(crate) fn nan_from_null<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    Ok(Option::<f64>::deserialize(d)?.unwrap_or(f64::NAN))
}

/// Comprehensive statistics for a benchmark run
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Statistics {
    /// Number of samples
    pub count: usize,
    /// Arithmetic mean (ns)
    #[serde(deserialize_with = "nan_from_null")]
    pub mean: f64,
    /// Median (50th percentile) (ns)
    #[serde(deserialize_with = "nan_from_null")]
    pub median: f64,
    /// Standard deviation (ns)
    #[serde(deserialize_with = "nan_from_null")]
    pub std_dev: f64,
    /// Minimum observed value (ns)
    pub min: u64,
    /// Maximum observed value (ns)
    pub max: u64,
    /// 1st percentile (ns)
    pub p1: u64,
    /// 5th percentile (ns)
    pub p5: u64,
    /// 25th percentile (ns)
    pub p25: u64,
    /// 75th percentile (ns)
    pub p75: u64,
    /// 95th percentile (ns)
    pub p95: u64,
    /// 99th percentile (ns)
    pub p99: u64,
    /// 99.9th percentile (ns)
    pub p999: u64,
    /// 99.99th percentile (ns)
    pub p9999: u64,
    /// Bootstrap confidence interval (low, high) at configured level
    #[serde(deserialize_with = "nan_from_null")]
    pub ci_low: f64,
    /// Bootstrap confidence interval high bound
    #[serde(deserialize_with = "nan_from_null")]
    pub ci_high: f64,
    /// Confidence level used (e.g., 95.0)
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub confidence_level: f64,
    /// Number of outliers removed (if filtering enabled)
    pub outliers_removed: usize,
}

impl Statistics {
    /// Compute statistics from raw latency samples.
    ///
    /// **Every published field is computed from the full, unfiltered sample
    /// set** — `mean`, `std_dev`, `ci_low`/`ci_high` included. `filter_outliers`
    /// now controls exactly one thing: whether `outliers_removed` is populated
    /// with a count of how many samples fall outside Tukey's
    /// `[Q1 - 1.5*IQR, Q3 + 1.5*IQR]` fence. It is a *diagnostic*, and no
    /// reported number is computed from what survives the fence.
    ///
    /// Two rounds of the same bug lived here. The first filtered before taking
    /// order statistics, so `max`, `p99`, `p99.9` and `p99.99` could not exceed
    /// `Q3 + 1.5*IQR` no matter what the machine did. The second left `mean`
    /// and `std_dev` on the fenced subset, which bounded
    /// `cv = std_dev / mean` — the published determinism number — by
    /// construction: the fence deletes precisely the preemptions and page
    /// faults that *constitute* jitter, so a row could report `cv: 0.05` beside
    /// `max_jitter_ns: 250000` and both were "correct". Worse, the four other
    /// benchmark binaries computed `cv` on unfiltered samples, so a field of the
    /// same name meant two different things depending on which binary wrote it.
    ///
    /// For a real-time middleware the tail *is* the measurement: an OS
    /// preemption or a page fault on a control loop is the event that misses the
    /// deadline, not an artifact to be discarded. If a fenced central estimate
    /// is genuinely wanted for some other purpose, call [`trimmed_mean`] and
    /// [`trimmed_std_dev`], which say so in their names.
    pub fn from_samples(samples: &[u64], confidence_level: f64, filter_outliers: bool) -> Self {
        if samples.is_empty() {
            return Self::empty(confidence_level);
        }

        let mut sorted = samples.to_vec();
        sorted.sort_unstable();

        // Diagnostic only: how many samples sit outside Tukey's fence. Nothing
        // below is computed from the survivors.
        let outliers_removed = if filter_outliers {
            samples.len() - self::filter_outliers(samples).len()
        } else {
            0
        };

        let count = sorted.len();
        let mean_val = mean(&sorted);
        let median_val = median(&sorted);
        let std_dev_val = std_dev(&sorted);

        // Full sample set here too, for the reason in the doc above: fencing the
        // central estimates bounds `cv = std_dev / mean` by construction, and
        // `cv` is the published determinism figure.
        let (ci_low, ci_high) = bootstrap_ci(&sorted, confidence_level, 10_000);

        Self {
            count,
            mean: mean_val,
            median: median_val,
            std_dev: std_dev_val,
            min: sorted[0],
            max: sorted[count - 1],
            p1: calculate_percentile(&sorted, 1.0),
            p5: calculate_percentile(&sorted, 5.0),
            p25: calculate_percentile(&sorted, 25.0),
            p75: calculate_percentile(&sorted, 75.0),
            p95: calculate_percentile(&sorted, 95.0),
            p99: calculate_percentile(&sorted, 99.0),
            p999: calculate_percentile(&sorted, 99.9),
            p9999: calculate_percentile(&sorted, 99.99),
            ci_low,
            ci_high,
            confidence_level,
            outliers_removed,
        }
    }

    /// Statistics for a run that produced no samples.
    ///
    /// The float fields are `NaN`, which `serde_json` writes as `null`. A zero
    /// here would read as "0 ns latency, 0 ns jitter" — the best possible
    /// result — in every table and chart downstream, which is the exact failure
    /// mode this module exists to prevent. The integer fields cannot carry
    /// `null`; check [`Statistics::is_empty`] before reading them.
    fn empty(confidence_level: f64) -> Self {
        Self {
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
            confidence_level,
            outliers_removed: 0,
        }
    }

    /// True when no samples were collected. The integer order statistics are
    /// meaningless in that case and must not be published.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Coefficient of variation, `std_dev / mean`, on the **unfiltered**
    /// samples.
    ///
    /// This is the determinism figure. Use it rather than recomputing
    /// `std_dev / mean` at the call site, so that every binary publishes a `cv`
    /// that means the same thing.
    pub fn cv(&self) -> f64 {
        if self.mean > 0.0 {
            self.std_dev / self.mean
        } else {
            f64::NAN
        }
    }

    /// Number of observations at or beyond the `p`th percentile — the
    /// observations that actually determine it.
    ///
    /// `p99.9` of 100,000 samples rests on 100 observations; `p99.99` of the
    /// same run rests on 10, and `p99.99` of 10,000 samples is the second
    /// largest sample wearing a percentile's name.
    ///
    /// Computed in integer arithmetic on hundredths of a percent. `1.0 - 99.9 /
    /// 100.0` is 0.0009999999999999998 in binary floating point, which floors
    /// 100,000 samples to 99 exceedances and would report `p99.9` as
    /// unsupported at exactly the sample count that supports it — a rule that
    /// mis-fires on its own boundary is not a rule.
    pub fn quantile_exceedances(&self, p: f64) -> usize {
        let hundredths = (p * 100.0).round().clamp(0.0, 10_000.0) as u64;
        let beyond = 10_000 - hundredths;
        ((self.count as u64).saturating_mul(beyond) / 10_000) as usize
    }

    /// Whether the `p`th percentile has enough observations behind it to be
    /// worth publishing. See [`MIN_TAIL_EXCEEDANCES`].
    pub fn quantile_supported(&self, p: f64) -> bool {
        self.quantile_exceedances(p) >= MIN_TAIL_EXCEEDANCES
    }

    /// Whether `p999` is supported by at least [`MIN_TAIL_EXCEEDANCES`]
    /// observations (needs n >= 100,000).
    pub fn p999_supported(&self) -> bool {
        self.quantile_supported(99.9)
    }

    /// Whether `p9999` is supported by at least [`MIN_TAIL_EXCEEDANCES`]
    /// observations (needs n >= 1,000,000).
    pub fn p9999_supported(&self) -> bool {
        self.quantile_supported(99.99)
    }
}

/// Minimum number of observations beyond a quantile before that quantile is
/// reportable.
///
/// **The rule: report the `p`th percentile only when `n * (1 - p/100) >= 100`.**
/// The quantile estimate's relative standard error is roughly `1/sqrt(k)` where
/// `k` is the number of exceedances, so 100 exceedances buys about +/-10% — the
/// loosest band in which a claimed tail improvement can be distinguished from
/// resampling noise at all. Below that the figure moves by tens of percent
/// between identical runs and any comparison drawn from it is unfalsifiable.
///
/// Concretely: `p99` needs n >= 10,000, `p99.9` needs n >= 100,000, and
/// `p99.99` needs n >= 1,000,000. A binary that collects 100,000 samples per
/// scenario may publish `p99.9` and must not publish `p99.99`.
pub const MIN_TAIL_EXCEEDANCES: usize = 100;

/// Calculate arithmetic mean
pub fn mean(samples: &[u64]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    samples.iter().map(|&x| x as f64).sum::<f64>() / samples.len() as f64
}

/// Calculate median (50th percentile)
/// Expects sorted input
pub fn median(sorted_samples: &[u64]) -> f64 {
    if sorted_samples.is_empty() {
        return 0.0;
    }
    let len = sorted_samples.len();
    if len.is_multiple_of(2) {
        (sorted_samples[len / 2 - 1] as f64 + sorted_samples[len / 2] as f64) / 2.0
    } else {
        sorted_samples[len / 2] as f64
    }
}

/// Calculate standard deviation (population)
pub fn std_dev(samples: &[u64]) -> f64 {
    if samples.len() < 2 {
        return 0.0;
    }
    let m = mean(samples);
    let variance = samples
        .iter()
        .map(|&x| {
            let diff = x as f64 - m;
            diff * diff
        })
        .sum::<f64>()
        / samples.len() as f64;
    variance.sqrt()
}

/// Calculate coefficient of variation (CV = std_dev / mean) on the samples as
/// given.
///
/// Never pass this the output of [`filter_outliers`]. The fence removes the
/// preemptions and faults that the coefficient of variation exists to report,
/// which makes the result bounded by construction rather than measured.
/// Lower is better for real-time determinism.
pub fn coefficient_of_variation(samples: &[u64]) -> f64 {
    let m = mean(samples);
    if m == 0.0 {
        return 0.0;
    }
    std_dev(samples) / m
}

/// Mean of the samples that fall inside Tukey's 1.5*IQR fence.
///
/// Named for what it is. This is a *trimmed* central estimate and is not
/// interchangeable with [`Statistics::mean`]: it deliberately discards the tail,
/// so it must never feed a jitter, determinism or worst-case figure.
pub fn trimmed_mean(samples: &[u64]) -> f64 {
    let kept = filter_outliers(samples);
    if kept.is_empty() {
        return mean(samples);
    }
    mean(&kept)
}

/// Standard deviation of the samples inside Tukey's 1.5*IQR fence.
///
/// See [`trimmed_mean`] for why this is not a determinism metric.
pub fn trimmed_std_dev(samples: &[u64]) -> f64 {
    let kept = filter_outliers(samples);
    if kept.is_empty() {
        return std_dev(samples);
    }
    std_dev(&kept)
}

/// Count samples that came out as exactly 0 ns.
///
/// A latency benchmark cannot observe a genuine 0 ns operation. Zeros are the
/// signature of an overhead subtraction that over-corrected: the calibrated
/// timing overhead is a *minimum* captured once, so whenever the instantaneous
/// instrumentation cost falls below it the `saturating_sub` floors the sample at
/// zero and it enters the distribution indistinguishable from a very fast
/// operation. That is the Tukey bug inverted onto the left tail — it drags the
/// median and the mean down and nothing counts it. Count it.
pub fn count_zero_samples(samples: &[u64]) -> usize {
    samples.iter().filter(|&&x| x == 0).count()
}

/// Calculate percentile using linear interpolation
/// Expects sorted input
pub fn calculate_percentile(sorted_samples: &[u64], percentile: f64) -> u64 {
    if sorted_samples.is_empty() {
        return 0;
    }
    if percentile <= 0.0 {
        return sorted_samples[0];
    }
    if percentile >= 100.0 {
        return sorted_samples[sorted_samples.len() - 1];
    }

    let len = sorted_samples.len();
    let rank = (percentile / 100.0) * (len - 1) as f64;
    let lower = rank.floor() as usize;
    let upper = (lower + 1).min(len - 1);
    let frac = rank - lower as f64;

    let lower_val = sorted_samples[lower] as f64;
    let upper_val = sorted_samples[upper] as f64;

    (lower_val + frac * (upper_val - lower_val)) as u64
}

/// Bootstrap confidence interval estimation
///
/// Uses bootstrap resampling to estimate the confidence interval for the mean.
/// This is more robust than parametric methods for non-normal distributions.
pub fn bootstrap_ci(samples: &[u64], confidence_level: f64, iterations: usize) -> (f64, f64) {
    if samples.is_empty() {
        return (0.0, 0.0);
    }
    if samples.len() == 1 {
        // A confidence interval cannot be estimated from one observation.
        // Returning `(val, val)` reported a zero-width 95% interval, i.e.
        // perfect precision, from the least precise possible measurement.
        return (f64::NAN, f64::NAN);
    }

    // Use a simple LCG PRNG for reproducibility (not cryptographic, just benchmark)
    let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_BABE;
    let lcg_next = |state: &mut u64| -> usize {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((*state >> 33) as usize) % samples.len()
    };

    let mut bootstrap_means = Vec::with_capacity(iterations);

    for _ in 0..iterations {
        // Resample with replacement
        let sum: f64 = (0..samples.len())
            .map(|_| samples[lcg_next(&mut rng_state)] as f64)
            .sum();
        bootstrap_means.push(sum / samples.len() as f64);
    }

    bootstrap_means.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let alpha = (100.0 - confidence_level) / 100.0;
    let lower_idx = ((alpha / 2.0) * iterations as f64) as usize;
    let upper_idx = ((1.0 - alpha / 2.0) * iterations as f64) as usize;

    let lower_idx = lower_idx.min(iterations - 1);
    let upper_idx = upper_idx.min(iterations - 1);

    (bootstrap_means[lower_idx], bootstrap_means[upper_idx])
}

/// Filter outliers using Tukey's method (IQR-based)
///
/// Removes samples that fall outside [Q1 - 1.5*IQR, Q3 + 1.5*IQR].
///
/// Only ever use this for central estimates (mean, std dev, confidence
/// interval). Applying it before a tail statistic caps `max`/`p99.9` at the
/// upper fence, which makes a "worst-case latency" number that cannot be
/// large — see `Statistics::from_samples`.
pub fn filter_outliers(samples: &[u64]) -> Vec<u64> {
    if samples.len() < 4 {
        return samples.to_vec();
    }

    let mut sorted = samples.to_vec();
    sorted.sort_unstable();

    let q1 = calculate_percentile(&sorted, 25.0) as f64;
    let q3 = calculate_percentile(&sorted, 75.0) as f64;
    let iqr = q3 - q1;

    // Tukey fence: 1.5 * IQR
    let lower_fence = q1 - 1.5 * iqr;
    let upper_fence = q3 + 1.5 * iqr;

    samples
        .iter()
        .filter(|&&x| {
            let xf = x as f64;
            xf >= lower_fence && xf <= upper_fence
        })
        .copied()
        .collect()
}

// ============================================================================
// Normality Testing
// ============================================================================

/// Result of normality analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NormalityAnalysis {
    /// Sample size used for analysis
    pub sample_size: usize,
    /// Skewness (0 = symmetric like normal)
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub skewness: f64,
    /// Excess kurtosis (0 = normal, >0 = heavy tails, <0 = light tails)
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub kurtosis: f64,
    /// Jarque-Bera test statistic
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub jarque_bera_stat: f64,
    /// Jarque-Bera p-value (>0.05 suggests normality)
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub jarque_bera_pvalue: f64,
    /// Anderson-Darling test statistic
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub anderson_darling_stat: f64,
    /// D'Agostino-Pearson K² statistic
    #[serde(deserialize_with = "crate::stats::nan_from_null")]
    pub dagostino_k2: f64,
    /// Is distribution likely normal? (based on combined tests)
    pub is_likely_normal: bool,
    /// Recommended statistical approach based on results
    pub recommendation: String,
}

impl NormalityAnalysis {
    /// Perform comprehensive normality analysis on samples
    pub fn analyze(samples: &[u64]) -> Self {
        if samples.len() < 20 {
            return Self::insufficient_samples(samples.len());
        }

        // Convert to f64 for analysis
        let values: Vec<f64> = samples.iter().map(|&x| x as f64).collect();

        let n = values.len() as f64;
        let m = mean(samples);
        let s = std_dev(samples);

        // Calculate skewness (Fisher's definition)
        let skewness = if s > 0.0 {
            let m3: f64 = values.iter().map(|&x| ((x - m) / s).powi(3)).sum::<f64>() / n;
            m3
        } else {
            0.0
        };

        // Calculate excess kurtosis (Fisher's definition)
        let kurtosis = if s > 0.0 {
            let m4: f64 = values.iter().map(|&x| ((x - m) / s).powi(4)).sum::<f64>() / n;
            m4 - 3.0 // Excess kurtosis (normal = 0)
        } else {
            0.0
        };

        // Jarque-Bera test
        // JB = (n/6) * (S² + K²/4)
        // Under null hypothesis of normality, JB ~ chi²(2)
        let jb_stat = (n / 6.0) * (skewness.powi(2) + kurtosis.powi(2) / 4.0);
        let jb_pvalue = chi2_survival(jb_stat, 2.0);

        // Anderson-Darling test (simplified)
        let ad_stat = anderson_darling_statistic(&values, m, s);

        // D'Agostino-Pearson K² test
        let k2_stat = dagostino_k2(skewness, kurtosis, n);

        // Combined assessment
        // Normal if: |skewness| < 2, |kurtosis| < 7, and at least one test passes
        let skew_ok = skewness.abs() < 2.0;
        let kurt_ok = kurtosis.abs() < 7.0;
        let jb_ok = jb_pvalue > 0.05;

        let is_likely_normal = skew_ok && kurt_ok && jb_ok;

        let recommendation = if is_likely_normal {
            "Distribution appears approximately normal. Parametric tests (t-test) may be appropriate.".to_string()
        } else if !skew_ok {
            format!(
                "Distribution is {} skewed (skewness={:.2}). Use non-parametric methods (median, percentiles).",
                if skewness > 0.0 { "right" } else { "left" },
                skewness
            )
        } else if !kurt_ok {
            format!(
                "Distribution has {} tails (kurtosis={:.2}). Use robust statistics.",
                if kurtosis > 0.0 { "heavy" } else { "light" },
                kurtosis
            )
        } else {
            "Distribution deviates from normality. Non-parametric methods recommended.".to_string()
        };

        Self {
            sample_size: samples.len(),
            skewness,
            kurtosis,
            jarque_bera_stat: jb_stat,
            jarque_bera_pvalue: jb_pvalue,
            anderson_darling_stat: ad_stat,
            dagostino_k2: k2_stat,
            is_likely_normal,
            recommendation,
        }
    }

    /// Result for a sample set too small to test.
    ///
    /// Every statistic is `NaN` (`null` in JSON). The previous values —
    /// `skewness: 0.0`, `kurtosis: 0.0`, `jarque_bera_pvalue: 1.0` — are not
    /// neutral placeholders: they are, respectively, "perfectly symmetric",
    /// "exactly normal tails" and "cannot reject normality with certainty".
    /// A table or chart built from the JSON reads them as the strongest
    /// possible evidence of normality, produced by a test that never ran.
    fn insufficient_samples(n: usize) -> Self {
        Self {
            sample_size: n,
            skewness: f64::NAN,
            kurtosis: f64::NAN,
            jarque_bera_stat: f64::NAN,
            jarque_bera_pvalue: f64::NAN,
            anderson_darling_stat: f64::NAN,
            dagostino_k2: f64::NAN,
            is_likely_normal: false,
            recommendation: format!(
                "Insufficient samples ({}) for normality testing. Need at least 20.",
                n
            ),
        }
    }

    /// Print a formatted report of the normality analysis
    pub fn print_report(&self) {
        println!("╔══════════════════════════════════════════════════════════════════╗");
        println!("║                    NORMALITY ANALYSIS                            ║");
        println!("╠══════════════════════════════════════════════════════════════════╣");
        println!(
            "║ Sample size:      {:>8}                                       ║",
            self.sample_size
        );
        println!(
            "║ Skewness:         {:>8.4} (0 = symmetric)                      ║",
            self.skewness
        );
        println!(
            "║ Excess Kurtosis:  {:>8.4} (0 = normal tails)                   ║",
            self.kurtosis
        );
        println!("╠══════════════════════════════════════════════════════════════════╣");
        println!(
            "║ Jarque-Bera:      {:>8.2} (p={:.4})                           ║",
            self.jarque_bera_stat, self.jarque_bera_pvalue
        );
        println!(
            "║ Anderson-Darling: {:>8.4}                                       ║",
            self.anderson_darling_stat
        );
        println!(
            "║ D'Agostino K²:    {:>8.4}                                       ║",
            self.dagostino_k2
        );
        println!("╠══════════════════════════════════════════════════════════════════╣");
        println!(
            "║ Assessment:       {}                                        ║",
            if self.is_likely_normal {
                "LIKELY NORMAL"
            } else {
                "NON-NORMAL   "
            }
        );
        println!("╠══════════════════════════════════════════════════════════════════╣");
        println!("║ Recommendation:                                                  ║");
        // Word wrap the recommendation
        for line in textwrap(&self.recommendation, 60) {
            println!("║ {:60} ║", line);
        }
        println!("╚══════════════════════════════════════════════════════════════════╝");
    }
}

/// Calculate skewness of samples
pub fn skewness(samples: &[u64]) -> f64 {
    if samples.len() < 3 {
        return 0.0;
    }
    let m = mean(samples);
    let s = std_dev(samples);
    if s == 0.0 {
        return 0.0;
    }
    let n = samples.len() as f64;
    samples
        .iter()
        .map(|&x| ((x as f64 - m) / s).powi(3))
        .sum::<f64>()
        / n
}

/// Calculate excess kurtosis of samples (normal distribution = 0)
pub fn excess_kurtosis(samples: &[u64]) -> f64 {
    if samples.len() < 4 {
        return 0.0;
    }
    let m = mean(samples);
    let s = std_dev(samples);
    if s == 0.0 {
        return 0.0;
    }
    let n = samples.len() as f64;
    let m4: f64 = samples
        .iter()
        .map(|&x| ((x as f64 - m) / s).powi(4))
        .sum::<f64>()
        / n;
    m4 - 3.0
}

/// Jarque-Bera test for normality
/// Returns (test_statistic, p_value)
pub fn jarque_bera_test(samples: &[u64]) -> (f64, f64) {
    let n = samples.len() as f64;
    let s = skewness(samples);
    let k = excess_kurtosis(samples);

    // JB = (n/6) * (S² + K²/4)
    let jb = (n / 6.0) * (s.powi(2) + k.powi(2) / 4.0);

    // P-value from chi-squared distribution with 2 degrees of freedom
    let pvalue = chi2_survival(jb, 2.0);

    (jb, pvalue)
}

/// Anderson-Darling test statistic (simplified implementation)
fn anderson_darling_statistic(values: &[f64], mean: f64, std_dev: f64) -> f64 {
    if std_dev == 0.0 || values.len() < 8 {
        return 0.0;
    }

    let n = values.len();
    let mut sorted: Vec<f64> = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Standardize values
    let z: Vec<f64> = sorted.iter().map(|&x| (x - mean) / std_dev).collect();

    // Calculate A² statistic
    let mut sum = 0.0;
    for i in 0..n {
        let cdf_zi = standard_normal_cdf(z[i]);
        let cdf_zn_i = standard_normal_cdf(z[n - 1 - i]);

        // Avoid log(0) or log(1)
        let cdf_zi = cdf_zi.clamp(1e-10, 1.0 - 1e-10);
        let cdf_zn_i = cdf_zn_i.clamp(1e-10, 1.0 - 1e-10);

        sum += (2.0 * (i + 1) as f64 - 1.0) * (cdf_zi.ln() + (1.0 - cdf_zn_i).ln());
    }

    let a2 = -(n as f64) - sum / n as f64;

    // Apply correction factor for unknown mean and variance
    a2 * (1.0 + 0.75 / n as f64 + 2.25 / (n * n) as f64)
}

/// D'Agostino-Pearson K² test statistic
fn dagostino_k2(skewness: f64, kurtosis: f64, n: f64) -> f64 {
    // Simplified: just combine skewness and kurtosis z-scores
    // Full implementation requires transformation functions
    let z_s = skewness / (6.0 / n).sqrt();
    let z_k = kurtosis / (24.0 / n).sqrt();
    z_s.powi(2) + z_k.powi(2)
}

/// Standard normal CDF approximation (Abramowitz and Stegun)
fn standard_normal_cdf(x: f64) -> f64 {
    // Constants for approximation
    const A1: f64 = 0.254829592;
    const A2: f64 = -0.284496736;
    const A3: f64 = 1.421413741;
    const A4: f64 = -1.453152027;
    const A5: f64 = 1.061405429;
    const P: f64 = 0.3275911;

    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();

    let t = 1.0 / (1.0 + P * x);
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t3 * t;
    let t5 = t4 * t;

    let y = 1.0 - (A1 * t + A2 * t2 + A3 * t3 + A4 * t4 + A5 * t5) * (-x * x / 2.0).exp();

    0.5 * (1.0 + sign * y)
}

/// Chi-squared survival function (1 - CDF) approximation
/// For df=2: P(X > x) = e^(-x/2)
fn chi2_survival(x: f64, df: f64) -> f64 {
    if df == 2.0 {
        // Exact formula for df=2
        (-x / 2.0).exp()
    } else {
        // Wilson-Hilferty approximation for general df
        let k = df;
        let z = ((x / k).powf(1.0 / 3.0) - (1.0 - 2.0 / (9.0 * k))) / (2.0 / (9.0 * k)).sqrt();
        1.0 - standard_normal_cdf(z)
    }
}

/// Simple text wrapper for console output
fn textwrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut current_line = String::new();

    for word in text.split_whitespace() {
        if current_line.is_empty() {
            current_line = word.to_string();
        } else if current_line.len() + 1 + word.len() <= width {
            current_line.push(' ');
            current_line.push_str(word);
        } else {
            lines.push(current_line);
            current_line = word.to_string();
        }
    }

    if !current_line.is_empty() {
        lines.push(current_line);
    }

    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mean() {
        let samples = vec![100, 200, 300, 400, 500];
        assert!((mean(&samples) - 300.0).abs() < 0.001);
    }

    #[test]
    fn test_median_odd() {
        let samples = vec![100, 200, 300, 400, 500];
        assert!((median(&samples) - 300.0).abs() < 0.001);
    }

    #[test]
    fn test_median_even() {
        let samples = vec![100, 200, 300, 400];
        assert!((median(&samples) - 250.0).abs() < 0.001);
    }

    #[test]
    fn test_percentile() {
        let samples = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        assert_eq!(calculate_percentile(&samples, 50.0), 5);
        assert_eq!(calculate_percentile(&samples, 0.0), 1);
        assert_eq!(calculate_percentile(&samples, 100.0), 10);
    }

    #[test]
    fn test_std_dev() {
        let samples = vec![2, 4, 4, 4, 5, 5, 7, 9];
        let sd = std_dev(&samples);
        assert!((sd - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_cv() {
        let samples = vec![100, 100, 100, 100]; // Perfect consistency
        assert!(coefficient_of_variation(&samples).abs() < 0.001);

        let variable = vec![50, 100, 150, 200]; // High variation
        assert!(coefficient_of_variation(&variable) > 0.3);
    }

    #[test]
    fn test_outlier_filtering() {
        let samples = vec![100, 101, 102, 103, 104, 105, 10000]; // 10000 is outlier
        let filtered = filter_outliers(&samples);
        assert!(!filtered.contains(&10000));
        assert!(filtered.len() < samples.len());
    }

    #[test]
    fn test_bootstrap_ci() {
        let samples: Vec<u64> = (0..1000).map(|x| 100 + (x % 20)).collect();
        let (low, high) = bootstrap_ci(&samples, 95.0, 1000);
        assert!(low < high);
        assert!(low > 90.0);
        assert!(high < 130.0);
    }

    #[test]
    fn test_statistics_from_samples() {
        let samples: Vec<u64> = (0..1000).map(|x| 500 + (x % 100)).collect();
        let stats = Statistics::from_samples(&samples, 95.0, true);
        assert!(stats.count > 0);
        assert!(stats.mean > 0.0);
        assert!(stats.p99 >= stats.p95);
        assert!(stats.ci_low <= stats.ci_high);
    }

    /// The `filter_outliers` flag must not change a single published number.
    ///
    /// Two rounds of the same bug lived in `from_samples`: first the order
    /// statistics were taken from the fenced subset (capping `max` and `p99.9`
    /// at Tukey's upper fence), then the mean and standard deviation stayed on
    /// it (capping `cv`, the determinism figure, at whatever the fence allowed).
    /// The flag is now a diagnostic counter and nothing else, so a genuine
    /// 50 us stall in an otherwise tight distribution must survive into every
    /// statistic with the flag set exactly as it does with it clear.
    #[test]
    fn outlier_flag_changes_no_published_statistic() {
        let mut samples: Vec<u64> = (0..999).map(|x| 100 + (x % 5)).collect();
        samples.push(50_000); // a real preemption, not a measurement artifact

        let filtered = Statistics::from_samples(&samples, 95.0, true);
        let unfiltered = Statistics::from_samples(&samples, 95.0, false);

        assert_eq!(
            filtered.max, 50_000,
            "max must come from the unfiltered samples"
        );
        assert_eq!(
            filtered.count,
            samples.len(),
            "count must be the full sample count"
        );
        assert_eq!(
            filtered.outliers_removed, 1,
            "the stall is still counted as outside the fence, as a diagnostic"
        );
        assert_eq!(unfiltered.outliers_removed, 0);

        assert_eq!(filtered.max, unfiltered.max);
        assert_eq!(filtered.p99, unfiltered.p99);
        assert_eq!(filtered.p999, unfiltered.p999);
        assert_eq!(filtered.p9999, unfiltered.p9999);
        assert_eq!(filtered.mean, unfiltered.mean);
        assert_eq!(filtered.std_dev, unfiltered.std_dev);
        assert_eq!(filtered.cv(), unfiltered.cv());
    }

    /// The determinism figure must be able to report a bad number.
    ///
    /// One 50 us stall in a 1000-sample run of a ~100 ns operation is a CV well
    /// above 1. The old filtered CV reported ~0.02 for this exact distribution,
    /// because the fence deleted the only sample that carried any jitter.
    #[test]
    fn cv_reports_the_stall_it_exists_to_report() {
        let mut samples: Vec<u64> = (0..999).map(|x| 100 + (x % 5)).collect();
        samples.push(50_000);

        let stats = Statistics::from_samples(&samples, 95.0, true);
        assert!(
            stats.cv() > 1.0,
            "cv must see the stall, got {} (fenced cv would be ~0.02)",
            stats.cv()
        );

        // The distinctly-named trimmed estimator is the one allowed to hide it.
        assert!(trimmed_mean(&samples) < stats.mean);
        assert!(trimmed_std_dev(&samples) < stats.std_dev);
    }

    /// A zero-width Tukey fence must not swallow the distribution.
    #[test]
    fn constant_samples_survive_outlier_filtering() {
        let samples = vec![7_u64; 8]; // IQR is 0, so the fence is a single point
        let stats = Statistics::from_samples(&samples, 95.0, true);
        assert_eq!(stats.count, 8);
        assert_eq!(stats.max, 7);
        assert_eq!(stats.min, 7);
        assert_eq!(stats.mean, 7.0);
    }

    /// A quantile must not be published unless enough observations lie beyond
    /// it. `p99.99` of 100,000 samples rests on 10 observations.
    #[test]
    fn tail_quantiles_declare_their_support() {
        // Support depends only on the sample count, so set it directly rather
        // than running a 10,000-resample bootstrap over a million samples to
        // learn how many samples there are.
        let with_count = |n: usize| {
            let mut s = Statistics::empty(95.0);
            s.count = n;
            s
        };

        let s = with_count(100_000);
        assert_eq!(s.quantile_exceedances(99.0), 1_000);
        assert_eq!(s.quantile_exceedances(99.9), 100);
        assert_eq!(s.quantile_exceedances(99.99), 10);
        assert!(s.p999_supported(), "n=1e5 supports p99.9 exactly");
        assert!(!s.p9999_supported(), "n=1e5 cannot support p99.99");

        assert!(
            with_count(1_000_000).p9999_supported(),
            "n=1e6 supports p99.99"
        );
        assert!(
            !with_count(10_000).p999_supported(),
            "n=1e4 cannot support p99.9"
        );
        assert!(
            with_count(10_000).quantile_supported(99.0),
            "n=1e4 supports p99 exactly"
        );

        // And the real path agrees, at a size a unit test can afford.
        let real = Statistics::from_samples(&vec![100_u64; 2_000], 95.0, false);
        assert_eq!(real.count, 2_000);
        assert_eq!(real.quantile_exceedances(99.0), 20);
        assert!(!real.quantile_supported(99.0));
    }

    /// An empty run must not read as a perfect one.
    #[test]
    fn empty_statistics_are_not_zero() {
        let s = Statistics::from_samples(&[], 95.0, false);
        assert!(s.is_empty());
        assert!(s.mean.is_nan(), "a 0.0 mean reads as 0 ns latency");
        assert!(s.median.is_nan());
        assert!(s.std_dev.is_nan());
        assert!(s.ci_low.is_nan());
        assert!(s.ci_high.is_nan());
        assert!(s.cv().is_nan());
        // serde_json writes non-finite floats as null.
        let j = serde_json::to_string(&s).unwrap();
        assert!(j.contains("\"mean\":null"), "{}", j);
    }

    /// Over-subtracted overhead floors samples at exactly 0 ns; count them.
    #[test]
    fn zero_samples_are_counted() {
        let samples = vec![0_u64, 0, 120, 130, 0, 140];
        assert_eq!(count_zero_samples(&samples), 3);
        assert_eq!(count_zero_samples(&[100, 200]), 0);
    }

    #[test]
    fn test_skewness_symmetric() {
        // Symmetric distribution should have ~0 skewness
        let samples: Vec<u64> = (0..100).map(|x| 100 + (x % 20) - 10).collect();
        let skew = skewness(&samples);
        assert!(skew.abs() < 1.0, "Expected low skewness, got {}", skew);
    }

    #[test]
    fn test_skewness_right_skewed() {
        // Right-skewed: many small values, few large values
        let mut samples: Vec<u64> = vec![100; 90];
        samples.extend(vec![1000; 10]);
        let skew = skewness(&samples);
        assert!(skew > 0.0, "Expected positive skewness, got {}", skew);
    }

    #[test]
    fn test_excess_kurtosis() {
        // A uniform distribution has negative excess kurtosis
        let samples: Vec<u64> = (100..200).collect();
        let kurt = excess_kurtosis(&samples);
        assert!(
            kurt < 0.0,
            "Uniform should have negative kurtosis, got {}",
            kurt
        );
    }

    #[test]
    fn test_jarque_bera_symmetric() {
        // Roughly symmetric data should have low JB statistic
        let samples: Vec<u64> = (0..1000).map(|x| 1000 + (x % 100)).collect();
        let (jb, _pvalue) = jarque_bera_test(&samples);
        // For roughly uniform data, JB should be small but pvalue depends on sample size
        assert!(jb >= 0.0, "JB statistic should be non-negative");
    }

    #[test]
    fn test_normality_analysis() {
        // Create a dataset and run full normality analysis
        let samples: Vec<u64> = (0..1000).map(|x| 500 + (x % 100)).collect();
        let analysis = NormalityAnalysis::analyze(&samples);

        assert_eq!(analysis.sample_size, 1000);
        assert!(analysis.jarque_bera_stat >= 0.0);
        assert!(analysis.jarque_bera_pvalue >= 0.0);
        assert!(analysis.jarque_bera_pvalue <= 1.0);
    }

    #[test]
    fn test_normality_insufficient_samples() {
        let samples: Vec<u64> = vec![100, 200, 300];
        let analysis = NormalityAnalysis::analyze(&samples);
        assert!(!analysis.is_likely_normal);
        assert!(analysis.recommendation.contains("Insufficient"));
    }

    #[test]
    fn test_standard_normal_cdf() {
        // CDF at 0 should be 0.5 for standard normal
        let cdf_0 = standard_normal_cdf(0.0);
        assert!(
            (cdf_0 - 0.5).abs() < 0.01,
            "CDF(0) should be ~0.5, got {}",
            cdf_0
        );

        // CDF should be monotonically increasing
        let cdf_neg = standard_normal_cdf(-2.0);
        let cdf_pos = standard_normal_cdf(2.0);
        assert!(cdf_neg < cdf_0, "CDF should be increasing");
        assert!(cdf_0 < cdf_pos, "CDF should be increasing");

        // Extreme values
        assert!(standard_normal_cdf(-4.0) < 0.01);
        assert!(standard_normal_cdf(4.0) > 0.99);
    }

    #[test]
    fn test_chi2_survival() {
        // For df=2, survival function is e^(-x/2)
        let survival = chi2_survival(2.0, 2.0);
        let expected = (-1.0_f64).exp(); // e^(-1)
        assert!(
            (survival - expected).abs() < 0.01,
            "Expected {}, got {}",
            expected,
            survival
        );

        // Survival at 0 should be 1
        let survival_0 = chi2_survival(0.0, 2.0);
        assert!(
            (survival_0 - 1.0).abs() < 0.01,
            "Survival(0) should be ~1, got {}",
            survival_0
        );
    }

    /// A run that produced no samples has to survive being written and read
    /// back. `Statistics::empty` stores `NaN` deliberately — a zero would read
    /// as "0 ns latency", the best possible result, in every table downstream —
    /// and `serde_json` writes `NaN` as `null`. Nothing taught the deserializer
    /// to read `null` back, so the regression gate died on the first real
    /// report containing an empty benchmark with `invalid type: null, expected
    /// f64` and took the whole gate down with it, rather than reporting the one
    /// empty run.
    #[test]
    fn a_zero_sample_run_survives_the_json_round_trip() {
        let empty = Statistics::from_samples(&[], 95.0, false);
        assert!(empty.mean.is_nan(), "empty() must not report 0 ns");

        let json = serde_json::to_string(&empty).expect("serialize");
        assert!(
            json.contains("null"),
            "NaN is expected to serialize as null: {json}"
        );

        let back: Statistics = serde_json::from_str(&json).expect("round trip");
        assert_eq!(back.count, 0);
        for (name, v) in [
            ("mean", back.mean),
            ("median", back.median),
            ("std_dev", back.std_dev),
            ("ci_low", back.ci_low),
            ("ci_high", back.ci_high),
        ] {
            assert!(v.is_nan(), "{name} came back as {v}, not NaN");
        }
    }
}
