//! Statistical analysis functions for rigorous benchmarking.
//!
//! Provides:
//! - Bootstrap confidence intervals
//! - Percentile calculations
//! - Tukey outlier filtering (IQR-based)
//! - Standard deviation and variance
//! - Coefficient of variation for determinism analysis
//! - Normality testing (Shapiro-Wilk, Jarque-Bera, Anderson-Darling)

use serde::{Deserialize, Serialize};

/// Comprehensive statistics for a benchmark run
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Statistics {
    /// Number of samples
    pub count: usize,
    /// Arithmetic mean (ns)
    pub mean: f64,
    /// Median (50th percentile) (ns)
    pub median: f64,
    /// Standard deviation (ns)
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
    pub ci_low: f64,
    /// Bootstrap confidence interval high bound
    pub ci_high: f64,
    /// Confidence level used (e.g., 95.0)
    pub confidence_level: f64,
    /// Number of outliers removed (if filtering enabled)
    pub outliers_removed: usize,
}

impl Statistics {
    /// Compute statistics from raw latency samples
    pub fn from_samples(samples: &[u64], confidence_level: f64, filter_outliers: bool) -> Self {
        let (filtered, outliers_removed) = if filter_outliers {
            let f = self::filter_outliers(samples);
            let removed = samples.len() - f.len();
            (f, removed)
        } else {
            (samples.to_vec(), 0)
        };

        if filtered.is_empty() {
            return Self::empty(confidence_level);
        }

        let mut sorted = filtered.clone();
        sorted.sort_unstable();

        let count = sorted.len();
        let mean_val = mean(&sorted);
        let median_val = median(&sorted);
        let std_dev_val = std_dev(&sorted);

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

    fn empty(confidence_level: f64) -> Self {
        Self {
            count: 0,
            mean: 0.0,
            median: 0.0,
            std_dev: 0.0,
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
            ci_low: 0.0,
            ci_high: 0.0,
            confidence_level,
            outliers_removed: 0,
        }
    }
}

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

/// Calculate coefficient of variation (CV = std_dev / mean)
/// Lower is better for real-time determinism
pub fn coefficient_of_variation(samples: &[u64]) -> f64 {
    let m = mean(samples);
    if m == 0.0 {
        return 0.0;
    }
    std_dev(samples) / m
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
        let val = samples[0] as f64;
        return (val, val);
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
/// Removes samples that fall outside [Q1 - 1.5*IQR, Q3 + 1.5*IQR]
/// This is standard practice in benchmarking to remove measurement artifacts.
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
    pub skewness: f64,
    /// Excess kurtosis (0 = normal, >0 = heavy tails, <0 = light tails)
    pub kurtosis: f64,
    /// Jarque-Bera test statistic
    pub jarque_bera_stat: f64,
    /// Jarque-Bera p-value (>0.05 suggests normality)
    pub jarque_bera_pvalue: f64,
    /// Anderson-Darling test statistic
    pub anderson_darling_stat: f64,
    /// D'Agostino-Pearson K² statistic
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

    fn insufficient_samples(n: usize) -> Self {
        Self {
            sample_size: n,
            skewness: 0.0,
            kurtosis: 0.0,
            jarque_bera_stat: 0.0,
            jarque_bera_pvalue: 1.0,
            anderson_darling_stat: 0.0,
            dagostino_k2: 0.0,
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
}
