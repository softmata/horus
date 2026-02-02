//! High-precision timing utilities for benchmarking.
//!
//! Provides:
//! - RDTSC-based cycle counting for x86_64
//! - Cycle-to-nanosecond conversion with calibration
//! - Fallback timing for non-x86 platforms

use std::time::{Duration, Instant};

/// Read Time Stamp Counter (RDTSC) for cycle-accurate timing
///
/// Returns CPU cycle count. On modern CPUs with constant_tsc,
/// this is monotonic and consistent across cores.
#[cfg(target_arch = "x86_64")]
#[inline(always)]
pub fn rdtsc() -> u64 {
    unsafe { core::arch::x86_64::_rdtsc() }
}

#[cfg(not(target_arch = "x86_64"))]
#[inline(always)]
pub fn rdtsc() -> u64 {
    // Fallback: use SystemTime-based timing (less precise)
    // Note: Instant doesn't have a fixed epoch, so we use SystemTime for consistency
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// RDTSCP with serialization - more accurate for timing small sections
///
/// RDTSCP waits for all previous instructions to complete before reading,
/// providing more accurate measurements.
#[cfg(target_arch = "x86_64")]
#[inline(always)]
pub fn rdtscp() -> u64 {
    let mut aux: u32 = 0;
    unsafe { core::arch::x86_64::__rdtscp(&mut aux) }
}

#[cfg(not(target_arch = "x86_64"))]
#[inline(always)]
pub fn rdtscp() -> u64 {
    rdtsc()
}

/// Serializing fence before timing measurement
#[cfg(target_arch = "x86_64")]
#[inline(always)]
pub fn serialize() {
    unsafe {
        core::arch::x86_64::_mm_mfence();
        core::arch::x86_64::_mm_lfence();
    }
}

#[cfg(not(target_arch = "x86_64"))]
#[inline(always)]
pub fn serialize() {
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
}

/// Calibration result for RDTSC
#[derive(Debug, Clone, Copy)]
pub struct RdtscCalibration {
    /// CPU frequency in Hz
    pub freq_hz: f64,
    /// Nanoseconds per cycle
    pub ns_per_cycle: f64,
    /// Cycles per nanosecond
    pub cycles_per_ns: f64,
    /// Overhead of RDTSC itself (cycles)
    pub overhead_cycles: u64,
}

impl RdtscCalibration {
    /// Convert cycles to nanoseconds
    #[inline(always)]
    pub fn cycles_to_ns(&self, cycles: u64) -> u64 {
        ((cycles as f64) * self.ns_per_cycle) as u64
    }

    /// Convert nanoseconds to cycles
    #[inline(always)]
    pub fn ns_to_cycles(&self, ns: u64) -> u64 {
        ((ns as f64) * self.cycles_per_ns) as u64
    }
}

/// Calibrate RDTSC by measuring against system clock
///
/// Measures over `duration_ms` milliseconds for accuracy.
/// Returns calibration data for cycle-to-time conversion.
pub fn calibrate_rdtsc(duration_ms: u64) -> RdtscCalibration {
    let duration = Duration::from_millis(duration_ms);

    // Measure RDTSC overhead
    let overhead = measure_rdtsc_overhead();

    // Measure frequency
    serialize();
    let start_cycles = rdtsc();
    let start_time = Instant::now();

    // Busy wait for calibration period
    while start_time.elapsed() < duration {
        std::hint::spin_loop();
    }

    let end_cycles = rdtsc();
    serialize();
    let elapsed = start_time.elapsed();

    let cycles = end_cycles.wrapping_sub(start_cycles);
    let elapsed_ns = elapsed.as_nanos() as f64;

    let freq_hz = (cycles as f64) / elapsed.as_secs_f64();
    let ns_per_cycle = elapsed_ns / (cycles as f64);
    let cycles_per_ns = (cycles as f64) / elapsed_ns;

    RdtscCalibration {
        freq_hz,
        ns_per_cycle,
        cycles_per_ns,
        overhead_cycles: overhead,
    }
}

/// Measure the overhead of our timing pattern: serialize() + rdtsc() + rdtscp()
///
/// This measures the EXACT pattern used in benchmarks:
///   serialize();
///   let start = rdtsc();
///   // <work goes here - we measure with nothing>
///   let end = rdtscp();
///   end - start = overhead
///
/// The result is the cycles consumed by the timing infrastructure itself.
fn measure_rdtsc_overhead() -> u64 {
    let iterations = 10000;
    let mut min_overhead = u64::MAX;

    // Warmup
    for _ in 0..1000 {
        serialize();
        let start = rdtsc();
        let end = rdtscp();
        let _ = end.wrapping_sub(start);
    }

    // Measure minimum overhead (represents no cache misses, best case)
    for _ in 0..iterations {
        serialize();
        let start = rdtsc();
        // No work - measuring pure timing overhead
        let end = rdtscp();

        let overhead = end.wrapping_sub(start);
        if overhead < min_overhead {
            min_overhead = overhead;
        }
    }

    min_overhead
}

/// Convert cycles to nanoseconds using given calibration
#[inline(always)]
pub fn cycles_to_ns(cycles: u64, calibration: &RdtscCalibration) -> u64 {
    calibration.cycles_to_ns(cycles)
}

/// High-precision timer for benchmark measurements
pub struct PrecisionTimer {
    calibration: RdtscCalibration,
}

impl PrecisionTimer {
    /// Create a new precision timer with fresh calibration
    pub fn new() -> Self {
        Self {
            calibration: calibrate_rdtsc(100), // 100ms calibration
        }
    }

    /// Create with custom calibration duration
    pub fn with_calibration(duration_ms: u64) -> Self {
        Self {
            calibration: calibrate_rdtsc(duration_ms),
        }
    }

    /// Get calibration data
    pub fn calibration(&self) -> &RdtscCalibration {
        &self.calibration
    }

    /// Start a measurement, returning cycle count
    #[inline(always)]
    pub fn start(&self) -> u64 {
        serialize();
        rdtsc()
    }

    /// End a measurement and return elapsed nanoseconds
    #[inline(always)]
    pub fn elapsed_ns(&self, start_cycles: u64) -> u64 {
        let end = rdtscp();
        let cycles = end.wrapping_sub(start_cycles);
        self.calibration
            .cycles_to_ns(cycles.saturating_sub(self.calibration.overhead_cycles))
    }

    /// Measure a closure and return elapsed nanoseconds
    #[inline(always)]
    pub fn measure<F, R>(&self, f: F) -> (R, u64)
    where
        F: FnOnce() -> R,
    {
        let start = self.start();
        let result = std::hint::black_box(f());
        let elapsed = self.elapsed_ns(start);
        (result, elapsed)
    }

    /// Measure a closure N times and return all latencies
    pub fn measure_n<F>(&self, iterations: usize, mut f: F) -> Vec<u64>
    where
        F: FnMut(),
    {
        let mut latencies = Vec::with_capacity(iterations);

        for _ in 0..iterations {
            let start = self.start();
            f();
            std::hint::black_box(());
            latencies.push(self.elapsed_ns(start));
        }

        latencies
    }
}

impl Default for PrecisionTimer {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple timing guard for scoped measurements
pub struct TimingGuard<'a> {
    timer: &'a PrecisionTimer,
    start_cycles: u64,
    result: &'a mut u64,
}

impl<'a> TimingGuard<'a> {
    /// Create a new timing guard
    pub fn new(timer: &'a PrecisionTimer, result: &'a mut u64) -> Self {
        Self {
            timer,
            start_cycles: timer.start(),
            result,
        }
    }
}

impl<'a> Drop for TimingGuard<'a> {
    fn drop(&mut self) {
        *self.result = self.timer.elapsed_ns(self.start_cycles);
    }
}

/// Instant-based timer for platforms without RDTSC or when cycle counting isn't needed
pub struct InstantTimer;

impl InstantTimer {
    /// Measure a closure using std::time::Instant
    #[inline(always)]
    pub fn measure<F, R>(f: F) -> (R, u64)
    where
        F: FnOnce() -> R,
    {
        let start = Instant::now();
        let result = std::hint::black_box(f());
        let elapsed = start.elapsed().as_nanos() as u64;
        (result, elapsed)
    }

    /// Measure a closure N times
    pub fn measure_n<F>(iterations: usize, mut f: F) -> Vec<u64>
    where
        F: FnMut(),
    {
        let mut latencies = Vec::with_capacity(iterations);

        for _ in 0..iterations {
            let start = Instant::now();
            f();
            std::hint::black_box(());
            latencies.push(start.elapsed().as_nanos() as u64);
        }

        latencies
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rdtsc_monotonic() {
        let a = rdtsc();
        std::thread::sleep(Duration::from_micros(1));
        let b = rdtsc();
        assert!(b > a, "RDTSC should be monotonically increasing");
    }

    #[test]
    fn test_calibration() {
        let cal = calibrate_rdtsc(50); // 50ms calibration

        // Sanity checks
        assert!(
            cal.freq_hz > 100_000_000.0,
            "Frequency too low: {}",
            cal.freq_hz
        );
        assert!(
            cal.freq_hz < 10_000_000_000.0,
            "Frequency too high: {}",
            cal.freq_hz
        );
        assert!(cal.ns_per_cycle > 0.0);
        assert!(cal.cycles_per_ns > 0.0);
    }

    #[test]
    fn test_precision_timer() {
        let timer = PrecisionTimer::new();

        // Measure a sleep
        let (_, elapsed) = timer.measure(|| {
            std::thread::sleep(Duration::from_millis(1));
        });

        // Should be around 1ms = 1_000_000 ns, with some tolerance
        assert!(
            elapsed > 500_000 && elapsed < 10_000_000,
            "Unexpected elapsed time: {} ns",
            elapsed
        );
    }

    #[test]
    fn test_measure_n() {
        let timer = PrecisionTimer::new();

        // Use spin-wait to ensure measurable time (at least a few hundred ns)
        // This avoids sleep() overhead while being reliably measurable
        let latencies = timer.measure_n(20, || {
            let start = std::time::Instant::now();
            while start.elapsed().as_nanos() < 100 {
                std::hint::spin_loop();
            }
        });

        assert_eq!(latencies.len(), 20);

        // All measurements should be positive since we spin for at least 100ns
        let positive_count = latencies.iter().filter(|&&l| l > 0).count();
        assert!(
            positive_count == 20,
            "Expected all 20 measurements positive, got {}/20",
            positive_count
        );

        // Each measurement should be at least ~100ns
        let min_latency = *latencies.iter().min().unwrap();
        assert!(
            min_latency >= 50, // Allow some variance
            "Minimum latency {} ns is too low",
            min_latency
        );
    }

    #[test]
    fn test_instant_timer() {
        let (_, elapsed) = InstantTimer::measure(|| {
            std::thread::sleep(Duration::from_millis(1));
        });

        assert!(
            elapsed > 500_000 && elapsed < 10_000_000,
            "Unexpected elapsed time: {} ns",
            elapsed
        );
    }
}
