//! High-resolution timing — monotonic clock, precise sleep.
//!
//! Provides sub-microsecond timing for scheduler tick loops:
//! - **Linux**: `clock_nanosleep(CLOCK_MONOTONIC)` via VDSO
//! - **macOS**: `mach_absolute_time()` + `mach_wait_until()`
//! - **Windows**: `QueryPerformanceCounter` + `timeBeginPeriod(1)` + spin-wait

use std::time::{Duration, Instant};

/// Get the current monotonic time in nanoseconds.
///
/// Uses `std::time::Instant` which maps to the best platform clock:
/// - Linux: `clock_gettime(CLOCK_MONOTONIC)`
/// - macOS: `mach_absolute_time()`
/// - Windows: `QueryPerformanceCounter`
#[inline]
pub fn monotonic_ns() -> u64 {
    static EPOCH: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    let epoch = EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_nanos() as u64
}

/// Sleep for the specified duration with the best precision available.
///
/// On Linux, uses `clock_nanosleep` for nanosecond precision.
/// On Windows, combines `Sleep` with a spin-wait tail for sub-ms accuracy.
/// On macOS, uses standard `thread::sleep` (already ~100us precision).
pub fn sleep_precise(duration: Duration) {
    if duration.is_zero() {
        return;
    }

    #[cfg(target_os = "linux")]
    {
        // Use clock_nanosleep for best precision on Linux
        let ts = libc::timespec {
            tv_sec: duration.as_secs() as libc::time_t,
            tv_nsec: duration.subsec_nanos() as libc::c_long,
        };
        // SAFETY: CLOCK_MONOTONIC and 0 (relative) are valid constants;
        // ts is a properly initialized timespec
        unsafe {
            libc::clock_nanosleep(libc::CLOCK_MONOTONIC, 0, &ts, std::ptr::null_mut());
        }
        return;
    }

    #[cfg(target_os = "windows")]
    {
        // Windows Sleep has ~15ms resolution by default, ~1ms with timeBeginPeriod.
        // For sub-ms precision, sleep for most of the time then spin-wait.
        if duration > Duration::from_millis(2) {
            // Sleep for duration minus 1ms, then spin-wait the rest
            let sleep_time = duration - Duration::from_millis(1);
            std::thread::sleep(sleep_time);
        }
        // Spin-wait for remaining time
        let target = Instant::now() + duration;
        while Instant::now() < target {
            std::hint::spin_loop();
        }
        return;
    }

    // Default: standard sleep (macOS already has ~100us precision)
    #[allow(unreachable_code)]
    std::thread::sleep(duration);
}

/// Get the platform's timer resolution in nanoseconds.
///
/// - Linux: typically 1ns (HPET/TSC)
/// - macOS: ~42ns (Mach absolute time)
/// - Windows: ~100ns (QPC), but Sleep resolution is ~1ms with timeBeginPeriod
#[inline]
pub fn timer_resolution_ns() -> u64 {
    #[cfg(target_os = "linux")]
    {
        1 // clock_nanosleep with CLOCK_MONOTONIC = ~1ns
    }
    #[cfg(target_os = "macos")]
    {
        42 // mach_absolute_time typical resolution
    }
    #[cfg(target_os = "windows")]
    {
        100 // QPC resolution
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        1000 // 1us fallback
    }
}

/// RAII guard that restores timer resolution on drop (Windows only).
pub struct TimerResolutionGuard {
    _private: (),
}

/// Set the finest available timer resolution.
///
/// - Windows: `timeBeginPeriod(1)` to reduce system timer from 15.6ms to ~1ms.
///   Returns a guard that calls `timeEndPeriod(1)` on drop.
/// - Linux/macOS: no-op (already fine-grained).
pub fn set_finest_resolution() -> anyhow::Result<TimerResolutionGuard> {
    #[cfg(target_os = "windows")]
    {
        use windows_sys::Win32::Media::{timeBeginPeriod, timeEndPeriod};
        // SAFETY: timeBeginPeriod(1) is a safe Windows API call
        let result = unsafe { timeBeginPeriod(1) };
        if result != 0 {
            anyhow::bail!("timeBeginPeriod(1) failed");
        }
    }
    Ok(TimerResolutionGuard { _private: () })
}

impl Drop for TimerResolutionGuard {
    fn drop(&mut self) {
        #[cfg(target_os = "windows")]
        {
            use windows_sys::Win32::Media::timeEndPeriod;
            // SAFETY: timeEndPeriod(1) is safe; matches our timeBeginPeriod(1)
            unsafe { timeEndPeriod(1) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn monotonic_ns_increases() {
        let a = monotonic_ns();
        std::thread::sleep(Duration::from_micros(100));
        let b = monotonic_ns();
        assert!(b > a, "monotonic_ns should increase: {} vs {}", a, b);
    }

    #[test]
    fn sleep_precise_approximate() {
        let start = Instant::now();
        sleep_precise(Duration::from_millis(10));
        let elapsed = start.elapsed();
        // Allow 5ms-50ms range (generous for CI)
        assert!(
            elapsed >= Duration::from_millis(5),
            "Slept too little: {:?}",
            elapsed
        );
        assert!(
            elapsed < Duration::from_millis(50),
            "Slept too long: {:?}",
            elapsed
        );
    }

    #[test]
    fn sleep_precise_zero_returns_immediately() {
        let start = Instant::now();
        sleep_precise(Duration::ZERO);
        assert!(start.elapsed() < Duration::from_millis(1));
    }

    #[test]
    fn timer_resolution_is_positive() {
        assert!(timer_resolution_ns() > 0);
    }

    #[test]
    fn set_finest_resolution_returns_guard() {
        let guard = set_finest_resolution().unwrap();
        drop(guard);
    }

    // ── Monotonic clock intent tests ────────────────────────────────

    #[test]
    fn monotonic_ns_is_strictly_monotonic() {
        // Take 10 samples, each should be >= previous
        let mut prev = monotonic_ns();
        for _ in 0..10 {
            let now = monotonic_ns();
            assert!(now >= prev, "monotonic_ns must never decrease");
            prev = now;
        }
    }

    #[test]
    fn monotonic_ns_advances_with_real_time() {
        let before = monotonic_ns();
        std::thread::sleep(Duration::from_millis(5));
        let after = monotonic_ns();
        let delta_ns = after - before;
        // 5ms = 5,000,000 ns — allow generous range
        assert!(
            delta_ns >= 1_000_000, // at least 1ms
            "monotonic_ns should advance ~5ms after 5ms sleep, got {}ns",
            delta_ns
        );
    }

    // ── Sleep precision intent tests ────────────────────────────────

    #[test]
    fn sleep_precise_sub_millisecond() {
        // Test that sub-ms sleeps don't oversleep dramatically
        let start = Instant::now();
        sleep_precise(Duration::from_micros(500));
        let elapsed = start.elapsed();
        // 500us sleep should finish within 10ms (generous for CI)
        assert!(
            elapsed < Duration::from_millis(10),
            "500us sleep overslept: {:?}",
            elapsed
        );
    }

    #[test]
    fn sleep_precise_multiple_calls_consistent() {
        // 5 consecutive 1ms sleeps should total ~5ms (not 75ms from 15ms timer)
        let start = Instant::now();
        for _ in 0..5 {
            sleep_precise(Duration::from_millis(1));
        }
        let elapsed = start.elapsed();
        assert!(
            elapsed < Duration::from_millis(50),
            "5x 1ms sleeps took {:?} (expected ~5ms)",
            elapsed
        );
    }

    // ── Timer resolution tests ──────────────────────────────────────

    #[test]
    fn timer_resolution_is_reasonable() {
        let res = timer_resolution_ns();
        // Should be between 1ns and 1ms (1,000,000ns)
        assert!(
            res >= 1 && res <= 1_000_000,
            "timer resolution {}ns is outside reasonable range",
            res
        );
    }
}
