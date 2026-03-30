//! Stress test: TensorPool fragmentation and data region exhaustion.
//!
//! The TensorPool uses a **bump allocator** for the data region: `next_alloc_offset`
//! only moves forward. Freed slots return to the free-stack, but their data region
//! space is never reclaimed. Each successful alloc permanently consumes data space.
//!
//! This means total_data_consumed = sum of ALL successful allocs (not just live ones).
//! A pool will eventually exhaust its data region regardless of how many slots are freed.
//!
//! These tests verify:
//! 1. Free-stack slot recycling works correctly
//! 2. Data region exhaustion is handled gracefully (error, not panic)
//! 3. Alloc latency stays consistent as the pool fills
//! 4. Multi-threaded alloc/free causes no corruption
//! 5. Alternating free patterns don't break the free-stack

use horus_core::memory::{shm_base_dir, TensorPool, TensorPoolConfig};
use horus_core::types::{Device, TensorDtype};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::cleanup_stale_shm;

/// Clean up tensor pool SHM files (cleanup_stale_shm only removes topics/nodes).
fn cleanup_tensor_shm() {
    let tensors_dir = shm_base_dir().join("tensors");
    let _ = std::fs::remove_dir_all(&tensors_dir);
}

// ============================================================================
// Helpers
// ============================================================================

/// Simple deterministic PRNG (xorshift64).
struct FastRng {
    state: u64,
}

impl FastRng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn range(&mut self, lo: u64, hi: u64) -> u64 {
        lo + (self.next_u64() % (hi - lo))
    }
}

#[derive(Default)]
struct CycleStats {
    alloc_successes: u64,
    alloc_failures: u64,
    total_alloc_ns: u64,
    worst_alloc_ns: u64,
}

impl CycleStats {
    fn avg_alloc_us(&self) -> f64 {
        if self.alloc_successes == 0 {
            return 0.0;
        }
        (self.total_alloc_ns as f64 / self.alloc_successes as f64) / 1_000.0
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Sustained alloc/free cycles across multiple pool lifecycles.
///
/// Since the bump allocator never reclaims data space, a single pool has a
/// finite number of successful allocs. This test creates multiple pools over
/// 60 seconds, running each to exhaustion and measuring consistency.
#[test]
fn stress_tensorpool_repeated_lifecycle_60s() {
    cleanup_stale_shm();

    let test_duration = Duration::from_secs(60);
    let start = Instant::now();
    let mut cycle = 0;
    let mut all_stats: Vec<CycleStats> = Vec::new();

    // Use PID + monotonic counter for unique pool IDs across runs
    let pid_base = (std::process::id() % 10000) as u32;

    while start.elapsed() < test_duration {
        // Unique pool ID per cycle — avoids stale SHM file collisions
        let pool_id = 10000 + pid_base + cycle as u32;
        let config = TensorPoolConfig {
            pool_size: 32 * 1024 * 1024, // 32MB
            max_slots: 128,
            slot_alignment: 64,
            ..Default::default()
        };

        let pool = match TensorPool::new(pool_id, config) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("[cycle {}] Failed to create pool: {}", cycle, e);
                cycle += 1;
                continue;
            }
        };

        let mut stats = CycleStats::default();
        let mut rng = FastRng::new(42 + cycle as u64);
        let mut live_tensors = Vec::with_capacity(128);

        // Run alloc/free until data region exhausts
        let mut consecutive_failures = 0;
        loop {
            // Maintain ~16-32 live tensors
            let fill = live_tensors.len();
            let should_free = fill > 0 && (fill > 32 || rng.range(0, 100) < 50);

            if should_free {
                let idx = rng.range(0, live_tensors.len() as u64) as usize;
                let tensor = live_tensors.swap_remove(idx);
                pool.release(&tensor);
            } else {
                // Random small-medium tensor sizes (64B to 64KB)
                let size = match rng.range(0, 100) {
                    0..=59 => rng.range(16, 256),    // 60%: 64B-1KB
                    60..=84 => rng.range(256, 4096),  // 25%: 1KB-16KB
                    _ => rng.range(4096, 16384),       // 15%: 16KB-64KB
                };

                let alloc_start = Instant::now();
                match pool.alloc(&[size], TensorDtype::U8, Device::cpu()) {
                    Ok(tensor) => {
                        let ns = alloc_start.elapsed().as_nanos() as u64;
                        stats.alloc_successes += 1;
                        stats.total_alloc_ns += ns;
                        if ns > stats.worst_alloc_ns {
                            stats.worst_alloc_ns = ns;
                        }

                        // Verify writability
                        if let Ok(data) = pool.data_slice_mut(&tensor) {
                            if !data.is_empty() {
                                data[0] = 0xAB;
                            }
                        }
                        live_tensors.push(tensor);
                        consecutive_failures = 0;
                    }
                    Err(_) => {
                        stats.alloc_failures += 1;
                        consecutive_failures += 1;
                        // Free some tensors to reclaim slots (not data space)
                        if let Some(t) = live_tensors.pop() {
                            pool.release(&t);
                        }
                        // Data region is exhausted — move to next pool
                        if consecutive_failures > 50 {
                            break;
                        }
                    }
                }
            }
        }

        // Clean up remaining live tensors
        for t in &live_tensors {
            pool.release(t);
        }

        let final_stats = pool.stats();
        eprintln!(
            "[cycle {}] {} allocs, {} failures, avg: {:.1}us, worst: {:.1}us, data used: {:.1}MB",
            cycle,
            stats.alloc_successes,
            stats.alloc_failures,
            stats.avg_alloc_us(),
            stats.worst_alloc_ns as f64 / 1_000.0,
            final_stats.used_bytes as f64 / (1024.0 * 1024.0)
        );

        // Clean up ALL SHM files (including tensor pool) before next cycle
        drop(pool); // ensure mmap is unmapped before deleting backing file
        cleanup_stale_shm();
        cleanup_tensor_shm();

        all_stats.push(stats);
        cycle += 1;
    }

    eprintln!("[summary] {} pool lifecycles in 60s", cycle);

    // === Assertions ===

    // Should complete multiple cycles (proves repeated creation works)
    assert!(
        cycle >= 3,
        "Expected at least 3 pool lifecycles in 60s, got {}",
        cycle
    );

    // Most cycles should have many successful allocs
    // (a few may hit stale SHM files and exhaust immediately)
    let good_cycles = all_stats
        .iter()
        .filter(|s| s.alloc_successes >= 100)
        .count();
    assert!(
        good_cycles as f64 / all_stats.len() as f64 > 0.90,
        "Only {}/{} cycles had >=100 successful allocs (expected >90%)",
        good_cycles,
        all_stats.len()
    );

    // Alloc latency shouldn't degrade dramatically across cycles
    if all_stats.len() >= 2 {
        let first_avg = all_stats[0].avg_alloc_us();
        let last_avg = all_stats.last().unwrap().avg_alloc_us();
        if first_avg > 0.0 {
            let ratio = last_avg / first_avg;
            eprintln!(
                "[degradation] first cycle avg: {:.1}us, last: {:.1}us, ratio: {:.1}x",
                first_avg, last_avg, ratio
            );
            assert!(
                ratio < 20.0,
                "Alloc latency degraded {:.1}x across cycles",
                ratio
            );
        }
    }
}

/// Multi-threaded pool exhaustion: 4 threads race to fill a single pool.
/// Verifies no corruption, panics, or data races under contention.
#[test]
fn stress_tensorpool_multithread_exhaust() {
    cleanup_stale_shm();

    let pool_id = 9860 + (std::process::id() % 100) as u32;
    let config = TensorPoolConfig {
        pool_size: 16 * 1024 * 1024, // 16MB
        max_slots: 64,
        slot_alignment: 64,
        ..Default::default()
    };
    let pool = Arc::new(TensorPool::new(pool_id, config).expect("Failed to create pool"));

    let thread_count = 4;
    let running = Arc::new(AtomicBool::new(true));
    let total_allocs = Arc::new(AtomicU64::new(0));
    let total_failures = Arc::new(AtomicU64::new(0));
    let total_releases = Arc::new(AtomicU64::new(0));
    let data_corruptions = Arc::new(AtomicU64::new(0));

    let mut handles = Vec::new();
    for t in 0..thread_count {
        let pool = pool.clone();
        let running = running.clone();
        let allocs = total_allocs.clone();
        let failures = total_failures.clone();
        let releases = total_releases.clone();
        let corruptions = data_corruptions.clone();

        handles.push(std::thread::spawn(move || {
            let mut rng = FastRng::new(42 + t as u64 * 7919);
            let mut local_tensors = Vec::with_capacity(32);
            let marker = (t & 0xFF) as u8;

            while running.load(Ordering::Relaxed) {
                let fill = local_tensors.len();
                let should_free = fill > 0 && (fill > 16 || rng.range(0, 100) < 60);

                if should_free {
                    let idx = rng.range(0, local_tensors.len() as u64) as usize;
                    let tensor = local_tensors.swap_remove(idx);

                    // Verify marker before release
                    if let Ok(data) = pool.data_slice(&tensor) {
                        if !data.is_empty() && data[0] != marker {
                            corruptions.fetch_add(1, Ordering::Relaxed);
                        }
                    }

                    pool.release(&tensor);
                    releases.fetch_add(1, Ordering::Relaxed);
                } else {
                    let size = rng.range(64, 4096); // 64B - 4KB
                    match pool.alloc(&[size], TensorDtype::U8, Device::cpu()) {
                        Ok(tensor) => {
                            // Write thread-specific marker
                            if let Ok(data) = pool.data_slice_mut(&tensor) {
                                if !data.is_empty() {
                                    data[0] = marker;
                                }
                            }
                            local_tensors.push(tensor);
                            allocs.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(_) => {
                            failures.fetch_add(1, Ordering::Relaxed);
                            // Free some to relieve slot pressure
                            if let Some(t) = local_tensors.pop() {
                                pool.release(&t);
                                releases.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    }
                }
            }

            // Cleanup
            for t in &local_tensors {
                pool.release(t);
            }
        }));
    }

    // Run for 15 seconds
    std::thread::sleep(Duration::from_secs(15));
    running.store(false, Ordering::SeqCst);

    for h in handles {
        h.join().expect("Thread panicked during multithread exhaust test");
    }

    let allocs = total_allocs.load(Ordering::SeqCst);
    let failures = total_failures.load(Ordering::SeqCst);
    let releases = total_releases.load(Ordering::SeqCst);
    let corruptions = data_corruptions.load(Ordering::SeqCst);

    eprintln!(
        "[multithread] allocs: {}, failures: {}, releases: {}, corruptions: {}",
        allocs, failures, releases, corruptions
    );

    // No data corruption
    assert_eq!(
        corruptions, 0,
        "Detected {} data corruption events across 4 threads",
        corruptions
    );

    // Should have done substantial work
    assert!(
        allocs >= 50,
        "Expected at least 50 successful allocs across 4 threads, got {}",
        allocs
    );

    // No thread panics (handled by join above)
}

/// Worst-case slot fragmentation: fill all slots, free every other one,
/// reallocate. Tests that the free-stack recycles slots correctly even
/// after a fragmented free pattern.
#[test]
fn stress_tensorpool_alternating_free_pattern() {
    cleanup_stale_shm();

    let pool_id = 9900 + (std::process::id() % 100) as u32;
    let config = TensorPoolConfig {
        pool_size: 32 * 1024 * 1024, // 32MB
        max_slots: 128,
        slot_alignment: 64,
        ..Default::default()
    };
    let pool = TensorPool::new(pool_id, config).expect("Failed to create pool");

    // Phase 1: Fill all slots with tiny tensors (to conserve data space)
    let mut tensors = Vec::with_capacity(128);
    for _ in 0..128 {
        match pool.alloc(&[64], TensorDtype::U8, Device::cpu()) {
            Ok(t) => tensors.push(t),
            Err(_) => break,
        }
    }
    let initial_count = tensors.len();
    eprintln!(
        "[alternating] Phase 1: allocated {} small tensors",
        initial_count
    );

    // Phase 2: Free every other tensor (fragmented free list)
    let mut freed = 0;
    let mut kept = Vec::new();
    for (i, t) in tensors.into_iter().enumerate() {
        if i % 2 == 0 {
            pool.release(&t);
            freed += 1;
        } else {
            kept.push(t);
        }
    }
    eprintln!("[alternating] Phase 2: freed {}, kept {}", freed, kept.len());

    // Phase 3: Allocate into the freed slots — slots should recycle,
    // but each alloc still consumes NEW data space (bump allocator)
    let mut reallocated = 0;
    for _ in 0..freed {
        match pool.alloc(&[64], TensorDtype::U8, Device::cpu()) {
            Ok(t) => {
                // Verify we can write to the reallocated slot
                if let Ok(data) = pool.data_slice_mut(&t) {
                    if !data.is_empty() {
                        data[0] = 0xCD;
                    }
                }
                pool.release(&t);
                reallocated += 1;
            }
            Err(_) => break,
        }
    }
    eprintln!(
        "[alternating] Phase 3: reallocated {}/{} freed slots",
        reallocated, freed
    );

    // Clean up
    for t in &kept {
        pool.release(t);
    }

    let stats = pool.stats();
    eprintln!(
        "[alternating] Final: {} allocated, {:.1}KB used",
        stats.allocated_slots,
        stats.used_bytes as f64 / 1024.0
    );

    // All freed slots should be reclaimable (if data space allows)
    assert!(
        reallocated >= freed / 2,
        "Only reallocated {}/{} freed slots — free-stack not reclaiming properly",
        reallocated,
        freed
    );
}

/// Rapid slot churn: allocate and immediately release, measuring
/// how the free-stack handles high-frequency recycling.
#[test]
fn stress_tensorpool_rapid_slot_churn() {
    cleanup_stale_shm();

    let pool_id = 9950 + (std::process::id() % 50) as u32;
    let config = TensorPoolConfig {
        pool_size: 64 * 1024 * 1024, // 64MB — generous for many small allocs
        max_slots: 32,                // few slots, high reuse
        slot_alignment: 64,
        ..Default::default()
    };
    let pool = TensorPool::new(pool_id, config).expect("Failed to create pool");

    let churn_count = 10_000;
    let mut alloc_times_ns = Vec::with_capacity(churn_count);

    for _ in 0..churn_count {
        let start = Instant::now();
        match pool.alloc(&[128], TensorDtype::U8, Device::cpu()) {
            Ok(tensor) => {
                alloc_times_ns.push(start.elapsed().as_nanos() as u64);
                pool.release(&tensor);
            }
            Err(_) => {
                // Data region exhausted — expected with bump allocator
                break;
            }
        }
    }

    let completed = alloc_times_ns.len();
    eprintln!("[churn] Completed {}/{} alloc+release cycles", completed, churn_count);

    if completed > 100 {
        alloc_times_ns.sort();
        let p50 = alloc_times_ns[completed / 2];
        let p99 = alloc_times_ns[(completed as f64 * 0.99) as usize];
        let worst = alloc_times_ns[completed - 1];

        eprintln!(
            "[churn] Alloc latency: p50={:.1}us, p99={:.1}us, worst={:.1}us",
            p50 as f64 / 1_000.0,
            p99 as f64 / 1_000.0,
            worst as f64 / 1_000.0
        );

        // p99 should be under 1ms (alloc is lock-free CAS)
        assert!(
            p99 < 1_000_000,
            "p99 alloc latency {}us exceeds 1ms — free-stack contention?",
            p99 / 1_000
        );

        // Verify no dramatic latency spikes in late allocations
        // (would indicate free-stack degradation as pool fills)
        let first_quarter_avg: f64 =
            alloc_times_ns[..completed / 4].iter().map(|&x| x as f64).sum::<f64>()
                / (completed / 4) as f64;
        let last_quarter_avg: f64 =
            alloc_times_ns[completed * 3 / 4..].iter().map(|&x| x as f64).sum::<f64>()
                / (completed / 4) as f64;

        if first_quarter_avg > 0.0 {
            let ratio = last_quarter_avg / first_quarter_avg;
            eprintln!(
                "[churn] Latency degradation: first quarter avg={:.0}ns, last quarter avg={:.0}ns, ratio={:.1}x",
                first_quarter_avg, last_quarter_avg, ratio
            );
            // Allow some degradation but not catastrophic
            assert!(
                ratio < 10.0,
                "Alloc latency degraded {:.1}x from first to last quarter",
                ratio
            );
        }
    }

    // Should complete a meaningful number of cycles before data exhaustion
    assert!(
        completed >= 100,
        "Only {} alloc+release cycles before exhaustion (expected >=100)",
        completed
    );
}
