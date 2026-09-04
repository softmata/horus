#![allow(dead_code)]
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

// ============================================================================
// Pool identity
// ============================================================================
//
// Every test in this file gets its OWN base id and they all use the SAME
// `PID_SPREAD` modulus, so no two tests in this binary can ever derive the same
// pool id.
//
// They used to. `stress_tensorpool_alternating_free_pattern` used
// `9900 + pid % 100` and `stress_tensorpool_rapid_slot_churn` used
// `9950 + pid % 50`. For every pid where `pid % 100 >= 50` — half of all runs —
// those are the *same number*: with `p = pid % 100 >= 50`, `p % 50 == p - 50`,
// so `9950 + (p - 50) == 9900 + p`. libtest runs tests in name order, so
// `alternating` went first, created the pool with a 32 MiB / 128-slot geometry,
// and left the file behind — `cleanup_stale_shm()` only wipes topics/ and
// nodes/, `TensorPool`'s `Drop` deliberately does not unlink, so nothing
// removed it. `rapid_slot_churn` then asked for 64 MiB / 32 slots at that same
// id and `TensorPool::new` correctly refused to resize a pool it did not
// create:
//
//     Failed to create pool: Config(Other("Tensor pool 9989 exists but is only
//     33561664 bytes, smaller than the 67110720 bytes this process needs"))
//
// The two byte counts are the two geometries exactly, which is what pins the
// diagnosis to this collision rather than to a stale file from another run:
//   33561664 = 64 (PoolHeader) + 128 * 56 (SlotHeader) + 32 MiB  <- alternating
//   67110720 = 64 (PoolHeader) +  32 * 56 (SlotHeader) + 64 MiB  <- churn
// and 9989 is reachable by both formulas only at `pid % 100 == 89`.
//
// Bases are 1000 apart and live in an id range no other test file uses. The
// rest of the map, for anyone adding a pool: tensor_pool_concurrent.rs and
// resource_exhaustion.rs share the pid-spread band 20_000-20_599 (bases in
// tests/common, enforced by `pool_id_ranges_cannot_overlap`); regressions.rs
// pins 9800 and 9801; memory_coverage.rs holds 99990/99999; and
// horus_core/src/memory/tensor_pool.rs's unit tests take most fixed ids in
// 9600-10105.

/// Width of the pid-derived spread. Keeps concurrently running *processes* off
/// each other's pools; `clear_stale_pool` covers the leftovers of dead ones.
const PID_SPREAD: u32 = 1000;

const POOL_BASE_MULTITHREAD: u32 = 30_000;
const POOL_BASE_ALTERNATING: u32 = 31_000;
const POOL_BASE_CHURN: u32 = 32_000;
/// Reserved 33_000..35_000: this test adds a per-cycle offset on top of the
/// spread, so its band is two wide.
const POOL_BASE_LIFECYCLE: u32 = 33_000;

/// Pool id for one test: its own base plus this process's spread.
fn test_pool_id(base: u32) -> u32 {
    base + (std::process::id() % PID_SPREAD)
}

/// Path of a pool's backing SHM file.
fn pool_shm_path(pool_id: u32) -> std::path::PathBuf {
    shm_base_dir()
        .join("tensors")
        .join(format!("tensor_pool_{}", pool_id))
}

/// Remove a leftover pool file *before* creating, so this test creates rather
/// than attaches.
///
/// A run that panicked or was killed leaves its SHM file behind (`Drop` does not
/// unlink, and `cleanup_stale_shm()` only clears topics/ and nodes/). If a later
/// run's pid lands on the same spread slot, `TensorPool::new` attaches to that
/// stale file instead — and then either adopts a geometry the test did not ask
/// for, or fails outright when the leftover is smaller than the mapping this
/// config needs. Same idiom as `clear_stale_pool` in tensor_pool.rs's own tests.
fn clear_stale_pool(pool_id: u32) {
    let _ = std::fs::remove_file(pool_shm_path(pool_id));
}

/// Drop a pool's backing file once the test is done with it.
///
/// Targeted on purpose: the previous helper here did `remove_dir_all` on the
/// whole `tensors/` directory, which also destroys pools belonging to any other
/// test binary running at the same time.
fn remove_pool_file(pool_id: u32) {
    let _ = std::fs::remove_file(pool_shm_path(pool_id));
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
/// Creates a fresh pool per cycle over 60 seconds and hammers it with
/// alloc/free churn, checking that allocation latency stays consistent as
/// pools are created and destroyed repeatedly.
///
/// Each cycle is time-boxed. The loop used to end only on data-region
/// exhaustion, which was sound when the pool was a pure bump allocator that
/// never reclaimed space. Now that `release` returns space to the pool, this
/// workload holds ~16-32 tensors live and never exhausts — so exhaustion
/// alone is not a termination condition and the test ran forever (it is what
/// pinned the CI `Test` and MSRV jobs at the 6-hour ceiling). Exhaustion is
/// still honoured as an early exit for configurations that do hit it.
#[test]
fn stress_tensorpool_repeated_lifecycle_60s() {
    let _shm_guard = cleanup_stale_shm();

    let test_duration = Duration::from_secs(60);
    // Time-box each cycle so the 60s budget yields several lifecycles
    // (the assertions below want at least 3).
    let cycle_budget = Duration::from_secs(5);
    let start = Instant::now();
    let mut cycle = 0;
    let mut all_stats: Vec<CycleStats> = Vec::new();

    while start.elapsed() < test_duration {
        // Unique pool ID per cycle — avoids stale SHM file collisions
        // `% PID_SPREAD` keeps the id inside the reserved 33_000..35_000 band
        // even if a create-failure loop spins `cycle` far past the ~12 a 60s
        // run normally reaches.
        let pool_id = test_pool_id(POOL_BASE_LIFECYCLE) + (cycle as u32 % PID_SPREAD);
        clear_stale_pool(pool_id);
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

        // Run alloc/free until this cycle's time slice is up (or, for pools
        // that can exhaust, until the data region is spent).
        let cycle_start = Instant::now();
        let mut consecutive_failures = 0;
        loop {
            if cycle_start.elapsed() >= cycle_budget {
                break;
            }

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
                    60..=84 => rng.range(256, 4096), // 25%: 1KB-16KB
                    _ => rng.range(4096, 16384),     // 15%: 16KB-64KB
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
        let _shm_guard = cleanup_stale_shm();
        remove_pool_file(pool_id);

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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = test_pool_id(POOL_BASE_MULTITHREAD);
    clear_stale_pool(pool_id);
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
        h.join()
            .expect("Thread panicked during multithread exhaust test");
    }

    let allocs = total_allocs.load(Ordering::SeqCst);
    let failures = total_failures.load(Ordering::SeqCst);
    let releases = total_releases.load(Ordering::SeqCst);
    let corruptions = data_corruptions.load(Ordering::SeqCst);

    remove_pool_file(pool_id);

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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = test_pool_id(POOL_BASE_ALTERNATING);
    clear_stale_pool(pool_id);
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
    eprintln!(
        "[alternating] Phase 2: freed {}, kept {}",
        freed,
        kept.len()
    );

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

    drop(pool);
    remove_pool_file(pool_id);

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
    let _shm_guard = cleanup_stale_shm();

    let pool_id = test_pool_id(POOL_BASE_CHURN);
    clear_stale_pool(pool_id);
    let config = TensorPoolConfig {
        pool_size: 64 * 1024 * 1024, // 64MB — generous for many small allocs
        max_slots: 32,               // few slots, high reuse
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

    drop(pool);
    remove_pool_file(pool_id);

    eprintln!(
        "[churn] Completed {}/{} alloc+release cycles",
        completed, churn_count
    );

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
        let first_quarter_avg: f64 = alloc_times_ns[..completed / 4]
            .iter()
            .map(|&x| x as f64)
            .sum::<f64>()
            / (completed / 4) as f64;
        let last_quarter_avg: f64 = alloc_times_ns[completed * 3 / 4..]
            .iter()
            .map(|&x| x as f64)
            .sum::<f64>()
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
