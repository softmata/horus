//! Lock-free frame slot with version-dance (seqlock) protocol
//!
//! This module implements the core lock-free data structure for storing
//! per-frame transforms with history.

use std::cell::UnsafeCell;
use std::sync::atomic::{self, AtomicU32, AtomicU64, AtomicU8, Ordering};

use super::transform::Transform;

use super::types::{FrameId, FrameType, NO_PARENT};

/// A single transform entry with timestamp
#[derive(Clone, Copy, Default)]
#[repr(C)]
pub struct TransformEntry {
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
    /// The transform (translation + quaternion rotation)
    pub transform: Transform,
}

impl TransformEntry {
    /// Create a new transform entry
    pub fn new(transform: Transform, timestamp_ns: u64) -> Self {
        Self {
            timestamp_ns,
            transform,
        }
    }

    /// Create identity transform at given timestamp
    pub fn identity(timestamp_ns: u64) -> Self {
        Self {
            timestamp_ns,
            transform: Transform::identity(),
        }
    }
}

/// Lock-free frame slot with history ring buffer
///
/// Uses two seqlock (version-dance) counters for lock-free reads:
///
/// **`version`** — outer seqlock covering the entire slot (metadata + history):
/// - Odd = write in progress, Even = stable
/// - Readers spin-wait if odd, retry if the value changed across their read
///
/// **`history_version`** — inner seqlock covering the history ring-buffer only:
/// - Provides an explicit generation counter specifically for the multi-entry
///   history scan in `read_interpolated()` and `read_at()`.
/// - Catches ABA / re-order races where a concurrent writer completes a new
///   history entry between the reader's per-entry accesses, even if the outer
///   `version` momentarily returns to its starting parity.
///
/// Memory layout is cache-line aligned for optimal performance.
#[repr(C, align(64))]
pub struct FrameSlot {
    // === First cache line: hot read data ===
    /// Version counter for seqlock protocol
    /// Odd = write in progress, Even = stable
    version: AtomicU64,

    /// Sequence number (monotonically increasing write counter)
    sequence: AtomicU64,

    /// Parent frame ID
    parent: AtomicU32,

    /// Frame type (Static, Dynamic, Unallocated)
    frame_type: AtomicU8,

    /// Padding to align history to cache line
    _padding: [u8; 64 - 8 - 8 - 4 - 1],

    // === History buffer (separate cache lines) ===
    /// Ring buffer of transform history
    /// For static frames, only index 0 is used
    history: UnsafeCell<Vec<TransformEntry>>,

    /// History buffer capacity (set at creation, immutable)
    history_capacity: usize,

    /// Generation counter for the history ring-buffer.
    ///
    /// Incremented (to odd) before any history element is written and again
    /// (to even) after the write completes — matching the outer `version`
    /// seqlock pattern but scoped to history mutations only.
    ///
    /// `read_interpolated()` and `read_at()` load this before and after the
    /// multi-entry scan and retry if it changed, preventing torn reads of
    /// partially-overwritten history entries.
    history_version: AtomicU64,
}

// Safety: We guarantee thread-safety via the seqlock protocol
// Writers serialize via atomic version increment
// Readers retry on version mismatch
unsafe impl Sync for FrameSlot {}
unsafe impl Send for FrameSlot {}

impl FrameSlot {
    /// Create a new unallocated slot
    pub fn new(history_capacity: usize) -> Self {
        let history = vec![TransformEntry::default(); history_capacity];

        Self {
            version: AtomicU64::new(0),
            sequence: AtomicU64::new(0),
            parent: AtomicU32::new(NO_PARENT),
            frame_type: AtomicU8::new(FrameType::Unallocated as u8),
            _padding: [0; 64 - 8 - 8 - 4 - 1],
            history: UnsafeCell::new(history),
            history_capacity,
            history_version: AtomicU64::new(0),
        }
    }

    /// Initialize as a static frame
    pub fn init_static(&self, parent: FrameId) {
        self.parent.store(parent, Ordering::Release);
        self.frame_type
            .store(FrameType::Static as u8, Ordering::Release);
        self.sequence.store(0, Ordering::Release);
        self.version.store(0, Ordering::Release);
    }

    /// Initialize as a dynamic frame
    pub fn init_dynamic(&self, parent: FrameId) {
        self.parent.store(parent, Ordering::Release);
        self.frame_type
            .store(FrameType::Dynamic as u8, Ordering::Release);
        self.sequence.store(0, Ordering::Release);
        self.version.store(0, Ordering::Release);
    }

    /// Reset slot to unallocated state
    pub fn reset(&self) {
        // Mark as writing
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;

        self.parent.store(NO_PARENT, Ordering::Release);
        self.frame_type
            .store(FrameType::Unallocated as u8, Ordering::Release);
        self.sequence.store(0, Ordering::Release);

        let hv = self.history_version.fetch_add(1, Ordering::AcqRel) + 1;
        // SAFETY: Both the outer seqlock (`version`) and inner seqlock (`history_version`)
        // are held at an odd (write-in-progress) value; any concurrent reader that observes
        // an odd version will retry and will never commit a result that overlaps this
        // reset window. `self.history` is a UnsafeCell accessed exclusively here while
        // the write seqlock is held, so no aliased mutable references exist.
        unsafe {
            let history = &mut *self.history.get();
            for entry in history.iter_mut() {
                *entry = TransformEntry::default();
            }
        }
        self.history_version.store(hv + 1, Ordering::Release);

        // Mark as stable (outer seqlock)
        self.version.store(v + 1, Ordering::Release);
    }

    /// Check if this slot is allocated
    #[inline]
    pub fn is_allocated(&self) -> bool {
        self.frame_type.load(Ordering::Acquire) != FrameType::Unallocated as u8
    }

    /// Check if this is a static frame
    #[inline]
    pub fn is_static(&self) -> bool {
        self.frame_type.load(Ordering::Acquire) == FrameType::Static as u8
    }

    /// Get frame type
    #[inline]
    pub fn frame_type(&self) -> FrameType {
        FrameType::from(self.frame_type.load(Ordering::Acquire))
    }

    /// Get parent frame ID
    #[inline]
    pub fn parent(&self) -> FrameId {
        self.parent.load(Ordering::Acquire)
    }

    // ========================================================================
    // Writer Protocol
    // ========================================================================

    /// Update the frame's transform (lock-free write)
    ///
    /// # Thread Safety
    /// Reads are lock-free and safe to call concurrently with a single writer.
    /// **Requires: only one writer per frame at a time.** Concurrent writers on
    /// the same frame will corrupt the seqlock (fetch_add makes one writer's
    /// version even while it's still writing, causing readers to see torn data).
    /// The TransformFrame API enforces this by design: each frame has one logical owner.
    pub fn update(&self, transform: &Transform, timestamp_ns: u64) {
        // Step 1: Mark write in progress (odd version, outer seqlock)
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;
        debug_assert!(v & 1 == 1, "Version should be odd during write");

        // Step 2: Determine ring buffer slot
        let seq = self.sequence.fetch_add(1, Ordering::AcqRel);
        let idx = (seq as usize) % self.history_capacity;

        // Step 3: Mark history write in progress (odd history_version, inner seqlock)
        let hv = self.history_version.fetch_add(1, Ordering::AcqRel) + 1;
        debug_assert!(
            hv & 1 == 1,
            "history_version should be odd during history write"
        );

        // SAFETY: Both seqlocks are odd (write in progress); no reader will
        // commit a result that overlaps this write window.
        unsafe {
            let history = &mut *self.history.get();
            history[idx] = TransformEntry {
                timestamp_ns,
                transform: *transform,
            };
        }

        // Step 4: Mark history write complete (even history_version)
        self.history_version.store(hv + 1, Ordering::Release);

        // Step 5: Mark write complete (even version, outer seqlock)
        self.version.store(v + 1, Ordering::Release);
    }

    /// Set a static transform (only index 0 is used)
    pub fn set_static_transform(&self, transform: &Transform) {
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;
        let hv = self.history_version.fetch_add(1, Ordering::AcqRel) + 1;

        // SAFETY: Both seqlocks odd; readers will retry on mismatch.
        unsafe {
            let history = &mut *self.history.get();
            history[0] = TransformEntry {
                timestamp_ns: 0, // Static transforms have no timestamp
                transform: *transform,
            };
        }

        self.history_version.store(hv + 1, Ordering::Release);
        // For static frames, sequence stays at 1 to indicate "has data"
        self.sequence.store(1, Ordering::Release);
        self.version.store(v + 1, Ordering::Release);
    }

    // ========================================================================
    // Reader Protocol
    // ========================================================================

    /// Read the latest transform (lock-free read)
    ///
    /// Returns None if no transform has been written yet.
    ///
    /// # Seqlock invariant
    ///
    /// Both the initial (v1) and the re-check (v2) version loads **must** use
    /// `Ordering::Acquire`:
    ///
    /// - **v1 Acquire**: pairs with the writer's final `Release` store of the
    ///   even (stable) version.  This makes every data write that happened
    ///   before that store visible to us before we read any transform data.
    ///
    /// - **v2 Acquire**: ensures that, on weak-memory architectures (ARM64,
    ///   RISC-V), the CPU cannot reorder the v2 load to occur *before* our
    ///   data reads.  With `Relaxed` on v2 the hardware may speculatively
    ///   load the version, promote it past the data reads, and commit a
    ///   torn-read result even though v1 == v2 appeared to hold.
    ///
    /// Correct seqlock read pattern:
    /// ```text
    ///   v1 = load.Acquire(version)     // barrier: all writer stores visible
    ///   data = read(transform)          // inside the version pair
    ///   v2 = load.Acquire(version)     // barrier: data reads cannot float up
    ///   if v1 != v2 { retry }           // writer intervened — discard data
    /// ```
    pub fn read_latest(&self) -> Option<TransformEntry> {
        loop {
            // Step 1: Read version (must be even = stable).
            // Acquire pairs with writer's Release store of even version,
            // making all preceding data writes visible to this thread.
            let v1 = self.version.load(Ordering::Acquire);
            if v1 & 1 == 1 {
                // Writer in progress, spin
                std::hint::spin_loop();
                continue;
            }

            // Step 2: Read sequence
            let seq = self.sequence.load(Ordering::Acquire);
            if seq == 0 {
                return None; // Never written
            }

            // Step 3: Copy transform
            let entry = if self.is_static() {
                // Static frames always use index 0
                // SAFETY: Version was even (no write in progress). Value is validated by v2 == v1 check below.
                unsafe { (&*self.history.get())[0] }
            } else {
                let idx = ((seq - 1) as usize) % self.history_capacity;
                // SAFETY: Version was even (no write in progress). Value is validated by v2 == v1 check below.
                unsafe { (&*self.history.get())[idx] }
            };

            // Step 4: Verify version unchanged.
            // fence(Acquire) prevents v2 from floating above the raw data reads
            // on ARM64 (`dmb ishld`). This provides formal C++11 ordering
            // guarantees beyond those of the underlying UnsafeCell reads, and
            // is a no-op on x86 (TSO already provides this ordering).
            atomic::fence(Ordering::Acquire);
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return Some(entry);
            }
            // Version changed during our read, retry
        }
    }

    /// Read transform at (or nearest before) target timestamp
    pub fn read_at(&self, target_ts: u64) -> Option<TransformEntry> {
        // Static frames ignore timestamp
        if self.is_static() {
            return self.read_latest();
        }

        loop {
            // Outer seqlock: wait for a stable (even) slot version.
            let v1 = self.version.load(Ordering::Acquire);
            if v1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            let seq = self.sequence.load(Ordering::Acquire);
            if seq == 0 {
                return None;
            }

            // Inner seqlock: snapshot history_version before the history scan.
            // If a concurrent writer was mid-write, history_version is odd — retry.
            let hv1 = self.history_version.load(Ordering::Acquire);
            if hv1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            // SAFETY: Both seqlocks even (stable). Result validated by hv/v checks below.
            let result = unsafe { self.find_at_timestamp(&*self.history.get(), seq, target_ts) };

            // Inner check: history must not have changed during the scan.
            atomic::fence(Ordering::Acquire);
            let hv2 = self.history_version.load(Ordering::Acquire);
            if hv1 != hv2 {
                continue; // History mutated — discard and retry
            }

            // Outer check: slot version unchanged.
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return result;
            }
        }
    }

    /// Read transform with interpolation between two timestamps
    pub fn read_interpolated(&self, target_ts: u64) -> Option<Transform> {
        // Static frames ignore timestamp
        if self.is_static() {
            return self.read_latest().map(|e| e.transform);
        }

        loop {
            // Outer seqlock: wait for a stable (even) slot version.
            let v1 = self.version.load(Ordering::Acquire);
            if v1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            let seq = self.sequence.load(Ordering::Acquire);
            if seq == 0 {
                return None;
            }

            // Inner seqlock: snapshot history_version before the multi-entry scan.
            // A concurrent writer bumps history_version to odd before touching any
            // history element; reading an odd value means a write is in progress.
            let hv1 = self.history_version.load(Ordering::Acquire);
            if hv1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            // SAFETY: Both seqlocks even (stable). Result validated by hv/v checks below.
            let result =
                unsafe { self.interpolate_at_timestamp(&*self.history.get(), seq, target_ts) };

            // Inner check: history_version must be unchanged — guarantees the
            // multi-entry scan was not interrupted by a concurrent history write.
            atomic::fence(Ordering::Acquire);
            let hv2 = self.history_version.load(Ordering::Acquire);
            if hv1 != hv2 {
                continue; // History mutated during scan — discard and retry
            }

            // Outer check: slot version unchanged.
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return result;
            }
        }
    }

    // ========================================================================
    // Internal Helpers
    // ========================================================================

    /// Find entry at or before target timestamp
    ///
    /// # Safety
    ///
    /// `history` must be a valid reference obtained from `self.history.get()`
    /// while both the outer `version` and inner `history_version` seqlocks are
    /// in a stable (even) state. The caller must re-check both seqlocks after
    /// this function returns and discard the result if either changed.
    unsafe fn find_at_timestamp(
        &self,
        history: &[TransformEntry],
        seq: u64,
        target_ts: u64,
    ) -> Option<TransformEntry> {
        let available = seq.min(self.history_capacity as u64) as usize;

        // Scan from newest to oldest
        for offset in 0..available {
            let idx = ((seq - 1 - offset as u64) as usize) % self.history_capacity;
            let entry = &history[idx];

            if entry.timestamp_ns <= target_ts {
                return Some(*entry);
            }
        }

        // Return oldest if all are newer than target
        if available > 0 {
            let oldest_idx = ((seq - available as u64) as usize) % self.history_capacity;
            Some(history[oldest_idx])
        } else {
            None
        }
    }

    /// Interpolate between bracketing entries
    ///
    /// # Safety
    ///
    /// `history` must be a valid reference obtained from `self.history.get()`
    /// while both the outer `version` and inner `history_version` seqlocks are
    /// in a stable (even) state. The caller must re-check both seqlocks after
    /// this function returns and discard the result if either changed, as a
    /// concurrent writer may have partially overwritten history entries during
    /// the multi-entry scan.
    unsafe fn interpolate_at_timestamp(
        &self,
        history: &[TransformEntry],
        seq: u64,
        target_ts: u64,
    ) -> Option<Transform> {
        let available = seq.min(self.history_capacity as u64) as usize;
        if available == 0 {
            return None;
        }

        // Find bracketing entries (before and after target)
        let mut before: Option<&TransformEntry> = None;
        let mut after: Option<&TransformEntry> = None;

        // Scan from newest to oldest
        for offset in 0..available {
            let idx = ((seq - 1 - offset as u64) as usize) % self.history_capacity;
            let entry = &history[idx];

            if entry.timestamp_ns <= target_ts {
                before = Some(entry);
                break;
            }
            after = Some(entry);
        }

        match (before, after) {
            (Some(b), Some(a)) if a.timestamp_ns != b.timestamp_ns => {
                // Interpolate between entries
                let t = (target_ts.saturating_sub(b.timestamp_ns)) as f64
                    / (a.timestamp_ns.saturating_sub(b.timestamp_ns)) as f64;
                let t = t.clamp(0.0, 1.0);
                Some(b.transform.interpolate(&a.transform, t))
            }
            (Some(b), _) => Some(b.transform),
            (None, Some(a)) => Some(a.transform),
            (None, None) => None,
        }
    }

    /// Get time range of buffered transforms
    pub fn time_range(&self) -> Option<(u64, u64)> {
        if self.is_static() {
            return Some((0, u64::MAX)); // Static frames are always "in range"
        }

        loop {
            let v1 = self.version.load(Ordering::Acquire);
            if v1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }

            let seq = self.sequence.load(Ordering::Acquire);
            if seq == 0 {
                return None;
            }

            // SAFETY: Version was even (stable). Read is validated by v2 == v1 check below.
            let (oldest_ts, newest_ts) = unsafe {
                let history = &*self.history.get();
                let available = seq.min(self.history_capacity as u64) as usize;

                let newest_idx = ((seq - 1) as usize) % self.history_capacity;
                let oldest_idx = ((seq - available as u64) as usize) % self.history_capacity;

                (
                    history[oldest_idx].timestamp_ns,
                    history[newest_idx].timestamp_ns,
                )
            };

            atomic::fence(Ordering::Acquire);
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return Some((oldest_ts, newest_ts));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slot_lifecycle() {
        let slot = FrameSlot::new(16);

        // Initially unallocated
        assert!(!slot.is_allocated());
        assert_eq!(slot.frame_type(), FrameType::Unallocated);

        // Initialize as dynamic
        slot.init_dynamic(0);
        assert!(slot.is_allocated());
        assert_eq!(slot.frame_type(), FrameType::Dynamic);
        assert_eq!(slot.parent(), 0);

        // Reset
        slot.reset();
        assert!(!slot.is_allocated());
    }

    #[test]
    fn test_write_read() {
        let slot = FrameSlot::new(16);
        slot.init_dynamic(NO_PARENT);

        // Initially empty
        assert!(slot.read_latest().is_none());

        // Write a transform
        let tf = Transform::from_translation([1.0, 2.0, 3.0]);
        slot.update(&tf, 1000);

        // Read it back
        let entry = slot.read_latest().unwrap();
        assert_eq!(entry.timestamp_ns, 1000);
        assert!((entry.transform.translation[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_history_ring_buffer() {
        let slot = FrameSlot::new(4); // Small buffer
        slot.init_dynamic(NO_PARENT);

        // Write more than capacity
        for i in 0..10 {
            let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
            slot.update(&tf, i * 100);
        }

        // Latest should be the last write
        let entry = slot.read_latest().unwrap();
        assert!((entry.transform.translation[0] - 9.0).abs() < 1e-10);

        // Oldest available should be 10 - 4 = 6
        let oldest = slot.read_at(0).unwrap();
        assert!((oldest.transform.translation[0] - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_static_frame() {
        let slot = FrameSlot::new(16);
        slot.init_static(NO_PARENT);

        let tf = Transform::from_translation([1.0, 0.0, 0.0]);
        slot.set_static_transform(&tf);

        // Should always return the same transform regardless of timestamp
        let entry = slot.read_at(0).unwrap();
        assert!((entry.transform.translation[0] - 1.0).abs() < 1e-10);

        let entry = slot.read_at(u64::MAX).unwrap();
        assert!((entry.transform.translation[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolation() {
        let slot = FrameSlot::new(16);
        slot.init_dynamic(NO_PARENT);

        // Write two transforms
        let tf1 = Transform::from_translation([0.0, 0.0, 0.0]);
        let tf2 = Transform::from_translation([10.0, 0.0, 0.0]);

        slot.update(&tf1, 0);
        slot.update(&tf2, 100);

        // Interpolate at midpoint
        let tf_mid = slot.read_interpolated(50).unwrap();
        assert!((tf_mid.translation[0] - 5.0).abs() < 1e-10);

        // Interpolate at 25%
        let tf_25 = slot.read_interpolated(25).unwrap();
        assert!((tf_25.translation[0] - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_concurrent_access() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let slot = Arc::new(FrameSlot::new(32));
        slot.init_dynamic(NO_PARENT);

        let slot_writer = slot.clone();
        let slot_reader = slot.clone();

        // Use barrier to synchronize threads starting together
        let barrier = Arc::new(Barrier::new(2));
        let barrier_writer = barrier.clone();
        let barrier_reader = barrier.clone();

        // Spawn writer
        let writer = thread::spawn(move || {
            barrier_writer.wait();
            for i in 0..1000 {
                let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
                slot_writer.update(&tf, i);
            }
        });

        // Spawn reader
        let reader = thread::spawn(move || {
            barrier_reader.wait();
            let mut read_count = 0;
            for _ in 0..1000 {
                if slot_reader.read_latest().is_some() {
                    read_count += 1;
                }
                // Small yield to allow writer to make progress
                std::thread::yield_now();
            }
            read_count
        });

        writer.join().unwrap();
        let reads = reader.join().unwrap();

        // Should have successfully read many times (may be 0 if reader finishes first on slow systems)
        // The main point is no crashes during concurrent access
        let _ = reads;

        // Final value should be correct
        let entry = slot.read_latest().unwrap();
        assert!((entry.transform.translation[0] - 999.0).abs() < 1e-10);
    }

    /// Verify that concurrent history writes and read_interpolated() calls never
    /// produce NaN or Inf translations — which would indicate a torn read where
    /// two half-written floats were combined into a nonsensical interpolation.
    ///
    /// The writer pushes transforms with monotonically increasing x-translation
    /// (0.0, 1.0, 2.0, …) at 1ms virtual-time intervals.  The reader queries
    /// with a midpoint timestamp and verifies the result is always finite and
    /// within the known range [0.0, write_count].
    #[test]
    fn test_concurrent_interpolate_no_nan_or_torn_read() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        const WRITE_COUNT: u64 = 2000;
        const HISTORY_SIZE: usize = 64;
        const TICK_NS: u64 = 1_000_000; // 1ms per tick

        let slot = Arc::new(FrameSlot::new(HISTORY_SIZE));
        slot.init_dynamic(NO_PARENT);

        let barrier = Arc::new(Barrier::new(2));

        // Writer: push transforms at simulated 1kHz (1ms increments)
        let slot_w = slot.clone();
        let barrier_w = barrier.clone();
        let writer = thread::spawn(move || {
            barrier_w.wait();
            for i in 0..WRITE_COUNT {
                let tf = Transform::from_translation([i as f64, i as f64 * 0.5, 0.0]);
                slot_w.update(&tf, i * TICK_NS);
            }
        });

        // Reader: query midpoint timestamps and verify finite, in-range results
        let slot_r = slot.clone();
        let barrier_r = barrier.clone();
        let reader = thread::spawn(move || {
            barrier_r.wait();
            for i in 0..WRITE_COUNT {
                let query_ts = i * TICK_NS + TICK_NS / 2; // midpoint between ticks
                if let Some(tf) = slot_r.read_interpolated(query_ts) {
                    // All components must be finite — NaN/Inf indicates a torn read
                    assert!(
                        tf.translation[0].is_finite(),
                        "NaN/Inf in translation.x at query_ts={}",
                        query_ts
                    );
                    assert!(
                        tf.translation[1].is_finite(),
                        "NaN/Inf in translation.y at query_ts={}",
                        query_ts
                    );
                    assert!(
                        tf.translation[2].is_finite(),
                        "NaN/Inf in translation.z at query_ts={}",
                        query_ts
                    );
                    // x-translation must be within the written range [0, WRITE_COUNT]
                    assert!(
                        tf.translation[0] >= -1.0 && tf.translation[0] <= WRITE_COUNT as f64,
                        "x-translation {} out of expected range at query_ts={}",
                        tf.translation[0],
                        query_ts
                    );
                }
            }
        });

        writer.join().unwrap();
        reader.join().unwrap();
    }
}
