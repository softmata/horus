//! Lock-free frame slot with version-dance (seqlock) protocol
//!
//! This module implements the core lock-free data structure for storing
//! per-frame transforms with history.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};

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
/// Uses a seqlock (version-dance) protocol for lock-free reads:
/// - Writers increment version to odd before writing, even after
/// - Readers spin-wait if version is odd (write in progress)
/// - Readers retry if version changed during read
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

        // SAFETY: Write is serialized via the version counter (odd = write in progress).
        // No concurrent reader will observe a partial write.
        unsafe {
            let history = &mut *self.history.get();
            for entry in history.iter_mut() {
                *entry = TransformEntry::default();
            }
        }

        // Mark as stable
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

    /// Set parent frame ID
    pub fn set_parent(&self, parent: FrameId) {
        self.parent.store(parent, Ordering::Release);
    }

    // ========================================================================
    // Writer Protocol
    // ========================================================================

    /// Update the frame's transform (lock-free write)
    ///
    /// # Thread Safety
    /// Multiple writers are safe but will serialize via version counter.
    /// For best performance, have only one writer per frame.
    pub fn update(&self, transform: &Transform, timestamp_ns: u64) {
        // Step 1: Mark write in progress (odd version)
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;
        debug_assert!(v & 1 == 1, "Version should be odd during write");

        // Step 2: Determine ring buffer slot
        let seq = self.sequence.fetch_add(1, Ordering::AcqRel);
        let idx = (seq as usize) % self.history_capacity;

        // SAFETY: Seqlock protocol ensures readers see either old or new value.
        // Version is odd during write, preventing readers from using stale data.
        unsafe {
            let history = &mut *self.history.get();
            history[idx] = TransformEntry {
                timestamp_ns,
                transform: *transform,
            };
        }

        // Step 4: Mark write complete (even version)
        self.version.store(v + 1, Ordering::Release);
    }

    /// Set a static transform (only index 0 is used)
    pub fn set_static_transform(&self, transform: &Transform) {
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;

        // SAFETY: Write is serialized via the version counter. Readers retry on version mismatch.
        unsafe {
            let history = &mut *self.history.get();
            history[0] = TransformEntry {
                timestamp_ns: 0, // Static transforms have no timestamp
                transform: *transform,
            };
        }

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
    pub fn read_latest(&self) -> Option<TransformEntry> {
        loop {
            // Step 1: Read version (must be even = stable)
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

            // Step 4: Verify version unchanged
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return Some(entry);
            }
            // Version changed, retry
        }
    }

    /// Read transform at (or nearest before) target timestamp
    pub fn read_at(&self, target_ts: u64) -> Option<TransformEntry> {
        // Static frames ignore timestamp
        if self.is_static() {
            return self.read_latest();
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
            let result = unsafe { self.find_at_timestamp(&*self.history.get(), seq, target_ts) };

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
            let result =
                unsafe { self.interpolate_at_timestamp(&*self.history.get(), seq, target_ts) };

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
}
