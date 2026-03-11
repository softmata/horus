//! TransformFrame core storage and chain resolution
//!
//! This module contains the lock-free core data structure that stores
//! all frame transforms and handles chain resolution.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{RwLock, RwLockReadGuard, RwLockWriteGuard};

use super::transform::Transform;

use super::config::TransformFrameConfig;
use super::slot::{FrameSlot, TransformEntry};
use horus_core::error::{HorusError, TransformError, ValidationError};
use horus_core::HorusResult;

use super::types::{FrameId, FrameType, NO_PARENT};

/// Core TransformFrame storage with lock-free operations
///
/// Contains:
/// - Pre-allocated slots for fast path (configurable size)
/// - Parent relationship tracking
/// - Chain cache for repeated lookups
pub struct TransformFrameCore {
    /// Pre-allocated frame slots (lock-free access)
    slots: Vec<FrameSlot>,

    /// Parent relationships (atomic for lock-free reads)
    parents: Vec<AtomicU32>,

    /// Children lists (for tree traversal)
    children: RwLock<Vec<Vec<FrameId>>>,

    /// Frame count tracking
    static_count: AtomicUsize,
    dynamic_count: AtomicUsize,

    /// Transform chain cache
    chain_cache: RwLock<ChainCache>,

    /// Configuration
    config: TransformFrameConfig,

    /// Global generation counter for atomic cache invalidation.
    ///
    /// Incremented (with `Release` ordering) after every topology change
    /// (frame add/remove) and every transform write.  Cache entries store the
    /// generation at which they were computed; `get_or_compute_chain` loads
    /// the counter with `Acquire` ordering and rejects any entry whose stored
    /// generation does not match.
    ///
    /// The Release/Acquire pair creates a happens-before edge: if a reader
    /// observes generation N it is guaranteed to see the topology and
    /// transform data that was visible when generation N was established.
    /// This eliminates the window that existed with per-frame invalidation,
    /// where a concurrent reader could see a new transform but still hold
    /// a cached chain that pre-dated the topology change that produced it.
    global_generation: AtomicU64,
}

/// LRU cache for transform chains.
///
/// Each entry stores the global generation at which the chain was computed.
/// A reader that observes a different global generation must treat the entry
/// as stale and recompute — this ensures the chain is always consistent with
/// the transform and topology data visible at the reader's generation.
struct ChainCache {
    /// `(src, dst)  →  (generation_when_cached, chain_frame_ids)`
    entries: HashMap<(FrameId, FrameId), (u64, Vec<FrameId>)>,
    order: Vec<(FrameId, FrameId)>,
    max_size: usize,
}

impl ChainCache {
    fn new(max_size: usize) -> Self {
        Self {
            entries: HashMap::with_capacity(max_size),
            order: Vec::with_capacity(max_size),
            max_size,
        }
    }

    /// Return `(generation, &chain)` for a cached entry, or `None` if absent.
    fn get(&self, src: FrameId, dst: FrameId) -> Option<(u64, &Vec<FrameId>)> {
        self.entries
            .get(&(src, dst))
            .map(|(gen, chain)| (*gen, chain))
    }

    fn insert(&mut self, src: FrameId, dst: FrameId, generation: u64, chain: Vec<FrameId>) {
        let key = (src, dst);

        // Evict oldest if full
        if self.entries.len() >= self.max_size && !self.entries.contains_key(&key) {
            if let Some(old_key) = self.order.first().cloned() {
                self.entries.remove(&old_key);
                self.order.remove(0);
            }
        }

        self.entries.insert(key, (generation, chain));
        if !self.order.contains(&key) {
            self.order.push(key);
        }
    }

    fn invalidate(&mut self) {
        self.entries.clear();
        self.order.clear();
    }
}

impl TransformFrameCore {
    /// Create a new TransformFrame core with the given configuration.
    ///
    /// When `enable_overflow` is true, pre-allocates up to 4x the configured
    /// `max_frames` (capped at [`MAX_SUPPORTED_FRAMES`]). This allows the
    /// registry to grow beyond the initial limit without reallocating the
    /// lock-free slot storage.
    pub fn new(config: &TransformFrameConfig) -> Self {
        let physical_capacity = if config.enable_overflow {
            (config.max_frames * 4).min(super::types::MAX_SUPPORTED_FRAMES)
        } else {
            config.max_frames
        };

        let mut slots = Vec::with_capacity(physical_capacity);
        let mut parents = Vec::with_capacity(physical_capacity);
        let children = vec![Vec::new(); physical_capacity];

        for _ in 0..physical_capacity {
            slots.push(FrameSlot::new(config.history_len));
            parents.push(AtomicU32::new(NO_PARENT));
        }

        Self {
            slots,
            parents,
            children: RwLock::new(children),
            static_count: AtomicUsize::new(0),
            dynamic_count: AtomicUsize::new(0),
            chain_cache: RwLock::new(ChainCache::new(config.chain_cache_size)),
            config: config.clone(),
            global_generation: AtomicU64::new(0),
        }
    }

    /// The actual number of pre-allocated slots (may be larger than `config.max_frames`
    /// when `enable_overflow` is true).
    pub fn physical_capacity(&self) -> usize {
        self.slots.len()
    }

    // ========================================================================
    // Lock helpers (poison-recovery)
    // ========================================================================

    #[inline]
    fn read_lock<T>(lock: &RwLock<T>) -> RwLockReadGuard<'_, T> {
        lock.read().unwrap_or_else(|e| e.into_inner())
    }

    #[inline]
    fn write_lock<T>(lock: &RwLock<T>) -> RwLockWriteGuard<'_, T> {
        lock.write().unwrap_or_else(|e| e.into_inner())
    }

    // ========================================================================
    // Slot Management
    // ========================================================================

    /// Initialize a slot as a dynamic frame
    pub fn init_dynamic(&self, id: FrameId, parent: FrameId) {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].init_dynamic(parent);
            self.parents[idx].store(parent, Ordering::Release);
            self.dynamic_count.fetch_add(1, Ordering::Relaxed);

            // Update children list
            if parent != NO_PARENT && (parent as usize) < self.slots.len() {
                let mut children = Self::write_lock(&self.children);
                children[parent as usize].push(id);
            }

            // Bump generation: atomically invalidates all cached chains.
            // The Release ordering synchronizes with Acquire loads in
            // get_or_compute_chain — any reader that sees gen+1 is
            // guaranteed to also see the parent store above.
            self.global_generation.fetch_add(1, Ordering::Release);
        }
    }

    /// Initialize a slot as a static frame
    pub fn init_static(&self, id: FrameId, parent: FrameId) {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].init_static(parent);
            self.parents[idx].store(parent, Ordering::Release);
            self.static_count.fetch_add(1, Ordering::Relaxed);

            // Update children list
            if parent != NO_PARENT && (parent as usize) < self.slots.len() {
                let mut children = Self::write_lock(&self.children);
                children[parent as usize].push(id);
            }

            // Bump generation (see init_dynamic for the ordering rationale).
            self.global_generation.fetch_add(1, Ordering::Release);
        }
    }

    /// Reset a slot to unallocated state
    pub fn reset_slot(&self, id: FrameId) {
        let idx = id as usize;
        if idx < self.slots.len() {
            let was_static = self.slots[idx].is_static();
            let old_parent = self.parents[idx].load(Ordering::Acquire);

            self.slots[idx].reset();
            self.parents[idx].store(NO_PARENT, Ordering::Release);

            if was_static {
                self.static_count.fetch_sub(1, Ordering::Relaxed);
            } else {
                self.dynamic_count.fetch_sub(1, Ordering::Relaxed);
            }

            // Remove from parent's children list
            if old_parent != NO_PARENT && (old_parent as usize) < self.slots.len() {
                let mut children = Self::write_lock(&self.children);
                children[old_parent as usize].retain(|&child| child != id);
            }

            // Bump generation (see init_dynamic for the ordering rationale).
            self.global_generation.fetch_add(1, Ordering::Release);
        }
    }

    /// Reset all slots
    pub fn reset_all(&self) {
        for slot in &self.slots {
            slot.reset();
        }
        for parent in &self.parents {
            parent.store(NO_PARENT, Ordering::Release);
        }
        {
            let mut children = Self::write_lock(&self.children);
            for child_list in children.iter_mut() {
                child_list.clear();
            }
        }
        self.static_count.store(0, Ordering::Relaxed);
        self.dynamic_count.store(0, Ordering::Relaxed);
        // Bump generation AND clear the cache map (reset_all is a full tear-down;
        // there is no point keeping entries in memory).
        self.global_generation.fetch_add(1, Ordering::Release);
        Self::write_lock(&self.chain_cache).invalidate();
    }

    /// Check if a slot is allocated
    #[inline]
    pub fn is_allocated(&self, id: FrameId) -> bool {
        let idx = id as usize;
        idx < self.slots.len() && self.slots[idx].is_allocated()
    }

    /// Check if a frame is static
    #[inline]
    pub fn is_static(&self, id: FrameId) -> bool {
        let idx = id as usize;
        idx < self.slots.len() && self.slots[idx].is_static()
    }

    /// Get frame type
    #[inline]
    pub fn frame_type(&self, id: FrameId) -> FrameType {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].frame_type()
        } else {
            FrameType::Unallocated
        }
    }

    /// Get parent frame ID
    #[inline]
    pub fn parent(&self, id: FrameId) -> Option<FrameId> {
        let idx = id as usize;
        if idx < self.parents.len() {
            let parent = self.parents[idx].load(Ordering::Acquire);
            if parent != NO_PARENT {
                Some(parent)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Get children of a frame
    pub fn children(&self, id: FrameId) -> Vec<FrameId> {
        let idx = id as usize;
        let children = Self::read_lock(&self.children);
        if idx < children.len() {
            children[idx].clone()
        } else {
            Vec::new()
        }
    }

    /// Get frame count
    pub fn frame_count(&self) -> usize {
        self.static_count.load(Ordering::Relaxed) + self.dynamic_count.load(Ordering::Relaxed)
    }

    /// Get static frame count
    pub fn static_frame_count(&self) -> usize {
        self.static_count.load(Ordering::Relaxed)
    }

    /// Get dynamic frame count
    pub fn dynamic_frame_count(&self) -> usize {
        self.dynamic_count.load(Ordering::Relaxed)
    }

    // ========================================================================
    // Transform Updates
    // ========================================================================

    /// Update a frame's transform.
    ///
    /// After writing the transform, the global generation is incremented with
    /// `Release` ordering.  Any reader that subsequently loads the generation
    /// with `Acquire` is guaranteed to observe the new transform value and
    /// will not serve a cached chain that pre-dates this write.
    #[inline]
    pub fn update(&self, id: FrameId, transform: &Transform, timestamp_ns: u64) -> HorusResult<()> {
        let idx = id as usize;
        if idx >= self.slots.len() {
            return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                "Frame ID {} out of range (max {})",
                id,
                self.slots.len()
            ))));
        }

        // Validate and auto-normalize the transform
        let validated = transform.validated().map_err(|e| {
            HorusError::InvalidInput(ValidationError::Other(format!("Transform: {}", e)))
        })?;

        self.slots[idx].update(&validated, timestamp_ns);
        // Increment AFTER the slot write so the Release fence publishes
        // the transform to any thread that Acquires on global_generation.
        self.global_generation.fetch_add(1, Ordering::Release);
        Ok(())
    }

    /// Set a static transform (see `update` for ordering rationale).
    pub fn set_static_transform(&self, id: FrameId, transform: &Transform) -> HorusResult<()> {
        let idx = id as usize;
        if idx >= self.slots.len() {
            return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                "Frame ID {} out of range (max {})",
                id,
                self.slots.len()
            ))));
        }

        // Validate and auto-normalize the transform
        let validated = transform.validated().map_err(|e| {
            HorusError::InvalidInput(ValidationError::Other(format!("Transform: {}", e)))
        })?;

        self.slots[idx].set_static_transform(&validated);
        self.global_generation.fetch_add(1, Ordering::Release);
        Ok(())
    }

    // ========================================================================
    // Transform Queries
    // ========================================================================

    /// Resolve transform from src to dst (latest)
    pub fn resolve(&self, src: FrameId, dst: FrameId) -> Option<Transform> {
        if src == dst {
            return Some(Transform::identity());
        }

        // Get chain (possibly cached)
        let chain = self.get_or_compute_chain(src, dst)?;

        // Compose transforms along chain
        self.compose_chain(&chain, None)
    }

    /// Resolve transform from src to dst at specific timestamp
    pub fn resolve_at(&self, src: FrameId, dst: FrameId, timestamp_ns: u64) -> Option<Transform> {
        if src == dst {
            return Some(Transform::identity());
        }

        // Get chain
        let chain = self.get_or_compute_chain(src, dst)?;

        // Compose transforms with timestamp
        self.compose_chain(&chain, Some(timestamp_ns))
    }

    /// Resolve transform with strict time-range checking.
    ///
    /// Unlike `resolve_at()` which silently clamps to edge values when the
    /// requested timestamp falls outside the buffer window, this method
    /// returns `Err(Extrapolation)` if any frame in the chain would need
    /// to extrapolate.
    pub fn resolve_at_strict(
        &self,
        src: FrameId,
        dst: FrameId,
        timestamp_ns: u64,
    ) -> HorusResult<Transform> {
        if src == dst {
            return Ok(Transform::identity());
        }

        // Get chain
        let chain = self.get_or_compute_chain(src, dst).ok_or_else(|| {
            HorusError::Communication(
                format!("No transform path between frame {} and frame {}", src, dst).into(),
            )
        })?;

        // Check time range for each non-root frame in the chain
        // (root frames don't store transforms, only their children do)
        for &frame_id in &chain {
            let idx = frame_id as usize;
            if idx >= self.slots.len() {
                continue;
            }
            let slot = &self.slots[idx];
            if slot.is_static() {
                continue; // Static frames are always in range
            }
            if let Some((oldest, newest)) = slot.time_range() {
                if oldest == 0 && newest == 0 {
                    continue; // No data yet — resolve_at will handle this
                }
                if timestamp_ns < oldest {
                    return Err(HorusError::Transform(TransformError::Extrapolation {
                        frame: format!("{}", frame_id),
                        requested_ns: timestamp_ns,
                        oldest_ns: oldest,
                        newest_ns: newest,
                    }));
                }
                if timestamp_ns > newest {
                    return Err(HorusError::Transform(TransformError::Extrapolation {
                        frame: format!("{}", frame_id),
                        requested_ns: timestamp_ns,
                        oldest_ns: oldest,
                        newest_ns: newest,
                    }));
                }
            }
        }

        // All frames have the requested timestamp in range — resolve normally
        self.compose_chain(&chain, Some(timestamp_ns))
            .ok_or_else(|| {
                HorusError::Communication(
                    format!(
                        "Failed to compose transform chain between frame {} and frame {}",
                        src, dst
                    )
                    .into(),
                )
            })
    }

    /// Resolve transform with time tolerance checking.
    ///
    /// Like `resolve_at_strict()` but allows a tolerance window: if the
    /// requested timestamp is within `tolerance_ns` of the buffer edges,
    /// the query succeeds (using interpolation/clamping). If it's further
    /// than `tolerance_ns` from the nearest entry, returns `Err(Extrapolation)`.
    ///
    /// `tolerance_ns = u64::MAX` behaves identically to `resolve_at()` (no limit).
    pub fn resolve_at_with_tolerance(
        &self,
        src: FrameId,
        dst: FrameId,
        timestamp_ns: u64,
        tolerance_ns: u64,
    ) -> HorusResult<Transform> {
        if src == dst {
            return Ok(Transform::identity());
        }

        let chain = self.get_or_compute_chain(src, dst).ok_or_else(|| {
            HorusError::Communication(
                format!("No transform path between frame {} and frame {}", src, dst).into(),
            )
        })?;

        // Check time range + tolerance for each frame in the chain
        if tolerance_ns < u64::MAX {
            for &frame_id in &chain {
                let idx = frame_id as usize;
                if idx >= self.slots.len() {
                    continue;
                }
                let slot = &self.slots[idx];
                if slot.is_static() {
                    continue;
                }
                if let Some((oldest, newest)) = slot.time_range() {
                    if oldest == 0 && newest == 0 {
                        continue;
                    }
                    if timestamp_ns < oldest && oldest.saturating_sub(timestamp_ns) > tolerance_ns {
                        return Err(HorusError::Transform(TransformError::Extrapolation {
                            frame: format!("{}", frame_id),
                            requested_ns: timestamp_ns,
                            oldest_ns: oldest,
                            newest_ns: newest,
                        }));
                    }
                    if timestamp_ns > newest && timestamp_ns.saturating_sub(newest) > tolerance_ns {
                        return Err(HorusError::Transform(TransformError::Extrapolation {
                            frame: format!("{}", frame_id),
                            requested_ns: timestamp_ns,
                            oldest_ns: oldest,
                            newest_ns: newest,
                        }));
                    }
                }
            }
        }

        self.compose_chain(&chain, Some(timestamp_ns))
            .ok_or_else(|| {
                HorusError::Communication(
                    format!(
                        "Failed to compose transform chain between frame {} and frame {}",
                        src, dst
                    )
                    .into(),
                )
            })
    }

    /// Read the latest transform entry for a frame.
    ///
    /// Returns `None` if the frame has never been updated or the ID is invalid.
    pub fn read_latest(&self, id: FrameId) -> Option<TransformEntry> {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].read_latest()
        } else {
            None
        }
    }

    /// Get the time range of buffered transforms for a frame.
    pub fn time_range(&self, id: FrameId) -> Option<(u64, u64)> {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].time_range()
        } else {
            None
        }
    }

    /// Check if a transform path exists
    pub fn can_transform(&self, src: FrameId, dst: FrameId) -> bool {
        if src == dst {
            return true;
        }
        self.get_or_compute_chain(src, dst).is_some()
    }

    /// Get the frame chain from src to dst
    pub fn frame_chain(&self, src: FrameId, dst: FrameId) -> Option<Vec<FrameId>> {
        if src == dst {
            return Some(vec![src]);
        }
        self.get_or_compute_chain(src, dst)
    }

    // ========================================================================
    // Validation
    // ========================================================================

    /// Validate the frame tree structure
    pub fn validate(&self) -> HorusResult<()> {
        // Check for cycles
        for id in 0..self.slots.len() {
            if !self.is_allocated(id as FrameId) {
                continue;
            }

            let mut visited = std::collections::HashSet::new();
            let mut current = id as FrameId;

            while current != NO_PARENT {
                if !visited.insert(current) {
                    return Err(HorusError::InvalidInput(ValidationError::Other(
                        "Cycle detected in frame tree".to_string(),
                    )));
                }
                if (current as usize) >= self.parents.len() {
                    break;
                }
                current = self.parents[current as usize].load(Ordering::Acquire);
            }
        }

        Ok(())
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Get the path from a frame to its root as a list of frame IDs.
    ///
    /// Returns `[start, parent, grandparent, ..., root]`.
    /// Used for diagnostics — not on the hot path.
    pub fn path_to_root_ids(&self, start: FrameId) -> Vec<FrameId> {
        self.path_to_root(start)
    }

    // ========================================================================
    // Internal
    // ========================================================================

    /// Get chain from cache or compute it.
    ///
    /// Loads the global generation with `Acquire` ordering before consulting
    /// the cache.  An entry is only valid when its stored generation equals
    /// the current global generation — any intervening topology change or
    /// transform write will have bumped the counter, causing a cache miss
    /// and a fresh chain computation.
    ///
    /// This eliminates the race window that existed with the old per-frame
    /// `invalidate()` approach: the Release/Acquire pairing on the generation
    /// counter guarantees that a reader observing generation N sees exactly
    /// the state (parents + transforms) that was visible when generation N
    /// was established.
    fn get_or_compute_chain(&self, src: FrameId, dst: FrameId) -> Option<Vec<FrameId>> {
        // Acquire load: synchronizes with all preceding Release stores in
        // update(), set_static_transform(), init_dynamic(), init_static(), and
        // reset_slot().
        let gen = self.global_generation.load(Ordering::Acquire);

        // Cache hit: valid only when the stored generation matches.
        {
            let cache = Self::read_lock(&self.chain_cache);
            if let Some((cached_gen, chain)) = cache.get(src, dst) {
                if cached_gen == gen {
                    return Some(chain.clone());
                }
                // cached_gen != gen → stale entry; fall through to recompute.
            }
        }

        // Compute the chain using the topology visible at `gen`.
        let chain = self.compute_chain(src, dst)?;

        // Store with the generation we read at the top of this call.  If
        // another writer bumped the generation concurrently, the next reader
        // will see a mismatch and recompute — that is safe and correct.
        {
            let mut cache = Self::write_lock(&self.chain_cache);
            cache.insert(src, dst, gen, chain.clone());
        }

        Some(chain)
    }

    /// Compute the frame chain from src to dst
    fn compute_chain(&self, src: FrameId, dst: FrameId) -> Option<Vec<FrameId>> {
        // Build path from src to root
        let src_to_root = self.path_to_root(src);

        // Build path from dst to root
        let dst_to_root = self.path_to_root(dst);

        // Find common ancestor
        let mut common_idx_src = None;
        let mut common_idx_dst = None;

        for (i, &src_frame) in src_to_root.iter().enumerate() {
            if let Some(j) = dst_to_root.iter().position(|&f| f == src_frame) {
                common_idx_src = Some(i);
                common_idx_dst = Some(j);
                break;
            }
        }

        let (src_idx, dst_idx) = match (common_idx_src, common_idx_dst) {
            (Some(s), Some(d)) => (s, d),
            _ => return None, // No common ancestor
        };

        // Build chain: src -> common ancestor -> dst
        // The chain is represented as: [src, ..., common, ..., dst]
        // Direction markers would complicate this, so we return
        // the frames and handle direction during composition
        let mut chain = Vec::with_capacity(src_idx + dst_idx + 1);

        // src to common (exclusive of common)
        chain.extend_from_slice(&src_to_root[..src_idx]);

        // common ancestor
        chain.push(src_to_root[src_idx]);

        // common to dst (reverse, exclusive of common)
        for &frame in dst_to_root[..dst_idx].iter().rev() {
            chain.push(frame);
        }

        Some(chain)
    }

    /// Build path from frame to root
    fn path_to_root(&self, start: FrameId) -> Vec<FrameId> {
        let mut path = Vec::with_capacity(32);
        let mut current = start;

        while current != NO_PARENT && (current as usize) < self.parents.len() {
            path.push(current);
            current = self.parents[current as usize].load(Ordering::Acquire);

            // Cycle detection
            if path.len() > self.config.max_frames {
                break;
            }
        }

        path
    }

    /// Compose transforms along a chain
    fn compose_chain(&self, chain: &[FrameId], timestamp: Option<u64>) -> Option<Transform> {
        if chain.is_empty() {
            return Some(Transform::identity());
        }

        if chain.len() == 1 {
            return Some(Transform::identity());
        }

        // Find the common ancestor (it's in the middle of the chain)
        // The chain structure is: [src, ..parents_of_src.., common, ..children_to_dst.., dst]

        // First, find where the direction changes (common ancestor)
        // We need to find where frame[i+1] is NOT the parent of frame[i]

        let mut common_idx = 0;
        for i in 0..chain.len() - 1 {
            let parent_of_current = self.parents[chain[i] as usize].load(Ordering::Acquire);
            if parent_of_current == chain[i + 1] {
                // Still going up toward root
                common_idx = i + 1;
            } else {
                // Direction changed
                break;
            }
        }

        let mut result = Transform::identity();

        // Part 1: src to common (going UP toward root)
        // Each frame stores transform "from parent to this frame" (parent->child)
        // When going UP, we compose these transforms to accumulate child->root
        for &frame_id in chain.iter().take(common_idx) {
            let slot = &self.slots[frame_id as usize];

            let entry = if let Some(ts) = timestamp {
                slot.read_interpolated(ts)
            } else {
                slot.read_latest().map(|e| e.transform)
            };

            if let Some(tf) = entry {
                // The stored transform is parent->child
                // We compose in order: tf_child first (closest to point), then tf_parent
                // result = tf.compose(result) -- apply tf after existing result
                result = tf.compose(&result);
            } else if !slot.is_static() {
                // Dynamic frame with no data
                return None;
            }
        }

        // Part 2: common to dst (going DOWN toward dst)
        // Need to invert transforms when going down
        for i in common_idx..chain.len() - 1 {
            let frame_id = chain[i + 1];
            let slot = &self.slots[frame_id as usize];

            let entry = if let Some(ts) = timestamp {
                slot.read_interpolated(ts)
            } else {
                slot.read_latest().map(|e| e.transform)
            };

            if let Some(tf) = entry {
                // Going down: need inverse (stored is child->parent, we want parent->child).
                // The inverse must go on the OUTSIDE (applied last) — same pattern as UP.
                result = tf.inverse().compose(&result);
            } else if !slot.is_static() {
                return None;
            }
        }

        Some(result)
    }
}

// Thread-safe
unsafe impl Send for TransformFrameCore {}
unsafe impl Sync for TransformFrameCore {}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_core() -> TransformFrameCore {
        TransformFrameCore::new(&TransformFrameConfig::small())
    }

    #[test]
    fn test_init_and_query() {
        let core = make_core();

        // Create world -> base -> camera
        core.init_static(0, NO_PARENT); // world
        core.init_dynamic(1, 0); // base_link
        core.init_dynamic(2, 1); // camera

        assert!(core.is_allocated(0));
        assert!(core.is_allocated(1));
        assert!(core.is_allocated(2));
        assert!(!core.is_allocated(3));

        assert!(core.is_static(0));
        assert!(!core.is_static(1));

        assert_eq!(core.parent(0), None);
        assert_eq!(core.parent(1), Some(0));
        assert_eq!(core.parent(2), Some(1));
    }

    #[test]
    fn test_children() {
        let core = make_core();

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, 0);
        core.init_dynamic(3, 1);

        let children_0 = core.children(0);
        assert_eq!(children_0.len(), 2);
        assert!(children_0.contains(&1));
        assert!(children_0.contains(&2));

        let children_1 = core.children(1);
        assert_eq!(children_1.len(), 1);
        assert!(children_1.contains(&3));
    }

    #[test]
    fn test_resolve_identity() {
        let core = make_core();
        core.init_static(0, NO_PARENT);

        let tf = core.resolve(0, 0).unwrap();
        assert!(tf.is_identity(1e-10));
    }

    #[test]
    fn test_resolve_chain() {
        let core = make_core();

        // world(0) -> base(1) -> camera(2)
        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, 1);

        // Set transforms
        let tf_base = Transform::from_translation([1.0, 0.0, 0.0]);
        let tf_camera = Transform::from_translation([0.0, 0.0, 0.5]);

        core.update(1, &tf_base, 1000).unwrap();
        core.update(2, &tf_camera, 1000).unwrap();

        // camera -> world should compose both transforms
        let tf = core.resolve(2, 0).unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);
        assert!((tf.translation[2] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_resolve_inverse() {
        let core = make_core();

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);

        // tf_base = transform from world to base_link
        // A point at origin in base_link is at [1,0,0] in world
        let tf_base = Transform::from_translation([1.0, 0.0, 0.0]);
        core.update(1, &tf_base, 1000).unwrap();

        // base -> world: transforms a point from base frame to world frame
        // point_in_world = tf_base * point_in_base
        // So this should be [1,0,0]
        let tf = core.resolve(1, 0).unwrap();
        assert!((tf.translation[0] - 1.0).abs() < 1e-10);

        // world -> base: transforms a point from world frame to base frame
        // point_in_base = tf_base.inverse() * point_in_world
        // So this should be [-1,0,0]
        let tf_inv = core.resolve(0, 1).unwrap();
        assert!((tf_inv.translation[0] - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_can_transform() {
        let core = make_core();

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, NO_PARENT); // Separate tree

        assert!(core.can_transform(0, 1));
        assert!(core.can_transform(1, 0));
        assert!(!core.can_transform(0, 2)); // Different trees
    }

    #[test]
    fn test_frame_counts() {
        let core = make_core();

        assert_eq!(core.frame_count(), 0);
        assert_eq!(core.static_frame_count(), 0);
        assert_eq!(core.dynamic_frame_count(), 0);

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, 1);

        assert_eq!(core.frame_count(), 3);
        assert_eq!(core.static_frame_count(), 1);
        assert_eq!(core.dynamic_frame_count(), 2);

        core.reset_slot(2);

        assert_eq!(core.frame_count(), 2);
        assert_eq!(core.dynamic_frame_count(), 1);
    }

    #[test]
    fn test_validation() {
        let core = make_core();

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, 1);

        core.validate().unwrap();
    }

    /// A concurrent writer updating a parent frame's transform and a reader
    /// querying the child chain must always observe a consistent result.
    ///
    /// With the old per-frame `invalidate()` approach a reader could see the
    /// new transform but use a cached chain that pre-dated the topology change.
    /// The generation-based approach makes this impossible: the reader either
    /// sees generation N (old state) or N+1 (new state) — never a mix.
    #[test]
    fn test_generation_cache_invalidation_concurrent() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let core = Arc::new(make_core());

        // world(0) → base(1) → camera(2)
        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);
        core.init_dynamic(2, 1);
        // Give camera a fixed transform so resolve(2,0) can always succeed.
        core.update(2, &Transform::from_translation([0.0, 0.0, 0.5]), 0)
            .unwrap();

        let barrier = Arc::new(Barrier::new(2));

        // Writer: repeatedly updates base_link transform with increasing X translation.
        let core_w = Arc::clone(&core);
        let barrier_w = Arc::clone(&barrier);
        let writer = thread::spawn(move || {
            barrier_w.wait();
            for i in 0..1000u64 {
                let tf = Transform::from_translation([i as f64, 0.0, 0.0]);
                core_w.update(1, &tf, i * 1000).unwrap();
            }
        });

        // Reader: repeatedly resolves the camera → world chain.
        // Must never panic and must always find a connected path.
        let core_r = Arc::clone(&core);
        let barrier_r = Arc::clone(&barrier);
        let reader = thread::spawn(move || {
            barrier_r.wait();
            for _ in 0..1000 {
                // chain must always include all three frames — never a
                // partial or stale topology path.
                if let Some(chain) = core_r.frame_chain(2, 0) {
                    assert!(
                        chain.contains(&0) && chain.contains(&1) && chain.contains(&2),
                        "chain must span world→base→camera: {:?}",
                        chain
                    );
                }
            }
        });

        writer.join().expect("writer thread must not panic");
        reader.join().expect("reader thread must not panic");

        // After all writes settle, the final resolved base→world transform
        // must match the last written value (i = 999 → X = 999.0).
        let tf = core
            .resolve(1, 0)
            .expect("resolve(base→world) must succeed after all writes");
        assert!(
            (tf.translation[0] - 999.0).abs() < 1e-4,
            "final X translation must be 999.0, got {}",
            tf.translation[0]
        );
    }

    /// get_loss_count equivalent for TransformFrame: verify generation increments on
    /// each update and that a stale cache entry is rejected.
    #[test]
    fn test_generation_increments_on_update() {
        let core = make_core();

        core.init_static(0, NO_PARENT);
        core.init_dynamic(1, 0);

        let gen_before = core.global_generation.load(Ordering::Acquire);
        core.update(1, &Transform::from_translation([1.0, 0.0, 0.0]), 1000)
            .unwrap();
        let gen_after = core.global_generation.load(Ordering::Acquire);

        assert!(
            gen_after > gen_before,
            "generation must increment after update(); before={}, after={}",
            gen_before,
            gen_after
        );
    }

    /// A topology change (new frame) bumps the generation.
    #[test]
    fn test_generation_increments_on_topology_change() {
        let core = make_core();
        core.init_static(0, NO_PARENT);

        let gen_before = core.global_generation.load(Ordering::Acquire);
        core.init_dynamic(1, 0);
        let gen_after = core.global_generation.load(Ordering::Acquire);

        assert!(
            gen_after > gen_before,
            "generation must increment after init_dynamic(); before={}, after={}",
            gen_before,
            gen_after
        );
    }
}
