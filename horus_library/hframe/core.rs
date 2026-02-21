//! HFrame core storage and chain resolution
//!
//! This module contains the lock-free core data structure that stores
//! all frame transforms and handles chain resolution.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::RwLock;

use super::transform::Transform;

use super::config::HFrameConfig;
use super::slot::FrameSlot;
use horus_core::error::HorusError;
use horus_core::HorusResult;

use super::types::{FrameId, FrameType, NO_PARENT};

/// Core HFrame storage with lock-free operations
///
/// Contains:
/// - Pre-allocated slots for fast path (configurable size)
/// - Parent relationship tracking
/// - Chain cache for repeated lookups
pub struct HFrameCore {
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
    config: HFrameConfig,
}

/// LRU cache for transform chains
struct ChainCache {
    entries: HashMap<(FrameId, FrameId), Vec<FrameId>>,
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

    fn get(&self, src: FrameId, dst: FrameId) -> Option<&Vec<FrameId>> {
        self.entries.get(&(src, dst))
    }

    fn insert(&mut self, src: FrameId, dst: FrameId, chain: Vec<FrameId>) {
        let key = (src, dst);

        // Evict oldest if full
        if self.entries.len() >= self.max_size && !self.entries.contains_key(&key) {
            if let Some(old_key) = self.order.first().cloned() {
                self.entries.remove(&old_key);
                self.order.remove(0);
            }
        }

        self.entries.insert(key, chain);
        self.order.push(key);
    }

    fn invalidate(&mut self) {
        self.entries.clear();
        self.order.clear();
    }
}

impl HFrameCore {
    /// Create a new HFrame core with the given configuration
    pub fn new(config: &HFrameConfig) -> Self {
        let mut slots = Vec::with_capacity(config.max_frames);
        let mut parents = Vec::with_capacity(config.max_frames);
        let children = vec![Vec::new(); config.max_frames];

        for _ in 0..config.max_frames {
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
        }
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
                let mut children = self.children.write().unwrap();
                children[parent as usize].push(id);
            }

            // Invalidate cache
            self.chain_cache.write().unwrap().invalidate();
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
                let mut children = self.children.write().unwrap();
                children[parent as usize].push(id);
            }

            // Invalidate cache
            self.chain_cache.write().unwrap().invalidate();
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
                let mut children = self.children.write().unwrap();
                children[old_parent as usize].retain(|&child| child != id);
            }

            // Invalidate cache
            self.chain_cache.write().unwrap().invalidate();
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
            let mut children = self.children.write().unwrap();
            for child_list in children.iter_mut() {
                child_list.clear();
            }
        }
        self.static_count.store(0, Ordering::Relaxed);
        self.dynamic_count.store(0, Ordering::Relaxed);
        self.chain_cache.write().unwrap().invalidate();
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
        let children = self.children.read().unwrap();
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

    /// Update a frame's transform
    #[inline]
    pub fn update(&self, id: FrameId, transform: &Transform, timestamp_ns: u64) {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].update(transform, timestamp_ns);
        }
    }

    /// Set a static transform
    pub fn set_static_transform(&self, id: FrameId, transform: &Transform) {
        let idx = id as usize;
        if idx < self.slots.len() {
            self.slots[idx].set_static_transform(transform);
        }
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
                    return Err(HorusError::InvalidInput("Cycle detected in frame tree".to_string()));
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
    // Internal
    // ========================================================================

    /// Get chain from cache or compute it
    fn get_or_compute_chain(&self, src: FrameId, dst: FrameId) -> Option<Vec<FrameId>> {
        // Try cache first
        {
            let cache = self.chain_cache.read().unwrap();
            if let Some(chain) = cache.get(src, dst) {
                return Some(chain.clone());
            }
        }

        // Compute chain
        let chain = self.compute_chain(src, dst)?;

        // Cache it
        {
            let mut cache = self.chain_cache.write().unwrap();
            cache.insert(src, dst, chain.clone());
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
                // Going down: need inverse (stored is parent->child, we want child->parent)
                result = result.compose(&tf.inverse());
            } else if !slot.is_static() {
                return None;
            }
        }

        Some(result)
    }
}

// Thread-safe
unsafe impl Send for HFrameCore {}
unsafe impl Sync for HFrameCore {}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_core() -> HFrameCore {
        HFrameCore::new(&HFrameConfig::small())
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

        core.update(1, &tf_base, 1000);
        core.update(2, &tf_camera, 1000);

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
        core.update(1, &tf_base, 1000);

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

        assert!(core.validate().is_ok());
    }
}
