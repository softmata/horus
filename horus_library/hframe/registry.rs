//! Frame name registry - maps string names to frame IDs
//!
//! The registry provides the user-friendly string-based API while
//! internally using integer IDs for performance.

use std::collections::HashMap;
use std::sync::{Arc, RwLock};

use horus_core::error::HorusError;
use horus_core::HorusResult;

use super::core::HFrameCore;
use super::types::{FrameId, NO_PARENT};

/// Frame name registry
///
/// Provides bidirectional mapping between frame names and IDs.
/// Uses RwLock for thread-safe access (not on hot path).
pub struct FrameRegistry {
    /// Name to ID mapping
    name_to_id: RwLock<HashMap<String, FrameId>>,

    /// ID to name mapping (indexed by frame ID)
    id_to_name: RwLock<Vec<Option<String>>>,

    /// Reference to core storage
    core: Arc<HFrameCore>,

    /// Next available ID for allocation
    next_id: RwLock<FrameId>,

    /// Maximum frames
    max_frames: usize,
}

impl FrameRegistry {
    /// Create a new frame registry
    pub fn new(core: Arc<HFrameCore>, max_frames: usize) -> Self {
        Self {
            name_to_id: RwLock::new(HashMap::with_capacity(max_frames)),
            id_to_name: RwLock::new(vec![None; max_frames]),
            core,
            next_id: RwLock::new(0),
            max_frames,
        }
    }

    /// Register a new dynamic frame
    ///
    /// Returns the assigned frame ID.
    pub fn register(&self, name: &str, parent_name: Option<&str>) -> HorusResult<FrameId> {
        // Resolve parent ID (short-lived read lock, released before write phase).
        let parent_id = if let Some(parent) = parent_name {
            let name_map = self.name_to_id.read().unwrap();
            *name_map
                .get(parent)
                .ok_or_else(|| HorusError::NotFound(format!("Parent frame '{}'", parent)))?
        } else {
            NO_PARENT
        };

        // Atomically: duplicate check → reserve HashMap capacity → allocate ID
        // → insert.  Holding the write lock for the entire sequence guarantees
        // that no concurrent thread can consume the reserved slot between
        // `try_reserve` and `insert`, and that `next_id` is only incremented
        // after all allocations succeed (no ID leak on OOM).
        let id = {
            let mut name_map = self.name_to_id.write().unwrap();

            // Duplicate check under write lock (eliminates the read→write
            // TOCTOU window of the old code).
            if name_map.contains_key(name) {
                return Err(HorusError::AlreadyExists(format!("Frame '{}'", name)));
            }

            // Pre-reserve HashMap capacity.  If this returns Err (OOM),
            // `next_id` has NOT been touched — no frame ID is leaked.
            name_map.try_reserve(1).map_err(|_| {
                HorusError::Memory(format!(
                    "OOM: cannot allocate name-map entry for frame '{}'",
                    name
                ))
            })?;

            // Allocate the frame ID.  `next_id` is incremented here; this is
            // safe because `try_reserve` above guarantees the insert below
            // will not reallocate (and we hold the write lock so no other
            // thread can consume the reserved capacity).
            let id = self.allocate_id()?;

            // Insert into name_map (infallible: capacity was pre-reserved).
            name_map.insert(name.to_string(), id);

            // Update id_to_name (pre-allocated Vec, always in-bounds).
            {
                let mut id_map = self.id_to_name.write().unwrap();
                if (id as usize) < id_map.len() {
                    id_map[id as usize] = Some(name.to_string());
                }
            }

            id
        };

        // Initialize slot in core (outside the name-map locks).
        self.core.init_dynamic(id, parent_id);

        Ok(id)
    }

    /// Register a static frame
    pub fn register_static(&self, name: &str, parent_name: Option<&str>) -> HorusResult<FrameId> {
        // Resolve parent ID (short-lived read lock, released before write phase).
        let parent_id = if let Some(parent) = parent_name {
            let name_map = self.name_to_id.read().unwrap();
            *name_map
                .get(parent)
                .ok_or_else(|| HorusError::NotFound(format!("Parent frame '{}'", parent)))?
        } else {
            NO_PARENT
        };

        // Same atomic reserve → allocate → insert sequence as `register()`.
        // See that function for the ordering rationale.
        let id = {
            let mut name_map = self.name_to_id.write().unwrap();

            if name_map.contains_key(name) {
                return Err(HorusError::AlreadyExists(format!("Frame '{}'", name)));
            }

            name_map.try_reserve(1).map_err(|_| {
                HorusError::Memory(format!(
                    "OOM: cannot allocate name-map entry for frame '{}'",
                    name
                ))
            })?;

            let id = self.allocate_id()?;

            name_map.insert(name.to_string(), id);

            {
                let mut id_map = self.id_to_name.write().unwrap();
                if (id as usize) < id_map.len() {
                    id_map[id as usize] = Some(name.to_string());
                }
            }

            id
        };

        // Initialize slot in core as static.
        self.core.init_static(id, parent_id);

        Ok(id)
    }

    /// Unregister a frame (only dynamic frames can be unregistered)
    pub fn unregister(&self, name: &str) -> HorusResult<()> {
        let id = {
            let name_map = self.name_to_id.read().unwrap();
            *name_map
                .get(name)
                .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", name)))?
        };

        // Check if static
        if self.core.is_static(id) {
            return Err(HorusError::PermissionDenied(format!(
                "Cannot unregister static frame '{}'",
                name
            )));
        }

        // Reset the slot
        self.core.reset_slot(id);

        // Remove from mappings
        {
            let mut name_map = self.name_to_id.write().unwrap();
            let mut id_map = self.id_to_name.write().unwrap();

            name_map.remove(name);
            if (id as usize) < id_map.len() {
                id_map[id as usize] = None;
            }
        }

        Ok(())
    }

    /// Look up frame ID by name
    #[inline]
    pub fn lookup(&self, name: &str) -> Option<FrameId> {
        let name_map = self.name_to_id.read().unwrap();
        name_map.get(name).copied()
    }

    /// Look up frame name by ID
    #[inline]
    pub fn lookup_name(&self, id: FrameId) -> Option<String> {
        let id_map = self.id_to_name.read().unwrap();
        if (id as usize) < id_map.len() {
            id_map[id as usize].clone()
        } else {
            None
        }
    }

    /// Check if a frame exists
    pub fn exists(&self, name: &str) -> bool {
        let name_map = self.name_to_id.read().unwrap();
        name_map.contains_key(name)
    }

    /// Get all registered frame names
    pub fn all_names(&self) -> Vec<String> {
        let name_map = self.name_to_id.read().unwrap();
        name_map.keys().cloned().collect()
    }

    /// Get number of registered frames
    pub fn count(&self) -> usize {
        let name_map = self.name_to_id.read().unwrap();
        name_map.len()
    }

    /// Get or create a frame (useful for auto-registration)
    ///
    /// If the frame exists, returns its ID. Otherwise creates it.
    pub fn get_or_create(&self, name: &str, parent_name: Option<&str>) -> HorusResult<FrameId> {
        // Fast path: check if exists
        if let Some(id) = self.lookup(name) {
            return Ok(id);
        }

        // Slow path: create
        self.register(name, parent_name)
    }

    /// Rename a frame
    pub fn rename(&self, old_name: &str, new_name: &str) -> HorusResult<()> {
        // Check new name doesn't exist
        {
            let name_map = self.name_to_id.read().unwrap();
            if name_map.contains_key(new_name) {
                return Err(HorusError::AlreadyExists(format!("Frame '{}'", new_name)));
            }
        }

        // Get ID for old name
        let id = {
            let name_map = self.name_to_id.read().unwrap();
            *name_map
                .get(old_name)
                .ok_or_else(|| HorusError::NotFound(format!("Frame '{}'", old_name)))?
        };

        // Update mappings
        {
            let mut name_map = self.name_to_id.write().unwrap();
            let mut id_map = self.id_to_name.write().unwrap();

            name_map.remove(old_name);
            name_map.insert(new_name.to_string(), id);

            if (id as usize) < id_map.len() {
                id_map[id as usize] = Some(new_name.to_string());
            }
        }

        Ok(())
    }

    /// Clear all frames
    pub fn clear(&self) {
        let mut name_map = self.name_to_id.write().unwrap();
        let mut id_map = self.id_to_name.write().unwrap();
        let mut next_id = self.next_id.write().unwrap();

        name_map.clear();
        for slot in id_map.iter_mut() {
            *slot = None;
        }
        *next_id = 0;

        // Reset all slots in core
        self.core.reset_all();
    }

    // ========================================================================
    // Internal
    // ========================================================================

    /// Allocate a new frame ID
    fn allocate_id(&self) -> HorusResult<FrameId> {
        let mut next_id = self.next_id.write().unwrap();

        if (*next_id as usize) >= self.max_frames {
            // Try to find a free slot (from unregistered frames)
            if let Some(free_id) = self.find_free_slot() {
                return Ok(free_id);
            }
            return Err(HorusError::InvalidInput(format!(
                "Maximum frame limit ({}) reached",
                self.max_frames
            )));
        }

        let id = *next_id;
        *next_id += 1;
        Ok(id)
    }

    /// Find a free slot (from unregistered frames)
    fn find_free_slot(&self) -> Option<FrameId> {
        let id_map = self.id_to_name.read().unwrap();
        for (idx, slot) in id_map.iter().enumerate() {
            if slot.is_none() && !self.core.is_allocated(idx as FrameId) {
                return Some(idx as FrameId);
            }
        }
        None
    }
}

// Thread-safe
unsafe impl Send for FrameRegistry {}
unsafe impl Sync for FrameRegistry {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hframe::config::HFrameConfig;

    fn make_registry() -> FrameRegistry {
        let config = HFrameConfig::small();
        let core = Arc::new(HFrameCore::new(&config));
        FrameRegistry::new(core, config.max_frames)
    }

    #[test]
    fn test_register_lookup() {
        let registry = make_registry();

        let id = registry.register("world", None).unwrap();
        assert_eq!(id, 0);

        let found_id = registry.lookup("world");
        assert_eq!(found_id, Some(0));

        let found_name = registry.lookup_name(0);
        assert_eq!(found_name, Some("world".to_string()));
    }

    #[test]
    fn test_parent_resolution() {
        let registry = make_registry();

        registry.register("world", None).unwrap();
        let base_id = registry.register("base_link", Some("world")).unwrap();

        assert_eq!(base_id, 1);
    }

    #[test]
    fn test_parent_not_found() {
        let registry = make_registry();

        let result = registry.register("orphan", Some("nonexistent"));
        assert!(
            matches!(result, Err(HorusError::NotFound(ref msg)) if msg.contains("Parent frame"))
        );
    }

    #[test]
    fn test_duplicate_registration() {
        let registry = make_registry();

        registry.register("world", None).unwrap();
        let result = registry.register("world", None);
        assert!(matches!(result, Err(HorusError::AlreadyExists(_))));
    }

    #[test]
    fn test_unregister() {
        let registry = make_registry();

        registry.register("temp", None).unwrap();
        assert!(registry.exists("temp"));

        registry.unregister("temp").unwrap();
        assert!(!registry.exists("temp"));
    }

    #[test]
    fn test_rename() {
        let registry = make_registry();

        let id = registry.register("old_name", None).unwrap();
        registry.rename("old_name", "new_name").unwrap();

        assert!(!registry.exists("old_name"));
        assert!(registry.exists("new_name"));
        assert_eq!(registry.lookup("new_name"), Some(id));
    }

    #[test]
    fn test_get_or_create() {
        let registry = make_registry();

        // Create
        let id1 = registry.get_or_create("frame", None).unwrap();

        // Get existing
        let id2 = registry.get_or_create("frame", None).unwrap();

        assert_eq!(id1, id2);
    }

    /// After a failed registration (duplicate), the next successful registration
    /// must still receive the next sequential ID — verifying that next_id is
    /// not incremented when an error is returned.
    #[test]
    fn test_next_id_not_incremented_on_failure() {
        let registry = make_registry();

        // First frame gets ID 0.
        let id_a = registry.register("a", None).unwrap();
        assert_eq!(id_a, 0);

        // Duplicate registration must fail.
        let result = registry.register("a", None);
        assert!(matches!(result, Err(HorusError::AlreadyExists(_))));

        // Parent-not-found must also fail without consuming an ID.
        let result2 = registry.register("orphan", Some("nonexistent"));
        assert!(matches!(result2, Err(HorusError::NotFound(_))));

        // Next successful registration must get ID 1 (not 2 or 3).
        let id_b = registry.register("b", None).unwrap();
        assert_eq!(id_b, 1);
    }

    #[test]
    fn test_all_names() {
        let registry = make_registry();

        registry.register("a", None).unwrap();
        registry.register("b", None).unwrap();
        registry.register("c", None).unwrap();

        let names = registry.all_names();
        assert_eq!(names.len(), 3);
        assert!(names.contains(&"a".to_string()));
        assert!(names.contains(&"b".to_string()));
        assert!(names.contains(&"c".to_string()));
    }
}
