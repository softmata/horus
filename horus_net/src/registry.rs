//! TopicRegistry — tracks all local topics for discovery announcements.
//!
//! `Topic::new()` registers, `Drop` unregisters. Thread-safe via RwLock.
//! Global singleton via OnceLock. Signal mechanism for eventfd wakeup
//! when new data or topology changes occur.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock, RwLock};

use crate::wire::topic_hash;

/// Global singleton registry.
static GLOBAL_REGISTRY: OnceLock<Arc<TopicRegistry>> = OnceLock::new();

/// Get or create the global TopicRegistry.
pub fn global_registry() -> Arc<TopicRegistry> {
    GLOBAL_REGISTRY
        .get_or_init(|| Arc::new(TopicRegistry::new()))
        .clone()
}

/// Role of a topic on this machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TopicRole {
    Publisher,
    #[default]
    Subscriber,
    Both,
}

impl TopicRole {
    /// Merge a new role into an existing role.
    pub fn merge(self, other: TopicRole) -> TopicRole {
        match (self, other) {
            (TopicRole::Both, _) | (_, TopicRole::Both) => TopicRole::Both,
            (TopicRole::Publisher, TopicRole::Subscriber)
            | (TopicRole::Subscriber, TopicRole::Publisher) => TopicRole::Both,
            (TopicRole::Publisher, TopicRole::Publisher) => TopicRole::Publisher,
            (TopicRole::Subscriber, TopicRole::Subscriber) => TopicRole::Subscriber,
        }
    }

    /// Remove a role. Returns None if no roles remain.
    pub fn remove(self, role: TopicRole) -> Option<TopicRole> {
        match (self, role) {
            (TopicRole::Both, TopicRole::Publisher) => Some(TopicRole::Subscriber),
            (TopicRole::Both, TopicRole::Subscriber) => Some(TopicRole::Publisher),
            (TopicRole::Both, TopicRole::Both) => None,
            (TopicRole::Publisher, TopicRole::Publisher) => None,
            (TopicRole::Subscriber, TopicRole::Subscriber) => None,
            // Removing a role that doesn't exist — keep what we have
            (existing, _) => Some(existing),
        }
    }

    /// Wire encoding: 1=pub, 2=sub, 3=both.
    pub fn to_wire(self) -> u8 {
        match self {
            TopicRole::Publisher => 1,
            TopicRole::Subscriber => 2,
            TopicRole::Both => 3,
        }
    }

    pub fn from_wire(v: u8) -> Option<Self> {
        match v {
            1 => Some(TopicRole::Publisher),
            2 => Some(TopicRole::Subscriber),
            3 => Some(TopicRole::Both),
            _ => None,
        }
    }
}

/// A registered topic entry.
#[derive(Debug, Clone, Default)]
pub struct TopicEntry {
    /// Topic name (e.g., "robot.imu").
    pub name: String,
    /// FNV-1a hash of the type name (u32).
    pub type_hash: u32,
    /// Size of the type in bytes (0 if variable-size/serialized).
    pub type_size: u32,
    /// Role on this machine.
    pub role: TopicRole,
    /// Whether the type is POD (zero-copy eligible).
    pub is_pod: bool,
    /// Whether this is a system topic (_horus.presence, _horus.logs, _horus.estop).
    /// System topics bypass import/export guards and use special reliability/priority.
    pub is_system: bool,
}

/// Reserved system topic names for observability.
pub const SYSTEM_TOPIC_PRESENCE: &str = "_horus.presence";
pub const SYSTEM_TOPIC_LOGS: &str = "_horus.logs";
pub const SYSTEM_TOPIC_ESTOP: &str = "_horus.estop";

/// Check if a topic name is a reserved system topic.
pub fn is_system_topic(name: &str) -> bool {
    name == SYSTEM_TOPIC_PRESENCE
        || name == SYSTEM_TOPIC_LOGS
        || name == SYSTEM_TOPIC_ESTOP
}

/// Callback type for topology change notifications.
type ChangeCallback = Box<dyn Fn() + Send + Sync>;

/// Central registry of all local topics.
///
/// Thread-safe: RwLock protects the HashMap.
/// Lock contention is minimal — register/unregister happen at node startup/shutdown,
/// not on the hot path.
pub struct TopicRegistry {
    entries: RwLock<HashMap<String, TopicEntry>>,
    /// Optional callback invoked when topology changes (topic added/removed).
    /// Used to signal the event loop (write to eventfd/pipe).
    on_change: RwLock<Option<ChangeCallback>>,
}

impl TopicRegistry {
    pub fn new() -> Self {
        Self {
            entries: RwLock::new(HashMap::new()),
            on_change: RwLock::new(None),
        }
    }

    /// Register a topic. If it already exists, merges the role.
    pub fn register(
        &self,
        name: &str,
        type_hash: u32,
        type_size: u32,
        role: TopicRole,
        is_pod: bool,
    ) {
        let mut entries = self.entries.write().unwrap();
        if let Some(existing) = entries.get_mut(name) {
            existing.role = existing.role.merge(role);
        } else {
            entries.insert(
                name.to_string(),
                TopicEntry {
                    name: name.to_string(),
                    type_hash,
                    type_size,
                    role,
                    is_pod,
                    is_system: is_system_topic(name),
                },
            );
        }
        drop(entries);
        self.notify_change();
    }

    /// Unregister a role from a topic. Removes the entry if no roles remain.
    pub fn unregister(&self, name: &str, role: TopicRole) {
        let mut entries = self.entries.write().unwrap();
        let should_remove = if let Some(existing) = entries.get_mut(name) {
            match existing.role.remove(role) {
                Some(new_role) => {
                    existing.role = new_role;
                    false
                }
                None => true,
            }
        } else {
            false
        };
        if should_remove {
            entries.remove(name);
        }
        drop(entries);
        self.notify_change();
    }

    /// Snapshot of all entries for discovery announcements.
    /// Get a topic entry by name for type validation.
    pub fn get_entry(&self, name: &str) -> Option<TopicEntry> {
        let entries = self.entries.read().ok()?;
        entries.get(name).cloned()
    }

    pub fn entries(&self) -> Vec<TopicEntry> {
        self.entries.read().unwrap().values().cloned().collect()
    }

    /// Check if any local process subscribes to this topic.
    pub fn has_subscribers(&self, name: &str) -> bool {
        self.entries
            .read()
            .unwrap()
            .get(name)
            .map(|e| matches!(e.role, TopicRole::Subscriber | TopicRole::Both))
            .unwrap_or(false)
    }

    /// Check if any local process publishes this topic.
    pub fn has_publishers(&self, name: &str) -> bool {
        self.entries
            .read()
            .unwrap()
            .get(name)
            .map(|e| matches!(e.role, TopicRole::Publisher | TopicRole::Both))
            .unwrap_or(false)
    }

    /// Get a specific entry by name.
    pub fn get(&self, name: &str) -> Option<TopicEntry> {
        self.entries.read().unwrap().get(name).cloned()
    }

    /// Number of registered topics.
    pub fn len(&self) -> usize {
        self.entries.read().unwrap().len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.read().unwrap().is_empty()
    }

    /// Set a callback that fires when topology changes.
    /// Used by the Replicator to signal the event loop.
    pub fn set_on_change(&self, callback: impl Fn() + Send + Sync + 'static) {
        *self.on_change.write().unwrap() = Some(Box::new(callback));
    }

    fn notify_change(&self) {
        if let Some(cb) = self.on_change.read().unwrap().as_ref() {
            cb();
        }
    }
}

impl Default for TopicRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience: register a publisher topic.
pub fn register_publisher(name: &str, type_name: &str, type_size: u32, is_pod: bool) {
    global_registry().register(name, topic_hash(type_name), type_size, TopicRole::Publisher, is_pod);
}

/// Convenience: register a subscriber topic.
pub fn register_subscriber(name: &str, type_name: &str, type_size: u32, is_pod: bool) {
    global_registry().register(name, topic_hash(type_name), type_size, TopicRole::Subscriber, is_pod);
}

/// Convenience: unregister a publisher topic.
pub fn unregister_publisher(name: &str) {
    global_registry().unregister(name, TopicRole::Publisher);
}

/// Convenience: unregister a subscriber topic.
pub fn unregister_subscriber(name: &str) {
    global_registry().unregister(name, TopicRole::Subscriber);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_registry() -> TopicRegistry {
        TopicRegistry::new()
    }

    #[test]
    fn register_and_list() {
        let reg = make_registry();
        reg.register("imu", 100, 64, TopicRole::Publisher, true);
        reg.register("cmd_vel", 200, 16, TopicRole::Subscriber, true);

        let entries = reg.entries();
        assert_eq!(entries.len(), 2);
    }

    #[test]
    fn register_merge_roles() {
        let reg = make_registry();
        reg.register("imu", 100, 64, TopicRole::Publisher, true);
        reg.register("imu", 100, 64, TopicRole::Subscriber, true);

        let entry = reg.get("imu").unwrap();
        assert_eq!(entry.role, TopicRole::Both);
    }

    #[test]
    fn unregister_partial() {
        let reg = make_registry();
        reg.register("imu", 100, 64, TopicRole::Both, true);
        reg.unregister("imu", TopicRole::Publisher);

        let entry = reg.get("imu").unwrap();
        assert_eq!(entry.role, TopicRole::Subscriber);
    }

    #[test]
    fn unregister_full_removes() {
        let reg = make_registry();
        reg.register("imu", 100, 64, TopicRole::Publisher, true);
        reg.unregister("imu", TopicRole::Publisher);

        assert!(reg.get("imu").is_none());
        assert_eq!(reg.len(), 0);
    }

    #[test]
    fn unregister_nonexistent_noop() {
        let reg = make_registry();
        reg.unregister("ghost", TopicRole::Publisher); // No panic
        assert_eq!(reg.len(), 0);
    }

    #[test]
    fn has_subscribers_and_publishers() {
        let reg = make_registry();
        reg.register("imu", 100, 64, TopicRole::Publisher, true);
        reg.register("cmd", 200, 16, TopicRole::Subscriber, true);
        reg.register("both", 300, 32, TopicRole::Both, true);

        assert!(!reg.has_subscribers("imu"));
        assert!(reg.has_publishers("imu"));

        assert!(reg.has_subscribers("cmd"));
        assert!(!reg.has_publishers("cmd"));

        assert!(reg.has_subscribers("both"));
        assert!(reg.has_publishers("both"));

        assert!(!reg.has_subscribers("missing"));
    }

    #[test]
    fn on_change_callback() {
        use std::sync::atomic::{AtomicU32, Ordering};

        let reg = make_registry();
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = counter.clone();
        reg.set_on_change(move || {
            counter_clone.fetch_add(1, Ordering::Relaxed);
        });

        reg.register("a", 1, 1, TopicRole::Publisher, true);
        reg.register("b", 2, 2, TopicRole::Subscriber, true);
        reg.unregister("a", TopicRole::Publisher);

        assert_eq!(counter.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn thread_safety() {
        let reg = Arc::new(make_registry());
        let mut handles = Vec::new();

        for i in 0..10 {
            let reg = reg.clone();
            handles.push(std::thread::spawn(move || {
                let name = format!("topic_{i}");
                reg.register(&name, i as u32, 8, TopicRole::Publisher, true);
                std::thread::yield_now();
                reg.register(&name, i as u32, 8, TopicRole::Subscriber, true);
                std::thread::yield_now();
                let entry = reg.get(&name).unwrap();
                assert_eq!(entry.role, TopicRole::Both);
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        assert_eq!(reg.len(), 10);
    }

    #[test]
    fn role_wire_roundtrip() {
        for role in [TopicRole::Publisher, TopicRole::Subscriber, TopicRole::Both] {
            let wire = role.to_wire();
            let decoded = TopicRole::from_wire(wire).unwrap();
            assert_eq!(role, decoded);
        }
        assert!(TopicRole::from_wire(0).is_none());
        assert!(TopicRole::from_wire(255).is_none());
    }
}
