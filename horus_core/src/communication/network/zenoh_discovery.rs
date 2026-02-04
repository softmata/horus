//! Zenoh Topic Type Introspection
//!
//! Provides runtime introspection of topic message types for dynamic subscribers
//! and debugging. Caches type information to avoid repeated queries.
//!
//! # Features
//!
//! - Query message type for any Zenoh topic
//! - Get message field definitions (for known types)
//! - Support for both HORUS-native and ROS2 message types
//! - LRU cache for type information
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::zenoh_discovery::TypeRegistry;
//!
//! let registry = TypeRegistry::new();
//!
//! // Register a type
//! registry.register_type::<geometry_msgs::Twist>("geometry_msgs/msg/Twist");
//!
//! // Query type info
//! if let Some(info) = registry.get_type_info("geometry_msgs/msg/Twist") {
//!     println!("Fields: {:?}", info.fields);
//! }
//! ```

use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::any::TypeId;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

// ============================================================================
// Type Information
// ============================================================================

/// Information about a message type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TypeInfo {
    /// Fully qualified type name (e.g., "geometry_msgs/msg/Twist")
    pub name: String,
    /// Package name (e.g., "geometry_msgs")
    pub package: String,
    /// Short type name (e.g., "Twist")
    pub short_name: String,
    /// Whether this is a ROS2 type (vs HORUS-native)
    pub is_ros2: bool,
    /// Field definitions
    pub fields: Vec<FieldInfo>,
    /// Size in bytes (if known, 0 for dynamic)
    pub size_bytes: usize,
    /// Whether the type has variable-length fields
    pub is_dynamic: bool,
}

impl TypeInfo {
    /// Create a new TypeInfo for a ROS2 message type
    pub fn ros2(package: &str, short_name: &str, fields: Vec<FieldInfo>) -> Self {
        let is_dynamic = fields.iter().any(|f| f.is_dynamic);
        let size = if is_dynamic {
            0
        } else {
            fields.iter().map(|f| f.size_bytes).sum()
        };

        Self {
            name: format!("{}/msg/{}", package, short_name),
            package: package.to_string(),
            short_name: short_name.to_string(),
            is_ros2: true,
            fields,
            size_bytes: size,
            is_dynamic,
        }
    }

    /// Create a new TypeInfo for a HORUS-native type
    pub fn horus(name: &str, fields: Vec<FieldInfo>) -> Self {
        let is_dynamic = fields.iter().any(|f| f.is_dynamic);
        let size = if is_dynamic {
            0
        } else {
            fields.iter().map(|f| f.size_bytes).sum()
        };

        Self {
            name: name.to_string(),
            package: "horus".to_string(),
            short_name: name.to_string(),
            is_ros2: false,
            fields,
            size_bytes: size,
            is_dynamic,
        }
    }
}

/// Information about a field within a message type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldInfo {
    /// Field name
    pub name: String,
    /// Field type (e.g., "float64", "string", "geometry_msgs/msg/Vector3")
    pub field_type: String,
    /// Whether this is an array
    pub is_array: bool,
    /// Array size (0 for dynamic arrays)
    pub array_size: usize,
    /// Size in bytes (0 for dynamic)
    pub size_bytes: usize,
    /// Whether this field has variable length
    pub is_dynamic: bool,
    /// Offset from start of message (for fixed-layout types)
    pub offset: usize,
}

impl FieldInfo {
    /// Create a scalar field
    pub fn scalar(name: &str, field_type: &str, size_bytes: usize) -> Self {
        Self {
            name: name.to_string(),
            field_type: field_type.to_string(),
            is_array: false,
            array_size: 0,
            size_bytes,
            is_dynamic: false,
            offset: 0,
        }
    }

    /// Create a fixed-size array field
    pub fn array(name: &str, field_type: &str, element_size: usize, count: usize) -> Self {
        Self {
            name: name.to_string(),
            field_type: field_type.to_string(),
            is_array: true,
            array_size: count,
            size_bytes: element_size * count,
            is_dynamic: false,
            offset: 0,
        }
    }

    /// Create a dynamic-length field (string, dynamic array)
    pub fn dynamic(name: &str, field_type: &str) -> Self {
        Self {
            name: name.to_string(),
            field_type: field_type.to_string(),
            is_array: field_type.ends_with("[]"),
            array_size: 0,
            size_bytes: 0,
            is_dynamic: true,
            offset: 0,
        }
    }
}

// ============================================================================
// Primitive Type Sizes
// ============================================================================

/// Get the size of a primitive type in bytes
pub fn primitive_size(type_name: &str) -> Option<usize> {
    match type_name {
        "bool" | "int8" | "uint8" | "byte" | "char" => Some(1),
        "int16" | "uint16" => Some(2),
        "int32" | "uint32" | "float32" => Some(4),
        "int64" | "uint64" | "float64" => Some(8),
        _ => None,
    }
}

/// Check if a type is a primitive
pub fn is_primitive(type_name: &str) -> bool {
    primitive_size(type_name).is_some()
}

// ============================================================================
// Type Registry
// ============================================================================

/// Registry of known message types with introspection support
pub struct TypeRegistry {
    /// Type info by name
    types: Arc<RwLock<HashMap<String, TypeInfo>>>,
    /// Type info by Rust TypeId (for registered generic types)
    type_ids: Arc<RwLock<HashMap<TypeId, String>>>,
    /// Topic to type mapping (cached from discovery)
    topic_types: Arc<RwLock<HashMap<String, CachedType>>>,
    /// Cache TTL
    cache_ttl: Duration,
}

/// Cached type information for a topic
struct CachedType {
    type_name: String,
    cached_at: Instant,
}

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TypeRegistry {
    /// Create a new type registry with default cache TTL (5 minutes)
    pub fn new() -> Self {
        Self::with_cache_ttl(Duration::from_secs(300))
    }

    /// Create with custom cache TTL
    pub fn with_cache_ttl(cache_ttl: Duration) -> Self {
        let registry = Self {
            types: Arc::new(RwLock::new(HashMap::new())),
            type_ids: Arc::new(RwLock::new(HashMap::new())),
            topic_types: Arc::new(RwLock::new(HashMap::new())),
            cache_ttl,
        };

        // Register standard ROS2 primitive types
        registry.register_primitives();

        registry
    }

    /// Register primitive types
    fn register_primitives(&self) {
        let primitives = [
            ("bool", 1),
            ("int8", 1),
            ("uint8", 1),
            ("byte", 1),
            ("char", 1),
            ("int16", 2),
            ("uint16", 2),
            ("int32", 4),
            ("uint32", 4),
            ("float32", 4),
            ("int64", 8),
            ("uint64", 8),
            ("float64", 8),
        ];

        let mut types = self.types.write();
        for (name, size) in primitives {
            types.insert(
                name.to_string(),
                TypeInfo {
                    name: name.to_string(),
                    package: "builtin".to_string(),
                    short_name: name.to_string(),
                    is_ros2: true,
                    fields: Vec::new(),
                    size_bytes: size,
                    is_dynamic: false,
                },
            );
        }

        // String is dynamic
        types.insert(
            "string".to_string(),
            TypeInfo {
                name: "string".to_string(),
                package: "builtin".to_string(),
                short_name: "string".to_string(),
                is_ros2: true,
                fields: Vec::new(),
                size_bytes: 0,
                is_dynamic: true,
            },
        );
    }

    // ========================================================================
    // Type Registration
    // ========================================================================

    /// Register a type with its info
    pub fn register_type_info(&self, info: TypeInfo) {
        log::debug!("type_registry: Registered type '{}'", info.name);
        self.types.write().insert(info.name.clone(), info);
    }

    /// Register a Rust type with its ROS2/HORUS type name
    pub fn register_type<T: 'static>(&self, type_name: &str) {
        let type_id = TypeId::of::<T>();
        self.type_ids.write().insert(type_id, type_name.to_string());
        log::trace!(
            "type_registry: Registered Rust type {:?} as '{}'",
            type_id,
            type_name
        );
    }

    /// Get type name for a Rust type
    pub fn get_type_name<T: 'static>(&self) -> Option<String> {
        let type_id = TypeId::of::<T>();
        self.type_ids.read().get(&type_id).cloned()
    }

    // ========================================================================
    // Type Queries
    // ========================================================================

    /// Get type info by name
    pub fn get_type_info(&self, type_name: &str) -> Option<TypeInfo> {
        self.types.read().get(type_name).cloned()
    }

    /// Check if a type is registered
    pub fn has_type(&self, type_name: &str) -> bool {
        self.types.read().contains_key(type_name)
    }

    /// Get all registered type names
    pub fn get_type_names(&self) -> Vec<String> {
        self.types.read().keys().cloned().collect()
    }

    /// Get field info for a type
    pub fn get_fields(&self, type_name: &str) -> Vec<FieldInfo> {
        self.types
            .read()
            .get(type_name)
            .map(|t| t.fields.clone())
            .unwrap_or_default()
    }

    // ========================================================================
    // Topic Type Queries
    // ========================================================================

    /// Get message type for a topic (from cache or discovery)
    pub fn get_topic_type(&self, topic: &str) -> Option<String> {
        let cache = self.topic_types.read();
        if let Some(cached) = cache.get(topic) {
            if cached.cached_at.elapsed() < self.cache_ttl {
                return Some(cached.type_name.clone());
            }
        }
        None
    }

    /// Set the message type for a topic (cache it)
    pub fn set_topic_type(&self, topic: &str, type_name: &str) {
        self.topic_types.write().insert(
            topic.to_string(),
            CachedType {
                type_name: type_name.to_string(),
                cached_at: Instant::now(),
            },
        );
        log::trace!(
            "type_registry: Cached type '{}' for topic '{}'",
            type_name,
            topic
        );
    }

    /// Get type info for a topic
    pub fn get_topic_type_info(&self, topic: &str) -> Option<TypeInfo> {
        let type_name = self.get_topic_type(topic)?;
        self.get_type_info(&type_name)
    }

    /// Clear expired cache entries
    pub fn clear_expired(&self) {
        let mut cache = self.topic_types.write();
        cache.retain(|_, v| v.cached_at.elapsed() < self.cache_ttl);
    }

    /// Clear all cache entries
    pub fn clear_cache(&self) {
        self.topic_types.write().clear();
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    /// Get registry statistics
    pub fn stats(&self) -> RegistryStats {
        RegistryStats {
            registered_types: self.types.read().len(),
            cached_topics: self.topic_types.read().len(),
            rust_types: self.type_ids.read().len(),
        }
    }
}

/// Type registry statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegistryStats {
    /// Number of registered types
    pub registered_types: usize,
    /// Number of cached topic-type mappings
    pub cached_topics: usize,
    /// Number of registered Rust types
    pub rust_types: usize,
}

// ============================================================================
// Standard Message Type Definitions
// ============================================================================

/// Register common ROS2 message types
pub fn register_common_ros2_types(registry: &TypeRegistry) {
    // std_msgs/msg/Header
    registry.register_type_info(TypeInfo::ros2(
        "std_msgs",
        "Header",
        vec![
            FieldInfo::scalar("stamp.sec", "int32", 4),
            FieldInfo::scalar("stamp.nanosec", "uint32", 4),
            FieldInfo::dynamic("frame_id", "string"),
        ],
    ));

    // geometry_msgs/msg/Vector3
    registry.register_type_info(TypeInfo::ros2(
        "geometry_msgs",
        "Vector3",
        vec![
            FieldInfo::scalar("x", "float64", 8),
            FieldInfo::scalar("y", "float64", 8),
            FieldInfo::scalar("z", "float64", 8),
        ],
    ));

    // geometry_msgs/msg/Twist
    registry.register_type_info(TypeInfo::ros2(
        "geometry_msgs",
        "Twist",
        vec![
            FieldInfo::scalar("linear.x", "float64", 8),
            FieldInfo::scalar("linear.y", "float64", 8),
            FieldInfo::scalar("linear.z", "float64", 8),
            FieldInfo::scalar("angular.x", "float64", 8),
            FieldInfo::scalar("angular.y", "float64", 8),
            FieldInfo::scalar("angular.z", "float64", 8),
        ],
    ));

    // geometry_msgs/msg/Point
    registry.register_type_info(TypeInfo::ros2(
        "geometry_msgs",
        "Point",
        vec![
            FieldInfo::scalar("x", "float64", 8),
            FieldInfo::scalar("y", "float64", 8),
            FieldInfo::scalar("z", "float64", 8),
        ],
    ));

    // geometry_msgs/msg/Quaternion
    registry.register_type_info(TypeInfo::ros2(
        "geometry_msgs",
        "Quaternion",
        vec![
            FieldInfo::scalar("x", "float64", 8),
            FieldInfo::scalar("y", "float64", 8),
            FieldInfo::scalar("z", "float64", 8),
            FieldInfo::scalar("w", "float64", 8),
        ],
    ));

    // sensor_msgs/msg/Imu
    registry.register_type_info(TypeInfo::ros2(
        "sensor_msgs",
        "Imu",
        vec![
            FieldInfo::dynamic("header", "std_msgs/msg/Header"),
            FieldInfo::scalar("orientation.x", "float64", 8),
            FieldInfo::scalar("orientation.y", "float64", 8),
            FieldInfo::scalar("orientation.z", "float64", 8),
            FieldInfo::scalar("orientation.w", "float64", 8),
            FieldInfo::array("orientation_covariance", "float64", 8, 9),
            FieldInfo::scalar("angular_velocity.x", "float64", 8),
            FieldInfo::scalar("angular_velocity.y", "float64", 8),
            FieldInfo::scalar("angular_velocity.z", "float64", 8),
            FieldInfo::array("angular_velocity_covariance", "float64", 8, 9),
            FieldInfo::scalar("linear_acceleration.x", "float64", 8),
            FieldInfo::scalar("linear_acceleration.y", "float64", 8),
            FieldInfo::scalar("linear_acceleration.z", "float64", 8),
            FieldInfo::array("linear_acceleration_covariance", "float64", 8, 9),
        ],
    ));

    // nav_msgs/msg/Odometry
    registry.register_type_info(TypeInfo::ros2(
        "nav_msgs",
        "Odometry",
        vec![
            FieldInfo::dynamic("header", "std_msgs/msg/Header"),
            FieldInfo::dynamic("child_frame_id", "string"),
            FieldInfo::scalar("pose.pose.position.x", "float64", 8),
            FieldInfo::scalar("pose.pose.position.y", "float64", 8),
            FieldInfo::scalar("pose.pose.position.z", "float64", 8),
            FieldInfo::scalar("pose.pose.orientation.x", "float64", 8),
            FieldInfo::scalar("pose.pose.orientation.y", "float64", 8),
            FieldInfo::scalar("pose.pose.orientation.z", "float64", 8),
            FieldInfo::scalar("pose.pose.orientation.w", "float64", 8),
            FieldInfo::dynamic("pose.covariance", "float64[]"),
            FieldInfo::scalar("twist.twist.linear.x", "float64", 8),
            FieldInfo::scalar("twist.twist.linear.y", "float64", 8),
            FieldInfo::scalar("twist.twist.linear.z", "float64", 8),
            FieldInfo::scalar("twist.twist.angular.x", "float64", 8),
            FieldInfo::scalar("twist.twist.angular.y", "float64", 8),
            FieldInfo::scalar("twist.twist.angular.z", "float64", 8),
            FieldInfo::dynamic("twist.covariance", "float64[]"),
        ],
    ));

    log::debug!("type_registry: Registered common ROS2 types");
}

// ============================================================================
// Global Registry
// ============================================================================

lazy_static::lazy_static! {
    /// Global type registry singleton
    pub static ref GLOBAL_TYPE_REGISTRY: TypeRegistry = {
        let registry = TypeRegistry::new();
        register_common_ros2_types(&registry);
        registry
    };
}

/// Get the global type registry
pub fn global_type_registry() -> &'static TypeRegistry {
    &GLOBAL_TYPE_REGISTRY
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_type_registry_primitives() {
        let registry = TypeRegistry::new();

        assert!(registry.has_type("float64"));
        assert!(registry.has_type("string"));
        assert!(!registry.has_type("unknown_type"));

        let float64 = registry.get_type_info("float64").unwrap();
        assert_eq!(float64.size_bytes, 8);
        assert!(!float64.is_dynamic);

        let string = registry.get_type_info("string").unwrap();
        assert!(string.is_dynamic);
    }

    #[test]
    fn test_type_registration() {
        let registry = TypeRegistry::new();

        let twist = TypeInfo::ros2(
            "geometry_msgs",
            "Twist",
            vec![
                FieldInfo::scalar("linear.x", "float64", 8),
                FieldInfo::scalar("linear.y", "float64", 8),
                FieldInfo::scalar("linear.z", "float64", 8),
                FieldInfo::scalar("angular.x", "float64", 8),
                FieldInfo::scalar("angular.y", "float64", 8),
                FieldInfo::scalar("angular.z", "float64", 8),
            ],
        );

        registry.register_type_info(twist);

        assert!(registry.has_type("geometry_msgs/msg/Twist"));
        let info = registry.get_type_info("geometry_msgs/msg/Twist").unwrap();
        assert_eq!(info.fields.len(), 6);
        assert_eq!(info.size_bytes, 48);
        assert!(!info.is_dynamic);
    }

    #[test]
    fn test_topic_type_cache() {
        let registry = TypeRegistry::new();

        registry.set_topic_type("/cmd_vel", "geometry_msgs/msg/Twist");

        assert_eq!(
            registry.get_topic_type("/cmd_vel"),
            Some("geometry_msgs/msg/Twist".to_string())
        );

        // Non-existent topic
        assert_eq!(registry.get_topic_type("/unknown"), None);
    }

    #[test]
    fn test_rust_type_registration() {
        let registry = TypeRegistry::new();

        registry.register_type::<f64>("float64");

        assert_eq!(
            registry.get_type_name::<f64>(),
            Some("float64".to_string())
        );
        assert_eq!(registry.get_type_name::<i32>(), None);
    }

    #[test]
    fn test_field_info() {
        let scalar = FieldInfo::scalar("x", "float64", 8);
        assert!(!scalar.is_array);
        assert!(!scalar.is_dynamic);

        let array = FieldInfo::array("data", "uint8", 1, 100);
        assert!(array.is_array);
        assert_eq!(array.size_bytes, 100);

        let dynamic = FieldInfo::dynamic("name", "string");
        assert!(dynamic.is_dynamic);
    }

    #[test]
    fn test_primitive_size() {
        assert_eq!(primitive_size("float64"), Some(8));
        assert_eq!(primitive_size("int32"), Some(4));
        assert_eq!(primitive_size("bool"), Some(1));
        assert_eq!(primitive_size("string"), None);
        assert_eq!(primitive_size("geometry_msgs/msg/Twist"), None);
    }

    #[test]
    fn test_common_ros2_types() {
        let registry = TypeRegistry::new();
        register_common_ros2_types(&registry);

        assert!(registry.has_type("geometry_msgs/msg/Twist"));
        assert!(registry.has_type("geometry_msgs/msg/Vector3"));
        assert!(registry.has_type("sensor_msgs/msg/Imu"));
        assert!(registry.has_type("nav_msgs/msg/Odometry"));

        let twist = registry.get_type_info("geometry_msgs/msg/Twist").unwrap();
        assert_eq!(twist.package, "geometry_msgs");
        assert_eq!(twist.short_name, "Twist");
    }
}
