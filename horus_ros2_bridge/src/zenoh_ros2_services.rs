//! ROS2 Service Protocol for Zenoh Bridge
//!
//! Implements the ROS2 service request/response pattern over Zenoh.
//!
//! # ROS2 Service Protocol
//!
//! ROS2 services use the following topic naming convention:
//! - Request: `rq/{service_name}/Request`
//! - Response: `rs/{service_name}/Response`
//!
//! The service messages are encoded using CDR (Common Data Representation)
//! for compatibility with native ROS2 nodes.
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::zenoh_ros2_services::*;
//!
//! // Create a service client
//! let client = Ros2ServiceClient::new("add_two_ints", config).await?;
//!
//! // Call the service
//! let request = AddTwoIntsRequest { a: 1, b: 2 };
//! let response: AddTwoIntsResponse = client.call(&request).await?;
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use parking_lot::RwLock;
use serde::{de::DeserializeOwned, Serialize};

use horus_core::communication::network::queryable::QueryError;
use horus_core::communication::network::zenoh_config::ZenohConfig;

// ============================================================================
// ROS2 Service Topic Naming
// ============================================================================

/// ROS2 service topic prefixes
pub const ROS2_REQUEST_PREFIX: &str = "rq";
pub const ROS2_RESPONSE_PREFIX: &str = "rs";

/// Convert a service name to ROS2 request topic
///
/// Example: `add_two_ints` -> `rq/add_two_ints/Request`
pub fn service_to_request_topic(service_name: &str) -> String {
    format!(
        "{}/{}/Request",
        ROS2_REQUEST_PREFIX,
        service_name.trim_start_matches('/')
    )
}

/// Convert a service name to ROS2 response topic
///
/// Example: `add_two_ints` -> `rs/add_two_ints/Response`
pub fn service_to_response_topic(service_name: &str) -> String {
    format!(
        "{}/{}/Response",
        ROS2_RESPONSE_PREFIX,
        service_name.trim_start_matches('/')
    )
}

/// Parse a ROS2 service topic to extract service name and message type
///
/// Returns `Some((service_name, is_request))` if the topic matches the pattern
pub fn parse_service_topic(topic: &str) -> Option<(String, bool)> {
    let parts: Vec<&str> = topic.split('/').collect();

    if parts.len() < 3 {
        return None;
    }

    let prefix = parts[0];
    let message_type = parts[parts.len() - 1];

    // Extract service name (everything between prefix and Request/Response)
    let service_parts = &parts[1..parts.len() - 1];
    let service_name = service_parts.join("/");

    match (prefix, message_type) {
        ("rq", "Request") => Some((service_name, true)),
        ("rs", "Response") => Some((service_name, false)),
        _ => None,
    }
}

/// Apply namespace to a service topic
pub fn apply_namespace_to_service(service_name: &str, namespace: &str) -> String {
    if namespace.is_empty() {
        service_name.to_string()
    } else {
        format!(
            "{}/{}",
            namespace.trim_end_matches('/'),
            service_name.trim_start_matches('/')
        )
    }
}

// ============================================================================
// ROS2 Service Request/Response Headers
// ============================================================================

/// ROS2 Service Request Header
///
/// Matches the rmw_request_id_t structure in ROS2
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ros2RequestHeader {
    /// GUID of the requester (16 bytes)
    pub writer_guid: [u8; 16],
    /// Sequence number of the request
    pub sequence_number: i64,
}

impl Default for Ros2RequestHeader {
    fn default() -> Self {
        Self {
            writer_guid: [0u8; 16],
            sequence_number: 0,
        }
    }
}

impl Ros2RequestHeader {
    /// Create a new request header with a unique GUID
    pub fn new(sequence_number: i64) -> Self {
        let mut writer_guid = [0u8; 16];
        // Generate a unique GUID using UUID v4
        let uuid = uuid::Uuid::new_v4();
        writer_guid.copy_from_slice(uuid.as_bytes());

        Self {
            writer_guid,
            sequence_number,
        }
    }

    /// Encode header as CDR bytes (24 bytes total: 16 GUID + 8 seq)
    pub fn encode_cdr(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(24);
        data.extend_from_slice(&self.writer_guid);
        data.extend_from_slice(&self.sequence_number.to_le_bytes());
        data
    }

    /// Decode header from CDR bytes
    pub fn decode_cdr(data: &[u8]) -> Option<Self> {
        if data.len() < 24 {
            return None;
        }

        let mut writer_guid = [0u8; 16];
        writer_guid.copy_from_slice(&data[0..16]);

        let sequence_number = i64::from_le_bytes(data[16..24].try_into().ok()?);

        Some(Self {
            writer_guid,
            sequence_number,
        })
    }
}

// ============================================================================
// CDR Serialization Helpers
// ============================================================================

/// Serialize a message to CDR format
///
/// Uses the cdr-encoding crate when the zenoh-ros2 feature is enabled
#[cfg(feature = "zenoh-ros2")]
pub fn serialize_cdr<T: Serialize>(msg: &T) -> Result<Vec<u8>, Ros2ServiceError> {
    cdr_encoding::to_vec(msg).map_err(|e| Ros2ServiceError::SerializationError(e.to_string()))
}

/// Deserialize a message from CDR format
#[cfg(feature = "zenoh-ros2")]
pub fn deserialize_cdr<T: DeserializeOwned>(data: &[u8]) -> Result<T, Ros2ServiceError> {
    cdr_encoding::from_slice(data).map_err(|e| Ros2ServiceError::SerializationError(e.to_string()))
}

/// Fallback serialization using MessagePack when CDR is not available
#[cfg(not(feature = "zenoh-ros2"))]
pub fn serialize_cdr<T: Serialize>(msg: &T) -> Result<Vec<u8>, Ros2ServiceError> {
    // Use MessagePack as fallback - still compatible with HORUS-to-HORUS communication
    rmp_serde::to_vec(msg).map_err(|e| Ros2ServiceError::SerializationError(e.to_string()))
}

#[cfg(not(feature = "zenoh-ros2"))]
pub fn deserialize_cdr<T: DeserializeOwned>(data: &[u8]) -> Result<T, Ros2ServiceError> {
    rmp_serde::from_slice(data).map_err(|e| Ros2ServiceError::SerializationError(e.to_string()))
}

// ============================================================================
// Error Types
// ============================================================================

/// Errors that can occur during ROS2 service calls
#[derive(Debug, Clone)]
pub enum Ros2ServiceError {
    /// Service not found
    ServiceNotFound(String),
    /// Request timeout
    Timeout(Duration),
    /// Serialization error
    SerializationError(String),
    /// Zenoh error
    ZenohError(String),
    /// Invalid response
    InvalidResponse(String),
    /// Service handler error
    HandlerError(String),
    /// Connection error
    ConnectionError(String),
    /// Configuration error
    ConfigError(String),
}

impl std::fmt::Display for Ros2ServiceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ServiceNotFound(s) => write!(f, "Service not found: {}", s),
            Self::Timeout(d) => write!(f, "Service call timed out after {:?}", d),
            Self::SerializationError(e) => write!(f, "Serialization error: {}", e),
            Self::ZenohError(e) => write!(f, "Zenoh error: {}", e),
            Self::InvalidResponse(e) => write!(f, "Invalid response: {}", e),
            Self::HandlerError(e) => write!(f, "Service handler error: {}", e),
            Self::ConnectionError(e) => write!(f, "Connection error: {}", e),
            Self::ConfigError(e) => write!(f, "Configuration error: {}", e),
        }
    }
}

impl std::error::Error for Ros2ServiceError {}

impl From<QueryError> for Ros2ServiceError {
    fn from(e: QueryError) -> Self {
        match e {
            QueryError::Timeout => Ros2ServiceError::Timeout(Duration::from_secs(5)),
            QueryError::NotFound => Ros2ServiceError::ServiceNotFound("unknown".into()),
            QueryError::TooManyPending => {
                Ros2ServiceError::ConnectionError("too many pending requests".into())
            }
            QueryError::ServiceError(status) => {
                Ros2ServiceError::HandlerError(format!("{:?}", status))
            }
            QueryError::SerializationError => {
                Ros2ServiceError::SerializationError("unknown".into())
            }
        }
    }
}

// ============================================================================
// Service Configuration
// ============================================================================

/// Configuration for ROS2 service client/server
#[derive(Debug, Clone)]
pub struct Ros2ServiceConfig {
    /// Base Zenoh configuration
    pub zenoh_config: ZenohConfig,
    /// Service call timeout
    pub timeout: Duration,
    /// Maximum concurrent requests (for server)
    pub max_concurrent_requests: usize,
    /// Whether to use CDR encoding (requires zenoh-ros2 feature)
    pub use_cdr: bool,
    /// Service namespace (for multi-robot)
    pub namespace: String,
}

impl Default for Ros2ServiceConfig {
    fn default() -> Self {
        Self {
            zenoh_config: ZenohConfig::ros2(0),
            timeout: Duration::from_secs(5),
            max_concurrent_requests: 100,
            use_cdr: cfg!(feature = "zenoh-ros2"),
            namespace: String::new(),
        }
    }
}

impl Ros2ServiceConfig {
    /// Create a new service configuration with ROS2 defaults
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the service namespace
    pub fn with_namespace(mut self, namespace: &str) -> Self {
        self.namespace = namespace.to_string();
        self
    }

    /// Set the timeout
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Get the request topic for a service
    pub fn request_topic(&self, service_name: &str) -> String {
        let full_service = apply_namespace_to_service(service_name, &self.namespace);
        service_to_request_topic(&full_service)
    }

    /// Get the response topic for a service
    pub fn response_topic(&self, service_name: &str) -> String {
        let full_service = apply_namespace_to_service(service_name, &self.namespace);
        service_to_response_topic(&full_service)
    }
}

// ============================================================================
// Service Traits
// ============================================================================

/// Trait for ROS2 service request types
pub trait Ros2ServiceRequest: Serialize + DeserializeOwned + Clone + Send + Sync {
    /// The corresponding response type
    type Response: Ros2ServiceResponse;

    /// The service name (e.g., "std_srvs/srv/Trigger")
    fn service_type() -> &'static str;
}

/// Trait for ROS2 service response types
pub trait Ros2ServiceResponse: Serialize + DeserializeOwned + Clone + Send + Sync {}

// ============================================================================
// Pending Request Tracking
// ============================================================================

/// A pending service request awaiting a response
struct PendingRequest {
    /// Sequence number (for debugging/tracing)
    #[allow(dead_code)]
    sequence_number: i64,
    /// Request sent timestamp
    sent_at: std::time::Instant,
    /// Response (filled when received)
    response: Option<Vec<u8>>,
    /// Timeout duration
    timeout: Duration,
}

/// Tracks pending service requests
#[derive(Default)]
pub struct RequestTracker {
    /// Next sequence number
    next_sequence: AtomicU64,
    /// Pending requests by sequence number
    pending: RwLock<HashMap<i64, PendingRequest>>,
    /// Statistics
    stats: ServiceStats,
}

impl RequestTracker {
    pub fn new() -> Self {
        Self {
            next_sequence: AtomicU64::new(1),
            pending: RwLock::new(HashMap::new()),
            stats: ServiceStats::default(),
        }
    }

    /// Create a new request with a unique sequence number
    pub fn create_request(&self, timeout: Duration) -> (i64, Ros2RequestHeader) {
        let seq = self.next_sequence.fetch_add(1, Ordering::Relaxed) as i64;
        let header = Ros2RequestHeader::new(seq);

        let mut pending = self.pending.write();
        pending.insert(
            seq,
            PendingRequest {
                sequence_number: seq,
                sent_at: std::time::Instant::now(),
                response: None,
                timeout,
            },
        );

        self.stats.requests_sent.fetch_add(1, Ordering::Relaxed);
        (seq, header)
    }

    /// Handle a received response
    pub fn handle_response(&self, sequence_number: i64, data: Vec<u8>) -> bool {
        let mut pending = self.pending.write();

        if let Some(req) = pending.get_mut(&sequence_number) {
            req.response = Some(data);
            self.stats
                .responses_received
                .fetch_add(1, Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    /// Wait for a response with timeout
    pub fn wait_response(&self, sequence_number: i64) -> Result<Vec<u8>, Ros2ServiceError> {
        let deadline;

        {
            let pending = self.pending.read();
            if let Some(req) = pending.get(&sequence_number) {
                deadline = req.sent_at + req.timeout;
            } else {
                return Err(Ros2ServiceError::ServiceNotFound(
                    "request not tracked".into(),
                ));
            }
        }

        loop {
            // Check if response is available
            {
                let mut pending = self.pending.write();
                if let Some(req) = pending.get_mut(&sequence_number) {
                    if let Some(response) = req.response.take() {
                        pending.remove(&sequence_number);
                        return Ok(response);
                    }
                } else {
                    return Err(Ros2ServiceError::ServiceNotFound("request expired".into()));
                }
            }

            // Check timeout
            if std::time::Instant::now() >= deadline {
                let mut pending = self.pending.write();
                if let Some(req) = pending.remove(&sequence_number) {
                    self.stats.timeouts.fetch_add(1, Ordering::Relaxed);
                    return Err(Ros2ServiceError::Timeout(req.timeout));
                }
                return Err(Ros2ServiceError::Timeout(Duration::from_secs(5)));
            }

            // Brief sleep before checking again
            std::thread::sleep(Duration::from_micros(100));
        }
    }

    /// Try to get a response without blocking
    pub fn try_get_response(&self, sequence_number: i64) -> Option<Vec<u8>> {
        let mut pending = self.pending.write();

        if let Some(req) = pending.get_mut(&sequence_number) {
            if let Some(response) = req.response.take() {
                pending.remove(&sequence_number);
                return Some(response);
            }
        }

        None
    }

    /// Clean up expired requests
    pub fn cleanup_expired(&self) -> usize {
        let mut pending = self.pending.write();
        let now = std::time::Instant::now();

        let expired: Vec<i64> = pending
            .iter()
            .filter(|(_, req)| now.duration_since(req.sent_at) > req.timeout)
            .map(|(seq, _)| *seq)
            .collect();

        let count = expired.len();
        for seq in expired {
            pending.remove(&seq);
        }

        if count > 0 {
            self.stats
                .timeouts
                .fetch_add(count as u64, Ordering::Relaxed);
        }

        count
    }

    /// Get pending count
    pub fn pending_count(&self) -> usize {
        self.pending.read().len()
    }

    /// Get statistics
    pub fn stats(&self) -> &ServiceStats {
        &self.stats
    }
}

// ============================================================================
// Service Statistics
// ============================================================================

/// Statistics for service calls
#[derive(Debug, Default)]
pub struct ServiceStats {
    /// Number of requests sent
    pub requests_sent: AtomicU64,
    /// Number of responses received
    pub responses_received: AtomicU64,
    /// Number of requests handled (server)
    pub requests_handled: AtomicU64,
    /// Number of errors
    pub errors: AtomicU64,
    /// Number of timeouts
    pub timeouts: AtomicU64,
}

impl ServiceStats {
    /// Get success rate as percentage (0-100)
    pub fn success_rate(&self) -> f64 {
        let sent = self.requests_sent.load(Ordering::Relaxed);
        let received = self.responses_received.load(Ordering::Relaxed);

        if sent == 0 {
            100.0
        } else {
            (received as f64 / sent as f64) * 100.0
        }
    }
}

// ============================================================================
// Service Handler Registry
// ============================================================================

/// Type alias for service handler function
pub type ServiceHandler = Box<dyn Fn(&[u8]) -> Result<Vec<u8>, String> + Send + Sync>;

/// Registry for service handlers
#[derive(Default)]
pub struct ServiceRegistry {
    /// Registered handlers by service name
    handlers: RwLock<HashMap<String, ServiceHandler>>,
}

impl ServiceRegistry {
    pub fn new() -> Self {
        Self {
            handlers: RwLock::new(HashMap::new()),
        }
    }

    /// Register a service handler
    pub fn register<F>(&self, service_name: &str, handler: F)
    where
        F: Fn(&[u8]) -> Result<Vec<u8>, String> + Send + Sync + 'static,
    {
        let mut handlers = self.handlers.write();
        handlers.insert(service_name.to_string(), Box::new(handler));
    }

    /// Unregister a service handler
    pub fn unregister(&self, service_name: &str) -> bool {
        let mut handlers = self.handlers.write();
        handlers.remove(service_name).is_some()
    }

    /// Handle an incoming service request
    pub fn handle_request(
        &self,
        service_name: &str,
        request_data: &[u8],
    ) -> Result<Vec<u8>, Ros2ServiceError> {
        let handlers = self.handlers.read();

        if let Some(handler) = handlers.get(service_name) {
            handler(request_data).map_err(|e| Ros2ServiceError::HandlerError(e))
        } else {
            Err(Ros2ServiceError::ServiceNotFound(service_name.to_string()))
        }
    }

    /// Get list of registered services
    pub fn services(&self) -> Vec<String> {
        self.handlers.read().keys().cloned().collect()
    }

    /// Check if a service is registered
    pub fn has_service(&self, service_name: &str) -> bool {
        self.handlers.read().contains_key(service_name)
    }
}

// ============================================================================
// Common ROS2 Service Types
// ============================================================================

/// std_srvs/srv/Trigger request (empty)
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct TriggerRequest {}

/// std_srvs/srv/Trigger response
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct TriggerResponse {
    pub success: bool,
    pub message: String,
}

impl Ros2ServiceRequest for TriggerRequest {
    type Response = TriggerResponse;
    fn service_type() -> &'static str {
        "std_srvs/srv/Trigger"
    }
}

impl Ros2ServiceResponse for TriggerResponse {}

/// std_srvs/srv/SetBool request
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct SetBoolRequest {
    pub data: bool,
}

/// std_srvs/srv/SetBool response
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct SetBoolResponse {
    pub success: bool,
    pub message: String,
}

impl Ros2ServiceRequest for SetBoolRequest {
    type Response = SetBoolResponse;
    fn service_type() -> &'static str {
        "std_srvs/srv/SetBool"
    }
}

impl Ros2ServiceResponse for SetBoolResponse {}

/// std_srvs/srv/Empty request
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct EmptyRequest {}

/// std_srvs/srv/Empty response
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
pub struct EmptyResponse {}

impl Ros2ServiceRequest for EmptyRequest {
    type Response = EmptyResponse;
    fn service_type() -> &'static str {
        "std_srvs/srv/Empty"
    }
}

impl Ros2ServiceResponse for EmptyResponse {}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ========== Topic Naming Tests ==========

    #[test]
    fn test_service_to_request_topic() {
        assert_eq!(
            service_to_request_topic("add_two_ints"),
            "rq/add_two_ints/Request"
        );
        assert_eq!(
            service_to_request_topic("/robot/get_status"),
            "rq/robot/get_status/Request"
        );
        assert_eq!(
            service_to_request_topic("ns/sub/service"),
            "rq/ns/sub/service/Request"
        );
    }

    #[test]
    fn test_service_to_response_topic() {
        assert_eq!(
            service_to_response_topic("add_two_ints"),
            "rs/add_two_ints/Response"
        );
        assert_eq!(
            service_to_response_topic("/robot/get_status"),
            "rs/robot/get_status/Response"
        );
    }

    #[test]
    fn test_parse_service_topic_request() {
        let result = parse_service_topic("rq/add_two_ints/Request");
        assert_eq!(result, Some(("add_two_ints".to_string(), true)));

        let result = parse_service_topic("rq/robot/get_status/Request");
        assert_eq!(result, Some(("robot/get_status".to_string(), true)));
    }

    #[test]
    fn test_parse_service_topic_response() {
        let result = parse_service_topic("rs/add_two_ints/Response");
        assert_eq!(result, Some(("add_two_ints".to_string(), false)));

        let result = parse_service_topic("rs/nested/path/service/Response");
        assert_eq!(result, Some(("nested/path/service".to_string(), false)));
    }

    #[test]
    fn test_parse_service_topic_invalid() {
        // Not a service topic
        assert_eq!(parse_service_topic("rt/cmd_vel"), None);
        // Wrong suffix
        assert_eq!(parse_service_topic("rq/service/Data"), None);
        // Too short
        assert_eq!(parse_service_topic("rq/Request"), None);
    }

    #[test]
    fn test_apply_namespace() {
        assert_eq!(
            apply_namespace_to_service("get_map", "robot1"),
            "robot1/get_map"
        );
        assert_eq!(apply_namespace_to_service("get_map", ""), "get_map");
        assert_eq!(
            apply_namespace_to_service("/get_map", "robot1/"),
            "robot1/get_map"
        );
    }

    // ========== Request Header Tests ==========

    #[test]
    fn test_request_header_encode_decode() {
        let header = Ros2RequestHeader::new(42);
        let encoded = header.encode_cdr();

        assert_eq!(encoded.len(), 24);

        let decoded = Ros2RequestHeader::decode_cdr(&encoded).unwrap();
        assert_eq!(decoded.writer_guid, header.writer_guid);
        assert_eq!(decoded.sequence_number, 42);
    }

    #[test]
    fn test_request_header_unique_guid() {
        let header1 = Ros2RequestHeader::new(1);
        let header2 = Ros2RequestHeader::new(2);

        // GUIDs should be unique
        assert_ne!(header1.writer_guid, header2.writer_guid);
    }

    // ========== Request Tracker Tests ==========

    #[test]
    fn test_request_tracker_basic() {
        let tracker = RequestTracker::new();

        let (seq1, _) = tracker.create_request(Duration::from_secs(5));
        let (seq2, _) = tracker.create_request(Duration::from_secs(5));

        assert_ne!(seq1, seq2);
        assert_eq!(tracker.pending_count(), 2);
    }

    #[test]
    fn test_request_tracker_response() {
        let tracker = RequestTracker::new();

        let (seq, _) = tracker.create_request(Duration::from_secs(5));

        // Simulate response
        assert!(tracker.handle_response(seq, vec![1, 2, 3]));

        // Get response
        let response = tracker.try_get_response(seq);
        assert_eq!(response, Some(vec![1, 2, 3]));
        assert_eq!(tracker.pending_count(), 0);
    }

    #[test]
    fn test_request_tracker_cleanup() {
        let tracker = RequestTracker::new();

        // Create requests with very short timeout
        for _ in 0..5 {
            tracker.create_request(Duration::from_millis(1));
        }

        // Wait for them to expire
        std::thread::sleep(Duration::from_millis(10));

        let expired = tracker.cleanup_expired();
        assert_eq!(expired, 5);
        assert_eq!(tracker.pending_count(), 0);
    }

    // ========== Service Registry Tests ==========

    #[test]
    fn test_service_registry_basic() {
        let registry = ServiceRegistry::new();

        registry.register("echo", |data| Ok(data.to_vec()));

        assert!(registry.has_service("echo"));
        assert!(!registry.has_service("unknown"));

        let result = registry.handle_request("echo", &[1, 2, 3]).unwrap();
        assert_eq!(result, vec![1, 2, 3]);
    }

    #[test]
    fn test_service_registry_unregister() {
        let registry = ServiceRegistry::new();

        registry.register("temp", |_| Ok(vec![]));
        assert!(registry.has_service("temp"));

        assert!(registry.unregister("temp"));
        assert!(!registry.has_service("temp"));

        // Unregistering again should return false
        assert!(!registry.unregister("temp"));
    }

    #[test]
    fn test_service_registry_error_handling() {
        let registry = ServiceRegistry::new();

        registry.register("fail", |_| Err("intentional error".to_string()));

        let result = registry.handle_request("fail", &[]);
        assert!(matches!(result, Err(Ros2ServiceError::HandlerError(_))));
    }

    #[test]
    fn test_service_registry_not_found() {
        let registry = ServiceRegistry::new();

        let result = registry.handle_request("nonexistent", &[]);
        assert!(matches!(result, Err(Ros2ServiceError::ServiceNotFound(_))));
    }

    // ========== Service Config Tests ==========

    #[test]
    fn test_service_config_topics() {
        let config = Ros2ServiceConfig::new().with_namespace("robot1");

        assert_eq!(config.request_topic("get_map"), "rq/robot1/get_map/Request");
        assert_eq!(
            config.response_topic("get_map"),
            "rs/robot1/get_map/Response"
        );
    }

    #[test]
    fn test_service_config_no_namespace() {
        let config = Ros2ServiceConfig::new();

        assert_eq!(config.request_topic("get_map"), "rq/get_map/Request");
    }

    // ========== Serialization Tests ==========

    #[test]
    fn test_trigger_request_serialization() {
        let request = TriggerRequest {};
        let data = serialize_cdr(&request).unwrap();
        let decoded: TriggerRequest = deserialize_cdr(&data).unwrap();
        // Empty struct, just verify round-trip works
        let _ = decoded;
    }

    #[test]
    fn test_trigger_response_serialization() {
        let response = TriggerResponse {
            success: true,
            message: "Done".to_string(),
        };
        let data = serialize_cdr(&response).unwrap();
        let decoded: TriggerResponse = deserialize_cdr(&data).unwrap();

        assert_eq!(decoded.success, true);
        assert_eq!(decoded.message, "Done");
    }

    #[test]
    fn test_set_bool_request_serialization() {
        let request = SetBoolRequest { data: true };
        let data = serialize_cdr(&request).unwrap();
        let decoded: SetBoolRequest = deserialize_cdr(&data).unwrap();

        assert_eq!(decoded.data, true);
    }

    // ========== Statistics Tests ==========

    #[test]
    fn test_service_stats() {
        let tracker = RequestTracker::new();

        // Create and respond to some requests
        let (seq1, _) = tracker.create_request(Duration::from_secs(5));
        let (seq2, _) = tracker.create_request(Duration::from_secs(5));

        tracker.handle_response(seq1, vec![]);

        let stats = tracker.stats();
        assert_eq!(stats.requests_sent.load(Ordering::Relaxed), 2);
        assert_eq!(stats.responses_received.load(Ordering::Relaxed), 1);

        // 50% success rate (1 of 2)
        assert!((stats.success_rate() - 50.0).abs() < 0.01);
    }

    // ========== Namespace Isolation Tests ==========

    #[test]
    fn test_multi_robot_service_isolation() {
        let config1 = Ros2ServiceConfig::new().with_namespace("robot1");
        let config2 = Ros2ServiceConfig::new().with_namespace("robot2");

        let topic1 = config1.request_topic("get_plan");
        let topic2 = config2.request_topic("get_plan");

        assert_ne!(topic1, topic2);
        assert!(topic1.contains("robot1"));
        assert!(topic2.contains("robot2"));
    }

    #[test]
    fn test_service_topic_roundtrip() {
        // Ensure topic generation and parsing are consistent
        let service = "navigation/get_plan";
        let request_topic = service_to_request_topic(service);

        let parsed = parse_service_topic(&request_topic);
        assert_eq!(parsed, Some((service.to_string(), true)));
    }

    // ==========================================================================
    // Integration Tests: ServiceRegistry + RequestTracker
    // ==========================================================================

    /// Custom service types for integration testing
    #[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
    struct AddTwoIntsRequest {
        a: i64,
        b: i64,
    }

    #[derive(Debug, Clone, Default, Serialize, serde::Deserialize)]
    struct AddTwoIntsResponse {
        sum: i64,
    }

    impl Ros2ServiceRequest for AddTwoIntsRequest {
        type Response = AddTwoIntsResponse;
        fn service_type() -> &'static str {
            "example_interfaces/srv/AddTwoInts"
        }
    }

    impl Ros2ServiceResponse for AddTwoIntsResponse {}

    #[derive(Debug, Clone, Serialize, serde::Deserialize)]
    struct ComputePlanRequest {
        start_x: f64,
        start_y: f64,
        goal_x: f64,
        goal_y: f64,
        avoid_obstacles: bool,
    }

    #[derive(Debug, Clone, Serialize, serde::Deserialize)]
    struct ComputePlanResponse {
        success: bool,
        waypoints: Vec<(f64, f64)>,
        total_distance: f64,
        error_message: String,
    }

    impl Ros2ServiceRequest for ComputePlanRequest {
        type Response = ComputePlanResponse;
        fn service_type() -> &'static str {
            "nav_msgs/srv/ComputePlan"
        }
    }

    impl Ros2ServiceResponse for ComputePlanResponse {}

    #[test]
    fn test_service_registry_request_tracker_integration() {
        // Simulate a full service call flow
        let registry = ServiceRegistry::new();
        let tracker = RequestTracker::new();

        // Register a service handler that adds two integers
        registry.register("add_two_ints", |data| {
            let request: AddTwoIntsRequest = deserialize_cdr(data).map_err(|e| e.to_string())?;
            let response = AddTwoIntsResponse {
                sum: request.a + request.b,
            };
            serialize_cdr(&response).map_err(|e| e.to_string())
        });

        // Client creates a request
        let (seq, _header) = tracker.create_request(Duration::from_secs(5));

        // Serialize the request
        let request = AddTwoIntsRequest { a: 5, b: 7 };
        let request_data = serialize_cdr(&request).unwrap();

        // Server handles the request
        let response_data = registry
            .handle_request("add_two_ints", &request_data)
            .unwrap();

        // Client receives response
        tracker.handle_response(seq, response_data.clone());

        // Client gets and deserializes response
        let received = tracker.try_get_response(seq).unwrap();
        let response: AddTwoIntsResponse = deserialize_cdr(&received).unwrap();

        assert_eq!(response.sum, 12);
        assert_eq!(tracker.stats().requests_sent.load(Ordering::Relaxed), 1);
        assert_eq!(
            tracker.stats().responses_received.load(Ordering::Relaxed),
            1
        );
    }

    #[test]
    fn test_multiple_services_registered() {
        let registry = ServiceRegistry::new();

        // Register multiple services
        registry.register("add", |data| {
            let req: AddTwoIntsRequest = deserialize_cdr(data).map_err(|e| e.to_string())?;
            let resp = AddTwoIntsResponse { sum: req.a + req.b };
            serialize_cdr(&resp).map_err(|e| e.to_string())
        });

        registry.register("multiply", |data| {
            let req: AddTwoIntsRequest = deserialize_cdr(data).map_err(|e| e.to_string())?;
            let resp = AddTwoIntsResponse { sum: req.a * req.b };
            serialize_cdr(&resp).map_err(|e| e.to_string())
        });

        registry.register("trigger", |_| {
            let resp = TriggerResponse {
                success: true,
                message: "triggered".to_string(),
            };
            serialize_cdr(&resp).map_err(|e| e.to_string())
        });

        // Verify all services are registered
        let services = registry.services();
        assert_eq!(services.len(), 3);
        assert!(registry.has_service("add"));
        assert!(registry.has_service("multiply"));
        assert!(registry.has_service("trigger"));

        // Call each service
        let req = AddTwoIntsRequest { a: 3, b: 4 };
        let req_data = serialize_cdr(&req).unwrap();

        let add_resp: AddTwoIntsResponse =
            deserialize_cdr(&registry.handle_request("add", &req_data).unwrap()).unwrap();
        assert_eq!(add_resp.sum, 7);

        let mul_resp: AddTwoIntsResponse =
            deserialize_cdr(&registry.handle_request("multiply", &req_data).unwrap()).unwrap();
        assert_eq!(mul_resp.sum, 12);

        let trigger_resp: TriggerResponse =
            deserialize_cdr(&registry.handle_request("trigger", &[]).unwrap()).unwrap();
        assert!(trigger_resp.success);
    }

    #[test]
    fn test_concurrent_requests_tracking() {
        let tracker = RequestTracker::new();

        // Create multiple concurrent requests
        let (seq1, _) = tracker.create_request(Duration::from_secs(5));
        let (seq2, _) = tracker.create_request(Duration::from_secs(5));
        let (seq3, _) = tracker.create_request(Duration::from_secs(5));

        assert_eq!(tracker.pending_count(), 3);

        // Respond to them in different order
        tracker.handle_response(seq2, vec![2, 2, 2]);
        tracker.handle_response(seq1, vec![1, 1, 1]);
        tracker.handle_response(seq3, vec![3, 3, 3]);

        // Verify correct routing
        assert_eq!(tracker.try_get_response(seq1), Some(vec![1, 1, 1]));
        assert_eq!(tracker.try_get_response(seq2), Some(vec![2, 2, 2]));
        assert_eq!(tracker.try_get_response(seq3), Some(vec![3, 3, 3]));

        assert_eq!(tracker.pending_count(), 0);
    }

    #[test]
    fn test_service_replacement() {
        let registry = ServiceRegistry::new();

        // Register initial handler
        registry.register("test_service", |_| Ok(vec![1]));

        let result1 = registry.handle_request("test_service", &[]).unwrap();
        assert_eq!(result1, vec![1]);

        // Replace with new handler
        registry.register("test_service", |_| Ok(vec![2]));

        let result2 = registry.handle_request("test_service", &[]).unwrap();
        assert_eq!(result2, vec![2]);
    }

    #[test]
    fn test_complex_service_request_response() {
        let registry = ServiceRegistry::new();

        registry.register("compute_plan", |data| {
            let req: ComputePlanRequest = deserialize_cdr(data).map_err(|e| e.to_string())?;

            // Simulate path planning
            let dx = req.goal_x - req.start_x;
            let dy = req.goal_y - req.start_y;
            let distance = (dx * dx + dy * dy).sqrt();

            let waypoints = vec![
                (req.start_x, req.start_y),
                (req.start_x + dx / 2.0, req.start_y + dy / 2.0),
                (req.goal_x, req.goal_y),
            ];

            let resp = ComputePlanResponse {
                success: true,
                waypoints,
                total_distance: distance,
                error_message: String::new(),
            };

            serialize_cdr(&resp).map_err(|e| e.to_string())
        });

        let request = ComputePlanRequest {
            start_x: 0.0,
            start_y: 0.0,
            goal_x: 3.0,
            goal_y: 4.0,
            avoid_obstacles: true,
        };

        let req_data = serialize_cdr(&request).unwrap();
        let resp_data = registry.handle_request("compute_plan", &req_data).unwrap();
        let response: ComputePlanResponse = deserialize_cdr(&resp_data).unwrap();

        assert!(response.success);
        assert_eq!(response.waypoints.len(), 3);
        assert!((response.total_distance - 5.0).abs() < 0.001); // 3-4-5 triangle
    }

    #[test]
    fn test_response_to_unknown_request() {
        let tracker = RequestTracker::new();

        // Try to respond to non-existent request
        assert!(!tracker.handle_response(9999, vec![1, 2, 3]));

        // Verify stats unchanged
        assert_eq!(
            tracker.stats().responses_received.load(Ordering::Relaxed),
            0
        );
    }

    #[test]
    fn test_duplicate_response_handling() {
        let tracker = RequestTracker::new();

        let (seq, _) = tracker.create_request(Duration::from_secs(5));

        // First response
        assert!(tracker.handle_response(seq, vec![1, 2, 3]));
        assert_eq!(
            tracker.stats().responses_received.load(Ordering::Relaxed),
            1
        );

        // Get the response
        let response = tracker.try_get_response(seq);
        assert_eq!(response, Some(vec![1, 2, 3]));

        // Request is removed after getting response, duplicate should fail
        assert!(!tracker.handle_response(seq, vec![4, 5, 6]));
    }

    #[test]
    fn test_request_header_sequence_increment() {
        let tracker = RequestTracker::new();

        let (seq1, header1) = tracker.create_request(Duration::from_secs(5));
        let (seq2, header2) = tracker.create_request(Duration::from_secs(5));
        let (seq3, header3) = tracker.create_request(Duration::from_secs(5));

        // Sequence numbers should be strictly increasing
        assert!(seq1 < seq2);
        assert!(seq2 < seq3);

        // Headers should match their sequence numbers
        assert_eq!(header1.sequence_number, seq1);
        assert_eq!(header2.sequence_number, seq2);
        assert_eq!(header3.sequence_number, seq3);
    }

    #[test]
    fn test_service_handler_deserialization_error() {
        let registry = ServiceRegistry::new();

        registry.register("typed_service", |data| {
            let _: AddTwoIntsRequest = deserialize_cdr(data).map_err(|e| e.to_string())?;
            Ok(vec![])
        });

        // Send invalid data
        let result = registry.handle_request("typed_service", &[0xFF, 0xFF, 0xFF]);
        assert!(matches!(result, Err(Ros2ServiceError::HandlerError(_))));
    }

    #[test]
    fn test_service_config_builder_chain() {
        let config = Ros2ServiceConfig::new()
            .with_namespace("robot1")
            .with_timeout(Duration::from_secs(10));

        assert_eq!(config.namespace, "robot1");
        assert_eq!(config.timeout, Duration::from_secs(10));
        assert_eq!(config.max_concurrent_requests, 100);
    }

    #[test]
    fn test_request_stats_comprehensive() {
        let tracker = RequestTracker::new();

        // Create 10 requests
        let sequences: Vec<i64> = (0..10)
            .map(|_| tracker.create_request(Duration::from_secs(5)).0)
            .collect();

        assert_eq!(tracker.stats().requests_sent.load(Ordering::Relaxed), 10);
        assert_eq!(tracker.pending_count(), 10);

        // Respond to first 7
        for &seq in &sequences[..7] {
            tracker.handle_response(seq, vec![]);
            tracker.try_get_response(seq);
        }

        assert_eq!(
            tracker.stats().responses_received.load(Ordering::Relaxed),
            7
        );
        assert_eq!(tracker.pending_count(), 3);
        assert!((tracker.stats().success_rate() - 70.0).abs() < 0.01);
    }

    #[test]
    fn test_multi_namespace_services() {
        // Simulate multi-robot scenario with isolated service namespaces
        let config1 = Ros2ServiceConfig::new().with_namespace("robot_a");
        let config2 = Ros2ServiceConfig::new().with_namespace("robot_b");

        // Each robot has the same logical service name but different topics
        let service_name = "get_battery_status";

        assert_eq!(
            config1.request_topic(service_name),
            "rq/robot_a/get_battery_status/Request"
        );
        assert_eq!(
            config2.request_topic(service_name),
            "rq/robot_b/get_battery_status/Request"
        );

        // Verify isolation
        assert_ne!(
            config1.request_topic(service_name),
            config2.request_topic(service_name)
        );
    }

    #[test]
    fn test_service_error_display() {
        let errors = vec![
            Ros2ServiceError::ServiceNotFound("test_service".into()),
            Ros2ServiceError::Timeout(Duration::from_secs(5)),
            Ros2ServiceError::SerializationError("invalid".into()),
            Ros2ServiceError::ZenohError("connection lost".into()),
            Ros2ServiceError::InvalidResponse("malformed".into()),
            Ros2ServiceError::HandlerError("panic".into()),
            Ros2ServiceError::ConnectionError("refused".into()),
            Ros2ServiceError::ConfigError("invalid config".into()),
        ];

        for error in errors {
            let display = format!("{}", error);
            assert!(!display.is_empty());
        }
    }

    #[test]
    fn test_request_header_default() {
        let header = Ros2RequestHeader::default();
        assert_eq!(header.writer_guid, [0u8; 16]);
        assert_eq!(header.sequence_number, 0);
    }

    #[test]
    fn test_request_header_decode_too_short() {
        let short_data = vec![0u8; 16]; // Only 16 bytes, need 24
        let result = Ros2RequestHeader::decode_cdr(&short_data);
        assert!(result.is_none());
    }

    #[test]
    fn test_empty_service_serialization() {
        let request = EmptyRequest {};
        let data = serialize_cdr(&request).unwrap();
        let decoded: EmptyRequest = deserialize_cdr(&data).unwrap();
        let _ = decoded;

        let response = EmptyResponse {};
        let data = serialize_cdr(&response).unwrap();
        let decoded: EmptyResponse = deserialize_cdr(&data).unwrap();
        let _ = decoded;
    }

    #[test]
    fn test_registry_list_services() {
        let registry = ServiceRegistry::new();

        registry.register("svc_a", |_| Ok(vec![]));
        registry.register("svc_b", |_| Ok(vec![]));
        registry.register("svc_c", |_| Ok(vec![]));

        let mut services = registry.services();
        services.sort();

        assert_eq!(services, vec!["svc_a", "svc_b", "svc_c"]);
    }

    #[test]
    fn test_threaded_request_response() {
        use std::sync::Arc;
        use std::thread;

        let registry = Arc::new(ServiceRegistry::new());
        let tracker = Arc::new(RequestTracker::new());

        // Register an echo service
        registry.register("echo", |data| Ok(data.to_vec()));

        // Spawn multiple client threads
        let handles: Vec<_> = (0..5)
            .map(|i| {
                let registry = Arc::clone(&registry);
                let tracker = Arc::clone(&tracker);

                thread::spawn(move || {
                    let (seq, _) = tracker.create_request(Duration::from_secs(5));

                    let request_data = vec![i as u8; 10];
                    let response_data = registry.handle_request("echo", &request_data).unwrap();

                    tracker.handle_response(seq, response_data.clone());
                    let received = tracker.try_get_response(seq).unwrap();

                    assert_eq!(received, request_data);
                })
            })
            .collect();

        for handle in handles {
            handle.join().unwrap();
        }

        assert_eq!(tracker.stats().requests_sent.load(Ordering::Relaxed), 5);
        assert_eq!(
            tracker.stats().responses_received.load(Ordering::Relaxed),
            5
        );
    }

    #[test]
    fn test_service_type_info() {
        assert_eq!(TriggerRequest::service_type(), "std_srvs/srv/Trigger");
        assert_eq!(SetBoolRequest::service_type(), "std_srvs/srv/SetBool");
        assert_eq!(EmptyRequest::service_type(), "std_srvs/srv/Empty");
        assert_eq!(
            AddTwoIntsRequest::service_type(),
            "example_interfaces/srv/AddTwoInts"
        );
    }

    #[test]
    fn test_set_bool_response_serialization() {
        let response = SetBoolResponse {
            success: false,
            message: "Failed to set value".to_string(),
        };

        let data = serialize_cdr(&response).unwrap();
        let decoded: SetBoolResponse = deserialize_cdr(&data).unwrap();

        assert_eq!(decoded.success, false);
        assert_eq!(decoded.message, "Failed to set value");
    }
}
