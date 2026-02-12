//! Query/Response pattern for request-reply communication
//!
//! Provides RPC-style request-response semantics on top of the pub/sub
//! infrastructure, enabling service calls and synchronous queries.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Default query timeout
const DEFAULT_QUERY_TIMEOUT: Duration = Duration::from_secs(5);
/// Maximum pending queries
const MAX_PENDING_QUERIES: usize = 1000;

/// Query configuration
#[derive(Debug, Clone)]
pub struct QueryConfig {
    /// Timeout for waiting for response
    pub timeout: Duration,
    /// Maximum retries
    pub max_retries: u32,
    /// Retry backoff multiplier
    pub retry_backoff: f64,
}

impl Default for QueryConfig {
    fn default() -> Self {
        Self {
            timeout: DEFAULT_QUERY_TIMEOUT,
            max_retries: 3,
            retry_backoff: 2.0,
        }
    }
}

impl QueryConfig {
    /// Fast query with short timeout
    pub fn fast() -> Self {
        Self {
            timeout: Duration::from_millis(100),
            max_retries: 1,
            retry_backoff: 1.0,
        }
    }

    /// Reliable query with retries
    pub fn reliable() -> Self {
        Self {
            timeout: Duration::from_secs(10),
            max_retries: 5,
            retry_backoff: 2.0,
        }
    }
}

/// A query request
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct QueryRequest {
    /// Unique query ID
    pub query_id: u64,
    /// Request payload
    pub payload: Vec<u8>,
    /// Timestamp (microseconds since epoch)
    pub timestamp_us: u64,
    /// Reply-to topic (optional, for routing responses)
    pub reply_to: Option<String>,
}

impl QueryRequest {
    pub fn new(query_id: u64, payload: Vec<u8>) -> Self {
        Self {
            query_id,
            payload,
            timestamp_us: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            reply_to: None,
        }
    }

    pub fn with_reply_to(mut self, topic: &str) -> Self {
        self.reply_to = Some(topic.to_string());
        self
    }

    pub fn encode(&self) -> Result<Vec<u8>, Box<bincode::ErrorKind>> {
        bincode::serialize(self)
    }

    pub fn decode(data: &[u8]) -> Result<Self, Box<bincode::ErrorKind>> {
        bincode::deserialize(data)
    }
}

/// A query response
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct QueryResponse {
    /// Query ID this responds to
    pub query_id: u64,
    /// Response payload
    pub payload: Vec<u8>,
    /// Timestamp
    pub timestamp_us: u64,
    /// Success/error status
    pub status: ResponseStatus,
}

/// Response status
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResponseStatus {
    /// Successful response
    Ok,
    /// Error occurred
    Error,
    /// Service not found
    NotFound,
    /// Timeout on service side
    Timeout,
    /// Service busy
    Busy,
}

impl QueryResponse {
    pub fn ok(query_id: u64, payload: Vec<u8>) -> Self {
        Self {
            query_id,
            payload,
            timestamp_us: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            status: ResponseStatus::Ok,
        }
    }

    pub fn error(query_id: u64, status: ResponseStatus) -> Self {
        Self {
            query_id,
            payload: Vec::new(),
            timestamp_us: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            status,
        }
    }

    pub fn encode(&self) -> Result<Vec<u8>, Box<bincode::ErrorKind>> {
        bincode::serialize(self)
    }

    pub fn decode(data: &[u8]) -> Result<Self, Box<bincode::ErrorKind>> {
        bincode::deserialize(data)
    }
}

/// Pending query waiting for response
struct PendingQuery {
    /// When the query was sent
    sent_at: Instant,
    /// Timeout duration
    timeout: Duration,
    /// Response (filled when received)
    response: Option<QueryResponse>,
}

/// Query client for sending queries and receiving responses
pub struct QueryClient {
    /// Configuration
    config: QueryConfig,
    /// Next query ID
    next_query_id: AtomicU64,
    /// Pending queries waiting for responses
    pending: Arc<Mutex<HashMap<u64, PendingQuery>>>,
}

impl QueryClient {
    pub fn new(config: QueryConfig) -> Self {
        Self {
            config,
            next_query_id: AtomicU64::new(1),
            pending: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Create a new query request
    pub fn create_query(&self, payload: Vec<u8>) -> QueryRequest {
        let query_id = self.next_query_id.fetch_add(1, Ordering::Relaxed);
        QueryRequest::new(query_id, payload)
    }

    /// Register a query as pending (call before sending)
    pub fn register_query(&self, query_id: u64) -> Result<(), QueryError> {
        let mut pending = self.pending.lock().unwrap();

        if pending.len() >= MAX_PENDING_QUERIES {
            return Err(QueryError::TooManyPending);
        }

        pending.insert(
            query_id,
            PendingQuery {
                sent_at: Instant::now(),
                timeout: self.config.timeout,
                response: None,
            },
        );

        Ok(())
    }

    /// Handle a received response
    pub fn handle_response(&self, response: QueryResponse) -> bool {
        let mut pending = self.pending.lock().unwrap();

        if let Some(query) = pending.get_mut(&response.query_id) {
            query.response = Some(response);
            true
        } else {
            // Unknown query ID (maybe timed out already)
            false
        }
    }

    /// Wait for a response to a specific query
    pub fn wait_response(&self, query_id: u64) -> Result<QueryResponse, QueryError> {
        let deadline = Instant::now() + self.config.timeout;

        loop {
            // Check if we have a response
            {
                let mut pending = self.pending.lock().unwrap();
                if let Some(query) = pending.get_mut(&query_id) {
                    if let Some(response) = query.response.take() {
                        pending.remove(&query_id);
                        return Ok(response);
                    }
                } else {
                    return Err(QueryError::NotFound);
                }
            }

            // Check timeout
            if Instant::now() >= deadline {
                let mut pending = self.pending.lock().unwrap();
                pending.remove(&query_id);
                return Err(QueryError::Timeout);
            }

            // Brief sleep before checking again
            std::thread::sleep(Duration::from_micros(100));
        }
    }

    /// Try to get a response without blocking
    pub fn try_get_response(&self, query_id: u64) -> Option<QueryResponse> {
        let mut pending = self.pending.lock().unwrap();

        if let Some(query) = pending.get_mut(&query_id) {
            if let Some(response) = query.response.take() {
                pending.remove(&query_id);
                return Some(response);
            }
        }

        None
    }

    /// Clean up timed out queries
    pub fn cleanup_expired(&self) -> usize {
        let mut pending = self.pending.lock().unwrap();
        let now = Instant::now();

        let expired: Vec<u64> = pending
            .iter()
            .filter(|(_, q)| now.duration_since(q.sent_at) > q.timeout)
            .map(|(id, _)| *id)
            .collect();

        let count = expired.len();
        for id in expired {
            pending.remove(&id);
        }

        count
    }

    /// Get number of pending queries
    pub fn pending_count(&self) -> usize {
        self.pending.lock().unwrap().len()
    }
}

impl Default for QueryClient {
    fn default() -> Self {
        Self::new(QueryConfig::default())
    }
}

/// Query handler function type
pub type QueryHandler = Box<dyn Fn(&[u8]) -> Vec<u8> + Send + Sync>;

/// Query server for handling incoming queries
pub struct QueryServer {
    /// Registered handlers by service name
    handlers: Arc<Mutex<HashMap<String, QueryHandler>>>,
    /// Statistics
    stats: QueryServerStats,
}

/// Query server statistics
#[derive(Debug, Default)]
pub struct QueryServerStats {
    pub queries_received: AtomicU64,
    pub queries_handled: AtomicU64,
    pub queries_failed: AtomicU64,
}

impl QueryServer {
    pub fn new() -> Self {
        Self {
            handlers: Arc::new(Mutex::new(HashMap::new())),
            stats: QueryServerStats::default(),
        }
    }

    /// Register a handler for a service
    pub fn register<F>(&self, service: &str, handler: F)
    where
        F: Fn(&[u8]) -> Vec<u8> + Send + Sync + 'static,
    {
        let mut handlers = self.handlers.lock().unwrap();
        handlers.insert(service.to_string(), Box::new(handler));
    }

    /// Unregister a service handler
    pub fn unregister(&self, service: &str) -> bool {
        let mut handlers = self.handlers.lock().unwrap();
        handlers.remove(service).is_some()
    }

    /// Handle an incoming query
    pub fn handle_query(&self, service: &str, request: QueryRequest) -> QueryResponse {
        self.stats.queries_received.fetch_add(1, Ordering::Relaxed);

        let handlers = self.handlers.lock().unwrap();

        if let Some(handler) = handlers.get(service) {
            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                handler(&request.payload)
            })) {
                Ok(response_payload) => {
                    self.stats.queries_handled.fetch_add(1, Ordering::Relaxed);
                    QueryResponse::ok(request.query_id, response_payload)
                }
                Err(_) => {
                    self.stats.queries_failed.fetch_add(1, Ordering::Relaxed);
                    QueryResponse::error(request.query_id, ResponseStatus::Error)
                }
            }
        } else {
            self.stats.queries_failed.fetch_add(1, Ordering::Relaxed);
            QueryResponse::error(request.query_id, ResponseStatus::NotFound)
        }
    }

    /// Get statistics
    pub fn stats(&self) -> &QueryServerStats {
        &self.stats
    }

    /// Get list of registered services
    pub fn services(&self) -> Vec<String> {
        self.handlers.lock().unwrap().keys().cloned().collect()
    }
}

impl Default for QueryServer {
    fn default() -> Self {
        Self::new()
    }
}

/// Query error types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QueryError {
    /// Query timed out
    Timeout,
    /// Query not found (already completed or expired)
    NotFound,
    /// Too many pending queries
    TooManyPending,
    /// Service returned error
    ServiceError(ResponseStatus),
    /// Serialization error
    SerializationError(String),
}

impl std::fmt::Display for QueryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "Query timed out"),
            Self::NotFound => write!(f, "Query not found"),
            Self::TooManyPending => write!(f, "Too many pending queries"),
            Self::ServiceError(status) => write!(f, "Service error: {:?}", status),
            Self::SerializationError(e) => write!(f, "Serialization error: {}", e),
        }
    }
}

impl std::error::Error for QueryError {}

/// Convenience trait for typed queries
pub trait Queryable: Sized + serde::Serialize + serde::de::DeserializeOwned {
    /// Convert to query payload
    fn to_query_payload(&self) -> Result<Vec<u8>, QueryError> {
        bincode::serialize(self).map_err(|e| QueryError::SerializationError(e.to_string()))
    }

    /// Create from response payload
    fn from_response_payload(data: &[u8]) -> Result<Self, QueryError> {
        bincode::deserialize(data).map_err(|e| QueryError::SerializationError(e.to_string()))
    }
}

// Implement Queryable for common types
impl<T: serde::Serialize + serde::de::DeserializeOwned> Queryable for T {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_query_request_encode_decode() {
        let request = QueryRequest::new(42, vec![1, 2, 3, 4]);
        let encoded = request.encode().unwrap();
        let decoded = QueryRequest::decode(&encoded).unwrap();

        assert_eq!(decoded.query_id, 42);
        assert_eq!(decoded.payload, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_query_response_encode_decode() {
        let response = QueryResponse::ok(42, vec![5, 6, 7]);
        let encoded = response.encode().unwrap();
        let decoded = QueryResponse::decode(&encoded).unwrap();

        assert_eq!(decoded.query_id, 42);
        assert_eq!(decoded.payload, vec![5, 6, 7]);
        assert_eq!(decoded.status, ResponseStatus::Ok);
    }

    #[test]
    fn test_query_client() {
        let client = QueryClient::new(QueryConfig::default());

        let query = client.create_query(vec![1, 2, 3]);
        assert!(query.query_id > 0);

        client.register_query(query.query_id).unwrap();
        assert_eq!(client.pending_count(), 1);

        // Simulate response
        let response = QueryResponse::ok(query.query_id, vec![4, 5, 6]);
        assert!(client.handle_response(response));

        let result = client.try_get_response(query.query_id).unwrap();
        assert_eq!(result.payload, vec![4, 5, 6]);
        assert_eq!(client.pending_count(), 0);
    }

    #[test]
    fn test_query_server() {
        let server = QueryServer::new();

        // Register an echo handler
        server.register("echo", |data| data.to_vec());

        // Register a doubler handler
        server.register("double", |data| {
            data.iter().map(|b| b.wrapping_mul(2)).collect()
        });

        assert_eq!(server.services().len(), 2);

        // Test echo
        let request = QueryRequest::new(1, vec![1, 2, 3]);
        let response = server.handle_query("echo", request);
        assert_eq!(response.status, ResponseStatus::Ok);
        assert_eq!(response.payload, vec![1, 2, 3]);

        // Test double
        let request = QueryRequest::new(2, vec![1, 2, 3]);
        let response = server.handle_query("double", request);
        assert_eq!(response.status, ResponseStatus::Ok);
        assert_eq!(response.payload, vec![2, 4, 6]);

        // Test unknown service
        let request = QueryRequest::new(3, vec![]);
        let response = server.handle_query("unknown", request);
        assert_eq!(response.status, ResponseStatus::NotFound);
    }

    #[test]
    fn test_query_cleanup() {
        let config = QueryConfig {
            timeout: Duration::from_millis(10),
            ..Default::default()
        };
        let client = QueryClient::new(config);

        // Register some queries
        for i in 0..5 {
            let query = client.create_query(vec![i]);
            client.register_query(query.query_id).unwrap();
        }

        assert_eq!(client.pending_count(), 5);

        // Wait for timeout
        std::thread::sleep(Duration::from_millis(20));

        let expired = client.cleanup_expired();
        assert_eq!(expired, 5);
        assert_eq!(client.pending_count(), 0);
    }

    #[test]
    fn test_typed_query() {
        #[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq)]
        struct MyRequest {
            value: i32,
        }

        #[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq)]
        struct _MyResponse {
            result: i32,
        }

        let request = MyRequest { value: 42 };
        let payload = request.to_query_payload().unwrap();

        let decoded = MyRequest::from_response_payload(&payload).unwrap();
        assert_eq!(decoded.value, 42);
    }

    #[test]
    fn test_too_many_pending() {
        let client = QueryClient::new(QueryConfig::default());

        // Fill up pending queries
        for _ in 0..MAX_PENDING_QUERIES {
            let query = client.create_query(vec![]);
            client.register_query(query.query_id).unwrap();
        }

        // Next one should fail
        let query = client.create_query(vec![]);
        let result = client.register_query(query.query_id);
        assert!(matches!(result, Err(QueryError::TooManyPending)));
    }
}
