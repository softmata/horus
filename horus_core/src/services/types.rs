#![allow(dead_code)]
//! Core types for the HORUS service system.
//!
//! Services provide synchronous request/response communication — the HORUS
//! equivalent of ROS2 services.  A service call blocks the caller until
//! the server returns a response (or a timeout elapses).
//!
//! Internally each service is backed by two topics:
//! - `{name}.request`  — clients publish `ServiceRequest<Req>` here
//! - `{name}.response` — server publishes `ServiceResponse<Res>` here
//!
//! Correlation is done via a monotonically-increasing `request_id` that
//! the client embeds in the request and the server echoes in the response.

use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::fmt::Debug;
use thiserror::Error;

// ─── Service trait ────────────────────────────────────────────────────────────

/// Defines a request/response service.
///
/// Use the [`service!`] macro to define services declaratively rather than
/// implementing this trait manually.
///
/// ```rust,ignore
/// use horus_core::service;
///
/// service! {
///     AddTwoInts {
///         request { a: i64, b: i64 }
///         response { sum: i64 }
///     }
/// }
///
/// // Use the service:
/// let client = ServiceClient::<AddTwoInts>::new()?;
/// let resp = client.call(AddTwoIntsRequest { a: 3, b: 4 }, 1_u64.secs())?;
/// assert_eq!(resp.sum, 7);
/// ```
pub trait Service: Send + Sync + 'static {
    /// The request payload type.
    type Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static;
    /// The response payload type.
    type Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static;

    /// Service name — used as the topic prefix.
    ///
    /// By convention this is `snake_case` (e.g. `"add_two_ints"`).
    fn name() -> &'static str;

    /// Full topic name for the request channel.
    fn request_topic() -> String {
        format!("{}.request", Self::name())
    }

    /// Full topic name for the response channel.
    fn response_topic() -> String {
        format!("{}.response", Self::name())
    }

    /// Human-readable name of the request type (for introspection).
    fn request_type_name() -> &'static str {
        std::any::type_name::<Self::Request>()
    }

    /// Human-readable name of the response type (for introspection).
    fn response_type_name() -> &'static str {
        std::any::type_name::<Self::Response>()
    }
}

// ─── Wire types ───────────────────────────────────────────────────────────────

/// Request wrapper that flows over the request topic.
///
/// The `request_id` is used to correlate requests with responses when
/// multiple clients are calling the same service concurrently.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ServiceRequest<Req> {
    /// Unique, monotonically-increasing correlation ID assigned by the client.
    pub request_id: u64,
    /// The actual request payload.
    pub payload: Req,
    /// Per-client response topic name.
    ///
    /// When set, the server publishes the response to this topic instead of
    /// the shared `{service}.response` topic. This eliminates response routing
    /// contention when multiple clients call the same service concurrently.
    ///
    /// Format: `{service}.response.{client_id}` (e.g., `add_two_ints.response.42`)
    #[serde(default)]
    pub response_topic: Option<String>,
}

/// Response wrapper that flows over the response topic.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ServiceResponse<Res> {
    /// Echoes the `request_id` from the corresponding [`ServiceRequest`].
    pub request_id: u64,
    /// `true` if the server handled the request successfully.
    pub ok: bool,
    /// The response payload; `Some` when `ok == true`.
    pub payload: Option<Res>,
    /// Error message; `Some` when `ok == false`.
    pub error: Option<String>,
}

impl<Res> ServiceResponse<Res> {
    /// Create a successful response.
    #[inline]
    pub fn success(request_id: u64, payload: Res) -> Self {
        Self {
            request_id,
            ok: true,
            payload: Some(payload),
            error: None,
        }
    }

    /// Create an error response.
    #[inline]
    pub fn failure(request_id: u64, error: impl Into<String>) -> Self {
        Self {
            request_id,
            ok: false,
            payload: None,
            error: Some(error.into()),
        }
    }
}

// ─── Error type ───────────────────────────────────────────────────────────────

/// Error returned by service call operations.
#[derive(Debug, Clone, Error)]
pub enum ServiceError {
    /// The call timed out waiting for a response.
    #[error("service call timed out")]
    Timeout,
    /// The server processed the request but returned an error.
    #[error("service returned error: {0}")]
    ServiceFailed(String),
    /// No server has registered for this service.
    #[error("no server is available for this service")]
    NoServer,
    /// Topic I/O error.
    #[error("transport error: {0}")]
    Transport(String),
}

impl ServiceError {
    /// Whether this error is transient (retry may succeed).
    ///
    /// - `Timeout` → transient (server may be busy)
    /// - `Transport` → transient (network glitch)
    /// - `ServiceFailed` → permanent (server explicitly returned error)
    /// - `NoServer` → permanent (no server registered)
    pub fn is_transient(&self) -> bool {
        matches!(self, ServiceError::Timeout | ServiceError::Transport(_))
    }
}

impl From<ServiceError> for crate::error::HorusError {
    fn from(err: ServiceError) -> Self {
        let message = err.to_string();
        crate::error::HorusError::Contextual {
            message: format!("Service call failed: {}", message),
            source: Box::new(err),
        }
    }
}

/// Result type for service calls.
pub type ServiceResult<T> = Result<T, ServiceError>;

// ─── Metadata (for discovery / introspection) ────────────────────────────────

/// Metadata about a running service (returned by `horus service list`).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ServiceInfo {
    /// Service name (e.g. `"add_two_ints"`).
    pub name: String,
    /// Rust type name of the request type.
    pub request_type: String,
    /// Rust type name of the response type.
    pub response_type: String,
    /// Number of active server instances (typically 0 or 1).
    pub servers: usize,
    /// Number of known clients.
    pub clients: usize,
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::error::Error as StdError;

    // ── ServiceError construction and Display ────────────────────────────

    #[test]
    fn service_error_timeout_display() {
        let err = ServiceError::Timeout;
        assert_eq!(err.to_string(), "service call timed out");
    }

    #[test]
    fn service_error_service_failed_display() {
        let err = ServiceError::ServiceFailed("division by zero".to_string());
        assert_eq!(err.to_string(), "service returned error: division by zero");
    }

    #[test]
    fn service_error_no_server_display() {
        let err = ServiceError::NoServer;
        assert_eq!(err.to_string(), "no server is available for this service");
    }

    #[test]
    fn service_error_transport_display() {
        let err = ServiceError::Transport("shm segment unavailable".to_string());
        assert_eq!(err.to_string(), "transport error: shm segment unavailable");
    }

    #[test]
    fn service_error_service_failed_empty_message() {
        let err = ServiceError::ServiceFailed(String::new());
        assert_eq!(err.to_string(), "service returned error: ");
    }

    #[test]
    fn service_error_transport_empty_message() {
        let err = ServiceError::Transport(String::new());
        assert_eq!(err.to_string(), "transport error: ");
    }

    // ── ServiceError::is_transient ───────────────────────────────────────

    #[test]
    fn service_error_timeout_is_transient() {
        assert!(ServiceError::Timeout.is_transient());
    }

    #[test]
    fn service_error_transport_is_transient() {
        assert!(ServiceError::Transport("io".to_string()).is_transient());
    }

    #[test]
    fn service_error_service_failed_is_permanent() {
        assert!(!ServiceError::ServiceFailed("oops".to_string()).is_transient());
    }

    #[test]
    fn service_error_no_server_is_permanent() {
        assert!(!ServiceError::NoServer.is_transient());
    }

    // ── ServiceError → HorusError conversion ────────────────────────────

    #[test]
    fn service_error_converts_to_horus_error() {
        let svc_err = ServiceError::Timeout;
        let horus_err: crate::error::HorusError = svc_err.into();
        let msg = horus_err.to_string();
        assert!(
            msg.contains("Service call failed"),
            "expected 'Service call failed' in: {msg}"
        );
        assert!(msg.contains("timed out"), "expected 'timed out' in: {msg}");
    }

    #[test]
    fn service_error_conversion_preserves_source_chain() {
        let svc_err = ServiceError::NoServer;
        let horus_err: crate::error::HorusError = svc_err.into();
        // Contextual variant must expose a source()
        let src = horus_err.source();
        assert!(src.is_some(), "Contextual must expose source()");
    }

    #[test]
    fn service_error_clone_preserves_variant() {
        let err = ServiceError::ServiceFailed("bad input".to_string());
        let cloned = err.clone();
        assert!(matches!(cloned, ServiceError::ServiceFailed(ref s) if s == "bad input"));
    }

    #[test]
    fn service_error_debug_format() {
        let err = ServiceError::Timeout;
        let dbg = format!("{:?}", err);
        assert!(
            dbg.contains("Timeout"),
            "Debug output should mention variant: {dbg}"
        );
    }

    // ── ServiceResponse ──────────────────────────────────────────────────

    #[test]
    fn service_response_success_fields() {
        let resp = ServiceResponse::<i32>::success(42, 100);
        assert_eq!(resp.request_id, 42);
        assert!(resp.ok);
        assert_eq!(resp.payload, Some(100));
        assert!(resp.error.is_none());
    }

    #[test]
    fn service_response_failure_fields() {
        let resp = ServiceResponse::<i32>::failure(99, "kaboom");
        assert_eq!(resp.request_id, 99);
        assert!(!resp.ok);
        assert!(resp.payload.is_none());
        assert_eq!(resp.error.as_deref(), Some("kaboom"));
    }

    #[test]
    fn service_response_failure_empty_message() {
        let resp = ServiceResponse::<()>::failure(1, "");
        assert!(!resp.ok);
        assert_eq!(resp.error.as_deref(), Some(""));
    }

    #[test]
    fn service_response_success_with_zero_request_id() {
        let resp = ServiceResponse::<String>::success(0, "ok".to_string());
        assert_eq!(resp.request_id, 0);
        assert!(resp.ok);
    }

    #[test]
    fn service_response_success_with_max_request_id() {
        let resp = ServiceResponse::<u8>::success(u64::MAX, 255);
        assert_eq!(resp.request_id, u64::MAX);
        assert!(resp.ok);
        assert_eq!(resp.payload, Some(255));
    }

    #[test]
    fn service_response_clone() {
        let resp = ServiceResponse::<i32>::success(10, 42);
        let cloned = resp.clone();
        assert_eq!(cloned.request_id, 10);
        assert_eq!(cloned.payload, Some(42));
    }

    // ── ServiceRequest ───────────────────────────────────────────────────

    #[test]
    fn service_request_with_response_topic() {
        let req = ServiceRequest {
            request_id: 7,
            payload: "hello",
            response_topic: Some("my_svc.response.42".to_string()),
        };
        assert_eq!(req.request_id, 7);
        assert_eq!(req.payload, "hello");
        assert_eq!(req.response_topic.as_deref(), Some("my_svc.response.42"));
    }

    #[test]
    fn service_request_without_response_topic() {
        let req = ServiceRequest {
            request_id: 1,
            payload: 42i64,
            response_topic: None,
        };
        assert!(req.response_topic.is_none());
    }

    #[test]
    fn service_request_clone() {
        let req = ServiceRequest {
            request_id: 5,
            payload: vec![1, 2, 3],
            response_topic: Some("t".to_string()),
        };
        let cloned = req.clone();
        assert_eq!(cloned.request_id, 5);
        assert_eq!(cloned.payload, vec![1, 2, 3]);
        assert_eq!(cloned.response_topic.as_deref(), Some("t"));
    }

    #[test]
    fn service_request_serde_roundtrip() {
        let req = ServiceRequest {
            request_id: 123,
            payload: "test_payload".to_string(),
            response_topic: Some("svc.response.99".to_string()),
        };
        let json = serde_json::to_string(&req).unwrap();
        let deser: ServiceRequest<String> = serde_json::from_str(&json).unwrap();
        assert_eq!(deser.request_id, 123);
        assert_eq!(deser.payload, "test_payload");
        assert_eq!(deser.response_topic.as_deref(), Some("svc.response.99"));
    }

    #[test]
    fn service_request_serde_response_topic_defaults_to_none() {
        // response_topic has #[serde(default)] — omitting it from JSON should yield None
        let json = r#"{"request_id": 1, "payload": 42}"#;
        let deser: ServiceRequest<i32> = serde_json::from_str(json).unwrap();
        assert!(deser.response_topic.is_none());
    }

    #[test]
    fn service_response_serde_roundtrip_success() {
        let resp = ServiceResponse::<Vec<u8>>::success(77, vec![0xDE, 0xAD]);
        let json = serde_json::to_string(&resp).unwrap();
        let deser: ServiceResponse<Vec<u8>> = serde_json::from_str(&json).unwrap();
        assert_eq!(deser.request_id, 77);
        assert!(deser.ok);
        assert_eq!(deser.payload, Some(vec![0xDE, 0xAD]));
        assert!(deser.error.is_none());
    }

    #[test]
    fn service_response_serde_roundtrip_failure() {
        let resp = ServiceResponse::<()>::failure(88, "not found");
        let json = serde_json::to_string(&resp).unwrap();
        let deser: ServiceResponse<()> = serde_json::from_str(&json).unwrap();
        assert_eq!(deser.request_id, 88);
        assert!(!deser.ok);
        assert!(deser.payload.is_none());
        assert_eq!(deser.error.as_deref(), Some("not found"));
    }

    // ── ServiceInfo ──────────────────────────────────────────────────────

    #[test]
    fn service_info_construction() {
        let info = ServiceInfo {
            name: "edge_svc_a1".to_string(),
            request_type: "MyReq".to_string(),
            response_type: "MyRes".to_string(),
            servers: 1,
            clients: 3,
        };
        assert_eq!(info.name, "edge_svc_a1");
        assert_eq!(info.servers, 1);
        assert_eq!(info.clients, 3);
    }

    #[test]
    fn service_info_zero_servers_and_clients() {
        let info = ServiceInfo {
            name: "edge_svc_a2".to_string(),
            request_type: "()".to_string(),
            response_type: "()".to_string(),
            servers: 0,
            clients: 0,
        };
        assert_eq!(info.servers, 0);
        assert_eq!(info.clients, 0);
    }

    #[test]
    fn service_info_serde_roundtrip() {
        let info = ServiceInfo {
            name: "edge_svc_a3".to_string(),
            request_type: "Ping".to_string(),
            response_type: "Pong".to_string(),
            servers: 2,
            clients: 10,
        };
        let json = serde_json::to_string(&info).unwrap();
        let deser: ServiceInfo = serde_json::from_str(&json).unwrap();
        assert_eq!(deser.name, "edge_svc_a3");
        assert_eq!(deser.request_type, "Ping");
        assert_eq!(deser.response_type, "Pong");
        assert_eq!(deser.servers, 2);
        assert_eq!(deser.clients, 10);
    }

    #[test]
    fn service_info_clone() {
        let info = ServiceInfo {
            name: "edge_svc_a4".to_string(),
            request_type: "R".to_string(),
            response_type: "S".to_string(),
            servers: 1,
            clients: 1,
        };
        let cloned = info.clone();
        assert_eq!(cloned.name, info.name);
    }

    // ── Service trait default methods ────────────────────────────────────

    // A minimal manual Service impl for testing trait defaults.
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct EdgeReq {
        val: u32,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct EdgeRes {
        ok: bool,
    }

    struct EdgeService;

    impl Service for EdgeService {
        type Request = EdgeReq;
        type Response = EdgeRes;

        fn name() -> &'static str {
            "edge_service_types_test"
        }
    }

    #[test]
    fn service_trait_request_topic() {
        assert_eq!(
            EdgeService::request_topic(),
            "edge_service_types_test.request"
        );
    }

    #[test]
    fn service_trait_response_topic() {
        assert_eq!(
            EdgeService::response_topic(),
            "edge_service_types_test.response"
        );
    }

    #[test]
    fn service_trait_request_type_name() {
        let name = EdgeService::request_type_name();
        assert!(
            name.contains("EdgeReq"),
            "expected type name to contain 'EdgeReq', got: {name}"
        );
    }

    #[test]
    fn service_trait_response_type_name() {
        let name = EdgeService::response_type_name();
        assert!(
            name.contains("EdgeRes"),
            "expected type name to contain 'EdgeRes', got: {name}"
        );
    }

    // ── ServiceResult alias ──────────────────────────────────────────────

    #[test]
    fn service_result_ok_variant() {
        let result: ServiceResult<i32> = Ok(42);
        let Ok(value) = result else {
            panic!("expected Ok variant");
        };
        assert_eq!(value, 42);
    }

    #[test]
    fn service_result_err_variant() {
        let result: ServiceResult<i32> = Err(ServiceError::Timeout);
        let Err(err) = result else {
            panic!("expected Err variant");
        };
        assert!(matches!(err, ServiceError::Timeout));
    }

    // ── Edge: ServiceError with very long messages ───────────────────────

    #[test]
    fn service_error_service_failed_long_message() {
        let long_msg = "x".repeat(10_000);
        let err = ServiceError::ServiceFailed(long_msg.clone());
        let display = err.to_string();
        assert!(display.contains(&long_msg));
    }

    #[test]
    fn service_error_transport_unicode_message() {
        let err = ServiceError::Transport("conexion fallida".to_string());
        let display = err.to_string();
        assert!(display.contains("conexion fallida"));
    }

    // ── Edge: ServiceResponse with unusual payload types ─────────────────

    #[test]
    fn service_response_success_unit_payload() {
        let resp = ServiceResponse::<()>::success(1, ());
        assert!(resp.ok);
        assert_eq!(resp.payload, Some(()));
    }

    #[test]
    fn service_response_success_nested_option_payload() {
        let resp = ServiceResponse::<Option<i32>>::success(1, Some(42));
        assert_eq!(resp.payload, Some(Some(42)));
    }

    #[test]
    fn service_response_success_none_option_payload() {
        let resp = ServiceResponse::<Option<i32>>::success(1, None);
        assert_eq!(resp.payload, Some(None));
    }

    // ── Edge: manually crafted inconsistent response ─────────────────────

    #[test]
    fn service_response_manual_ok_true_but_no_payload() {
        // This is the degenerate case that clients handle
        let resp = ServiceResponse::<i32> {
            request_id: 1,
            ok: true,
            payload: None,
            error: None,
        };
        assert!(resp.ok);
        assert!(resp.payload.is_none());
    }

    #[test]
    fn service_response_manual_ok_false_but_has_payload() {
        // Inconsistent but structurally valid
        let resp = ServiceResponse::<i32> {
            request_id: 1,
            ok: false,
            payload: Some(999),
            error: Some("error but also data".to_string()),
        };
        assert!(!resp.ok);
        assert_eq!(resp.payload, Some(999));
    }

    #[test]
    fn service_response_manual_ok_false_no_error_message() {
        // The client code handles this with "unknown error" fallback
        let resp = ServiceResponse::<i32> {
            request_id: 1,
            ok: false,
            payload: None,
            error: None,
        };
        assert!(!resp.ok);
        assert!(resp.error.is_none());
    }
}
