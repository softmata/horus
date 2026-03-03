//! Core types for the HORUS service system.
//!
//! Services provide synchronous request/response communication — the HORUS
//! equivalent of ROS2 services.  A service call blocks the caller until
//! the server returns a response (or a timeout elapses).
//!
//! Internally each service is backed by two topics:
//! - `{name}/request`  — clients publish `ServiceRequest<Req>` here
//! - `{name}/response` — server publishes `ServiceResponse<Res>` here
//!
//! Correlation is done via a monotonically-increasing `request_id` that
//! the client embeds in the request and the server echoes in the response.

use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::fmt::Debug;

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
/// let client = SyncServiceClient::<AddTwoInts>::new()?;
/// let resp = client.call(AddTwoIntsRequest { a: 3, b: 4 }, Duration::from_secs(1))?;
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
        format!("{}/request", Self::name())
    }

    /// Full topic name for the response channel.
    fn response_topic() -> String {
        format!("{}/response", Self::name())
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
#[derive(Debug, Clone)]
pub enum ServiceError {
    /// The call timed out waiting for a response.
    Timeout,
    /// The server processed the request but returned an error.
    ServiceFailed(String),
    /// No server has registered for this service.
    NoServer,
    /// Topic I/O error.
    Transport(String),
}

impl std::fmt::Display for ServiceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout => write!(f, "service call timed out"),
            Self::ServiceFailed(msg) => write!(f, "service returned error: {}", msg),
            Self::NoServer => write!(f, "no server is available for this service"),
            Self::Transport(msg) => write!(f, "transport error: {}", msg),
        }
    }
}

impl std::error::Error for ServiceError {}

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
