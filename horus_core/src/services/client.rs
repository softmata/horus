//! Service client implementations.
//!
//! Two client types are provided:
//!
//! - [`ServiceClient`] — blocking call, suitable for scripts, tests, and
//!   one-shot operations.
//! - [`AsyncServiceClient`] — non-blocking, returns a pending handle; suitable
//!   for integration inside HORUS nodes that cannot block their tick.
//!
//! # Example — sync client
//!
//! ```rust,ignore
//! use horus_core::services::ServiceClient;
//! use std::time::Duration;
//!
//! let client = ServiceClient::<AddTwoInts>::new()?;
//! let response = client.call(
//!     AddTwoIntsRequest { a: 3, b: 4 },
//!     1_u64.secs(),
//! )?;
//! println!("3 + 4 = {}", response.sum);
//! ```

use std::fmt::Debug;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{de::DeserializeOwned, Serialize};

use crate::communication::Topic;
use crate::error::{HorusResult, RetryConfig};
use crate::core::DurationExt;
use crate::services::types::{
    Service, ServiceError, ServiceRequest, ServiceResponse, ServiceResult,
};

// ─── Request ID generator ─────────────────────────────────────────────────────

/// Process-wide monotonic request ID counter.
static NEXT_REQUEST_ID: AtomicU64 = AtomicU64::new(1);

fn next_request_id() -> u64 {
    NEXT_REQUEST_ID.fetch_add(1, Ordering::Relaxed)
}

/// Process-wide monotonic client ID counter for per-client response topics.
static NEXT_CLIENT_ID: AtomicU64 = AtomicU64::new(1);

fn next_client_id() -> u64 {
    NEXT_CLIENT_ID.fetch_add(1, Ordering::Relaxed)
}

// ─── ServiceClient ────────────────────────────────────────────────────────

/// Blocking service client.
///
/// Each client creates its own per-client response topic to eliminate
/// contention when multiple clients call the same service concurrently.
/// The server reads the `response_topic` field from the request and
/// publishes the response to the client's dedicated topic.
///
/// Topic naming: `{service}.response.{client_id}`
pub struct ServiceClient<S: Service> {
    req_topic: Topic<ServiceRequest<S::Request>>,
    res_topic: Topic<ServiceResponse<S::Response>>,
    /// Per-client response topic name (e.g., "add_two_ints.response.42")
    response_topic_name: String,
    /// How often to check for a response while blocking (default: 1 ms).
    poll_interval: Duration,
}

impl<S: Service> ServiceClient<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new synchronous client for service `S`.
    pub fn new() -> HorusResult<Self> {
        Self::with_poll_interval(1_u64.ms())
    }

    /// Create a client with a custom polling interval.
    pub fn with_poll_interval(poll_interval: Duration) -> HorusResult<Self> {
        let req_topic = Topic::new_with_kind(
            &S::request_topic(),
            crate::communication::TopicKind::ServiceRequest as u8,
        )?;

        // Per-client response topic: eliminates contention when multiple
        // clients call the same service. Each client polls its own topic.
        let client_id = next_client_id();
        let response_topic_name = format!("{}.{}", S::response_topic(), client_id);
        let res_topic = Topic::new_with_kind(
            &response_topic_name,
            crate::communication::TopicKind::ServiceResponse as u8,
        )?;

        Ok(Self {
            req_topic,
            res_topic,
            response_topic_name,
            poll_interval,
        })
    }

    /// Call the service synchronously.
    ///
    /// Blocks until a matching response is received or `timeout` elapses.
    /// The response is published to this client's per-client response topic,
    /// eliminating contention with other concurrent clients.
    pub fn call(&mut self, request: S::Request, timeout: Duration) -> ServiceResult<S::Response> {
        let request_id = next_request_id();

        // Send the request with per-client response topic.
        self.req_topic.send(ServiceRequest {
            request_id,
            payload: request,
            response_topic: Some(self.response_topic_name.clone()),
        });

        // Poll for a matching response.
        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(ServiceError::Timeout);
            }

            if let Some(resp) = self.res_topic.recv() {
                if resp.request_id == request_id {
                    return if resp.ok {
                        Ok(resp.payload.ok_or(ServiceError::ServiceFailed(
                            "server returned ok=true but no payload".to_string(),
                        ))?)
                    } else {
                        Err(ServiceError::ServiceFailed(
                            resp.error.unwrap_or_else(|| "unknown error".to_string()),
                        ))
                    };
                }
                // Response was for a different request — keep polling.
            }

            std::thread::sleep(self.poll_interval);
        }
    }

    /// Call the service with automatic retry on transient errors.
    ///
    /// Uses default retry config (3 retries, 10ms initial backoff, 2x multiplier).
    /// Only retries on transient errors (`Timeout`, `Transport`).
    /// Permanent errors (`ServiceFailed`, `NoServer`) propagate immediately.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let resp = client.call_resilient(
    ///     AddTwoIntsRequest { a: 3, b: 4 },
    ///     1_u64.secs(),
    /// )?;
    /// ```
    pub fn call_resilient(
        &mut self,
        request: S::Request,
        timeout: Duration,
    ) -> ServiceResult<S::Response> {
        self.call_resilient_with(request, timeout, RetryConfig::default())
    }

    /// Call the service with automatic retry using a custom retry config.
    ///
    /// Only retries on transient errors (`Timeout`, `Transport`).
    /// Permanent errors (`ServiceFailed`, `NoServer`) propagate immediately.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let config = RetryConfig::new(5, 50_u64.ms());
    /// let resp = client.call_resilient_with(
    ///     AddTwoIntsRequest { a: 3, b: 4 },
    ///     1_u64.secs(),
    ///     config,
    /// )?;
    /// ```
    pub fn call_resilient_with(
        &mut self,
        request: S::Request,
        timeout: Duration,
        config: RetryConfig,
    ) -> ServiceResult<S::Response> {
        let mut backoff = config.initial_backoff();

        for attempt in 0..=config.max_retries() {
            match self.call(request.clone(), timeout) {
                Ok(resp) => return Ok(resp),
                Err(e) => {
                    if attempt == config.max_retries() || !e.is_transient() {
                        return Err(e);
                    }
                    std::thread::sleep(backoff);
                    backoff = Duration::from_secs_f64(
                        (backoff.as_secs_f64() * config.backoff_multiplier())
                            .min(config.max_backoff().as_secs_f64()),
                    );
                }
            }
        }

        unreachable!()
    }

    /// Call the service and return `Ok(None)` on timeout instead of `Err`.
    pub fn call_optional(
        &mut self,
        request: S::Request,
        timeout: Duration,
    ) -> ServiceResult<Option<S::Response>> {
        match self.call(request, timeout) {
            Ok(resp) => Ok(Some(resp)),
            Err(ServiceError::Timeout) => Ok(None),
            Err(e) => Err(e),
        }
    }
}

// ─── AsyncServiceClient ───────────────────────────────────────────────────────

/// Pending call handle returned by [`AsyncServiceClient::call_async`].
///
/// Poll [`PendingServiceCall::check`] until it returns `Some`, or use
/// [`PendingServiceCall::wait`] to block.
pub struct PendingServiceCall<Res>
where
    Res: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    request_id: u64,
    res_topic: Arc<Topic<ServiceResponse<Res>>>,
    sent_at: Instant,
    timeout: Duration,
    poll_interval: Duration,
}

impl<Res> PendingServiceCall<Res>
where
    Res: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Check whether the response has arrived (non-blocking).
    ///
    /// Returns:
    /// - `Ok(Some(res))` — response received.
    /// - `Ok(None)` — still waiting.
    /// - `Err(Timeout)` — deadline exceeded.
    /// - `Err(ServiceFailed)` — server returned an error.
    pub fn check(&mut self) -> ServiceResult<Option<Res>> {
        if self.sent_at.elapsed() >= self.timeout {
            return Err(ServiceError::Timeout);
        }

        if let Some(resp) = self.res_topic.recv() {
            if resp.request_id == self.request_id {
                return if resp.ok {
                    Ok(Some(resp.payload.ok_or(ServiceError::ServiceFailed(
                            "server returned ok=true but no payload".to_string(),
                        ))?))
                } else {
                    Err(ServiceError::ServiceFailed(
                        resp.error.unwrap_or_else(|| "unknown error".to_string()),
                    ))
                };
            }
        }
        Ok(None)
    }

    /// Block until the response arrives or the timeout elapses.
    pub fn wait(mut self) -> ServiceResult<Res> {
        loop {
            match self.check()? {
                Some(res) => return Ok(res),
                None => std::thread::sleep(self.poll_interval),
            }
        }
    }

    /// Whether the deadline has passed (the call may have timed out).
    pub fn is_expired(&self) -> bool {
        self.sent_at.elapsed() >= self.timeout
    }
}

/// Non-blocking service client.
///
/// Sends a request and returns a [`PendingServiceCall`] handle that can be
/// checked each tick without blocking the scheduler.
pub struct AsyncServiceClient<S: Service> {
    req_topic: Topic<ServiceRequest<S::Request>>,
    res_topic: Arc<Topic<ServiceResponse<S::Response>>>,
    response_topic_name: String,
    poll_interval: Duration,
}

impl<S: Service> AsyncServiceClient<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new async client for service `S`.
    pub fn new() -> HorusResult<Self> {
        Self::with_poll_interval(1_u64.ms())
    }

    /// Create a client with a custom polling interval.
    pub fn with_poll_interval(poll_interval: Duration) -> HorusResult<Self> {
        let req_topic = Topic::new(&S::request_topic())?;
        // Per-client response topic for async client
        let client_id = next_client_id();
        let response_topic_name = format!("{}.{}", S::response_topic(), client_id);
        let res_topic = Arc::new(Topic::new(&response_topic_name)?);
        Ok(Self {
            req_topic,
            res_topic,
            response_topic_name,
            poll_interval,
        })
    }

    /// Send a request and immediately return a pending handle.
    ///
    /// The caller is responsible for polling the handle until it resolves.
    pub fn call_async(
        &mut self,
        request: S::Request,
        timeout: Duration,
    ) -> PendingServiceCall<S::Response> {
        let request_id = next_request_id();
        self.req_topic.send(ServiceRequest {
            request_id,
            payload: request,
            response_topic: Some(self.response_topic_name.clone()),
        });
        PendingServiceCall {
            request_id,
            res_topic: self.res_topic.clone(),
            sent_at: Instant::now(),
            timeout,
            poll_interval: self.poll_interval,
        }
    }
}
