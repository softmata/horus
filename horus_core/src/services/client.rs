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
use crate::core::DurationExt;
use crate::error::{HorusResult, RetryConfig};
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

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    // ── Test service types ───────────────────────────────────────────────

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct CliTestReq {
        value: i64,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct CliTestRes {
        result: i64,
    }

    struct CliTestService;

    impl Service for CliTestService {
        type Request = CliTestReq;
        type Response = CliTestRes;

        fn name() -> &'static str {
            "cli_edge_test_svc"
        }
    }

    // ── Request ID generator ─────────────────────────────────────────────

    #[test]
    fn request_ids_are_monotonic() {
        let id1 = next_request_id();
        let id2 = next_request_id();
        let id3 = next_request_id();
        assert!(id2 > id1, "IDs must be monotonically increasing");
        assert!(id3 > id2, "IDs must be monotonically increasing");
    }

    #[test]
    fn client_ids_are_monotonic() {
        let id1 = next_client_id();
        let id2 = next_client_id();
        assert!(id2 > id1, "Client IDs must be monotonically increasing");
    }

    #[test]
    fn request_ids_are_unique_across_calls() {
        let ids: Vec<u64> = (0..100).map(|_| next_request_id()).collect();
        let mut deduped = ids.clone();
        deduped.sort();
        deduped.dedup();
        assert_eq!(ids.len(), deduped.len(), "All request IDs must be unique");
    }

    // ── ServiceClient construction ───────────────────────────────────────

    #[test]
    fn service_client_new_succeeds() {
        let client = ServiceClient::<CliTestService>::new();
        assert!(client.is_ok(), "ServiceClient::new() should succeed");
    }

    #[test]
    fn service_client_custom_poll_interval() {
        let client = ServiceClient::<CliTestService>::with_poll_interval(Duration::from_millis(50));
        assert!(
            client.is_ok(),
            "ServiceClient with custom poll interval should succeed"
        );
    }

    #[test]
    fn service_client_zero_poll_interval() {
        // Zero poll interval is technically valid (busy-spin)
        let client = ServiceClient::<CliTestService>::with_poll_interval(Duration::ZERO);
        assert!(
            client.is_ok(),
            "ServiceClient with zero poll interval should succeed"
        );
    }

    #[test]
    fn service_client_large_poll_interval() {
        let client =
            ServiceClient::<CliTestService>::with_poll_interval(Duration::from_secs(3600));
        assert!(
            client.is_ok(),
            "ServiceClient with very large poll interval should succeed"
        );
    }

    // ── ServiceClient::call — timeout behavior ──────────────────────────

    #[test]
    fn service_client_call_times_out_with_no_server() {
        let mut client = ServiceClient::<CliTestService>::new().unwrap();
        let result = client.call(CliTestReq { value: 42 }, Duration::from_millis(5));
        assert!(result.is_err());
        assert!(
            matches!(result.unwrap_err(), ServiceError::Timeout),
            "Call with no server should timeout"
        );
    }

    #[test]
    fn service_client_call_zero_timeout_returns_timeout() {
        let mut client = ServiceClient::<CliTestService>::new().unwrap();
        let result = client.call(CliTestReq { value: 1 }, Duration::ZERO);
        assert!(result.is_err());
        assert!(
            matches!(result.unwrap_err(), ServiceError::Timeout),
            "Zero timeout should return Timeout immediately"
        );
    }

    // ── ServiceClient::call_optional ─────────────────────────────────────

    #[test]
    fn service_client_call_optional_returns_none_on_timeout() {
        let mut client = ServiceClient::<CliTestService>::new().unwrap();
        let result = client.call_optional(CliTestReq { value: 1 }, Duration::from_millis(5));
        assert!(result.is_ok());
        assert!(
            result.unwrap().is_none(),
            "call_optional should return Ok(None) on timeout"
        );
    }

    #[test]
    fn service_client_call_optional_zero_timeout() {
        let mut client = ServiceClient::<CliTestService>::new().unwrap();
        let result = client.call_optional(CliTestReq { value: 1 }, Duration::ZERO);
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    // ── Multiple clients get distinct response topics ────────────────────

    #[test]
    fn multiple_clients_get_unique_response_topics() {
        let c1 = ServiceClient::<CliTestService>::new().unwrap();
        let c2 = ServiceClient::<CliTestService>::new().unwrap();
        assert_ne!(
            c1.response_topic_name, c2.response_topic_name,
            "Each client must get a unique per-client response topic"
        );
    }

    #[test]
    fn client_response_topic_contains_service_name() {
        let client = ServiceClient::<CliTestService>::new().unwrap();
        assert!(
            client
                .response_topic_name
                .starts_with("cli_edge_test_svc.response."),
            "Response topic should start with service response prefix, got: {}",
            client.response_topic_name
        );
    }

    // ── AsyncServiceClient construction ──────────────────────────────────

    #[test]
    fn async_service_client_new_succeeds() {
        let client = AsyncServiceClient::<CliTestService>::new();
        assert!(client.is_ok());
    }

    #[test]
    fn async_service_client_custom_poll_interval() {
        let client =
            AsyncServiceClient::<CliTestService>::with_poll_interval(Duration::from_millis(100));
        assert!(client.is_ok());
    }

    #[test]
    fn async_client_response_topic_contains_service_name() {
        let client = AsyncServiceClient::<CliTestService>::new().unwrap();
        assert!(
            client
                .response_topic_name
                .starts_with("cli_edge_test_svc.response."),
            "Async client response topic should start with service response prefix, got: {}",
            client.response_topic_name
        );
    }

    // ── PendingServiceCall ───────────────────────────────────────────────

    #[test]
    fn pending_call_is_expired_after_zero_timeout() {
        let mut client = AsyncServiceClient::<CliTestService>::new().unwrap();
        let pending = client.call_async(CliTestReq { value: 1 }, Duration::ZERO);
        // A zero-timeout pending call should be expired almost immediately.
        // Sleep briefly to guarantee the deadline has passed.
        std::thread::sleep(Duration::from_millis(1));
        assert!(pending.is_expired());
    }

    #[test]
    fn pending_call_not_expired_with_large_timeout() {
        let mut client = AsyncServiceClient::<CliTestService>::new().unwrap();
        let pending = client.call_async(CliTestReq { value: 1 }, Duration::from_secs(3600));
        assert!(!pending.is_expired());
    }

    #[test]
    fn pending_call_check_returns_timeout_after_expiry() {
        let mut client = AsyncServiceClient::<CliTestService>::new().unwrap();
        let mut pending = client.call_async(CliTestReq { value: 1 }, Duration::ZERO);
        std::thread::sleep(Duration::from_millis(1));
        let result = pending.check();
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), ServiceError::Timeout));
    }

    #[test]
    fn pending_call_check_returns_none_when_no_response_yet() {
        let mut client = AsyncServiceClient::<CliTestService>::new().unwrap();
        let mut pending = client.call_async(CliTestReq { value: 1 }, Duration::from_secs(60));
        // No server running, so no response — but not yet timed out
        let result = pending.check();
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn pending_call_wait_returns_timeout_with_zero_duration() {
        let mut client = AsyncServiceClient::<CliTestService>::new().unwrap();
        let pending = client.call_async(CliTestReq { value: 1 }, Duration::ZERO);
        std::thread::sleep(Duration::from_millis(1));
        let result = pending.wait();
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), ServiceError::Timeout));
    }

    // ── ServiceClient::call_resilient ────────────────────────────────────

    #[test]
    fn call_resilient_exhausts_retries_on_timeout() {
        let mut client =
            ServiceClient::<CliTestService>::with_poll_interval(Duration::from_millis(1)).unwrap();
        // With 0 retries and tiny timeout, should fail on first attempt
        let config = RetryConfig::new(0, Duration::from_millis(1));
        let result = client.call_resilient_with(
            CliTestReq { value: 1 },
            Duration::from_millis(1),
            config,
        );
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), ServiceError::Timeout));
    }

    // ── Service trait topic methods ──────────────────────────────────────

    #[test]
    fn service_name_is_snake_case() {
        assert_eq!(CliTestService::name(), "cli_edge_test_svc");
    }

    #[test]
    fn request_topic_format() {
        assert_eq!(
            CliTestService::request_topic(),
            "cli_edge_test_svc.request"
        );
    }

    #[test]
    fn response_topic_format() {
        assert_eq!(
            CliTestService::response_topic(),
            "cli_edge_test_svc.response"
        );
    }
}
