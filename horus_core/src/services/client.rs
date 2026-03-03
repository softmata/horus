//! Service client implementations.
//!
//! Two client types are provided:
//!
//! - [`SyncServiceClient`] — blocking call, suitable for scripts, tests, and
//!   one-shot operations.
//! - [`AsyncServiceClient`] — non-blocking, returns a pending handle; suitable
//!   for integration inside HORUS nodes that cannot block their tick.
//!
//! # Example — sync client
//!
//! ```rust,ignore
//! use horus_core::services::SyncServiceClient;
//! use std::time::Duration;
//!
//! let client = SyncServiceClient::<AddTwoInts>::new()?;
//! let response = client.call(
//!     AddTwoIntsRequest { a: 3, b: 4 },
//!     Duration::from_secs(1),
//! )?;
//! println!("3 + 4 = {}", response.sum);
//! ```

use std::fmt::Debug;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::{de::DeserializeOwned, Serialize};

use crate::communication::Topic;
use crate::error::HorusResult;
use crate::services::types::{Service, ServiceError, ServiceRequest, ServiceResponse, ServiceResult};

// ─── Request ID generator ─────────────────────────────────────────────────────

/// Process-wide monotonic request ID counter.
static NEXT_REQUEST_ID: AtomicU64 = AtomicU64::new(1);

fn next_request_id() -> u64 {
    NEXT_REQUEST_ID.fetch_add(1, Ordering::Relaxed)
}

// ─── SyncServiceClient ────────────────────────────────────────────────────────

/// Blocking service client.
///
/// Creates topics on construction and reuses them for subsequent calls.
/// Thread-safe: the internal topics use atomic operations.
pub struct SyncServiceClient<S: Service> {
    req_topic: Topic<ServiceRequest<S::Request>>,
    res_topic: Topic<ServiceResponse<S::Response>>,
    /// How often to check for a response while blocking (default: 1 ms).
    poll_interval: Duration,
}

impl<S: Service> SyncServiceClient<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new synchronous client for service `S`.
    pub fn new() -> HorusResult<Self> {
        Self::with_poll_interval(Duration::from_millis(1))
    }

    /// Create a client with a custom polling interval.
    pub fn with_poll_interval(poll_interval: Duration) -> HorusResult<Self> {
        let req_topic = Topic::new(&S::request_topic())?;
        let res_topic = Topic::new(&S::response_topic())?;
        Ok(Self {
            req_topic,
            res_topic,
            poll_interval,
        })
    }

    /// Call the service synchronously.
    ///
    /// Blocks until a matching response is received or `timeout` elapses.
    pub fn call(
        &mut self,
        request: S::Request,
        timeout: Duration,
    ) -> ServiceResult<S::Response> {
        let request_id = next_request_id();

        // Send the request.
        self.req_topic.send(ServiceRequest {
            request_id,
            payload: request,
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
                        Ok(resp.payload.expect("ok response must have payload"))
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
                    Ok(Some(resp.payload.expect("ok response must have payload")))
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
    poll_interval: Duration,
}

impl<S: Service> AsyncServiceClient<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new async client for service `S`.
    pub fn new() -> HorusResult<Self> {
        Self::with_poll_interval(Duration::from_millis(1))
    }

    /// Create a client with a custom polling interval.
    pub fn with_poll_interval(poll_interval: Duration) -> HorusResult<Self> {
        let req_topic = Topic::new(&S::request_topic())?;
        let res_topic = Arc::new(Topic::new(&S::response_topic())?);
        Ok(Self {
            req_topic,
            res_topic,
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
