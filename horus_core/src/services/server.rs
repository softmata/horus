//! Service server implementation.
//!
//! A [`ServiceServer`] listens on the service's request topic, calls a
//! user-provided handler for each incoming request, and publishes the result
//! on the response topic.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::services::{ServiceServerBuilder, ServiceResult};
//! use horus_core::service;
//!
//! service! {
//!     AddTwoInts {
//!         request { a: i64, b: i64 }
//!         response { sum: i64 }
//!     }
//! }
//!
//! let server = ServiceServerBuilder::<AddTwoInts>::new()
//!     .on_request(|req| Ok(AddTwoIntsResponse { sum: req.a + req.b }))
//!     .build()?;
//!
//! // The server now runs in a background thread.
//! // Keep the handle alive while the server should be active.
//! ```

use std::fmt::Debug;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use serde::{de::DeserializeOwned, Serialize};

use crate::communication::Topic;
use crate::error::HorusResult;
use crate::services::types::{Service, ServiceRequest, ServiceResponse};
use crate::core::DurationExt;

// ─── Handler type alias ───────────────────────────────────────────────────────

/// Handler function type for service requests.
///
/// Takes the request payload and returns either a response or an error string.
pub type RequestHandler<Req, Res> = Box<dyn Fn(Req) -> Result<Res, String> + Send + Sync + 'static>;

// ─── ServiceServer ────────────────────────────────────────────────────────────

/// A running service server.
///
/// Dropping this handle shuts down the background polling thread.
pub struct ServiceServer<S: Service> {
    _shutdown: Arc<AtomicBool>,
    _thread: Option<thread::JoinHandle<()>>,
    _phantom: std::marker::PhantomData<S>,
    /// Service name for display.
    pub name: &'static str,
}

impl<S: Service> ServiceServer<S> {
    /// Stop the server early (happens automatically on drop).
    pub fn stop(&self) {
        self._shutdown.store(true, Ordering::SeqCst);
    }
}

impl<S: Service> Drop for ServiceServer<S> {
    fn drop(&mut self) {
        self._shutdown.store(true, Ordering::SeqCst);
        if let Some(handle) = self._thread.take() {
            let _ = handle.join();
        }
    }
}

// ─── ServiceServerBuilder ─────────────────────────────────────────────────────

/// Builder for [`ServiceServer`].
pub struct ServiceServerBuilder<S: Service> {
    handler: Option<RequestHandler<S::Request, S::Response>>,
    /// How often to poll the request topic (default: 5 ms).
    poll_interval: Duration,
}

impl<S: Service> ServiceServerBuilder<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new builder for service `S`.
    pub fn new() -> Self {
        Self {
            handler: None,
            poll_interval: 5_u64.ms(),
        }
    }

    /// Register the handler that is called for each incoming request.
    ///
    /// The handler receives the request payload and should return either a
    /// response payload (`Ok`) or an error message (`Err`).
    pub fn on_request<F>(mut self, handler: F) -> Self
    where
        F: Fn(S::Request) -> Result<S::Response, String> + Send + Sync + 'static,
    {
        self.handler = Some(Box::new(handler));
        self
    }

    /// Override the polling interval (default: 5 ms).
    pub fn poll_interval(mut self, interval: Duration) -> Self {
        self.poll_interval = interval;
        self
    }

    /// Build and start the service server.
    ///
    /// Returns an error if the request/response topics cannot be created.
    pub fn build(self) -> HorusResult<ServiceServer<S>> {
        let handler = self.handler.unwrap_or_else(|| {
            Box::new(|_req| Err("no handler registered for this service".to_string()))
        });

        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_clone = shutdown.clone();
        let poll_interval = self.poll_interval;

        let req_topic_name = S::request_topic();
        let res_topic_name = S::response_topic();

        // Create topics up front to catch errors before spawning the thread.
        let mut req_topic: Topic<ServiceRequest<S::Request>> = Topic::new(&req_topic_name)?;
        let res_topic: Topic<ServiceResponse<S::Response>> = Topic::new(&res_topic_name)?;

        let handle = thread::Builder::new()
            .name(format!("horus_srv_{}", S::name()))
            .spawn(move || {
                run_server_loop(
                    &mut req_topic,
                    &res_topic,
                    &handler,
                    &shutdown_clone,
                    poll_interval,
                );
            })
            .map_err(|e| {
                crate::error::HorusError::Config(crate::error::ConfigError::Other(format!(
                    "failed to spawn service server thread: {}",
                    e
                )))
            })?;

        Ok(ServiceServer {
            _shutdown: shutdown,
            _thread: Some(handle),
            _phantom: std::marker::PhantomData,
            name: S::name(),
        })
    }
}

impl<S: Service> Default for ServiceServerBuilder<S>
where
    S::Request: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    S::Response: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    fn default() -> Self {
        Self::new()
    }
}

// ─── Server loop ─────────────────────────────────────────────────────────────

fn run_server_loop<Req, Res>(
    req_topic: &mut Topic<ServiceRequest<Req>>,
    res_topic: &Topic<ServiceResponse<Res>>,
    handler: &RequestHandler<Req, Res>,
    shutdown: &AtomicBool,
    poll_interval: Duration,
) where
    Req: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    Res: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    // Cache per-client response topics to avoid re-creating them on every request.
    // Key: client response topic name, Value: Topic handle.
    let mut client_topics: std::collections::HashMap<String, Topic<ServiceResponse<Res>>> =
        std::collections::HashMap::new();

    loop {
        if shutdown.load(Ordering::SeqCst) {
            break;
        }

        // Drain ALL pending requests before sleeping.
        let mut processed = 0;
        while let Some(req) = req_topic.recv() {
            let request_id = req.request_id;
            let client_response_topic = req.response_topic;

            let response = match handler(req.payload) {
                Ok(payload) => ServiceResponse::success(request_id, payload),
                Err(err_msg) => ServiceResponse::failure(request_id, err_msg),
            };

            // Route response to per-client topic if specified,
            // otherwise fall back to shared response topic.
            if let Some(ref topic_name) = client_response_topic {
                let client_topic = client_topics
                    .entry(topic_name.clone())
                    .or_insert_with(|| {
                        Topic::new_with_kind(
                            topic_name,
                            crate::communication::TopicKind::ServiceResponse as u8,
                        )
                        .unwrap_or_else(|_| {
                            // Fallback: if per-client topic creation fails,
                            // this shouldn't happen in normal operation.
                            panic!("Failed to create per-client response topic: {}", topic_name);
                        })
                    });
                client_topic.send(response);
            } else {
                // Legacy path: shared response topic (for backward compat)
                res_topic.send(response);
            }

            processed += 1;

            // Safety valve: don't starve the shutdown check.
            if processed >= 1000 {
                break;
            }
        }

        thread::sleep(poll_interval);
    }
}
