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
use crate::core::DurationExt;
use crate::error::HorusResult;
use crate::services::types::{Service, ServiceRequest, ServiceResponse};

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

        // Create typed topics up front to catch errors before spawning the thread.
        let mut req_topic: Topic<ServiceRequest<S::Request>> = Topic::new(&req_topic_name)?;
        let res_topic: Topic<ServiceResponse<S::Response>> = Topic::new(&res_topic_name)?;

        // Service name for the JSON gateway (horus service call).
        // The CLI resolves user-provided names (CamelCase or snake_case) to this.
        let svc_name = S::name().to_string();

        let handle = thread::Builder::new()
            .name(format!("horus_srv_{}", S::name()))
            .spawn(move || {
                // File-based JSON gateway: no Topic needed. Server polls a request file,
                // CLI writes JSON to it. Simple, no IPC migration issues.
                run_server_loop(
                    &mut req_topic,
                    &res_topic,
                    &handler,
                    &shutdown_clone,
                    poll_interval,
                    svc_name,
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
    svc_name: String,
) where
    Req: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
    Res: Clone + Debug + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    // Cache per-client response topics to avoid re-creating them on every request.
    let mut client_topics: std::collections::HashMap<String, Topic<ServiceResponse<Res>>> =
        std::collections::HashMap::new();

    // Pre-compute the gateway directory and request file path.
    // The CLI writes to .service_gateway/{svc_name}.request.json and the server
    // writes the response to .service_gateway/{svc_name}.response.{id}.json.
    let gateway_dir = crate::memory::shm_topics_dir().join(".service_gateway");

    loop {
        if shutdown.load(Ordering::SeqCst) {
            break;
        }

        // JSON gateway: process CLI requests (horus service call).
        // Uses a simple file-based gateway — no ring buffer, no migration issues.
        // CLI writes JSON to .service_gateway/{svc_name}.request.json
        // Server reads it, processes, writes response to .service_gateway/{svc_name}.response.{id}.json
        {
            let req_file = gateway_dir.join(format!("{}.request.json", svc_name));
            if let Ok(data) = std::fs::read(&req_file) {
                let _ = std::fs::remove_file(&req_file);
                if let Ok(json_req) = serde_json::from_slice::<ServiceRequest<serde_json::Value>>(&data) {
                    if let Ok(typed_payload) = serde_json::from_value::<Req>(json_req.payload) {
                        let request_id = json_req.request_id;
                        let response = match handler(typed_payload) {
                            Ok(payload) => {
                                let json_payload = serde_json::to_value(&payload)
                                    .unwrap_or(serde_json::Value::Null);
                                ServiceResponse::success(request_id, json_payload)
                            }
                            Err(err_msg) => ServiceResponse::failure(request_id, err_msg),
                        };
                        let res_file = gateway_dir.join(format!(
                            "{}.response.{}.json",
                            svc_name,
                            request_id
                        ));
                        let _ = serde_json::to_vec(&response).map(|bytes| {
                            let _ = std::fs::write(&res_file, &bytes);
                        });
                    }
                }
            }
        }

        // Drain ALL pending typed requests before sleeping.
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
                let client_topic = client_topics.entry(topic_name.clone()).or_insert_with(|| {
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

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::types::ServiceError;
    use serde::{Deserialize, Serialize};

    // ── Test service types ───────────────────────────────────────────────

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct SrvTestReq {
        input: String,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct SrvTestRes {
        output: String,
    }

    struct SrvTestService;

    impl Service for SrvTestService {
        type Request = SrvTestReq;
        type Response = SrvTestRes;

        fn name() -> &'static str {
            "srv_edge_test_svc"
        }
    }

    // ── ServiceServerBuilder defaults ────────────────────────────────────

    #[test]
    fn server_builder_default_poll_interval() {
        let builder = ServiceServerBuilder::<SrvTestService>::new();
        assert_eq!(
            builder.poll_interval,
            Duration::from_millis(5),
            "Default poll interval should be 5ms"
        );
    }

    #[test]
    fn server_builder_default_has_no_handler() {
        let builder = ServiceServerBuilder::<SrvTestService>::new();
        assert!(
            builder.handler.is_none(),
            "Default builder should have no handler"
        );
    }

    #[test]
    fn server_builder_default_trait() {
        let builder = ServiceServerBuilder::<SrvTestService>::default();
        assert!(builder.handler.is_none());
        assert_eq!(builder.poll_interval, Duration::from_millis(5));
    }

    // ── ServiceServerBuilder configuration ──────────────────────────────

    #[test]
    fn server_builder_custom_poll_interval() {
        let builder =
            ServiceServerBuilder::<SrvTestService>::new().poll_interval(Duration::from_millis(100));
        assert_eq!(builder.poll_interval, Duration::from_millis(100));
    }

    #[test]
    fn server_builder_zero_poll_interval() {
        let builder =
            ServiceServerBuilder::<SrvTestService>::new().poll_interval(Duration::ZERO);
        assert_eq!(builder.poll_interval, Duration::ZERO);
    }

    #[test]
    fn server_builder_large_poll_interval() {
        let builder =
            ServiceServerBuilder::<SrvTestService>::new().poll_interval(Duration::from_secs(3600));
        assert_eq!(builder.poll_interval, Duration::from_secs(3600));
    }

    #[test]
    fn server_builder_on_request_sets_handler() {
        let builder = ServiceServerBuilder::<SrvTestService>::new().on_request(|req| {
            Ok(SrvTestRes {
                output: req.input.to_uppercase(),
            })
        });
        assert!(
            builder.handler.is_some(),
            "on_request should set the handler"
        );
    }

    #[test]
    fn server_builder_handler_is_callable() {
        let builder = ServiceServerBuilder::<SrvTestService>::new().on_request(|req| {
            Ok(SrvTestRes {
                output: format!("echo: {}", req.input),
            })
        });
        let handler = builder.handler.unwrap();
        let result = handler(SrvTestReq {
            input: "hello".to_string(),
        });
        assert!(result.is_ok());
        assert_eq!(result.unwrap().output, "echo: hello");
    }

    #[test]
    fn server_builder_handler_can_return_error() {
        let builder = ServiceServerBuilder::<SrvTestService>::new()
            .on_request(|_req| Err("service busy".to_string()));
        let handler = builder.handler.unwrap();
        let result = handler(SrvTestReq {
            input: "test".to_string(),
        });
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "service busy");
    }

    #[test]
    fn server_builder_chained_configuration() {
        let builder = ServiceServerBuilder::<SrvTestService>::new()
            .poll_interval(Duration::from_millis(50))
            .on_request(|req| {
                Ok(SrvTestRes {
                    output: req.input,
                })
            });
        assert_eq!(builder.poll_interval, Duration::from_millis(50));
        assert!(builder.handler.is_some());
    }

    // ── ServiceServer build and lifecycle ────────────────────────────────

    #[test]
    fn server_build_without_handler_uses_default() {
        // Building without calling on_request should succeed (default handler
        // returns an error for every request).
        let server = ServiceServerBuilder::<SrvTestService>::new().build();
        assert!(server.is_ok(), "Building without handler should succeed");
        // Clean up
        drop(server);
    }

    #[test]
    fn server_build_with_handler_succeeds() {
        let server = ServiceServerBuilder::<SrvTestService>::new()
            .on_request(|req| {
                Ok(SrvTestRes {
                    output: req.input,
                })
            })
            .build();
        assert!(server.is_ok());
        drop(server);
    }

    #[test]
    fn server_name_matches_service() {
        let server = ServiceServerBuilder::<SrvTestService>::new()
            .on_request(|req| {
                Ok(SrvTestRes {
                    output: req.input,
                })
            })
            .build()
            .unwrap();
        assert_eq!(server.name, "srv_edge_test_svc");
        drop(server);
    }

    #[test]
    fn server_stop_is_idempotent() {
        let server = ServiceServerBuilder::<SrvTestService>::new()
            .on_request(|_| Ok(SrvTestRes { output: String::new() }))
            .build()
            .unwrap();
        // Calling stop multiple times should not panic
        server.stop();
        server.stop();
        server.stop();
        drop(server);
    }

    #[test]
    fn server_drop_shuts_down_cleanly() {
        let server = ServiceServerBuilder::<SrvTestService>::new()
            .on_request(|_| Ok(SrvTestRes { output: String::new() }))
            .build()
            .unwrap();
        // Drop should not hang or panic
        drop(server);
    }

    // ── Default handler behavior ─────────────────────────────────────────

    #[test]
    fn default_handler_returns_error_string() {
        // Simulate what the unwrap_or_else fallback handler does
        let handler: RequestHandler<SrvTestReq, SrvTestRes> =
            Box::new(|_req| Err("no handler registered for this service".to_string()));
        let result = handler(SrvTestReq {
            input: "test".to_string(),
        });
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            "no handler registered for this service"
        );
    }

    // ── ServiceResponse construction used by server loop ─────────────────

    #[test]
    fn server_loop_success_response_format() {
        let resp = ServiceResponse::<SrvTestRes>::success(
            42,
            SrvTestRes {
                output: "done".to_string(),
            },
        );
        assert_eq!(resp.request_id, 42);
        assert!(resp.ok);
        assert_eq!(resp.payload.as_ref().unwrap().output, "done");
        assert!(resp.error.is_none());
    }

    #[test]
    fn server_loop_failure_response_format() {
        let resp = ServiceResponse::<SrvTestRes>::failure(99, "handler panicked");
        assert_eq!(resp.request_id, 99);
        assert!(!resp.ok);
        assert!(resp.payload.is_none());
        assert_eq!(resp.error.as_deref(), Some("handler panicked"));
    }

    // ── End-to-end: server + client ──────────────────────────────────────

    // A separate service for the E2E test to avoid topic name collisions.
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct E2eReq {
        a: i64,
        b: i64,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct E2eRes {
        sum: i64,
    }

    struct E2eAddService;

    impl Service for E2eAddService {
        type Request = E2eReq;
        type Response = E2eRes;

        fn name() -> &'static str {
            "srv_e2e_add_svc"
        }
    }

    #[test]
    fn server_client_roundtrip() {
        let _server = ServiceServerBuilder::<E2eAddService>::new()
            .poll_interval(Duration::from_millis(1))
            .on_request(|req| Ok(E2eRes { sum: req.a + req.b }))
            .build()
            .unwrap();

        let mut client =
            crate::services::client::ServiceClient::<E2eAddService>::with_poll_interval(
                Duration::from_millis(1),
            )
            .unwrap();

        let result = client.call(E2eReq { a: 3, b: 4 }, Duration::from_secs(5));
        assert!(result.is_ok(), "E2E call should succeed: {:?}", result.err());
        assert_eq!(result.unwrap().sum, 7);
    }

    // A service for the error-returning E2E test.
    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct E2eFailReq {
        should_fail: bool,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    struct E2eFailRes {
        ok: bool,
    }

    struct E2eFailService;

    impl Service for E2eFailService {
        type Request = E2eFailReq;
        type Response = E2eFailRes;

        fn name() -> &'static str {
            "srv_e2e_fail_svc"
        }
    }

    #[test]
    fn server_returns_error_to_client() {
        let _server = ServiceServerBuilder::<E2eFailService>::new()
            .poll_interval(Duration::from_millis(1))
            .on_request(|req| {
                if req.should_fail {
                    Err("intentional failure".to_string())
                } else {
                    Ok(E2eFailRes { ok: true })
                }
            })
            .build()
            .unwrap();

        let mut client =
            crate::services::client::ServiceClient::<E2eFailService>::with_poll_interval(
                Duration::from_millis(1),
            )
            .unwrap();

        let result = client.call(
            E2eFailReq { should_fail: true },
            Duration::from_secs(2),
        );
        assert!(result.is_err());
        match result.unwrap_err() {
            ServiceError::ServiceFailed(msg) => {
                assert_eq!(msg, "intentional failure");
            }
            other => panic!("Expected ServiceFailed, got: {:?}", other),
        }
    }

    #[test]
    fn call_optional_returns_some_on_success() {
        // Separate service to avoid topic collisions.
        #[derive(Clone, Debug, Serialize, Deserialize)]
        struct OptReq {
            v: i32,
        }
        #[derive(Clone, Debug, Serialize, Deserialize)]
        struct OptRes {
            v: i32,
        }
        struct OptService;
        impl Service for OptService {
            type Request = OptReq;
            type Response = OptRes;
            fn name() -> &'static str {
                "srv_e2e_opt_svc"
            }
        }

        let _server = ServiceServerBuilder::<OptService>::new()
            .poll_interval(Duration::from_millis(1))
            .on_request(|req| Ok(OptRes { v: req.v * 2 }))
            .build()
            .unwrap();

        let mut client =
            crate::services::client::ServiceClient::<OptService>::with_poll_interval(
                Duration::from_millis(1),
            )
            .unwrap();

        let result = client.call_optional(OptReq { v: 5 }, Duration::from_secs(2));
        assert!(result.is_ok());
        let inner = result.unwrap();
        assert!(inner.is_some());
        assert_eq!(inner.unwrap().v, 10);
    }
}
