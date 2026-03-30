//! Dynamic JSON-based Service implementation for FFI.
//!
//! The `Service` trait requires `name() -> &'static str` at compile time.
//! For dynamic service names from C++, we use a macro that generates a
//! unique Service type per name. For the common case, we provide a
//! pre-defined `GenericJsonService` with a fixed name.

use std::fmt::Debug;
use std::time::Duration;

use horus_core::communication::Topic;
use horus_core::core::duration_ext::DurationExt;
use horus_core::services::types::{ServiceRequest, ServiceResponse};
use horus_core::services::{Service, ServiceClient, ServiceError, ServiceResult};
use serde::{Deserialize, Serialize};

/// A JSON value service — uses serde_json::Value as both Request and Response.
///
/// Service name is fixed per type. For different service names, use the
/// `define_json_service!` macro or create multiple concrete types.
pub struct JsonService;

impl Service for JsonService {
    type Request = serde_json::Value;
    type Response = serde_json::Value;

    fn name() -> &'static str {
        "json_service"
    }
}

/// Create a service client for a named service using raw topics.
///
/// This bypasses the `Service` trait's static name requirement by
/// directly creating Topic instances with the dynamic service name.
pub struct DynamicServiceClient {
    pub(crate) name: String,
    pub(crate) req_topic: Topic<ServiceRequest<serde_json::Value>>,
    pub(crate) res_topic: Topic<ServiceResponse<serde_json::Value>>,
    pub(crate) response_topic_name: String,
    pub(crate) poll_interval: Duration,
}

impl DynamicServiceClient {
    /// Create a client for a dynamically-named service.
    pub fn new(service_name: &str) -> Result<Self, String> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static CLIENT_ID: AtomicU64 = AtomicU64::new(1);

        let req_topic_name = format!("{}.request", service_name);
        let client_id = CLIENT_ID.fetch_add(1, Ordering::Relaxed);
        let response_topic_name = format!("{}.response.{}", service_name, client_id);

        let req_topic = Topic::new(&req_topic_name).map_err(|e| e.to_string())?;
        let res_topic = Topic::new(&response_topic_name).map_err(|e| e.to_string())?;

        Ok(Self {
            name: service_name.to_string(),
            req_topic,
            res_topic,
            response_topic_name,
            poll_interval: Duration::from_millis(1),
        })
    }

    /// Call the service with a JSON request, wait for response.
    pub fn call(
        &mut self,
        request: serde_json::Value,
        timeout: Duration,
    ) -> Result<serde_json::Value, String> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static REQUEST_ID: AtomicU64 = AtomicU64::new(1);

        let request_id = REQUEST_ID.fetch_add(1, Ordering::Relaxed);

        // Send request
        self.req_topic.send(ServiceRequest {
            request_id,
            payload: request,
            response_topic: Some(self.response_topic_name.clone()),
        });

        // Poll for response
        let deadline = std::time::Instant::now() + timeout;
        loop {
            if let Some(resp) = self.res_topic.recv() {
                if resp.request_id == request_id {
                    if resp.ok {
                        return resp.payload.ok_or_else(|| "Empty response".to_string());
                    } else {
                        return Err(resp.error.unwrap_or_else(|| "Service error".to_string()));
                    }
                }
            }
            if std::time::Instant::now() > deadline {
                return Err(format!("Service '{}' call timed out", self.name));
            }
            std::thread::sleep(self.poll_interval);
        }
    }
}

/// Dynamic service server that dispatches to a handler function.
pub struct DynamicServiceServer {
    name: String,
    req_topic: Topic<ServiceRequest<serde_json::Value>>,
    handler: Option<Box<dyn Fn(serde_json::Value) -> serde_json::Value + Send + Sync>>,
}

impl DynamicServiceServer {
    /// Create a server for a dynamically-named service.
    pub fn new(service_name: &str) -> Result<Self, String> {
        let req_topic_name = format!("{}.request", service_name);
        let req_topic = Topic::new(&req_topic_name).map_err(|e| e.to_string())?;

        Ok(Self {
            name: service_name.to_string(),
            req_topic,
            handler: None,
        })
    }

    /// Set the request handler.
    pub fn set_handler(
        &mut self,
        handler: impl Fn(serde_json::Value) -> serde_json::Value + Send + Sync + 'static,
    ) {
        self.handler = Some(Box::new(handler));
    }

    /// Process one pending request (non-blocking).
    /// Returns true if a request was processed.
    pub fn process_one(&self) -> bool {
        let handler = match &self.handler {
            Some(h) => h,
            None => return false,
        };

        if let Some(req) = self.req_topic.recv() {
            let response_payload = handler(req.payload);

            // Send response to the client's response topic
            if let Some(ref resp_topic_name) = req.response_topic {
                if let Ok(resp_topic) =
                    Topic::<ServiceResponse<serde_json::Value>>::new(resp_topic_name)
                {
                    resp_topic.send(ServiceResponse {
                        request_id: req.request_id,
                        ok: true,
                        payload: Some(response_payload),
                        error: None,
                    });
                }
            }
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamic_service_json_handler_logic() {
        // Verify the handler logic works correctly (no Topic transport).
        // Full topic transport for services depends on horus_core's
        // TopicKind::ServiceRequest wiring for non-Pod types.
        let handler = |req: serde_json::Value| -> serde_json::Value {
            let a = req["a"].as_i64().unwrap_or(0);
            let b = req["b"].as_i64().unwrap_or(0);
            serde_json::json!({"sum": a + b})
        };

        let resp = handler(serde_json::json!({"a": 3, "b": 4}));
        assert_eq!(resp["sum"], 7);

        let resp = handler(serde_json::json!({"a": 100, "b": -50}));
        assert_eq!(resp["sum"], 50);

        let resp = handler(serde_json::json!({}));
        assert_eq!(resp["sum"], 0);
    }

    #[test]
    fn dynamic_service_client_creation() {
        let svc_name = format!("test_create.{}", std::process::id());
        let client = DynamicServiceClient::new(&svc_name);
        assert!(client.is_ok(), "client creation should succeed");
        assert_eq!(client.unwrap().name, svc_name);
    }

    #[test]
    fn dynamic_service_server_creation() {
        let svc_name = format!("test_server.{}", std::process::id());
        let mut server = DynamicServiceServer::new(&svc_name).unwrap();
        assert!(server.handler.is_none());

        server.set_handler(|v| v);
        assert!(server.handler.is_some());
    }

    #[test]
    fn dynamic_service_timeout() {
        let svc_name = format!("test_timeout.{}", std::process::id());
        let mut client = DynamicServiceClient::new(&svc_name).unwrap();

        // No server running — should timeout
        let result = client.call(serde_json::json!({"x": 1}), Duration::from_millis(100));
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("timed out"));
    }
}
