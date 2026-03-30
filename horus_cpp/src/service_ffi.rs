//! FFI wrappers for HORUS services (request/response RPC).
//!
//! ## Design
//!
//! Rust services use generics: `ServiceClient<S: Service>` with associated
//! `Request` and `Response` types. This can't cross FFI directly.
//!
//! We use **JSON-based type erasure**:
//! - C++ serializes request to JSON bytes
//! - Rust deserializes, routes to the correct service, gets response
//! - Rust serializes response to JSON bytes
//! - C++ deserializes response
//!
//! For POD message types used as request/response, the overhead is ~5μs
//! (JSON round-trip). This is acceptable for services (which are inherently
//! higher-latency than topics due to the request-response pattern).
//!
//! ## Alternative: Monomorphized Services
//!
//! For performance-critical services, the `impl_service_ffi!` macro can
//! generate typed FFI per service (like `impl_topic_ffi!` does for topics).
//! This is planned for a future phase.

use std::time::Duration;

use horus_core::communication::Topic;
use crate::types_ffi::JsonWireMessage;

/// Opaque service client using Pod-based JsonWireMessage transport.
pub struct FfiServiceClient {
    name: String,
    req_topic: Option<Topic<JsonWireMessage>>,
    res_topic: Option<Topic<JsonWireMessage>>,
    next_id: std::sync::atomic::AtomicU64,
}

/// Opaque service server using Pod-based JsonWireMessage transport.
pub struct FfiServiceServer {
    name: String,
    req_topic: Option<Topic<JsonWireMessage>>,
    handler: Option<Box<dyn Fn(&[u8]) -> Vec<u8> + Send + Sync>>,
}

// ─── Service Client FFI ──────────────────────────────────────────────────────

/// Create a new service client for the given service name.
///
/// The client will communicate via topics:
///   - `{name}.request` — publishes requests
///   - `{name}.response.{client_id}` — receives responses
pub fn service_client_new(name: &str) -> Box<FfiServiceClient> {
    let req_topic = Topic::<JsonWireMessage>::new(&format!("{}.request", name)).ok();
    let client_id = std::process::id();
    let res_topic = Topic::<JsonWireMessage>::new(&format!("{}.response.{}", name, client_id)).ok();
    Box::new(FfiServiceClient {
        name: name.to_string(),
        req_topic,
        res_topic,
        next_id: std::sync::atomic::AtomicU64::new(1),
    })
}

/// Get the service name.
pub fn service_client_name(client: &FfiServiceClient) -> &str {
    &client.name
}

/// Call the service with JSON-encoded request, returns JSON-encoded response.
///
/// `request_json` is a JSON string encoding the request payload.
/// `timeout_us` is the timeout in microseconds.
///
/// Returns JSON string of the response payload on success,
/// or an error string on failure.
///
/// Call the service with JSON request, returns JSON response.
///
/// Sends via Pod-based JsonWireMessage through Topic (works same-process and cross-process).
pub fn service_client_call(
    client: &FfiServiceClient,
    request_json: &str,
    timeout_us: u64,
) -> Result<String, String> {
    // Validate JSON
    let _: serde_json::Value = serde_json::from_str(request_json)
        .map_err(|e| format!("Invalid request JSON: {}", e))?;

    let req_topic = client.req_topic.as_ref()
        .ok_or_else(|| format!("Service '{}' request topic not initialized", client.name))?;
    let res_topic = client.res_topic.as_ref()
        .ok_or_else(|| format!("Service '{}' response topic not initialized", client.name))?;

    let msg_id = client.next_id.fetch_add(1, std::sync::atomic::Ordering::Relaxed);

    // Send request as JsonWireMessage (Pod — works through any Topic backend)
    let wire_msg = JsonWireMessage::from_json(request_json, msg_id, 0)
        .ok_or_else(|| "Request JSON too large (max 3968 bytes)".to_string())?;
    req_topic.send(wire_msg);

    // Poll for response
    let timeout = std::time::Duration::from_micros(timeout_us);
    let deadline = std::time::Instant::now() + timeout;
    loop {
        if let Some(resp) = res_topic.recv() {
            if resp.msg_id == msg_id {
                return resp.to_json().ok_or_else(|| "Invalid response encoding".to_string());
            }
        }
        if std::time::Instant::now() > deadline {
            return Err(format!("Service '{}' call timed out after {}µs", client.name, timeout_us));
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

// ─── Service Server FFI ──────────────────────────────────────────────────────

/// Create a new service server for the given service name.
pub fn service_server_new(name: &str) -> Box<FfiServiceServer> {
    let req_topic = Topic::<JsonWireMessage>::new(&format!("{}.request", name)).ok();
    Box::new(FfiServiceServer {
        name: name.to_string(),
        req_topic,
        handler: None,
    })
}

/// Set the request handler as a function that takes JSON request and returns JSON response.
///
/// The handler is called for each incoming request. It receives the request
/// payload as a JSON string and must return the response payload as a JSON string.
pub fn service_server_set_handler(
    server: &mut FfiServiceServer,
    handler: extern "C" fn(*const u8, usize, *mut u8, *mut usize) -> bool,
) {
    // Wrap the extern "C" handler in a safe Rust closure
    server.handler = Some(Box::new(move |request_bytes: &[u8]| -> Vec<u8> {
        // Allocate response buffer (4KB initial)
        let mut response_buf = vec![0u8; 4096];
        let mut response_len: usize = 0;

        let ok = handler(
            request_bytes.as_ptr(),
            request_bytes.len(),
            response_buf.as_mut_ptr(),
            &mut response_len as *mut usize,
        );

        if ok && response_len <= response_buf.len() {
            response_buf.truncate(response_len);
            response_buf
        } else {
            b"null".to_vec()
        }
    }));
}

/// Get the service server name.
pub fn service_server_name(server: &FfiServiceServer) -> &str {
    &server.name
}

/// Check if the server has a handler set.
pub fn service_server_has_handler(server: &FfiServiceServer) -> bool {
    server.handler.is_some()
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn client_creation() {
        let client = service_client_new("add_two_ints");
        assert_eq!(service_client_name(&client), "add_two_ints");
    }

    #[test]
    fn client_call_validates_json() {
        let client = service_client_new("test_svc_json_validate");
        let result = service_client_call(&client, "not json{{{", 1_000_000);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid request JSON"));
    }

    #[test]
    fn client_call_with_valid_json_no_server() {
        let client = service_client_new("test_svc_no_server");
        // Valid JSON but no server running — should timeout
        let result = service_client_call(&client, r#"{"a": 1, "b": 2}"#, 100_000);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("timed out"));
    }

    #[test]
    fn service_roundtrip_same_process() {
        // Uses Pod-based JsonWireMessage — works same-process!
        let svc = format!("svc_roundtrip.{}", std::process::id());

        // Create server-side req topic and client-side topics
        let server_req = horus_core::communication::Topic::<JsonWireMessage>::new(
            &format!("{}.request", svc)).unwrap();
        let client_id = std::process::id();
        let client_res = horus_core::communication::Topic::<JsonWireMessage>::new(
            &format!("{}.response.{}", svc, client_id)).unwrap();

        // Client sends request
        let req = JsonWireMessage::from_json(r#"{"a":3,"b":4}"#, 1, 0).unwrap();
        let client_req = horus_core::communication::Topic::<JsonWireMessage>::new(
            &format!("{}.request", svc)).unwrap();
        client_req.send(req);

        // Server receives
        let received = server_req.recv();
        assert!(received.is_some(), "server should receive request");
        let req_json = received.unwrap().to_json().unwrap();
        let req_val: serde_json::Value = serde_json::from_str(&req_json).unwrap();
        let sum = req_val["a"].as_i64().unwrap() + req_val["b"].as_i64().unwrap();

        // Server sends response
        let resp = JsonWireMessage::from_json(
            &serde_json::json!({"sum": sum}).to_string(), 1, 1).unwrap();
        let server_res = horus_core::communication::Topic::<JsonWireMessage>::new(
            &format!("{}.response.{}", svc, client_id)).unwrap();
        server_res.send(resp);

        // Client receives response
        let response = client_res.recv();
        assert!(response.is_some(), "client should receive response");
        let resp_json = response.unwrap().to_json().unwrap();
        let resp_val: serde_json::Value = serde_json::from_str(&resp_json).unwrap();
        assert_eq!(resp_val["sum"], 7, "3+4=7");
    }

    #[test]
    fn server_creation() {
        let server = service_server_new("add_two_ints");
        assert_eq!(service_server_name(&server), "add_two_ints");
        assert!(!service_server_has_handler(&server));
    }

    #[test]
    fn server_handler_set() {
        let mut server = service_server_new("echo");

        extern "C" fn echo_handler(
            req_ptr: *const u8,
            req_len: usize,
            res_ptr: *mut u8,
            res_len: *mut usize,
        ) -> bool {
            unsafe {
                std::ptr::copy_nonoverlapping(req_ptr, res_ptr, req_len);
                *res_len = req_len;
            }
            true
        }

        service_server_set_handler(&mut server, echo_handler);
        assert!(service_server_has_handler(&server));

        // Test the handler through the stored closure
        let handler = server.handler.as_ref().unwrap();
        let response = handler(b"hello");
        assert_eq!(response, b"hello");
    }

    #[test]
    fn server_handler_with_json() {
        let mut server = service_server_new("json_echo");

        extern "C" fn json_handler(
            req_ptr: *const u8,
            req_len: usize,
            res_ptr: *mut u8,
            res_len: *mut usize,
        ) -> bool {
            // Echo the request as response
            unsafe {
                if req_len <= 4096 {
                    std::ptr::copy_nonoverlapping(req_ptr, res_ptr, req_len);
                    *res_len = req_len;
                    true
                } else {
                    false
                }
            }
        }

        service_server_set_handler(&mut server, json_handler);
        let handler = server.handler.as_ref().unwrap();
        let input = br#"{"sum": 42}"#;
        let output = handler(input);
        assert_eq!(output, input);
    }
}
