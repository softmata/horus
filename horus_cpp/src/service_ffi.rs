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
//! ## Alternative: Direct Topic<T: Pod> for Typed Services
//!
//! For performance-critical services, callers can skip this JSON layer and
//! use `Topic<T: Pod>` directly with `<svc>.request` / `<svc>.response.<id>`
//! naming. This gives ~500ns per call (vs ~5µs JSON) at the cost of requiring
//! the request/response types to be `bytemuck::Pod`. The existing
//! `Publisher<T>` / `Subscriber<T>` ergonomic wrappers are sufficient — no
//! new FFI needed. A templated C++ `horus::Service<Req, Resp>` wrapper is a
//! natural follow-up once a first consumer arrives.

use crate::types_ffi::JsonWireMessage;
use horus_core::communication::Topic;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

// Wire `msg_type` tags: requests on `{name}.request`, responses on the
// per-client `{name}.response.{pid}` topic.
const MSG_REQUEST: u8 = 0;
const MSG_RESPONSE: u8 = 1;

/// Process-unique request id. The high 32 bits carry the client PID so the
/// server can route the response back to `{name}.response.{pid}`; the low 32
/// bits are a per-client counter.
fn next_request_id(counter: &AtomicU64) -> u64 {
    let n = counter.fetch_add(1, Ordering::Relaxed) & 0xFFFF_FFFF;
    ((std::process::id() as u64) << 32) | n
}

/// Opaque service client using Pod-based JsonWireMessage transport.
pub struct FfiServiceClient {
    name: String,
    req_topic: Option<Topic<JsonWireMessage>>,
    res_topic: Option<Topic<JsonWireMessage>>,
    next_id: AtomicU64,
}

/// Opaque service server using Pod-based JsonWireMessage transport.
#[allow(dead_code, clippy::type_complexity)]
pub struct FfiServiceServer {
    name: String,
    req_topic: Option<Topic<JsonWireMessage>>,
    handler: Option<Box<dyn Fn(&[u8]) -> Vec<u8> + Send + Sync>>,
    // Per-client response topics (`{name}.response.{pid}`), created on demand.
    res_topics: HashMap<u32, Topic<JsonWireMessage>>,
}

// ─── Service Client FFI ──────────────────────────────────────────────────────

/// Create a new service client for the given service name.
///
/// The client will communicate via topics:
///   - `{name}.request` — publishes requests
///   - `{name}.response.{client_id}` — receives responses
pub fn service_client_new(name: &str) -> Box<FfiServiceClient> {
    let req_topic = Topic::<JsonWireMessage>::new(format!("{}.request", name)).ok();
    let client_id = std::process::id();
    let res_topic = Topic::<JsonWireMessage>::new(format!("{}.response.{}", name, client_id)).ok();
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
    let _: serde_json::Value =
        serde_json::from_str(request_json).map_err(|e| format!("Invalid request JSON: {}", e))?;

    let req_topic = client
        .req_topic
        .as_ref()
        .ok_or_else(|| format!("Service '{}' request topic not initialized", client.name))?;
    let res_topic = client
        .res_topic
        .as_ref()
        .ok_or_else(|| format!("Service '{}' response topic not initialized", client.name))?;

    let msg_id = next_request_id(&client.next_id);

    // Send request as JsonWireMessage (Pod — works through any Topic backend)
    let wire_msg = JsonWireMessage::from_json(request_json, msg_id, MSG_REQUEST)
        .ok_or_else(|| "Request JSON too large (max 3968 bytes)".to_string())?;
    req_topic.send(wire_msg);

    // Poll for response
    let timeout = std::time::Duration::from_micros(timeout_us);
    let deadline = std::time::Instant::now() + timeout;
    loop {
        if let Some(resp) = res_topic.recv() {
            if resp.msg_id == msg_id {
                return resp
                    .to_json()
                    .ok_or_else(|| "Invalid response encoding".to_string());
            }
        }
        if std::time::Instant::now() > deadline {
            return Err(format!(
                "Service '{}' call timed out after {}µs",
                client.name, timeout_us
            ));
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

// ─── Service Server FFI ──────────────────────────────────────────────────────

/// Create a new service server for the given service name.
pub fn service_server_new(name: &str) -> Box<FfiServiceServer> {
    let req_topic = Topic::<JsonWireMessage>::new(format!("{}.request", name)).ok();
    Box::new(FfiServiceServer {
        name: name.to_string(),
        req_topic,
        handler: None,
        res_topics: HashMap::new(),
    })
}

/// Set the request handler as an `extern "C"` function.
///
/// Signature `(req_ptr, req_len, res_ptr, res_len) -> bool`:
/// - `req_ptr` / `req_len`: the request payload (JSON bytes).
/// - `res_ptr`: output buffer for the response payload (JSON bytes).
/// - `res_len`: **IN/OUT** — on entry, the capacity of `res_ptr` in bytes; on
///   return, the number of bytes the handler wrote. The handler MUST write no
///   more than the incoming capacity. Returning `false`, or reporting a length
///   greater than the capacity, yields no response.
pub fn service_server_set_handler(
    server: &mut FfiServiceServer,
    handler: extern "C" fn(*const u8, usize, *mut u8, *mut usize) -> bool,
) {
    // Wrap the extern "C" handler in a safe Rust closure.
    server.handler = Some(Box::new(move |request_bytes: &[u8]| -> Vec<u8> {
        // Response buffer sized to the wire payload cap. `response_len` is IN/OUT:
        // on entry it advertises the buffer capacity; on return it is the number
        // of bytes the handler wrote. Initializing it to the capacity — not 0 —
        // is what closes the heap overflow (F1): a handler that respects it can
        // never write past `response_buf`. Because the capacity equals the wire
        // limit, an accepted response can never be silently dropped downstream (F4).
        let mut response_buf = vec![0u8; JsonWireMessage::MAX_PAYLOAD];
        let mut response_len: usize = response_buf.len();

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
            // Handler failed, or reported more bytes than the advertised capacity
            // (a contract violation) — do not forward a truncated/oversized body.
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

/// Drive the server once: receive pending requests, invoke the handler, and
/// publish each response back to the calling client's response topic. Call this
/// repeatedly from a single thread (the one the server was created on).
pub fn service_server_process(server: &mut FfiServiceServer) {
    let requests: Vec<JsonWireMessage> = match server.req_topic {
        Some(ref t) => std::iter::from_fn(|| t.recv()).collect(),
        None => Vec::new(),
    };
    for req in requests {
        let msg_id = req.msg_id;
        let req_json = match req.to_json() {
            Some(j) => j,
            None => continue,
        };
        let response_bytes = match server.handler {
            Some(ref h) => h(req_json.as_bytes()),
            None => continue, // no handler: drop the request
        };

        // Route the response to the client's per-PID response topic, matching
        // the client's `{name}.response.{pid}` subscription. The PID is encoded
        // in the high 32 bits of the request id.
        let client_pid = (msg_id >> 32) as u32;
        if !server.res_topics.contains_key(&client_pid) {
            let topic_name = format!("{}.response.{}", server.name, client_pid);
            if let Ok(t) = Topic::<JsonWireMessage>::new(topic_name) {
                server.res_topics.insert(client_pid, t);
            }
        }
        if let Some(topic) = server.res_topics.get(&client_pid) {
            let response_json = String::from_utf8_lossy(&response_bytes);
            if let Some(wire) = JsonWireMessage::from_json(&response_json, msg_id, MSG_RESPONSE) {
                topic.send(wire);
            }
        }
    }
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
        let server_req =
            horus_core::communication::Topic::<JsonWireMessage>::new(format!("{}.request", svc))
                .unwrap();
        let client_id = std::process::id();
        let client_res = horus_core::communication::Topic::<JsonWireMessage>::new(format!(
            "{}.response.{}",
            svc, client_id
        ))
        .unwrap();

        // Client sends request
        let req = JsonWireMessage::from_json(r#"{"a":3,"b":4}"#, 1, 0).unwrap();
        let client_req =
            horus_core::communication::Topic::<JsonWireMessage>::new(format!("{}.request", svc))
                .unwrap();
        client_req.send(req);

        // Server receives
        let received = server_req.recv();
        assert!(received.is_some(), "server should receive request");
        let req_json = received.unwrap().to_json().unwrap();
        let req_val: serde_json::Value = serde_json::from_str(&req_json).unwrap();
        let sum = req_val["a"].as_i64().unwrap() + req_val["b"].as_i64().unwrap();

        // Server sends response
        let resp =
            JsonWireMessage::from_json(&serde_json::json!({"sum": sum}).to_string(), 1, 1).unwrap();
        let server_res = horus_core::communication::Topic::<JsonWireMessage>::new(format!(
            "{}.response.{}",
            svc, client_id
        ))
        .unwrap();
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

    #[test]
    fn server_answers_a_client_call_end_to_end() {
        use std::time::{Duration, Instant};

        // Echo handler: response bytes = request bytes.
        extern "C" fn echo(req: *const u8, req_len: usize, res: *mut u8, res_len: *mut usize) -> bool {
            unsafe {
                std::ptr::copy_nonoverlapping(req, res, req_len);
                *res_len = req_len;
            }
            true
        }

        let name = format!("svc_e2e.{}", std::process::id());
        let mut server = service_server_new(&name);
        service_server_set_handler(&mut server, echo);

        let client = service_client_new(&name);
        std::thread::sleep(Duration::from_millis(50));

        // service_client_call blocks polling for its response, so run it on a
        // thread and drive the server from here.
        let call = std::thread::spawn(move || service_client_call(&client, r#"{"ping":1}"#, 3_000_000));

        let deadline = Instant::now() + Duration::from_secs(4);
        while !call.is_finished() {
            service_server_process(&mut server);
            if Instant::now() > deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }

        let resp = call.join().unwrap();
        assert!(resp.is_ok(), "service call should be answered: {resp:?}");
        assert!(resp.unwrap().contains("ping"), "response should echo the request");
    }

    // F1 regression: the FFI must advertise the real output-buffer capacity to
    // the handler via `*res_len` (IN). The bug initialized it to 0, so a handler
    // had no bound and a large response overflowed the Rust-side buffer. A handler
    // that fills to the advertised capacity must (a) observe the true wire cap,
    // not 0, and (b) not overflow the buffer.
    #[test]
    fn service_handler_is_told_capacity_and_can_fill_it() {
        extern "C" fn fill_to_capacity(
            _req: *const u8,
            _req_len: usize,
            res: *mut u8,
            res_len: *mut usize,
        ) -> bool {
            let cap = unsafe { *res_len };
            unsafe {
                std::ptr::write_bytes(res, 0xAB, cap);
                *res_len = cap;
            }
            true
        }

        let mut server = service_server_new("cap_fill");
        service_server_set_handler(&mut server, fill_to_capacity);
        let handler = server.handler.as_ref().unwrap();
        let out = handler(b"{}");
        // Old bug: capacity was 0 => out.len() == 0. Fixed: the full wire cap,
        // filled without overflowing the buffer.
        assert_eq!(out.len(), JsonWireMessage::MAX_PAYLOAD);
        assert!(out.iter().all(|&b| b == 0xAB));
    }

    // F1: a handler that reports more bytes than the advertised capacity (a
    // contract violation) is rejected — no truncated or oversized body is sent.
    #[test]
    fn service_handler_overreported_length_is_rejected() {
        extern "C" fn overreport(
            _req: *const u8,
            _req_len: usize,
            _res: *mut u8,
            res_len: *mut usize,
        ) -> bool {
            unsafe { *res_len = JsonWireMessage::MAX_PAYLOAD + 1_000_000 };
            true
        }

        let mut server = service_server_new("cap_overreport");
        service_server_set_handler(&mut server, overreport);
        let handler = server.handler.as_ref().unwrap();
        assert_eq!(handler(b"{}"), b"null");
    }
}
