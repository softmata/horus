#![allow(dead_code)]
//! Root tests for the Services RPC module.
//!
//! Tests ServiceServerBuilder, ServiceClient, AsyncServiceClient, and the
//! full request-response lifecycle. Verifies: builder pattern, timeout handling,
//! retry logic, async polling, and end-to-end roundtrip.

use horus_core::core::DurationExt;
use horus_core::service;
use horus_core::services::*;

// Define a test service using the service! macro
service! {
    AddTwoInts {
        request { a: i64, b: i64 }
        response { sum: i64 }
    }
}

// Each test that creates a server needs its own service type to avoid
// SHM topic name collisions when tests run in parallel.
service! { EchoBuild    { request { message: String } response { reply: String } } }
service! { EchoPoll     { request { message: String } response { reply: String } } }
service! { EchoStop     { request { message: String } response { reply: String } } }
service! { EchoTimeout  { request { message: String } response { reply: String } } }
service! { EchoOptional { request { message: String } response { reply: String } } }
service! { EchoE2E      { request { message: String } response { reply: String } } }
service! { EchoFail     { request { message: String } response { reply: String } } }
service! { EchoAsync    { request { message: String } response { reply: String } } }
service! { EchoPending  { request { message: String } response { reply: String } } }
service! { EchoExpire   { request { message: String } response { reply: String } } }

// ═══════════════════════════════════════════════════════════════════════════
// Types / Wire format
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_response_success() {
    let resp = ServiceResponse::success(42, "hello".to_string());
    assert!(resp.ok);
    assert_eq!(resp.request_id, 42);
    assert_eq!(resp.payload, Some("hello".to_string()));
    assert!(resp.error.is_none());
}

#[test]
fn test_service_response_failure() {
    let resp: ServiceResponse<String> = ServiceResponse::failure(99, "something broke");
    assert!(!resp.ok);
    assert_eq!(resp.request_id, 99);
    assert!(resp.payload.is_none());
    assert_eq!(resp.error, Some("something broke".to_string()));
}

#[test]
fn test_service_error_transient() {
    assert!(ServiceError::Timeout.is_transient());
    assert!(ServiceError::Transport("net".into()).is_transient());
    assert!(!ServiceError::ServiceFailed("bad".into()).is_transient());
    assert!(!ServiceError::NoServer.is_transient());
}

#[test]
fn test_service_trait_topic_names() {
    assert_eq!(AddTwoInts::request_topic(), "add_two_ints.request");
    assert_eq!(AddTwoInts::response_topic(), "add_two_ints.response");
    // Verify topic names follow the {snake_case_name}.{request|response} pattern
    let e2e_name = EchoE2E::name();
    assert_eq!(EchoE2E::request_topic(), format!("{}.request", e2e_name));
    assert_eq!(EchoE2E::response_topic(), format!("{}.response", e2e_name));
}

#[test]
fn test_service_info_fields() {
    let info = ServiceInfo {
        name: "test_svc".into(),
        request_type: "Req".into(),
        response_type: "Res".into(),
        servers: 1,
        clients: 2,
    };
    assert_eq!(info.name, "test_svc");
    assert_eq!(info.servers, 1);
    assert_eq!(info.clients, 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// ServerBuilder
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_server_builder_default_builds() {
    let server = ServiceServerBuilder::<EchoBuild>::new().build();
    assert!(
        server.is_ok(),
        "default builder should build: {:?}",
        server.err()
    );
    // Server runs with default "no handler" — will return error for any request
}

#[test]
fn test_server_builder_with_handler() {
    let server = ServiceServerBuilder::<AddTwoInts>::new()
        .on_request(|req| Ok(AddTwoIntsResponse { sum: req.a + req.b }))
        .build();
    server.unwrap();
}

#[test]
fn test_server_builder_custom_poll_interval() {
    let server = ServiceServerBuilder::<EchoPoll>::new()
        .poll_interval(1_u64.ms())
        .on_request(|req| Ok(EchoPollResponse { reply: req.message }))
        .build();
    server.unwrap();
}

#[test]
fn test_server_stop() {
    let server = ServiceServerBuilder::<EchoStop>::new()
        .on_request(|req| Ok(EchoStopResponse { reply: req.message }))
        .build()
        .unwrap();
    server.stop();
    // Server thread should terminate — drop will join it
}

// ═══════════════════════════════════════════════════════════════════════════
// ServiceClient
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_client_new_succeeds() {
    let client = ServiceClient::<EchoTimeout>::new();
    assert!(
        client.is_ok(),
        "client creation should succeed: {:?}",
        client.err()
    );
}

#[test]
fn test_client_call_timeout_no_server() {
    let mut client = ServiceClient::<EchoTimeout>::new().unwrap();
    let result = client.call(
        EchoTimeoutRequest {
            message: "hello".into(),
        },
        50_u64.ms(),
    );
    // No server running — should timeout
    assert!(result.is_err());
    match result.unwrap_err() {
        ServiceError::Timeout => {} // expected
        other => panic!("expected Timeout, got: {:?}", other),
    }
}

#[test]
fn test_client_call_optional_returns_none_on_timeout() {
    let mut client = ServiceClient::<EchoOptional>::new().unwrap();
    let result = client.call_optional(
        EchoOptionalRequest {
            message: "hello".into(),
        },
        50_u64.ms(),
    );
    assert!(result.is_ok());
    assert!(result.unwrap().is_none(), "should return None on timeout");
}

// ═══════════════════════════════════════════════════════════════════════════
// End-to-end: Server + Client roundtrip
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_end_to_end_add_two_ints() {
    // Start server
    let _server = ServiceServerBuilder::<AddTwoInts>::new()
        .on_request(|req| Ok(AddTwoIntsResponse { sum: req.a + req.b }))
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    // Give server thread time to start
    std::thread::sleep(10_u64.ms());

    // Call from client
    let mut client = ServiceClient::<AddTwoInts>::new().unwrap();
    let result = client.call(AddTwoIntsRequest { a: 3, b: 4 }, 1_u64.secs());

    assert!(result.is_ok(), "call should succeed: {:?}", result.err());
    assert_eq!(result.unwrap().sum, 7);
}

#[test]
fn test_end_to_end_echo() {
    let _server = ServiceServerBuilder::<EchoE2E>::new()
        .on_request(|req| {
            Ok(EchoE2EResponse {
                reply: format!("ECHO: {}", req.message),
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(10_u64.ms());

    let mut client = ServiceClient::<EchoE2E>::new().unwrap();
    let result = client.call(
        EchoE2ERequest {
            message: "hello world".into(),
        },
        1_u64.secs(),
    );

    assert!(result.is_ok(), "echo should succeed: {:?}", result.err());
    assert_eq!(result.unwrap().reply, "ECHO: hello world");
}

#[test]
fn test_end_to_end_server_returns_error() {
    let _server = ServiceServerBuilder::<EchoFail>::new()
        .on_request(|_req| Err("intentional failure".to_string()))
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(10_u64.ms());

    let mut client = ServiceClient::<EchoFail>::new().unwrap();
    let result = client.call(
        EchoFailRequest {
            message: "fail".into(),
        },
        1_u64.secs(),
    );

    assert!(result.is_err());
    match result.unwrap_err() {
        ServiceError::ServiceFailed(msg) => {
            assert!(msg.contains("intentional failure"));
        }
        other => panic!("expected ServiceFailed, got: {:?}", other),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// AsyncServiceClient
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_async_client_new_succeeds() {
    let client = AsyncServiceClient::<EchoAsync>::new();
    client.unwrap();
}

#[test]
fn test_async_client_call_returns_pending() {
    let mut client = AsyncServiceClient::<EchoPending>::new().unwrap();
    let pending = client.call_async(
        EchoPendingRequest {
            message: "async".into(),
        },
        100_u64.ms(),
    );
    // Immediately returns — not blocking
    assert!(!pending.is_expired());
}

#[test]
fn test_pending_call_expires_after_timeout() {
    let mut client = AsyncServiceClient::<EchoExpire>::new().unwrap();
    let mut pending = client.call_async(
        EchoExpireRequest {
            message: "timeout".into(),
        },
        10_u64.ms(),
    );
    std::thread::sleep(20_u64.ms());
    assert!(pending.is_expired());
    let result = pending.check();
    result.unwrap_err();
}
