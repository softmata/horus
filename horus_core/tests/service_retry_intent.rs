#![allow(dead_code)]
//! Service retry behavior and error handling intent tests.
//!
//! Tests: correct request-response mapping, rapid call handling, state
//! recovery after timeout, multi-service coexistence, and call_resilient()
//! retry logic on transient failures.
//!
//! Each test uses unique service types (via `service!`) to avoid SHM topic
//! collisions. We intentionally do NOT call `cleanup_stale_shm()` in tests
//! that start servers, because parallel tests would delete each other's SHM
//! directories. Unique type names provide sufficient isolation.

use horus_core::core::DurationExt;
use horus_core::error::RetryConfig;
use horus_core::service;
use horus_core::services::*;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
#[allow(unused_imports)]
use common::cleanup_stale_shm;

// ─── Unique service types per test (avoid SHM topic collisions) ──────────────

service! { IntentDouble     { request { val: i64 } response { result: i64 } } }
service! { IntentRapid      { request { val: i64 } response { result: i64 } } }
service! { IntentTimeout    { request { val: i64 } response { result: i64 } } }
service! { IntentCoexistA   { request { val: i64 } response { result: i64 } } }
service! { IntentCoexistB   { request { val: i64 } response { result: i64 } } }
service! { IntentResilient  { request { val: i64 } response { result: i64 } } }

// ═══════════════════════════════════════════════════════════════════════════════
// 1. INTENT: "Service calls return the correct response for each request."
//    Server doubles input. Send 10 different values. Assert each response
//    is exactly 2x the input.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_intent_request_response_correct() {
    let _server = ServiceServerBuilder::<IntentDouble>::new()
        .on_request(|req| {
            Ok(IntentDoubleResponse {
                result: req.val * 2,
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(300));

    let mut client = ServiceClient::<IntentDouble>::new().unwrap();

    // Warmup call to ensure server is responsive
    let _ = client.call(IntentDoubleRequest { val: 0 }, 5_u64.secs());

    for i in 1..=10 {
        let resp = client
            .call(IntentDoubleRequest { val: i }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("call with val={} failed: {:?}", i, e));
        assert_eq!(
            resp.result,
            i * 2,
            "val={}: expected {}, got {}",
            i,
            i * 2,
            resp.result
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. INTENT: "Server handles rapid successive calls without dropping or
//    mixing responses."
//    Send 50 calls in a tight loop with unique values. Assert all 50
//    responses match their requests.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_intent_server_handles_rapid_calls() {
    let _server = ServiceServerBuilder::<IntentRapid>::new()
        .on_request(|req| {
            Ok(IntentRapidResponse {
                result: req.val * 2,
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(300));

    let mut client = ServiceClient::<IntentRapid>::new().unwrap();

    // Warmup
    let _ = client.call(IntentRapidRequest { val: 0 }, 5_u64.secs());

    // 50 rapid-fire calls — no sleep between them
    for i in 1..=50 {
        let resp = client
            .call(IntentRapidRequest { val: i }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("rapid call {} failed: {:?}", i, e));
        assert_eq!(
            resp.result,
            i * 2,
            "rapid call {}: expected {}, got {}",
            i,
            i * 2,
            resp.result
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. INTENT: "After a timeout, subsequent calls still work correctly."
//    Call with no server (times out). Start server. Call again. Assert
//    second call succeeds with correct response.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_intent_timeout_does_not_corrupt_state() {
    let mut client = ServiceClient::<IntentTimeout>::new().unwrap();

    // First call — no server running, should timeout
    let result = client.call(IntentTimeoutRequest { val: 42 }, 50_u64.ms());
    assert!(result.is_err(), "call without server should fail");
    match result.unwrap_err() {
        ServiceError::Timeout => {} // expected
        other => panic!("expected Timeout, got: {:?}", other),
    }

    // Now start the server
    let _server = ServiceServerBuilder::<IntentTimeout>::new()
        .on_request(|req| {
            Ok(IntentTimeoutResponse {
                result: req.val * 2,
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(300));

    // Second call — server is now running, should succeed with correct value
    let resp = client
        .call(IntentTimeoutRequest { val: 42 }, 5_u64.secs())
        .expect("call after server start should succeed");
    assert_eq!(
        resp.result, 84,
        "after timeout recovery: expected 84, got {}",
        resp.result
    );

    // Third call — verify state is still clean after recovery
    let resp2 = client
        .call(IntentTimeoutRequest { val: 7 }, 5_u64.secs())
        .expect("subsequent call should also succeed");
    assert_eq!(
        resp2.result, 14,
        "follow-up call: expected 14, got {}",
        resp2.result
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. INTENT: "Multiple different services can run simultaneously without
//    interference."
//    Create service A (doubles) and service B (triples). Call both.
//    Assert A returns 2x, B returns 3x.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_intent_multiple_service_types_coexist() {
    // Server A doubles
    let _server_a = ServiceServerBuilder::<IntentCoexistA>::new()
        .on_request(|req| {
            Ok(IntentCoexistAResponse {
                result: req.val * 2,
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    // Server B triples
    let _server_b = ServiceServerBuilder::<IntentCoexistB>::new()
        .on_request(|req| {
            Ok(IntentCoexistBResponse {
                result: req.val * 3,
            })
        })
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(300));

    let mut client_a = ServiceClient::<IntentCoexistA>::new().unwrap();
    let mut client_b = ServiceClient::<IntentCoexistB>::new().unwrap();

    // Warmup both
    let _ = client_a.call(IntentCoexistARequest { val: 0 }, 5_u64.secs());
    let _ = client_b.call(IntentCoexistBRequest { val: 0 }, 5_u64.secs());

    // Interleave calls to both services
    for i in 1..=10 {
        let resp_a = client_a
            .call(IntentCoexistARequest { val: i }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("service A call {} failed: {:?}", i, e));
        let resp_b = client_b
            .call(IntentCoexistBRequest { val: i }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("service B call {} failed: {:?}", i, e));

        assert_eq!(
            resp_a.result,
            i * 2,
            "service A: val={}, expected {}, got {}",
            i,
            i * 2,
            resp_a.result
        );
        assert_eq!(
            resp_b.result,
            i * 3,
            "service B: val={}, expected {}, got {}",
            i,
            i * 3,
            resp_b.result
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. INTENT: "call_resilient() retries on transient errors and eventually
//    succeeds."
//    Server fails first 2 calls, succeeds on 3rd. Client uses
//    call_resilient with max_retries=5. Assert it eventually succeeds.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_call_resilient_retries_on_transient_failure() {
    // call_resilient retries on Timeout (transient). We simulate transient
    // failure by delaying the server start — the first few calls timeout,
    // then the server comes up and the retry succeeds.
    //
    // Strategy: create the client first. Then launch a background thread
    // that starts the server after a delay. The client's call_resilient
    // will timeout on the first attempts, then succeed once the server
    // is live and polling.

    let server_done = Arc::new(AtomicU32::new(0));
    let server_done_clone = server_done.clone();

    // Create the client first — this creates the SHM topic files.
    let mut client = ServiceClient::<IntentResilient>::new().unwrap();

    // Spawn server start after a short delay. The server will map the
    // same SHM topic files that the client already created.
    let server_handle = std::thread::spawn(move || {
        // Delay enough so the first retry attempt times out
        std::thread::sleep(Duration::from_millis(300));
        let server = ServiceServerBuilder::<IntentResilient>::new()
            .on_request(|req| {
                Ok(IntentResilientResponse {
                    result: req.val * 2,
                })
            })
            .poll_interval(1_u64.ms())
            .build()
            .unwrap();
        server_done_clone.store(1, Ordering::SeqCst);

        // Keep server alive until main thread signals completion
        while server_done_clone.load(Ordering::SeqCst) < 2 {
            std::thread::sleep(Duration::from_millis(10));
        }
        drop(server);
    });

    // Use call_resilient with short per-call timeout but generous retries.
    // Timing: server starts at ~300ms. Each attempt uses 100ms timeout.
    // With max_retries=10 and 10ms initial backoff (2x multiplier), the
    // total time budget is: attempt0(100ms) + 10ms + attempt1(100ms) +
    // 20ms + attempt2(100ms) + 40ms + attempt3(100ms)... The server comes
    // up around attempt 2-3 and a subsequent retry succeeds.
    let config = RetryConfig::new(10, 10_u64.ms());
    let result =
        client.call_resilient_with(IntentResilientRequest { val: 21 }, 100_u64.ms(), config);

    assert!(
        result.is_ok(),
        "call_resilient should eventually succeed after server starts: {:?}",
        result.err()
    );
    assert_eq!(
        result.unwrap().result,
        42,
        "call_resilient should return correct response"
    );

    // Signal server thread to shut down
    server_done.store(2, Ordering::SeqCst);
    server_handle
        .join()
        .expect("server thread should not panic");
}
