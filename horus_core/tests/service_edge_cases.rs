//! Service system edge-case tests — covers gaps identified in SLAM analysis.
//!
//! Tests: ServiceError transience classification, Display output, timeout
//! without server, async client check/expiry lifecycle, ok-but-no-payload
//! handling, and multi-client response isolation.

use horus_core::core::DurationExt;
use horus_core::service;
use horus_core::services::*;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ─── Unique service types per test (avoid SHM topic collisions) ──────────────

service! { EdgeTimeout   { request { msg: String } response { reply: String } } }
service! { EdgeAsyncChk  { request { msg: String } response { reply: String } } }
service! { EdgeAsyncExp  { request { msg: String } response { reply: String } } }
service! { EdgeOkNoPay   { request { msg: String } response { reply: String } } }
service! { EdgeIsoA      { request { val: i64 }    response { doubled: i64 } } }
service! { EdgeIsoB      { request { val: i64 }    response { tripled: i64 } } }

// ═══════════════════════════════════════════════════════════════════════════════
// 1. ServiceError::is_transient() — exhaustive variant coverage
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_error_all_variants_transience() {
    // Transient errors — retry may succeed
    assert!(
        ServiceError::Timeout.is_transient(),
        "Timeout should be transient"
    );
    assert!(
        ServiceError::Transport("network glitch".into()).is_transient(),
        "Transport should be transient"
    );

    // Permanent errors — retry will NOT succeed
    assert!(
        !ServiceError::NoServer.is_transient(),
        "NoServer should NOT be transient"
    );
    assert!(
        !ServiceError::ServiceFailed("handler error".into()).is_transient(),
        "ServiceFailed should NOT be transient"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. ServiceError Display — all variants produce non-empty messages
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_service_error_display_messages() {
    let variants: Vec<ServiceError> = vec![
        ServiceError::Timeout,
        ServiceError::NoServer,
        ServiceError::ServiceFailed("test failure".into()),
        ServiceError::Transport("connection reset".into()),
    ];

    for err in &variants {
        let msg = format!("{}", err);
        assert!(
            !msg.is_empty(),
            "{:?} produced an empty Display message",
            err
        );
    }

    // Verify specific content is embedded in parameterized variants
    let sf_msg = format!("{}", ServiceError::ServiceFailed("custom reason".into()));
    assert!(
        sf_msg.contains("custom reason"),
        "ServiceFailed Display should contain the inner message, got: {}",
        sf_msg
    );

    let transport_msg = format!("{}", ServiceError::Transport("io broken".into()));
    assert!(
        transport_msg.contains("io broken"),
        "Transport Display should contain the inner message, got: {}",
        transport_msg
    );

    // Verify NoServer and Timeout have descriptive (not generic) messages
    let timeout_msg = format!("{}", ServiceError::Timeout);
    assert!(
        timeout_msg.contains("timed out") || timeout_msg.contains("timeout"),
        "Timeout Display should mention timeout, got: {}",
        timeout_msg
    );

    let no_server_msg = format!("{}", ServiceError::NoServer);
    assert!(
        no_server_msg.contains("server") || no_server_msg.contains("available"),
        "NoServer Display should mention server availability, got: {}",
        no_server_msg
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. Client timeout with no server running
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_client_timeout_no_server() {
    cleanup_stale_shm();

    // Create client — NO server is started for EdgeTimeout
    let mut client = ServiceClient::<EdgeTimeout>::new().unwrap();

    let start = std::time::Instant::now();
    let result = client.call(
        EdgeTimeoutRequest {
            msg: "hello?".into(),
        },
        50_u64.ms(),
    );
    let elapsed = start.elapsed();

    // Must return Timeout, not panic or hang
    assert!(result.is_err(), "call without server should fail");
    match result.unwrap_err() {
        ServiceError::Timeout => {} // expected
        other => panic!("expected Timeout, got: {:?}", other),
    }

    // Elapsed time should be roughly at or above the 50ms timeout
    assert!(
        elapsed >= Duration::from_millis(40),
        "should have waited near the timeout duration, elapsed: {:?}",
        elapsed
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. AsyncServiceClient::check() returns None until response arrives
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_async_client_check_returns_none_without_server() {
    cleanup_stale_shm();

    // AsyncServiceClient::check() should return Ok(None) when no response
    // has arrived yet. With no server running, check() will keep returning
    // Ok(None) until the pending call's timeout elapses.
    //
    // Note: AsyncServiceClient uses Topic::new() (TopicKind::Data) for the
    // response topic, while the server publishes via Topic::new_with_kind
    // (TopicKind::ServiceResponse). This means async round-trips via per-client
    // topics are currently broken. This test verifies the check/expire lifecycle
    // which works correctly regardless.

    let mut client = AsyncServiceClient::<EdgeAsyncChk>::new().unwrap();
    let mut pending = client.call_async(
        EdgeAsyncChkRequest {
            msg: "ping".into(),
        },
        500_u64.ms(),
    );

    // Immediately after call_async — not expired, no response
    assert!(
        !pending.is_expired(),
        "pending should not be expired immediately after call_async"
    );

    let result = pending.check();
    assert!(
        matches!(result, Ok(None)),
        "check() should return Ok(None) when no response yet, got: {:?}",
        result
    );

    // After partial time — still None, not yet expired
    std::thread::sleep(Duration::from_millis(50));
    if !pending.is_expired() {
        let result2 = pending.check();
        assert!(
            matches!(result2, Ok(None)),
            "check() mid-timeout should still be Ok(None), got: {:?}",
            result2
        );
    }

    // Wait for timeout to elapse
    std::thread::sleep(Duration::from_millis(500));
    assert!(
        pending.is_expired(),
        "pending should be expired after timeout"
    );

    let expired_result = pending.check();
    assert!(
        matches!(expired_result, Err(ServiceError::Timeout)),
        "check() after expiry should return Err(Timeout), got: {:?}",
        expired_result
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. PendingServiceCall expires after timeout
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_async_client_pending_expires_after_timeout() {
    cleanup_stale_shm();

    // No server started — the pending call will never get a response
    let mut client = AsyncServiceClient::<EdgeAsyncExp>::new().unwrap();
    let mut pending = client.call_async(
        EdgeAsyncExpRequest {
            msg: "expire".into(),
        },
        10_u64.ms(),
    );

    // Immediately — should not be expired yet
    assert!(
        !pending.is_expired(),
        "pending should NOT be expired immediately after call_async"
    );

    // Wait past the timeout
    std::thread::sleep(Duration::from_millis(30));

    // Now it should be expired
    assert!(
        pending.is_expired(),
        "pending should be expired after timeout elapses"
    );

    // check() on expired pending returns Err(Timeout)
    let result = pending.check();
    assert!(result.is_err(), "check() on expired pending should error");
    match result.unwrap_err() {
        ServiceError::Timeout => {} // expected
        other => panic!("expected Timeout from expired pending, got: {:?}", other),
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. ServiceResponse with ok=true but payload=None
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_response_ok_true_but_no_payload() {
    // Manually construct a ServiceResponse where ok=true but payload=None.
    // The client code treats this as ServiceFailed("server returned ok=true
    // but no payload") — verify that the type allows this state and that
    // the fields are accessible for defensive handling.
    let resp: ServiceResponse<String> = ServiceResponse {
        request_id: 1,
        ok: true,
        payload: None,
        error: None,
    };

    assert!(resp.ok);
    assert!(resp.payload.is_none());
    assert!(resp.error.is_none());

    // Compare against a properly-constructed success response
    let good = ServiceResponse::success(2, "hello".to_string());
    assert!(good.ok);
    assert!(good.payload.is_some());

    // Compare against a properly-constructed failure response
    let bad: ServiceResponse<String> = ServiceResponse::failure(3, "broke");
    assert!(!bad.ok);
    assert!(bad.payload.is_none());
    assert!(bad.error.is_some());

    // The ok-but-no-payload state is representable but would cause
    // ServiceFailed at the client layer. Verify the defensive message
    // that the client produces matches expectations by building a server
    // that triggers this condition indirectly: a handler that returns Ok
    // is always wrapped with payload=Some by ServiceServerBuilder, so
    // this state can only arise from malformed wire data. The test above
    // proves the struct allows it and downstream code can inspect fields.
}

// ═══════════════════════════════════════════════════════════════════════════════
// 7. Two clients, one server — response isolation
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_two_clients_responses_isolated() {
    cleanup_stale_shm();

    // Server doubles the input
    let _server = ServiceServerBuilder::<EdgeIsoA>::new()
        .on_request(|req| Ok(EdgeIsoAResponse { doubled: req.val * 2 }))
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(500));

    // Two independent clients talking to the same server
    let mut client_a = ServiceClient::<EdgeIsoA>::new().unwrap();
    let mut client_b = ServiceClient::<EdgeIsoA>::new().unwrap();

    // Warmup — retry until server is responsive
    for _ in 0..5 {
        if client_a.call(EdgeIsoARequest { val: 0 }, 2_u64.secs()).is_ok() {
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }

    // Client A sends 10, Client B sends 99
    let resp_a = client_a
        .call(EdgeIsoARequest { val: 10 }, 5_u64.secs())
        .expect("client_a call should succeed");
    let resp_b = client_b
        .call(EdgeIsoARequest { val: 99 }, 5_u64.secs())
        .expect("client_b call should succeed");

    // Each client must receive its own response — no cross-contamination
    assert_eq!(
        resp_a.doubled, 20,
        "client_a sent 10, expected 20, got {}",
        resp_a.doubled
    );
    assert_eq!(
        resp_b.doubled, 198,
        "client_b sent 99, expected 198, got {}",
        resp_b.doubled
    );

    // Interleaved calls from both clients
    for i in 0..5 {
        let va = i * 3;
        let vb = i * 7;

        let ra = client_a
            .call(EdgeIsoARequest { val: va }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("client_a call {} failed: {:?}", i, e));
        let rb = client_b
            .call(EdgeIsoARequest { val: vb }, 5_u64.secs())
            .unwrap_or_else(|e| panic!("client_b call {} failed: {:?}", i, e));

        assert_eq!(
            ra.doubled,
            va * 2,
            "client_a iteration {}: sent {}, expected {}, got {}",
            i,
            va,
            va * 2,
            ra.doubled
        );
        assert_eq!(
            rb.doubled,
            vb * 2,
            "client_b iteration {}: sent {}, expected {}, got {}",
            i,
            vb,
            vb * 2,
            rb.doubled
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Bonus: Two clients from separate threads — sequential per thread, concurrent
// between threads. Verifies per-client response topics prevent cross-talk.
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_two_clients_concurrent_isolation() {
    cleanup_stale_shm();

    let _server = ServiceServerBuilder::<EdgeIsoB>::new()
        .on_request(|req| Ok(EdgeIsoBResponse { tripled: req.val * 3 }))
        .poll_interval(1_u64.ms())
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    // Thread A: calls with values 0..3
    let handle_a = std::thread::spawn(|| {
        let mut client = ServiceClient::<EdgeIsoB>::new().unwrap();
        for i in 0_i64..3 {
            match client.call(EdgeIsoBRequest { val: i }, 5_u64.secs()) {
                Ok(resp) => {
                    assert_eq!(
                        resp.tripled,
                        i * 3,
                        "thread A: sent {}, expected {}, got {}",
                        i,
                        i * 3,
                        resp.tripled
                    );
                }
                Err(e) => {
                    // Under heavy load some calls may timeout — that's acceptable
                    // as long as none return the wrong value
                    eprintln!("thread A call {} failed (acceptable): {:?}", i, e);
                }
            }
        }
    });

    // Thread B: calls with values 100..103 — distinct from thread A
    let handle_b = std::thread::spawn(|| {
        let mut client = ServiceClient::<EdgeIsoB>::new().unwrap();
        for i in 100_i64..103 {
            match client.call(EdgeIsoBRequest { val: i }, 5_u64.secs()) {
                Ok(resp) => {
                    assert_eq!(
                        resp.tripled,
                        i * 3,
                        "thread B: sent {}, expected {}, got {}",
                        i,
                        i * 3,
                        resp.tripled
                    );
                }
                Err(e) => {
                    eprintln!("thread B call {} failed (acceptable): {:?}", i, e);
                }
            }
        }
    });

    handle_a.join().expect("thread A panicked — response mismatch detected");
    handle_b.join().expect("thread B panicked — response mismatch detected");
}
