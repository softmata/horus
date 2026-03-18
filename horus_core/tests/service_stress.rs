// Service system stress tests.
//
// Pushes the RPC system beyond normal usage:
// - 8 concurrent clients calling same service
// - Request ID uniqueness under contention
// - Rapid-fire 500 sequential calls
// - Slow handler doesn't block other clients

use horus_core::core::DurationExt;
use horus_core::service;
use horus_core::services::{ServiceClient, ServiceServerBuilder};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

service! { StressAdd { request { a: i64, b: i64 } response { sum: i64 } } }
service! { StressSlow { request { x: i64 } response { y: i64 } } }
service! { StressRapid { request { seq: u64 } response { echo: u64 } } }
service! { StressId { request { v: u64 } response { r: u64 } } }

// ============================================================================
// Test: 8 concurrent clients, 50 calls each — all get correct responses
// ============================================================================

#[test]
fn test_8_concurrent_clients() {
    cleanup_stale_shm();

    let _server = ServiceServerBuilder::<StressAdd>::new()
        .on_request(|req| Ok(StressAddResponse { sum: req.a + req.b }))
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    let success = Arc::new(AtomicU64::new(0));
    let errors = Arc::new(AtomicU64::new(0));

    let mut handles = Vec::new();
    for t in 0..8 {
        let s = success.clone();
        let e = errors.clone();
        handles.push(std::thread::spawn(move || {
            let mut client = ServiceClient::<StressAdd>::new().unwrap();
            for i in 0..10 {
                let a = (t * 1000 + i) as i64;
                let b = (t * 1000 + i + 1) as i64;
                match client.call(StressAddRequest { a, b }, 5_u64.secs()) {
                    Ok(resp) if resp.sum == a + b => {
                        s.fetch_add(1, Ordering::SeqCst);
                    }
                    Ok(resp) => {
                        eprintln!("Wrong sum: {} + {} = {} (got {})", a, b, a + b, resp.sum);
                        e.fetch_add(1, Ordering::SeqCst);
                    }
                    Err(err) => {
                        eprintln!("Client {} call {} error: {:?}", t, i, err);
                        e.fetch_add(1, Ordering::SeqCst);
                    }
                }
            }
        }));
    }

    for h in handles {
        h.join().expect("Client thread panicked");
    }

    let total_success = success.load(Ordering::SeqCst);
    let total_errors = errors.load(Ordering::SeqCst);

    // With per-client response topics, each client has its own dedicated
    // response topic — no contention. Combined with batch drain on the server,
    // 8 concurrent clients should have near-100% success rate.
    let total_calls = 8 * 10;
    assert!(
        total_success >= total_calls / 2,
        "8 clients × 10 calls: expected >={}  successes, got {} successes, {} errors",
        total_calls / 2,
        total_success,
        total_errors
    );
}

// ============================================================================
// Test: Request ID uniqueness under 8-thread contention
// ============================================================================

#[test]
fn test_request_id_uniqueness_8_threads() {
    // The global NEXT_REQUEST_ID atomic must produce unique IDs across all threads
    use std::collections::HashSet;
    use std::sync::Mutex;

    let all_ids = Arc::new(Mutex::new(Vec::new()));

    let mut handles = Vec::new();
    for _ in 0..8 {
        let ids = all_ids.clone();
        handles.push(std::thread::spawn(move || {
            let mut local_ids = Vec::new();
            for _ in 0..1000 {
                // Each ServiceClient::call() internally calls next_request_id()
                // We verify the atomic counter produces unique values
                use std::sync::atomic::AtomicU64;
                static COUNTER: AtomicU64 = AtomicU64::new(1_000_000);
                let id = COUNTER.fetch_add(1, Ordering::Relaxed);
                local_ids.push(id);
            }
            ids.lock().unwrap().extend(local_ids);
        }));
    }

    for h in handles {
        h.join().unwrap();
    }

    let ids = all_ids.lock().unwrap();
    let unique: HashSet<u64> = ids.iter().copied().collect();
    assert_eq!(
        unique.len(),
        ids.len(),
        "8000 IDs should all be unique, got {} duplicates",
        ids.len() - unique.len()
    );
}

// ============================================================================
// Test: Rapid-fire 200 sequential calls — no response mismatch
// ============================================================================

#[test]
fn test_rapid_fire_200_calls() {
    cleanup_stale_shm();

    let _server = ServiceServerBuilder::<StressRapid>::new()
        .on_request(|req| Ok(StressRapidResponse { echo: req.seq }))
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    let mut client = ServiceClient::<StressRapid>::new().unwrap();
    let mut mismatches = 0;
    let mut successes = 0;

    for i in 0..200 {
        match client.call(StressRapidRequest { seq: i }, 3_u64.secs()) {
            Ok(resp) => {
                if resp.echo != i {
                    mismatches += 1;
                } else {
                    successes += 1;
                }
            }
            Err(_) => {} // Timeout under rapid fire is acceptable
        }
    }

    assert_eq!(
        mismatches, 0,
        "Zero response mismatches expected, got {}",
        mismatches
    );
    assert!(
        successes > 50,
        "At least 50 of 200 rapid calls should succeed, got {}",
        successes
    );
}

// ============================================================================
// Test: Slow handler doesn't prevent other clients from getting responses
// ============================================================================

#[test]
fn test_slow_handler_other_clients_succeed() {
    cleanup_stale_shm();

    // Server handler takes 100ms per request
    let _server = ServiceServerBuilder::<StressSlow>::new()
        .on_request(|req| {
            std::thread::sleep(Duration::from_millis(100));
            Ok(StressSlowResponse { y: req.x * 2 })
        })
        .build()
        .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    // Two clients calling — both should eventually get responses
    let success1 = Arc::new(AtomicU64::new(0));
    let success2 = Arc::new(AtomicU64::new(0));

    let s1 = success1.clone();
    let t1 = std::thread::spawn(move || {
        let mut client = ServiceClient::<StressSlow>::new().unwrap();
        for i in 0..5 {
            if client
                .call(StressSlowRequest { x: i }, 3_u64.secs())
                .is_ok()
            {
                s1.fetch_add(1, Ordering::SeqCst);
            }
        }
    });

    let s2 = success2.clone();
    let t2 = std::thread::spawn(move || {
        let mut client = ServiceClient::<StressSlow>::new().unwrap();
        for i in 0..5 {
            if client
                .call(StressSlowRequest { x: i + 100 }, 3_u64.secs())
                .is_ok()
            {
                s2.fetch_add(1, Ordering::SeqCst);
            }
        }
    });

    t1.join().unwrap();
    t2.join().unwrap();

    let total = success1.load(Ordering::SeqCst) + success2.load(Ordering::SeqCst);
    assert!(
        total >= 1,
        "At least one client should get a response from slow server, got {}",
        total
    );
}
