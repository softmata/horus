//! A panic in user code must not take the server down with it.
//!
//! The scheduler already treats a node's `tick()` this way — it catches the
//! panic, marks the node and keeps running. Service servers did not, and the
//! failure was silent: a handler panic unwound the polling thread, so the
//! service stopped answering forever while the `ServiceServer` handle stayed
//! alive and `stop()` still appeared to work. Every later call, from every
//! client, timed out with nothing to indicate the server had gone.

use horus_core::service;
use horus_core::services::{ServiceClient, ServiceServerBuilder};
use std::time::Duration;

service! {
    FaultProbe {
        request { poison: bool }
        response { ok: bool }
    }
}

#[test]
fn a_service_survives_a_panicking_handler() {
    let _server = ServiceServerBuilder::<FaultProbe>::new()
        .on_request(|req| {
            // Panic on one specific input; every other request is answerable.
            assert!(!req.poison, "handler panic under test");
            Ok(FaultProbeResponse { ok: true })
        })
        .build()
        .expect("server");

    let mut client = ServiceClient::<FaultProbe>::new().expect("client");
    std::thread::sleep(Duration::from_millis(150));

    // Sanity: it answers before the panic.
    let first = client.call(
        FaultProbeRequest { poison: false },
        Duration::from_millis(700),
    );
    assert!(
        first.is_ok(),
        "service did not answer before the panic: {first:?}"
    );

    // Poison it. This call itself is expected to fail.
    let _ = client.call(
        FaultProbeRequest { poison: true },
        Duration::from_millis(700),
    );

    // The service must still be alive for everyone else.
    let after = client.call(
        FaultProbeRequest { poison: false },
        Duration::from_millis(700),
    );
    assert!(
        after.is_ok(),
        "the service stopped answering after one handler panic — a single bad \
         request took the service down for every client: {after:?}"
    );
}
