#![no_main]
//! Fuzz service_client_call with arbitrary request strings.
//! 1ms timeout — we aren't testing success, only that the call can't crash.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Ok(s) = std::str::from_utf8(data) else { return };
    let client = horus_cpp::service_client_new("fuzz.svc");
    let _ = horus_cpp::service_client_call(&client, s, 1_000);
});
