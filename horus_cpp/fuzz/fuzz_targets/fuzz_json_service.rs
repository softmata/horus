#![no_main]
//! Fuzz JsonWireMessage::from_json against arbitrary UTF-8 input.
//! Any panic or hang is a bug; clean `None` returns are fine.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let s = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(_) => return,
    };
    let _ = horus_cpp::JsonWireMessage::from_json(s, 0, 0);
});
