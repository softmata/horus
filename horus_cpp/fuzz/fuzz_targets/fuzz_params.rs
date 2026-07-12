#![no_main]
//! Fuzz params_set with arbitrary key/JSON-value pairs.
//! Input is split into (key, json_value) at first NUL byte.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let split = data.iter().position(|&b| b == 0).unwrap_or(data.len() / 2);
    let key_bytes = &data[..split];
    let val_bytes = if split < data.len() { &data[split + 1..] } else { &[][..] };
    let Ok(key) = std::str::from_utf8(key_bytes) else { return };
    let Ok(val) = std::str::from_utf8(val_bytes) else { return };
    let p = horus_cpp::params_new();
    let _ = horus_cpp::params_set(&p, key, val);
});
