#![no_main]
//! Fuzz Topic<JsonWireMessage> send/recv with arbitrary JSON payloads.

use horus_core::communication::Topic;
use horus_cpp::JsonWireMessage;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let Ok(s) = std::str::from_utf8(data) else { return };
    let topic_name = format!("fuzz.topic.{}", std::process::id());
    let Ok(topic) = Topic::<JsonWireMessage>::new(topic_name) else { return };
    if let Some(msg) = JsonWireMessage::from_json(s, 1, 0) {
        topic.send(msg);
        let _ = topic.recv();
    }
});
