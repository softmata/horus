// Test: Rust side of Generic Hub cross-language communication
//
// Compile: rustc --edition 2021 -L ../target/release/deps test_generic_rust_side.rs
// Or better: Add as example in horus crate

use horus::prelude::*;
use horus_library::messages::GenericMessage;
use serde_json::Value;
use std::collections::HashMap;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "=".repeat(70));
    println!("TEST: Rust ↔ Python (Generic Data)");
    println!("{}", "=".repeat(70));

    // Create generic hub (same topic as Python)
    let hub = Topic::<GenericMessage>::new("cross_lang_topic")?;

    // Send data from Rust
    let mut rust_data = HashMap::new();
    rust_data.insert("source", "rust");
    rust_data.insert("message", "Hello from Rust!");
    rust_data.insert("number", "123");

    let msg = GenericMessage::from_value(&rust_data)?;
    hub.send(msg, None)?;
    println!("Rust sent: {:?}", rust_data);

    // Try to receive data from Python
    std::thread::sleep(std::time::Duration::from_millis(100));

    if let Some(received_msg) = hub.recv(None) {
        // Deserialize from MessagePack
        let value: Value = rmp_serde::from_slice(&received_msg.data)?;
        println!("Rust received: {}", serde_json::to_string_pretty(&value)?);

        // Access nested data
        if let Some(sensor_readings) = value.get("sensor_readings") {
            if let Some(temp) = sensor_readings.get("temperature") {
                println!("Temperature from Python: {}", temp);
            }
        }

        if let Some(measurements) = value.get("measurements").and_then(|v| v.as_array()) {
            println!("Measurements from Python: {:?}", measurements);
        }

        println!("[OK] Rust side working!");
    } else {
        println!("[WARNING] No message received (Python might not have sent yet)");
    }

    Ok(())
}
