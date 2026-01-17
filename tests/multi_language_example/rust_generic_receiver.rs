#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! horus = { path = "../../../horus" }
//! horus_library = { path = "../../../horus_library" }
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! rmp-serde = "1.1"
//! ```

// Rust Generic Receiver - receives arbitrary data from Python
//
// Compile: cargo build --release --example rust_generic_receiver
// Or: Add to horus/examples/

use horus::prelude::*;
use horus_library::messages::GenericMessage;
use serde_json::Value;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "=".repeat(70));
    println!("Rust Generic Receiver (Cross-Language)");
    println!("{}", "=".repeat(70));

    // Create generic hub (same topic as Python)
    let hub = Topic::<GenericMessage>::new("generic_cross_lang")?;

    println!("\nWaiting for data from Python...");
    println!("(Run 'python_generic_sender.py' in another terminal)");
    println!("(Ctrl+C to stop)\n");

    let mut count = 0;
    loop {
        if let Some(msg) = hub.recv(None) {
            count += 1;

            // Deserialize MessagePack to JSON
            let value: Value = rmp_serde::from_slice(&msg.data)?;

            // Pretty print
            println!("─────────────────────────────────────────────────────────────────");
            println!("Message #{}", count);
            println!("─────────────────────────────────────────────────────────────────");
            println!("{}", serde_json::to_string_pretty(&value)?);

            // Access specific fields
            if let Some(source) = value.get("source").and_then(|v| v.as_str()) {
                println!("\nSource: {}", source);
            }

            if let Some(readings) = value.get("readings") {
                if let Some(temp) = readings.get("temperature") {
                    println!("Temperature: {:.1}°C", temp.as_f64().unwrap_or(0.0));
                }
                if let Some(humidity) = readings.get("humidity") {
                    println!("Humidity: {:.1}%", humidity.as_f64().unwrap_or(0.0));
                }
                if let Some(pressure) = readings.get("pressure") {
                    println!("Pressure: {:.1} hPa", pressure.as_f64().unwrap_or(0.0));
                }
            }

            if let Some(measurements) = value.get("measurements").and_then(|v| v.as_array()) {
                println!("[DATA] Measurements ({} values)", measurements.len());
            }

            println!();
        }

        std::thread::sleep(Duration::from_millis(100));
    }
}
