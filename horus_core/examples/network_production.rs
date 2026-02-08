use colored::Colorize;
/// Production-Ready Network Communication Example
///
/// Demonstrates:
/// - Router-based communication with automatic reconnection
/// - Discovery service with auto-response
/// - Proper error handling and logging
/// - Connection health monitoring
///
/// Run with: cargo run --example network_production --release
use horus_core::communication::network::{ReconnectContext, ReconnectStrategy};
use horus_core::communication::Topic;
use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SensorData {
    sensor_id: String,
    temperature: f32,
    pressure: f32,
    timestamp_ns: u64,
}

impl LogSummary for SensorData {
    fn log_summary(&self) -> String {
        format!(
            "Sensor {} - temp: {:.1}°C, pressure: {:.1} kPa",
            self.sensor_id, self.temperature, self.pressure
        )
    }
}

/// Simulated sensor publisher with reconnection
fn run_publisher() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "Starting PRODUCTION sensor publisher".cyan().bold());
    println!("   Connecting to router at localhost:7777");
    println!("   Press Ctrl+C to stop\n");

    // Production-grade reconnection strategy
    let reconnect_strategy = ReconnectStrategy::production();
    let mut reconnect_ctx = ReconnectContext::new(reconnect_strategy);

    // Attempt connection with reconnection logic
    let hub: Topic<SensorData> = loop {
        println!("Attempt {}: Connecting...", reconnect_ctx.attempt + 1);

        match Topic::new("sensors@router") {
            Ok(hub) => {
                reconnect_ctx.mark_connected();
                println!("{}", "Connected successfully!".green().bold());
                println!();
                break hub;
            }
            Err(e) => {
                reconnect_ctx.begin_reconnect();
                eprintln!("{} Connection failed: {}", "[ERROR]".red().bold(), e);

                if !reconnect_ctx.should_retry() {
                    eprintln!("{}", "Max retries exceeded. Giving up.".red());
                    return Err("Failed to connect after max retries".into());
                }

                let delay = reconnect_ctx.backoff_delay();
                println!(
                    "   Retrying in {:.1}s... (attempt {})",
                    delay.as_secs_f64(),
                    reconnect_ctx.attempt
                );
                reconnect_ctx.wait_backoff();
            }
        }
    };

    // Main publishing loop
    let mut counter = 0u64;
    loop {
        let data = SensorData {
            sensor_id: "TEMP_001".to_string(),
            temperature: 20.0 + (counter as f32 * 0.1) % 10.0,
            pressure: 101.3 + (counter as f32 * 0.01) % 2.0,
            timestamp_ns: counter,
        };

        match hub.send(data.clone()) {
            Ok(_) => {
                println!("Sent: {}", data.log_summary());
            }
            Err(_) => {
                eprintln!("{}", "Send failed - connection may be lost".yellow());
                // In production, you would initiate reconnection here
            }
        }

        counter += 1;
        std::thread::sleep(Duration::from_secs(1));
    }
}

/// Simulated data consumer with health monitoring
fn run_subscriber() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", "Starting PRODUCTION sensor subscriber".cyan().bold());
    println!("   Connecting to router at localhost:7777");
    println!("   Press Ctrl+C to stop\n");

    // Connect with reconnection support
    let reconnect_strategy = ReconnectStrategy::production();
    let mut reconnect_ctx = ReconnectContext::new(reconnect_strategy);

    let hub: Topic<SensorData> = loop {
        println!("Attempt {}: Connecting...", reconnect_ctx.attempt + 1);

        match Topic::new("sensors@router") {
            Ok(hub) => {
                reconnect_ctx.mark_connected();
                println!("{}", "Connected successfully!".green().bold());
                println!();
                break hub;
            }
            Err(e) => {
                reconnect_ctx.begin_reconnect();
                eprintln!("{} Connection failed: {}", "[ERROR]".red().bold(), e);

                if !reconnect_ctx.should_retry() {
                    return Err("Failed to connect after max retries".into());
                }

                let delay = reconnect_ctx.backoff_delay();
                println!("   Retrying in {:.1}s...", delay.as_secs_f64());
                reconnect_ctx.wait_backoff();
            }
        }
    };

    // Main receiving loop
    let mut last_timestamp = 0u64;
    let mut message_count = 0u64;

    println!("Waiting for sensor data...\n");

    loop {
        match hub.recv() {
            Some(data) => {
                message_count += 1;

                // Check for message loss (timestamp gaps)
                if last_timestamp > 0 && data.timestamp_ns != last_timestamp + 1 {
                    let lost = data.timestamp_ns - last_timestamp - 1;
                    eprintln!(
                        "{} Detected {} lost messages",
                        "[WARNING]".yellow().bold(),
                        lost
                    );
                }
                last_timestamp = data.timestamp_ns;

                println!(
                    "Received: {} (total: {})",
                    data.log_summary(),
                    message_count
                );

                // Health check: alert if temperature out of range
                if data.temperature > 28.0 {
                    println!("   {} High temperature alert!", "[ALERT]".red().bold());
                }
            }
            None => {
                // No data available, wait briefly
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage:");
        eprintln!("  cargo run --example network_production --release -- publisher");
        eprintln!("  cargo run --example network_production --release -- subscriber");
        eprintln!();
        eprintln!("Start the router first:");
        eprintln!("  cargo run --release --bin horus_router");
        return Ok(());
    }

    match args[1].as_str() {
        "publisher" | "pub" => run_publisher(),
        "subscriber" | "sub" => run_subscriber(),
        _ => {
            eprintln!("Unknown mode: {}", args[1]);
            eprintln!("Use 'publisher' or 'subscriber'");
            Ok(())
        }
    }
}
