//! Simple Topic SPSC Example
//!
//! This example demonstrates basic producer/consumer communication using Topic.
//! Topic provides ultra-low latency point-to-point IPC between two processes.
//!
//! Run this example:
//! ```bash
//! cargo run --example topic_spsc_simple
//! ```

use horus_core::memory::shm_topics_dir;
use horus_core::{Node, Topic, Scheduler};
use std::thread;
use std::time::Duration;

/// Producer node - sends sensor readings
struct SensorNode {
    data_link: Topic<f32>,
    counter: f32,
}

impl SensorNode {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            // Create as producer
            data_link: Topic::new("sensor_data")?,
            counter: 0.0,
        })
    }
}

impl Node for SensorNode {
    fn name(&self) -> &'static str {
        "SensorNode"
    }

    fn tick(&mut self) {
        self.counter += 0.1;

        // Simulate sensor reading
        let sensor_value = (self.counter.sin() * 100.0) + 100.0;

        // Send via Topic (ultra-low latency)
        match self.data_link.send(sensor_value) {
            Ok(()) => {
                println!("[{}] Sent: {:.2}", self.name(), sensor_value);
            }
            Err(value) => {
                eprintln!("[{}] Buffer full! Dropped: {:.2}", self.name(), value);
            }
        }

        // Control loop rate
        thread::sleep(Duration::from_millis(100));
    }
}

/// Consumer node - receives sensor readings and processes them
struct ControllerNode {
    data_link: Topic<f32>,
    last_value: Option<f32>,
}

impl ControllerNode {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            // Create as consumer
            data_link: Topic::new("sensor_data")?,
            last_value: None,
        })
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &'static str {
        "ControllerNode"
    }

    fn tick(&mut self) {
        // Non-blocking receive
        if let Some(value) = self.data_link.recv() {
            self.last_value = Some(value);
            println!("[{}] Received: {:.2}", self.name(), value);

            // Process the value
            if value > 150.0 {
                println!("[{}]   High reading detected!", self.name());
            }
        }

        // Always process last known value (even if no new message)
        if let Some(value) = self.last_value {
            let _control_output = value * 0.5; // Simple control law
        }

        thread::sleep(Duration::from_millis(50));
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== HORUS Topic (SPSC) Simple Example ===\n");
    println!("This example demonstrates ultra-low latency point-to-point IPC.\n");

    // Clean up any previous shared memory
    let _ = std::fs::remove_file(shm_topics_dir().join("horus_links_sensor_data"));

    // Create nodes
    let sensor = SensorNode::new()?;
    let controller = ControllerNode::new()?;

    println!("Topic topology:");
    println!("  SensorNode --[Topic: sensor_data]--> ControllerNode\n");

    // Create scheduler
    let mut scheduler = Scheduler::new();

    // Add nodes with priorities (lower order = runs first)
    scheduler.add(sensor)
        .order(0)  // High priority
        .done();
    scheduler.add(controller)
        .order(1)  // Normal priority
        .done();

    println!("Starting nodes... (Ctrl+C to stop)\n");

    // Run forever (Ctrl+C to stop)
    scheduler.run()?;

    Ok(())
}
