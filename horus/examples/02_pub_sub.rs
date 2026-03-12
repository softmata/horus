//! # Example 2: Publish / Subscribe
//!
//! Two nodes communicate via a Topic — one publishes sensor data,
//! the other subscribes and processes it.
//!
//! ```bash
//! cargo run --example 02_pub_sub
//! ```

use horus::prelude::*;


// Define a custom message type with the message! macro.
// This auto-derives Clone, Debug, Serialize, Deserialize, and LogSummary.
message! {
    /// Temperature reading from a sensor
    TemperatureReading {
        sensor_id: u32,
        celsius: f64,
    }
}

/// Publishes temperature readings.
struct SensorNode {
    publisher: Topic<TemperatureReading>,
    tick_count: u32,
}

impl SensorNode {
    fn new() -> Result<Self> {
        Ok(Self {
            publisher: Topic::new("temperature")?,
            tick_count: 0,
        })
    }
}

impl Node for SensorNode {
    fn name(&self) -> &str {
        "SensorNode"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        let reading = TemperatureReading {
            sensor_id: 1,
            celsius: 20.0 + (self.tick_count as f64 * 0.5),
        };
        println!("[Sensor] Publishing: {:.1}°C", reading.celsius);
        self.publisher.send(reading);
    }
}

/// Subscribes to temperature readings and checks for alerts.
struct MonitorNode {
    subscriber: Topic<TemperatureReading>,
}

impl MonitorNode {
    fn new() -> Result<Self> {
        Ok(Self {
            subscriber: Topic::new("temperature")?,
        })
    }
}

impl Node for MonitorNode {
    fn name(&self) -> &str {
        "MonitorNode"
    }

    fn tick(&mut self) {
        if let Some(reading) = self.subscriber.recv() {
            if reading.celsius > 22.0 {
                println!(
                    "[Monitor] WARNING: Sensor {} at {:.1}°C (above threshold)",
                    reading.sensor_id, reading.celsius
                );
            } else {
                println!(
                    "[Monitor] OK: Sensor {} at {:.1}°C",
                    reading.sensor_id, reading.celsius
                );
            }
        }
    }
}

fn main() -> Result<()> {
    println!("=== HORUS Example 2: Pub/Sub ===\n");

    let mut scheduler = Scheduler::new().tick_rate(2_u64.hz());

    // Sensor publishes first (order 0), monitor reads after (order 1)
    scheduler.add(SensorNode::new()?).order(0).build()?;
    scheduler.add(MonitorNode::new()?).order(1).build()?;

    scheduler.run_for(4_u64.secs())?;

    println!("\nDone!");
    Ok(())
}
