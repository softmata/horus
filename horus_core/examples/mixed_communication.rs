use horus_core::scheduling::Scheduler;
use horus_core::{Node, Result, Topic};
use std::time::Duration;

// Node that publishes telemetry data (multiple subscribers can consume)
pub struct TelemetryNode {
    telemetry_hub: Topic<f32>,
    counter: f32,
}

impl TelemetryNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            telemetry_hub: Topic::new("telemetry")?,
            counter: 0.0,
        })
    }
}

impl Node for TelemetryNode {
    fn name(&self) -> &'static str {
        "TelemetryNode"
    }

    fn init(&mut self) -> Result<()> {
        println!("TelemetryNode initialized - broadcasting via Topic");
        Ok(())
    }

    fn tick(&mut self) {
        self.counter += 1.0;
        self.telemetry_hub.send(self.counter).ok();
        if self.counter as u32 % 100 == 0 {
            println!("[TelemetryNode] Published: {}", self.counter);
        }
    }
}

// Control node for real-time control (single producer/consumer, lowest latency)
pub struct ControlNode {
    command_link: Topic<f32>,
    response_link: Topic<f32>,
}

impl ControlNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            command_link: Topic::new("command")?,
            response_link: Topic::new("response")?,
        })
    }
}

impl Node for ControlNode {
    fn name(&self) -> &'static str {
        "ControlNode"
    }

    fn init(&mut self) -> Result<()> {
        println!("ControlNode initialized - using Topic for low-latency control");
        Ok(())
    }

    fn tick(&mut self) {
        // Send control command through Topic (ultra-low latency SPSC backend)
        let command = 42.0;
        self.command_link.send(command).ok();

        // Check for response
        if let Some(response) = self.response_link.recv() {
            if response as u32 % 100 == 0 {
                println!("[ControlNode] Got response: {}", response);
            }
        }
    }
}

// Actuator node that receives commands (low latency)
pub struct ActuatorNode {
    command_link: Topic<f32>,
    response_link: Topic<f32>,
    processed: u32,
}

impl ActuatorNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            command_link: Topic::new("command")?,
            response_link: Topic::new("response")?,
            processed: 0,
        })
    }
}

impl Node for ActuatorNode {
    fn name(&self) -> &'static str {
        "ActuatorNode"
    }

    fn init(&mut self) -> Result<()> {
        println!("ActuatorNode initialized - receiving via Topic");
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(command) = self.command_link.recv() {
            self.processed += 1;
            // Process command and send response
            let response = command + self.processed as f32;
            self.response_link.send(response).ok();
        }
    }
}

// Logger node that subscribes to telemetry (one of many possible subscribers)
pub struct LoggerNode {
    telemetry_hub: Topic<f32>,
    logs_received: u32,
}

impl LoggerNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            telemetry_hub: Topic::new("telemetry")?,
            logs_received: 0,
        })
    }
}

impl Node for LoggerNode {
    fn name(&self) -> &'static str {
        "LoggerNode"
    }

    fn init(&mut self) -> Result<()> {
        println!("LoggerNode initialized - subscribing to Topic");
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(telemetry) = self.telemetry_hub.recv() {
            self.logs_received += 1;
            if self.logs_received % 100 == 0 {
                println!(
                    "[LoggerNode] Logged telemetry #{}: value={}",
                    self.logs_received, telemetry
                );
            }
        }
    }
}

// Analytics node that also subscribes to telemetry (demonstrating multiple subscribers)
pub struct AnalyticsNode {
    telemetry_hub: Topic<f32>,
    sum: f32,
    count: u32,
}

impl AnalyticsNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            telemetry_hub: Topic::new("telemetry")?,
            sum: 0.0,
            count: 0,
        })
    }
}

impl Node for AnalyticsNode {
    fn name(&self) -> &'static str {
        "AnalyticsNode"
    }

    fn init(&mut self) -> Result<()> {
        println!("AnalyticsNode initialized - also subscribing to Topic");
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(telemetry) = self.telemetry_hub.recv() {
            self.sum += telemetry;
            self.count += 1;
            if self.count % 1000 == 0 {
                let avg = self.sum / self.count as f32;
                println!(
                    "[AnalyticsNode] Average after {} samples: {:.2}",
                    self.count, avg
                );
            }
        }
    }
}

fn main() -> Result<()> {
    println!("=== Mixed Communication Example ===");
    println!("Demonstrating Topic API with automatic backend selection:\n");
    println!("- Control loop uses Topic with SPSC backend (248ns latency) for real-time control");
    println!("- Telemetry uses Topic with MPMC backend (437ns latency) for multiple subscribers\n");

    // Create scheduler
    let mut scheduler = Scheduler::new();

    // Add nodes for low-latency control (Topic will auto-select SPSC backend)
    scheduler.add(ControlNode::new()?)
        .order(10)
        .done();
    scheduler.add(ActuatorNode::new()?)
        .order(10)
        .done();

    // Add nodes for broadcast (Topic will auto-select MPMC backend)
    scheduler.add(TelemetryNode::new()?)
        .order(20)
        .done();
    scheduler.add(LoggerNode::new()?)
        .order(100)
        .done();
    scheduler.add(AnalyticsNode::new()?)
        .order(100)
        .done();

    println!("Running scheduler with 5 nodes (Topic auto-selects optimal backend)...\n");

    // Run for a short time
    scheduler.run_for(Duration::from_secs(3))?;

    println!("\n=== Demonstration Complete ===");
    println!("✓ Control nodes communicated via Topic with SPSC backend (lowest latency)");
    println!(" Telemetry was broadcast via Hub to multiple subscribers");
    println!(" All nodes coexisted in the same scheduler");

    Ok(())
}
