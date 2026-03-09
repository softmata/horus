//! # Example 5: Real-Time Nodes
//!
//! Shows how to use real-time Node methods for time-critical applications like
//! motor control, with tick budgets and deadline monitoring.
//!
//! ```bash
//! cargo run --example 05_realtime
//! ```

use horus::prelude::*;
use std::time::Duration;

message! {
    /// Motor command
    MotorCmd {
        position: f64,
        velocity: f64,
    }
}

/// A PID controller running as a real-time node.
///
/// Real-time nodes get dedicated thread execution with deadline monitoring.
struct PidController {
    publisher: Topic<MotorCmd>,
    setpoint: f64,
    current: f64,
    kp: f64,
    ticks: u32,
}

impl PidController {
    fn new() -> Result<Self> {
        Ok(Self {
            publisher: Topic::new("motor_cmd")?,
            setpoint: 1.0,
            current: 0.0,
            kp: 0.5,
            ticks: 0,
        })
    }
}

impl Node for PidController {
    fn name(&self) -> &str {
        "PidController"
    }

    fn tick(&mut self) {
        self.ticks += 1;

        // Simple P-controller
        let error = self.setpoint - self.current;
        let output = self.kp * error;
        self.current += output * 0.01; // Simulate plant response

        let cmd = MotorCmd {
            position: self.current,
            velocity: output,
        };

        if self.ticks % 50 == 0 {
            println!(
                "[PID @100Hz] tick {} — pos={:.3}, vel={:.3}, err={:.3}",
                self.ticks, cmd.position, cmd.velocity, error
            );
        }

        self.publisher.send(cmd);
    }

    fn tick_budget(&self) -> Option<Duration> {
        Some(Duration::from_micros(200)) // 200μs max execution time
    }

    fn deadline(&self) -> Duration {
        Duration::from_millis(1) // 1ms deadline for 1kHz control
    }

    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        DeadlineMissPolicy::Skip // Occasional miss tolerated
    }
}

/// A safety monitor running at lower priority.
struct SafetyMonitor {
    subscriber: Topic<MotorCmd>,
    max_velocity: f64,
    ticks: u32,
}

impl SafetyMonitor {
    fn new() -> Result<Self> {
        Ok(Self {
            subscriber: Topic::new("motor_cmd")?,
            max_velocity: 2.0,
            ticks: 0,
        })
    }
}

impl Node for SafetyMonitor {
    fn name(&self) -> &str {
        "SafetyMonitor"
    }

    fn tick(&mut self) {
        self.ticks += 1;
        if let Some(cmd) = self.subscriber.recv() {
            if cmd.velocity.abs() > self.max_velocity {
                println!(
                    "[Safety @10Hz] ALERT: velocity {:.2} exceeds limit {:.2}!",
                    cmd.velocity, self.max_velocity
                );
            } else if self.ticks % 5 == 0 {
                println!(
                    "[Safety @10Hz] tick {} — all nominal (vel={:.3})",
                    self.ticks, cmd.velocity
                );
            }
        }
    }
}

fn main() -> Result<()> {
    println!("=== HORUS Example 5: Real-Time Nodes ===\n");

    let mut scheduler = Scheduler::new().tick_hz(100.0);

    // PID controller: RT node on dedicated thread
    scheduler
        .add(PidController::new()?)
        .order(0)
        .rate_hz(100.0)
        .build()?;

    // Safety monitor: best-effort at 10 Hz
    scheduler
        .add(SafetyMonitor::new()?)
        .order(10)
        .rate_hz(10.0)
        .build()?;

    // Use NodeTier for explicit classification
    println!("Node tiers:");
    println!(
        "  PID: UltraFast (default policy: {:?})",
        NodeTier::UltraFast.default_failure_policy()
    );
    println!(
        "  Safety: Normal (default policy: {:?})",
        NodeTier::Normal.default_failure_policy()
    );
    println!();

    scheduler.run_for(Duration::from_secs(3))?;

    println!("\nDone!");
    Ok(())
}
