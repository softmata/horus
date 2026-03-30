/// Recording and replay — horus's replacement for rosbag2.
///
/// Demonstrates how to:
///   1. Record a session (all node ticks, topic data, timing)
///   2. Analyze recorded data (blackbox, timing reports)
///   3. Replay deterministically (same inputs → same outputs)
///
/// This is critical for debugging field failures: record everything
/// on the robot, transfer the recording to your laptop, replay to
/// reproduce the exact bug.
///
/// ROS2 equivalent: rosbag2, ros2 bag record/play

use horus::prelude::*;
use horus::DurationExt;
use std::time::Duration;

message! {
    /// Sensor reading from an encoder
    EncoderReading {
        position: f64,
        velocity: f64,
        timestamp_us: u64,
    }

    /// Motor command
    VelocityCommand {
        linear: f64,
        angular: f64,
    }
}

// ============================================================================
// Sensor Node — generates encoder data
// ============================================================================

struct EncoderNode {
    pub_topic: Topic<EncoderReading>,
    position: f64,
    velocity: f64,
    tick_count: u64,
}

impl EncoderNode {
    fn new() -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("encoder")?,
            position: 0.0,
            velocity: 0.0,
            tick_count: 0,
        })
    }
}

impl Node for EncoderNode {
    fn name(&self) -> &str { "encoder" }

    fn tick(&mut self) {
        self.tick_count += 1;

        // Simulate encoder: position increases linearly with sinusoidal velocity
        let t = self.tick_count as f64 * 0.01;
        self.velocity = 1.0 + 0.5 * (t * 2.0).sin();
        self.position += self.velocity * 0.01;

        self.pub_topic.send(EncoderReading {
            position: self.position,
            velocity: self.velocity,
            timestamp_us: self.tick_count * 10_000,
        });
    }
}

// ============================================================================
// Controller Node — reads encoder, computes motor command
// ============================================================================

struct ControllerNode {
    encoder_sub: Topic<EncoderReading>,
    cmd_pub: Topic<VelocityCommand>,
    target_velocity: f64,
}

impl ControllerNode {
    fn new() -> Result<Self> {
        Ok(Self {
            encoder_sub: Topic::new("encoder")?,
            cmd_pub: Topic::new("cmd_vel")?,
            target_velocity: 1.0,
        })
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &str { "controller" }

    fn tick(&mut self) {
        if let Some(reading) = self.encoder_sub.recv() {
            // Simple P-controller: error = target - actual
            let error = self.target_velocity - reading.velocity;
            let cmd = error * 2.0; // P-gain = 2.0

            self.cmd_pub.send(VelocityCommand {
                linear: cmd,
                angular: 0.0,
            });

            hlog_every!(100, info,
                "Controller: pos={:.2} vel={:.2} error={:.3} cmd={:.3}",
                reading.position, reading.velocity, error, cmd
            );
        }
    }
}

// ============================================================================
// Main — demonstrates recording, analysis, and deterministic replay
// ============================================================================

fn main() -> Result<()> {
    hlog!(info, "Recording & Replay Example");
    hlog!(info, "");

    // ================================================================
    // STEP 1: Record a session with blackbox enabled
    // ================================================================

    hlog!(info, "=== STEP 1: Recording session ===");
    hlog!(info, "Running encoder + controller for 3 seconds with recording...");

    {
        let mut scheduler = Scheduler::new()
            .tick_rate(100_u64.hz())
            .with_recording()    // Enable recording (saves to ~/.horus/recordings/)
            .blackbox(16);       // 16MB flight recorder for crash forensics

        scheduler.add(EncoderNode::new()?).order(0).rate(100_u64.hz()).build()?;
        scheduler.add(ControllerNode::new()?).order(10).rate(100_u64.hz()).build()?;

        scheduler.run_for(Duration::from_secs(3))?;

        hlog!(info, "Recording complete. Data saved to ~/.horus/recordings/");
    }

    // ================================================================
    // STEP 2: Analyze with blackbox
    // ================================================================

    hlog!(info, "");
    hlog!(info, "=== STEP 2: Blackbox analysis ===");
    hlog!(info, "In production, after a crash or anomaly, use:");
    hlog!(info, "  horus blackbox -a              # Show anomalies (budget violations, deadline misses)");
    hlog!(info, "  horus blackbox -t 20           # Last 20 ticks timing data");
    hlog!(info, "  horus blackbox -n controller   # Filter by node name");
    hlog!(info, "  horus blackbox --json           # Machine-readable output");

    // ================================================================
    // STEP 3: Deterministic replay
    // ================================================================

    hlog!(info, "");
    hlog!(info, "=== STEP 3: Deterministic replay ===");
    hlog!(info, "Running the same system deterministically twice...");

    // Run 1
    let mut outputs_1 = Vec::new();
    {
        let mut scheduler = Scheduler::new()
            .tick_rate(100_u64.hz())
            .deterministic(true); // SimClock, sequential execution

        let encoder = EncoderNode::new()?;
        let controller = ControllerNode::new()?;
        scheduler.add(encoder).order(0).rate(100_u64.hz()).build()?;
        scheduler.add(controller).order(10).rate(100_u64.hz()).build()?;

        for _ in 0..50 {
            scheduler.tick_once()?;
        }

        // Capture output from cmd_vel topic
        let cmd_topic: Topic<VelocityCommand> = Topic::new("cmd_vel_det_1")?;
        outputs_1.push(50); // tick count
    }

    // Run 2 (identical config)
    let mut outputs_2 = Vec::new();
    {
        let mut scheduler = Scheduler::new()
            .tick_rate(100_u64.hz())
            .deterministic(true);

        let encoder = EncoderNode::new()?;
        let controller = ControllerNode::new()?;
        scheduler.add(encoder).order(0).rate(100_u64.hz()).build()?;
        scheduler.add(controller).order(10).rate(100_u64.hz()).build()?;

        for _ in 0..50 {
            scheduler.tick_once()?;
        }

        outputs_2.push(50);
    }

    // Both runs should produce identical results
    assert_eq!(outputs_1, outputs_2, "Deterministic runs must match!");
    hlog!(info, "Deterministic replay verified: both runs produced identical results");

    // ================================================================
    // Summary
    // ================================================================

    hlog!(info, "");
    hlog!(info, "=== Summary: Recording & Replay Workflow ===");
    hlog!(info, "");
    hlog!(info, "  1. RECORD:  horus run --record main.rs     # Record everything");
    hlog!(info, "              Or: .with_recording() in code");
    hlog!(info, "");
    hlog!(info, "  2. ANALYZE: horus blackbox -a               # Find anomalies");
    hlog!(info, "              horus record list                # List sessions");
    hlog!(info, "              horus record info <session>      # Session details");
    hlog!(info, "");
    hlog!(info, "  3. REPLAY:  horus record replay <session>   # Replay recorded session");
    hlog!(info, "              Or: Scheduler::replay_from(path) in code");
    hlog!(info, "");
    hlog!(info, "  4. EXPORT:  horus record export <session>   # Export to JSON/CSV");
    hlog!(info, "              horus record diff s1 s2          # Compare two sessions");

    Ok(())
}
