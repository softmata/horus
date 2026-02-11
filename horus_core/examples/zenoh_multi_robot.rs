//! Zenoh Multi-Robot Mesh Example
//!
//! This example demonstrates multi-robot communication using Zenoh transport.
//! Zenoh enables seamless pub/sub across multiple robots without a central broker.
//!
//! Features demonstrated:
//! - Multi-robot mesh networking (peer-to-peer)
//! - Cloud connectivity for remote monitoring
//! - ROS2 interoperability mode
//!
//! Run this example:
//! ```bash
//! # Terminal 1 - Robot 1
//! cargo run --example zenoh_multi_robot --features zenoh-transport -- robot1
//!
//! # Terminal 2 - Robot 2
//! cargo run --example zenoh_multi_robot --features zenoh-transport -- robot2
//!
//! # Terminal 3 - Monitor (receives from all robots)
//! cargo run --example zenoh_multi_robot --features zenoh-transport -- monitor
//! ```

#[cfg(feature = "zenoh-transport")]
use std::thread;
#[cfg(feature = "zenoh-transport")]
use std::time::Duration;

#[cfg(feature = "zenoh-transport")]
use horus_core::communication::Topic;
#[cfg(feature = "zenoh-transport")]
use horus_core::core::LogSummary;
#[cfg(feature = "zenoh-transport")]
use serde::{Deserialize, Serialize};

/// Robot pose message (shared between all robots)
#[cfg(feature = "zenoh-transport")]
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    robot_id: String,
    x: f64,
    y: f64,
    theta: f64,
    timestamp: u64,
}

#[cfg(feature = "zenoh-transport")]
impl LogSummary for RobotPose {
    fn log_summary(&self) -> String {
        format!("Pose({}: {:.2},{:.2})", self.robot_id, self.x, self.y)
    }
}

#[cfg(feature = "zenoh-transport")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== HORUS Zenoh Multi-Robot Example ===\n");

    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    let role = args.get(1).map(|s| s.as_str()).unwrap_or("robot1");

    match role {
        "robot1" => run_robot("robot1", 0.0, 0.0),
        "robot2" => run_robot("robot2", 5.0, 5.0),
        "monitor" => run_monitor(),
        _ => {
            println!("Usage: zenoh_multi_robot [robot1|robot2|monitor]");
            Ok(())
        }
    }
}

#[cfg(feature = "zenoh-transport")]
fn run_robot(robot_id: &str, start_x: f64, start_y: f64) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Starting {} at position ({}, {})",
        robot_id, start_x, start_y
    );
    println!("Using Zenoh peer-to-peer mesh networking\n");

    // Create publisher hub for poses using endpoint string format
    // Format: topic@zenoh
    let pose_hub: Topic<RobotPose> = Topic::new("robot_poses@zenoh")?;

    // Subscribe to poses from other robots
    let peer_hub: Topic<RobotPose> = Topic::new("robot_poses@zenoh")?;

    println!("Topology:");
    println!(
        "  {} --[Zenoh: robot_poses]--> All other robots\n",
        robot_id
    );

    let mut x;
    let mut y;
    let mut theta = 0.0f64;
    let mut tick = 0u64;

    println!("Publishing poses... (Ctrl+C to stop)\n");

    loop {
        // Simulate robot movement (circular path)
        theta += 0.05;
        x = start_x + theta.cos() * 2.0;
        y = start_y + theta.sin() * 2.0;

        let pose = RobotPose {
            robot_id: robot_id.to_string(),
            x,
            y,
            theta,
            timestamp: tick,
        };

        // Send our pose
        pose_hub.send(pose.clone());
        println!(
            "[{}] Published: x={:.2}, y={:.2}, theta={:.2}",
            robot_id, x, y, theta
        );

        // Check for peer poses
        while let Some(peer_pose) = peer_hub.recv() {
            if peer_pose.robot_id != robot_id {
                println!(
                    "  -> Received from {}: x={:.2}, y={:.2}",
                    peer_pose.robot_id, peer_pose.x, peer_pose.y
                );
            }
        }

        tick += 1;
        thread::sleep(Duration::from_millis(500));
    }
}

#[cfg(feature = "zenoh-transport")]
fn run_monitor() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting fleet monitor");
    println!("Subscribing to all robot poses via Zenoh mesh\n");

    // Create subscriber hub for all poses
    let pose_hub: Topic<RobotPose> = Topic::new("robot_poses@zenoh")?;

    println!("Listening for robot poses... (Ctrl+C to stop)\n");

    let mut robot_positions: std::collections::HashMap<String, RobotPose> =
        std::collections::HashMap::new();

    loop {
        // Receive all available poses
        while let Some(pose) = pose_hub.recv() {
            let robot_id = pose.robot_id.clone();
            robot_positions.insert(robot_id.clone(), pose.clone());

            println!(
                "[Monitor] {} -> ({:.2}, {:.2}) theta={:.2}",
                robot_id, pose.x, pose.y, pose.theta
            );
        }

        // Print fleet summary every few seconds
        if !robot_positions.is_empty() {
            println!("\n--- Fleet Status ({} robots) ---", robot_positions.len());
            for (id, pose) in &robot_positions {
                println!("  {}: ({:.2}, {:.2})", id, pose.x, pose.y);
            }
            println!("---\n");
        }

        thread::sleep(Duration::from_secs(2));
    }
}

#[cfg(not(feature = "zenoh-transport"))]
fn main() {
    println!("This example requires the zenoh-transport feature.");
    println!("Run with: cargo run --example zenoh_multi_robot --features zenoh-transport");
}
