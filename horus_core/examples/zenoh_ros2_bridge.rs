//! Zenoh ROS2 Bridge Example
//!
//! This example demonstrates HORUS communicating with ROS2 nodes via Zenoh.
//! It uses the zenoh-bridge-ros2dds or native Zenoh ROS2 compatibility.
//!
//! Prerequisites:
//! - ROS2 Humble or later installed
//! - zenoh-bridge-ros2dds running (or zenoh-plugin-ros2dds)
//!
//! Run this example:
//! ```bash
//! # Terminal 1 - Start ROS2 Zenoh bridge
//! ros2 run zenoh_bridge_ros2dds zenoh_bridge_ros2dds
//!
//! # Terminal 2 - Run this HORUS node
//! cargo run --example zenoh_ros2_bridge --features zenoh-transport
//!
//! # Terminal 3 - ROS2 subscriber
//! ros2 topic echo /horus_status std_msgs/msg/String
//!
//! # Terminal 4 - ROS2 publisher (to test receiving)
//! ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
//! ```

use std::thread;
use std::time::Duration;

#[cfg(feature = "zenoh-transport")]
use horus_core::communication::Topic;
#[cfg(feature = "zenoh-transport")]
use horus_core::core::LogSummary;
#[cfg(feature = "zenoh-transport")]
use serde::{Deserialize, Serialize};

// ROS2-compatible Twist message (simplified)
// In real usage, you'd use horus_messages::geometry::Twist
#[cfg(feature = "zenoh-transport")]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

#[cfg(feature = "zenoh-transport")]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct Twist {
    linear: Vector3,
    angular: Vector3,
}

#[cfg(feature = "zenoh-transport")]
impl LogSummary for Twist {
    fn log_summary(&self) -> String {
        format!("Twist(lin={:.2}, ang={:.2})", self.linear.x, self.angular.z)
    }
}

// Simple string message for status
#[cfg(feature = "zenoh-transport")]
#[derive(Debug, Clone, Serialize, Deserialize)]
struct StringMsg {
    data: String,
}

#[cfg(feature = "zenoh-transport")]
impl LogSummary for StringMsg {
    fn log_summary(&self) -> String {
        format!("String({})", &self.data[..self.data.len().min(20)])
    }
}

#[cfg(feature = "zenoh-transport")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== HORUS Zenoh ROS2 Bridge Example ===\n");
    println!("This demonstrates HORUS <-> ROS2 communication via Zenoh.\n");

    println!("Configuration:");
    println!("  Mode: ROS2 compatible");
    println!("  Domain ID: 0");
    println!("  Topic naming: rt/<topic> (ROS2 convention)\n");

    // Publisher for status messages
    // In ROS2 mode, "horus_status" becomes "rt/horus_status"
    let status_hub: Topic<StringMsg> = Topic::new("horus_status@zenoh/ros2")?;

    // Subscriber for cmd_vel (ROS2 Twist messages)
    let cmd_vel_hub: Topic<Twist> = Topic::new("cmd_vel@zenoh/ros2")?;

    println!("Topics:");
    println!("  Publishing to: /horus_status (String)");
    println!("  Subscribing to: /cmd_vel (Twist)\n");

    println!("Running... (Ctrl+C to stop)\n");
    println!("To test:");
    println!("  ROS2 subscriber: ros2 topic echo /horus_status std_msgs/msg/String");
    println!("  ROS2 publisher:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{{linear: {{x: 0.5}}}}\"\n");

    let mut tick = 0u64;

    loop {
        // Send status
        let status = StringMsg {
            data: format!("HORUS node alive, tick={}", tick),
        };
        let mut send_ctx: Option<&mut horus_core::NodeInfo> = None;
        let _ = status_hub.send(status);
        println!("[HORUS] Published status: tick={}", tick);

        // Check for cmd_vel commands (non-blocking recv with no context)
        let mut ctx: Option<&mut horus_core::NodeInfo> = None;
        while let Some(twist) = cmd_vel_hub.recv() {
            println!(
                "[HORUS] Received cmd_vel: linear=({:.2}, {:.2}, {:.2}), angular=({:.2}, {:.2}, {:.2})",
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z
            );

            // Process the command (in real code, would control motors)
            if twist.linear.x > 0.0 {
                println!("  -> Moving forward at {:.2} m/s", twist.linear.x);
            } else if twist.linear.x < 0.0 {
                println!("  -> Moving backward at {:.2} m/s", twist.linear.x.abs());
            }
            if twist.angular.z.abs() > 0.01 {
                println!("  -> Rotating at {:.2} rad/s", twist.angular.z);
            }
        }

        tick += 1;
        thread::sleep(Duration::from_secs(1));
    }
}

#[cfg(not(feature = "zenoh-transport"))]
fn main() {
    println!("This example requires the zenoh-transport feature.");
    println!("Run with: cargo run --example zenoh_ros2_bridge --features zenoh-transport");
}
