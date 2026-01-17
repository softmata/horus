#!/usr/bin/env rust
//! Rust Controller Node - Multi-Language Example
//!
//! Subscribes to robot pose data from Python node and generates velocity commands.
//! Demonstrates seamless Rust ↔ Python communication using standardized messages.

use horus::prelude::*;

// Controller node state
struct ControllerNode {
    pose_hub: Topic<Pose2D>,
    cmd_hub: Topic<CmdVel>,
    last_pose: Option<Pose2D>,
}

impl ControllerNode {
    fn new() -> Result<Self> {
        Ok(ControllerNode {
            pose_hub: Topic::<Pose2D>::new("robot_pose")?,
            cmd_hub: Topic::<CmdVel>::new("cmd_vel")?,
            last_pose: None,
        })
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &'static str {
        "controller_node"
    }
    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Try to receive typed Pose2D from Python node (automatic logging via ctx)
        if let Some(pose) = self.pose_hub.recv(&mut ctx) {
            // Simple proportional controller based on distance from origin
            let distance_from_origin = (pose.x * pose.x + pose.y * pose.y).sqrt();
            let linear_vel = if distance_from_origin > 1.5 {
                1.5 // Move forward if far from origin
            } else {
                0.5 // Slow down if close
            };

            // Angular velocity to maintain circular motion
            let angular_vel = 0.5;

            // Create typed CmdVel message - same API as Python!
            let cmd = CmdVel::new(linear_vel as f32, angular_vel as f32);

            // Send with automatic logging via ctx
            let _ = self.cmd_hub.send(cmd, &mut ctx);

            self.last_pose = Some(pose);
        }
    }
}

fn main() -> Result<()> {
    // Use HORUS scheduler - demonstrates cross-language communication
    // using standardized typed messages (Pose2D, CmdVel) from horus_library
    let mut scheduler = Scheduler::new().name("multi_language_example");

    scheduler.add(
        Box::new(ControllerNode::new()?),
        0,          // priority
        Some(true), // enable logging
    );

    scheduler.run()
}
