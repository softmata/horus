//! # HORUS Core
//!
//! The core runtime system for the HORUS robotics framework.
//!
//! HORUS is a distributed real-time robotics system designed for high-performance
//! applications. This crate provides the fundamental building blocks:
//!
//! - **Nodes**: Independent computational units that process data
//! - **Communication**: Publisher-subscriber message passing between nodes
//! - **Memory**: High-performance shared memory and zero-copy messaging
//! - **Scheduling**: Real-time task scheduling and execution
//! - **Monitoring**: Cross-process system monitoring and diagnostics
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus_core::{Node, NodeInfo, Scheduler, Hub};
//!
//! struct ExampleNode {
//!     output: Hub<String>,
//! }
//!
//! impl Node for ExampleNode {
//!     fn name(&self) -> &'static str { "example" }
//!
//!     fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
//!         let _ = self.output.send("Hello HORUS!".into(), &mut ctx);
//!     }
//! }
//! ```

pub mod backend;
pub mod communication;
pub mod core;
pub mod driver;
pub mod error;
pub mod hardware;
pub mod memory;
pub mod ml;
pub mod params;
pub mod scheduling;

// Re-export commonly used types for easy access
pub use communication::{Hub, Link, LinkMetrics, PodLink, PodMessage};
pub use core::{
    HealthStatus, Node, NodeConfig, NodeHeartbeat, NodeInfo, NodeInfoExt, NodeMetrics, NodeState,
    TopicMetadata,
};
pub use error::{HorusError, HorusResult};
// Clean aliases for user-facing API
pub use error::{Error, Result};
pub use params::RuntimeParams;
pub use scheduling::Scheduler;

// Re-export communication traits for backend-agnostic usage
pub use communication::traits::{Channel, Publisher, Subscriber};

// Re-export driver utilities (no traits - drivers are simple structs)
pub use driver::{DriverCategory, DriverStatus, DriversConfig, SingleDriverConfig};
