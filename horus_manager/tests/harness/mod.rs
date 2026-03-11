//! Live test harness for HORUS monitor acceptance testing.
//!
//! Provides [`HorusTestRuntime`] which can spin up simulated nodes by writing
//! presence files, SHM topic files, and blackbox events — exactly what the
//! monitor's discovery layer reads.  No real processes need to be spawned:
//! we use the current process's PID so the liveness checks pass.
//!
//! # Example
//!
//! ```rust,ignore
//! let mut rt = HorusTestRuntime::new();
//! rt.add_node(TestNodeConfig::sensor("lidar", "scan_data", "LaserScan"))
//!   .add_topic("scan_data", 4096);
//!
//! assert!(rt.wait_ready(2_u64.secs()));
//!
//! let nodes = horus_manager::discovery::discover_nodes().unwrap();
//! assert!(nodes.iter().any(|n| n.name == "lidar"));
//! // Cleanup happens automatically on drop.
//! ```

use horus_core::core::DurationExt;
mod fixtures;
mod runtime;

#[allow(unused_imports)]
pub use fixtures::TestNodeConfig;
#[allow(unused_imports)]
pub use runtime::HorusTestRuntime;
