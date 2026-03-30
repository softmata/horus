//! Test utilities for HORUS — available with the `test-utils` feature.
//!
//! Provides mock implementations and fault injection utilities for testing
//! error paths and failure recovery without requiring real infrastructure.

mod mock_topic;
pub mod shm_fault;
mod test_node;

pub use mock_topic::{MockTopic, MockTopicConfig};
pub use shm_fault::{MockNetworkEndpoint, NetworkFault, ShmFault, ShmFaultInjector};
pub use test_node::{TestNode, TestNodeBuilder};

/// Spawn a test thread with a reduced stack size (512KB).
///
/// Identical to `std::thread::spawn` but uses 512KB stacks instead of the
/// default 2MB. This prevents stack overflow when `cargo test` runs many
/// multi-threaded tests in parallel (48 tests × 6 threads × 2MB = 576MB
/// vs 48 × 6 × 512KB = 144MB).
///
/// Test closures are small (spin loops, barrier waits, send/recv) — 512KB
/// is more than sufficient.
pub fn test_spawn<F, T>(f: F) -> std::thread::JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    std::thread::Builder::new()
        .stack_size(512 * 1024)
        .spawn(f)
        .expect("test_spawn: failed to spawn thread")
}
