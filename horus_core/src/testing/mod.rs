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
