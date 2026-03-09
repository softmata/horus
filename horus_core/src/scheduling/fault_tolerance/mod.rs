/// Circuit breaker pattern for fault tolerance
pub mod circuit_breaker;

/// Tier-aware failure policies
pub mod failure_policy;

pub use circuit_breaker::CircuitState;
pub use failure_policy::FailureHandler;
pub use failure_policy::{FailureHandlerStats, FailurePolicy};
