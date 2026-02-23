/// Circuit breaker pattern for fault tolerance
pub mod circuit_breaker;

/// Tier-aware failure policies
pub mod failure_policy;

pub use circuit_breaker::CircuitState;
pub(crate) use failure_policy::{FailureAction, FailureHandler};
pub use failure_policy::{FailureHandlerStats, FailurePolicy};
