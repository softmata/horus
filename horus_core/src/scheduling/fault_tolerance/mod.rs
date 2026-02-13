/// Circuit breaker pattern for fault tolerance
pub mod circuit_breaker;

/// Tier-aware failure policies
pub mod failure_policy;

pub use circuit_breaker::{CircuitBreaker, CircuitState};
pub use failure_policy::{FailureAction, FailureHandler, FailureHandlerStats, FailurePolicy};
