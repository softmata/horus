//! Common traits for HORUS IPC implementations
//!
//! This module defines the abstraction layer for HORUS native IPC,
//! providing a consistent API for nodes and schedulers.

use crate::error::HorusResult;
use std::fmt::Debug;

/// Common trait for publisher/sender implementations
pub trait Publisher<T>: Send + Sync + Clone + Debug {
    /// Send a message - returns Ok on success, Err on failure
    fn send(&self, msg: T) -> HorusResult<()>;

    /// Try to send without blocking
    fn try_send(&self, msg: T) -> bool {
        self.send(msg).is_ok()
    }
}

/// Common trait for subscriber/receiver implementations
pub trait Subscriber<T>: Send + Sync + Clone + Debug {
    /// Receive a message without blocking
    fn recv(&self) -> Option<T>;

    /// Check if messages are available
    fn has_messages(&self) -> bool {
        false // Default implementation
    }
}

/// Common trait for point-to-point communication
pub trait Channel<T>: Publisher<T> + Subscriber<T> {
    /// Create a new channel with specified capacity
    fn new(capacity: usize) -> Self;
}
