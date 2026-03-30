//! # HORUS Universal Types
//!
//! IPC-optimized message types used by ALL horus verticals — math primitives,
//! diagnostics, time synchronization, and generic cross-language messages.
//!
//! These types are `#[repr(C)]`, Pod-safe for zero-copy shared memory transfer.

pub mod diagnostics;
pub mod generic;
pub mod math;
pub mod time;

pub mod prelude;

// Re-export all types at crate root for convenience
pub use diagnostics::*;
pub use generic::{GenericMessage, MAX_GENERIC_PAYLOAD};
pub use math::*;
pub use time::*;
