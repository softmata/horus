//! # HORUS Standard Library
//!
//! **Note:** This is an internal crate. Users should depend on `horus` and
//! import from `horus::prelude::*`.

#[doc(hidden)]
pub mod messages;
pub mod simulation;
pub mod topic_convention;
#[doc(hidden)]
pub mod transform_frame;

// Re-export core traits needed for message types
#[doc(hidden)]
pub use horus_core::core::LogSummary;

// Re-export message types at the crate root for convenience
#[doc(hidden)]
pub use messages::*;

// Re-export softmata-core for canonical type access
#[doc(hidden)]
pub use softmata_core;
