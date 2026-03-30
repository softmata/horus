//! # HORUS C++ Bindings
//!
//! This crate provides the FFI boundary between Rust and C++ for the HORUS
//! robotics framework. It wraps `horus_core` types in opaque, FFI-safe wrappers
//! and exposes them via CXX bridge modules.
//!
//! ## Architecture
//!
//! ```text
//! C++ user code
//!   └─ horus/horus.hpp (generated C++ headers)
//!       └─ CXX bridge (this crate)
//!           └─ horus_core (Rust runtime)
//! ```
//!
//! ## Modules
//!
//! - `scheduler_ffi` — Scheduler creation, configuration, run loop
//! - `node_ffi` — NodeBuilder, CppNodeAdapter, tick callbacks
//! - `topic_ffi` — Publisher/Subscriber with loan pattern (zero-copy)
//! - `error_ffi` — Error type mapping (HorusError → C++ exception)
//! - `types_ffi` — Shared type definitions (Duration, Frequency, Miss, etc.)

// FFI wrapper modules — each contains opaque types + FFI functions
mod types_ffi;
mod scheduler_ffi;
mod node_ffi;
mod topic_ffi;
mod json_service;
mod service_ffi;
mod action_ffi;
mod transform_ffi;
mod params_ffi;
mod pool_ffi;
mod c_api;

#[cfg(test)]
mod layout_tests;
#[cfg(test)]
mod e2e_tests;
#[cfg(test)]
mod roundtrip_tests;

// Re-export for build.rs bridge compilation
pub use types_ffi::*;
pub use scheduler_ffi::*;
pub use node_ffi::*;
pub use topic_ffi::*;
pub use service_ffi::*;
pub use action_ffi::*;
pub use transform_ffi::*;
pub use params_ffi::*;
pub use pool_ffi::*;
