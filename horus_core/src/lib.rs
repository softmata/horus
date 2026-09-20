//! # HORUS Core
//!
//! The core runtime system for the HORUS robotics framework.
//!
//! HORUS is a distributed real-time robotics system designed for high-performance
//! applications. This crate provides the fundamental building blocks:
//!
//! - **Nodes**: Independent computational units that process data
//! - **Communication**: Publisher-subscriber message passing between nodes
//! - **Memory**: High-performance shared memory and zero-copy messaging
//! - **Scheduling**: Real-time task scheduling and execution
//! - **Monitoring**: Cross-process system monitoring and diagnostics
//! - **Actions**: Long-running tasks with progress feedback and cancellation
//!
//! **Note:** This is an internal crate. Users should depend on `horus` and
//! import from `horus::prelude::*`.

// Public modules — accessible cross-crate, hidden from user docs by default.
// Users should go through `horus::prelude`, not import from `horus_core`
// directly, and the crate note above says so.
//
// The hiding is `cfg_attr`'d rather than unconditional so a MAINTAINER can lift
// it on stable:
//
//     cargo doc -p horus_core --no-deps --document-private-items \
//       --features internal-docs --open
//
// Before this was a feature, the only way to read the internals was the nightly
// incantation in horus_core/README.md — `-Z unstable-options
// --document-hidden-items` — because `--document-private-items` disables
// rustdoc's private-stripping pass and not its hidden-stripping pass. That put
// the API browser for a 112k-line crate behind an unstable flag, so on stable
// `cargo doc --open` showed one module (`terminal`), zero structs and an empty
// search index.
//
// CI documents WITH the feature (ci.yml, Documentation job) precisely so
// `-D warnings` sees these modules: while they were unconditionally hidden,
// rustdoc never resolved their links and four broken ones had accumulated,
// including a `HorusError::OutOfRange` the error refactor had deleted.
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod actions;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod communication;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod core;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod drivers;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod error;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod memory;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod params;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod scheduling;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod services;
/// Non-panicking console output.
///
/// Exposed because `println!`/`eprintln!` panic when the write fails, and a
/// robot's stdout fails routinely: the supervisor that launched it exits, or an
/// operator pipes `horus run` into `head`. Anything printing from a node, a
/// background thread, or a safety path should use these instead.
pub mod terminal;
#[cfg_attr(not(feature = "internal-docs"), doc(hidden))]
pub mod types;
pub(crate) mod utils;

// Test utilities — available under `test-utils` feature or in test builds
#[cfg(any(test, feature = "test-utils"))]
#[doc(hidden)]
pub mod testing;

// Crate-internal re-exports (used by `crate::HorusError` etc. within this crate,
// and by horus_py / macro-generated code cross-crate).
#[doc(hidden)]
pub use actions::{
    Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
    ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
    GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
};
#[doc(hidden)]
pub use communication::{
    set_topic_verbose, PodMessage, SendBlockingError, Topic, TopicKind, TOPIC_VERBOSE_OFFSET,
};
#[doc(hidden)]
pub use core::{
    HealthStatus, LogSummary, Node, NodeMetrics, NodePresence, NodeState, Rate, RtStats, Stopwatch,
    TopicMetadata,
};
#[doc(hidden)]
pub use error::{
    retry_transient, CommunicationError, ConfigError, HorusContext, HorusError, HorusResult,
    MemoryError, NodeError, NotFoundError, ParseError, ResourceError, Result, RetryConfig,
    SerializationError, Severity, TimeoutError, TransformError, ValidationError,
};
#[doc(hidden)]
pub use params::RuntimeParams;
pub use scheduling::control::ControlCommand;
#[doc(hidden)]
pub use scheduling::Scheduler;
#[doc(hidden)]
pub use services::{
    AsyncServiceClient, Service, ServiceClient, ServiceError, ServiceRequest, ServiceResponse,
    ServiceResult, ServiceServer, ServiceServerBuilder,
};

// Re-export dependencies used by macro-generated code and horus_py
#[doc(hidden)]
pub use bytemuck;
#[doc(hidden)]
pub use paste;
#[doc(hidden)]
pub use serde_json;
#[doc(hidden)]
pub use serde_yaml;
