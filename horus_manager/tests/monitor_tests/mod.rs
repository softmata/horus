//! Comprehensive test utilities for the HORUS monitor handlers.
//!
//! Provides reusable builders, helpers, and SHM fixture management for testing
//! the Axum-based web monitor without starting a real TCP listener.
//!
//! # Modules
//!
//! - [`builders`] — `test_app_state()` and `test_router()` for constructing
//!   a testable Axum application.
//! - [`helpers`] — Factory functions for `NodeStatus`, `SharedMemoryInfo`,
//!   `BlackBoxEvent`, and response assertion helpers.
//! - [`shm_fixtures`] — Functions to create and clean up real shared memory
//!   presence and topic files for integration-level testing.

#[allow(dead_code)]
pub mod builders;
#[allow(dead_code)]
pub mod helpers;
#[allow(dead_code)]
pub mod shm_fixtures;
