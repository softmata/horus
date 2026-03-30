//! HORUS Manager Library
//!
//! This library provides the core functionality for the HORUS manager.

pub mod cargo_gen;
pub mod cargo_utils;
pub mod cli_output;
pub mod cmake_gen;
pub mod commands;
pub mod config;
pub mod dependency_resolver;
pub mod discovery;
pub mod dispatch;
pub mod error_wrapper;
pub mod fingerprint;
pub mod fs_utils;
pub mod graph;
pub mod lockfile;
pub mod manifest;
pub mod native_sync;
pub mod node_detector;
pub mod paths;
pub mod plugins;
pub mod progress;
pub mod pyproject_gen;
pub mod registry;
pub mod run_with_prefix;
pub mod source_resolver;
pub mod system_deps;
pub mod toolchain;
pub mod urdf;
pub mod version;
pub mod workspace;

/// Global mutex to serialize tests that change the current working directory.
#[cfg(test)]
pub static CWD_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
