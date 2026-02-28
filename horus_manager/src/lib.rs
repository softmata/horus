//! HORUS Manager Library
//!
//! This library provides the core functionality for the HORUS manager.

pub mod cargo_utils;
pub mod cli_output;
pub mod commands;
pub mod config;
pub mod dependency_resolver;
pub mod discovery;
pub mod graph;
pub mod monitor;
pub mod monitor_tui;
pub mod node_detector;
pub mod paths;
pub mod plugins;
pub mod progress;
pub mod registry;
pub mod security;
pub mod system_deps;
pub mod version;
pub mod workspace;
pub mod yaml_utils;
