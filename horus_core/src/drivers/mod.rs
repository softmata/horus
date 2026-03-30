//! Hardware configuration and node loading from `horus.toml`.
//!
//! Provides [`NodeParams`] for typed config access, a node registry
//! ([`register!`](register_driver!) macro), and [`load()`] / [`load_from()`]
//! to create nodes from the `[hardware]` config section.
//!
//! Users access this via `horus::hardware`.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::hardware;
//!
//! // Load from horus.toml [hardware]
//! let nodes = hardware::load()?;
//! for (name, node) in nodes {
//!     sched.add(node).build()?;
//! }
//! ```

pub mod exec_driver;
pub mod params;
pub mod registry;

pub use exec_driver::ExecDriver;
pub use params::{FromToml, NodeParams};
pub use registry::{register, NodeFactory};

#[cfg(test)]
mod tests;

use crate::core::Node;
use crate::error::{ConfigError, HorusResult};
use std::path::Path;

/// Stub node for simulated hardware entries.
///
/// Created when `sim = true` and `HORUS_SIM_MODE` is set.
/// Does nothing — the simulator plugin publishes to the same topics.
struct SimStubNode {
    node_name: String,
}

impl Node for SimStubNode {
    fn name(&self) -> &str {
        &self.node_name
    }
    fn tick(&mut self) {}
}

/// Load hardware nodes from `horus.toml` `[hardware]` section.
///
/// Searches for `horus.toml` in the current directory and parents (up to 10 levels).
/// Returns a list of `(name, node)` pairs ready to add to a scheduler.
///
/// Each entry in `[hardware]` must have a `use` field naming a registered node type.
/// All other keys (except `sim` and `args`) become [`NodeParams`] passed to the factory.
///
/// When `HORUS_SIM_MODE=1` is set (via `horus run --sim`), entries with `sim = true`
/// are replaced with stub nodes.
///
/// # Example
///
/// ```rust,ignore
/// let nodes = horus::hardware::load()?;
/// for (name, node) in nodes {
///     sched.add(node).build()?;
/// }
/// ```
pub fn load() -> HorusResult<Vec<(String, Box<dyn Node>)>> {
    let path = find_manifest()?;
    load_from(&path)
}

/// Load hardware nodes from a specific config file.
///
/// Useful for testing with alternate configs or multi-robot setups.
pub fn load_from<P: AsRef<Path>>(path: P) -> HorusResult<Vec<(String, Box<dyn Node>)>> {
    let path = path.as_ref();
    let content = std::fs::read_to_string(path)
        .map_err(|e| ConfigError::Other(format!("failed to read {}: {}", path.display(), e)))?;

    let table: toml::Value = toml::from_str(&content)
        .map_err(|e| ConfigError::Other(format!("failed to parse {}: {}", path.display(), e)))?;

    // Support both [hardware] (new) and [drivers] (legacy) section names
    let hw_table = table
        .get("hardware")
        .or_else(|| table.get("drivers"))
        .and_then(|v| v.as_table())
        .cloned()
        .unwrap_or_default();

    let sim_mode = std::env::var("HORUS_SIM_MODE").is_ok();

    let selective_targets: Option<Vec<String>> = if sim_mode {
        std::env::var("HORUS_SIM_TARGETS")
            .ok()
            .map(|s| s.split(',').map(String::from).collect())
    } else {
        None
    };

    // Reserved keys that are NOT passed as NodeParams
    const RESERVED: &[&str] = &[
        "use",
        "sim",
        "args",
        "terra",
        "package",
        "node",
        "crate",
        "source",
        "pip",
        "exec",
        "simulated",
    ];

    let mut nodes: Vec<(String, Box<dyn Node>)> = Vec::new();

    for (name, value) in &hw_table {
        let config = match value.as_table() {
            Some(t) => t,
            None => {
                // Legacy simple value — skip
                log::warn!("hardware.{name}: expected table, skipping");
                continue;
            }
        };

        // Check sim override
        let is_sim_target = config.get("sim").and_then(|v| v.as_bool()).unwrap_or(false);

        if sim_mode && is_sim_target {
            let should_sim = match &selective_targets {
                Some(targets) => targets.iter().any(|t| t == name),
                None => true, // no filter = all sim targets
            };
            if should_sim {
                log::info!("hardware.{name}: simulation mode — using stub");
                nodes.push((
                    name.clone(),
                    Box::new(SimStubNode {
                        node_name: format!("{name}_sim_stub"),
                    }),
                ));
                continue;
            }
        }

        // Determine the node type name from 'use' field (new) or legacy source keys
        let use_name = config
            .get("use")
            .and_then(|v| v.as_str())
            .map(String::from)
            // Legacy fallback: check old source keys
            .or_else(|| {
                config
                    .get("terra")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("node")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("package")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("exec")
                    .and_then(|v| v.as_str())
                    .map(|s| format!("exec:{s}"))
            })
            .or_else(|| config.get("pip").and_then(|v| v.as_str()).map(String::from))
            .or_else(|| {
                config
                    .get("crate")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            });

        let use_name = match use_name {
            Some(n) => n,
            None => {
                log::warn!(
                    "hardware.{name}: missing 'use' field — skipping. \
                     Set use = \"node_type\" to specify which node to create."
                );
                continue;
            }
        };

        // Collect non-reserved keys as NodeParams
        let param_map: std::collections::HashMap<String, toml::Value> = config
            .iter()
            .filter(|(k, _)| !RESERVED.contains(&k.as_str()))
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        let params = NodeParams::new(param_map);

        // Dispatch by prefix
        let node: Box<dyn Node> = if let Some(exec_path) = use_name.strip_prefix("exec:") {
            // External binary subprocess
            let args: Vec<String> = config
                .get("args")
                .and_then(|v| v.as_array())
                .map(|arr| {
                    arr.iter()
                        .filter_map(|v| v.as_str().map(String::from))
                        .collect()
                })
                .unwrap_or_default();

            Box::new(ExecDriver::from_config(exec_path, args, &params)?)
        } else {
            // Look up in node registry
            match registry::lookup(&use_name) {
                Some(factory) => factory(&params)?,
                None => {
                    // Check if it's a known name that hasn't been registered
                    let registered = registry::list_registered();
                    let suggestion = if registered.is_empty() {
                        "No node types are registered. Call register!() or terra_horus::register_all() first.".to_string()
                    } else {
                        format!("Registered types: {}", registered.join(", "))
                    };
                    return Err(ConfigError::Other(format!(
                        "hardware.{name}: unknown node type '{use_name}'. {suggestion}"
                    ))
                    .into());
                }
            }
        };

        nodes.push((name.clone(), node));
    }

    if !nodes.is_empty() {
        log::info!("Loaded {} hardware node(s) from config", nodes.len());
    }

    Ok(nodes)
}

/// Parse the `[hardware]`/`[drivers]` config and return `(name, use_name, params)` tuples
/// without creating nodes. Used by Python bindings which handle node instantiation themselves.
pub fn load_config_entries<P: AsRef<Path>>(
    path: P,
) -> HorusResult<Vec<(String, String, NodeParams)>> {
    let path = path.as_ref();
    let content = std::fs::read_to_string(path)
        .map_err(|e| ConfigError::Other(format!("failed to read {}: {}", path.display(), e)))?;

    let table: toml::Value = toml::from_str(&content)
        .map_err(|e| ConfigError::Other(format!("failed to parse {}: {}", path.display(), e)))?;

    let hw_table = table
        .get("hardware")
        .or_else(|| table.get("drivers"))
        .and_then(|v| v.as_table())
        .cloned()
        .unwrap_or_default();

    const RESERVED: &[&str] = &[
        "use",
        "sim",
        "args",
        "terra",
        "package",
        "node",
        "crate",
        "source",
        "pip",
        "exec",
        "simulated",
    ];

    let mut entries = Vec::new();

    for (name, value) in &hw_table {
        let config = match value.as_table() {
            Some(t) => t,
            None => continue,
        };

        // Match the same fallback chain as load() — use, terra, node, package, exec, pip, crate
        let use_name = config
            .get("use")
            .and_then(|v| v.as_str())
            .map(String::from)
            .or_else(|| {
                config
                    .get("terra")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("node")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("package")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            })
            .or_else(|| {
                config
                    .get("exec")
                    .and_then(|v| v.as_str())
                    .map(|s| format!("exec:{s}"))
            })
            .or_else(|| config.get("pip").and_then(|v| v.as_str()).map(String::from))
            .or_else(|| {
                config
                    .get("crate")
                    .and_then(|v| v.as_str())
                    .map(String::from)
            });

        let use_name = match use_name {
            Some(n) => n,
            None => continue, // Skip entries with no use/legacy key (same as load())
        };

        let param_map: std::collections::HashMap<String, toml::Value> = config
            .iter()
            .filter(|(k, _)| !RESERVED.contains(&k.as_str()))
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        entries.push((name.clone(), use_name, NodeParams::new(param_map)));
    }

    Ok(entries)
}

/// Search upward from current directory for `horus.toml`.
pub fn find_manifest() -> HorusResult<std::path::PathBuf> {
    let mut current = std::env::current_dir()
        .map_err(|e| ConfigError::Other(format!("failed to get current directory: {}", e)))?;

    for _ in 0..10 {
        let candidate = current.join("horus.toml");
        if candidate.exists() {
            return Ok(candidate);
        }
        if let Some(parent) = current.parent() {
            current = parent.to_path_buf();
        } else {
            break;
        }
    }

    Err(ConfigError::Other(
        "horus.toml not found in current directory or parents. \
         Run this command from a horus project directory, or use \
         hardware::load_from() with an explicit path."
            .to_string(),
    )
    .into())
}
