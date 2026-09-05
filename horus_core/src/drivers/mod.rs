//! Hardware configuration and node loading from `horus.toml`.
//!
//! Provides [`NodeParams`] for typed config access, a node registry
//! ([`register_driver!`](crate::register_driver) macro), and [`load()`] / [`load_from()`]
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

/// Whether a `[hardware.<name>]` table asks to be substituted by a stub under
/// `horus run --sim`.
///
/// `simulated` is an ALIAS of `sim`, not a second key. `horus add <name>
/// --driver --source sim` — and `--source simulated`, and `--source sim3d` —
/// all write `simulated = true` into horus.toml, so HORUS generated a key its
/// own loader then ignored: the entry constructed the REAL driver under
/// `horus run --sim`, which is the exact failure `--sim` exists to prevent.
/// `horus check` said nothing either, because `simulated` is a known field
/// rather than an unknown one, and the loader's `RESERVED` list kept it away
/// from the driver as well.
///
/// A free function so the truth table can be tested without a config file or
/// the process-global environment — the same reason `sim_mode_enabled` is one.
fn asks_for_simulation(config: &toml::value::Table) -> bool {
    config
        .get("sim")
        .or_else(|| config.get("simulated"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false)
}

/// Whether a `HORUS_SIM_MODE` value asks for simulation.
///
/// Presence alone used to mean "on", so the two most natural ways to say *off*
/// both turned simulation on. A deploy script setting `HORUS_SIM_MODE=0` to
/// force real hardware silently got inert stubs: actuators never commanded,
/// sensors never read, nothing logged. Parse the value, the way
/// `HORUS_NET_ENABLED` already does.
///
/// A free function taking the value so the truth table can be tested without
/// racing on the process-global environment. It used to be a *copy* of this
/// predicate living inside `#[cfg(test)] mod sim_mode_tests`, which is a thing
/// that can silently stop matching what ships.
fn sim_mode_enabled(value: Option<&str>) -> bool {
    value
        .map(|v| !(v.is_empty() || v == "0" || v.eq_ignore_ascii_case("false")))
        .unwrap_or(false)
}

/// Whether this process is running under `horus run --sim`.
fn sim_mode_active() -> bool {
    sim_mode_enabled(std::env::var("HORUS_SIM_MODE").ok().as_deref())
}

/// The `HORUS_SIM_TARGETS` filter, or `None` for "every entry that asks".
fn sim_targets() -> Option<Vec<String>> {
    std::env::var("HORUS_SIM_TARGETS")
        .ok()
        .map(|s| s.split(',').map(String::from).collect())
}

/// Whether this named entry gets a stub in this process.
///
/// The one predicate behind both loaders. `load_from` substitutes a
/// `SimStubNode`; `load_config_entries` reports it to its caller. They used to
/// disagree: `load_config_entries` had no simulation handling at all, so the
/// Python path built the REAL driver under `horus run --sim` -- the exact
/// failure `--sim` exists to prevent, and the one `asks_for_simulation`
/// already documents.
fn entry_is_simulated(name: &str, config: &toml::value::Table) -> bool {
    if !asks_for_simulation(config) || !sim_mode_active() {
        return false;
    }
    match sim_targets() {
        Some(targets) => targets.iter().any(|t| t == name),
        None => true, // no filter = all sim targets
    }
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

        if entry_is_simulated(name, config) {
            log::info!("hardware.{name}: simulation mode — using stub");
            nodes.push((
                name.clone(),
                Box::new(SimStubNode {
                    node_name: format!("{name}_sim_stub"),
                }),
            ));
            continue;
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

            Box::new(ExecDriver::from_config(name, exec_path, args, &params)?)
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
/// The robot's name from `[robot].name` in the manifest.
///
/// The manifest documents this field as "used in topic naming", and the
/// convention it supports is `"{robot_name}.{sensor}.{data_type}"` — but nothing
/// exposed it, so a Python or Rust node had no way to read the name it is
/// supposed to build its topic names from. `test_robot_name_from_config` called
/// a `robot_name()` that did not exist and failed with
/// `'list' object has no attribute 'robot_name'`.
///
/// Returns `None` when there is no `[robot]` section or no name in it.
pub fn robot_name_from<P: AsRef<Path>>(path: P) -> HorusResult<Option<String>> {
    let path = path.as_ref();
    let content = std::fs::read_to_string(path)
        .map_err(|e| ConfigError::Other(format!("failed to read {}: {}", path.display(), e)))?;
    let table: toml::Value = toml::from_str(&content)
        .map_err(|e| ConfigError::Other(format!("failed to parse {}: {}", path.display(), e)))?;
    Ok(table
        .get("robot")
        .and_then(|v| v.get("name"))
        .and_then(|v| v.as_str())
        .map(str::to_string))
}

/// The robot's name from the manifest found by [`find_manifest`].
pub fn robot_name() -> HorusResult<Option<String>> {
    robot_name_from(find_manifest()?)
}

/// Parse `[hardware]` into `(name, use_name, params, simulated)` tuples.
///
/// The non-instantiating half of [`load_from`]: it resolves the same key
/// fallback chain and the same simulation predicate, but hands the decision
/// back instead of constructing a node. The Python binding uses it, which is
/// why `simulated` is reported rather than acted on -- a Python driver class
/// is not a `Box<dyn Node>` this crate can substitute a stub for.
///
/// `simulated` is true when the entry asks for it (`sim` or its alias
/// `simulated`) AND this process runs under `horus run --sim`. It used to be
/// absent entirely, so `hardware.load()` in Python built the REAL driver under
/// `--sim` while the same manifest under Rust got a stub.
pub fn load_config_entries<P: AsRef<Path>>(
    path: P,
) -> HorusResult<Vec<(String, String, NodeParams, bool)>> {
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

        entries.push((
            name.clone(),
            use_name,
            NodeParams::new(param_map),
            entry_is_simulated(name, config),
        ));
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

#[cfg(test)]
mod sim_target_tests {
    use super::asks_for_simulation;

    fn table(toml_src: &str) -> toml::value::Table {
        toml::from_str(toml_src).expect("test table")
    }

    /// `simulated` must substitute the stub, because HORUS writes that key.
    ///
    /// `horus add <name> --driver --source sim` (and `--source simulated`, and
    /// `--source sim3d`) all emit `simulated = true`. The loader read only
    /// `sim`, so an entry HORUS generated itself constructed the REAL driver
    /// under `horus run --sim` — the exact failure `--sim` exists to prevent —
    /// and `horus check` stayed quiet, because `simulated` is a known field
    /// rather than an unknown one.
    #[test]
    fn simulated_is_an_alias_of_sim() {
        assert!(
            asks_for_simulation(&table("use = \"rplidar\"\nsimulated = true\n")),
            "`simulated = true` is what `horus add --source sim` writes; the \
             loader must honour it"
        );
        assert!(asks_for_simulation(&table(
            "use = \"rplidar\"\nsim = true\n"
        )));
    }

    /// Neither key, or either set false, means the real driver.
    #[test]
    fn the_real_driver_is_the_default() {
        assert!(!asks_for_simulation(&table("use = \"rplidar\"\n")));
        assert!(!asks_for_simulation(&table(
            "use = \"rplidar\"\nsim = false\n"
        )));
        assert!(!asks_for_simulation(&table(
            "use = \"rplidar\"\nsimulated = false\n"
        )));
    }

    /// An explicit `sim` wins over the alias — it is the documented key.
    #[test]
    fn sim_takes_precedence_over_the_alias() {
        assert!(!asks_for_simulation(&table(
            "sim = false\nsimulated = true\n"
        )));
    }
}

#[cfg(test)]
mod sim_mode_tests {
    // The predicate under test is the one that ships. It used to be a private
    // copy declared right here, which could stop matching `load_from` without
    // a single test going red.
    use super::sim_mode_enabled;

    /// Presence alone used to mean "on", so the two most natural ways to say
    /// *off* both turned simulation on. A deploy script setting
    /// `HORUS_SIM_MODE=0` to force real hardware silently got inert stubs:
    /// actuators never commanded, sensors never read, nothing logged.
    #[test]
    fn falsy_values_disable_simulation() {
        assert!(!sim_mode_enabled(Some("0")));
        assert!(!sim_mode_enabled(Some("false")));
        assert!(!sim_mode_enabled(Some("False")));
        assert!(!sim_mode_enabled(Some("FALSE")));
        assert!(!sim_mode_enabled(Some("")));
    }

    #[test]
    fn truthy_values_enable_simulation() {
        assert!(sim_mode_enabled(Some("1")));
        assert!(sim_mode_enabled(Some("true")));
        assert!(sim_mode_enabled(Some("TRUE")));
        assert!(sim_mode_enabled(Some("yes")));
    }

    #[test]
    fn unset_means_disabled() {
        assert!(!sim_mode_enabled(None));
    }

    /// Matches the `HORUS_NET_ENABLED` predicate in the scheduler, so the two
    /// env vars cannot drift into disagreeing about what "0" means.
    #[test]
    fn agrees_with_horus_net_enabled_convention() {
        for falsy in ["0", "false", "False"] {
            let net_disabled = falsy == "0" || falsy.eq_ignore_ascii_case("false");
            assert_eq!(
                !sim_mode_enabled(Some(falsy)),
                net_disabled,
                "HORUS_SIM_MODE and HORUS_NET_ENABLED must agree on {falsy:?}"
            );
        }
    }
}
