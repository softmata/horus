//! Driver configuration and hardware connection support.
//!
//! Provides [`HardwareSet`] for loading pre-configured hardware connections
//! from `horus.toml` `[drivers]` config, and [`DriverParams`] for typed
//! access to driver configuration values.
//!
//! Users access this via `horus::drivers`.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::drivers;
//!
//! // Load from horus.toml [drivers]
//! let mut hw = drivers::load()?;
//!
//! // Typed Terra handle
//! let bus = hw.dynamixel("arm")?;
//!
//! // Or load from alternate config (testing)
//! let mut hw = drivers::load_from("tests/test_drivers.toml")?;
//! ```

pub mod exec_driver;
pub mod hardware_set;
pub mod params;
pub mod registry;
pub mod terra_map;

pub use exec_driver::ExecDriver;
pub use hardware_set::{DriverHandle, DriverType, HardwareSet, TopicMapping};
pub use params::{DriverParams, FromToml};
pub use registry::{register, DriverFactory};
pub use terra_map::TerraCrateInfo;

#[cfg(test)]
mod tests;

use crate::error::{ConfigError, HorusResult};
use std::path::Path;

/// Load hardware connections from `horus.toml` `[drivers]` section.
///
/// Searches for `horus.toml` in the current directory and parents (up to 10 levels).
/// Returns a [`HardwareSet`] with pre-configured driver entries.
///
/// # Example
///
/// ```rust,ignore
/// let mut hw = horus::drivers::load()?;
/// let bus = hw.dynamixel("arm")?;
/// ```
pub fn load() -> HorusResult<HardwareSet> {
    let path = find_manifest()?;
    load_from(&path)
}

/// Load hardware connections from a specific config file.
///
/// Useful for testing with alternate configs or multi-robot setups.
///
/// # Example
///
/// ```rust,ignore
/// let mut hw = horus::drivers::load_from("tests/test_drivers.toml")?;
/// ```
pub fn load_from<P: AsRef<Path>>(path: P) -> HorusResult<HardwareSet> {
    let path = path.as_ref();
    let content = std::fs::read_to_string(path)
        .map_err(|e| ConfigError::Other(format!("failed to read {}: {}", path.display(), e)))?;

    let table: toml::Value = toml::from_str(&content)
        .map_err(|e| ConfigError::Other(format!("failed to parse {}: {}", path.display(), e)))?;

    let drivers_table = table
        .get("drivers")
        .and_then(|v| v.as_table())
        .cloned()
        .unwrap_or_default();

    let mut hw = HardwareSet::from_toml_table(&drivers_table)?;

    // Read robot name from [robot] section if present
    if let Some(robot_name) = table
        .get("robot")
        .and_then(|r| r.get("name"))
        .and_then(|n| n.as_str())
    {
        hw.set_robot_name(robot_name);
    }

    // Auto-apply sim overrides when HORUS_SIM_MODE is set (from `horus run --sim`)
    if std::env::var("HORUS_SIM_MODE").is_ok() {
        if let Some(sim_table) = table.get("sim-drivers").and_then(|v| v.as_table()) {
            let selective: Option<Vec<String>> = std::env::var("HORUS_SIM_TARGETS")
                .ok()
                .map(|s| s.split(',').map(String::from).collect());

            let table_to_apply = if let Some(ref targets) = selective {
                // Selective mode: only override named drivers
                sim_table
                    .iter()
                    .filter(|(k, _)| targets.contains(k))
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect()
            } else {
                sim_table.clone()
            };

            let count = hw.apply_sim_overrides_from_toml(&table_to_apply);
            if count > 0 {
                log::info!(
                    "Simulation mode: {} driver(s) overridden with [sim-drivers]",
                    count
                );
            }
        }
    }

    Ok(hw)
}

/// Search upward from current directory for `horus.toml`.
fn find_manifest() -> HorusResult<std::path::PathBuf> {
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
         drivers::load_from() with an explicit path."
            .to_string(),
    )
    .into())
}
