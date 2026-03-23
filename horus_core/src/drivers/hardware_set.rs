//! `HardwareSet` — pre-configured hardware connections from `[drivers]` config.
//!
//! Constructed by [`super::load()`] or [`super::load_from()`]. Provides typed
//! accessors for Terra handles and factory dispatch for registry/local drivers.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::drivers;
//!
//! let mut hw = drivers::load()?;
//!
//! // Terra typed handle
//! let bus = hw.dynamixel("arm")?;
//!
//! // Registry package node
//! let node = hw.package("force_sensor")?;
//!
//! // Local registered driver
//! let node = hw.local("conveyor")?;
//! ```

use std::collections::HashMap;

use crate::core::Node;
use crate::error::{ConfigError, HorusResult};

use super::params::DriverParams;
use super::terra_map;

// ── Driver source type ──────────────────────────────────────────────────────

/// Identifies where a driver comes from.
#[derive(Debug, Clone, PartialEq)]
pub enum DriverType {
    /// Terra HAL driver — `terra = "dynamixel"` in config.
    Terra(String),
    /// Registry package — `package = "horus-driver-ati-netft"` in config.
    Package(String),
    /// Local code — `node = "ConveyorDriver"` in config.
    Local(String),
    /// Legacy simple value — `camera = "opencv"` or `camera = true`.
    Legacy,
    /// Rust crate from crates.io — `crate = "rplidar-driver", source = "crates-io"`.
    CratesIo(String),
    /// Python package from PyPI — `pip = "adafruit-bno055"`.
    PyPI(String),
    /// External binary subprocess — `exec = "./realsense_bridge"`.
    Exec(std::path::PathBuf),
    /// Simulated by a simulator plugin (sim3d, mujoco, isaac, etc.) — `simulated = true`.
    Simulated,
}

// ── Driver handle ───────────────────────────────────────────────────────────

/// Opaque handle to a configured driver.
///
/// In the initial implementation, this wraps `DriverParams` only. When Terra
/// crates are wired as dependencies, this will hold actual connection handles
/// (e.g., `DynamixelBus`, `RpLidar`, etc.).
#[derive(Debug)]
pub struct DriverHandle {
    /// The driver's config params.
    params: DriverParams,
    /// Terra driver name (if terra source).
    terra_name: Option<String>,
}

impl DriverHandle {
    /// Access the config params.
    pub fn params(&self) -> &DriverParams {
        &self.params
    }

    /// Get the Terra driver shortname (e.g., `"dynamixel"`).
    pub fn terra_name(&self) -> Option<&str> {
        self.terra_name.as_deref()
    }
}

// ── Topic mapping ───────────────────────────────────────────────────────────

/// Topic mapping for auto-bridging drivers to HORUS topics.
///
/// Configured via `topic`, `topic_state`, `topic_command` fields in
/// `horus.toml` `[drivers.NAME]` entries. Used by bridge layers
/// (e.g., `terra-horus`) to auto-create sensor/actuator topic forwarding.
#[derive(Debug, Clone, Default)]
pub struct TopicMapping {
    /// Sensor data topic (e.g., `"sensors/imu"`).
    pub topic: Option<String>,
    /// Joint state output topic (e.g., `"arm/joint_states"`).
    pub topic_state: Option<String>,
    /// Joint command input topic (e.g., `"arm/joint_commands"`).
    pub topic_command: Option<String>,
}

// ── Driver entry ────────────────────────────────────────────────────────────

/// Internal entry for a configured driver.
#[derive(Debug)]
struct DriverEntry {
    driver_type: DriverType,
    params: DriverParams,
    topics: TopicMapping,
}

// ── Terra stub node ─────────────────────────────────────────────────────────

/// Stub node wrapping a Terra driver handle for the generic `.node()` accessor.
///
/// Holds config params but has an empty `tick()`. Use typed accessors
/// (`.dynamixel()`, `.rplidar()`, etc.) for full hardware integration.
struct TerraStubNode {
    driver_name: String,
    #[allow(dead_code)]
    terra_name: String,
    #[allow(dead_code)]
    params: DriverParams,
}

impl crate::core::Node for TerraStubNode {
    fn name(&self) -> &str {
        &self.driver_name
    }

    fn tick(&mut self) {}
}

// ── Sim3d stub node ────────────────────────────────────────────────────────

/// Stub node for sensors simulated by horus-sim3d.
///
/// All actual work (physics, sensor data, topic publishing) happens in the
/// sim3d process. This stub tells the scheduler the sensor exists and
/// reports healthy status.
struct SimulatedStubNode {
    driver_name: String,
    #[allow(dead_code)]
    params: DriverParams,
}

impl crate::core::Node for SimulatedStubNode {
    fn name(&self) -> &str {
        &self.driver_name
    }

    fn tick(&mut self) {}
}

// ── HardwareSet ─────────────────────────────────────────────────────────────

/// Pre-configured hardware connections loaded from `horus.toml` `[drivers]`.
///
/// Does NOT create nodes or register anything in the scheduler.
/// Users extract handles and wrap them in their own `Node` implementations.
#[derive(Debug)]
pub struct HardwareSet {
    entries: HashMap<String, DriverEntry>,
    /// Robot name for topic name resolution. When set, typed accessors
    /// like `.scan()` resolve topic names via the shared convention
    /// `{robot_name}.{sensor_name}.{data_type}`.
    robot_name: Option<String>,
}

impl HardwareSet {
    /// Build from parsed `[drivers]` TOML table.
    ///
    /// Called by `drivers::load()` and `drivers::load_from()`.
    pub fn from_toml_table(table: &toml::value::Table) -> HorusResult<Self> {
        let mut entries = HashMap::new();

        for (name, value) in table {
            let entry = match value {
                toml::Value::Table(cfg) => {
                    // Config table: determine type from terra/package/node key
                    let terra = cfg.get("terra").and_then(|v| v.as_str()).map(String::from);
                    let package = cfg
                        .get("package")
                        .and_then(|v| v.as_str())
                        .map(String::from);
                    let node = cfg.get("node").and_then(|v| v.as_str()).map(String::from);

                    // Extract topic mapping fields
                    let topic = cfg.get("topic").and_then(|v| v.as_str()).map(String::from);
                    let topic_state = cfg
                        .get("topic_state")
                        .and_then(|v| v.as_str())
                        .map(String::from);
                    let topic_command = cfg
                        .get("topic_command")
                        .and_then(|v| v.as_str())
                        .map(String::from);

                    // Extract new driver source keys
                    let crate_name = cfg
                        .get("crate")
                        .and_then(|v| v.as_str())
                        .map(String::from);
                    let pip = cfg.get("pip").and_then(|v| v.as_str()).map(String::from);
                    let exec = cfg
                        .get("exec")
                        .and_then(|v| v.as_str())
                        .map(|s| std::path::PathBuf::from(s));
                    let simulated = cfg
                        .get("simulated")
                        .or_else(|| cfg.get("sim3d")) // backwards compat
                        .and_then(|v| v.as_bool())
                        .unwrap_or(false);

                    // Collect remaining keys as params (skip config/bridge keys)
                    const RESERVED: &[&str] = &[
                        "terra",
                        "package",
                        "node",
                        "crate",
                        "source",
                        "pip",
                        "exec",
                        "simulated",
                        "sim3d", // backwards compat
                        "args",
                        "topic",
                        "topic_state",
                        "topic_command",
                    ];
                    let mut params_map = HashMap::new();
                    for (k, v) in cfg {
                        if !RESERVED.contains(&k.as_str()) {
                            params_map.insert(k.clone(), v.clone());
                        }
                    }

                    // Store args in params for downstream use
                    if let Some(args) = cfg.get("args") {
                        params_map.insert("args".to_string(), args.clone());
                    }

                    let driver_type = if let Some(t) = terra {
                        DriverType::Terra(t)
                    } else if let Some(p) = package {
                        DriverType::Package(p)
                    } else if let Some(n) = node {
                        DriverType::Local(n)
                    } else if let Some(c) = crate_name {
                        DriverType::CratesIo(c)
                    } else if let Some(p) = pip {
                        DriverType::PyPI(p)
                    } else if let Some(e) = exec {
                        DriverType::Exec(e)
                    } else if simulated {
                        DriverType::Simulated
                    } else {
                        return Err(ConfigError::Other(format!(
                            "driver '{}': config table must have 'terra', 'package', 'node', 'crate', 'pip', 'exec', or 'sim3d' key",
                            name
                        ))
                        .into());
                    };

                    DriverEntry {
                        driver_type,
                        params: DriverParams::new(params_map),
                        topics: TopicMapping {
                            topic,
                            topic_state,
                            topic_command,
                        },
                    }
                }
                toml::Value::String(_) | toml::Value::Boolean(_) => {
                    // Legacy: camera = "opencv" or gps = true
                    DriverEntry {
                        driver_type: DriverType::Legacy,
                        params: DriverParams::empty(),
                        topics: TopicMapping::default(),
                    }
                }
                _ => {
                    return Err(ConfigError::Other(format!(
                        "driver '{}': expected table, string, or bool",
                        name
                    ))
                    .into());
                }
            };

            entries.insert(name.clone(), entry);
        }

        Ok(Self { entries, robot_name: None })
    }

    // ── Terra typed accessors ────────────────────────────────────────────

    /// Get a Dynamixel servo bus handle.
    pub fn dynamixel(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "dynamixel")
    }

    /// Get an RPLiDAR scanner handle.
    pub fn rplidar(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "rplidar")
    }

    /// Get an Intel RealSense camera handle.
    pub fn realsense(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "realsense")
    }

    /// Get an I2C device handle.
    pub fn i2c(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "i2c")
    }

    /// Get a serial/UART port handle.
    pub fn serial(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "serial")
    }

    /// Get a CAN bus handle.
    pub fn can(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "can")
    }

    /// Get a GPIO pin handle.
    pub fn gpio(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "gpio")
    }

    /// Get a PWM output handle.
    pub fn pwm(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "pwm")
    }

    /// Get a USB device handle.
    pub fn usb(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "usb")
    }

    /// Get a webcam (V4L2) handle.
    pub fn webcam(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "webcam")
    }

    /// Get a gamepad/input device handle.
    pub fn input(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "input")
    }

    /// Get a Bluetooth LE device handle.
    pub fn bluetooth(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "bluetooth")
    }

    /// Get a network device handle (TCP/UDP — LiDAR, industrial controllers).
    pub fn net(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "net")
    }

    /// Get an EtherCAT bus handle.
    pub fn ethercat(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "ethercat")
    }

    /// Get an SPI device handle.
    pub fn spi(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "spi")
    }

    /// Get an ADC device handle.
    pub fn adc(&mut self, name: &str) -> HorusResult<DriverHandle> {
        self.get_terra_handle(name, "adc")
    }

    /// Get a raw driver handle (escape hatch for any driver type).
    pub fn raw(&mut self, name: &str) -> HorusResult<DriverHandle> {
        let entry = self.entries.get(name).ok_or_else(|| {
            ConfigError::Other(format!(
                "driver '{}' not found in [drivers] config. Available: [{}]",
                name,
                self.list().join(", ")
            ))
        })?;

        Ok(DriverHandle {
            params: entry.params.clone(),
            terra_name: match &entry.driver_type {
                DriverType::Terra(t) => Some(t.clone()),
                _ => None,
            },
        })
    }

    // ── Factory accessors ────────────────────────────────────────────────

    /// Get a driver as `Box<dyn Node>` regardless of source type.
    ///
    /// - **Terra** drivers: wrapped in a stub node holding the config params.
    ///   Use typed accessors (`.dynamixel()`, `.rplidar()`, etc.) for full functionality.
    /// - **Package** drivers: delegates to [`.package()`](Self::package).
    /// - **Local** drivers: delegates to [`.local()`](Self::local).
    pub fn node(&mut self, name: &str) -> HorusResult<Box<dyn Node>> {
        let entry = self.entries.get(name).ok_or_else(|| {
            ConfigError::Other(format!(
                "driver '{}' not found in [drivers] config. Available: [{}]",
                name,
                self.list().join(", ")
            ))
        })?;

        match &entry.driver_type {
            DriverType::Terra(terra_name) => {
                log::info!(
                    "driver '{}': wrapping terra '{}' as generic node. \
                     For full hardware access, use the typed accessor (e.g., hw.dynamixel()).",
                    name,
                    terra_name
                );
                Ok(Box::new(TerraStubNode {
                    driver_name: name.to_string(),
                    terra_name: terra_name.clone(),
                    params: entry.params.clone(),
                }))
            }
            DriverType::Package(_) => self.package(name),
            DriverType::Local(_) => self.local(name),
            DriverType::Legacy => Err(ConfigError::Other(format!(
                "driver '{}': legacy driver (simple string/bool) cannot be instantiated as a node. \
                 Use a config table with terra/package/node key instead.",
                name
            ))
            .into()),
            DriverType::CratesIo(_) | DriverType::PyPI(_) => Err(ConfigError::Other(format!(
                "driver '{}': CratesIo/PyPI drivers require build-time adapter code generation. \
                 Use `horus build` first, then register the adapter via register_driver!.",
                name
            ))
            .into()),
            DriverType::Exec(path) => {
                Ok(Box::new(super::exec_driver::ExecDriver::from_params(
                    name,
                    path.clone(),
                    &entry.params,
                )))
            }
            DriverType::Simulated => {
                Ok(Box::new(SimulatedStubNode {
                    driver_name: name.to_string(),
                    params: entry.params.clone(),
                }))
            }
        }
    }

    /// Instantiate a driver from a registry package.
    ///
    /// Calls the package's `create(&DriverParams)` factory function.
    /// The package must be installed via `horus add <package>`.
    pub fn package(&mut self, name: &str) -> HorusResult<Box<dyn Node>> {
        let entry = self.entries.get(name).ok_or_else(|| {
            ConfigError::Other(format!("driver '{}' not found in [drivers] config", name))
        })?;

        match &entry.driver_type {
            DriverType::Package(pkg) => {
                let factory = super::registry::lookup(pkg).ok_or_else(|| {
                    ConfigError::Other(format!(
                        "driver '{}': registry package '{}' not registered. \
                         Ensure it is installed (`horus add {}`) and linked as a dependency. \
                         The package must use `register_driver!` to export its factory.",
                        name, pkg, pkg
                    ))
                })?;
                factory(&entry.params)
            }
            other => Err(ConfigError::Other(format!(
                "driver '{}': expected package driver (package = ...), got {:?}",
                name,
                std::mem::discriminant(other)
            ))
            .into()),
        }
    }

    /// Instantiate a driver from local code registered via `register_driver!`.
    ///
    /// Looks up the factory in the compile-time registry and calls it with
    /// the config params.
    pub fn local(&mut self, name: &str) -> HorusResult<Box<dyn Node>> {
        let entry = self.entries.get(name).ok_or_else(|| {
            ConfigError::Other(format!("driver '{}' not found in [drivers] config", name))
        })?;

        match &entry.driver_type {
            DriverType::Local(node_name) => {
                let factory = super::registry::lookup(node_name).ok_or_else(|| {
                    ConfigError::Other(format!(
                        "driver '{}': local node '{}' not registered. \
                         Add `register_driver!({}, YourType::from_params);` in your code.",
                        name, node_name, node_name
                    ))
                })?;
                factory(&entry.params)
            }
            other => Err(ConfigError::Other(format!(
                "driver '{}': expected local driver (node = ...), got {:?}",
                name,
                std::mem::discriminant(other)
            ))
            .into()),
        }
    }

    // ── Introspection ────────────────────────────────────────────────────

    /// List all configured driver names.
    pub fn list(&self) -> Vec<&str> {
        let mut names: Vec<&str> = self.entries.keys().map(|s| s.as_str()).collect();
        names.sort();
        names
    }

    /// Check if a driver is configured.
    pub fn has(&self, name: &str) -> bool {
        self.entries.contains_key(name)
    }

    /// Get config params for a driver.
    pub fn params(&self, name: &str) -> Option<&DriverParams> {
        self.entries.get(name).map(|e| &e.params)
    }

    /// Get the driver source type.
    pub fn driver_type(&self, name: &str) -> Option<&DriverType> {
        self.entries.get(name).map(|e| &e.driver_type)
    }

    /// Get topic mappings for a driver (for auto-bridging).
    pub fn topic_mapping(&self, name: &str) -> Option<&TopicMapping> {
        self.entries.get(name).map(|e| &e.topics)
    }

    /// Number of configured drivers.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether there are no configured drivers.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    // ── Topic resolution ────────────────────────────────────────────────

    /// Set the robot name for topic name resolution.
    ///
    /// Called automatically by `drivers::load_from()` when `[robot].name`
    /// exists in horus.toml. Users should set the name in horus.toml
    /// rather than calling this directly.
    pub(crate) fn set_robot_name(&mut self, name: impl Into<String>) {
        self.robot_name = Some(name.into());
    }

    /// Get the configured robot name, or "robot" as default.
    pub fn robot_name(&self) -> &str {
        self.robot_name.as_deref().unwrap_or("robot")
    }

    /// Resolve the topic name string for a sensor (internal).
    ///
    /// Users should use the naming convention directly:
    /// `"{robot_name}.{sensor_name}.{data_type}"` (e.g., `"turtlebot.front_lidar.scan"`)
    ///
    /// This method exists for internal alignment testing between sim and drivers.
    pub(crate) fn resolve_topic_name(&self, name: &str, data_type: &str) -> String {
        // Check explicit TopicMapping first (backwards compat)
        if let Some(mapping) = self.topic_mapping(name) {
            if let Some(ref topic) = mapping.topic {
                return topic.clone();
            }
            if let Some(ref topic_state) = mapping.topic_state {
                return topic_state.clone();
            }
        }

        // Shared naming convention
        let robot = self.robot_name();
        let mut segments = Vec::with_capacity(3);
        if !robot.is_empty() {
            segments.push(robot);
        }
        if !name.is_empty() {
            segments.push(name);
        }
        if !data_type.is_empty() {
            segments.push(data_type);
        }
        segments.join(".")
    }

    // ── Sim override ────────────────────────────────────────────────────

    /// Apply simulation overrides from `[sim-drivers]` section.
    ///
    /// For each entry in `sim_overrides`:
    /// - If the name matches an existing driver: replace its type with `Sim3d`
    ///   and merge params (noise, rate overrides)
    /// - If no match: insert as a new `Sim3d` driver entry
    ///
    /// Returns the number of drivers overridden.
    pub fn apply_sim_overrides(&mut self, sim_overrides: &std::collections::BTreeMap<String, DriverType>) -> usize {
        let mut count = 0;
        for (name, _) in sim_overrides {
            if let Some(entry) = self.entries.get_mut(name) {
                entry.driver_type = DriverType::Simulated;
                count += 1;
            } else {
                // New sim-only driver not in [drivers]
                self.entries.insert(name.clone(), DriverEntry {
                    driver_type: DriverType::Simulated,
                    params: DriverParams::empty(),
                    topics: TopicMapping::default(),
                });
                count += 1;
            }
        }
        count
    }

    /// Apply simulation overrides from raw TOML values.
    ///
    /// Simpler API that accepts the sim_drivers map directly from the manifest.
    /// Each entry's driver type is replaced with `Sim3d`.
    pub fn apply_sim_overrides_from_toml(&mut self, sim_table: &toml::value::Table) -> usize {
        let mut count = 0;
        for (name, _value) in sim_table {
            if let Some(entry) = self.entries.get_mut(name) {
                entry.driver_type = DriverType::Simulated;
                count += 1;
            } else {
                self.entries.insert(name.clone(), DriverEntry {
                    driver_type: DriverType::Simulated,
                    params: DriverParams::empty(),
                    topics: TopicMapping::default(),
                });
                count += 1;
            }
        }
        count
    }

    // ── Internal helpers ─────────────────────────────────────────────────

    fn get_terra_handle(&self, name: &str, _expected_accessor: &str) -> HorusResult<DriverHandle> {
        let entry = self.entries.get(name).ok_or_else(|| {
            ConfigError::Other(format!(
                "driver '{}' not found in [drivers] config. Available: [{}]",
                name,
                self.list().join(", ")
            ))
        })?;

        match &entry.driver_type {
            DriverType::Terra(terra_name) => {
                // Validate the terra name is known
                if terra_map::resolve(terra_name).is_none() {
                    log::warn!(
                        "driver '{}': unknown terra driver '{}' — \
                         connection will use raw params only",
                        name,
                        terra_name
                    );
                }

                Ok(DriverHandle {
                    params: entry.params.clone(),
                    terra_name: Some(terra_name.clone()),
                })
            }
            other => Err(ConfigError::Other(format!(
                "driver '{}': expected terra driver, got {:?}. \
                 Use .raw() for non-terra drivers.",
                name,
                std::mem::discriminant(other)
            ))
            .into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_table(toml_str: &str) -> toml::value::Table {
        let value: toml::Value = toml::from_str(toml_str).unwrap();
        value.as_table().unwrap().clone()
    }

    #[test]
    fn from_toml_terra_driver() {
        let table = test_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            baudrate = 1000000
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("arm"));
        assert_eq!(hw.len(), 1);
        assert!(matches!(hw.driver_type("arm"), Some(DriverType::Terra(t)) if t == "dynamixel"));
        let params = hw.params("arm").unwrap();
        assert_eq!(params.get::<String>("port").unwrap(), "/dev/ttyUSB0");
        assert_eq!(params.get::<u32>("baudrate").unwrap(), 1_000_000);
    }

    #[test]
    fn from_toml_package_driver() {
        let table = test_table(
            r#"
            [sensor]
            package = "horus-driver-ati-netft"
            address = "192.168.1.100"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(
            matches!(hw.driver_type("sensor"), Some(DriverType::Package(p)) if p == "horus-driver-ati-netft")
        );
    }

    #[test]
    fn from_toml_local_driver() {
        let table = test_table(
            r#"
            [conveyor]
            node = "ConveyorDriver"
            port = "/dev/ttyACM0"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(
            matches!(hw.driver_type("conveyor"), Some(DriverType::Local(n)) if n == "ConveyorDriver")
        );
    }

    #[test]
    fn from_toml_legacy_string() {
        let table = test_table(r#"camera = "opencv""#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("camera"));
        assert!(matches!(hw.driver_type("camera"), Some(DriverType::Legacy)));
    }

    #[test]
    fn from_toml_legacy_bool() {
        let table = test_table(r#"gps = true"#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("gps"));
        assert!(matches!(hw.driver_type("gps"), Some(DriverType::Legacy)));
    }

    #[test]
    fn from_toml_mixed() {
        let table = test_table(
            r#"
            camera = "opencv"
            gps = true
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            [sensor]
            package = "horus-driver-x"
            [conveyor]
            node = "MyDriver"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 5);
        let names = hw.list();
        assert!(names.contains(&"camera"));
        assert!(names.contains(&"gps"));
        assert!(names.contains(&"arm"));
        assert!(names.contains(&"sensor"));
        assert!(names.contains(&"conveyor"));
    }

    #[test]
    fn list_sorted() {
        let table = test_table(
            r#"
            zebra = true
            alpha = true
            mid = true
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.list(), vec!["alpha", "mid", "zebra"]);
    }

    #[test]
    fn has_and_missing() {
        let table = test_table(r#"arm = true"#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("arm"));
        assert!(!hw.has("leg"));
    }

    #[test]
    fn dynamixel_accessor() {
        let table = test_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let handle = hw.dynamixel("arm").unwrap();
        assert_eq!(handle.terra_name(), Some("dynamixel"));
        assert_eq!(
            handle.params().get::<String>("port").unwrap(),
            "/dev/ttyUSB0"
        );
    }

    #[test]
    fn dynamixel_accessor_wrong_type_errors() {
        let table = test_table(
            r#"
            [sensor]
            package = "horus-driver-x"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.dynamixel("sensor").is_err());
    }

    #[test]
    fn spi_accessor() {
        let table = test_table(
            r#"
            [display]
            terra = "spi"
            bus = 0
            device = 1
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let handle = hw.spi("display").unwrap();
        assert_eq!(handle.terra_name(), Some("spi"));
        assert_eq!(handle.params().get::<u32>("bus").unwrap(), 0);
    }

    #[test]
    fn adc_accessor() {
        let table = test_table(
            r#"
            [voltage]
            terra = "adc"
            channel = 3
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let handle = hw.adc("voltage").unwrap();
        assert_eq!(handle.terra_name(), Some("adc"));
        assert_eq!(handle.params().get::<u32>("channel").unwrap(), 3);
    }

    #[test]
    fn missing_driver_error_lists_available() {
        let table = test_table(r#"arm = true"#);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let err = hw.dynamixel("leg").unwrap_err().to_string();
        assert!(err.contains("leg"), "error should mention missing name");
        assert!(err.contains("arm"), "error should list available drivers");
    }

    #[test]
    fn config_table_without_source_key_errors() {
        let table = test_table(
            r#"
            [broken]
            port = "/dev/ttyUSB0"
        "#,
        );
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("terra"));
        assert!(err.contains("package"));
        assert!(err.contains("node"));
    }

    #[test]
    fn empty_table() {
        let table = toml::value::Table::new();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.is_empty());
        assert_eq!(hw.len(), 0);
        assert!(hw.list().is_empty());
    }

    #[test]
    fn params_not_found() {
        let table = test_table(r#"arm = true"#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.params("nonexistent").is_none());
        assert!(hw.driver_type("nonexistent").is_none());
    }

    // ── Package factory dispatch tests ────────────────────────────────

    struct PkgStub;
    impl crate::core::Node for PkgStub {
        fn name(&self) -> &str {
            "pkg_stub"
        }
        fn tick(&mut self) {}
    }

    #[test]
    fn package_factory_dispatch() {
        super::super::registry::register("test-pkg-driver", |_params| Ok(Box::new(PkgStub)));
        let table = test_table(
            r#"
            [sensor]
            package = "test-pkg-driver"
            address = "192.168.1.100"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.package("sensor").unwrap();
        assert_eq!(node.name(), "pkg_stub");
    }

    #[test]
    fn package_unregistered_errors_clearly() {
        let table = test_table(
            r#"
            [sensor]
            package = "nonexistent-pkg-xyz"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.package("sensor");
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error for unregistered package"),
        };
        assert!(err.contains("nonexistent-pkg-xyz"));
        assert!(err.contains("horus add"));
    }

    #[test]
    fn package_wrong_type_errors() {
        let table = test_table(
            r#"
            [arm]
            terra = "dynamixel"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.package("arm").is_err());
    }

    // ── .node() generic accessor tests ────────────────────────────────

    #[test]
    fn node_generic_terra() {
        let table = test_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("arm").unwrap();
        assert_eq!(node.name(), "arm");
    }

    #[test]
    fn node_generic_local() {
        super::super::registry::register("NodeGenericLocal", |_params| Ok(Box::new(PkgStub)));
        let table = test_table(
            r#"
            [conv]
            node = "NodeGenericLocal"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("conv").unwrap();
        assert_eq!(node.name(), "pkg_stub");
    }

    #[test]
    fn node_generic_package() {
        super::super::registry::register("test-node-pkg", |_params| Ok(Box::new(PkgStub)));
        let table = test_table(
            r#"
            [sensor]
            package = "test-node-pkg"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("sensor").unwrap();
        assert_eq!(node.name(), "pkg_stub");
    }

    #[test]
    fn node_generic_legacy_errors() {
        let table = test_table(r#"camera = "opencv""#);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.node("camera").is_err());
    }

    // ── TopicMapping tests ────────────────────────────────────────────

    #[test]
    fn topic_mapping_sensor() {
        let table = test_table(
            r#"
            [imu]
            terra = "mpu6050"
            bus = "i2c-1"
            address = 104
            topic = "sensors/imu"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("imu").unwrap();
        assert_eq!(mapping.topic.as_deref(), Some("sensors/imu"));
        assert!(mapping.topic_state.is_none());
        assert!(mapping.topic_command.is_none());
        // topic should NOT be in params
        let params = hw.params("imu").unwrap();
        assert!(!params.has("topic"));
        assert!(params.has("bus"));
        assert!(params.has("address"));
    }

    #[test]
    fn topic_mapping_actuator() {
        let table = test_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            baudrate = 1000000
            topic_state = "arm/joint_states"
            topic_command = "arm/joint_commands"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("arm").unwrap();
        assert!(mapping.topic.is_none());
        assert_eq!(mapping.topic_state.as_deref(), Some("arm/joint_states"));
        assert_eq!(mapping.topic_command.as_deref(), Some("arm/joint_commands"));
        // topic fields should NOT be in params
        let params = hw.params("arm").unwrap();
        assert!(!params.has("topic_state"));
        assert!(!params.has("topic_command"));
        assert!(params.has("port"));
        assert!(params.has("baudrate"));
    }

    #[test]
    fn topic_mapping_none_when_no_topic_fields() {
        let table = test_table(
            r#"
            [lidar]
            terra = "rplidar"
            port = "/dev/ttyUSB1"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("lidar").unwrap();
        assert!(mapping.topic.is_none());
        assert!(mapping.topic_state.is_none());
        assert!(mapping.topic_command.is_none());
    }

    #[test]
    fn topic_mapping_legacy_driver_default() {
        let table = test_table(r#"camera = "opencv""#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("camera").unwrap();
        assert!(mapping.topic.is_none());
        assert!(mapping.topic_state.is_none());
        assert!(mapping.topic_command.is_none());
    }

    #[test]
    fn topic_mapping_all_three_fields() {
        let table = test_table(
            r#"
            [robot]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            topic = "sensors/feedback"
            topic_state = "robot/state"
            topic_command = "robot/cmd"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("robot").unwrap();
        assert_eq!(mapping.topic.as_deref(), Some("sensors/feedback"));
        assert_eq!(mapping.topic_state.as_deref(), Some("robot/state"));
        assert_eq!(mapping.topic_command.as_deref(), Some("robot/cmd"));
    }

    #[test]
    fn topic_mapping_missing_driver() {
        let table = test_table(r#"arm = true"#);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.topic_mapping("nonexistent").is_none());
    }
}
