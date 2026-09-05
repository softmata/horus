#![allow(dead_code)]
//! Integration tests for the hardware loading API.
//!
//! Tests: load_from() with [hardware] config, NodeParams, registry,
//! sim override, exec prefix, unknown node type, and legacy [drivers] fallback.

#[cfg(test)]
mod driver_tests {
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::{Arc, Mutex};

    use crate::core::Node;
    use crate::drivers::params::NodeParams;
    use crate::drivers::registry;
    use crate::error::HorusResult;

    // Guard for tests that manipulate HORUS_SIM_MODE env var (process-global).
    // Tests acquiring this lock run sequentially, preventing env var races.
    static SIM_ENV_LOCK: Mutex<()> = Mutex::new(());

    // ── Test helper: stub node ───────────────────────────────────────

    struct StubNode {
        node_name: String,
        tick_count: Arc<AtomicU32>,
        #[allow(dead_code)]
        value: u32,
    }

    impl StubNode {
        fn new(name: &str, tick_count: Arc<AtomicU32>) -> Self {
            Self {
                node_name: name.to_string(),
                tick_count,
                value: 0,
            }
        }

        fn from_params(params: &NodeParams) -> HorusResult<Self> {
            let value = params.get_or("value", 0u32);
            Ok(Self {
                node_name: "stub".to_string(),
                tick_count: Arc::new(AtomicU32::new(0)),
                value,
            })
        }
    }

    impl Node for StubNode {
        fn name(&self) -> &str {
            &self.node_name
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::Relaxed);
        }
    }

    // ── Registry tests ───────────────────────────────────────────────

    #[test]
    fn register_and_lookup() {
        registry::register("HwTestStub", |params| {
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let factory = registry::lookup("HwTestStub");
        assert!(factory.is_some());

        let params = NodeParams::empty();
        let node = factory.unwrap()(&params).unwrap();
        assert_eq!(node.name(), "stub");
    }

    #[test]
    fn lookup_missing_returns_none() {
        assert!(registry::lookup("HwNonexistent").is_none());
    }

    #[test]
    fn list_registered_includes_entry() {
        registry::register("HwListTest", |_| {
            Ok(Box::new(StubNode::new(
                "list_test",
                Arc::new(AtomicU32::new(0)),
            )))
        });
        let names = registry::list_registered();
        assert!(names.contains(&"HwListTest".to_string()));
    }

    // ── NodeParams tests ─────────────────────────────────────────────

    #[test]
    fn node_params_get_string() {
        let mut map = HashMap::new();
        map.insert(
            "port".to_string(),
            toml::Value::String("/dev/ttyUSB0".into()),
        );
        let params = NodeParams::new(map);
        let port: String = params.get("port").unwrap();
        assert_eq!(port, "/dev/ttyUSB0");
    }

    #[test]
    fn node_params_get_or_default() {
        let params = NodeParams::empty();
        let baud: u32 = params.get_or("baudrate", 115200);
        assert_eq!(baud, 115200);
    }

    #[test]
    fn node_params_missing_key_errors() {
        let params = NodeParams::empty();
        let result = params.get::<String>("missing");
        result.unwrap_err();
    }

    // ── load_from() tests ────────────────────────────────────────────

    fn write_temp_toml(content: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("horus_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(format!("test_{}.toml", rand_name()));
        std::fs::write(&path, content).unwrap();
        path
    }

    fn rand_name() -> u64 {
        use std::time::SystemTime;
        SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    #[test]
    fn load_empty_hardware_section() {
        let path = write_temp_toml("[hardware]\n");
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert!(nodes.is_empty());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_no_hardware_section() {
        let path = write_temp_toml("[package]\nname = \"test\"\n");
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert!(nodes.is_empty());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_registered_node_type() {
        registry::register("LoadTestNode", |params| {
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let content = r#"
[hardware.sensor]
use = "LoadTestNode"
value = 42
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].0, "sensor");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_unknown_node_type_errors() {
        let content = r#"
[hardware.sensor]
use = "CompletelyUnknownType"
"#;
        let path = write_temp_toml(content);
        let result = crate::drivers::load_from(&path);
        assert!(result.is_err());
        let err = result.err().unwrap().to_string();
        assert!(err.contains("unknown node type"));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_missing_use_field_warns_and_skips() {
        let content = r#"
[hardware.sensor]
port = "/dev/ttyUSB0"
"#;
        // No 'use' field, no legacy keys → should skip (not error)
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert!(nodes.is_empty());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_legacy_drivers_section_fallback() {
        registry::register("LegacyTestNode", |params| {
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let content = r#"
[drivers.sensor]
use = "LegacyTestNode"
value = 10
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].0, "sensor");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_legacy_terra_key_fallback() {
        registry::register("dynamixel", |params| {
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let content = r#"
[hardware.arm]
terra = "dynamixel"
port = "/dev/ttyUSB0"
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].0, "arm");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_legacy_node_key_fallback() {
        registry::register("MyCustomNode", |params| {
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let content = r#"
[hardware.custom]
node = "MyCustomNode"
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_sim_mode_creates_stub() {
        let _guard = SIM_ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        // SAFETY: guarded by SIM_ENV_LOCK to prevent parallel env var mutation
        unsafe {
            std::env::set_var("HORUS_SIM_MODE", "1");
        }

        let content = r#"
[hardware.sensor]
use = "NonexistentButSimmed"
sim = true
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].0, "sensor");
        assert!(nodes[0].1.name().contains("sim_stub"));

        unsafe {
            std::env::remove_var("HORUS_SIM_MODE");
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_sim_mode_selective_targets() {
        let _guard = SIM_ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        unsafe {
            std::env::set_var("HORUS_SIM_MODE", "1");
        }
        unsafe {
            std::env::set_var("HORUS_SIM_TARGETS", "lidar");
        }

        registry::register("SelectiveTestA", |p| {
            Ok(Box::new(StubNode::from_params(p)?))
        });
        registry::register("SelectiveTestB", |p| {
            Ok(Box::new(StubNode::from_params(p)?))
        });

        let content = r#"
[hardware.lidar]
use = "SelectiveTestA"
sim = true

[hardware.imu]
use = "SelectiveTestB"
sim = true
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();

        // lidar should be sim stub (in targets), imu should try to load
        let lidar = nodes.iter().find(|(n, _)| n == "lidar").unwrap();
        assert!(lidar.1.name().contains("sim_stub"));

        // imu has sim=true but is NOT in HORUS_SIM_TARGETS, so it loads normally
        let imu = nodes.iter().find(|(n, _)| n == "imu").unwrap();
        assert_eq!(imu.1.name(), "stub"); // from factory

        unsafe {
            std::env::remove_var("HORUS_SIM_MODE");
        }
        unsafe {
            std::env::remove_var("HORUS_SIM_TARGETS");
        }
        std::fs::remove_file(&path).ok();
    }

    /// The two loaders must agree about what `--sim` covers.
    ///
    /// `load_from` substituted a stub while `load_config_entries` -- the whole
    /// of the Python path -- reported nothing, so `hardware.load()` in Python
    /// constructed the REAL driver under `horus run --sim`. Constructing a
    /// driver is where the port is opened and the motor enabled, so `--sim`
    /// was inert on that path. Load one manifest through both and require the
    /// same verdict per entry.
    #[test]
    fn both_loaders_agree_on_what_sim_mode_covers() {
        let _guard = SIM_ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        // SAFETY: guarded by SIM_ENV_LOCK to prevent parallel env var mutation
        unsafe {
            std::env::set_var("HORUS_SIM_MODE", "1");
        }

        registry::register("AgreementReal", |p| Ok(Box::new(StubNode::from_params(p)?)));

        let content = r#"
[hardware.simmed]
use = "AgreementReal"
sim = true

[hardware.aliased]
use = "AgreementReal"
simulated = true

[hardware.real]
use = "AgreementReal"
"#;
        let path = write_temp_toml(content);

        let nodes = crate::drivers::load_from(&path).unwrap();
        let entries = crate::drivers::load_config_entries(&path).unwrap();

        for (name, node) in &nodes {
            let stubbed_by_load = node.name().contains("sim_stub");
            let (_, _, _, simulated) = entries
                .iter()
                .find(|(n, _, _, _)| n == name)
                .unwrap_or_else(|| panic!("load_config_entries dropped {name}"));
            assert_eq!(
                stubbed_by_load, *simulated,
                "the two loaders disagree about {name}: load_from stubbed={stubbed_by_load}, \
                 load_config_entries simulated={simulated}"
            );
        }

        // And the verdicts themselves are the right ones.
        let simulated_names: Vec<&str> = entries
            .iter()
            .filter(|(_, _, _, sim)| *sim)
            .map(|(n, _, _, _)| n.as_str())
            .collect();
        assert_eq!(simulated_names, vec!["aliased", "simmed"]);

        unsafe {
            std::env::remove_var("HORUS_SIM_MODE");
        }
        std::fs::remove_file(&path).ok();
    }

    /// Outside `--sim`, `sim = true` changes nothing on either path.
    #[test]
    fn config_entries_report_no_simulation_without_sim_mode() {
        let _guard = SIM_ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        unsafe {
            std::env::remove_var("HORUS_SIM_MODE");
        }

        let content = r#"
[hardware.sensor]
use = "AnythingAtAll"
sim = true
"#;
        let path = write_temp_toml(content);
        let entries = crate::drivers::load_config_entries(&path).unwrap();
        assert_eq!(entries.len(), 1);
        assert!(
            !entries[0].3,
            "`sim = true` alone must not simulate - only `--sim` does"
        );
        std::fs::remove_file(&path).ok();
    }

    /// HORUS_SIM_TARGETS must narrow both loaders the same way.
    #[test]
    fn config_entries_honour_the_selective_target_filter() {
        let _guard = SIM_ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        unsafe {
            std::env::set_var("HORUS_SIM_MODE", "1");
            std::env::set_var("HORUS_SIM_TARGETS", "lidar");
        }

        let content = r#"
[hardware.lidar]
use = "SomeType"
sim = true

[hardware.imu]
use = "SomeType"
sim = true
"#;
        let path = write_temp_toml(content);
        let entries = crate::drivers::load_config_entries(&path).unwrap();
        let sim_of = |want: &str| {
            entries
                .iter()
                .find(|(n, _, _, _)| n == want)
                .map(|(_, _, _, s)| *s)
                .unwrap()
        };
        assert!(sim_of("lidar"), "lidar is in HORUS_SIM_TARGETS");
        assert!(!sim_of("imu"), "imu is not in HORUS_SIM_TARGETS");

        unsafe {
            std::env::remove_var("HORUS_SIM_MODE");
            std::env::remove_var("HORUS_SIM_TARGETS");
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_exec_prefix() {
        let content = r#"
[hardware.camera]
use = "exec:/bin/echo"
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].0, "camera");
        // The driver is named after the [hardware.<name>] key, matching the
        // name the scheduler knows it by. This used to assert "echo" — the
        // binary's file stem — which is the defect, not the contract: two
        // [hardware] entries pointing at one binary were indistinguishable in
        // the driver's own logs while the scheduler knew them apart.
        assert_eq!(nodes[0].1.name(), "camera");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_params_passed_to_factory() {
        registry::register("ParamPassTest", |params| {
            let value: u32 = params.get("value")?;
            assert_eq!(value, 999);
            Ok(Box::new(StubNode {
                node_name: "param_test".to_string(),
                tick_count: Arc::new(AtomicU32::new(0)),
                value,
            }))
        });

        let content = r#"
[hardware.sensor]
use = "ParamPassTest"
value = 999
port = "/dev/ttyUSB0"
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 1);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_reserved_keys_not_in_params() {
        registry::register("ReservedKeyTest", |params| {
            // 'use' and 'sim' should NOT be in params
            assert!(!params.has("use"));
            assert!(!params.has("sim"));
            // But 'port' should be
            assert!(params.has("port"));
            Ok(Box::new(StubNode::from_params(params)?))
        });

        let content = r#"
[hardware.sensor]
use = "ReservedKeyTest"
sim = false
port = "/dev/ttyUSB0"
"#;
        let path = write_temp_toml(content);
        let _nodes = crate::drivers::load_from(&path).unwrap();
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_multiple_hardware_entries() {
        registry::register("MultiA", |p| Ok(Box::new(StubNode::from_params(p)?)));
        registry::register("MultiB", |p| Ok(Box::new(StubNode::from_params(p)?)));

        let content = r#"
[hardware.sensor_a]
use = "MultiA"

[hardware.sensor_b]
use = "MultiB"
"#;
        let path = write_temp_toml(content);
        let nodes = crate::drivers::load_from(&path).unwrap();
        assert_eq!(nodes.len(), 2);
        std::fs::remove_file(&path).ok();
    }
}
