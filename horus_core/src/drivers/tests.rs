//! Integration tests for the driver API.
//!
//! Tests all three driver paths (terra, package, local) and verifies
//! integration with the scheduler via tick_once().

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    use crate::core::Node;
    use crate::drivers::hardware_set::HardwareSet;
    use crate::drivers::params::DriverParams;
    use crate::drivers::registry;
    use crate::error::HorusResult;
    use crate::scheduling::Scheduler;

    // ── Test helper: stub driver node ─────────────────────────────────

    struct StubDriverNode {
        name: String,
        tick_count: Arc<AtomicU32>,
        value: u32,
    }

    impl StubDriverNode {
        fn new(name: &str, tick_count: Arc<AtomicU32>) -> Self {
            Self {
                name: name.to_string(),
                tick_count,
                value: 0,
            }
        }

        fn from_params_with_counter(
            params: &DriverParams,
            counter: Arc<AtomicU32>,
        ) -> HorusResult<Self> {
            let value = params.get_or("value", 42u32);
            Ok(Self {
                name: "stub_from_params".to_string(),
                tick_count: counter,
                value,
            })
        }
    }

    impl Node for StubDriverNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::Relaxed);
        }
    }

    // ── HardwareSet construction tests ────────────────────────────────

    #[test]
    fn from_toml_terra_driver_has_correct_type() {
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            baudrate = 1000000
            servo_ids = [1, 2, 3, 4, 5, 6]
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("arm"));
        assert!(matches!(
            hw.driver_type("arm"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "dynamixel"
        ));
    }

    #[test]
    fn from_toml_package_driver_has_correct_type() {
        let table = parse_toml_table(
            r#"
            [force_sensor]
            package = "horus-driver-ati-netft"
            address = "192.168.1.100"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(
            hw.driver_type("force_sensor"),
            Some(crate::drivers::DriverType::Package(p)) if p == "horus-driver-ati-netft"
        ));
    }

    #[test]
    fn from_toml_local_driver_has_correct_type() {
        let table = parse_toml_table(
            r#"
            [conveyor]
            node = "ConveyorDriver"
            port = "/dev/ttyACM0"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(
            hw.driver_type("conveyor"),
            Some(crate::drivers::DriverType::Local(n)) if n == "ConveyorDriver"
        ));
    }

    #[test]
    fn from_toml_mixed_config() {
        let table = parse_toml_table(
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
            port = "/dev/ttyACM0"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 5);
        assert!(hw.has("camera"));
        assert!(hw.has("gps"));
        assert!(hw.has("arm"));
        assert!(hw.has("sensor"));
        assert!(hw.has("conveyor"));
    }

    // ── Terra handle accessor tests ──────────────────────────────────

    #[test]
    fn terra_handle_returns_params() {
        let table = parse_toml_table(
            r#"
            [lidar]
            terra = "rplidar"
            port = "/dev/ttyUSB1"
            scan_mode = "sensitivity"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let handle = hw.rplidar("lidar").unwrap();
        assert_eq!(handle.terra_name(), Some("rplidar"));
        assert_eq!(
            handle.params().get::<String>("port").unwrap(),
            "/dev/ttyUSB1"
        );
        assert_eq!(
            handle.params().get::<String>("scan_mode").unwrap(),
            "sensitivity"
        );
    }

    #[test]
    fn terra_handle_wrong_source_type_errors() {
        let table = parse_toml_table(
            r#"
            [sensor]
            package = "some-package"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let err = hw.dynamixel("sensor").unwrap_err().to_string();
        assert!(err.contains("sensor"), "error should mention driver name");
    }

    #[test]
    fn terra_handle_missing_driver_lists_available() {
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let err = hw.dynamixel("leg").unwrap_err().to_string();
        assert!(err.contains("leg"));
        assert!(err.contains("arm"));
    }

    // ── Local driver registry + HardwareSet::local() integration ─────

    // Simple non-capturing stub for registry tests
    struct SimpleStub;
    impl SimpleStub {
        fn from_params(_params: &DriverParams) -> HorusResult<Self> {
            Ok(Self)
        }
    }
    impl Node for SimpleStub {
        fn name(&self) -> &str {
            "simple_stub"
        }
        fn tick(&mut self) {}
    }

    #[test]
    fn local_driver_registered_and_instantiated() {
        // Register with fn pointer (no captures)
        registry::register("IntegTestDriver", |_params| Ok(Box::new(SimpleStub)));

        let table = parse_toml_table(
            r#"
            [my_driver]
            node = "IntegTestDriver"
            value = 99
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.local("my_driver").unwrap();
        assert_eq!(node.name(), "simple_stub");
    }

    #[test]
    fn local_driver_unregistered_errors_clearly() {
        let table = parse_toml_table(
            r#"
            [mystery]
            node = "UnregisteredDriver123"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.local("mystery");
        assert!(result.is_err());
        let err = result.err().unwrap().to_string();
        assert!(err.contains("UnregisteredDriver123"));
        assert!(err.contains("register_driver"));
    }

    // ── Config table validation ──────────────────────────────────────

    #[test]
    fn config_table_without_source_key_errors() {
        let table = parse_toml_table(
            r#"
            [broken]
            port = "/dev/ttyUSB0"
            baudrate = 115200
        "#,
        );
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("terra"));
        assert!(err.contains("package"));
        assert!(err.contains("node"));
    }

    // ── Scheduler integration: add() + tick_once() ─────────────

    #[test]
    fn driver_node_ticks_in_scheduler() {
        let counter = Arc::new(AtomicU32::new(0));

        let mut sched = Scheduler::new().deterministic(true);
        sched
            .add(StubDriverNode::new("tick_test", counter.clone()))
            .build()
            .unwrap();
        sched.tick_once().unwrap();

        assert_eq!(counter.load(Ordering::Relaxed), 1);

        sched.tick_once().unwrap();
        assert_eq!(counter.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn multiple_driver_nodes_tick_in_order() {
        let counter_a = Arc::new(AtomicU32::new(0));
        let counter_b = Arc::new(AtomicU32::new(0));

        let mut sched = Scheduler::new().deterministic(true);
        sched
            .add(StubDriverNode::new("driver_a", counter_a.clone()))
            .order(0)
            .build()
            .unwrap();
        sched
            .add(StubDriverNode::new("driver_b", counter_b.clone()))
            .order(1)
            .build()
            .unwrap();
        sched.tick_once().unwrap();

        assert_eq!(counter_a.load(Ordering::Relaxed), 1);
        assert_eq!(counter_b.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn boxed_dyn_node_works_with_add() {
        // This is what hw.local() returns — Box<dyn Node>
        let counter = Arc::new(AtomicU32::new(0));
        let node: Box<dyn Node> = Box::new(StubDriverNode::new("boxed", counter.clone()));

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(node).build().unwrap();
        sched.tick_once().unwrap();

        assert_eq!(counter.load(Ordering::Relaxed), 1);
    }

    // Global atomic for scheduler integration test (fn pointers can't capture)
    static SCHED_INTEG_COUNTER: AtomicU32 = AtomicU32::new(0);

    struct SchedIntegStub;
    impl Node for SchedIntegStub {
        fn name(&self) -> &str {
            "sched_integ"
        }
        fn tick(&mut self) {
            SCHED_INTEG_COUNTER.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[test]
    fn local_driver_from_config_ticks_in_scheduler() {
        SCHED_INTEG_COUNTER.store(0, Ordering::Relaxed);

        registry::register("SchedulerIntegDriver", |_params| {
            Ok(Box::new(SchedIntegStub))
        });

        let table = parse_toml_table(
            r#"
            [motor]
            node = "SchedulerIntegDriver"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.local("motor").unwrap();

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(node).build().unwrap();
        sched.tick_once().unwrap();

        assert_eq!(SCHED_INTEG_COUNTER.load(Ordering::Relaxed), 1);
    }

    // ── Introspection ────────────────────────────────────────────────

    #[test]
    fn list_returns_sorted_names() {
        let table = parse_toml_table(
            r#"
            zebra = true
            alpha = true
            [mid]
            terra = "i2c"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.list(), vec!["alpha", "mid", "zebra"]);
    }

    #[test]
    fn params_accessible_for_terra_driver() {
        let table = parse_toml_table(
            r#"
            [imu]
            terra = "bno055"
            bus = "i2c-1"
            address = 40
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("imu").unwrap();
        assert_eq!(params.get::<String>("bus").unwrap(), "i2c-1");
        assert_eq!(params.get::<u32>("address").unwrap(), 40);
    }

    #[test]
    fn empty_drivers_section() {
        let table = toml::value::Table::new();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.is_empty());
        assert_eq!(hw.len(), 0);
        assert!(hw.list().is_empty());
    }

    // ── Legacy backward compat ───────────────────────────────────────

    #[test]
    fn legacy_string_and_bool_preserved() {
        let table = parse_toml_table(
            r#"
            camera = "opencv"
            gps = true
            imu = false
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 3);
        assert!(matches!(
            hw.driver_type("camera"),
            Some(crate::drivers::DriverType::Legacy)
        ));
        assert!(matches!(
            hw.driver_type("gps"),
            Some(crate::drivers::DriverType::Legacy)
        ));
    }

    // ── Helper ───────────────────────────────────────────────────────

    fn parse_toml_table(s: &str) -> toml::value::Table {
        let value: toml::Value = toml::from_str(s).unwrap();
        value.as_table().unwrap().clone()
    }
}
