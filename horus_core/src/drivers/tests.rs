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

    // ══════════════════════════════════════════════════════════════════
    // ═══ BATTLE TESTS ═════════════════════════════════════════════════
    // ══════════════════════════════════════════════════════════════════

    // ── Config parsing edge cases ────────────────────────────────────

    #[test]
    fn battle_test_config_deeply_nested_params() {
        // Arrays of tables and nested objects in driver params
        let table = parse_toml_table(
            r#"
            [robot_arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            servo_ids = [1, 2, 3, 4, 5, 6]
            gains = [0.1, 0.2, 0.3]
            labels = ["shoulder", "elbow", "wrist"]
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("robot_arm").unwrap();
        let ids: Vec<u8> = params.get("servo_ids").unwrap();
        assert_eq!(ids, vec![1, 2, 3, 4, 5, 6]);
        let gains: Vec<f64> = params.get("gains").unwrap();
        assert_eq!(gains.len(), 3);
        assert!((gains[0] - 0.1).abs() < 1e-10);
        let labels: Vec<String> = params.get("labels").unwrap();
        assert_eq!(labels, vec!["shoulder", "elbow", "wrist"]);
    }

    #[test]
    fn battle_test_config_unicode_driver_names() {
        // Unicode in driver names (TOML requires quoted keys for non-ASCII)
        let table = parse_toml_table(
            r#"
            ["sensor_robótico"]
            terra = "dynamixel"
            description = "Bras robotique — données en temps réel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("sensor_robótico"));
        let params = hw.params("sensor_robótico").unwrap();
        let desc: String = params.get("description").unwrap();
        assert!(desc.contains("réel"));
    }

    #[test]
    fn battle_test_config_unicode_param_values() {
        let table = parse_toml_table(
            r#"
            [motor]
            terra = "dynamixel"
            label = "モーター制御"
            note = "日本語テスト 🤖"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("motor").unwrap();
        assert_eq!(params.get::<String>("label").unwrap(), "モーター制御");
        assert!(params.get::<String>("note").unwrap().contains("日本語"));
    }

    #[test]
    fn battle_test_config_very_long_string_values() {
        let long_val = "x".repeat(10_000);
        let toml_str = format!(
            r#"
            [sensor]
            terra = "serial"
            data = "{}"
        "#,
            long_val
        );
        let table = parse_toml_table(&toml_str);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("sensor").unwrap();
        let data: String = params.get("data").unwrap();
        assert_eq!(data.len(), 10_000);
    }

    #[test]
    fn battle_test_config_many_drivers_50_plus() {
        // Generate 60 drivers in a single config
        let mut toml_str = String::new();
        for i in 0..60 {
            toml_str.push_str(&format!(
                "[driver_{}]\nterra = \"serial\"\nport = \"/dev/ttyUSB{}\"\n\n",
                i, i
            ));
        }
        let table = parse_toml_table(&toml_str);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 60);
        for i in 0..60 {
            let name = format!("driver_{}", i);
            assert!(hw.has(&name), "missing driver {}", name);
            let params = hw.params(&name).unwrap();
            let port: String = params.get("port").unwrap();
            assert_eq!(port, format!("/dev/ttyUSB{}", i));
        }
    }

    #[test]
    fn battle_test_config_empty_string_values() {
        let table = parse_toml_table(
            r#"
            [sensor]
            terra = "serial"
            port = ""
            label = ""
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("sensor").unwrap();
        assert_eq!(params.get::<String>("port").unwrap(), "");
        assert_eq!(params.get::<String>("label").unwrap(), "");
    }

    #[test]
    fn battle_test_config_all_terra_types_from_map() {
        // Every known terra driver name should parse correctly
        let terra_names = [
            "dynamixel",
            "rplidar",
            "vesc",
            "modbus",
            "mavlink",
            "robotiq",
            "ublox",
            "nmea",
            "mpu6050",
            "icm20689",
            "bno055",
            "bme280",
            "canopen",
            "odrive",
            "realsense",
            "webcam",
            "velodyne",
            "ouster",
            "livox",
            "hesai",
            "ethercat",
            "ethernetip",
            "profinet",
            "i2c",
            "spi",
            "serial",
            "uart",
            "can",
            "gpio",
            "pwm",
            "adc",
            "usb",
            "bluetooth",
            "ble",
            "input",
            "gamepad",
            "net",
            "tcp",
            "udp",
            "virtual",
            "mock",
            "zed",
            "oakd",
        ];
        let mut toml_str = String::new();
        for (i, name) in terra_names.iter().enumerate() {
            // Use index-based driver name to avoid duplicates
            toml_str.push_str(&format!(
                "[d_{}]\nterra = \"{}\"\nport = \"/dev/test{}\"\n\n",
                i, name, i
            ));
        }
        let table = parse_toml_table(&toml_str);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), terra_names.len());
        for i in 0..terra_names.len() {
            let dname = format!("d_{}", i);
            assert!(hw.has(&dname), "missing driver d_{} (terra={})", i, terra_names[i]);
            assert!(matches!(
                hw.driver_type(&dname),
                Some(crate::drivers::DriverType::Terra(_))
            ));
        }
    }

    #[test]
    fn battle_test_config_mixed_legacy_string_and_config_table() {
        let table = parse_toml_table(
            r#"
            camera = "opencv"
            gps = true
            imu = false

            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"

            [laser]
            package = "horus-driver-laser"
            address = "10.0.0.1"

            [custom]
            node = "MyCustomDriver"
            rate = 100
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 6);
        // Legacy
        assert!(matches!(
            hw.driver_type("camera"),
            Some(crate::drivers::DriverType::Legacy)
        ));
        assert!(matches!(
            hw.driver_type("gps"),
            Some(crate::drivers::DriverType::Legacy)
        ));
        assert!(matches!(
            hw.driver_type("imu"),
            Some(crate::drivers::DriverType::Legacy)
        ));
        // Terra
        assert!(matches!(
            hw.driver_type("arm"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "dynamixel"
        ));
        // Package
        assert!(matches!(
            hw.driver_type("laser"),
            Some(crate::drivers::DriverType::Package(p)) if p == "horus-driver-laser"
        ));
        // Local
        assert!(matches!(
            hw.driver_type("custom"),
            Some(crate::drivers::DriverType::Local(n)) if n == "MyCustomDriver"
        ));
    }

    #[test]
    fn battle_test_config_invalid_missing_source_key() {
        let table = parse_toml_table(
            r#"
            [broken]
            port = "/dev/ttyUSB0"
            baudrate = 115200
            mode = "fast"
        "#,
        );
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("broken"));
        assert!(err.contains("terra"));
        assert!(err.contains("package"));
        assert!(err.contains("node"));
    }

    #[test]
    fn battle_test_config_integer_value_rejected() {
        // An integer value (not table, string, or bool) should be rejected
        let mut table = toml::value::Table::new();
        table.insert("bad_driver".to_string(), toml::Value::Integer(42));
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad_driver"));
    }

    #[test]
    fn battle_test_config_float_value_rejected() {
        let mut table = toml::value::Table::new();
        table.insert("bad_driver".to_string(), toml::Value::Float(3.14));
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad_driver"));
    }

    #[test]
    fn battle_test_config_array_value_rejected() {
        let mut table = toml::value::Table::new();
        table.insert(
            "bad_driver".to_string(),
            toml::Value::Array(vec![toml::Value::Integer(1)]),
        );
        let err = HardwareSet::from_toml_table(&table)
            .unwrap_err()
            .to_string();
        assert!(err.contains("bad_driver"));
    }

    #[test]
    fn battle_test_config_whitespace_heavy_values() {
        let table = parse_toml_table(
            r#"
            [sensor]
            terra = "serial"
            port = "  /dev/ttyUSB0  "
            label = "   spaces   everywhere   "
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("sensor").unwrap();
        // TOML preserves whitespace inside strings
        assert_eq!(params.get::<String>("port").unwrap(), "  /dev/ttyUSB0  ");
        assert!(params.get::<String>("label").unwrap().contains("spaces"));
    }

    #[test]
    fn battle_test_config_comments_preserved_in_toml() {
        // TOML comments are ignored by the parser, verify config still loads
        let table = parse_toml_table(
            r#"
            # This is a comment about the arm driver
            [arm]
            terra = "dynamixel" # inline comment
            port = "/dev/ttyUSB0"
            # baudrate is optional
            baudrate = 1000000
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("arm"));
        let params = hw.params("arm").unwrap();
        assert_eq!(params.get::<u32>("baudrate").unwrap(), 1_000_000);
    }

    #[test]
    fn battle_test_config_boolean_false_legacy_driver() {
        // Even false booleans should be loaded as Legacy
        let table = parse_toml_table(
            r#"
            disabled_sensor = false
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("disabled_sensor"));
        assert!(matches!(
            hw.driver_type("disabled_sensor"),
            Some(crate::drivers::DriverType::Legacy)
        ));
    }

    #[test]
    fn battle_test_config_numeric_param_types() {
        let table = parse_toml_table(
            r#"
            [sensor]
            terra = "i2c"
            address_u8 = 104
            offset_i32 = -42
            big_number = 5000000000
            gain_float = 3.14159
            rate_int_as_float = 100
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("sensor").unwrap();
        assert_eq!(params.get::<u8>("address_u8").unwrap(), 104);
        assert_eq!(params.get::<i32>("offset_i32").unwrap(), -42);
        assert_eq!(params.get::<u64>("big_number").unwrap(), 5_000_000_000);
        assert!((params.get::<f64>("gain_float").unwrap() - 3.14159).abs() < 1e-10);
        // Integer accessed as f64 should work
        assert!((params.get::<f64>("rate_int_as_float").unwrap() - 100.0).abs() < 1e-10);
    }

    #[test]
    fn battle_test_config_reserved_keys_filtered_from_params() {
        // topic, topic_state, topic_command, terra, package, node are reserved
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            topic = "arm/data"
            topic_state = "arm/state"
            topic_command = "arm/cmd"
            port = "/dev/ttyUSB0"
            baudrate = 115200
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("arm").unwrap();
        // Reserved keys must NOT be in params
        assert!(!params.has("terra"));
        assert!(!params.has("topic"));
        assert!(!params.has("topic_state"));
        assert!(!params.has("topic_command"));
        // User keys must be present
        assert!(params.has("port"));
        assert!(params.has("baudrate"));
    }

    // ── DriverParams battle tests ────────────────────────────────────

    fn make_params(pairs: &[(&str, toml::Value)]) -> DriverParams {
        let mut map = HashMap::new();
        for (k, v) in pairs {
            map.insert(k.to_string(), v.clone());
        }
        DriverParams::new(map)
    }

    #[test]
    fn battle_test_params_all_supported_types() {
        let params = make_params(&[
            ("s", toml::Value::String("hello".into())),
            ("i64", toml::Value::Integer(42)),
            ("f64", toml::Value::Float(2.718)),
            ("b", toml::Value::Boolean(true)),
            (
                "arr",
                toml::Value::Array(vec![
                    toml::Value::Integer(1),
                    toml::Value::Integer(2),
                    toml::Value::Integer(3),
                ]),
            ),
        ]);
        assert_eq!(params.get::<String>("s").unwrap(), "hello");
        assert_eq!(params.get::<i64>("i64").unwrap(), 42);
        assert!((params.get::<f64>("f64").unwrap() - 2.718).abs() < 1e-10);
        assert!(params.get::<bool>("b").unwrap());
        assert_eq!(params.get::<Vec<u8>>("arr").unwrap(), vec![1, 2, 3]);
    }

    #[test]
    fn battle_test_params_missing_key_returns_error() {
        let params = DriverParams::empty();
        let err = params.get::<String>("nonexistent").unwrap_err().to_string();
        assert!(err.contains("nonexistent"));
        assert!(err.contains("required"));
    }

    #[test]
    fn battle_test_params_type_mismatch_string_for_int() {
        let params = make_params(&[("val", toml::Value::String("not_a_number".into()))]);
        let result = params.get::<u32>("val");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("val"));
    }

    #[test]
    fn battle_test_params_type_mismatch_int_for_string() {
        let params = make_params(&[("val", toml::Value::Integer(42))]);
        let result = params.get::<String>("val");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("string"));
    }

    #[test]
    fn battle_test_params_type_mismatch_bool_for_int() {
        let params = make_params(&[("val", toml::Value::Boolean(true))]);
        assert!(params.get::<u32>("val").is_err());
        assert!(params.get::<i64>("val").is_err());
        assert!(params.get::<f64>("val").is_err());
    }

    #[test]
    fn battle_test_params_type_mismatch_int_for_bool() {
        let params = make_params(&[("val", toml::Value::Integer(1))]);
        assert!(params.get::<bool>("val").is_err());
    }

    #[test]
    fn battle_test_params_type_mismatch_string_for_bool() {
        let params = make_params(&[("val", toml::Value::String("true".into()))]);
        assert!(params.get::<bool>("val").is_err());
    }

    #[test]
    fn battle_test_params_default_value_on_missing() {
        let params = DriverParams::empty();
        assert_eq!(params.get_or("missing", 42u32), 42);
        assert_eq!(params.get_or("missing", "default".to_string()), "default");
        assert_eq!(params.get_or("missing", true), true);
        assert!((params.get_or("missing", 3.14f64) - 3.14).abs() < 1e-10);
    }

    #[test]
    fn battle_test_params_default_value_on_type_mismatch() {
        let params = make_params(&[("val", toml::Value::String("not_a_number".into()))]);
        // Requesting u32 on a string value should return the default
        assert_eq!(params.get_or("val", 999u32), 999);
    }

    #[test]
    fn battle_test_params_empty_operations() {
        let params = DriverParams::empty();
        assert!(params.is_empty());
        assert_eq!(params.len(), 0);
        assert!(!params.has("anything"));
        assert!(params.raw("anything").is_none());
        assert_eq!(params.keys().count(), 0);
    }

    #[test]
    fn battle_test_params_very_large_integer() {
        let params = make_params(&[("big", toml::Value::Integer(i64::MAX))]);
        assert_eq!(params.get::<i64>("big").unwrap(), i64::MAX);
        // Should fail for u32 (overflow)
        assert!(params.get::<u32>("big").is_err());
        // Should work for u64 since MAX i64 fits
        assert_eq!(params.get::<u64>("big").unwrap(), i64::MAX as u64);
    }

    #[test]
    fn battle_test_params_negative_for_unsigned_fails() {
        let params = make_params(&[("val", toml::Value::Integer(-1))]);
        assert!(params.get::<u32>("val").is_err());
        assert!(params.get::<u64>("val").is_err());
        assert!(params.get::<u8>("val").is_err());
        // But should work for signed types
        assert_eq!(params.get::<i32>("val").unwrap(), -1);
        assert_eq!(params.get::<i64>("val").unwrap(), -1);
    }

    #[test]
    fn battle_test_params_u8_boundary_values() {
        let params_0 = make_params(&[("val", toml::Value::Integer(0))]);
        assert_eq!(params_0.get::<u8>("val").unwrap(), 0);

        let params_255 = make_params(&[("val", toml::Value::Integer(255))]);
        assert_eq!(params_255.get::<u8>("val").unwrap(), 255);

        let params_256 = make_params(&[("val", toml::Value::Integer(256))]);
        assert!(params_256.get::<u8>("val").is_err());
    }

    #[test]
    fn battle_test_params_f32_from_integer() {
        let params = make_params(&[("val", toml::Value::Integer(100))]);
        let v: f32 = params.get("val").unwrap();
        assert!((v - 100.0).abs() < 1e-6);
    }

    #[test]
    fn battle_test_params_f32_from_float() {
        let params = make_params(&[("val", toml::Value::Float(3.14))]);
        let v: f32 = params.get("val").unwrap();
        assert!((v - 3.14).abs() < 1e-5);
    }

    #[test]
    fn battle_test_params_vec_string() {
        let arr = vec![
            toml::Value::String("a".into()),
            toml::Value::String("b".into()),
            toml::Value::String("c".into()),
        ];
        let params = make_params(&[("names", toml::Value::Array(arr))]);
        let names: Vec<String> = params.get("names").unwrap();
        assert_eq!(names, vec!["a", "b", "c"]);
    }

    #[test]
    fn battle_test_params_vec_f64() {
        let arr = vec![
            toml::Value::Float(1.1),
            toml::Value::Float(2.2),
            toml::Value::Float(3.3),
        ];
        let params = make_params(&[("vals", toml::Value::Array(arr))]);
        let vals: Vec<f64> = params.get("vals").unwrap();
        assert_eq!(vals.len(), 3);
        assert!((vals[0] - 1.1).abs() < 1e-10);
    }

    #[test]
    fn battle_test_params_vec_bool() {
        let arr = vec![
            toml::Value::Boolean(true),
            toml::Value::Boolean(false),
            toml::Value::Boolean(true),
        ];
        let params = make_params(&[("flags", toml::Value::Array(arr))]);
        let flags: Vec<bool> = params.get("flags").unwrap();
        assert_eq!(flags, vec![true, false, true]);
    }

    #[test]
    fn battle_test_params_empty_vec() {
        let params = make_params(&[("empty_arr", toml::Value::Array(vec![]))]);
        let vals: Vec<u32> = params.get("empty_arr").unwrap();
        assert!(vals.is_empty());
    }

    #[test]
    fn battle_test_params_vec_type_mismatch_element() {
        // Mixed array — integers and strings — requesting Vec<u32> should fail
        let arr = vec![
            toml::Value::Integer(1),
            toml::Value::String("oops".into()),
            toml::Value::Integer(3),
        ];
        let params = make_params(&[("mixed", toml::Value::Array(arr))]);
        let result = params.get::<Vec<u32>>("mixed");
        assert!(result.is_err());
        // Error should mention element index
        assert!(result.unwrap_err().to_string().contains("[1]"));
    }

    #[test]
    fn battle_test_params_raw_access() {
        let params = make_params(&[
            ("port", toml::Value::String("/dev/ttyUSB0".into())),
            ("rate", toml::Value::Integer(100)),
        ]);
        assert!(params.raw("port").unwrap().is_str());
        assert!(params.raw("rate").unwrap().is_integer());
        assert!(params.raw("nonexistent").is_none());
    }

    #[test]
    fn battle_test_params_keys_iteration() {
        let params = make_params(&[
            ("alpha", toml::Value::Integer(1)),
            ("beta", toml::Value::Integer(2)),
            ("gamma", toml::Value::Integer(3)),
        ]);
        let mut keys: Vec<&str> = params.keys().collect();
        keys.sort();
        assert_eq!(keys, vec!["alpha", "beta", "gamma"]);
    }

    #[test]
    fn battle_test_params_many_params() {
        // 100 params in a single DriverParams
        let pairs: Vec<(String, toml::Value)> = (0..100)
            .map(|i| (format!("param_{}", i), toml::Value::Integer(i)))
            .collect();
        let ref_pairs: Vec<(&str, toml::Value)> =
            pairs.iter().map(|(k, v)| (k.as_str(), v.clone())).collect();
        let params = make_params(&ref_pairs);
        assert_eq!(params.len(), 100);
        for i in 0..100 {
            let key = format!("param_{}", i);
            assert_eq!(params.get::<i64>(&key).unwrap(), i);
        }
    }

    // ── Registry/Factory battle tests ────────────────────────────────

    // Counter for registry tests that need static globals (fn pointers can't capture)
    static FACTORY_ERROR_COUNTER: AtomicU32 = AtomicU32::new(0);

    struct ErrorStub;
    impl Node for ErrorStub {
        fn name(&self) -> &str {
            "error_stub"
        }
        fn tick(&mut self) {}
    }

    #[test]
    fn battle_test_registry_many_drivers() {
        // Register 100+ drivers and verify they can all be looked up
        for i in 0..120 {
            let name = format!("BattleMassDriver_{}", i);
            // We need to leak the name because register takes &str
            // but the factory is a fn pointer, not a closure
            registry::register(&name, |_params| Ok(Box::new(ErrorStub)));
        }
        for i in 0..120 {
            let name = format!("BattleMassDriver_{}", i);
            assert!(
                registry::lookup(&name).is_some(),
                "driver {} not found",
                name
            );
        }
    }

    #[test]
    fn battle_test_registry_overwrite_lookup_cycle() {
        // Register, overwrite, verify latest factory wins
        static V1_COUNT: AtomicU32 = AtomicU32::new(0);
        static V2_COUNT: AtomicU32 = AtomicU32::new(0);

        struct V1Stub;
        impl Node for V1Stub {
            fn name(&self) -> &str {
                "v1"
            }
            fn tick(&mut self) {
                V1_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }
        struct V2Stub;
        impl Node for V2Stub {
            fn name(&self) -> &str {
                "v2"
            }
            fn tick(&mut self) {
                V2_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }

        registry::register("OverwriteTestDriver", |_| Ok(Box::new(V1Stub)));
        registry::register("OverwriteTestDriver", |_| Ok(Box::new(V2Stub)));

        let factory = registry::lookup("OverwriteTestDriver").unwrap();
        let node = factory(&DriverParams::empty()).unwrap();
        // The second registration (V2) should win
        assert_eq!(node.name(), "v2");
    }

    #[test]
    fn battle_test_registry_factory_returns_error() {
        registry::register("ErrorFactory", |_params| {
            Err(crate::error::ConfigError::Other("factory failed on purpose".into()).into())
        });

        let factory = registry::lookup("ErrorFactory").unwrap();
        let result = factory(&DriverParams::empty());
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("factory failed"));
    }

    #[test]
    fn battle_test_registry_list_after_registrations() {
        let unique_name = "UniqueListTestDriver_12345";
        registry::register(unique_name, |_| Ok(Box::new(ErrorStub)));

        let names = registry::list_registered();
        assert!(
            names.contains(&unique_name.to_string()),
            "list should contain {}",
            unique_name
        );
    }

    #[test]
    fn battle_test_registry_case_sensitivity() {
        registry::register("CaseSensitive", |_| Ok(Box::new(ErrorStub)));

        assert!(registry::lookup("CaseSensitive").is_some());
        assert!(
            registry::lookup("casesensitive").is_none(),
            "lookup should be case-sensitive"
        );
        assert!(
            registry::lookup("CASESENSITIVE").is_none(),
            "lookup should be case-sensitive"
        );
        assert!(
            registry::lookup("caseSensitive").is_none(),
            "lookup should be case-sensitive"
        );
    }

    #[test]
    fn battle_test_registry_empty_name() {
        registry::register("", |_| Ok(Box::new(ErrorStub)));
        assert!(registry::lookup("").is_some());
    }

    #[test]
    fn battle_test_registry_unicode_name() {
        registry::register("ドライバー", |_| Ok(Box::new(ErrorStub)));
        assert!(registry::lookup("ドライバー").is_some());
        assert!(registry::lookup("driver").is_none());
    }

    #[test]
    fn battle_test_registry_name_with_special_chars() {
        registry::register("my-driver_v2.1", |_| Ok(Box::new(ErrorStub)));
        assert!(registry::lookup("my-driver_v2.1").is_some());
    }

    #[test]
    fn battle_test_registry_factory_receives_correct_params() {
        registry::register("ParamCheckFactory", |params| {
            let port: String = params.get("port")?;
            let rate: u32 = params.get("rate")?;
            assert_eq!(port, "/dev/ttyUSB0");
            assert_eq!(rate, 115200);
            Ok(Box::new(ErrorStub))
        });

        let mut map = HashMap::new();
        map.insert(
            "port".to_string(),
            toml::Value::String("/dev/ttyUSB0".into()),
        );
        map.insert("rate".to_string(), toml::Value::Integer(115200));
        let params = DriverParams::new(map);

        let factory = registry::lookup("ParamCheckFactory").unwrap();
        factory(&params).unwrap();
    }

    // ── HardwareSet battle tests ─────────────────────────────────────

    #[test]
    fn battle_test_hw_access_nonexistent_driver_terra() {
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let err = hw.dynamixel("nonexistent").unwrap_err().to_string();
        assert!(err.contains("nonexistent"));
        assert!(err.contains("arm"), "error should list available drivers");
    }

    #[test]
    fn battle_test_hw_access_nonexistent_driver_local() {
        let table = parse_toml_table(
            r#"
            [motor]
            node = "SomeDriver"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.local("nonexistent");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("nonexistent"));
    }

    #[test]
    fn battle_test_hw_access_nonexistent_driver_package() {
        let table = parse_toml_table(
            r#"
            [sensor]
            package = "horus-driver-x"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.package("nonexistent");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("nonexistent"));
    }

    #[test]
    fn battle_test_hw_type_safe_accessor_mismatch() {
        // Try to use dynamixel accessor on a package driver
        let table = parse_toml_table(
            r#"
            [sensor]
            package = "horus-driver-x"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.dynamixel("sensor").is_err());
        assert!(hw.rplidar("sensor").is_err());
        assert!(hw.i2c("sensor").is_err());
        assert!(hw.serial("sensor").is_err());
        assert!(hw.can("sensor").is_err());
    }

    #[test]
    fn battle_test_hw_type_safe_accessor_on_local_driver() {
        // Try to use terra accessors on a local driver
        let table = parse_toml_table(
            r#"
            [motor]
            node = "MyMotor"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.dynamixel("motor").is_err());
        assert!(hw.rplidar("motor").is_err());
    }

    #[test]
    fn battle_test_hw_type_safe_accessor_on_legacy_driver() {
        let table = parse_toml_table(r#"cam = "opencv""#);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.dynamixel("cam").is_err());
    }

    #[test]
    fn battle_test_hw_empty_set_operations() {
        let table = toml::value::Table::new();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.is_empty());
        assert_eq!(hw.len(), 0);
        assert!(hw.list().is_empty());
        assert!(!hw.has("anything"));
        assert!(hw.params("anything").is_none());
        assert!(hw.driver_type("anything").is_none());
        assert!(hw.topic_mapping("anything").is_none());
        assert!(hw.dynamixel("anything").is_err());
        assert!(hw.local("anything").is_err());
        assert!(hw.package("anything").is_err());
        assert!(hw.raw("anything").is_err());
        assert!(hw.node("anything").is_err());
    }

    #[test]
    fn battle_test_hw_list_many_drivers_sorted() {
        let mut toml_str = String::new();
        // Generate drivers with names that test sorting
        let names = [
            "zebra", "alpha", "middle", "beta", "omega", "gamma", "delta",
        ];
        for name in &names {
            toml_str.push_str(&format!("{} = true\n", name));
        }
        let table = parse_toml_table(&toml_str);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let list = hw.list();
        let mut sorted_names: Vec<&str> = names.to_vec();
        sorted_names.sort();
        assert_eq!(list, sorted_names);
    }

    #[test]
    fn battle_test_hw_contains_check_all() {
        // Note: legacy key=value drivers must come BEFORE table sections in TOML
        let table = parse_toml_table(
            r#"
            camera = "opencv"

            [arm]
            terra = "dynamixel"
            [sensor]
            package = "horus-driver-x"
            [motor]
            node = "MyMotor"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.has("arm"));
        assert!(hw.has("sensor"));
        assert!(hw.has("motor"));
        assert!(hw.has("camera"));
        assert!(!hw.has("nonexistent"));
    }

    #[test]
    fn battle_test_hw_raw_accessor_all_types() {
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            [sensor]
            package = "horus-driver-x"
            addr = "10.0.0.1"
            [motor]
            node = "MyMotor"
            rate = 100
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        // raw() works on any driver type
        let arm_handle = hw.raw("arm").unwrap();
        assert_eq!(arm_handle.terra_name(), Some("dynamixel"));
        assert_eq!(
            arm_handle.params().get::<String>("port").unwrap(),
            "/dev/ttyUSB0"
        );

        let sensor_handle = hw.raw("sensor").unwrap();
        assert!(sensor_handle.terra_name().is_none());
        assert_eq!(
            sensor_handle.params().get::<String>("addr").unwrap(),
            "10.0.0.1"
        );

        let motor_handle = hw.raw("motor").unwrap();
        assert!(motor_handle.terra_name().is_none());
        assert_eq!(motor_handle.params().get::<u32>("rate").unwrap(), 100);
    }

    #[test]
    fn battle_test_hw_raw_accessor_not_found() {
        let table = parse_toml_table(r#"arm = true"#);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let err = hw.raw("ghost").unwrap_err().to_string();
        assert!(err.contains("ghost"));
        assert!(err.contains("arm"), "should list available drivers");
    }

    #[test]
    fn battle_test_hw_node_accessor_legacy_errors() {
        let table = parse_toml_table(
            r#"
            camera = "opencv"
            gps = true
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        // .node() should fail for legacy drivers
        let result = hw.node("camera");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("legacy") || err.contains("Legacy"));
        assert!(hw.node("gps").is_err());
    }

    #[test]
    fn battle_test_hw_node_accessor_terra() {
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("arm").unwrap();
        // Terra stub node uses the driver name
        assert_eq!(node.name(), "arm");
    }

    #[test]
    fn battle_test_hw_node_accessor_local_registered() {
        registry::register("BattleNodeLocalTest", |_| {
            Ok(Box::new(SimpleStub))
        });
        let table = parse_toml_table(
            r#"
            [conveyor]
            node = "BattleNodeLocalTest"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("conveyor").unwrap();
        assert_eq!(node.name(), "simple_stub");
    }

    #[test]
    fn battle_test_hw_node_accessor_local_unregistered() {
        let table = parse_toml_table(
            r#"
            [conveyor]
            node = "NeverRegistered99999"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.node("conveyor");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("NeverRegistered99999"));
        assert!(err.contains("register_driver"));
    }

    #[test]
    fn battle_test_hw_node_not_found() {
        let table = parse_toml_table(r#"arm = true"#);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.node("ghost");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("ghost"));
    }

    #[test]
    fn battle_test_hw_all_typed_accessors() {
        // Verify every typed accessor works on the correct terra name
        let accessor_tests = [
            ("dynamixel", "dyn_driver"),
            ("rplidar", "rp_driver"),
            ("realsense", "rs_driver"),
            ("i2c", "i2c_driver"),
            ("serial", "serial_driver"),
            ("can", "can_driver"),
            ("gpio", "gpio_driver"),
            ("pwm", "pwm_driver"),
            ("usb", "usb_driver"),
            ("webcam", "webcam_driver"),
            ("input", "input_driver"),
            ("bluetooth", "bt_driver"),
            ("net", "net_driver"),
            ("ethercat", "ec_driver"),
            ("spi", "spi_driver"),
            ("adc", "adc_driver"),
        ];

        let mut toml_str = String::new();
        for (terra, name) in &accessor_tests {
            toml_str.push_str(&format!(
                "[{}]\nterra = \"{}\"\nport = \"/dev/test\"\n\n",
                name, terra
            ));
        }
        let table = parse_toml_table(&toml_str);
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        // Each accessor should succeed on its own terra type
        assert!(hw.dynamixel("dyn_driver").is_ok());
        assert!(hw.rplidar("rp_driver").is_ok());
        assert!(hw.realsense("rs_driver").is_ok());
        assert!(hw.i2c("i2c_driver").is_ok());
        assert!(hw.serial("serial_driver").is_ok());
        assert!(hw.can("can_driver").is_ok());
        assert!(hw.gpio("gpio_driver").is_ok());
        assert!(hw.pwm("pwm_driver").is_ok());
        assert!(hw.usb("usb_driver").is_ok());
        assert!(hw.webcam("webcam_driver").is_ok());
        assert!(hw.input("input_driver").is_ok());
        assert!(hw.bluetooth("bt_driver").is_ok());
        assert!(hw.net("net_driver").is_ok());
        assert!(hw.ethercat("ec_driver").is_ok());
        assert!(hw.spi("spi_driver").is_ok());
        assert!(hw.adc("adc_driver").is_ok());
    }

    #[test]
    fn battle_test_hw_topic_mapping_full_config() {
        let table = parse_toml_table(
            r#"
            [robot]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            topic = "robot/data"
            topic_state = "robot/state"
            topic_command = "robot/cmd"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("robot").unwrap();
        assert_eq!(mapping.topic.as_deref(), Some("robot/data"));
        assert_eq!(mapping.topic_state.as_deref(), Some("robot/state"));
        assert_eq!(mapping.topic_command.as_deref(), Some("robot/cmd"));
    }

    #[test]
    fn battle_test_hw_topic_mapping_partial() {
        let table = parse_toml_table(
            r#"
            [sensor]
            terra = "i2c"
            topic = "sensors/imu"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let mapping = hw.topic_mapping("sensor").unwrap();
        assert_eq!(mapping.topic.as_deref(), Some("sensors/imu"));
        assert!(mapping.topic_state.is_none());
        assert!(mapping.topic_command.is_none());
    }

    // ── Terra map battle tests ───────────────────────────────────────

    #[test]
    fn battle_test_terra_map_all_aliases() {
        use crate::drivers::terra_map;
        // Verify all alias pairs resolve to the same crate
        let alias_pairs = [
            ("ublox", "nmea"),
            ("mpu6050", "icm20689"),
            ("serial", "uart"),
            ("bluetooth", "ble"),
            ("input", "gamepad"),
            ("virtual", "mock"),
        ];
        for (a, b) in &alias_pairs {
            let info_a = terra_map::resolve(a).expect(&format!("{} should resolve", a));
            let info_b = terra_map::resolve(b).expect(&format!("{} should resolve", b));
            assert_eq!(
                info_a.crate_name, info_b.crate_name,
                "{} and {} should map to same crate",
                a, b
            );
        }
    }

    #[test]
    fn battle_test_terra_map_net_aliases() {
        use crate::drivers::terra_map;
        for name in &["net", "tcp", "udp"] {
            let info = terra_map::resolve(name).unwrap();
            assert_eq!(info.crate_name, "terra-net");
            assert_eq!(info.accessor, terra_map::DriverAccessor::Net);
        }
    }

    #[test]
    fn battle_test_terra_map_industrial_protocols() {
        use crate::drivers::terra_map;
        let info = terra_map::resolve("ethercat").unwrap();
        assert_eq!(info.crate_name, "terra-ethercat");
        assert_eq!(info.accessor, terra_map::DriverAccessor::EtherCat);

        let info = terra_map::resolve("ethernetip").unwrap();
        assert_eq!(info.crate_name, "terra-ethernetip");
        assert_eq!(info.accessor, terra_map::DriverAccessor::EthernetIp);

        let info = terra_map::resolve("profinet").unwrap();
        assert_eq!(info.crate_name, "terra-profinet");
        assert_eq!(info.accessor, terra_map::DriverAccessor::Profinet);
    }

    #[test]
    fn battle_test_terra_map_camera_drivers() {
        use crate::drivers::terra_map;
        let drivers = [
            ("realsense", "terra-realsense", terra_map::DriverAccessor::RealSense),
            ("webcam", "terra-webcam", terra_map::DriverAccessor::Webcam),
            ("zed", "terra-zed", terra_map::DriverAccessor::RealSense),
            ("oakd", "terra-oakd", terra_map::DriverAccessor::RealSense),
        ];
        for (name, expected_crate, expected_accessor) in &drivers {
            let info = terra_map::resolve(name).expect(&format!("{} should resolve", name));
            assert_eq!(info.crate_name, *expected_crate, "crate mismatch for {}", name);
            assert_eq!(info.accessor, *expected_accessor, "accessor mismatch for {}", name);
        }
    }

    #[test]
    fn battle_test_terra_map_unknown_names() {
        use crate::drivers::terra_map;
        let unknown = [
            "",
            "DYNAMIXEL",
            "Dynamixel",
            "ros2",
            "foobar",
            "dynamixel ",
            " dynamixel",
        ];
        for name in &unknown {
            assert!(
                terra_map::resolve(name).is_none(),
                "'{}' should not resolve",
                name
            );
        }
    }

    #[test]
    fn battle_test_terra_map_feature_present_or_none() {
        use crate::drivers::terra_map;
        // Drivers with features
        assert!(terra_map::resolve("dynamixel").unwrap().feature.is_some());
        assert!(terra_map::resolve("rplidar").unwrap().feature.is_some());
        assert!(terra_map::resolve("vesc").unwrap().feature.is_some());
        // Standalone crate drivers (no feature needed)
        assert!(terra_map::resolve("realsense").unwrap().feature.is_none());
        assert!(terra_map::resolve("webcam").unwrap().feature.is_none());
        assert!(terra_map::resolve("ethercat").unwrap().feature.is_none());
        assert!(terra_map::resolve("i2c").unwrap().feature.is_none());
    }

    // ── Scheduler integration battle tests ───────────────────────────

    #[test]
    fn battle_test_sched_many_driver_nodes() {
        // 25 driver nodes in a single scheduler
        let counters: Vec<Arc<AtomicU32>> = (0..25).map(|_| Arc::new(AtomicU32::new(0))).collect();

        let mut sched = Scheduler::new().deterministic(true);
        for (i, counter) in counters.iter().enumerate() {
            sched
                .add(StubDriverNode::new(
                    &format!("driver_{}", i),
                    counter.clone(),
                ))
                .order(i as u32)
                .build()
                .unwrap();
        }

        // Tick 5 times
        for _ in 0..5 {
            sched.tick_once().unwrap();
        }

        // All nodes should have ticked exactly 5 times
        for (i, counter) in counters.iter().enumerate() {
            assert_eq!(
                counter.load(Ordering::Relaxed),
                5,
                "driver_{} should have ticked 5 times",
                i
            );
        }
    }

    #[test]
    fn battle_test_sched_driver_node_ordering() {
        // Verify that nodes with different orders all tick per cycle
        let counter_first = Arc::new(AtomicU32::new(0));
        let counter_mid = Arc::new(AtomicU32::new(0));
        let counter_last = Arc::new(AtomicU32::new(0));

        let mut sched = Scheduler::new().deterministic(true);
        sched
            .add(StubDriverNode::new("first", counter_first.clone()))
            .order(0)
            .build()
            .unwrap();
        sched
            .add(StubDriverNode::new("mid", counter_mid.clone()))
            .order(50)
            .build()
            .unwrap();
        sched
            .add(StubDriverNode::new("last", counter_last.clone()))
            .order(100)
            .build()
            .unwrap();

        sched.tick_once().unwrap();

        assert_eq!(counter_first.load(Ordering::Relaxed), 1);
        assert_eq!(counter_mid.load(Ordering::Relaxed), 1);
        assert_eq!(counter_last.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn battle_test_sched_many_ticks() {
        // Tick 1000 times and verify accumulation
        let counter = Arc::new(AtomicU32::new(0));
        let mut sched = Scheduler::new().deterministic(true);
        sched
            .add(StubDriverNode::new("fast_ticker", counter.clone()))
            .build()
            .unwrap();

        for _ in 0..1000 {
            sched.tick_once().unwrap();
        }

        assert_eq!(counter.load(Ordering::Relaxed), 1000);
    }

    #[test]
    fn battle_test_sched_boxed_dyn_from_local_driver() {
        // This is the real workflow: HardwareSet::local() returns Box<dyn Node>
        static BOXED_COUNTER: AtomicU32 = AtomicU32::new(0);
        struct BoxedCounterStub;
        impl Node for BoxedCounterStub {
            fn name(&self) -> &str {
                "boxed_counter"
            }
            fn tick(&mut self) {
                BOXED_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }

        BOXED_COUNTER.store(0, Ordering::Relaxed);

        registry::register("BattleBoxedCounterDriver", |_| {
            Ok(Box::new(BoxedCounterStub))
        });

        let table = parse_toml_table(
            r#"
            [counter]
            node = "BattleBoxedCounterDriver"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.local("counter").unwrap();

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(node).build().unwrap();
        for _ in 0..10 {
            sched.tick_once().unwrap();
        }

        assert_eq!(BOXED_COUNTER.load(Ordering::Relaxed), 10);
    }

    #[test]
    fn battle_test_sched_multiple_local_drivers() {
        // Multiple different local drivers in one scheduler
        static MOTOR_COUNTER: AtomicU32 = AtomicU32::new(0);
        static SENSOR_COUNTER: AtomicU32 = AtomicU32::new(0);

        struct MotorStub;
        impl Node for MotorStub {
            fn name(&self) -> &str {
                "motor_node"
            }
            fn tick(&mut self) {
                MOTOR_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }
        struct SensorStub;
        impl Node for SensorStub {
            fn name(&self) -> &str {
                "sensor_node"
            }
            fn tick(&mut self) {
                SENSOR_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }

        MOTOR_COUNTER.store(0, Ordering::Relaxed);
        SENSOR_COUNTER.store(0, Ordering::Relaxed);

        registry::register("BattleMotorDriver", |_| Ok(Box::new(MotorStub)));
        registry::register("BattleSensorDriver", |_| Ok(Box::new(SensorStub)));

        let table = parse_toml_table(
            r#"
            [motor]
            node = "BattleMotorDriver"
            [sensor]
            node = "BattleSensorDriver"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let motor_node = hw.local("motor").unwrap();
        let sensor_node = hw.local("sensor").unwrap();

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(motor_node).order(0).build().unwrap();
        sched.add(sensor_node).order(1).build().unwrap();

        for _ in 0..5 {
            sched.tick_once().unwrap();
        }

        assert_eq!(MOTOR_COUNTER.load(Ordering::Relaxed), 5);
        assert_eq!(SENSOR_COUNTER.load(Ordering::Relaxed), 5);
    }

    #[test]
    fn battle_test_sched_terra_stub_node_ticks() {
        // .node() on terra driver returns a TerraStubNode which should tick fine
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("arm").unwrap();

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(node).build().unwrap();
        // Should not panic or error
        sched.tick_once().unwrap();
        sched.tick_once().unwrap();
    }

    // ── End-to-end: config → HardwareSet → params → registry ────────

    #[test]
    fn battle_test_e2e_full_workflow() {
        // Full end-to-end: define config, parse HardwareSet, register factory,
        // instantiate local driver, verify params, tick in scheduler
        static E2E_COUNTER: AtomicU32 = AtomicU32::new(0);
        struct E2EDriver {
            port: String,
            rate: u32,
        }
        impl Node for E2EDriver {
            fn name(&self) -> &str {
                "e2e_driver"
            }
            fn tick(&mut self) {
                E2E_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }

        E2E_COUNTER.store(0, Ordering::Relaxed);

        registry::register("E2ETestDriver", |params| {
            let port: String = params.get("port")?;
            let rate: u32 = params.get_or("rate", 100);
            Ok(Box::new(E2EDriver { port, rate }))
        });

        let table = parse_toml_table(
            r#"
            [actuator]
            node = "E2ETestDriver"
            port = "/dev/ttyUSB0"
            rate = 200
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        // Verify params are accessible before instantiation
        let params = hw.params("actuator").unwrap();
        assert_eq!(params.get::<String>("port").unwrap(), "/dev/ttyUSB0");
        assert_eq!(params.get::<u32>("rate").unwrap(), 200);

        // Instantiate and tick
        let node = hw.local("actuator").unwrap();
        assert_eq!(node.name(), "e2e_driver");

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(node).build().unwrap();
        for _ in 0..3 {
            sched.tick_once().unwrap();
        }
        assert_eq!(E2E_COUNTER.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn battle_test_e2e_mixed_driver_types_in_scheduler() {
        // Mix terra stub nodes and local registered nodes in the same scheduler
        static MIXED_LOCAL_COUNTER: AtomicU32 = AtomicU32::new(0);
        struct MixedLocalStub;
        impl Node for MixedLocalStub {
            fn name(&self) -> &str {
                "mixed_local"
            }
            fn tick(&mut self) {
                MIXED_LOCAL_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }

        MIXED_LOCAL_COUNTER.store(0, Ordering::Relaxed);
        registry::register("BattleMixedLocal", |_| Ok(Box::new(MixedLocalStub)));

        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"

            [custom]
            node = "BattleMixedLocal"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let terra_node = hw.node("arm").unwrap();
        let local_node = hw.local("custom").unwrap();

        let mut sched = Scheduler::new().deterministic(true);
        sched.add(terra_node).order(0).build().unwrap();
        sched.add(local_node).order(1).build().unwrap();

        for _ in 0..3 {
            sched.tick_once().unwrap();
        }

        assert_eq!(MIXED_LOCAL_COUNTER.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn battle_test_e2e_params_from_toml_to_factory() {
        // Verify that complex params flow correctly from TOML through HardwareSet
        // to the factory function
        registry::register("BattleParamFlowDriver", |params| {
            // Verify all param types arrive correctly
            let name: String = params.get("label")?;
            assert_eq!(name, "test_motor");
            let speed: u32 = params.get("max_speed")?;
            assert_eq!(speed, 1000);
            let gain: f64 = params.get("pid_gain")?;
            assert!((gain - 0.5).abs() < 1e-10);
            let enabled: bool = params.get("enabled")?;
            assert!(enabled);
            let ids: Vec<u8> = params.get("servo_ids")?;
            assert_eq!(ids, vec![1, 2, 3]);
            Ok(Box::new(SimpleStub))
        });

        let table = parse_toml_table(
            r#"
            [motor]
            node = "BattleParamFlowDriver"
            label = "test_motor"
            max_speed = 1000
            pid_gain = 0.5
            enabled = true
            servo_ids = [1, 2, 3]
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        hw.local("motor").unwrap(); // factory assertions will verify
    }

    #[test]
    fn battle_test_hw_package_accessor_on_local_errors() {
        // Using .package() on a local driver should error
        registry::register("BattlePkgLocalMismatch", |_| Ok(Box::new(SimpleStub)));
        let table = parse_toml_table(
            r#"
            [motor]
            node = "BattlePkgLocalMismatch"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.package("motor").is_err());
    }

    #[test]
    fn battle_test_hw_local_accessor_on_package_errors() {
        // Using .local() on a package driver should error
        let table = parse_toml_table(
            r#"
            [sensor]
            package = "horus-driver-x"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.local("sensor");
        assert!(result.is_err());
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected error"),
        };
        assert!(err.contains("sensor"));
    }

    #[test]
    fn battle_test_hw_local_accessor_on_terra_errors() {
        // Using .local() on a terra driver should error
        let table = parse_toml_table(
            r#"
            [arm]
            terra = "dynamixel"
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(hw.local("arm").is_err());
    }

    #[test]
    fn battle_test_config_single_driver_many_params() {
        // A driver with many params
        let mut toml_str = String::from("[big_driver]\nterra = \"serial\"\n");
        for i in 0..50 {
            toml_str.push_str(&format!("param_{} = {}\n", i, i));
        }
        let table = parse_toml_table(&toml_str);
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        let params = hw.params("big_driver").unwrap();
        assert_eq!(params.len(), 50);
        for i in 0..50 {
            assert_eq!(
                params.get::<i64>(&format!("param_{}", i)).unwrap(),
                i,
            );
        }
    }

    #[test]
    fn battle_test_params_clone() {
        // DriverParams derives Clone — verify it works
        let params = make_params(&[
            ("port", toml::Value::String("/dev/ttyUSB0".into())),
            ("rate", toml::Value::Integer(100)),
        ]);
        let cloned = params.clone();
        assert_eq!(cloned.get::<String>("port").unwrap(), "/dev/ttyUSB0");
        assert_eq!(cloned.get::<u32>("rate").unwrap(), 100);
        assert_eq!(cloned.len(), params.len());
    }

    #[test]
    fn battle_test_hw_multiple_terra_drivers_same_type() {
        // Multiple drivers of the same terra type (e.g., two dynamixel buses)
        let table = parse_toml_table(
            r#"
            [left_arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB0"
            baudrate = 1000000

            [right_arm]
            terra = "dynamixel"
            port = "/dev/ttyUSB1"
            baudrate = 1000000
        "#,
        );
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 2);

        let left = hw.dynamixel("left_arm").unwrap();
        assert_eq!(left.params().get::<String>("port").unwrap(), "/dev/ttyUSB0");

        let right = hw.dynamixel("right_arm").unwrap();
        assert_eq!(
            right.params().get::<String>("port").unwrap(),
            "/dev/ttyUSB1"
        );
    }

    #[test]
    fn battle_test_hw_driver_type_discriminants() {
        // Note: legacy key=value must come before table sections in TOML
        let table = parse_toml_table(
            r#"
            leg = true

            [t]
            terra = "dynamixel"
            [p]
            package = "horus-driver-x"
            [l]
            node = "MyDriver"
        "#,
        );
        let hw = HardwareSet::from_toml_table(&table).unwrap();

        // Verify that DriverType::Terra, Package, Local, Legacy are all distinct
        assert!(matches!(hw.driver_type("t"), Some(crate::drivers::DriverType::Terra(_))));
        assert!(matches!(hw.driver_type("p"), Some(crate::drivers::DriverType::Package(_))));
        assert!(matches!(hw.driver_type("l"), Some(crate::drivers::DriverType::Local(_))));
        assert!(matches!(hw.driver_type("leg"), Some(crate::drivers::DriverType::Legacy)));
    }

    // ── Topic name alignment tests ──────────────────────────────────────

    #[test]
    fn resolve_topic_name_follows_convention() {
        let toml_str = r#"
[front_lidar]
terra = "rplidar"
port = "/dev/ttyUSB0"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        hw.set_robot_name("turtlebot");

        let topic = hw.resolve_topic_name("front_lidar", "scan");
        assert_eq!(topic, "turtlebot.front_lidar.scan");

        // This should match what sim3d produces for the same robot+sensor
        assert_eq!(
            topic,
            horus_library::topic_convention::sensor_topic("turtlebot", "front_lidar", "scan")
        );
    }

    #[test]
    fn resolve_topic_name_explicit_mapping_overrides_convention() {
        let toml_str = r#"
[arm]
terra = "dynamixel"
topic_state = "robot.arm.joint_states"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        hw.set_robot_name("turtlebot");

        // Explicit topic_state takes priority over convention
        let topic = hw.resolve_topic_name("arm", "joint_state");
        assert_eq!(topic, "robot.arm.joint_states");
    }

    #[test]
    fn resolve_topic_name_default_robot_name() {
        let toml_str = r#"
[imu]
terra = "bno055"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        // No set_robot_name — should use default "robot"
        let topic = hw.resolve_topic_name("imu", "imu");
        assert_eq!(topic, "robot.imu.imu");
    }

    #[test]
    fn resolve_topic_name_sim3d_driver_same_as_real() {
        let toml_str = r#"
[front_lidar]
simulated = true
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        hw.set_robot_name("turtlebot");

        // Sim3d driver resolves to same topic name as a real driver would
        let topic = hw.resolve_topic_name("front_lidar", "scan");
        assert_eq!(topic, "turtlebot.front_lidar.scan");
    }

    #[test]
    fn resolve_topic_name_all_sensor_types_aligned() {
        let toml_str = r#"
[lidar]
terra = "rplidar"
[imu]
terra = "bno055"
[camera]
terra = "realsense"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        hw.set_robot_name("bot");

        let pairs = [
            ("lidar", "scan", "bot.lidar.scan"),
            ("imu", "imu", "bot.imu.imu"),
            ("camera", "image", "bot.camera.image"),
        ];
        for (name, suffix, expected) in &pairs {
            assert_eq!(
                hw.resolve_topic_name(name, suffix),
                *expected,
                "Alignment failed for {}.{}",
                name,
                suffix
            );
        }
    }

    // ── New DriverType variant tests ─────────────────────────────────

    #[test]
    fn parse_crates_io_driver() {
        let toml_str = r#"
[lidar]
crate = "rplidar-driver"
source = "crates-io"
adapter = "scan"
port = "/dev/ttyUSB0"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::CratesIo(s)) if s == "rplidar-driver"));
        // adapter and port should be in params
        let params = hw.params("lidar").unwrap();
        assert_eq!(params.get_or::<String>("adapter", String::new()), "scan");
        assert_eq!(params.get_or::<String>("port", String::new()), "/dev/ttyUSB0");
    }

    #[test]
    fn parse_pypi_driver() {
        let toml_str = r#"
[imu]
pip = "adafruit-bno055"
adapter = "imu"
bus = 1
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(hw.driver_type("imu"), Some(crate::drivers::DriverType::PyPI(s)) if s == "adafruit-bno055"));
    }

    #[test]
    fn parse_exec_driver() {
        let toml_str = r#"
[camera]
exec = "./realsense_bridge"
args = ["--serial", "12345"]
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(hw.driver_type("camera"), Some(crate::drivers::DriverType::Exec(_))));
        let params = hw.params("camera").unwrap();
        // args should be in params
        assert!(params.has("args"));
    }

    #[test]
    fn parse_sim3d_driver() {
        let toml_str = r#"
[lidar]
simulated = true
noise = 0.01
rate = 10
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Simulated)));
        let params = hw.params("lidar").unwrap();
        assert!((params.get_or::<f64>("noise", 0.0) - 0.01).abs() < 1e-6);
    }

    #[test]
    fn parse_mixed_all_driver_types() {
        let toml_str = r#"
[t]
terra = "dynamixel"
[p]
package = "horus-driver-ati"
[l]
node = "ConveyorDriver"
[c]
crate = "rplidar-driver"
[py]
pip = "adafruit-bno055"
[ex]
exec = "./bridge"
[s]
simulated = true
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let hw = HardwareSet::from_toml_table(&table).unwrap();
        assert_eq!(hw.len(), 7);
        assert!(matches!(hw.driver_type("t"), Some(crate::drivers::DriverType::Terra(_))));
        assert!(matches!(hw.driver_type("p"), Some(crate::drivers::DriverType::Package(_))));
        assert!(matches!(hw.driver_type("l"), Some(crate::drivers::DriverType::Local(_))));
        assert!(matches!(hw.driver_type("c"), Some(crate::drivers::DriverType::CratesIo(_))));
        assert!(matches!(hw.driver_type("py"), Some(crate::drivers::DriverType::PyPI(_))));
        assert!(matches!(hw.driver_type("ex"), Some(crate::drivers::DriverType::Exec(_))));
        assert!(matches!(hw.driver_type("s"), Some(crate::drivers::DriverType::Simulated)));
    }

    #[test]
    fn sim3d_driver_returns_stub_node() {
        let toml_str = r#"
[lidar]
simulated = true
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let node = hw.node("lidar").unwrap();
        assert_eq!(node.name(), "lidar");
    }

    #[test]
    fn crates_io_driver_errors_on_node_instantiation() {
        let toml_str = r#"
[lidar]
crate = "rplidar-driver"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();
        let result = hw.node("lidar");
        assert!(result.is_err());
    }

    // ── Sim override tests ──────────────────────────────────────────

    #[test]
    fn apply_sim_overrides_replaces_driver_type() {
        let toml_str = r#"
[lidar]
terra = "rplidar"
port = "/dev/ttyUSB0"
[imu]
terra = "bno055"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        // Before override
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Terra(_))));
        assert!(matches!(hw.driver_type("imu"), Some(crate::drivers::DriverType::Terra(_))));

        // Apply sim override for lidar only
        let mut overrides = std::collections::BTreeMap::new();
        overrides.insert("lidar".to_string(), crate::drivers::DriverType::Simulated);
        let count = hw.apply_sim_overrides(&overrides);

        assert_eq!(count, 1);
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Simulated)));
        assert!(matches!(hw.driver_type("imu"), Some(crate::drivers::DriverType::Terra(_)))); // unchanged
    }

    #[test]
    fn apply_sim_overrides_inserts_new_driver() {
        let toml_str = r#"
[lidar]
terra = "rplidar"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        assert_eq!(hw.len(), 1);

        let mut overrides = std::collections::BTreeMap::new();
        overrides.insert("virtual_camera".to_string(), crate::drivers::DriverType::Simulated);
        let count = hw.apply_sim_overrides(&overrides);

        assert_eq!(count, 1);
        assert_eq!(hw.len(), 2);
        assert!(matches!(hw.driver_type("virtual_camera"), Some(crate::drivers::DriverType::Simulated)));
    }

    #[test]
    fn set_robot_name_changes_topic_resolution() {
        let toml_str = r#"
[lidar]
terra = "rplidar"
"#;
        let table: toml::value::Table = toml::from_str(toml_str).unwrap();
        let mut hw = HardwareSet::from_toml_table(&table).unwrap();

        assert_eq!(hw.robot_name(), "robot"); // default
        assert_eq!(hw.resolve_topic_name("lidar", "scan"), "robot.lidar.scan");

        hw.set_robot_name("turtlebot");
        assert_eq!(hw.robot_name(), "turtlebot");
        assert_eq!(hw.resolve_topic_name("lidar", "scan"), "turtlebot.lidar.scan");
    }

    // ── load_from() + sim override integration tests ───────────────────
    //
    // These test the sim-mode override path by calling the lower-level
    // HardwareSet methods directly, avoiding process-wide env vars that
    // would be flaky under parallel `cargo test`.

    #[test]
    fn test_drivers_load_with_sim_mode_all() {
        // Simulate what load_from() does when HORUS_SIM_MODE=1:
        // parse [drivers], then apply all [sim-drivers] entries.
        let drivers_table = parse_toml_table(
            r#"
            [lidar]
            terra = "rplidar"
            port = "/dev/ttyUSB0"

            [imu]
            terra = "bno055"
            bus = "i2c-1"
        "#,
        );
        let sim_table = parse_toml_table(
            r#"
            [lidar]
            simulated = true
            noise = 0.01

            [imu]
            simulated = true
            noise = 0.005
        "#,
        );

        let mut hw = HardwareSet::from_toml_table(&drivers_table).unwrap();

        // Before sim override: both are Terra
        assert!(matches!(
            hw.driver_type("lidar"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "rplidar"
        ));
        assert!(matches!(
            hw.driver_type("imu"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "bno055"
        ));

        // Apply full sim override (no selective filtering)
        let count = hw.apply_sim_overrides_from_toml(&sim_table);
        assert_eq!(count, 2);

        // After override: both are Simulated
        assert!(matches!(
            hw.driver_type("lidar"),
            Some(crate::drivers::DriverType::Simulated)
        ));
        assert!(matches!(
            hw.driver_type("imu"),
            Some(crate::drivers::DriverType::Simulated)
        ));
    }

    #[test]
    fn test_drivers_load_with_sim_mode_selective() {
        // Simulate HORUS_SIM_TARGETS=lidar: only override lidar, leave imu as Terra.
        let drivers_table = parse_toml_table(
            r#"
            [lidar]
            terra = "rplidar"
            port = "/dev/ttyUSB0"

            [imu]
            terra = "bno055"
            bus = "i2c-1"
        "#,
        );
        let full_sim_table = parse_toml_table(
            r#"
            [lidar]
            simulated = true
            noise = 0.01

            [imu]
            simulated = true
            noise = 0.005
        "#,
        );

        let mut hw = HardwareSet::from_toml_table(&drivers_table).unwrap();

        // Selective filtering: only keep entries matching targets (like load_from does)
        let targets = vec!["lidar".to_string()];
        let filtered_sim_table: toml::value::Table = full_sim_table
            .iter()
            .filter(|(k, _)| targets.contains(k))
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        let count = hw.apply_sim_overrides_from_toml(&filtered_sim_table);
        assert_eq!(count, 1);

        // lidar is now Simulated
        assert!(matches!(
            hw.driver_type("lidar"),
            Some(crate::drivers::DriverType::Simulated)
        ));
        // imu stays Terra
        assert!(matches!(
            hw.driver_type("imu"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "bno055"
        ));
    }

    #[test]
    fn test_drivers_load_reads_robot_name() {
        // Write a temp horus.toml with [robot] name and [drivers], call load_from()
        let dir = tempfile::tempdir().unwrap();
        let toml_path = dir.path().join("horus.toml");
        std::fs::write(
            &toml_path,
            r#"
[robot]
name = "turtlebot"

[drivers.lidar]
terra = "rplidar"
port = "/dev/ttyUSB0"
"#,
        )
        .unwrap();

        let hw = crate::drivers::load_from(&toml_path).unwrap();
        assert_eq!(hw.robot_name(), "turtlebot");
        assert!(hw.has("lidar"));
    }

    #[test]
    fn test_drivers_load_no_sim_mode_unchanged() {
        // Without any sim overrides applied, drivers stay as their original type.
        let drivers_table = parse_toml_table(
            r#"
            [lidar]
            terra = "rplidar"
            port = "/dev/ttyUSB0"

            [imu]
            terra = "bno055"
            bus = "i2c-1"

            [camera]
            package = "horus-driver-realsense"
            serial = "12345"
        "#,
        );

        let hw = HardwareSet::from_toml_table(&drivers_table).unwrap();

        // No sim overrides applied -- all stay as original types
        assert!(matches!(
            hw.driver_type("lidar"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "rplidar"
        ));
        assert!(matches!(
            hw.driver_type("imu"),
            Some(crate::drivers::DriverType::Terra(t)) if t == "bno055"
        ));
        assert!(matches!(
            hw.driver_type("camera"),
            Some(crate::drivers::DriverType::Package(p)) if p == "horus-driver-realsense"
        ));
    }

    #[test]
    fn test_resolve_topic_name_with_loaded_robot_name() {
        // Write a temp horus.toml with [robot] name and [drivers], call load_from(),
        // then verify resolve_topic_name uses the loaded robot name.
        let dir = tempfile::tempdir().unwrap();
        let toml_path = dir.path().join("horus.toml");
        std::fs::write(
            &toml_path,
            r#"
[robot]
name = "bot"

[drivers.lidar]
terra = "rplidar"
port = "/dev/ttyUSB0"
"#,
        )
        .unwrap();

        let hw = crate::drivers::load_from(&toml_path).unwrap();
        assert_eq!(hw.robot_name(), "bot");
        assert_eq!(hw.resolve_topic_name("lidar", "scan"), "bot.lidar.scan");
    }

    // ── End-to-end: env var sim mode ────────────────────────────────
    // NOTE: These tests set/read process-global HORUS_SIM_MODE env var.
    // They MUST run with --test-threads=1 or be isolated. A shared mutex
    // ensures they don't interleave with each other.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn test_load_from_applies_sim_overrides_via_env_var() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // This tests the FULL --sim pipeline at horus_core level:
        // horus run --sim → sets HORUS_SIM_MODE=1 → drivers::load_from() reads it
        // → applies [sim-drivers] → driver types become Simulated

        let dir = tempfile::tempdir().unwrap();
        let toml_path = dir.path().join("horus.toml");
        std::fs::write(&toml_path, r#"
[robot]
name = "testbot"

[drivers.lidar]
terra = "rplidar"
port = "/dev/ttyUSB0"

[drivers.imu]
terra = "bno055"

[sim-drivers]
lidar = { simulated = true, noise = 0.01 }
imu = { simulated = true }
"#).unwrap();

        // Set env var (what horus run --sim does)
        // SAFETY: This test must clean up. Use a unique scope.
        std::env::set_var("HORUS_SIM_MODE", "1");

        let hw = crate::drivers::load_from(&toml_path).unwrap();

        // Clean up IMMEDIATELY before any assertions can panic
        std::env::remove_var("HORUS_SIM_MODE");
        std::env::remove_var("HORUS_SIM_TARGETS");

        // Both drivers should be overridden to Simulated
        assert!(
            matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Simulated)),
            "lidar should be Simulated, got {:?}", hw.driver_type("lidar")
        );
        assert!(
            matches!(hw.driver_type("imu"), Some(crate::drivers::DriverType::Simulated)),
            "imu should be Simulated, got {:?}", hw.driver_type("imu")
        );

        // Robot name should still be read
        assert_eq!(hw.robot_name(), "testbot");

        // Topic resolution should work
        assert_eq!(hw.resolve_topic_name("lidar", "scan"), "testbot.lidar.scan");
    }

    #[test]
    fn test_load_from_selective_sim_via_env_var() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let toml_path = dir.path().join("horus.toml");
        std::fs::write(&toml_path, r#"
[drivers.lidar]
terra = "rplidar"

[drivers.imu]
terra = "bno055"

[sim-drivers]
lidar = { simulated = true }
imu = { simulated = true }
"#).unwrap();

        // Selective: only simulate lidar
        std::env::set_var("HORUS_SIM_MODE", "1");
        std::env::set_var("HORUS_SIM_TARGETS", "lidar");

        let hw = crate::drivers::load_from(&toml_path).unwrap();

        std::env::remove_var("HORUS_SIM_MODE");
        std::env::remove_var("HORUS_SIM_TARGETS");

        // Only lidar should be Simulated
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Simulated)));
        // IMU should stay Terra
        assert!(matches!(hw.driver_type("imu"), Some(crate::drivers::DriverType::Terra(_))));
    }

    #[test]
    fn test_load_from_no_sim_mode_leaves_drivers_real() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let toml_path = dir.path().join("horus.toml");
        std::fs::write(&toml_path, r#"
[drivers.lidar]
terra = "rplidar"

[sim-drivers]
lidar = { simulated = true }
"#).unwrap();

        // Ensure env var is NOT set
        std::env::remove_var("HORUS_SIM_MODE");

        let hw = crate::drivers::load_from(&toml_path).unwrap();

        // Without HORUS_SIM_MODE, driver stays real
        assert!(matches!(hw.driver_type("lidar"), Some(crate::drivers::DriverType::Terra(_))));
    }
}
