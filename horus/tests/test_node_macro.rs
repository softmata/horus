//! Integration tests for the `node!` macro.
//!
//! These tests verify that the macro correctly generates:
//! - Struct definitions with Topic and data fields
//! - `new()` constructor
//! - `Node` trait implementation (name, init, tick, shutdown)
//! - `Default` trait implementation
//! - publishers() / subscribers() metadata
//! - Snake_case naming from CamelCase struct names
//! - Custom name override via `name:` section
//! - Rate specification via `rate` section

use horus::prelude::*;

// ============================================================================
// 1. Minimal node: just a name and empty tick
// ============================================================================

node! {
    MinimalNode {
        tick {
            // Empty tick — valid for a node that only exists for lifecycle
        }
    }
}

#[test]
fn test_minimal_node_compiles_and_implements_node() {
    let mut node = MinimalNode::new();
    // Node trait: name() defaults to snake_case of struct name
    assert_eq!(node.name(), "minimal_node");
    // init should succeed (default impl)
    assert!(node.init().is_ok());
    // tick should not panic
    node.tick();
    // shutdown should succeed
    assert!(node.shutdown().is_ok());
    // No publishers or subscribers
    assert!(node.publishers().is_empty());
    assert!(node.subscribers().is_empty());
}

#[test]
fn test_minimal_node_default_trait() {
    let node = MinimalNode::default();
    assert_eq!(node.name(), "minimal_node");
}

// ============================================================================
// 2. Node with pub/sub Topic fields
// ============================================================================

node! {
    SensorFusionNode {
        pub {
            fused_output: f64 -> "sensor_fusion/output",
        }

        sub {
            imu_input: f64 -> "sensor_fusion/imu",
            gps_input: f64 -> "sensor_fusion/gps",
        }

        tick {
            // Read from subscribers, fuse, publish
            if let Some(imu) = self.imu_input.recv() {
                self.fused_output.send(imu);
            }
        }
    }
}

#[test]
fn test_node_with_pub_sub_topics() {
    let node = SensorFusionNode::new();

    // Verify publisher metadata
    let pubs = node.publishers();
    assert_eq!(pubs.len(), 1);
    assert_eq!(pubs[0].topic_name, "sensor_fusion/output");

    // Verify subscriber metadata
    let subs = node.subscribers();
    assert_eq!(subs.len(), 2);
    assert_eq!(subs[0].topic_name, "sensor_fusion/imu");
    assert_eq!(subs[1].topic_name, "sensor_fusion/gps");
}

#[test]
fn test_node_pub_sub_topics_functional() {
    let mut node = SensorFusionNode::new();

    // Publisher can send without panic
    node.fused_output.send(42.0);

    // Subscribers can recv (returns None when no data sent to that topic yet in this context)
    let _val = node.imu_input.recv();

    // Tick should not panic even with no data
    node.tick();
}

// ============================================================================
// 3. Node with data fields
// ============================================================================

node! {
    CounterNode {
        data {
            count: u32 = 0,
            max_count: u32 = 100,
            label: String = String::from("counter"),
        }

        tick {
            if self.count < self.max_count {
                self.count += 1;
            }
        }
    }
}

#[test]
fn test_node_with_data_fields() {
    let mut node = CounterNode::new();

    // Verify default values
    assert_eq!(node.count, 0);
    assert_eq!(node.max_count, 100);
    assert_eq!(node.label, "counter");

    // Tick should increment count
    node.tick();
    assert_eq!(node.count, 1);

    // Tick multiple times
    for _ in 0..50 {
        node.tick();
    }
    assert_eq!(node.count, 51);
}

#[test]
fn test_node_data_respects_max() {
    let mut node = CounterNode::new();
    node.max_count = 5;

    for _ in 0..20 {
        node.tick();
    }
    // Should stop at max_count
    assert_eq!(node.count, 5);
}

// ============================================================================
// 4. Node with init and shutdown hooks
// ============================================================================

node! {
    LifecycleNode {
        data {
            initialized: bool = false,
            shut_down: bool = false,
            tick_count: u32 = 0,
        }

        init {
            self.initialized = true;
            Ok(())
        }

        tick {
            self.tick_count += 1;
        }

        shutdown {
            self.shut_down = true;
            Ok(())
        }
    }
}

#[test]
fn test_node_init_shutdown_hooks() {
    let mut node = LifecycleNode::new();

    // Before init
    assert!(!node.initialized);
    assert!(!node.shut_down);
    assert_eq!(node.tick_count, 0);

    // After init
    node.init().unwrap();
    assert!(node.initialized);

    // After ticking
    node.tick();
    node.tick();
    assert_eq!(node.tick_count, 2);

    // After shutdown
    node.shutdown().unwrap();
    assert!(node.shut_down);
}

// ============================================================================
// 5. Snake_case naming: CamelCase struct name → snake_case node name
// ============================================================================

node! {
    MyRobotArmController {
        tick {
            // Multi-word CamelCase should become snake_case
        }
    }
}

#[test]
fn test_snake_case_naming() {
    let node = MyRobotArmController::new();
    assert_eq!(node.name(), "my_robot_arm_controller");
}

// ============================================================================
// 6. Custom name override via name: section
// ============================================================================

node! {
    FlightControllerNode {
        name: "flight_controller",

        tick {
            // Custom name overrides auto-generated snake_case
        }
    }
}

#[test]
fn test_custom_name_override() {
    let node = FlightControllerNode::new();
    // Should use the explicit name, not "flight_controller_node"
    assert_eq!(node.name(), "flight_controller");
}

// ============================================================================
// 7. Node with rate specification
// ============================================================================

node! {
    HighFreqNode {
        rate 100.0

        tick {
            // 100Hz update rate
        }
    }
}

#[test]
fn test_node_rate_specification() {
    let node = HighFreqNode::new();
    assert_eq!(node.rate_hz(), Some(100.0));
}

// ============================================================================
// 8. Node without rate has None rate_hz
// ============================================================================

#[test]
fn test_node_without_rate_returns_none() {
    let node = MinimalNode::new();
    assert_eq!(node.rate_hz(), None);
}

// ============================================================================
// 9. Node with impl block for additional methods
// ============================================================================

node! {
    UtilityNode {
        data {
            value: f64 = 0.0,
        }

        tick {
            self.value = self.clamp_value(self.value + 1.0);
        }

        impl {
            fn clamp_value(&self, v: f64) -> f64 {
                v.min(10.0).max(0.0)
            }

            fn reset(&mut self) {
                self.value = 0.0;
            }
        }
    }
}

#[test]
fn test_node_with_impl_methods() {
    let mut node = UtilityNode::new();

    // Custom methods work
    assert_eq!(node.clamp_value(15.0), 10.0);
    assert_eq!(node.clamp_value(-5.0), 0.0);
    assert_eq!(node.clamp_value(5.0), 5.0);

    // Tick uses the custom method
    for _ in 0..20 {
        node.tick();
    }
    // Should be clamped at 10.0
    assert_eq!(node.value, 10.0);

    // Reset method
    node.reset();
    assert_eq!(node.value, 0.0);
}

// ============================================================================
// 10. Full-featured robotics node: pub + sub + data + init + shutdown + impl
// ============================================================================

node! {
    NavigationNode {
        pub {
            cmd_vel: f64 -> "nav/cmd_vel",
        }

        sub {
            goal: f64 -> "nav/goal",
            odom: f64 -> "nav/odom",
        }

        data {
            position: f64 = 0.0,
            target: f64 = 0.0,
            active: bool = false,
        }

        init {
            self.active = true;
            Ok(())
        }

        tick {
            if !self.active {
                return;
            }
            if let Some(goal) = self.goal.recv() {
                self.target = goal;
            }
            if let Some(pos) = self.odom.recv() {
                self.position = pos;
            }
            let error = self.target - self.position;
            let velocity = self.compute_velocity(error);
            self.cmd_vel.send(velocity);
        }

        shutdown {
            self.active = false;
            // Send zero velocity on shutdown (safety!)
            self.cmd_vel.send(0.0);
            Ok(())
        }

        impl {
            fn compute_velocity(&self, error: f64) -> f64 {
                // Simple P-controller
                let kp = 0.5;
                (kp * error).clamp(-1.0, 1.0)
            }
        }
    }
}

#[test]
fn test_full_featured_navigation_node() {
    let mut node = NavigationNode::new();

    // Auto-generated name
    assert_eq!(node.name(), "navigation_node");

    // Init sets active
    node.init().unwrap();
    assert!(node.active);

    // Publisher/subscriber metadata
    assert_eq!(node.publishers().len(), 1);
    assert_eq!(node.publishers()[0].topic_name, "nav/cmd_vel");
    assert_eq!(node.subscribers().len(), 2);

    // Custom method works
    assert_eq!(node.compute_velocity(2.0), 1.0); // clamped
    assert_eq!(node.compute_velocity(-0.4), -0.2);

    // Tick doesn't panic
    node.tick();

    // Shutdown deactivates
    node.shutdown().unwrap();
    assert!(!node.active);
}

// ============================================================================
// 11. Node is Send (required by Node: Send bound)
// ============================================================================

#[test]
fn test_node_is_send() {
    fn assert_send<T: Send>() {}
    assert_send::<MinimalNode>();
    assert_send::<CounterNode>();
    assert_send::<NavigationNode>();
}
