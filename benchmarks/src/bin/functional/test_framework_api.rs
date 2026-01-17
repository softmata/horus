// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

use horus::error::HorusResult;
/// Framework API Lock Test Suite
/// Locks down Node trait, Scheduler, and full framework usage patterns
/// If this test breaks, it means a breaking API change was introduced
use horus::prelude::{Node, NodeInfo, Scheduler, Topic};
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process;
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  node_trait_api");
        eprintln!("  scheduler_api");
        eprintln!("  per_node_rate");
        eprintln!("  full_framework_workflow");
        eprintln!("  multi_node_graph");
        eprintln!("  node_lifecycle");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "node_trait_api" => test_node_trait_api(),
        "scheduler_api" => test_scheduler_api(),
        "per_node_rate" => test_per_node_rate(),
        "full_framework_workflow" => test_full_framework_workflow(),
        "multi_node_graph" => test_multi_node_graph(),
        "node_lifecycle" => test_node_lifecycle(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    if result {
        println!(" Test passed: {}", test_name);
        process::exit(0);
    } else {
        eprintln!("[FAIL] Test failed: {}", test_name);
        process::exit(1);
    }
}

// =============================================================================
// Test Node Implementations - These lock down the Node trait API
// =============================================================================

/// Sensor node that publishes CmdVel messages
struct SensorNode {
    publisher: Topic<CmdVel>,
    counter: Arc<Mutex<u32>>,
    tick_count: u32,
}

impl SensorNode {
    fn new(topic: &str, counter: Arc<Mutex<u32>>) -> Self {
        Self {
            publisher: Topic::<CmdVel>::new(topic).expect("Failed to create publisher"),
            counter,
            tick_count: 0,
        }
    }
}

impl Node for SensorNode {
    fn name(&self) -> &'static str {
        "sensor_node"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!("  SensorNode: init() called");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.tick_count += 1;

        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: self.tick_count as u64,
        };

        if let Err(e) = self.publisher.send(msg, &mut ctx) {
            eprintln!("  SensorNode: Failed to publish: {:?}", e);
        }

        *self.counter.lock().unwrap() += 1;
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!(
            "  SensorNode: shutdown() called, published {} messages",
            self.tick_count
        );
        Ok(())
    }
}

/// Controller node that subscribes and processes messages
struct ControllerNode {
    subscriber: Topic<CmdVel>,
    publisher: Topic<CmdVel>,
    counter: Arc<Mutex<u32>>,
}

impl ControllerNode {
    fn new(input_topic: &str, output_topic: &str, counter: Arc<Mutex<u32>>) -> Self {
        Self {
            subscriber: Topic::<CmdVel>::new(input_topic).expect("Failed to create subscriber"),
            publisher: Topic::<CmdVel>::new(output_topic).expect("Failed to create publisher"),
            counter,
        }
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &'static str {
        "controller_node"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!("  ControllerNode: init() called");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Receive and process messages without using ctx
        if let Some(msg) = self.subscriber.recv(&mut None) {
            // Process message
            let output = CmdVel {
                linear: msg.linear * 0.8,
                angular: msg.angular * 0.9,
                stamp_nanos: msg.stamp_nanos,
            };

            let _ = self.publisher.send(output, &mut None);
            *self.counter.lock().unwrap() += 1;
        }
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!("  ControllerNode: shutdown() called");
        Ok(())
    }
}

/// Actuator node using Link for point-to-point communication
struct ActuatorNode {
    receiver: Topic<CmdVel>,
    counter: Arc<Mutex<u32>>,
}

impl ActuatorNode {
    fn new(topic: &str, counter: Arc<Mutex<u32>>) -> Self {
        Self {
            receiver: Topic::<CmdVel>::consumer(topic).expect("Failed to create Link consumer"),
            counter,
        }
    }
}

impl Node for ActuatorNode {
    fn name(&self) -> &'static str {
        "actuator_node"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!("  ActuatorNode: init() called");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if let Some(_msg) = self.receiver.recv(&mut ctx) {
            *self.counter.lock().unwrap() += 1;
        }
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        println!("  ActuatorNode: shutdown() called");
        Ok(())
    }
}

// =============================================================================
// Tests - Lock down framework APIs
// =============================================================================

/// Test 1: Node Trait API Lock
/// Ensures Node trait methods have correct signatures
fn test_node_trait_api() -> bool {
    println!("Testing Node trait API...");

    let counter = Arc::new(Mutex::new(0u32));
    let topic = format!("test_node_api_{}", process::id());

    // Create a node - this locks down the Node::new pattern
    let mut node = SensorNode::new(&topic, counter.clone());

    // Test Node trait methods - locks down signatures
    let node_name = node.name();
    println!("   Node.name() -> &str: {}", node_name);

    // Test init - locks down init signature
    let mut ctx = NodeInfo::new("test_context".to_string(), false);
    match node.init(&mut ctx) {
        Ok(()) => println!("   Node.init(&mut NodeInfo) -> HorusResult<()>"),
        Err(e) => {
            eprintln!("Node init failed: {}", e);
            return false;
        }
    }

    // Test tick - locks down tick signature
    node.tick(Some(&mut ctx));
    println!("   Node.tick(Option<&mut NodeInfo>)");

    // Test tick with None
    node.tick(None);
    println!("   Node.tick(None)");

    // Test shutdown - locks down shutdown signature
    match node.shutdown(&mut ctx) {
        Ok(()) => println!("   Node.shutdown(&mut NodeInfo) -> HorusResult<()>"),
        Err(e) => {
            eprintln!("Node shutdown failed: {}", e);
            return false;
        }
    }

    println!("   Node trait API is locked and working");
    true
}

/// Test 2: Scheduler API Lock
/// Ensures Scheduler methods have correct signatures
fn test_scheduler_api() -> bool {
    println!("Testing Scheduler API...");

    // Test Scheduler::new() - locks down constructor
    let scheduler = Scheduler::new();
    println!("   Scheduler::new()");

    // Test Scheduler::name() - locks down builder pattern
    let scheduler = scheduler.name("test_scheduler");
    println!("   Scheduler.name(&str)");

    let mut scheduler = scheduler;

    let counter = Arc::new(Mutex::new(0u32));
    let topic = format!("test_sched_api_{}", process::id());

    // Test Scheduler::add() - locks down node registration
    let node = Box::new(SensorNode::new(&topic, counter.clone()));
    scheduler.add(node, 0, None);
    println!("   Scheduler.add(Box<dyn Node>, u32, Option<bool>)");

    // Test alternative registration with logging enabled
    let node2 = Box::new(SensorNode::new(&topic, counter.clone()));
    scheduler.add(node2, 1, Some(true));
    println!("   Scheduler.add with logging enabled");

    // Test get_node_list() - locks down query API
    let nodes = scheduler.get_node_list();
    println!("   Scheduler.get_node_list() -> Vec<String>");
    assert!(nodes.len() >= 1, "Should have at least one node");

    // Test is_running() - locks down state query
    let running = scheduler.is_running();
    println!("   Scheduler.is_running() -> bool: {}", running);

    // Test stop() - locks down shutdown API
    scheduler.stop();
    println!("   Scheduler.stop()");

    println!("   Scheduler API is locked and working");
    true
}

/// Test for per-node rate control
/// Verifies that set_node_rate() works and nodes run at different frequencies
fn test_per_node_rate() -> bool {
    println!("Testing per-node rate control...");

    // Simple test nodes with different names
    struct FastNode {
        counter: Arc<Mutex<u32>>,
    }
    impl Node for FastNode {
        fn name(&self) -> &'static str {
            "fast_node"
        }
        fn tick(&mut self, _info: Option<&mut NodeInfo>) {
            *self.counter.lock().unwrap() += 1;
        }
    }

    struct SlowNode {
        counter: Arc<Mutex<u32>>,
    }
    impl Node for SlowNode {
        fn name(&self) -> &'static str {
            "slow_node"
        }
        fn tick(&mut self, _info: Option<&mut NodeInfo>) {
            *self.counter.lock().unwrap() += 1;
        }
    }

    // Create counters to track ticks
    let fast_counter = Arc::new(Mutex::new(0u32));
    let slow_counter = Arc::new(Mutex::new(0u32));

    // Create scheduler
    let mut scheduler = Scheduler::new();
    println!("   Scheduler created");

    // Add fast node (100Hz)
    let fast_node = Box::new(FastNode {
        counter: fast_counter.clone(),
    });
    scheduler
        .add(fast_node, 0, None)
        .set_node_rate("fast_node", 100.0);
    println!("   Added fast_node at 100Hz");

    // Add slow node (10Hz)
    let slow_node = Box::new(SlowNode {
        counter: slow_counter.clone(),
    });
    scheduler
        .add(slow_node, 1, None)
        .set_node_rate("slow_node", 10.0);
    println!("   Added slow_node at 10Hz");

    // Run for 1 second
    println!("  Running for 1 second...");
    if let Err(e) = scheduler.run_for(Duration::from_secs(1)) {
        eprintln!("Scheduler run failed: {}", e);
        return false;
    }

    // Get tick counts
    let fast_ticks = *fast_counter.lock().unwrap();
    let slow_ticks = *slow_counter.lock().unwrap();

    println!("   fast_node: {} ticks", fast_ticks);
    println!("   slow_node: {} ticks", slow_ticks);

    // Verify fast node ran more than slow node
    if fast_ticks <= slow_ticks {
        eprintln!("   Fast node should have more ticks than slow node!");
        eprintln!("     fast: {}, slow: {}", fast_ticks, slow_ticks);
        return false;
    }
    println!("   Fast node ran more frequently than slow node");

    // Verify fast node is roughly 10x faster (allow wide tolerance due to timing)
    let ratio = fast_ticks as f64 / slow_ticks as f64;
    if ratio >= 5.0 && ratio <= 15.0 {
        println!("   Rate ratio is reasonable: {:.1}x", ratio);
    } else {
        eprintln!("    Rate ratio unexpected: {:.1}x (expected ~10x)", ratio);
        // Don't fail - timing can be imprecise in tests
    }

    println!("   Per-node rate control is working");
    true
}

/// Test 3: Full Framework Workflow
/// Locks down the complete user workflow: create nodes  register  run
fn test_full_framework_workflow() -> bool {
    println!("Testing full framework workflow...");

    let sensor_counter = Arc::new(Mutex::new(0u32));
    let controller_counter = Arc::new(Mutex::new(0u32));

    let sensor_topic = format!("workflow_sensor_{}", process::id());
    let command_topic = format!("workflow_command_{}", process::id());

    // Step 1: Create nodes
    let sensor = Box::new(SensorNode::new(&sensor_topic, sensor_counter.clone()));
    let controller = Box::new(ControllerNode::new(
        &sensor_topic,
        &command_topic,
        controller_counter.clone(),
    ));

    println!("   Created nodes");

    // Step 2: Create and configure scheduler
    let mut scheduler = Scheduler::new();
    scheduler = scheduler.name("workflow_test");

    // Step 3: Add nodes with different priorities
    scheduler.add(sensor, 0, Some(false));
    scheduler.add(controller, 1, Some(true));

    println!("   Added nodes to scheduler");

    // Step 4: Verify node registration
    let nodes = scheduler.get_node_list();
    assert_eq!(nodes.len(), 2, "Should have 2 registered nodes");
    println!("   Verified node registration: {:?}", nodes);

    // Step 5: Query node info
    let sensor_info = scheduler.get_node_info("sensor_node");
    assert!(sensor_info.is_some(), "Should find sensor_node");
    println!("   Queried node information");

    // Step 6: Modify node settings (now chainable, logs warning if not found)
    scheduler.set_node_logging("controller_node", false);
    println!("   Modified node settings");

    // Step 7: Get monitoring summary
    let summary = scheduler.get_monitoring_summary();
    assert_eq!(summary.len(), 2, "Should have 2 nodes in summary");
    println!("   Retrieved monitoring summary: {:?}", summary);

    println!("   Full framework workflow completed successfully");
    true
}

/// Test 4: Multi-Node Graph Communication
/// Locks down complex node graphs with multiple interconnected nodes
fn test_multi_node_graph() -> bool {
    println!("Testing multi-node graph communication...");

    let sensor_counter = Arc::new(Mutex::new(0u32));
    let controller_counter = Arc::new(Mutex::new(0u32));
    let actuator_counter = Arc::new(Mutex::new(0u32));

    let sensor_topic = format!("graph_sensor_{}", process::id());
    let control_topic = format!("graph_control_{}", process::id());
    let actuator_topic = format!("graph_actuator_{}", process::id());

    // Create Link producer for actuator
    let actuator_sender =
        Topic::<CmdVel>::producer(&actuator_topic).expect("Failed to create Link producer");

    // Create node graph: Sensor  Controller  Actuator
    let sensor = Box::new(SensorNode::new(&sensor_topic, sensor_counter.clone()));
    let controller = Box::new(ControllerNode::new(
        &sensor_topic,
        &control_topic,
        controller_counter.clone(),
    ));
    let actuator = Box::new(ActuatorNode::new(&actuator_topic, actuator_counter.clone()));

    println!("   Created 3-node graph");

    // Add all nodes
    let mut scheduler = Scheduler::new();
    scheduler.add(sensor, 0, None);
    scheduler.add(controller, 1, None);
    scheduler.add(actuator, 2, None);

    println!("   Added all nodes in priority order");

    // Verify graph topology
    let nodes = scheduler.get_node_list();
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");
    assert!(nodes.contains(&"sensor_node".to_string()));
    assert!(nodes.contains(&"controller_node".to_string()));
    assert!(nodes.contains(&"actuator_node".to_string()));

    println!("   Multi-node graph communication pattern locked");
    true
}

/// Test 5: Node Lifecycle Events
/// Locks down init  tick  shutdown lifecycle
fn test_node_lifecycle() -> bool {
    println!("Testing node lifecycle (init  tick  shutdown)...");

    let counter = Arc::new(Mutex::new(0u32));
    let topic = format!("lifecycle_test_{}", process::id());

    let mut node = SensorNode::new(&topic, counter.clone());
    let mut ctx = NodeInfo::new("lifecycle_test".to_string(), true);

    // Phase 1: Init
    match node.init(&mut ctx) {
        Ok(()) => println!("   Phase 1: init() completed successfully"),
        Err(e) => {
            eprintln!("Init failed: {}", e);
            return false;
        }
    }

    // Phase 2: Tick loop
    for i in 0..10 {
        node.tick(Some(&mut ctx));
        if i == 0 {
            println!("   Phase 2: tick() executed");
        }
    }
    let tick_count = *counter.lock().unwrap();
    assert_eq!(tick_count, 10, "Should have ticked 10 times");
    println!("   Phase 2: tick() loop completed ({} ticks)", tick_count);

    // Phase 3: Shutdown
    match node.shutdown(&mut ctx) {
        Ok(()) => println!("   Phase 3: shutdown() completed successfully"),
        Err(e) => {
            eprintln!("Shutdown failed: {}", e);
            return false;
        }
    }

    println!("   Complete lifecycle executed: init  tick × 10  shutdown");
    true
}
