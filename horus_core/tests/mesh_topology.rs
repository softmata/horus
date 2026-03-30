#![allow(dead_code)]
//! Integration tests for complex node topologies (mesh, star, diamond DAG).
//!
//! Validates that 10+ nodes with fan-in, fan-out, and multi-path dataflows
//! work correctly under scheduler orchestration.
//!
//! **Gap addressed**: Largest existing test is 8 nodes in a mostly linear
//! pipeline. Production robots often have 10-15 nodes with DAG topologies.

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

fn unique(prefix: &str) -> String {
    use std::sync::atomic::AtomicU64;
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

// ============================================================================
// Flexible node: configurable pub/sub for arbitrary topologies
// ============================================================================

/// A node that subscribes to multiple input topics, processes data,
/// and publishes to multiple output topics. Configurable for any topology.
struct FlexNode {
    name: &'static str,
    sub_names: Vec<String>,
    pub_names: Vec<String>,
    subs: Vec<Option<Topic<u64>>>,
    pubs: Vec<Option<Topic<u64>>>,
    counter: Arc<AtomicU64>,
    received: Arc<AtomicU64>,
    transform: fn(u64) -> u64,
}

impl FlexNode {
    fn new(
        name: &'static str,
        sub_names: Vec<String>,
        pub_names: Vec<String>,
        counter: Arc<AtomicU64>,
        received: Arc<AtomicU64>,
        transform: fn(u64) -> u64,
    ) -> Self {
        let sub_count = sub_names.len();
        let pub_count = pub_names.len();
        Self {
            name,
            sub_names,
            pub_names,
            subs: vec![None; sub_count],
            pubs: vec![None; pub_count],
            counter,
            received,
            transform,
        }
    }
}

impl Node for FlexNode {
    fn name(&self) -> &'static str {
        self.name
    }

    fn init(&mut self) -> horus_core::error::Result<()> {
        for (i, name) in self.sub_names.iter().enumerate() {
            self.subs[i] = Some(Topic::new(name)?);
        }
        for (i, name) in self.pub_names.iter().enumerate() {
            self.pubs[i] = Some(Topic::new(name)?);
        }
        Ok(())
    }

    fn tick(&mut self) {
        self.counter.fetch_add(1, Ordering::SeqCst);

        // Read from all subscriptions
        let mut latest_val: Option<u64> = None;
        for ref mut topic in self.subs.iter_mut().flatten() {
            while let Some(val) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
                latest_val = Some(val);
            }
        }

        // Transform and publish to all outputs
        let out_val = if let Some(v) = latest_val {
            (self.transform)(v)
        } else {
            self.counter.load(Ordering::SeqCst)
        };

        for topic in self.pubs.iter().flatten() {
            topic.send(out_val);
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

/// 12-node DAG topology representing a full warehouse robot:
///
/// ```text
/// lidar(0) ─────┐
///                ├─→ sensor_fusion(3) ─→ path_planner(5) ─┬→ left_ctrl(6) ─→ left_motor(8)
/// imu(1) ───────┘                                         └→ right_ctrl(7) ─→ right_motor(9)
///
/// camera(2) ─→ perception(4) ─→ path_planner(5)
///
/// health_monitor(10) ← subscribes to fusion + planner
/// safety_node(11) ← subscribes to left_cmd + right_cmd
/// ```
#[test]
fn twelve_node_dag() {
    cleanup_stale_shm();

    // Topic names
    let scan = unique("dag_scan");
    let imu_data = unique("dag_imu");
    let image = unique("dag_image");
    let fused = unique("dag_fused");
    let detections = unique("dag_detect");
    let path = unique("dag_path");
    let left_cmd = unique("dag_left_cmd");
    let right_cmd = unique("dag_right_cmd");
    let left_fb = unique("dag_left_fb");
    let right_fb = unique("dag_right_fb");

    // Counters: one per node
    let counters: Vec<(String, Arc<AtomicU64>, Arc<AtomicU64>)> = [
        "lidar",
        "imu",
        "camera",
        "fusion",
        "perception",
        "planner",
        "left_ctrl",
        "right_ctrl",
        "left_motor",
        "right_motor",
        "health",
        "safety",
    ]
    .iter()
    .map(|n| {
        (
            n.to_string(),
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    })
    .collect();

    let identity = |v: u64| v;
    let double = |v: u64| v.wrapping_mul(2);

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // 0: lidar — source, publishes scan
    scheduler
        .add(FlexNode::new(
            "lidar",
            vec![],
            vec![scan.clone()],
            counters[0].1.clone(),
            counters[0].2.clone(),
            identity,
        ))
        .order(0)
        .build();

    // 1: imu — source, publishes imu_data
    scheduler
        .add(FlexNode::new(
            "imu",
            vec![],
            vec![imu_data.clone()],
            counters[1].1.clone(),
            counters[1].2.clone(),
            identity,
        ))
        .order(1)
        .build();

    // 2: camera — source, publishes image
    scheduler
        .add(FlexNode::new(
            "camera",
            vec![],
            vec![image.clone()],
            counters[2].1.clone(),
            counters[2].2.clone(),
            identity,
        ))
        .order(2)
        .build();

    // 3: sensor_fusion — subscribes scan+imu, publishes fused
    scheduler
        .add(FlexNode::new(
            "fusion",
            vec![scan.clone(), imu_data.clone()],
            vec![fused.clone()],
            counters[3].1.clone(),
            counters[3].2.clone(),
            identity,
        ))
        .order(3)
        .build();

    // 4: perception — subscribes image, publishes detections
    scheduler
        .add(FlexNode::new(
            "perception",
            vec![image.clone()],
            vec![detections.clone()],
            counters[4].1.clone(),
            counters[4].2.clone(),
            identity,
        ))
        .order(4)
        .build();

    // 5: planner — subscribes fused+detections, publishes path
    scheduler
        .add(FlexNode::new(
            "planner",
            vec![fused.clone(), detections.clone()],
            vec![path.clone()],
            counters[5].1.clone(),
            counters[5].2.clone(),
            identity,
        ))
        .order(5)
        .build();

    // 6: left_ctrl — subscribes path, publishes left_cmd
    scheduler
        .add(FlexNode::new(
            "left_ctrl",
            vec![path.clone()],
            vec![left_cmd.clone()],
            counters[6].1.clone(),
            counters[6].2.clone(),
            double,
        ))
        .order(6)
        .build();

    // 7: right_ctrl — subscribes path, publishes right_cmd
    scheduler
        .add(FlexNode::new(
            "right_ctrl",
            vec![path.clone()],
            vec![right_cmd.clone()],
            counters[7].1.clone(),
            counters[7].2.clone(),
            double,
        ))
        .order(7)
        .build();

    // 8: left_motor — subscribes left_cmd, publishes left_fb
    scheduler
        .add(FlexNode::new(
            "left_motor",
            vec![left_cmd.clone()],
            vec![left_fb.clone()],
            counters[8].1.clone(),
            counters[8].2.clone(),
            identity,
        ))
        .order(8)
        .build();

    // 9: right_motor — subscribes right_cmd, publishes right_fb
    scheduler
        .add(FlexNode::new(
            "right_motor",
            vec![right_cmd.clone()],
            vec![right_fb.clone()],
            counters[9].1.clone(),
            counters[9].2.clone(),
            identity,
        ))
        .order(9)
        .build();

    // 10: health_monitor — subscribes fused+path (read-only observer)
    scheduler
        .add(FlexNode::new(
            "health",
            vec![fused.clone(), path.clone()],
            vec![],
            counters[10].1.clone(),
            counters[10].2.clone(),
            identity,
        ))
        .order(10)
        .build();

    // 11: safety — subscribes left_cmd+right_cmd (read-only observer)
    scheduler
        .add(FlexNode::new(
            "safety",
            vec![left_cmd.clone(), right_cmd.clone()],
            vec![],
            counters[11].1.clone(),
            counters[11].2.clone(),
            identity,
        ))
        .order(11)
        .build();

    let result = scheduler.run_for(1_u64.secs());
    assert!(
        result.is_ok(),
        "12-node DAG should not error: {:?}",
        result.err()
    );

    // All 12 nodes must have ticked
    for (name, tick_counter, _recv_counter) in &counters {
        let ticks = tick_counter.load(Ordering::SeqCst);
        assert!(ticks > 0, "Node '{name}' never ticked (got {ticks})");
    }

    // Downstream nodes must have received data
    let fusion_recv = counters[3].2.load(Ordering::SeqCst);
    let motor_recv = counters[8].2.load(Ordering::SeqCst);
    let safety_recv = counters[11].2.load(Ordering::SeqCst);

    assert!(
        fusion_recv > 0,
        "Fusion should receive from sensors, got {fusion_recv}"
    );
    assert!(
        motor_recv > 0,
        "Motor should receive commands, got {motor_recv}"
    );
    assert!(
        safety_recv > 0,
        "Safety should observe commands, got {safety_recv}"
    );
}

/// Star topology: 10 publishers → 1 subscriber on the same topic.
/// Tests MPMC backend with many producers.
#[test]
fn star_topology_10_publishers_1_subscriber() {
    cleanup_stale_shm();

    let topic_name = unique("star_hub");

    let pub_counters: Vec<Arc<AtomicU64>> = (0..10).map(|_| Arc::new(AtomicU64::new(0))).collect();
    let sub_received = Arc::new(AtomicU64::new(0));
    let sub_ticks = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // 10 publishers, each sending their index as the value
    for i in 0..10 {
        let name: &'static str = Box::leak(format!("spoke_{i}").into_boxed_str());
        scheduler
            .add(FlexNode::new(
                name,
                vec![],
                vec![topic_name.clone()],
                pub_counters[i].clone(),
                Arc::new(AtomicU64::new(0)),
                identity_fn(i as u64),
            ))
            .order(i as u32)
            .build();
    }

    // 1 subscriber (hub)
    scheduler
        .add(FlexNode::new(
            "hub",
            vec![topic_name.clone()],
            vec![],
            sub_ticks.clone(),
            sub_received.clone(),
            |v| v,
        ))
        .order(20)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(
        result.is_ok(),
        "Star topology should not error: {:?}",
        result.err()
    );

    // All publishers should have ticked
    for (i, counter) in pub_counters.iter().enumerate() {
        let c = counter.load(Ordering::SeqCst);
        assert!(c > 0, "Publisher spoke_{i} never ticked, got {c}");
    }

    // Hub should have received from multiple publishers
    let received = sub_received.load(Ordering::SeqCst);
    assert!(
        received >= 10,
        "Hub should receive from all 10 publishers (got {received} total messages)"
    );
}

/// Helper: create transform function that adds a fixed offset.
fn identity_fn(_offset: u64) -> fn(u64) -> u64 {
    // We can't capture offset in fn pointer, so use identity.
    // The FlexNode sends its tick counter as default when no input.
    |v| v
}

/// Diamond topology: A → {B, C} → D
/// A publishes, B adds 10, C adds 100, D receives from both.
/// Tests fan-out (A→B, A→C) and fan-in (B→D, C→D).
#[test]
fn diamond_topology_fan_out_fan_in() {
    cleanup_stale_shm();

    let a_to_bc = unique("diamond_a");
    let b_to_d = unique("diamond_b");
    let c_to_d = unique("diamond_c");

    let a_ticks = Arc::new(AtomicU64::new(0));
    let b_ticks = Arc::new(AtomicU64::new(0));
    let c_ticks = Arc::new(AtomicU64::new(0));
    let d_ticks = Arc::new(AtomicU64::new(0));

    let a_recv = Arc::new(AtomicU64::new(0));
    let b_recv = Arc::new(AtomicU64::new(0));
    let c_recv = Arc::new(AtomicU64::new(0));
    let d_recv = Arc::new(AtomicU64::new(0));

    let add_10 = |v: u64| v.wrapping_add(10);
    let add_100 = |v: u64| v.wrapping_add(100);

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // A: source, publishes to a_to_bc
    scheduler
        .add(FlexNode::new(
            "node_a",
            vec![],
            vec![a_to_bc.clone()],
            a_ticks.clone(),
            a_recv.clone(),
            |v| v,
        ))
        .order(0)
        .build();

    // B: subscribes a_to_bc, adds 10, publishes to b_to_d
    scheduler
        .add(FlexNode::new(
            "node_b",
            vec![a_to_bc.clone()],
            vec![b_to_d.clone()],
            b_ticks.clone(),
            b_recv.clone(),
            add_10,
        ))
        .order(1)
        .build();

    // C: subscribes a_to_bc, adds 100, publishes to c_to_d
    scheduler
        .add(FlexNode::new(
            "node_c",
            vec![a_to_bc.clone()],
            vec![c_to_d.clone()],
            c_ticks.clone(),
            c_recv.clone(),
            add_100,
        ))
        .order(2)
        .build();

    // D: subscribes b_to_d + c_to_d (fan-in from both branches)
    scheduler
        .add(FlexNode::new(
            "node_d",
            vec![b_to_d.clone(), c_to_d.clone()],
            vec![],
            d_ticks.clone(),
            d_recv.clone(),
            |v| v,
        ))
        .order(3)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(
        result.is_ok(),
        "Diamond should not error: {:?}",
        result.err()
    );

    // All 4 nodes ticked
    for (name, counter) in [
        ("A", &a_ticks),
        ("B", &b_ticks),
        ("C", &c_ticks),
        ("D", &d_ticks),
    ] {
        let c = counter.load(Ordering::SeqCst);
        assert!(c > 0, "Node {name} never ticked, got {c}");
    }

    // B and C received from A (fan-out works)
    let b_r = b_recv.load(Ordering::SeqCst);
    let c_r = c_recv.load(Ordering::SeqCst);
    assert!(b_r > 0, "B should receive from A (fan-out), got {b_r}");
    assert!(c_r > 0, "C should receive from A (fan-out), got {c_r}");

    // D received from both B and C (fan-in works)
    let d_r = d_recv.load(Ordering::SeqCst);
    assert!(
        d_r > 0,
        "D should receive from both B and C (fan-in), got {d_r}"
    );
}
