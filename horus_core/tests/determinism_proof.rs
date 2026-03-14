//! Determinism proof: verify bit-identical outputs across multiple runs.
//!
//! This is the ultimate test for the determinism blueprint. A multi-node system
//! using horus::now(), horus::dt(), horus::rng() must produce identical outputs
//! every run in deterministic mode.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::{Node, TopicMetadata};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

// ── Test Nodes ──────────────────────────────────────────────────────────────

/// Sensor: produces incrementing counter as "sensor data"
struct SensorNode {
    counter: u64,
    outputs: Arc<Mutex<Vec<u64>>>,
}

impl SensorNode {
    fn new(outputs: Arc<Mutex<Vec<u64>>>) -> Self {
        Self {
            counter: 0,
            outputs,
        }
    }
}

impl Node for SensorNode {
    fn name(&self) -> &str {
        "sensor"
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: "sensor_data".into(),
            type_name: "u64".into(),
        }]
    }

    fn tick(&mut self) {
        self.counter += 1;
        self.outputs.lock().unwrap().push(self.counter);
    }
}

/// Controller: uses horus::dt() for integration and horus tick number
/// for deterministic computation. Produces output based on accumulated state.
struct ControllerNode {
    position: f64,
    velocity: f64,
    outputs: Arc<Mutex<Vec<u64>>>,
}

impl ControllerNode {
    fn new(outputs: Arc<Mutex<Vec<u64>>>) -> Self {
        Self {
            position: 0.0,
            velocity: 1.0,
            outputs,
        }
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &str {
        "controller"
    }

    fn subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: "sensor_data".into(),
            type_name: "u64".into(),
        }]
    }

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: "control_output".into(),
            type_name: "u64".into(),
        }]
    }

    fn tick(&mut self) {
        // Use framework dt for integration
        let dt = horus_core::core::tick_context::ctx_dt();
        self.position += self.velocity * dt.as_secs_f64();

        // Use framework tick number for deterministic computation
        let tick = horus_core::core::tick_context::ctx_tick();

        // Use framework RNG for noise
        let noise = horus_core::core::tick_context::ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });

        // Combine into a deterministic output
        let output = (self.position * 1_000_000.0) as u64 ^ tick ^ noise;
        self.outputs.lock().unwrap().push(output);
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

/// Run the system once in deterministic mode, return outputs.
fn run_deterministic_system(num_ticks: usize) -> (Vec<u64>, Vec<u64>) {
    let sensor_outputs = Arc::new(Mutex::new(Vec::new()));
    let controller_outputs = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new()
        .deterministic(true)
        .tick_rate(100_u64.hz());

    scheduler
        .add(SensorNode::new(sensor_outputs.clone()))
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    scheduler
        .add(ControllerNode::new(controller_outputs.clone()))
        .order(1)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    for _ in 0..num_ticks {
        scheduler.tick_once().unwrap();
    }

    let s = sensor_outputs.lock().unwrap().clone();
    let c = controller_outputs.lock().unwrap().clone();
    (s, c)
}

#[test]
fn deterministic_mode_produces_identical_outputs() {
    let num_runs = 10;
    let num_ticks = 50;

    let (first_sensor, first_controller) = run_deterministic_system(num_ticks);

    assert_eq!(
        first_sensor.len(),
        num_ticks,
        "Sensor should produce {} outputs",
        num_ticks
    );
    assert_eq!(
        first_controller.len(),
        num_ticks,
        "Controller should produce {} outputs",
        num_ticks
    );

    for run in 1..num_runs {
        let (sensor, controller) = run_deterministic_system(num_ticks);

        assert_eq!(
            sensor, first_sensor,
            "Run {} sensor outputs differ from run 0",
            run
        );
        assert_eq!(
            controller, first_controller,
            "Run {} controller outputs differ from run 0",
            run
        );
    }
}

#[test]
fn deterministic_mode_uses_simclock() {
    let outputs = Arc::new(Mutex::new(Vec::new()));

    struct ClockCheckNode {
        timestamps: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for ClockCheckNode {
        fn name(&self) -> &str {
            "clock_check"
        }
        fn tick(&mut self) {
            let now = horus_core::core::tick_context::ctx_now();
            self.timestamps.lock().unwrap().push(now.as_nanos());
        }
    }

    let mut scheduler = Scheduler::new()
        .deterministic(true)
        .tick_rate(100_u64.hz());

    scheduler
        .add(ClockCheckNode {
            timestamps: outputs.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let timestamps = outputs.lock().unwrap().clone();

    // SimClock should produce evenly spaced timestamps
    // At 100Hz, period = 10ms = 10_000_000ns
    // Each step advances by period, so timestamps should be multiples of 10ms
    assert_eq!(timestamps.len(), 10);

    // Verify timestamps are deterministic by running again
    let outputs2 = Arc::new(Mutex::new(Vec::new()));
    let mut scheduler2 = Scheduler::new()
        .deterministic(true)
        .tick_rate(100_u64.hz());

    scheduler2
        .add(ClockCheckNode {
            timestamps: outputs2.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler2.tick_once().unwrap();
    }

    let timestamps2 = outputs2.lock().unwrap().clone();
    assert_eq!(
        timestamps, timestamps2,
        "SimClock timestamps must be identical across runs"
    );
}

#[test]
fn deterministic_rng_is_reproducible() {
    struct RngNode {
        values: Arc<Mutex<Vec<u64>>>,
    }

    impl Node for RngNode {
        fn name(&self) -> &str {
            "rng_node"
        }
        fn tick(&mut self) {
            let v = horus_core::core::tick_context::ctx_with_rng(|rng| {
                use rand::Rng;
                rng.gen::<u64>()
            });
            self.values.lock().unwrap().push(v);
        }
    }

    let mut results = Vec::new();

    for _ in 0..5 {
        let values = Arc::new(Mutex::new(Vec::new()));
        let mut scheduler = Scheduler::new()
            .deterministic(true)
            .tick_rate(100_u64.hz());

        scheduler
            .add(RngNode {
                values: values.clone(),
            })
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();

        for _ in 0..20 {
            scheduler.tick_once().unwrap();
        }

        results.push(values.lock().unwrap().clone());
    }

    // All 5 runs must produce identical RNG sequences
    for (i, result) in results.iter().enumerate().skip(1) {
        assert_eq!(
            result, &results[0],
            "RNG run {} differs from run 0",
            i
        );
    }
}

#[test]
fn dependency_ordering_respected() {
    let execution_order = Arc::new(Mutex::new(Vec::new()));

    struct OrderNode {
        id: &'static str,
        log: Arc<Mutex<Vec<String>>>,
    }

    impl Node for OrderNode {
        fn name(&self) -> &str {
            self.id
        }
        fn publishers(&self) -> Vec<TopicMetadata> {
            match self.id {
                "producer" => vec![TopicMetadata {
                    topic_name: "data".into(),
                    type_name: "T".into(),
                }],
                _ => vec![],
            }
        }
        fn subscribers(&self) -> Vec<TopicMetadata> {
            match self.id {
                "consumer" => vec![TopicMetadata {
                    topic_name: "data".into(),
                    type_name: "T".into(),
                }],
                _ => vec![],
            }
        }
        fn tick(&mut self) {
            self.log.lock().unwrap().push(self.id.to_string());
        }
    }

    let mut scheduler = Scheduler::new()
        .deterministic(true)
        .tick_rate(100_u64.hz());

    // Add consumer FIRST (wrong order) — dependency graph should fix it
    scheduler
        .add(OrderNode {
            id: "consumer",
            log: execution_order.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    scheduler
        .add(OrderNode {
            id: "producer",
            log: execution_order.clone(),
        })
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    scheduler.tick_once().unwrap();

    let order = execution_order.lock().unwrap();
    assert_eq!(order.len(), 2);
    assert_eq!(
        order[0], "producer",
        "Producer must execute before consumer (dependency graph)"
    );
    assert_eq!(order[1], "consumer");
}

#[test]
fn normal_mode_unchanged() {
    // Verify normal mode (no .deterministic()) still works with the new code
    let ticked = Arc::new(AtomicU64::new(0));

    struct SimpleNode {
        count: Arc<AtomicU64>,
    }

    impl Node for SimpleNode {
        fn name(&self) -> &str {
            "simple"
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(SimpleNode {
            count: ticked.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    assert_eq!(ticked.load(Ordering::Relaxed), 10);
}
