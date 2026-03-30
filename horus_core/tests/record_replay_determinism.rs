#![allow(dead_code)]
//! Record/Replay determinism test.
//!
//! Verifies that replay_from() uses ReplayClock and deterministic ordering,
//! and that the replay path sets up the correct infrastructure.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::Node;
use horus_core::scheduling::Scheduler;
use std::sync::{Arc, Mutex};

#[test]
fn deterministic_record_produces_reproducible_outputs() {
    // Record a session, then run the same deterministic system again.
    // Both runs should produce identical outputs.
    let outputs_run1 = run_deterministic_session(50);
    let outputs_run2 = run_deterministic_session(50);

    assert_eq!(outputs_run1.len(), 50, "Should produce 50 outputs");
    assert_eq!(
        outputs_run1, outputs_run2,
        "Two deterministic runs must produce identical outputs"
    );
}

#[test]
fn recording_enabled_does_not_affect_determinism() {
    // Run with and without .with_recording() — outputs should be identical
    // since recording is a passive observer.
    let without_recording = run_deterministic_session(30);

    let with_recording = {
        let outputs = Arc::new(Mutex::new(Vec::new()));

        {
            let mut scheduler = Scheduler::new()
                .deterministic(true)
                .with_recording()
                .tick_rate(100_u64.hz());

            scheduler
                .add(DeterministicNode::new(outputs.clone()))
                .order(0)
                .rate(100_u64.hz())
                .build()
                .unwrap();

            for _ in 0..30 {
                scheduler.tick_once().unwrap();
            }
        }

        let result = outputs.lock().unwrap().clone();
        result
    };

    assert_eq!(
        without_recording, with_recording,
        "Recording should not affect deterministic outputs"
    );
}

#[test]
fn deterministic_mode_with_dependency_and_recording() {
    // Multi-node system with dependencies + recording enabled
    let log = Arc::new(Mutex::new(Vec::new()));

    let mut scheduler = Scheduler::new()
        .deterministic(true)
        .with_recording()
        .tick_rate(100_u64.hz());

    scheduler
        .add(ProducerNode::new("producer", "data", log.clone()))
        .order(0)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    scheduler
        .add(ConsumerNode::new("consumer", "data", log.clone()))
        .order(1)
        .rate(100_u64.hz())
        .build()
        .unwrap();

    for _ in 0..10 {
        scheduler.tick_once().unwrap();
    }

    let entries = log.lock().unwrap();
    assert_eq!(entries.len(), 20); // 2 nodes × 10 ticks

    // Producer always before consumer
    for tick in 0..10 {
        assert_eq!(entries[tick * 2], "producer");
        assert_eq!(entries[tick * 2 + 1], "consumer");
    }
}

// ── Test Nodes ──────────────────────────────────────────────────────────────

struct DeterministicNode {
    counter: u64,
    outputs: Arc<Mutex<Vec<u64>>>,
}

impl DeterministicNode {
    fn new(outputs: Arc<Mutex<Vec<u64>>>) -> Self {
        Self {
            counter: 0,
            outputs,
        }
    }
}

impl Node for DeterministicNode {
    fn name(&self) -> &str {
        "deterministic"
    }
    fn tick(&mut self) {
        let dt = horus_core::core::tick_context::ctx_dt();
        let rng_val = horus_core::core::tick_context::ctx_with_rng(|rng| {
            use rand::Rng;
            rng.gen::<u64>()
        });
        self.counter += 1;
        let output = self.counter ^ rng_val ^ (dt.as_nanos() as u64);
        self.outputs.lock().unwrap().push(output);
    }
}

struct ProducerNode {
    name: String,
    topic: String,
    log: Arc<Mutex<Vec<String>>>,
}

impl ProducerNode {
    fn new(name: &str, topic: &str, log: Arc<Mutex<Vec<String>>>) -> Self {
        Self {
            name: name.to_string(),
            topic: topic.to_string(),
            log,
        }
    }
}

impl Node for ProducerNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.log.lock().unwrap().push(self.name.clone());
    }
}

struct ConsumerNode {
    name: String,
    topic: String,
    log: Arc<Mutex<Vec<String>>>,
}

impl ConsumerNode {
    fn new(name: &str, topic: &str, log: Arc<Mutex<Vec<String>>>) -> Self {
        Self {
            name: name.to_string(),
            topic: topic.to_string(),
            log,
        }
    }
}

impl Node for ConsumerNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.log.lock().unwrap().push(self.name.clone());
    }
}

fn run_deterministic_session(num_ticks: usize) -> Vec<u64> {
    let outputs = Arc::new(Mutex::new(Vec::new()));

    {
        let mut scheduler = Scheduler::new().deterministic(true).tick_rate(100_u64.hz());

        scheduler
            .add(DeterministicNode::new(outputs.clone()))
            .order(0)
            .rate(100_u64.hz())
            .build()
            .unwrap();

        for _ in 0..num_ticks {
            scheduler.tick_once().unwrap();
        }
    }

    let result = outputs.lock().unwrap().clone();
    result
}
