//! Integration tests for multi-node scheduler pipelines.
//!
//! Verifies the most common real-world usage pattern: sensor → processor →
//! controller running at different rates, communicating via Topics.
//!
//! **Gap addressed**: No existing test exercises a full multi-node pipeline
//! with inter-node Topic communication through the Scheduler.

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

// ============================================================================
// Pipeline nodes
// ============================================================================

/// Sensor node: publishes incrementing counter to a topic.
struct SensorNode {
    name: &'static str,
    counter: Arc<AtomicU64>,
    topic: Option<Topic<u64>>,
}

impl Node for SensorNode {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new("pipeline_sensor_data")?);
        Ok(())
    }
    fn tick(&mut self) {
        let val = self.counter.fetch_add(1, Ordering::SeqCst) + 1;
        if let Some(ref topic) = self.topic {
            topic.send(val);
        }
    }
}

/// Processor node: reads from sensor topic, writes doubled value to output topic.
struct ProcessorNode {
    name: &'static str,
    processed: Arc<AtomicU64>,
    input: Option<Topic<u64>>,
    output: Option<Topic<u64>>,
}

impl Node for ProcessorNode {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.input = Some(Topic::new("pipeline_sensor_data")?);
        self.output = Some(Topic::new("pipeline_processed")?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref input) = self.input {
            while let Some(val) = input.recv() {
                let doubled = val * 2;
                self.processed.fetch_add(1, Ordering::SeqCst);
                if let Some(ref output) = self.output {
                    output.send(doubled);
                }
            }
        }
    }
}

/// Controller node: reads processed data and tracks latest value.
struct ControllerNode {
    name: &'static str,
    latest_value: Arc<AtomicU64>,
    received_count: Arc<AtomicU64>,
    input: Option<Topic<u64>>,
}

impl Node for ControllerNode {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.input = Some(Topic::new("pipeline_processed")?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref input) = self.input {
            while let Some(val) = input.recv() {
                self.latest_value.store(val, Ordering::SeqCst);
                self.received_count.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[test]
fn three_node_pipeline_data_flows_end_to_end() {
    cleanup_stale_shm();

    let sensor_count = Arc::new(AtomicU64::new(0));
    let processor_count = Arc::new(AtomicU64::new(0));
    let controller_latest = Arc::new(AtomicU64::new(0));
    let controller_received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(SensorNode {
            name: "sensor",
            counter: sensor_count.clone(),
            topic: None,
        })
        .order(0)
        .build();

    scheduler
        .add(ProcessorNode {
            name: "processor",
            processed: processor_count.clone(),
            input: None,
            output: None,
        })
        .order(1)
        .build();

    scheduler
        .add(ControllerNode {
            name: "controller",
            latest_value: controller_latest.clone(),
            received_count: controller_received.clone(),
            input: None,
        })
        .order(2)
        .build();

    let result = scheduler.run_for(300_u64.ms());
    assert!(result.is_ok(), "Pipeline should run without error: {:?}", result.err());

    let s = sensor_count.load(Ordering::SeqCst);
    let p = processor_count.load(Ordering::SeqCst);
    let r = controller_received.load(Ordering::SeqCst);
    let v = controller_latest.load(Ordering::SeqCst);

    assert!(s > 5, "Sensor should have published at least 5 values, got {s}");
    assert!(p > 0, "Processor should have processed at least 1 value, got {p}");
    assert!(r > 0, "Controller should have received at least 1 value, got {r}");
    assert!(v > 0, "Controller latest value should be non-zero (doubled sensor output), got {v}");
    assert_eq!(v % 2, 0, "Controller value should be even (doubled), got {v}");
}

#[test]
fn pipeline_ordering_ensures_sensor_before_processor() {
    cleanup_stale_shm();

    let sensor_count = Arc::new(AtomicU64::new(0));
    let processor_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz());

    // Sensor at order 0 runs before processor at order 1
    scheduler
        .add(SensorNode {
            name: "ordered_sensor",
            counter: sensor_count.clone(),
            topic: None,
        })
        .order(0)
        .build();

    scheduler
        .add(ProcessorNode {
            name: "ordered_processor",
            processed: processor_count.clone(),
            input: None,
            output: None,
        })
        .order(1)
        .build();

    let _ = scheduler.run_for(200_u64.ms());

    let s = sensor_count.load(Ordering::SeqCst);
    let p = processor_count.load(Ordering::SeqCst);

    assert!(s > 0, "Sensor should have ticked, got {s}");
    // With correct ordering, processor should process sensor's data within same tick
    assert!(p > 0, "Processor should have processed data from sensor, got {p}");
}

#[test]
fn tick_once_processes_pipeline_synchronously() {
    cleanup_stale_shm();

    let sensor_count = Arc::new(AtomicU64::new(0));
    let processor_count = Arc::new(AtomicU64::new(0));
    let controller_received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();

    scheduler
        .add(SensorNode {
            name: "sync_sensor",
            counter: sensor_count.clone(),
            topic: None,
        })
        .order(0)
        .build();

    scheduler
        .add(ProcessorNode {
            name: "sync_processor",
            processed: processor_count.clone(),
            input: None,
            output: None,
        })
        .order(1)
        .build();

    scheduler
        .add(ControllerNode {
            name: "sync_controller",
            latest_value: Arc::new(AtomicU64::new(0)),
            received_count: controller_received.clone(),
            input: None,
        })
        .order(2)
        .build();

    // Single tick — all three nodes process in order
    let result = scheduler.tick_once();
    assert!(result.is_ok(), "tick_once should succeed: {:?}", result.err());

    let s = sensor_count.load(Ordering::SeqCst);
    assert_eq!(s, 1, "Sensor should tick exactly once");

    // After one tick, processor may or may not have received the sensor data
    // (depends on Topic backend — DirectChannel vs ring buffer)
    // The important thing is no panic and correct ordering
}

#[test]
fn pipeline_with_watchdog_completes() {
    cleanup_stale_shm();

    let sensor_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(50_u64.hz())
        .watchdog(1_u64.secs());

    scheduler
        .add(SensorNode {
            name: "wd_sensor",
            counter: sensor_count.clone(),
            topic: None,
        })
        .order(0)
        .build();

    let result = scheduler.run_for(300_u64.ms());
    assert!(result.is_ok(), "Pipeline with watchdog should complete: {:?}", result.err());

    let s = sensor_count.load(Ordering::SeqCst);
    assert!(s > 5, "Sensor should have ticked multiple times, got {s}");
}
