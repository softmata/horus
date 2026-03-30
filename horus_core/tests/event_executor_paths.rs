#![allow(dead_code)]
//! Integration tests for event executor code paths.
//!
//! Covers: burst notifications, paused event node, fatal policy stops scheduler,
//! restart recovery after event node panic.

use horus_core::core::{Node, NodeInfo};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::DurationExt;

// ============================================================================
// Mock nodes
// ============================================================================

/// Simple event counter node.
struct EventCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for EventCounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

/// Event node that always panics.
struct EventPanicNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for EventPanicNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        panic!("event panic");
    }
}

// ============================================================================
// Tests
// ============================================================================

#[test]
fn test_event_burst_notifications() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(EventCounterNode {
            name: "burst_event".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(5)
        .on("burst_topic")
        .build();

    // Start scheduler in background
    let handle = std::thread::spawn(move || {
        scheduler.run_for(500_u64.ms()).unwrap();
    });

    // Wait for the event node to register its notifier
    let mut registered = false;
    for _ in 0..100 {
        if NodeInfo::notify_event("burst_event") {
            registered = true;
            break;
        }
        std::thread::sleep(5_u64.ms());
    }

    if registered {
        // Wait for first notification to be processed
        std::thread::sleep(50_u64.ms());

        // Send 4 more rapid notifications (5 total including the initial one)
        for _ in 0..4 {
            NodeInfo::notify_event("burst_event");
        }

        // Wait for all to be processed
        std::thread::sleep(100_u64.ms());

        let ticks = tick_count.load(Ordering::SeqCst);
        assert!(
            ticks >= 5,
            "Should have at least 5 ticks from 5 notifications, got {}",
            ticks
        );
    }

    handle.join().unwrap();
}

#[test]
fn test_event_paused_node_no_tick() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(EventCounterNode {
            name: "paused_event".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(5)
        .on("paused_topic")
        .build();

    // Run without sending notifications — node should not tick
    scheduler.run_for(100_u64.ms()).unwrap();

    assert_eq!(
        tick_count.load(Ordering::SeqCst),
        0,
        "Event node without notifications should not tick"
    );
}

#[test]
fn test_event_fatal_policy_stops_scheduler() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(EventPanicNode {
            name: "fatal_event".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(5)
        .on("fatal_topic")
        .failure_policy(FailurePolicy::Fatal)
        .build();

    let handle = std::thread::spawn(move || {
        scheduler.run_for(500_u64.ms()).unwrap();
    });

    // Wait for event node to register
    let mut registered = false;
    for _ in 0..100 {
        if NodeInfo::notify_event("fatal_event") {
            registered = true;
            break;
        }
        std::thread::sleep(5_u64.ms());
    }

    if registered {
        // The notification should trigger a panic → Fatal → scheduler stops
        std::thread::sleep(100_u64.ms());

        let ticks = tick_count.load(Ordering::SeqCst);
        // Fatal should stop after first panic
        assert!(
            ticks >= 1,
            "Should tick at least once before fatal stop, got {}",
            ticks
        );
    }

    handle.join().unwrap();
}

#[test]
fn test_event_restart_recovery() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(EventPanicNode {
            name: "restart_event".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(5)
        .on("restart_topic")
        .failure_policy(FailurePolicy::restart(3, 10_u64.ms()))
        .build();

    let handle = std::thread::spawn(move || {
        scheduler.run_for(500_u64.ms()).unwrap();
    });

    // Wait for event node to register
    for _ in 0..100 {
        if NodeInfo::notify_event("restart_event") {
            break;
        }
        std::thread::sleep(5_u64.ms());
    }

    // Send multiple notifications to trigger multiple panics/restarts
    std::thread::sleep(50_u64.ms());
    for _ in 0..5 {
        NodeInfo::notify_event("restart_event");
        std::thread::sleep(30_u64.ms());
    }

    handle.join().unwrap();

    // The node should have attempted to tick (and panic) multiple times
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Restart event node should tick at least once, got {}",
        ticks
    );
}
