#![cfg(feature = "macros")]
//! Edge-case integration tests for the `node!` macro.
//!
//! Focuses on four specific scenarios:
//! 1. Minimal node (simplest possible)
//! 2. Node with pub/sub topics
//! 3. Node with data fields and stateful ticking
//! 4. Node with init + shutdown lifecycle hooks

use horus::prelude::*;

// ============================================================================
// 1. Minimal node: simplest possible (just tick)
// ============================================================================

node! {
    Minimal {
        tick {
            // Empty body — valid minimal node
        }
    }
}

#[test]
fn test_node_macro_minimal() {
    let mut node = Minimal::new();

    // Must implement Node trait
    assert_eq!(node.name(), "minimal");

    // Default init/shutdown must succeed
    node.init().unwrap();
    node.tick();
    node.shutdown().unwrap();

    // No publishers or subscribers
    assert!(node.publishers().is_empty());
    assert!(node.subscribers().is_empty());

    // Default trait works
    let node2 = Minimal::default();
    assert_eq!(node2.name(), "minimal");

    // Can be added to a scheduler and ticked
    let mut scheduler = Scheduler::new();
    scheduler.add(Minimal::new()).order(0).build().unwrap();
    scheduler.tick_once().unwrap();
}

// ============================================================================
// 2. Node with pub/sub topics — verify both ends work
// ============================================================================

node! {
    PubSub {
        pub {
            output: u64 -> "edge.out.topic",
        }

        sub {
            input: u64 -> "edge.in.topic",
        }

        tick {
            if let Some(val) = self.input.recv() {
                self.output.send(val * 2);
            }
        }
    }
}

#[test]
fn test_node_macro_with_pub_sub() {
    let mut node = PubSub::new();

    // Publisher metadata
    let pubs = node.publishers();
    assert_eq!(pubs.len(), 1);
    assert_eq!(pubs[0].topic_name, "edge.out.topic");

    // Subscriber metadata
    let subs = node.subscribers();
    assert_eq!(subs.len(), 1);
    assert_eq!(subs[0].topic_name, "edge.in.topic");

    // No data yet — tick should not crash or publish
    node.tick();
    assert!(node.output.recv().is_none());

    // Send data to subscriber, tick, and verify the publisher got the doubled value
    node.input.send(21);
    node.tick();
    let result = node.output.recv();
    assert_eq!(result, Some(42));
}

// ============================================================================
// 3. Node with data fields — stateful counter
// ============================================================================

node! {
    Stateful {
        data {
            counter: u64 = 0,
        }

        tick {
            self.counter += 1;
        }
    }
}

#[test]
fn test_node_macro_with_data_fields() {
    let mut node = Stateful::new();

    // Initial value
    assert_eq!(node.counter, 0);

    // Run a few ticks
    node.tick();
    node.tick();
    node.tick();
    assert_eq!(node.counter, 3);

    // Run many more ticks
    for _ in 0..97 {
        node.tick();
    }
    assert_eq!(node.counter, 100);
}

// ============================================================================
// 4. Node with init + shutdown — verify lifecycle hooks are called
// ============================================================================

node! {
    Lifecycle {
        data {
            init_called: bool = false,
            shutdown_called: bool = false,
            ticks: u32 = 0,
        }

        init {
            self.init_called = true;
            Ok(())
        }

        tick {
            self.ticks += 1;
        }

        shutdown {
            self.shutdown_called = true;
            Ok(())
        }
    }
}

#[test]
fn test_node_macro_with_init_shutdown() {
    let mut node = Lifecycle::new();

    // Before init — nothing should be set
    assert!(!node.init_called);
    assert!(!node.shutdown_called);
    assert_eq!(node.ticks, 0);

    // After init
    node.init().unwrap();
    assert!(node.init_called);
    assert!(!node.shutdown_called);

    // After some ticks
    node.tick();
    node.tick();
    node.tick();
    assert_eq!(node.ticks, 3);
    assert!(!node.shutdown_called);

    // After shutdown
    node.shutdown().unwrap();
    assert!(node.shutdown_called);
    assert_eq!(node.ticks, 3); // ticks should be unchanged
}
