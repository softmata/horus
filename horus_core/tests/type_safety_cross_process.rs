//! Type safety test — verifies that opening a topic with the wrong type
//! is caught and rejected with a clear error message.
//!
//! Before this fix, Process A creating Topic<CmdVel>("motor.cmd") and
//! Process B opening Topic<LaserScan>("motor.cmd") would silently
//! reinterpret bytes as the wrong type — causing data corruption.

use horus_core::communication::topic::Topic;
use serde::{Deserialize, Serialize};

mod common;
use common::{cleanup_stale_shm, unique};

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct TypeA {
    x: f64,
    y: f64,
}
unsafe impl bytemuck::Pod for TypeA {}
unsafe impl bytemuck::Zeroable for TypeA {}

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[repr(C)]
struct TypeB {
    a: f64,
    b: f64,
}
unsafe impl bytemuck::Pod for TypeB {}
unsafe impl bytemuck::Zeroable for TypeB {}

#[test]
fn type_mismatch_rejected_same_process() {
    cleanup_stale_shm();
    let name = unique("type_safety");

    // Create topic with TypeA
    let _t1: Topic<TypeA> = Topic::new(&name).expect("TypeA should succeed");
    // Trigger initialization by sending
    _t1.send(TypeA { x: 1.0, y: 2.0 });

    // Try to open same topic with TypeB — should fail
    let result = Topic::<TypeB>::new(&name);
    match result {
        Err(e) => {
            let msg = format!("{}", e);
            println!("Correctly rejected type mismatch: {}", msg);
            assert!(
                msg.contains("Type mismatch") || msg.contains("type mismatch"),
                "Error should mention type mismatch, got: {}",
                msg
            );
        }
        Ok(_) => {
            panic!(
                "CRITICAL BUG: Topic<TypeB> was allowed to open a Topic<TypeA> — \
                    this would cause silent data corruption on a real robot!"
            );
        }
    }
}

#[test]
fn same_type_accepted() {
    cleanup_stale_shm();
    let name = unique("type_ok");

    let t1: Topic<TypeA> = Topic::new(&name).expect("first open");
    t1.send(TypeA { x: 1.0, y: 2.0 });

    // Same type should be accepted
    let t2: Topic<TypeA> = Topic::new(&name).expect("second open with same type should succeed");
    let msg = t2.recv();
    assert!(msg.is_some(), "Should receive the message from t1");
    println!("Same-type topic sharing works correctly ✓");
}
