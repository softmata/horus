//! An external safe-state request must actually safe the robot.
//!
//! `safety.on_link_lost = "safe_state"` routes a lost network link to
//! `trigger_external_safe_state`, whose installed hook set
//! `SafetyState::SafeState` and printed
//!
//!     SAFE STATE (external): ... — nodes safing, scheduler continues
//!
//! Nothing read that state. The hook's own write was the only reference to the
//! variant outside its own module, so the operator who deliberately chose
//! per-node safing over a full halt got neither: the robot kept executing its
//! last command with a reassuring line on stderr.
//!
//! This lives in its own test binary because the safe-state hook is a
//! process-global installed by a running scheduler; sharing a binary with other
//! scheduler tests would let their schedulers claim the hook.

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::node::Node;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

struct SafeStateRecordingNode {
    safed: Arc<AtomicU64>,
    ticks: Arc<AtomicU64>,
}

impl Node for SafeStateRecordingNode {
    fn name(&self) -> &str {
        "actuator"
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
    fn enter_safe_state(&mut self) {
        self.safed.fetch_add(1, Ordering::Relaxed);
    }
}

/// One test, not two, because the safe-state hook is a PROCESS-GLOBAL installed
/// by whichever scheduler is running. Two tests in this binary raced for it:
/// one scheduler claimed the hook and absorbed both triggers while the other
/// saw none, so the pair failed about half the time for a reason that had
/// nothing to do with the code under test.
#[test]
fn an_external_safe_state_request_safes_the_robot_exactly_once() {
    let safed = Arc::new(AtomicU64::new(0));
    let ticks = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());
    scheduler
        .add(SafeStateRecordingNode {
            safed: safed.clone(),
            ticks: ticks.clone(),
        })
        .build()
        .unwrap();

    // Fire the external trigger once the scheduler is running and has installed
    // its hook. Before it is installed, `trigger_external_safe_state` escalates
    // to an emergency stop instead, which is a different path.
    std::thread::spawn(|| {
        std::thread::sleep(std::time::Duration::from_millis(120));
        horus_core::scheduling::trigger_external_safe_state("test: peer lost".to_string());
    });

    scheduler.run_for(600_u64.ms()).unwrap();

    let n = safed.load(Ordering::Relaxed);
    let t = ticks.load(Ordering::Relaxed);

    assert!(
        t > 0,
        "the node never ticked, so this run exercised nothing"
    );

    // The fix: the request is consumed and the node is actually safed.
    assert!(
        n > 0,
        "an external safe-state request was raised and no node was safed. The \
         hook set SafetyState::SafeState and printed 'nodes safing', and nothing \
         consumed it — which is what a robot losing its link to the fleet \
         controller under on_link_lost=safe_state used to get."
    );

    // And it is an EDGE, not a level: one link loss produces one round of
    // safing, not a fresh `enter_safe_state()` every tick for the rest of the
    // run. `SafetyState::SafeState` stays latched as an observable condition,
    // which is why consuming the state itself would have been the wrong fix.
    assert!(
        n < t / 4,
        "safed {n} times over {t} ticks — the request is being re-consumed every \
         tick instead of once per raise"
    );
}
