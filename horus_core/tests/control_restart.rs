//! `horus node restart` must actually re-initialise the node.
//!
//! The CLI sent `PauseNode` immediately followed by `ResumeNode` and printed
//! "The scheduler will re-initialize this node on next tick". It did not.
//! Both arms only toggle a shared atomic; the code that re-runs `init()` —
//! `reinit_pending_nodes` — keys off `RegisteredNode::initialized`, and outside
//! registration nothing in production ever set that back to false. A grep of
//! the workspace found the only two `initialized = false` assignments in
//! `#[cfg(test)]` code.
//!
//! Four places in the runtime asserted the behaviour anyway: the clap help
//! ("re-initialize without killing scheduler"), `restart_node`'s doc comment,
//! an inline comment reading "Pause then resume triggers re-init in the
//! scheduler", and `reinit_pending_nodes`'s own comment, "set by control
//! commands". On a robot the operator restarts a wedged node, is told it
//! worked, and moves on.
//!
//! Run: `cargo test -p horus_core --test control_restart`

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::Node;
use horus_core::scheduling::control::ControlCommand;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

struct CountsInit {
    inits: Arc<AtomicU32>,
}

impl Node for CountsInit {
    fn name(&self) -> &str {
        "restartable"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.inits.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn tick(&mut self) {}
}

/// A `RestartNode` command re-runs `init()` while the scheduler is running.
///
/// Driven through `run_for` rather than `tick_once`: the control topic is
/// created in `run_with_filter`, so under `tick_once` `self.control_topic` is
/// None and no control command is ever polled. That is worth knowing on its
/// own — `horus node pause/resume/restart` reach a scheduler only while it is
/// in `run()`.
#[test]
fn restart_command_reinitialises_the_node() {
    let inits = Arc::new(AtomicU32::new(0));
    let sched_name = format!("restart_test_{}", std::process::id());
    let ctl_name = format!("horus.ctl.{}", sched_name);

    // Open the control topic before the run so the send cannot race setup.
    let ctl: horus_core::communication::Topic<ControlCommand> =
        horus_core::communication::Topic::new_with_kind(
            &ctl_name,
            horus_core::communication::TopicKind::System as u8,
        )
        .expect("control topic opens");

    let inits_for_thread = inits.clone();
    let name_for_thread = sched_name.clone();
    let handle = std::thread::spawn(move || {
        let mut scheduler = Scheduler::new()
            .name(&name_for_thread)
            .tick_rate(200_u64.hz());
        scheduler
            .add(CountsInit {
                inits: inits_for_thread,
            })
            .build()
            .expect("node registers");
        scheduler.run_for(1200_u64.ms()).expect("run_for");
    });

    // Poll for the startup init() rather than sleeping a fixed 400 ms.
    //
    // A fixed wait is a bet that the scheduler thread got scheduled inside it.
    // On a loaded runner — which is the normal state of CI, and of this repo's
    // own workstation under concurrent builds — it does not, and the test fails
    // reporting 0 inits as though `RestartNode` were broken. Polling turns the
    // same wall clock into an upper bound instead of an assumption: it returns
    // as soon as the init lands, and only the diagnostic changes if it never
    // does.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while inits.load(Ordering::SeqCst) < 1 && std::time::Instant::now() < deadline {
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    assert_eq!(
        inits.load(Ordering::SeqCst),
        1,
        "the node should have been initialised once by startup"
    );

    ctl.send(ControlCommand::RestartNode {
        name: "restartable".to_string(),
    });

    handle.join().expect("scheduler thread joins");
    assert_eq!(
        inits.load(Ordering::SeqCst),
        2,
        "RestartNode must clear `initialized` so reinit_pending_nodes re-runs init(); \
         Pause+Resume left it at 1 while the CLI reported success"
    );
}

/// A restart naming an unregistered node must not disturb a registered one.
#[test]
fn restart_for_an_unknown_node_leaves_the_others_alone() {
    let inits = Arc::new(AtomicU32::new(0));
    let sched_name = format!("restart_unknown_{}", std::process::id());
    let ctl_name = format!("horus.ctl.{}", sched_name);

    let ctl: horus_core::communication::Topic<ControlCommand> =
        horus_core::communication::Topic::new_with_kind(
            &ctl_name,
            horus_core::communication::TopicKind::System as u8,
        )
        .expect("control topic opens");

    let inits_for_thread = inits.clone();
    let name_for_thread = sched_name.clone();
    let handle = std::thread::spawn(move || {
        let mut scheduler = Scheduler::new()
            .name(&name_for_thread)
            .tick_rate(200_u64.hz());
        scheduler
            .add(CountsInit {
                inits: inits_for_thread,
            })
            .build()
            .expect("node registers");
        scheduler.run_for(1200_u64.ms()).expect("run_for");
    });

    std::thread::sleep(std::time::Duration::from_millis(400));
    assert_eq!(inits.load(Ordering::SeqCst), 1);

    ctl.send(ControlCommand::RestartNode {
        name: "no_such_node".to_string(),
    });

    handle.join().expect("scheduler thread joins");
    assert_eq!(
        inits.load(Ordering::SeqCst),
        1,
        "a restart naming an unregistered node must not re-init a different one"
    );
}
