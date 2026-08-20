//! CI Test Node — Standalone binary for integration testing
//!
//! A minimal HORUS node that publishes u64 counters at 10Hz.
//! Used by CI to verify node discovery (`horus node list`) and
//! topic listing (`horus topic list`) with live running nodes.
//!
//! ## Environment Variables
//!
//! - `HORUS_CI_TOPIC_NAME` — Topic name to publish on (default: `ci_test_topic`)
//! - `HORUS_CI_DURATION_MS` — How long to run in milliseconds (default: 5000)
//! - `HORUS_CI_READY_FILE` — Path to write when node is ready (default: none)
//! - `HORUS_CI_TOPIC_IN_CTOR` — Create the `Topic` in `new()` rather than
//!   `init()`. This is what node constructors normally do, and it runs before
//!   the scheduler has installed any node context — so the owner name the
//!   `Topic` captured used to be nothing at all.
//! - `HORUS_CI_BEST_EFFORT` — Omit `.rate()`, so the node stays on the main
//!   thread instead of being promoted to `ExecutionClass::Rt`. The two paths
//!   report their metrics through different mechanisms and have failed
//!   differently, so tests need to pick one.

use horus_core::communication::Topic;
use horus_core::core::DurationExt;
use horus_core::{Node, Scheduler};

struct CiTestNode {
    topic_name: String,
    publisher: Option<Topic<u64>>,
    counter: u64,
    ready_file: Option<String>,
    ready_written: bool,
}

impl CiTestNode {
    fn new() -> Self {
        let topic_name =
            std::env::var("HORUS_CI_TOPIC_NAME").unwrap_or_else(|_| "ci_test_topic".to_string());
        let ready_file = std::env::var("HORUS_CI_READY_FILE").ok();

        // Constructing the Topic here, outside any node context, is the normal
        // shape of a node constructor and the case that used to lose the owner.
        let publisher = std::env::var("HORUS_CI_TOPIC_IN_CTOR")
            .is_ok()
            .then(|| Topic::<u64>::new(&topic_name).expect("topic"));

        Self {
            topic_name,
            publisher,
            counter: 0,
            ready_file,
            ready_written: false,
        }
    }
}

impl Node for CiTestNode {
    fn name(&self) -> &str {
        "ci_test_node"
    }

    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        if self.publisher.is_none() {
            self.publisher = Some(Topic::<u64>::new(&self.topic_name)?);
        }
        eprintln!(
            "[ci_test_node] Initialized, publishing on '{}'",
            self.topic_name
        );
        Ok(())
    }

    fn tick(&mut self) {
        if let Some(ref publisher) = self.publisher {
            publisher.send(self.counter);
            self.counter += 1;
        }

        // Write ready file after first successful tick
        if !self.ready_written {
            if let Some(ref path) = self.ready_file {
                if std::fs::write(path, "ready").is_ok() {
                    eprintln!("[ci_test_node] Ready file written to {}", path);
                }
            }
            self.ready_written = true;
        }
    }

    fn shutdown(&mut self) -> horus_core::error::HorusResult<()> {
        eprintln!("[ci_test_node] Shutting down after {} ticks", self.counter);
        // Clean up ready file
        if let Some(ref path) = self.ready_file {
            let _ = std::fs::remove_file(path);
        }
        Ok(())
    }
}

fn main() {
    let duration_ms: u64 = std::env::var("HORUS_CI_DURATION_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(5000);

    let mut scheduler = Scheduler::new();
    // `.rate()` promotes the node to ExecutionClass::Rt, which moves it off the
    // main thread and onto the RT executor. Both paths must report correctly.
    if std::env::var("HORUS_CI_BEST_EFFORT").is_ok() {
        scheduler
            .add(CiTestNode::new())
            .order(0)
            .build()
            .expect("invalid node configuration");
    } else {
        scheduler
            .add(CiTestNode::new())
            .order(0)
            .rate(10_u64.hz())
            .build()
            .expect("invalid node configuration");
    }

    eprintln!("[ci_test_node] Running for {}ms", duration_ms);

    if let Err(e) = scheduler.run_for(duration_ms.ms()) {
        eprintln!("[ci_test_node] Error: {}", e);
        std::process::exit(1);
    }

    eprintln!("[ci_test_node] Finished cleanly");
}
