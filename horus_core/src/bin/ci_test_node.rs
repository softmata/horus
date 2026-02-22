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

use horus_core::communication::Topic;
use horus_core::core::node::TopicMetadata;
use horus_core::{Node, Scheduler};
use std::time::Duration;

struct CiTestNode {
    topic_name: String,
    publisher: Option<Topic<u64>>,
    counter: u64,
    ready_file: Option<String>,
    ready_written: bool,
}

impl CiTestNode {
    fn new() -> Self {
        let topic_name = std::env::var("HORUS_CI_TOPIC_NAME")
            .unwrap_or_else(|_| "ci_test_topic".to_string());
        let ready_file = std::env::var("HORUS_CI_READY_FILE").ok();

        Self {
            topic_name,
            publisher: None,
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
        let topic = Topic::<u64>::new(&self.topic_name)?;
        self.publisher = Some(topic);
        eprintln!("[ci_test_node] Initialized, publishing on '{}'", self.topic_name);
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

    fn publishers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: self.topic_name.clone(),
            type_name: "u64".to_string(),
        }]
    }
}

fn main() {
    let duration_ms: u64 = std::env::var("HORUS_CI_DURATION_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(5000);

    let mut scheduler = Scheduler::new();
    scheduler.add(CiTestNode::new())
        .order(0)
        .rate_hz(10.0)
        .done();

    eprintln!("[ci_test_node] Running for {}ms", duration_ms);

    if let Err(e) = scheduler.run_for(Duration::from_millis(duration_ms)) {
        eprintln!("[ci_test_node] Error: {}", e);
        std::process::exit(1);
    }

    eprintln!("[ci_test_node] Finished cleanly");
}
