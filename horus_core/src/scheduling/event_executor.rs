//! Event-driven executor for topic-triggered nodes.
//!
//! Nodes registered with `.on("topic_name")` are dispatched here. Each event
//! node gets its own polling thread that watches the associated HORUS topic
//! for new data. When data arrives, the node's `tick()` is called.
//!
//! # Architecture
//!
//! ```text
//!  EventExecutor
//!  ┌──────────────────────────────────────────┐
//!  │  Per-node watcher threads:               │
//!  │  ┌─ thread: watch "lidar_scan" ────────┐ │
//!  │  │  loop: topic.try_recv() → tick()    │ │
//!  │  └─────────────────────────────────────┘ │
//!  │  ┌─ thread: watch "camera_frame" ──────┐ │
//!  │  │  loop: topic.try_recv() → tick()    │ │
//!  │  └─────────────────────────────────────┘ │
//!  │  shared: running (AtomicBool)            │
//!  └──────────────────────────────────────────┘
//! ```
//!
//! The polling interval is short (1ms) to provide responsive triggering
//! while avoiding busy-waiting. Each thread independently checks the
//! shared `running` flag for shutdown.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use crate::terminal::print_line;

use super::fault_tolerance::FailureAction;
use super::primitives::NodeRunner;
use super::types::{ExecutionClass, RegisteredNode, SharedMonitors};

/// Event-driven executor that triggers node ticks based on topic data arrival.
///
/// Each event node is assigned to its own watcher thread. Shutdown is
/// coordinated via the shared `running` flag.
pub(crate) struct EventExecutor {
    handles: Vec<std::thread::JoinHandle<RegisteredNode>>,
}

/// Default polling interval for topic watchers (1ms).
const POLL_INTERVAL: Duration = Duration::from_millis(1);

impl EventExecutor {
    /// Start the event executor with the given event nodes.
    ///
    /// Each node must have `ExecutionClass::Event(topic_name)`. One watcher
    /// thread is spawned per node.
    pub fn start(
        nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        monitors: SharedMonitors,
    ) -> Self {
        let mut handles = Vec::with_capacity(nodes.len());

        for node in nodes {
            let topic_name = match &node.execution_class {
                ExecutionClass::Event(name) => name.clone(),
                _ => {
                    print_line(&format!(
                        "[Event] Warning: node '{}' has non-Event execution class, skipping",
                        node.name
                    ));
                    continue;
                }
            };

            let running = running.clone();
            let node_name = node.name.clone();
            let monitors = monitors.clone();

            let handle = std::thread::Builder::new()
                .name(format!("horus-event-{}", node_name))
                .spawn(move || Self::watcher_thread(node, topic_name, running, monitors))
                .unwrap_or_else(|e| {
                    panic!(
                        "Failed to spawn event watcher thread for '{}': {}",
                        node_name, e
                    )
                });

            handles.push(handle);
        }

        print_line(&format!(
            "[Event] Started {} event watcher threads",
            handles.len()
        ));

        Self { handles }
    }

    /// Stop all watcher threads and reclaim nodes.
    pub fn stop(self) -> Vec<RegisteredNode> {
        let mut nodes = Vec::with_capacity(self.handles.len());
        for handle in self.handles {
            match handle.join() {
                Ok(node) => nodes.push(node),
                Err(_) => {
                    print_line("[Event] Warning: watcher thread panicked during join");
                }
            }
        }
        print_line(&format!(
            "[Event] Stopped ({} nodes returning to scheduler)",
            nodes.len()
        ));
        nodes
    }

    /// Watcher thread main loop for a single event node.
    ///
    /// Polls the topic at `POLL_INTERVAL` cadence. When the topic's generation
    /// counter changes (indicating new data), the node's `tick()` is invoked.
    /// We use a simple generation counter approach: track how many times data
    /// has been received, and tick only when new data is available.
    fn watcher_thread(
        mut node: RegisteredNode,
        topic_name: String,
        running: Arc<AtomicBool>,
        monitors: SharedMonitors,
    ) -> RegisteredNode {
        print_line(&format!(
            "[Event] Watcher for '{}' watching topic '{}'",
            node.name, topic_name
        ));

        // Track tick count via an internal counter
        let mut tick_count: u64 = 0;

        // The event executor uses a notification mechanism:
        // We create a lightweight notifier that external code (or the topic
        // system) can signal. For now, we poll at POLL_INTERVAL using an
        // AtomicU64 counter that publishers can bump.
        //
        // The notifier is stored as a shared counter. When a publisher writes
        // to the topic, the counter is incremented. The watcher compares
        // its last-seen value to detect changes.
        let notifier = Arc::new(AtomicU64::new(0));
        let notifier_for_node = notifier.clone();

        // Store the notifier in the node's context for publishers to find
        // (future integration point — for now, we fall back to periodic polling)
        if let Some(ref mut ctx) = node.context {
            ctx.set_event_notifier(notifier_for_node);
        }

        let mut last_seen_generation: u64 = 0;

        while running.load(Ordering::Relaxed) {
            if !node.initialized || node.is_stopped || node.is_paused {
                std::thread::sleep(POLL_INTERVAL);
                continue;
            }

            // Check if the notifier has been bumped (new data available).
            // Process each missed generation individually — one notification = one tick.
            let current_generation = notifier.load(Ordering::Acquire);
            let missed = current_generation.saturating_sub(last_seen_generation);

            if missed > 0 {
                last_seen_generation = current_generation;

                // Tick once per missed notification
                for _ in 0..missed {
                    if !running.load(Ordering::Relaxed) {
                        break;
                    }

                    // Begin recording tick
                    if let Some(ref mut recorder) = node.recorder {
                        recorder.begin_tick(tick_count);
                    }

                    let tr = NodeRunner::run_tick(&mut node.node);

                    // Record stats
                    if let Some(ref mut stats) = node.rt_stats {
                        stats.record_execution(tr.duration);
                    }
                    monitors
                        .profiler
                        .lock()
                        .unwrap()
                        .record(&node.name, tr.duration);
                    if let Some(ref mut recorder) = node.recorder {
                        recorder.end_tick(tr.duration.as_nanos() as u64);
                    }
                    if let Some(ref mut ctx) = node.context {
                        ctx.record_tick();
                    }

                    match tr.result {
                        Ok(_) => {
                            node.failure_handler.record_success();
                            tick_count += 1;
                        }
                        Err(panic_err) => {
                            let action = node.failure_handler.record_failure();
                            monitors
                                .profiler
                                .lock()
                                .unwrap()
                                .record_node_failure(&node.name);
                            let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                                format!("[Event] Node '{}' panicked: {}", node.name, s)
                            } else if let Some(s) = panic_err.downcast_ref::<String>() {
                                format!("[Event] Node '{}' panicked: {}", node.name, s)
                            } else {
                                format!("[Event] Node '{}' panicked (unknown)", node.name)
                            };
                            print_line(&error_msg);
                            node.node.on_error(&error_msg);

                            match action {
                                FailureAction::StopScheduler
                                | FailureAction::FatalAfterRestarts => {
                                    print_line(&format!(
                                        "[Event] Fatal failure in '{}' — stopping scheduler",
                                        node.name
                                    ));
                                    running.store(false, Ordering::SeqCst);
                                    break;
                                }
                                FailureAction::RestartNode => {
                                    if let Some(ref mut ctx) = node.context {
                                        match ctx.restart() {
                                            Ok(_) => node.initialized = true,
                                            Err(e) => {
                                                print_line(&format!(
                                                    "[Event] Restart failed for '{}': {}",
                                                    node.name, e
                                                ));
                                                node.initialized = false;
                                            }
                                        }
                                    }
                                }
                                FailureAction::SkipNode | FailureAction::Continue => {}
                            }
                        }
                    }
                }
            } else {
                // No new data — sleep before polling again
                std::thread::sleep(POLL_INTERVAL);
            }
        }

        print_line(&format!(
            "[Event] Watcher for '{}' stopped after {} ticks",
            node.name, tick_count
        ));

        node
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Node, NodeInfo};
    use std::sync::Mutex;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
        }
    }

    struct CounterNode {
        name: String,
        count: Arc<AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn make_event_node(name: &str, topic: &str, count: Arc<AtomicU64>) -> RegisteredNode {
        use super::super::fault_tolerance::FailureHandler;

        let node = CounterNode {
            name: name.to_string(),
            count,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::Regular(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: ExecutionClass::Event(topic.to_string()),
            has_custom_failure_policy: false,
        }
    }

    #[test]
    fn test_event_executor_starts_and_stops() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_node", "test_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Let it run briefly — no notifications, so no ticks
        std::thread::sleep(Duration::from_millis(20));
        running.store(false, Ordering::SeqCst);

        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
        // No notifications were sent, so count should be 0
        assert_eq!(count.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_event_executor_ticks_on_notification() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node(
            "evt_node_notify",
            "test_topic",
            count.clone(),
        )];
        let running = Arc::new(AtomicBool::new(true));

        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Wait for the watcher thread to register its notifier in the global registry.
        // On loaded systems the thread may take longer to start.
        let mut registered = false;
        for _ in 0..50 {
            if crate::core::NodeInfo::notify_event("evt_node_notify") {
                registered = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(
            registered,
            "Event node notifier should be registered within 250ms"
        );

        // Wait for the watcher to process the notification we just sent
        std::thread::sleep(Duration::from_millis(50));

        let ticks = count.load(Ordering::Relaxed);
        assert!(
            ticks >= 1,
            "Event node should tick at least once after notification, got {}",
            ticks
        );

        // Send more notifications
        crate::core::NodeInfo::notify_event("evt_node_notify");
        crate::core::NodeInfo::notify_event("evt_node_notify");
        std::thread::sleep(Duration::from_millis(50));

        let ticks = count.load(Ordering::Relaxed);
        assert!(
            ticks >= 3,
            "Event node should tick 3+ times after 3 notifications, got {}",
            ticks
        );

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
    }

    #[test]
    fn test_event_executor_multiple_nodes() {
        let count_a = Arc::new(AtomicU64::new(0));
        let count_b = Arc::new(AtomicU64::new(0));
        let nodes = vec![
            make_event_node("evt_a", "topic_a", count_a.clone()),
            make_event_node("evt_b", "topic_b", count_b.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());
        std::thread::sleep(Duration::from_millis(10));

        // Notify only topic_a's node
        crate::core::NodeInfo::notify_event("evt_a");
        std::thread::sleep(Duration::from_millis(20));

        assert!(count_a.load(Ordering::Relaxed) >= 1, "evt_a should tick");
        assert_eq!(count_b.load(Ordering::Relaxed), 0, "evt_b should NOT tick");

        // Now notify topic_b's node
        crate::core::NodeInfo::notify_event("evt_b");
        std::thread::sleep(Duration::from_millis(20));

        assert!(
            count_b.load(Ordering::Relaxed) >= 1,
            "evt_b should tick after notification"
        );

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 2);
    }

    #[test]
    fn test_event_executor_no_tick_without_data() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_idle", "idle_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Wait without sending any notifications
        std::thread::sleep(Duration::from_millis(50));

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert_eq!(
            count.load(Ordering::Relaxed),
            0,
            "Event node should NOT tick without data"
        );
    }
}
