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
//!  ┌──────────────────────────────────────────────────────┐
//!  │  Per-node watcher threads:                           │
//!  │  ┌─ thread: watch "lidar_scan" ────────────────────┐ │
//!  │  │  loop: park on notifier → tick() per generation │ │
//!  │  └─────────────────────────────────────────────────┘ │
//!  │  ┌─ thread: watch "camera_frame" ──────────────────┐ │
//!  │  │  loop: park on notifier → tick() per generation │ │
//!  │  └─────────────────────────────────────────────────┘ │
//!  │  shared: running (AtomicBool), one notifier per node │
//!  └──────────────────────────────────────────────────────┘
//! ```
//!
//! # Wake-up
//!
//! A watcher **blocks on the notifier the publisher already bumps**. Every
//! `Topic::send()` calls `NodeInfo::notify_event(topic)`, which increments that
//! topic's `EventNotifier` generation and — only when a watcher is actually
//! parked on it — issues a `FUTEX_WAKE`. The watcher ticks once per generation
//! it missed.
//!
//! This replaced a `thread::sleep(1 ms)` poll of the same counter. That gave
//! every event node a wake latency of `uniform(0, 1 ms)` plus timer slack —
//! three to four orders of magnitude above the ~300 ns the transport delivers,
//! with the entire millisecond as jitter, on a mechanism the publisher was
//! already paying for. It also burned a wake-up per millisecond per event node
//! on a completely idle robot.
//!
//! [`WAKE_BACKSTOP`] is a **backstop, not the mechanism**: if a wake is ever
//! lost, the watcher falls back to polling. A missing wake is a latency event,
//! never a node that stops ticking. See [`IDLE_BACKSTOP_MAX`] for the one
//! guarantee this costs, stated plainly.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use crate::core::node::EventNotifier;
use crate::terminal::print_line;

use super::primitives::NodeRunner;
use super::types::{ExecutionClass, RegisteredNode, SharedMonitors};

/// Event-driven executor that triggers node ticks based on topic data arrival.
///
/// Each event node is assigned to its own watcher thread. Shutdown is
/// coordinated via the shared `running` flag.
pub(crate) struct EventExecutor {
    handles: Vec<std::thread::JoinHandle<RegisteredNode>>,
    /// One per live watcher thread, in the same order as `handles`.
    ///
    /// Held solely so shutdown can wake a parked watcher instead of waiting
    /// out its backstop. Without this the executor would still stop within
    /// `WAKE_BACKSTOP` per node — correct, but no better than the sleep-poll
    /// it replaced.
    notifiers: Vec<Arc<EventNotifier>>,
}

/// Upper bound on how long a watcher blocks without hearing from the publisher.
///
/// This is a **backstop**, not a polling cadence: in normal operation the
/// watcher is woken by the publisher's `FUTEX_WAKE` and this timeout never
/// expires. It exists so that a lost wake-up — a platform with no futex, a
/// notifier that moved to a mapping this process cannot be woken on, a
/// publisher that read a stale waiter count — degrades a node to the 1 ms
/// polling latency it used to have unconditionally, instead of stalling it.
///
/// Deliberately the same 1 ms the old sleep-poll used, so a lost wake on an
/// *active* topic costs exactly what every wake used to cost.
const WAKE_BACKSTOP: Duration = Duration::from_millis(1);

/// Consecutive fruitless waits before the backstop is allowed to lengthen.
///
/// 16 at [`WAKE_BACKSTOP`] is ~16 ms of complete silence on the topic. Any
/// notification at all resets this, so a node receiving data at any rate above
/// ~60 Hz never reaches it and never leaves the 1 ms backstop.
const SILENT_WAITS_BEFORE_BACKOFF: u32 = 16;

/// Ceiling on the lengthened backstop for a topic that has gone quiet.
///
/// # This constant is a trade, and here is what it costs
///
/// A fixed 1 ms backstop and "an idle event node consumes zero CPU" are not
/// simultaneously satisfiable: a 1 ms timeout means the waiter wakes 1000
/// times a second whether or not anything happened, which is precisely the
/// cost of the `thread::sleep(1 ms)` loop this change exists to remove. Ten
/// idle event nodes would still be 10,000 wake-ups a second, and 10,000
/// chances a second to preempt something that matters, on a robot that is
/// doing nothing.
///
/// So the backstop starts at [`WAKE_BACKSTOP`] and doubles to this ceiling
/// only after [`SILENT_WAITS_BEFORE_BACKOFF`] fruitless waits in a row. Idle
/// cost drops from ~1000 wake-ups/s/node to ~31.
///
/// **What it costs:** if the wake mechanism is ever broken — not merely slow,
/// broken — a node whose topic has been silent for more than ~16 ms will see
/// its *first* message up to 32 ms late instead of up to 1 ms late. Every
/// message after that is back to 1 ms until the topic goes quiet again. This
/// is a real widening of the worst case in one specific failure mode, taken
/// deliberately, in exchange for removing a permanent per-node timer.
///
/// **What it deliberately does NOT cost: safing latency.** The watcher also
/// polls `honor_safe_state_request` once per loop iteration, so lengthening
/// the wait would lengthen the time between the watchdog ladder raising a
/// safe-state request and the node acting on it — a safety guarantee, not a
/// performance number, and not something to trade for idle CPU. The
/// escalation is therefore switched off entirely for any node that can
/// receive such a request; see `backstop_may_escalate`.
///
/// **The one other consumer of the loop cadence** is the `is_paused` /
/// `is_stopped` gate, which a parked watcher also only re-reads on waking. A
/// pause therefore takes up to the current backstop to be *observed*. It takes
/// effect immediately in the only sense that matters — a parked node is by
/// definition not ticking, and the first publish that would have ticked it
/// wakes it into the gate — so this is a control-plane acknowledgement delay,
/// not a node that keeps running after being told to stop.
///
/// **Why that failure mode should not occur on Linux in-process:** a lost
/// wake-up is not reachable. The waiter hands `FUTEX_WAIT` the exact
/// generation it read, and the kernel re-compares the word under the bucket
/// lock before queueing the thread, so a publish that lands before queueing
/// returns `EAGAIN` rather than being slept through; a publish that lands
/// after queueing is ordered after the waiter's SeqCst increment of the waiter
/// count and therefore sees it. See `EventNotifier::publish`. The backstop
/// exists for the cases outside that proof: platforms with no futex (where it
/// is *the* delivery mechanism, and where the escalation below is compiled
/// out), and a future move of the counter into shared memory.
#[cfg(target_os = "linux")]
const IDLE_BACKSTOP_MAX: Duration = Duration::from_millis(32);

/// On a platform with no futex the timeout is not a backstop, it **is** the
/// delivery mechanism (`EventNotifier::wait_for_change` degenerates to a
/// sleep). Lengthening it there would not save a wake-up that a publish would
/// otherwise have cancelled — it would simply make every event node late. So
/// the escalation is compiled out by making the ceiling the floor.
#[cfg(not(target_os = "linux"))]
const IDLE_BACKSTOP_MAX: Duration = WAKE_BACKSTOP;

/// Re-check cadence for a node that is not currently tickable.
///
/// Separate constant from [`WAKE_BACKSTOP`] despite the identical value,
/// because it answers a different question. `is_paused`, `is_stopped` and the
/// failure-policy gate are written by *other* threads and are not published
/// through the notifier, so a paused node cannot park on it — resuming is not
/// a publish, and the topic it watches may be silent for hours.
const IDLE_RECHECK_INTERVAL: Duration = Duration::from_millis(1);

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
        let mut notifiers = Vec::with_capacity(nodes.len());

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

            // Created here rather than inside the watcher so the executor keeps
            // a handle on it: `stop()` needs to wake a parked watcher, and it
            // cannot reach one the watcher owns privately.
            let notifier = Arc::new(EventNotifier::new());
            let watcher_notifier = Arc::clone(&notifier);

            let handle = match std::thread::Builder::new()
                .name(format!("horus-event-{}", node_name))
                .spawn(move || {
                    Self::watcher_thread(node, topic_name, watcher_notifier, running, monitors)
                }) {
                Ok(h) => h,
                Err(e) => {
                    print_line(&format!(
                        "[Event] ERROR: Failed to spawn watcher thread for '{}': {}",
                        node_name, e
                    ));
                    // `notifier` drops here; nothing registered it, so no
                    // publisher can reach a counter with no watcher behind it.
                    continue;
                }
            };

            handles.push(handle);
            notifiers.push(notifier);
        }

        print_line(&format!(
            "[Event] Started {} event watcher threads",
            handles.len()
        ));

        Self { handles, notifiers }
    }

    /// Stop all watcher threads and reclaim nodes.
    ///
    /// Bounded, matching the guarantee `RtExecutor::stop` documents. A bare
    /// `handle.join()` here hung the whole scheduler shutdown whenever an event
    /// node blocked inside `tick()`, and `run_with_filter` runs the executor
    /// stops before it shuts down or safes any other node.
    ///
    /// The budget is a single overall deadline rather than one per thread:
    /// there is a watcher thread PER event node, so a per-thread budget would
    /// scale the worst case with the node count.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        // Release every parked watcher before joining. `running` is already
        // false by the time the scheduler calls this, so a woken watcher sees
        // an unchanged generation, re-tests the flag and exits — it does not
        // tick on the way out.
        //
        // Unconditional, with no waiter-count gate: this is shutdown, once per
        // node, and skipping the syscall on a stale count would trade it for
        // up to a `WAKE_BACKSTOP` of shutdown latency. The gate exists to keep
        // syscalls off the *publish* path, which this is not.
        self.wake_watchers();

        let deadline = std::time::Instant::now() + super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD;
        let mut nodes = Vec::with_capacity(self.handles.len());
        for handle in self.handles.drain(..) {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if let Some(node) = super::primitives::join_with_timeout(handle, "Event", remaining) {
                nodes.push(node);
            }
        }
        // Every thread is joined or detached; `Drop` has nothing left to wake.
        self.notifiers.clear();
        print_line(&format!(
            "[Event] Stopped ({} nodes returning to scheduler)",
            nodes.len()
        ));
        nodes
    }

    /// Wake every watcher parked on its notifier without moving any generation.
    ///
    /// Moving a generation here would make a stopping node tick once more,
    /// which is the opposite of what shutdown wants.
    fn wake_watchers(&self) {
        for notifier in &self.notifiers {
            notifier.wake_all();
        }
    }

    /// Watcher thread main loop for a single event node.
    ///
    /// Blocks on the node's [`EventNotifier`] until a publisher bumps its
    /// generation, then invokes `tick()` once per generation missed. Idle costs
    /// nothing: no timer, no wake-up, no CPU.
    fn watcher_thread(
        mut node: RegisteredNode,
        topic_name: String,
        notifier: Arc<EventNotifier>,
        running: Arc<AtomicBool>,
        monitors: SharedMonitors,
    ) -> RegisteredNode {
        print_line(&format!(
            "[Event] Watcher for '{}' watching topic '{}'",
            node.name, topic_name
        ));

        // Track tick count via an internal counter
        let mut tick_count: u64 = 0;

        // Seed BEFORE registration. The notifier is unreachable to publishers
        // until the line below puts it in the registry, so nothing can be
        // missed here; reading it after registering would instead drop any
        // notification that landed in between.
        let mut last_seen_generation: u32 = notifier.generation();

        // Backstop state. Reset to `WAKE_BACKSTOP` by any notification, so a
        // topic that is delivering never sits on a lengthened timeout.
        let mut backstop = WAKE_BACKSTOP;
        let mut silent_waits: u32 = 0;

        // Whether this node may let its backstop grow while its topic is
        // silent — see IDLE_BACKSTOP_MAX for the full argument.
        //
        // The loop below polls `honor_safe_state_request` once per iteration,
        // so the backstop is also the worst-case delay between the watchdog
        // ladder raising a safe-state request and this node entering its safe
        // state. That is a safety guarantee and is not available to trade for
        // idle CPU, so a node that can be asked to safe keeps the 1 ms
        // backstop it has always had — and with it the old idle wake-up rate.
        //
        // The condition is deliberately the same one the tick loop uses to
        // decide whether it owes the watchdog a feed, and a superset of the
        // registration test in `Scheduler::apply_safety_config`
        // (`add_critical_node` is reached only for an RT node under an active
        // watchdog, or any node with an explicit `.watchdog()`). A node
        // outside it is never handed to `SafetyMonitor::add_critical_node`, so
        // `request_safe_state` can never name it and there is nothing for a
        // longer wait to be late for.
        let backstop_may_escalate = !(node.is_rt_node || node.node_watchdog.is_some());

        // Register the notifier under both the node name and the topic name, so
        // `Topic::send()` — which knows its own name but not its subscribers —
        // can reach it. This is the Topic::send() → event tick wiring.
        if let Some(ref mut ctx) = node.context {
            ctx.set_event_notifier_with_topic(Arc::clone(&notifier), &topic_name);
        }

        while running.load(Ordering::Relaxed) {
            // Safing requested by the main thread's watchdog ladder. Checked
            // before the gates below — an Isolated node is precisely one that
            // is not ticking, so gating this on tickability would never run it.
            super::primitives::honor_safe_state_request(&mut node, &monitors);

            if !node.initialized
                || node.is_stopped
                || node.is_paused
                || !node.failure_policy_allows_tick()
            {
                // Not tickable. Deliberately a timed sleep and not a wait on
                // the notifier: these flags are set by other threads and are
                // never published through it, so parking here would make a
                // resume wait for the next message on a topic that may be
                // silent. Note the notifier keeps counting while we are here,
                // so nothing published during a pause is lost — it is
                // delivered as missed generations on resume, exactly as
                // before.
                std::thread::sleep(IDLE_RECHECK_INTERVAL);
                continue;
            }

            // One notification = one tick, and the counter is a *generation*:
            // `wrapping_sub` is required, not cosmetic, because it is 32 bits
            // wide (the width `FUTEX_WAIT` compares) and is allowed to wrap.
            let current_generation = notifier.generation();
            let missed = current_generation.wrapping_sub(last_seen_generation);

            if missed == 0 {
                // Nothing published since our last tick. Block on the notifier
                // the publisher already bumps rather than sleeping a
                // millisecond and looking again: an idle event node now costs
                // zero CPU and zero wake-ups, and reacts one scheduler wake-up
                // after the publish instead of `uniform(0, 1 ms)` after it.
                //
                // `current_generation` — the value we just read — is what the
                // kernel re-compares under its bucket lock, so a publish that
                // lands between the read and the syscall returns EAGAIN
                // immediately instead of being slept through.
                //
                // `running` is passed in so a `stop()` racing us is caught
                // after we announce ourselves as a waiter, not before.
                notifier.wait_for_change(current_generation, backstop, &running);

                // That wait produced nothing (either it timed out, or it was a
                // shutdown/spurious wake — a real publish would have moved the
                // generation and we would not be here). Lengthen the backstop
                // once the topic has been silent long enough that a
                // millisecond timer per node stops being worth its cost. See
                // IDLE_BACKSTOP_MAX for what this trades away.
                silent_waits = silent_waits.saturating_add(1);
                if backstop_may_escalate && silent_waits > SILENT_WAITS_BEFORE_BACKOFF {
                    backstop = (backstop * 2).min(IDLE_BACKSTOP_MAX);
                }
                continue;
            }

            // Data. Drop straight back to the tight backstop: this topic is
            // live, so a lost wake here must cost 1 ms, not 32.
            silent_waits = 0;
            backstop = WAKE_BACKSTOP;

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

                // FIX #5: this tick runs on THIS watcher thread — install the
                // per-tick context here so horus::now/dt/rng/budget_remaining
                // work for event nodes too (mirrors run_node_tick).
                super::primitives::set_node_tick_context(
                    &node,
                    &*monitors.clock,
                    monitors.tick_period,
                );
                let tr = NodeRunner::run_tick(&mut node.node);
                super::primitives::clear_node_tick_context();

                // Record stats
                if let Some(ref mut stats) = node.rt_stats {
                    stats.record_execution(tr.duration);
                }
                // Use try_lock to avoid priority inversion — skip if contended or poisoned
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record(&node.name, tr.duration);
                }
                if let Some(ref mut recorder) = node.recorder {
                    recorder.end_tick(tr.duration.as_nanos() as u64);
                }
                // Update live SHM registry (~5ns atomic writes)
                monitors.update_registry(&node, tr.duration.as_nanos() as u64);
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }

                match tr.result {
                    Ok(_) => {
                        tick_count += 1;
                        node.record_tick_success();
                        // FIX #2: feed the watchdog after a successful tick.
                        // Event nodes are critical only via an explicit
                        // `.watchdog()` (is_rt_node is false here). A hung
                        // tick never returns and a panic lands in Err(_), so
                        // neither refreshes the watchdog.
                        if node.is_rt_node || node.node_watchdog.is_some() {
                            if let Some(ref feeder) = monitors.watchdog {
                                feeder.feed(&node.name);
                            }
                        }
                    }
                    Err(panic_err) => {
                        if let Ok(mut profiler) = monitors.profiler.try_lock() {
                            profiler.record_node_failure(&node.name);
                        }
                        let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                            format!("[Event] Node '{}' panicked: {}", node.name, s)
                        } else if let Some(s) = panic_err.downcast_ref::<String>() {
                            format!("[Event] Node '{}' panicked: {}", node.name, s)
                        } else {
                            format!("[Event] Node '{}' panicked (unknown)", node.name)
                        };
                        // Count the failure. Only the Ok arm recorded metrics, so a node
                        // panicking on every tick reported `Health: Healthy, Errors: 0,
                        // Total Ticks: 0` while its P99 timing was recorded correctly —
                        // making the zero read as "idle" rather than "dead".
                        if let Some(ref mut ctx) = node.context {
                            ctx.record_tick_failure(error_msg.clone());
                        }

                        // Reflect sustained failure in the node's health state. The
                        // watchdog/deadline ladder is the only other writer, so a node that
                        // panicked every tick but never missed a *timing* target reported
                        // `Health: Healthy` forever — 232 errors out of 237 ticks, green.
                        //
                        // One panic can be transient; 3 consecutive is a state change.
                        if let Some(ref ctx) = node.context {
                            if ctx.consecutive_failures()
                                >= super::primitives::FAILURES_BEFORE_UNHEALTHY
                            {
                                node.health_state
                                    .store(super::types::NodeHealthState::Unhealthy);
                                monitors.node_controls.set_health(
                                    node.name.as_ref(),
                                    super::types::NodeHealthState::Unhealthy,
                                );
                            }
                        }

                        // Record to the blackbox so the flight recorder can see a crash.
                        // try_lock mirrors the RT path: never block an executor on it.
                        if let Some(ref bb) = monitors.blackbox {
                            if let Ok(mut bb) = bb.try_lock() {
                                bb.record(super::blackbox::BlackBoxEvent::NodeError {
                                    name: node.name.to_string(),
                                    error: error_msg.clone(),
                                    severity: crate::error::Severity::Fatal,
                                });
                            }
                        }

                        // `record_tick_failure` above already logged this at error level, which
                        // reaches both the console and the buffer `horus log` reads. This second
                        // copy was gated on `verbose` on the theory that verbose is opt-in — but
                        // `MonitoringConfig::verbose` defaults to *true*, so every default run
                        // printed the panic twice on two different streams (hlog to stderr, this to
                        // stdout), and a third time via the old `Node::on_error` default. With a
                        // Python node's traceback attached that is three multi-line blocks for one
                        // failure.
                        node.node.on_error(&error_msg);

                        // Enforce the failure policy (Fatal → safe + stop via
                        // shared `running`; Restart → re-init; Skip → gated).
                        if node.apply_failure_policy_after_panic() {
                            running.store(false, Ordering::SeqCst);
                            break;
                        }
                    }
                }
            }
        }

        print_line(&format!(
            "[Event] Watcher for '{}' stopped after {} ticks",
            node.name, tick_count
        ));

        node
    }
}

impl Drop for EventExecutor {
    fn drop(&mut self) {
        // Bounded here too: an early return or a panic can drop the executor
        // without ever calling `stop()`, and an unbounded join on that path
        // hangs exactly as badly.
        // Same reason as in `stop()`: an unwoken watcher would sit in its
        // backstop and burn the join budget for no reason.
        self.wake_watchers();
        let deadline = std::time::Instant::now() + super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD;
        for handle in self.handles.drain(..) {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            let _ = super::primitives::join_with_timeout(handle, "Event", remaining);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::{Miss, Node, NodeInfo};
    use std::sync::atomic::AtomicU64;
    use std::sync::Mutex;
    use std::time::Instant;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
            registry: None,
            registry_slots: Arc::new(std::collections::HashMap::new()),
            node_controls: Arc::new(super::super::types::NodeControlMap::default()),
            clock: Arc::new(crate::core::clock::WallClock::new()),
            tick_period: Duration::from_millis(1),
            watchdog: None,
            estop: None,
            safety: None,
        }
    }

    struct CounterNode {
        name: String,
        count: Arc<AtomicU64>,
        safed: Option<Arc<AtomicBool>>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        fn enter_safe_state(&mut self) {
            if let Some(ref safed) = self.safed {
                safed.store(true, Ordering::SeqCst);
            }
        }
    }

    fn make_event_node(name: &str, topic: &str, count: Arc<AtomicU64>) -> RegisteredNode {
        let node = CounterNode {
            name: name.to_string(),
            count,
            safed: None,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: false,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: ExecutionClass::Event(topic.to_string()),
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    #[test]
    fn test_event_executor_starts_and_stops() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_node", "test_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Let it run briefly — no notifications, so no ticks
        std::thread::sleep(20_u64.ms());
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
            std::thread::sleep(5_u64.ms());
        }
        assert!(
            registered,
            "Event node notifier should be registered within 250ms"
        );

        // Wait for the watcher to process the notification we just sent
        std::thread::sleep(50_u64.ms());

        let ticks = count.load(Ordering::Relaxed);
        assert!(
            ticks >= 1,
            "Event node should tick at least once after notification, got {}",
            ticks
        );

        // Send more notifications
        crate::core::NodeInfo::notify_event("evt_node_notify");
        crate::core::NodeInfo::notify_event("evt_node_notify");
        std::thread::sleep(50_u64.ms());

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
        std::thread::sleep(10_u64.ms());

        // Notify only topic_a's node
        crate::core::NodeInfo::notify_event("evt_a");
        std::thread::sleep(20_u64.ms());

        assert!(count_a.load(Ordering::Relaxed) >= 1, "evt_a should tick");
        assert_eq!(count_b.load(Ordering::Relaxed), 0, "evt_b should NOT tick");

        // Now notify topic_b's node
        crate::core::NodeInfo::notify_event("evt_b");
        std::thread::sleep(20_u64.ms());

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
        std::thread::sleep(50_u64.ms());

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert_eq!(
            count.load(Ordering::Relaxed),
            0,
            "Event node should NOT tick without data"
        );
    }

    /// Spin on a counter without adding measurable latency to what we measure.
    fn await_count(count: &AtomicU64, target: u64, limit: Duration) -> bool {
        let start = Instant::now();
        while count.load(Ordering::Acquire) < target {
            if start.elapsed() > limit {
                return false;
            }
            std::hint::spin_loop();
        }
        true
    }

    /// The whole point of research 1.16: an event node must be woken by the
    /// publisher, not discovered by a timer.
    ///
    /// Measured as a strictly serial ping-pong — notify, wait for the tick,
    /// notify again — so each round trip contains exactly one wake. Under the
    /// old `thread::sleep(1 ms)` poll each round trip cost `uniform(0, 1 ms)`,
    /// i.e. ~500 us on average and ~25 ms for the 50 below, with the sample
    /// mean concentrating hard around that by the CLT. The budget here is
    /// 10 ms — 2.5x the ~1-3 ms a futex wake actually costs for 50 trips, and
    /// 2.5x *under* what polling could achieve on its best day. It cannot be
    /// passed by the mechanism this replaced.
    #[test]
    fn wake_comes_from_the_publisher_not_from_the_backstop() {
        const TRIPS: u64 = 50;

        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_wake_latency", "wake_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));
        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Wait out thread start-up and registration; nothing below should be
        // measuring either.
        let mut registered = false;
        for _ in 0..200 {
            if crate::core::NodeInfo::notify_event("evt_wake_latency") {
                registered = true;
                break;
            }
            std::thread::sleep(5_u64.ms());
        }
        assert!(registered, "watcher never registered its notifier");
        assert!(
            await_count(&count, 1, 2_000_u64.ms()),
            "the first notification never produced a tick"
        );

        let start = Instant::now();
        for trip in 2..=TRIPS {
            assert!(
                crate::core::NodeInfo::notify_event("evt_wake_latency"),
                "notifier vanished from the registry mid-test"
            );
            assert!(
                await_count(&count, trip, 2_000_u64.ms()),
                "tick {trip} never arrived"
            );
        }
        let elapsed = start.elapsed();

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 1);

        let budget = Duration::from_millis(12);
        assert!(
            elapsed < budget,
            "{} serial notify->tick round trips took {:?} ({:?} each). The old \
             sleep-poll averaged ~500us per trip (~{:?} for this many), so this \
             is the signature of the watcher waiting out WAKE_BACKSTOP instead \
             of being woken by the publisher's FUTEX_WAKE.",
            TRIPS - 1,
            elapsed,
            elapsed / (TRIPS as u32 - 1),
            Duration::from_micros(500) * (TRIPS as u32 - 1),
        );
    }

    /// The idle backstop must never delay safing.
    ///
    /// The watcher polls `honor_safe_state_request` once per loop iteration, so
    /// however long it blocks is also the worst-case delay between the watchdog
    /// ladder raising a safe-state request and this node reaching its safe
    /// state. Letting an idle node's backstop grow to `IDLE_BACKSTOP_MAX` would
    /// therefore make safing up to 32x slower — a safety guarantee traded for
    /// idle CPU. `backstop_may_escalate` exists to forbid exactly that for any
    /// node the watchdog ladder can name, and this test is what holds it shut.
    ///
    /// The node here carries an explicit `.watchdog()`, which is the condition
    /// `Scheduler::apply_safety_config` uses to register a non-RT node with the
    /// safety monitor — i.e. precisely the population that can receive a
    /// request.
    #[test]
    fn a_watchdog_node_is_safed_promptly_even_while_its_topic_is_silent() {
        let count = Arc::new(AtomicU64::new(0));
        let safed = Arc::new(AtomicBool::new(false));
        let mut node = make_event_node("evt_safing", "safing_topic", count.clone());
        node.node = super::super::types::NodeKind::new(Box::new(CounterNode {
            name: "evt_safing".to_string(),
            count: count.clone(),
            safed: Some(safed.clone()),
        }));
        // Explicit per-node watchdog: this is what puts a non-RT node under
        // `add_critical_node`, and therefore in reach of `request_safe_state`.
        node.node_watchdog = Some(Duration::from_millis(50));

        let monitors = test_monitors();
        let controls = monitors.node_controls.clone();
        let running = Arc::new(AtomicBool::new(true));
        let executor = EventExecutor::start(vec![node], running.clone(), monitors);

        // Let the watcher reach its wait, and stay there long enough that an
        // ungated backstop would have escalated to the 32 ms ceiling.
        std::thread::sleep(300_u64.ms());

        // The ladder only reaches nodes present in the control map; the real
        // scheduler registers all five groups there at start-up.
        controls.register("evt_safing");

        let start = Instant::now();
        controls.request_safe_state("evt_safing");
        while !safed.load(Ordering::SeqCst) && start.elapsed() < 2_000_u64.ms() {
            std::hint::spin_loop();
        }
        let safing_latency = start.elapsed();

        running.store(false, Ordering::SeqCst);
        assert_eq!(executor.stop().len(), 1);

        assert!(
            safed.load(Ordering::SeqCst),
            "a safe-state request on an idle event node was never honoured"
        );
        assert!(
            safing_latency < 10_u64.ms(),
            "safing an idle watchdog-supervised event node took {safing_latency:?}. It \
             must stay within WAKE_BACKSTOP (1 ms) plus scheduling: this node is \
             reachable by the watchdog ladder, so its backstop is not allowed to \
             escalate. Idle CPU is not worth 32x on the safing path."
        );
    }

    /// The idle backoff must not cost latency when the wake path works.
    ///
    /// After ~250 ms of silence the watcher's backstop has escalated to
    /// `IDLE_BACKSTOP_MAX` (32 ms). If the notification were being *discovered*
    /// by the timeout rather than *delivered* by the futex wake, the tick after
    /// that silence would land up to 32 ms late. It must instead land in the
    /// same tens of microseconds a busy topic gets, because a wake cancels a
    /// pending timeout outright.
    ///
    /// This is the test that would catch the escalation being turned into a
    /// real latency regression.
    #[test]
    fn the_idle_backoff_costs_nothing_when_the_wake_arrives() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_backoff", "backoff_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));
        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        let mut registered = false;
        for _ in 0..200 {
            if crate::core::NodeInfo::notify_event("evt_backoff") {
                registered = true;
                break;
            }
            std::thread::sleep(5_u64.ms());
        }
        assert!(registered, "watcher never registered its notifier");
        assert!(
            await_count(&count, 1, 2_000_u64.ms()),
            "the first notification never produced a tick"
        );

        // Long enough for the backstop to walk 1 -> 2 -> 4 -> 8 -> 16 -> 32 ms
        // and sit at the ceiling for several more waits.
        std::thread::sleep(250_u64.ms());

        let start = Instant::now();
        assert!(crate::core::NodeInfo::notify_event("evt_backoff"));
        assert!(
            await_count(&count, 2, 2_000_u64.ms()),
            "the tick after a quiet period never arrived"
        );
        let latency = start.elapsed();

        running.store(false, Ordering::SeqCst);
        assert_eq!(executor.stop().len(), 1);

        assert!(
            latency < 5_u64.ms(),
            "waking a node after 250 ms of silence took {latency:?}. The escalated \
             backstop is up to 32 ms, so this is the signature of the notification \
             being found by the timeout rather than delivered by FUTEX_WAKE — the \
             idle backoff has turned into a latency regression."
        );
    }

    /// An idle event node must cost nothing.
    ///
    /// Voluntary context switches are the direct observable: the old
    /// `thread::sleep(1 ms)` produced one per millisecond per event node
    /// forever, so an idle robot with N event nodes paid 1000*N wake-ups a
    /// second to learn that nothing had happened. A futex wait that is never
    /// signalled produces one per WAKE_BACKSTOP only if the wait keeps timing
    /// out, and none at all while it is parked.
    ///
    /// Self-calibrating: the same window is measured with and without the
    /// executor, so unrelated threads in the test process cancel out. The
    /// threshold is a quarter of what polling would produce.
    #[cfg(target_os = "linux")]
    #[test]
    fn an_idle_event_node_does_not_wake_up_to_poll() {
        fn voluntary_context_switches() -> i64 {
            // SAFETY: `getrusage` writes a fully-initialised `rusage` into the
            // out-param; `RUSAGE_SELF` needs no other resource.
            let mut usage: libc::rusage = unsafe { std::mem::zeroed() };
            let rc = unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut usage) };
            assert_eq!(rc, 0, "getrusage failed");
            usage.ru_nvcsw as i64
        }

        let window = Duration::from_millis(400);

        // Baseline: same window, no event executor running.
        let before = voluntary_context_switches();
        std::thread::sleep(window);
        let baseline = voluntary_context_switches() - before;

        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_event_node("evt_idle_cpu", "idle_cpu_topic", count.clone())];
        let running = Arc::new(AtomicBool::new(true));
        let executor = EventExecutor::start(nodes, running.clone(), test_monitors());

        // Let start-up and registration settle out of the measured window.
        std::thread::sleep(100_u64.ms());
        let before = voluntary_context_switches();
        std::thread::sleep(window);
        let with_watcher = voluntary_context_switches() - before;

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
        assert_eq!(
            count.load(Ordering::Relaxed),
            0,
            "an idle event node ticked without any publish"
        );

        // 400 ms of 1 ms polling is 400 wake-ups. The escalating backstop
        // should produce ~31 over this window (16 at 1 ms, then 2/4/8/16, then
        // 32 ms each). Anything under 100 cannot be a 1 ms poll loop.
        let attributable = with_watcher - baseline;
        assert!(
            attributable < 100,
            "an idle event node added {attributable} voluntary context switches over \
             {window:?} (baseline {baseline}, with watcher {with_watcher}). A parked \
             futex waiter adds ~0; ~{} is the signature of a 1 ms poll loop, which \
             costs CPU on an idle robot and scales with the event-node count.",
            window.as_millis(),
        );
    }
}
