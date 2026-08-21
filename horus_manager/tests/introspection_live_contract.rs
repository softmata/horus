//! Introspection must describe the system that is running.
//!
//! Two commands answered questions about a live robot with values that were
//! never true:
//!
//! ```text
//! $ horus topic info cmd_vel        # while a controller publishes at 40 Hz
//!   Active: Yes
//!   Rate: 36.00 Hz
//!   Publishers:
//!     (none)
//!
//! $ horus node list
//!   NAME         STATUS   PRIORITY   RATE    TICKS
//!   controller   Running         0   40 Hz       0
//! ```
//!
//! "Who is writing to this topic?" is the first question of most robotics
//! debugging sessions, and the answer was empty while the answer was visibly
//! running. The tick counter said a ticking node had never ticked. And `RATE`
//! was the rate the node was *configured* with, so a node asked for 1000 Hz and
//! managing 200 still displayed 1000 — hiding the shortfall an operator is
//! looking for.
//!
//! Everything here runs against a real scheduler in a child process. In-process
//! assertions are not an option: `shm_namespace()` memoises in a `OnceLock`, so
//! the first discovery call pins the namespace for the whole test binary, and
//! Rust tests share one process. Each test gets its own namespace via `.env()`
//! on the child, and asserts through the `horus` CLI as another child.
//!
//! Run: `cargo test -p horus_manager --test introspection_live_contract`

use std::process::{Command, Stdio};
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// Serializes the live-scheduler tests.
///
/// Each spawns a real scheduler child and then repeatedly spawns the `horus`
/// CLI to interrogate it. Eight of those in parallel is enough process churn to
/// starve the very thing under test: on a machine at load average 14 all eight
/// failed together with "Topic not found", because the child had not reached
/// its first publish inside the poll window. That is the harness competing with
/// itself, not a defect in what it measures.
///
/// The same reasoning as `SCHEDULER_TEST_LOCK` in horus_core, for the same
/// reason.
static LIVE_TEST_LOCK: Mutex<()> = Mutex::new(());

fn serialize() -> MutexGuard<'static, ()> {
    LIVE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn test_node() -> &'static str {
    env!("CARGO_BIN_EXE_ci_test_node")
}

/// A scheduler running in its own SHM namespace.
struct LiveNode {
    child: std::process::Child,
    namespace: String,
    topic: String,
}

impl LiveNode {
    /// `best_effort` decides whether the node stays on the main thread or is
    /// promoted to the RT executor — the two report through different
    /// mechanisms, and each has been broken separately.
    fn spawn(namespace: &str, best_effort: bool) -> Self {
        Self::spawn_with(namespace, best_effort, false)
    }

    /// `topic_in_ctor` builds the `Topic` in the node's constructor rather than
    /// in `init()` — the ordinary shape of a node, and the one where the owner
    /// name was lost.
    fn spawn_with(namespace: &str, best_effort: bool, topic_in_ctor: bool) -> Self {
        let topic = format!("{namespace}_topic");
        let mut cmd = Command::new(test_node());
        cmd.env("HORUS_NAMESPACE", namespace)
            .env("HORUS_CI_TOPIC_NAME", &topic)
            .env("HORUS_CI_DURATION_MS", "60000")
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        if best_effort {
            cmd.env("HORUS_CI_BEST_EFFORT", "1");
        }
        if topic_in_ctor {
            cmd.env("HORUS_CI_TOPIC_IN_CTOR", "1");
        }
        let child = cmd.spawn().expect("ci_test_node must start");
        Self {
            child,
            namespace: namespace.to_string(),
            topic,
        }
    }

    fn cli(&self, args: &[&str]) -> String {
        let out = Command::new(horus())
            .args(args)
            .env("HORUS_NAMESPACE", &self.namespace)
            .output()
            .expect("horus must run");
        String::from_utf8_lossy(&out.stdout).into_owned()
            + &String::from_utf8_lossy(&out.stderr)
    }

    /// Poll `horus <args>` until its output satisfies `done`, or give up.
    ///
    /// Presence is refreshed about once a second and an achieved rate needs two
    /// samples, so the interesting values do not exist immediately. Polling
    /// beats a fixed sleep: it is faster when things work and still bounded
    /// when they do not.
    fn wait_for(&self, args: &[&str], done: impl Fn(&str) -> bool) -> Option<String> {
        // Generous because this is a liveness bound, not a timing
        // assertion: the values under test appear on a ~1 Hz presence refresh,
        // and a loaded machine can delay a child's first publish by seconds.
        let deadline = Instant::now() + Duration::from_secs(45);
        while Instant::now() < deadline {
            let out = self.cli(args);
            if done(&out) {
                return Some(out);
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        None
    }
}

impl Drop for LiveNode {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// The headline symptom. The node publishes with `.rate()`, which promotes it
/// to `ExecutionClass::Rt` and moves it off the main thread — and the presence
/// refresh iterated only the main-thread node list, so an executor-owned node
/// kept whatever was written at init. Init runs before the first `send()`, so
/// that record had no topic associations at all.
#[test]
fn topic_info_names_the_publisher_of_a_live_topic() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_pub_rt", false);
    let out = node
        .wait_for(&["topic", "info", &node.topic], |o| {
            o.contains("Publishers") && o.contains("ci_test_node")
        })
        .unwrap_or_else(|| {
            panic!(
                "topic info never named the publisher:\n{}",
                node.cli(&["topic", "info", &node.topic])
            )
        });

    assert!(
        !out.contains("(none)") || out.contains("ci_test_node"),
        "the publisher is running and publishing:\n{out}"
    );
}

/// The same, for a node that stays on the main thread. Both paths must work;
/// only one of them ever did, and it was the one `horus new` generates, which
/// is why this was never caught interactively.
#[test]
fn topic_info_names_a_main_thread_publisher() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_pub_be", true);
    node.wait_for(&["topic", "info", &node.topic], |o| {
        o.contains("ci_test_node")
    })
    .unwrap_or_else(|| {
        panic!(
            "topic info never named the publisher:\n{}",
            node.cli(&["topic", "info", &node.topic])
        )
    });
}

/// `node list` reads `tick_count` from the SHM registry slot, preferring it
/// over the presence file. The four executors each advance their own nodes'
/// slots; the main-thread path did not, so a BestEffort node reported 0 ticks
/// for as long as it ran.
#[test]
fn a_main_thread_node_reports_the_ticks_it_has_executed() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_ticks_be", true);
    let out = node
        .wait_for(&["node", "list"], |o| {
            o.lines()
                .filter(|l| l.contains("ci_test_node"))
                .any(|l| l.split_whitespace().any(|f| f.parse::<u64>().is_ok_and(|n| n > 0)))
        })
        .unwrap_or_else(|| {
            panic!(
                "tick count never advanced past 0:\n{}",
                node.cli(&["node", "list"])
            )
        });
    assert!(out.contains("ci_test_node"), "{out}");
}

#[test]
fn an_executor_owned_node_reports_the_ticks_it_has_executed() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_ticks_rt", false);
    node.wait_for(&["node", "info", "ci_test_node"], |o| {
        o.lines()
            .filter(|l| l.contains("Ticks"))
            .any(|l| l.split_whitespace().any(|f| f.parse::<u64>().is_ok_and(|n| n > 0)))
    })
    .unwrap_or_else(|| {
        panic!(
            "tick count never advanced past 0:\n{}",
            node.cli(&["node", "info", "ci_test_node"])
        )
    });
}

/// The measured rate. `RATE` was the configured value under a header that read
/// as throughput; nothing anywhere computed an achieved rate, so the number an
/// operator needs did not exist.
#[test]
fn node_info_reports_a_measured_rate_beside_the_configured_one() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_rate", true);
    let out = node
        .wait_for(&["node", "info", "ci_test_node"], |o| o.contains("achieving"))
        .unwrap_or_else(|| {
            panic!(
                "no measured rate was ever reported:\n{}",
                node.cli(&["node", "info", "ci_test_node"])
            )
        });

    // Both numbers come off the same line: "Rate: 100 Hz (achieving 89.3 Hz)".
    // Read the configured value rather than assuming it — dropping `.rate()`
    // makes the node inherit the scheduler's global tick rate, so hardcoding a
    // figure here tests the harness instead of the code.
    let line = out
        .lines()
        .find(|l| l.contains("achieving"))
        .unwrap_or_else(|| panic!("no rate line in:\n{out}"));
    let configured: f64 = line
        .split_whitespace()
        .find_map(|t| t.parse().ok())
        .unwrap_or_else(|| panic!("could not parse the configured rate from:\n{line}"));
    let measured: f64 = line
        .split("achieving ")
        .nth(1)
        .and_then(|s| s.split_whitespace().next())
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| panic!("could not parse the measured rate from:\n{line}"));

    assert!(
        measured > 0.0,
        "a running node cannot be achieving 0 Hz:\n{line}"
    );
    assert!(
        measured <= configured * 1.5,
        "measured {measured} Hz exceeds the configured {configured} Hz by more \
         than tolerance — this is probably the configured value copied, which is \
         the defect:\n{line}"
    );
}

/// `node list` is the view an operator actually runs, and it had no health
/// column at all — so a node `node info` already showed as Critical looked
/// unremarkable here.
#[test]
fn node_list_shows_health_and_the_measured_rate() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_cols", true);
    let out = node
        .wait_for(&["node", "list"], |o| o.contains("ci_test_node"))
        .unwrap_or_else(|| panic!("node never appeared:\n{}", node.cli(&["node", "list"])));

    assert!(out.contains("HEALTH"), "no health column:\n{out}");
    assert!(
        out.contains("ACTUAL"),
        "no measured-rate column — RATE alone cannot say \"asked for 1000, \
         managing 200\":\n{out}"
    );
}

/// No measurement is not a measurement of zero. Printing `0 Hz` for a node that
/// is running would be a worse lie than the one the column exists to fix.
#[test]
fn an_unmeasured_rate_is_not_reported_as_zero() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_unmeasured", true);
    let out = node
        .wait_for(&["node", "list"], |o| o.contains("ci_test_node"))
        .unwrap_or_else(|| panic!("node never appeared:\n{}", node.cli(&["node", "list"])));

    let row = out
        .lines()
        .find(|l| l.contains("ci_test_node"))
        .unwrap_or_default();
    assert!(
        !row.contains("0.0 Hz"),
        "an unmeasured rate must render as a dash, not 0.0 Hz:\n{row}"
    );
}

/// The case a real node actually is. `Topic::new` captured the owning node's
/// name from a thread-local that is only set inside a scheduler-managed tick —
/// and constructors run before the scheduler starts, so the handle recorded no
/// owner and nothing ever registered it. Building the topic in `init()` happens
/// to work, which is why the shipped test node did not catch this.
#[test]
fn a_topic_built_in_a_constructor_still_names_its_publisher() {
    let _serial = serialize();
    let node = LiveNode::spawn_with("live_ctor", true, true);
    node.wait_for(&["topic", "info", &node.topic], |o| {
        o.contains("ci_test_node")
    })
    .unwrap_or_else(|| {
        panic!(
            "a topic created in the node constructor never got a publisher:\n{}",
            node.cli(&["topic", "info", &node.topic])
        )
    });
}
