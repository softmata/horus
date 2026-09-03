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
        String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr)
    }

    /// Run `horus <args>` with a deadline, returning whatever it printed.
    ///
    /// `cli` waits for the process to exit, and `topic echo --count N` does not
    /// exit until it has printed N messages. Against the defect the echo test
    /// exists to catch — a ring that stops being written — that command never
    /// returns, so the test hung instead of failing and reported nothing at
    /// all. Bounded, the same run fails with the count it actually managed,
    /// which is the number that names the defect.
    fn cli_within(&self, args: &[&str], budget: Duration) -> String {
        let mut child = Command::new(horus())
            .args(args)
            .env("HORUS_NAMESPACE", &self.namespace)
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .expect("horus must run");
        // Drain stdout on its own thread: a pipe that fills stalls the child,
        // which would look exactly like the hang this is here to prevent.
        let mut pipe = child.stdout.take().expect("stdout was piped");
        let reader = std::thread::spawn(move || {
            use std::io::Read;
            let mut buf = String::new();
            let _ = pipe.read_to_string(&mut buf);
            buf
        });

        let deadline = Instant::now() + budget;
        loop {
            match child.try_wait() {
                Ok(Some(_)) | Err(_) => break,
                Ok(None) => {}
            }
            if Instant::now() >= deadline {
                let _ = child.kill();
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        let _ = child.wait();
        reader.join().unwrap_or_default()
    }

    /// Poll `horus <args>` until its output satisfies `done`, or give up.
    ///
    /// Presence is refreshed about once a second and an achieved rate needs two
    /// samples, so the interesting values do not exist immediately. Polling
    /// beats a fixed sleep: it is faster when things work and still bounded
    /// when they do not.
    /// Whether the spawned node process is alive and has registered at all.
    ///
    /// Distinguishes "this machine never got the child running" from "the child
    /// ran and the attribution is wrong". Without it, a saturated box produces
    /// the same failure text as the defect these tests exist to catch — the
    /// `wait_for` doc says exactly that, and it is why a bare timeout here is
    /// not a usable signal.
    fn node_registered(&self) -> bool {
        self.cli(&["node", "list"]).contains("ci_test_node")
    }

    /// Panic with a message that says which of the two it was.
    fn diagnose(&self, what: &str, topic_output: String) -> ! {
        if self.node_registered() {
            panic!(
                "{what}\nThe node IS registered, so the child ran and this is the \
                 attribution defect this test exists to catch:\n{topic_output}"
            );
        }
        panic!(
            "SKIP-WORTHY: the node never registered at all within the deadline, so \
             `{what}` could not be evaluated — the child did not get far enough on \
             this machine. This is not evidence about attribution.\nnode list:\n{}\n\
             topic info:\n{topic_output}",
            self.cli(&["node", "list"])
        );
    }

    fn wait_for(&self, args: &[&str], done: impl Fn(&str) -> bool) -> Option<String> {
        // 60s. This is a bound on *failure*, not on success: the loop returns
        // the moment the output is right, so a longer deadline costs nothing
        // when things work. The values under test appear on a ~1 Hz presence
        // refresh, and eight of these tests each spawn a scheduler process with
        // RT threads alongside the rest of the suite — on a busy machine a
        // shorter bound failed as "node never appeared", indistinguishable from
        // the defect these tests exist to catch.
        let deadline = Instant::now() + Duration::from_secs(60);
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
            o.lines().filter(|l| l.contains("ci_test_node")).any(|l| {
                l.split_whitespace()
                    .any(|f| f.parse::<u64>().is_ok_and(|n| n > 0))
            })
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
        o.lines().filter(|l| l.contains("Ticks")).any(|l| {
            l.split_whitespace()
                .any(|f| f.parse::<u64>().is_ok_and(|n| n > 0))
        })
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
        .wait_for(&["node", "info", "ci_test_node"], |o| {
            o.contains("achieving")
        })
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
        node.diagnose(
            "a topic created in the node constructor never got a publisher",
            node.cli(&["topic", "info", &node.topic]),
        )
    });
}

/// The ring a `Topic<u64>` gets from `auto_capacity`: one 4 KB page of 8-byte
/// messages, clamped to [16, 1024] and rounded up to a power of two. Confirmed
/// against a live segment of the very topic this test publishes on
/// (`capacity = 512`, `capacity_mask = 511`).
const U64_RING_CAPACITY: usize = 512;

/// How many messages this test asks `echo` for.
///
/// It has to exceed the ring, and by enough that the wrap is not the last thing
/// that happens. Every echo defect so far has lived exactly at the wrap: a
/// reader keying on a cursor that stops advancing at `capacity`, and a producer
/// that stops writing once the ring fills and nothing drains it. At
/// `--count 20` — one twenty-fifth of a ring — the run is over before the ring
/// is ever full, so it passed against both.
const ECHO_COUNT: usize = U64_RING_CAPACITY + 88;

/// `topic echo` must print every message, not sample the newest slot — and must
/// still be doing it after the ring has wrapped.
///
/// It read the latest slot and asked "is it different from the one I last
/// saw", which skips everything published while the poll slept: on a 40 Hz
/// topic that produced one message in twelve seconds. `echo` is the command
/// people leave running to watch a topic, and a silent sampler is worse than a
/// slow one — absence of output reads as absence of traffic.
///
/// The test node publishes a counter, so "did it miss anything" is answerable
/// exactly. Both shapes this has failed in are checked, because they look
/// nothing alike: a cursor that stops advancing at the wrap prints far fewer
/// values than it was asked for and then goes quiet, while a ring that stopped
/// being written prints the full count as the same lap over and over.
#[test]
fn topic_echo_streams_every_message_in_order() {
    let _serial = serialize();
    let node = LiveNode::spawn("live_echo", true);

    // Wait for the topic to exist before echoing.
    node.wait_for(&["topic", "list"], |o| o.contains(&node.topic))
        .unwrap_or_else(|| panic!("topic never appeared:\n{}", node.cli(&["topic", "list"])));

    let count = ECHO_COUNT.to_string();
    // The node publishes at about 90 Hz, so the ask is roughly seven seconds of
    // traffic. The budget is generous against that and still bounded, because
    // `echo` has no deadline of its own: it waits for the count forever.
    let out = node.cli_within(
        &["topic", "echo", &node.topic, "--count", &count],
        Duration::from_secs(45),
    );

    let values: Vec<u64> = out
        .lines()
        .filter_map(|l| l.split(": ").nth(1))
        .filter_map(|v| v.trim().parse().ok())
        .collect();

    assert!(
        values.len() > U64_RING_CAPACITY,
        "echo returned {} values; it was asked for {ECHO_COUNT}, and anything at \
         or under the {U64_RING_CAPACITY}-slot ring capacity leaves the wrap \
         untested:\n{out}",
        values.len()
    );

    let gaps: Vec<(u64, u64)> = values
        .windows(2)
        .filter(|w| w[1] != w[0] + 1)
        .map(|w| (w[0], w[1]))
        .collect();
    assert!(
        gaps.is_empty(),
        "echo skipped messages — the publisher emits a counter, so these jumps \
         are messages that were published and never printed: {gaps:?}\n{out}"
    );

    // A second, independent witness on the same evidence. The gap check above
    // does catch a replayed lap — 512 followed by 1 is not 513 — but it reports
    // one jump, and the shape is what names the defect: 600 printed values with
    // only 512 distinct ones is a ring being read round and round.
    let distinct = {
        let mut v = values.clone();
        v.sort_unstable();
        v.dedup();
        v.len()
    };
    assert_eq!(
        distinct,
        values.len(),
        "echo printed {} values but only {distinct} distinct ones — a ring that \
         stopped being written replays the same lap:\n{out}",
        values.len()
    );
}

/// A watch that stops updating must say so.
///
/// `topic hz` printed only on the polls where the message counter had moved, so
/// SIGKILLing the publisher under it left the last rate standing: the region
/// stays mapped after a SIGKILL, so every subsequent read still succeeds and
/// returns the same frozen number, and the loop then spun silently forever. On
/// a tty that leaves `Rate: 20.19 Hz (window: 10)` on screen with no trailing
/// newline — indistinguishable from a live readout — and off a tty it is total
/// silence. `examples/camera_perception/README.md` tells operators to run
/// `horus topic hz camera.image` to verify a camera is producing frames, which
/// is exactly the question that silence answered wrongly.
#[test]
fn topic_hz_reports_a_publisher_that_dies_mid_measurement() {
    let _serial = serialize();
    let mut node = LiveNode::spawn("live_hz_stall", true);
    node.wait_for(&["topic", "list"], |o| o.contains(&node.topic))
        .unwrap_or_else(|| panic!("topic never appeared:\n{}", node.cli(&["topic", "list"])));

    let mut hz = Command::new(horus())
        .args(["topic", "hz", &node.topic, "--window", "10"])
        .env("HORUS_NAMESPACE", &node.namespace)
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("horus must run");

    // Drain into a buffer the test can read while the child still runs. Both
    // phases below are driven off what `hz` has actually printed rather than
    // off fixed sleeps: on a box running the rest of this suite the publisher
    // can take seconds to reach its first send, and a sleep long enough to
    // cover that is a sleep that makes the test slow for everyone else.
    let seen = std::sync::Arc::new(Mutex::new(String::new()));
    let sink = seen.clone();
    let mut pipe = hz.stdout.take().expect("stdout was piped");
    let reader = std::thread::spawn(move || {
        use std::io::Read;
        let mut chunk = [0u8; 512];
        while let Ok(n) = pipe.read(&mut chunk) {
            if n == 0 {
                break;
            }
            sink.lock()
                .unwrap_or_else(|e| e.into_inner())
                .push_str(&String::from_utf8_lossy(&chunk[..n]));
        }
    });
    let text = || seen.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let wait_until = |needle: &str, budget: Duration| {
        let deadline = Instant::now() + budget;
        while Instant::now() < deadline {
            if text().contains(needle) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    };

    // The non-tty path prints one rate line a second, so this is the watch
    // measuring a live publisher.
    let measured = wait_until("average rate:", Duration::from_secs(30));
    let _ = node.child.kill();
    let _ = node.child.wait();
    // The 2 s stall timeout plus the 1 s line cadence, and slack.
    let reported = wait_until("no new messages for", Duration::from_secs(15));
    let out = text();
    let _ = hz.kill();
    let _ = hz.wait();
    let _ = reader.join();

    // Not a skip. If this fires, the stall line below would be satisfied by a
    // `topic hz` that never measured anything — the watch reports silence from
    // the moment it starts, so a publisher that never published produces the
    // same output as one that died. This assertion is the only thing standing
    // between that and a green run, so it has to fail rather than return: a
    // regression that stops `topic hz` printing rates at all must not be able
    // to turn this test green.
    assert!(
        measured,
        "`topic hz` printed no rate in 30s for a publisher `topic list` had \
         already shown, so the live half of this test never happened. Either \
         rate measurement itself regressed, or this box is loaded past the \
         point where the 30s budget means anything — check the load average \
         and /dev/shm before reading it as either:\n{out}"
    );
    assert!(
        reported,
        "the publisher was killed fifteen seconds ago and `topic hz` never said \
         so — the last rate it printed is still the last thing an operator \
         sees:\n{out}"
    );
}
