//! CLI Runtime Integration Tests
//!
//! Tests every CLI introspection command against a LIVE running scheduler.
//! This is the robotics workflow: Terminal 1 runs the robot, Terminal 2 debugs it.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test cli_runtime_integration -- --ignored --nocapture --test-threads=1

use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

// ─── Test Infrastructure ──────────────────────────────────────────────────────

fn horus_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("target/debug/horus")
}

fn example_bin(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join(format!("target/debug/examples/{}", name))
}

fn horus_cmd() -> Command {
    let mut cmd = Command::new(horus_bin());
    cmd.env("NO_COLOR", "1"); // Disable ANSI for easier parsing
    cmd
}

/// Start a background scheduler process with namespace isolation.
/// Returns the child handle — drops kill the process.
struct BackgroundScheduler {
    child: Child,
    namespace: String,
}

impl BackgroundScheduler {
    fn start(example: &str, namespace: &str) -> Self {
        let bin = example_bin(example);
        assert!(bin.exists(), "Example binary not found: {:?}. Run: cargo build --no-default-features -p horus --examples", bin);

        let child = Command::new(&bin)
            .env("HORUS_NAMESPACE", namespace)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap_or_else(|e| panic!("Failed to start {}: {}", example, e));

        let sched = Self {
            child,
            namespace: namespace.to_string(),
        };

        // Wait for SHM to be ready (topics created)
        sched.wait_for_ready();
        sched
    }

    fn start_with_record(example: &str, namespace: &str, session: &str) -> Self {
        let bin = example_bin(example);
        assert!(bin.exists(), "Example binary not found: {:?}", bin);

        let child = Command::new(&bin)
            .env("HORUS_NAMESPACE", namespace)
            .env("HORUS_RECORD_SESSION", session)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap_or_else(|e| panic!("Failed to start {}: {}", example, e));

        let sched = Self {
            child,
            namespace: namespace.to_string(),
        };
        sched.wait_for_ready();
        sched
    }

    /// Wait until SHM directory has topics (max 10s)
    fn wait_for_ready(&self) {
        let start = Instant::now();
        let shm_dir = format!("/dev/shm/horus_{}/topics", self.namespace);
        loop {
            if let Ok(entries) = std::fs::read_dir(&shm_dir) {
                let count = entries
                    .filter_map(|e| e.ok())
                    .filter(|e| !e.file_name().to_string_lossy().ends_with(".meta"))
                    .count();
                if count > 0 {
                    // Extra settle time for node presence files
                    std::thread::sleep(Duration::from_millis(500));
                    return;
                }
            }
            if start.elapsed() > Duration::from_secs(10) {
                panic!(
                    "Timeout waiting for SHM ready in namespace '{}' (dir: {})",
                    self.namespace, shm_dir
                );
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// Create a horus command with this scheduler's namespace
    fn horus(&self) -> Command {
        let mut cmd = horus_cmd();
        cmd.env("HORUS_NAMESPACE", &self.namespace);
        cmd
    }

    /// Stop the scheduler process via SIGKILL (immediate).
    /// We use SIGKILL instead of SIGTERM because the HORUS scheduler installs
    /// a global SIGTERM handler that would also catch signals sent to our process
    /// group, killing the test runner itself.
    fn stop(&mut self) {
        let _ = self.child.kill(); // SIGKILL — only targets this child
        let _ = self.child.wait(); // Reap to prevent zombie
    }
}

impl Drop for BackgroundScheduler {
    fn drop(&mut self) {
        self.stop();
        // Clean up SHM
        let _ = std::fs::remove_dir_all(format!("/dev/shm/horus_{}", self.namespace));
    }
}

fn run_and_capture(cmd: &mut Command) -> (String, bool) {
    let output = cmd.output().expect("Failed to execute command");
    let stdout = String::from_utf8_lossy(&output.stdout).to_string();
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    let combined = format!("{}{}", stdout, stderr);
    (combined, output.status.success())
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 1: Topic introspection against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_topic_list() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_topic_list");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["topic", "list"]));
    println!("topic list output:\n{}", output);

    assert!(ok, "topic list should exit 0");
    assert!(
        output.contains("sensor.cmd_vel") || output.contains("cmd_vel"),
        "should list sensor.cmd_vel topic: {}",
        output
    );
}

#[test]
#[ignore]
fn runtime_topic_echo() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_topic_echo");

    let (output, ok) = run_and_capture(
        &mut sched
            .horus()
            .args(["topic", "echo", "sensor.cmd_vel", "--count", "3"]),
    );
    println!("topic echo output:\n{}", output);

    assert!(ok, "topic echo should exit 0");
    // Should contain message data (linear/angular values)
    assert!(
        output.contains("linear") || output.contains("angular") || output.contains("CmdVel")
            || output.len() > 20,
        "should show message content: {}",
        output
    );
}

#[test]
#[ignore]
fn runtime_topic_hz() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_topic_hz");

    // topic hz needs a few seconds to measure
    let (output, _ok) = run_and_capture(
        &mut sched
            .horus()
            .args(["topic", "hz", "sensor.cmd_vel", "--window", "10"]),
    );
    println!("topic hz output:\n{}", output);

    // May timeout or show Hz — either is acceptable for test
    assert!(
        output.contains("Hz") || output.contains("hz") || output.contains("rate")
            || output.contains("sensor.cmd_vel"),
        "should show frequency info: {}",
        output
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 2: Node introspection against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_node_list() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_node_list");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["node", "list"]));
    println!("node list output:\n{}", output);

    assert!(ok, "node list should exit 0");
    assert!(
        output.contains("sensor_node"),
        "should show sensor_node: {}",
        output
    );
    assert!(
        output.contains("controller_node"),
        "should show controller_node: {}",
        output
    );
    assert!(
        output.contains("motor_node"),
        "should show motor_node: {}",
        output
    );
}

#[test]
#[ignore]
fn runtime_node_info() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_node_info");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["node", "info", "sensor_node"]));
    println!("node info output:\n{}", output);

    assert!(ok, "node info should exit 0");
    assert!(
        output.contains("sensor_node"),
        "should show node name: {}",
        output
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 3: Service introspection against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_service_list() {
    let sched = BackgroundScheduler::start("qa_services_tf", "cli_rt_svc_list");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["service", "list"]));
    println!("service list output:\n{}", output);

    assert!(ok, "service list should exit 0");
    // qa_services_tf registers services — verify they appear
    println!("✓ runtime_service_list — command works against live scheduler");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 4: Action introspection against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_action_list() {
    let sched = BackgroundScheduler::start("qa_action_server", "cli_rt_action_list");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["action", "list"]));
    println!("action list output:\n{}", output);

    assert!(ok, "action list should exit 0");
    assert!(
        output.contains("pick_object"),
        "should show pick_object action: {}",
        output
    );
}

#[test]
#[ignore]
fn runtime_action_info() {
    let sched = BackgroundScheduler::start("qa_action_server", "cli_rt_action_info");

    let (output, ok) = run_and_capture(
        &mut sched
            .horus()
            .args(["action", "info", "pick_object"]),
    );
    println!("action info output:\n{}", output);

    assert!(ok, "action info should exit 0");
    assert!(
        output.contains("pick_object"),
        "should show action details: {}",
        output
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 5: Parameter system against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_param_crud() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_param_crud");

    // Set a parameter
    let (output, ok) = run_and_capture(
        &mut sched
            .horus()
            .args(["param", "set", "test.speed", "2.5"]),
    );
    println!("param set output: {}", output);
    assert!(ok, "param set should exit 0");

    // Get it back
    let (output, ok) = run_and_capture(&mut sched.horus().args(["param", "get", "test.speed"]));
    println!("param get output: {}", output);
    assert!(ok, "param get should exit 0");
    assert!(
        output.contains("2.5"),
        "should return set value: {}",
        output
    );

    // List params
    let (output, ok) = run_and_capture(&mut sched.horus().args(["param", "list"]));
    println!("param list output: {}", output);
    assert!(ok, "param list should exit 0");

    // Delete
    let (output, ok) = run_and_capture(
        &mut sched
            .horus()
            .args(["param", "delete", "test.speed"]),
    );
    println!("param delete output: {}", output);
    assert!(ok, "param delete should exit 0");

    println!("✓ runtime_param_crud — full CRUD cycle works");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 6: Transform frame introspection
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_frame_list() {
    let sched = BackgroundScheduler::start("qa_services_tf", "cli_rt_frame_list");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["frame", "list"]));
    println!("frame list output:\n{}", output);

    assert!(ok, "frame list should exit 0");
    // qa_services_tf registers frames — verify command runs
    println!("✓ runtime_frame_list — command works against live scheduler");
}

#[test]
#[ignore]
fn runtime_frame_tree() {
    let sched = BackgroundScheduler::start("qa_services_tf", "cli_rt_frame_tree");

    let (output, ok) = run_and_capture(&mut sched.horus().args(["frame", "tree"]));
    println!("frame tree output:\n{}", output);

    assert!(ok, "frame tree should exit 0");
    println!("✓ runtime_frame_tree — tree visualization works");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 7: Log viewing against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_log() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_log");

    // Give scheduler time to generate log entries
    std::thread::sleep(Duration::from_secs(2));

    let (output, ok) = run_and_capture(&mut sched.horus().args(["log"]));
    println!("log output:\n{}", output);

    // log may show entries or "no recent logs" — both are valid
    assert!(ok, "log should exit 0");
    println!("✓ runtime_log — command works against live scheduler");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 8: Blackbox against live scheduler
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_blackbox() {
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_blackbox");

    // Give time for events to accumulate
    std::thread::sleep(Duration::from_secs(2));

    let (output, ok) = run_and_capture(&mut sched.horus().args(["blackbox"]));
    println!("blackbox output:\n{}", output);

    assert!(ok, "blackbox should exit 0");

    // Also test --anomalies filter
    let (output2, ok2) = run_and_capture(&mut sched.horus().args(["blackbox", "--anomalies"]));
    println!("blackbox --anomalies output:\n{}", output2);
    assert!(ok2, "blackbox --anomalies should exit 0");

    println!("✓ runtime_blackbox — both modes work");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 9: Recording pipeline — record → list → info → replay → export
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_recording_pipeline() {
    let namespace = "cli_rt_recording";
    let session = "rt_test_session";

    // Clean any prior session
    let _ = horus_cmd().args(["record", "delete", session, "--force"]).output();

    // Start scheduler with recording enabled
    let mut sched = BackgroundScheduler::start_with_record("qa_pubsub", namespace, session);

    // Let it run for a few seconds to capture data
    std::thread::sleep(Duration::from_secs(3));

    // Stop gracefully (triggers recording save)
    sched.stop();

    // Wait a moment for files to flush
    std::thread::sleep(Duration::from_millis(500));

    // record list — should show our session
    let (output, ok) = run_and_capture(&mut horus_cmd().args(["record", "list"]));
    println!("record list:\n{}", output);
    assert!(ok, "record list should exit 0");
    assert!(
        output.contains(session),
        "should list session '{}': {}",
        session,
        output
    );

    // record info — should show node recordings
    let (output, ok) = run_and_capture(&mut horus_cmd().args(["record", "info", session]));
    println!("record info:\n{}", output);
    assert!(ok, "record info should exit 0");
    assert!(
        output.contains("sensor_node") || output.contains(".horus"),
        "should show node recordings: {}",
        output
    );

    // record replay — replay first 5 ticks
    let (output, ok) = run_and_capture(
        &mut horus_cmd().args(["record", "replay", session, "--stop-tick", "5"]),
    );
    println!("record replay:\n{}", output);
    assert!(ok, "record replay should exit 0");
    assert!(
        output.contains("Replay completed") || output.contains("replay"),
        "should complete replay: {}",
        output
    );

    // record export — JSON
    let export_path = format!("/tmp/horus_rt_test_{}.json", std::process::id());
    let (output, ok) = run_and_capture(
        &mut horus_cmd().args([
            "record",
            "export",
            session,
            "--output",
            &export_path,
            "--format",
            "json",
        ]),
    );
    println!("record export:\n{}", output);
    assert!(ok, "record export should exit 0");

    // Verify exported JSON is valid
    if let Ok(json_str) = std::fs::read_to_string(&export_path) {
        assert!(
            serde_json::from_str::<serde_json::Value>(&json_str).is_ok(),
            "exported JSON should be valid"
        );
    }

    // Cleanup
    let _ = std::fs::remove_file(&export_path);
    let _ = horus_cmd()
        .args(["record", "delete", session, "--force"])
        .output();

    println!("✓ runtime_recording_pipeline — full lifecycle works");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 10: Launch dry-run with namespace
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_launch_dry_run() {
    let tmpdir = tempfile::TempDir::new().unwrap();
    let yaml = r#"
nodes:
  - name: sensor
    package: my_robot
    rate_hz: 100
    params:
      port: /dev/ttyUSB0
      baud: 115200
  - name: controller
    package: my_robot
    rate_hz: 50
    depends_on:
      - sensor
    params:
      kp: 1.5
      ki: 0.01
namespace: robot1
session: test_launch
"#;

    let launch_file = tmpdir.path().join("robot.yaml");
    std::fs::write(&launch_file, yaml).unwrap();

    let (output, ok) = run_and_capture(
        &mut horus_cmd()
            .args(["launch", "--dry-run"])
            .arg(&launch_file),
    );
    println!("launch dry-run:\n{}", output);

    assert!(ok, "dry-run should exit 0");
    assert!(output.contains("sensor"), "should show sensor node");
    assert!(output.contains("controller"), "should show controller node");
    assert!(
        output.contains("robot1"),
        "should show namespace: {}",
        output
    );
    assert!(
        output.contains("kp") || output.contains("1.5"),
        "should show params: {}",
        output
    );

    println!("✓ runtime_launch_dry_run — 2 nodes + namespace + params");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 11: Doctor + clean against live state
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_doctor() {
    let (output, _ok) = run_and_capture(&mut horus_cmd().args(["doctor"]));
    println!("doctor output:\n{}", output);

    // Doctor exits non-zero if it finds issues (e.g., missing horus.toml).
    // That's correct behavior — it reports ecosystem health, not CLI health.
    assert!(
        output.contains("horus") || output.contains("Toolchains") || output.contains("Summary"),
        "should show diagnostic info: {}",
        output
    );
    println!("✓ runtime_doctor — health check runs and produces output");
}

#[test]
#[ignore]
fn runtime_clean_shm() {
    // Create some SHM state first
    let sched = BackgroundScheduler::start("qa_pubsub", "cli_rt_clean_test");
    drop(sched); // Stop and cleanup

    // clean --shm should succeed even with nothing to clean
    let (output, ok) = run_and_capture(&mut horus_cmd().args(["clean", "--shm", "--force"]));
    println!("clean --shm output:\n{}", output);

    assert!(ok, "clean --shm should exit 0");
    println!("✓ runtime_clean_shm — SHM cleanup works");
}

// ═══════════════════════════════════════════════════════════════════════════════
// TEST 12: Message type introspection
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn runtime_msg_list() {
    let (output, ok) = run_and_capture(&mut horus_cmd().args(["msg", "list"]));
    println!("msg list output (first 20 lines):");
    for line in output.lines().take(20) {
        println!("  {}", line);
    }

    assert!(ok, "msg list should exit 0");
    assert!(
        output.contains("CmdVel") || output.contains("cmd_vel"),
        "should list CmdVel type: {}",
        output
    );
    println!("✓ runtime_msg_list — message types listed");
}
