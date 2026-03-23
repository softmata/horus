//! Reproduction of GitHub issue #37: cross-process Image pub/sub crash.
//!
//! Faithfully replicates the user's scenario:
//! - Two separate processes, each with its own Scheduler
//! - Publisher sends Image on a topic
//! - Subscriber receives Image on the same topic
//! - No workarounds (no dummy send, no check_migration_now)

use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Instant;

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_core::memory::Image;
use horus_core::scheduling::Scheduler;
use horus_core::Node;
use horus_core::types::ImageEncoding;

const CHILD_ENV: &str = "ISSUE37_CHILD";
const TOPIC_ENV: &str = "ISSUE37_TOPIC";
const TEST_ENV: &str = "ISSUE37_TEST";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

// ============================================================================
// Publisher node — equivalent to user's TestSendNode
// ============================================================================

struct TestSendNode {
    topic: Topic<Image>,
    frame_count: u32,
    width: u32,
    height: u32,
}

impl TestSendNode {
    fn new(topic_name: &str, width: u32, height: u32) -> Self {
        Self {
            topic: Topic::new(topic_name).expect("Topic::new for publisher"),
            frame_count: 0,
            width,
            height,
        }
    }
}

impl Node for TestSendNode {
    fn name(&self) -> &str {
        "test_send_node"
    }

    fn tick(&mut self) {
        let img = Image::new(self.width, self.height, ImageEncoding::Rgb8)
            .expect("Image::new");
        // Fill with recognizable pattern
        let val = ((self.frame_count + 1) * 7 % 256) as u8;
        let data = img.data_mut();
        for b in data.iter_mut() {
            *b = val;
        }
        self.topic.send(&img);
        self.frame_count += 1;
    }
}

// ============================================================================
// Subscriber node — equivalent to user's TestReceiveNode
// ============================================================================

struct TestRecvNode {
    topic: Topic<Image>,
    received: Arc<AtomicU32>,
    valid: Arc<AtomicU32>,
}

impl TestRecvNode {
    fn new(topic_name: &str, received: Arc<AtomicU32>, valid: Arc<AtomicU32>) -> Self {
        Self {
            topic: Topic::new(topic_name).expect("Topic::new for subscriber"),
            received,
            valid,
        }
    }
}

impl Node for TestRecvNode {
    fn name(&self) -> &str {
        "test_receive_node"
    }

    fn tick(&mut self) {
        if let Some(img) = self.topic.recv() {
            self.received.fetch_add(1, Ordering::Relaxed);
            let data = img.data();
            let non_zero = data.iter().filter(|&&b| b != 0).count();
            if non_zero > 0 && img.width() > 0 && img.height() > 0 {
                self.valid.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

// ============================================================================
// Child process entry: runs subscriber scheduler
// ============================================================================

fn child_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).unwrap();
    let received = Arc::new(AtomicU32::new(0));
    let valid = Arc::new(AtomicU32::new(0));

    let recv_node = TestRecvNode::new(&topic_name, received.clone(), valid.clone());

    let mut scheduler = Scheduler::new().tick_rate(30_u64.hz());
    scheduler
        .add(recv_node)
        .order(0)
        .rate(30_u64.hz())
        .build()
        .expect("scheduler add recv node");

    // Run for up to 10 seconds using tick_once
    let deadline = Instant::now() + 10_u64.secs();
    while Instant::now() < deadline {
        let _ = scheduler.tick_once();
        // Stop early if we've received enough
        if received.load(Ordering::Relaxed) >= 5 {
            break;
        }
    }

    let r = received.load(Ordering::Relaxed);
    let v = valid.load(Ordering::Relaxed);
    println!("RECEIVED:{}", r);
    println!("VALID:{}", v);
}

fn spawn_child(test_name: &str, topic_name: &str, test_id: &str) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TOPIC_ENV, topic_name)
        .env(TEST_ENV, test_id)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap()
}

fn parse_output(stdout: &str) -> (u32, u32) {
    let mut received = 0;
    let mut valid = 0;
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("RECEIVED:") {
            received = n.parse().unwrap_or(0);
        } else if let Some(n) = line.strip_prefix("VALID:") {
            valid = n.parse().unwrap_or(0);
        }
    }
    (received, valid)
}

// ============================================================================
// Test: Exact user scenario — small image (3x2), two schedulers
// ============================================================================

#[test]
fn issue_37_small_image_two_schedulers() {
    if is_child() {
        child_subscriber();
        return;
    }

    let topic_name = format!("issue37_small_{}", std::process::id());

    // Publisher scheduler in parent
    let send_node = TestSendNode::new(&topic_name, 3, 2);
    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(send_node)
        .order(0)
        .rate(1_u64.hz()) // 1Hz like user's code
        .build()
        .expect("scheduler add send node");

    // Spawn child subscriber
    let child = spawn_child("issue_37_small_image_two_schedulers", &topic_name, "small");

    // Let child start
    std::thread::sleep(500_u64.ms());

    // Run publisher for 8 seconds (should send ~8 frames at 1Hz)
    let deadline = Instant::now() + 8_u64.secs();
    while Instant::now() < deadline {
        let _ = scheduler.tick_once();
    }

    let output = child.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    eprintln!("=== stdout ===\n{}", stdout);
    if !stderr.is_empty() {
        eprintln!("=== stderr ===\n{}", stderr);
    }

    assert!(
        output.status.success(),
        "CHILD CRASHED (exit {:?}):\nstderr: {}",
        output.status.code(),
        stderr
    );

    let (received, valid) = parse_output(&stdout);
    eprintln!("Small image: received={}, valid={}", received, valid);
    assert!(received > 0, "Child received 0 images");
    assert_eq!(received, valid, "Some images had corrupted data");
}

// ============================================================================
// Test: User's original report — 324x244 image
// ============================================================================

#[test]
fn issue_37_large_image_two_schedulers() {
    if is_child() {
        child_subscriber();
        return;
    }

    let topic_name = format!("issue37_large_{}", std::process::id());

    let send_node = TestSendNode::new(&topic_name, 324, 244);
    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(send_node)
        .order(0)
        .rate(1_u64.hz())
        .build()
        .expect("scheduler add send node");

    let child = spawn_child("issue_37_large_image_two_schedulers", &topic_name, "large");
    std::thread::sleep(500_u64.ms());

    let deadline = Instant::now() + 8_u64.secs();
    while Instant::now() < deadline {
        let _ = scheduler.tick_once();
    }

    let output = child.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    eprintln!("=== stdout ===\n{}", stdout);
    if !stderr.is_empty() {
        eprintln!("=== stderr ===\n{}", stderr);
    }

    assert!(
        output.status.success(),
        "CHILD CRASHED (exit {:?}):\nstderr: {}",
        output.status.code(),
        stderr
    );

    let (received, valid) = parse_output(&stdout);
    eprintln!("Large image (324x244): received={}, valid={}", received, valid);
    assert!(received > 0, "Child received 0 images");
    assert_eq!(received, valid, "Some images had corrupted data");
}

// ============================================================================
// Test: VGA image (640x480) — common camera resolution
// ============================================================================

#[test]
fn issue_37_vga_image_two_schedulers() {
    if is_child() {
        child_subscriber();
        return;
    }

    let topic_name = format!("issue37_vga_{}", std::process::id());

    let send_node = TestSendNode::new(&topic_name, 640, 480);
    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(send_node)
        .order(0)
        .rate(1_u64.hz())
        .build()
        .expect("scheduler add send node");

    let child = spawn_child("issue_37_vga_image_two_schedulers", &topic_name, "vga");
    std::thread::sleep(500_u64.ms());

    let deadline = Instant::now() + 8_u64.secs();
    while Instant::now() < deadline {
        let _ = scheduler.tick_once();
    }

    let output = child.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    eprintln!("=== stdout ===\n{}", stdout);
    if !stderr.is_empty() {
        eprintln!("=== stderr ===\n{}", stderr);
    }

    assert!(
        output.status.success(),
        "CHILD CRASHED (exit {:?}):\nstderr: {}",
        output.status.code(),
        stderr
    );

    let (received, valid) = parse_output(&stdout);
    eprintln!("VGA image (640x480): received={}, valid={}", received, valid);
    assert!(received > 0, "Child received 0 images");
    assert_eq!(received, valid, "Some images had corrupted data");
}
