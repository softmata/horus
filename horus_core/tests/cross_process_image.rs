//! Cross-process Image pub/sub integration test.
//!
//! Reproduces GitHub issue #37: "Pub/Sub crashes across schedulers when
//! using Image messages (separate processes)".
//!
//! Two processes communicate Image messages through Topic<Image> over SHM:
//!   - Parent: Publisher — creates Images, fills pixels, sends via topic
//!   - Child:  Subscriber — receives Images, validates pixel data

use std::process::{Command, Stdio};
use std::time::Instant;

use horus_core::communication::topic::Topic;
use horus_core::memory::Image;
use horus_core::types::ImageEncoding;
use horus_core::core::DurationExt;

/// Env var that marks a child process invocation.
const CHILD_ENV: &str = "HORUS_IMAGE_IPC_CHILD";
/// Env var carrying the topic name to the child.
const TOPIC_NAME_ENV: &str = "HORUS_IMAGE_IPC_TOPIC";
/// Env var carrying the message count to the child.
const MSG_COUNT_ENV: &str = "HORUS_IMAGE_IPC_COUNT";

const IMAGE_WIDTH: u32 = 64;
const IMAGE_HEIGHT: u32 = 48;

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

/// Fill image with a deterministic pattern based on frame index.
fn fill_pattern(img: &mut Image, frame_idx: u32) {
    let data = img.data_mut();
    let bpp = img.encoding().bytes_per_pixel() as usize;
    let step = img.step() as usize;

    for y in 0..img.height() {
        for x in 0..img.width() {
            let offset = y as usize * step + x as usize * bpp;
            if offset + bpp <= data.len() {
                // R = frame_idx, G = x, B = y (all mod 256)
                data[offset] = (frame_idx % 256) as u8;
                if bpp > 1 {
                    data[offset + 1] = (x % 256) as u8;
                }
                if bpp > 2 {
                    data[offset + 2] = (y % 256) as u8;
                }
            }
        }
    }
}

/// Validate the pixel pattern for a given frame index.
fn validate_pattern(img: &Image, frame_idx: u32) -> bool {
    let data = img.data();
    let bpp = img.encoding().bytes_per_pixel() as usize;
    let step = img.step() as usize;

    // Check a few sample pixels (not all, for speed)
    let check_points = [(0, 0), (1, 0), (0, 1), (10, 10), (63, 47)];

    for &(x, y) in &check_points {
        if x >= img.width() || y >= img.height() {
            continue;
        }
        let offset = y as usize * step + x as usize * bpp;
        if offset + bpp > data.len() {
            return false;
        }
        if data[offset] != (frame_idx % 256) as u8 {
            return false;
        }
        if bpp > 1 && data[offset + 1] != (x % 256) as u8 {
            return false;
        }
        if bpp > 2 && data[offset + 2] != (y % 256) as u8 {
            return false;
        }
    }
    true
}

/// Child entry: receive Image messages, validate pixel data, print results.
fn child_recv_images() {
    let topic_name = std::env::var(TOPIC_NAME_ENV).expect("HORUS_IMAGE_IPC_TOPIC not set");
    let expected_count: usize = std::env::var(MSG_COUNT_ENV)
        .expect("HORUS_IMAGE_IPC_COUNT not set")
        .parse()
        .expect("invalid count");

    let topic: Topic<Image> = Topic::new(&topic_name).expect("child: Topic::new failed");

    let mut received_count = 0usize;
    let mut valid_count = 0usize;
    let mut errors = Vec::new();
    let deadline = Instant::now() + 15_u64.secs();

    // Receive until we get the expected count or timeout
    while received_count < expected_count && Instant::now() < deadline {
        match topic.recv() {
            Some(img) => {
                // Basic metadata checks
                if img.width() != IMAGE_WIDTH {
                    errors.push(format!(
                        "frame {}: width {} != expected {}",
                        received_count,
                        img.width(),
                        IMAGE_WIDTH
                    ));
                }
                if img.height() != IMAGE_HEIGHT {
                    errors.push(format!(
                        "frame {}: height {} != expected {}",
                        received_count,
                        img.height(),
                        IMAGE_HEIGHT
                    ));
                }
                if img.encoding() != ImageEncoding::Rgb8 {
                    errors.push(format!(
                        "frame {}: encoding {:?} != Rgb8",
                        received_count,
                        img.encoding()
                    ));
                }

                // Access pixel data (this is where the crash would happen)
                let data = img.data();
                if data.is_empty() {
                    errors.push(format!("frame {}: empty data", received_count));
                }

                // Validate pixel pattern
                // Use received_count + 1 because publisher sends frames 1..=N
                let frame_idx = received_count as u32 + 1;
                if validate_pattern(&img, frame_idx) {
                    valid_count += 1;
                }

                received_count += 1;
            }
            None => std::thread::yield_now(),
        }
    }

    // Print results for parent to parse
    println!("RECEIVED:{}", received_count);
    println!("VALID:{}", valid_count);
    for err in &errors {
        println!("ERROR:{}", err);
    }
}

/// Spawn a child process that runs the specified child function.
fn spawn_child(test_name: &str, topic_name: &str, msg_count: usize) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TOPIC_NAME_ENV, topic_name)
        .env(MSG_COUNT_ENV, msg_count.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child process")
}

/// Parse child stdout for results.
fn parse_child_output(stdout: &str) -> (usize, usize, Vec<String>) {
    let mut received = 0;
    let mut valid = 0;
    let mut errors = Vec::new();
    for line in stdout.lines() {
        if let Some(n) = line.strip_prefix("RECEIVED:") {
            received = n.parse().unwrap_or(0);
        } else if let Some(n) = line.strip_prefix("VALID:") {
            valid = n.parse().unwrap_or(0);
        } else if let Some(e) = line.strip_prefix("ERROR:") {
            errors.push(e.to_string());
        }
    }
    (received, valid, errors)
}

// ============================================================================
// Test 1: Cross-process Image roundtrip (basic sanity)
// ============================================================================

#[test]
fn cross_process_image_roundtrip() {
    if is_child() {
        child_recv_images();
        return;
    }

    let topic_name = format!("xproc_img_{}", std::process::id());
    let msg_count = 10usize;

    // Parent: create topic as publisher
    let topic: Topic<Image> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Send a dummy to register as producer
    {
        let dummy =
            Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("parent: Image::new");
        topic.send(&dummy);
    }

    // Spawn child (will register as consumer, trigger cross-process migration)
    let child = spawn_child("cross_process_image_roundtrip", &topic_name, msg_count);

    // Wait for child to register and trigger cross-process migration
    std::thread::sleep(1500_u64.ms());

    // Force migration check
    topic.check_migration_now();

    // Send Image messages with deterministic pixel patterns
    for i in 1..=msg_count as u32 {
        let mut img =
            Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("Image::new");
        fill_pattern(&mut img, i);
        img.set_frame_id(&format!("frame_{}", i));
        img.set_timestamp_ns(i as u64 * 1_000_000);
        topic.send(&img);

        // Small delay to avoid overwhelming the ring
        std::thread::sleep(10_u64.ms());
    }

    // Wait for child to finish
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        panic!(
            "Child process CRASHED (exit {:?}):\nstdout: {}\nstderr: {}",
            output.status.code(),
            stdout,
            stderr
        );
    }

    let (received, valid, errors) = parse_child_output(&stdout);

    if !errors.is_empty() {
        panic!(
            "Child reported errors:\n{}\nstdout: {}\nstderr: {}",
            errors.join("\n"),
            stdout,
            stderr
        );
    }

    assert!(
        received > 0,
        "Child received 0 images.\nstdout: {}\nstderr: {}",
        stdout,
        stderr
    );

    eprintln!(
        "Cross-process Image roundtrip: child received {}/{} images ({} valid)",
        received, msg_count, valid
    );
}

// ============================================================================
// Test 2: Cross-process Image high-throughput (stress test)
// ============================================================================

#[test]
fn cross_process_image_stress() {
    if is_child() {
        child_recv_images();
        return;
    }

    let topic_name = format!("xproc_img_stress_{}", std::process::id());
    let msg_count = 100usize;

    let topic: Topic<Image> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Register as producer
    {
        let dummy =
            Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("parent: Image::new");
        topic.send(&dummy);
    }

    let child = spawn_child("cross_process_image_stress", &topic_name, msg_count);

    std::thread::sleep(1500_u64.ms());
    topic.check_migration_now();

    // Burst send at higher rate
    for i in 1..=msg_count as u32 {
        let mut img =
            Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("Image::new");
        fill_pattern(&mut img, i);
        topic.send(&img);
    }

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "Child crashed under image stress.\nstdout: {}\nstderr: {}",
        stdout,
        stderr
    );

    let (received, _valid, errors) = parse_child_output(&stdout);

    if !errors.is_empty() {
        eprintln!(
            "Child reported {} errors (first 5):\n{}",
            errors.len(),
            errors
                .iter()
                .take(5)
                .cloned()
                .collect::<Vec<_>>()
                .join("\n")
        );
    }

    eprintln!(
        "Cross-process Image stress: child received {}/{} images ({}%)",
        received,
        msg_count,
        if msg_count > 0 {
            received * 100 / msg_count
        } else {
            0
        }
    );
}

// ============================================================================
// Test 3: Two-scheduler simulation (mimics real horus launch)
// ============================================================================

#[test]
fn cross_process_two_schedulers_image() {
    if is_child() {
        child_recv_images();
        return;
    }

    let topic_name = format!("xproc_sched_img_{}", std::process::id());
    let msg_count = 20usize;

    // Simulate scheduler 1 (publisher)
    let topic: Topic<Image> = Topic::new(&topic_name).expect("scheduler1: Topic::new");

    // Register
    {
        let dummy = Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("Image::new");
        topic.send(&dummy);
    }

    // Spawn scheduler 2 (subscriber) as separate process
    let child = spawn_child("cross_process_two_schedulers_image", &topic_name, msg_count);

    // Wait for cross-process detection and migration
    std::thread::sleep(2000_u64.ms());
    topic.check_migration_now();

    // Simulate scheduler ticks at 30Hz
    for i in 1..=msg_count as u32 {
        let mut img =
            Image::new(IMAGE_WIDTH, IMAGE_HEIGHT, ImageEncoding::Rgb8).expect("Image::new");
        fill_pattern(&mut img, i);
        topic.send(&img);
        std::thread::sleep(33_u64.ms()); // ~30Hz
    }

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        panic!(
            "Scheduler 2 (subscriber) CRASHED (exit {:?}):\nstdout: {}\nstderr: {}",
            output.status.code(),
            stdout,
            stderr
        );
    }

    let (received, valid, errors) = parse_child_output(&stdout);

    if !errors.is_empty() {
        panic!(
            "Scheduler 2 reported errors:\n{}\nstdout: {}\nstderr: {}",
            errors.join("\n"),
            stdout,
            stderr
        );
    }

    assert!(
        received > 0,
        "Scheduler 2 received 0 images.\nstdout: {}\nstderr: {}",
        stdout,
        stderr
    );

    eprintln!(
        "Two-scheduler Image pub/sub: received {}/{} images ({} valid)",
        received, msg_count, valid
    );
}
