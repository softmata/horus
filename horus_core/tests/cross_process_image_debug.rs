//! Focused debug test for cross-process Image pub/sub.
//! Reproduces GitHub issue #37 — no workarounds (no dummy, no check_migration_now).

use std::process::{Command, Stdio};
use std::time::Instant;

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_core::memory::Image;
use horus_core::types::ImageEncoding;

const CHILD_ENV: &str = "HORUS_IMG_DEBUG_CHILD";
const TOPIC_ENV: &str = "HORUS_IMG_DEBUG_TOPIC";
const TEST_ENV: &str = "HORUS_IMG_DEBUG_TEST";
const SIZE_ENV: &str = "HORUS_IMG_DEBUG_SIZE";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

fn parse_size() -> (u32, u32) {
    let s = std::env::var(SIZE_ENV).unwrap_or_else(|_| "4x4".into());
    let parts: Vec<&str> = s.split('x').collect();
    (parts[0].parse().unwrap(), parts[1].parse().unwrap())
}

fn child_main() {
    let topic_name = std::env::var(TOPIC_ENV).unwrap();
    let test = std::env::var(TEST_ENV).unwrap_or_default();
    let (exp_w, exp_h) = parse_size();
    let topic: Topic<Image> = Topic::new(&topic_name).expect("child Topic::new");

    let deadline = Instant::now() + 15_u64.secs();
    let mut count = 0u32;
    let target: u32 = match test.as_str() {
        "realistic" => 10,
        "large" => 5,
        _ => 5,
    };

    while Instant::now() < deadline && count < target {
        if let Some(img) = topic.recv() {
            let data = img.data();
            let first_8: Vec<u8> = data.iter().take(8).copied().collect();
            let non_zero = data.iter().filter(|&&b| b != 0).count();

            println!(
                "RECV:frame={} w={} h={} enc={:?} len={} nz={} first={:?}",
                count,
                img.width(),
                img.height(),
                img.encoding(),
                data.len(),
                non_zero,
                first_8
            );

            if img.width() != exp_w || img.height() != exp_h {
                println!(
                    "ERR:size_mismatch expected={}x{} got={}x{}",
                    exp_w,
                    exp_h,
                    img.width(),
                    img.height()
                );
            }

            count += 1;
        } else {
            std::thread::yield_now();
        }
    }
    println!("DONE:{}", count);
}

fn spawn_child(
    test_name: &str,
    topic_name: &str,
    test_id: &str,
    size: &str,
) -> std::process::Child {
    let exe = std::env::current_exe().unwrap();
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TOPIC_ENV, topic_name)
        .env(TEST_ENV, test_id)
        .env(SIZE_ENV, size)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap()
}

fn run_test(test_name: &str, test_id: &str, width: u32, height: u32, count: u32, rate_ms: u64) {
    let topic_name = format!("xproc_{}_{}", test_id, std::process::id());
    let size = format!("{}x{}", width, height);
    let topic: Topic<Image> = Topic::new(&topic_name).expect("Topic::new");

    let child = spawn_child(test_name, &topic_name, test_id, &size);
    std::thread::sleep(500_u64.ms());

    let img_bytes = (width * height * 3) as usize;
    eprintln!(
        "Parent: {}x{} ({} bytes/img), {} frames at {}ms",
        width, height, img_bytes, count, rate_ms
    );

    for i in 0..count {
        let img = Image::new(width, height, ImageEncoding::Rgb8).unwrap();
        let val = ((i as u16 + 1) * 7 % 256) as u8; // Avoid 0
        let data = img.data_mut();
        for b in data.iter_mut() {
            *b = val;
        }
        topic.send(&img);
        if rate_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(rate_ms));
        }
    }

    let output = child.wait_with_output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    eprintln!("=== Child stdout ===\n{}", stdout);
    if !stderr.is_empty() {
        eprintln!("=== Child stderr ===\n{}", stderr);
    }

    assert!(
        output.status.success(),
        "Child CRASHED: exit={:?}\nstderr: {}",
        output.status.code(),
        stderr
    );

    let received = stdout
        .lines()
        .find_map(|l| l.strip_prefix("DONE:"))
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(0);

    let valid = stdout
        .lines()
        .filter(|l| l.starts_with("RECV:") && l.contains("nz=") && !l.contains("nz=0"))
        .count();

    let errs: Vec<&str> = stdout.lines().filter(|l| l.starts_with("ERR:")).collect();

    eprintln!(
        "Result: received={}/{}, valid_data={}/{}, errors={}",
        received,
        count,
        valid,
        received,
        errs.len()
    );

    assert!(received > 0, "Child received 0 images. stdout:\n{}", stdout);

    if !errs.is_empty() {
        panic!("Errors: {:?}", errs);
    }

    assert_eq!(
        valid, received as usize,
        "Some images had zeroed pixel data! stdout:\n{}",
        stdout
    );
}

/// Small images (4x4) — baseline
#[test]
fn cross_process_image_small() {
    if is_child() {
        child_main();
        return;
    }
    run_test("cross_process_image_small", "small", 4, 4, 5, 100);
}

/// Medium images (64x48 = 9KB) — original test size
#[test]
fn cross_process_image_medium() {
    if is_child() {
        child_main();
        return;
    }
    run_test("cross_process_image_medium", "medium", 64, 48, 5, 200);
}

/// Large images (324x244 = 237KB) — original user report
#[test]
fn cross_process_image_large() {
    if is_child() {
        child_main();
        return;
    }
    run_test("cross_process_image_large", "large", 324, 244, 5, 500);
}

/// VGA images (640x480 = 921KB) — common camera resolution
#[test]
fn cross_process_image_vga() {
    if is_child() {
        child_main();
        return;
    }
    run_test("cross_process_image_vga", "vga", 640, 480, 5, 500);
}

/// User's exact scenario (3x2 at 1Hz)
#[test]
fn cross_process_image_user_scenario() {
    if is_child() {
        child_main();
        return;
    }
    run_test(
        "cross_process_image_user_scenario",
        "realistic",
        3,
        2,
        10,
        1000,
    );
}
