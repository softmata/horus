//! Issue #37 reproduction — subscriber binary.
//! Receives Image messages from a topic.

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_core::memory::Image;
use horus_core::scheduling::Scheduler;
use horus_core::Node;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

struct TestRecvNode {
    topic: Topic<Image>,
    received: Arc<AtomicU32>,
    valid: Arc<AtomicU32>,
}

impl Node for TestRecvNode {
    fn name(&self) -> &str {
        "test_receive_node"
    }

    fn tick(&mut self) {
        if let Some(img) = self.topic.recv() {
            let r = self.received.fetch_add(1, Ordering::Relaxed);
            let data = img.data();
            let non_zero = data.iter().filter(|&&b| b != 0).count();
            let first_8: Vec<u8> = data.iter().take(8).copied().collect();
            if non_zero > 0 {
                self.valid.fetch_add(1, Ordering::Relaxed);
                eprintln!(
                    "[recv] frame={} w={} h={} len={} nz={} first={:?}",
                    r,
                    img.width(),
                    img.height(),
                    data.len(),
                    non_zero,
                    first_8
                );
            } else {
                eprintln!(
                    "[recv] frame={} ZEROED w={} h={} len={}",
                    r,
                    img.width(),
                    img.height(),
                    data.len()
                );
            }
        }
    }
}

fn main() {
    let topic_name = std::env::var("TOPIC").unwrap_or_else(|_| "test_send.image".into());

    let received = Arc::new(AtomicU32::new(0));
    let valid = Arc::new(AtomicU32::new(0));

    let node = TestRecvNode {
        topic: Topic::new(&topic_name).expect("Topic::new"),
        received: received.clone(),
        valid: valid.clone(),
    };

    let mut scheduler = Scheduler::new().tick_rate(30_u64.hz());
    scheduler
        .add(node)
        .order(0)
        .rate(5_u64.hz()) // 5Hz like user's code
        .build()
        .expect("add node");

    // Run for 12 seconds
    let deadline = std::time::Instant::now() + 12_u64.secs();
    while std::time::Instant::now() < deadline {
        let _ = scheduler.tick_once();
    }

    let r = received.load(Ordering::Relaxed);
    let v = valid.load(Ordering::Relaxed);
    eprintln!("[recv] done: received={}, valid={}", r, v);

    // Exit 0 if we got valid data, 1 if nothing or corrupted
    if r > 0 && v == r {
        std::process::exit(0);
    } else if r == 0 {
        eprintln!("[recv] FAIL: received 0 images");
        std::process::exit(1);
    } else {
        eprintln!("[recv] FAIL: {}/{} images had zeroed data", r - v, r);
        std::process::exit(1);
    }
}
