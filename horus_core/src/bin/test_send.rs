//! Issue #37 reproduction — publisher binary.
//! Sends Image messages on a topic at 1Hz.

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_core::memory::Image;
use horus_core::scheduling::Scheduler;
use horus_core::types::ImageEncoding;
use horus_core::Node;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

struct TestSendNode {
    topic: Topic<Image>,
    frame: Arc<AtomicU32>,
    width: u32,
    height: u32,
}

impl Node for TestSendNode {
    fn name(&self) -> &str {
        "test_send_node"
    }

    fn tick(&mut self) {
        let i = self.frame.fetch_add(1, Ordering::Relaxed);
        let img = Image::new(self.width, self.height, ImageEncoding::Rgb8).expect("Image::new");
        let val = ((i + 1) * 7 % 256) as u8;
        let data = img.data_mut();
        for b in data.iter_mut() {
            *b = val;
        }
        self.topic.send(&img);
        eprintln!(
            "[send] frame={} val={} size={}x{}",
            i, val, self.width, self.height
        );
    }
}

fn main() {
    let topic_name = std::env::var("TOPIC").unwrap_or_else(|_| "test_send.image".into());
    let width: u32 = std::env::var("WIDTH")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(3);
    let height: u32 = std::env::var("HEIGHT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);

    let frame = Arc::new(AtomicU32::new(0));
    let node = TestSendNode {
        topic: Topic::new(&topic_name).expect("Topic::new"),
        frame: frame.clone(),
        width,
        height,
    };

    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());
    scheduler
        .add(node)
        .order(0)
        .rate(1_u64.hz())
        .build()
        .expect("add node");

    // Run for 10 seconds
    let deadline = std::time::Instant::now() + 10_u64.secs();
    while std::time::Instant::now() < deadline {
        let _ = scheduler.tick_once();
    }
    eprintln!("[send] done, sent {} frames", frame.load(Ordering::Relaxed));
}
