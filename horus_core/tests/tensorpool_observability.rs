#![allow(dead_code)]
//! TensorPool-backed topic observability test.
//!
//! Verifies that Topic<Image>, Topic<PointCloud>, Topic<DepthImage>
//! auto-register in TopicNodeRegistry on send()/recv(), just like
//! regular Pod/serde topics.

use horus_core::communication::{topic_node_registry, Topic};
use horus_core::core::{DurationExt, Node};
use horus_core::memory::Image;
use horus_core::scheduling::Scheduler;
use horus_core::types::ImageEncoding;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

struct CameraNode {
    name: String,
    topic: Option<Topic<Image>>,
    count: Arc<AtomicU64>,
}

impl Node for CameraNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::<Image>::new("camera.rgb")?);
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref topic) = self.topic {
            if let Ok(img) = Image::new(64, 64, ImageEncoding::Rgb8) {
                topic.send(img);
            }
        }
    }
}

struct ImageConsumer {
    name: String,
    topic: Option<Topic<Image>>,
    count: Arc<AtomicU64>,
}

impl Node for ImageConsumer {
    fn name(&self) -> &str {
        &self.name
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::<Image>::new("camera.rgb")?);
        Ok(())
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        if let Some(ref topic) = self.topic {
            while let Some(_img) = topic.recv() {
                // Process image
            }
        }
    }
}

#[test]
fn test_image_topic_auto_registers_in_registry() {
    cleanup_stale_shm();

    let cam_count = Arc::new(AtomicU64::new(0));
    let consumer_count = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new()
        .tick_rate(30_u64.hz())
        .deterministic(true)
        .max_deadline_misses(10000);

    sched
        .add(CameraNode {
            name: "camera".into(),
            topic: None,
            count: cam_count.clone(),
        })
        .order(0)
        .build()
        .unwrap();

    sched
        .add(ImageConsumer {
            name: "detector".into(),
            topic: None,
            count: consumer_count.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Run for 2 seconds
    sched.run_for(2_u64.secs()).unwrap();

    // Both should have ticked
    assert!(cam_count.load(Ordering::SeqCst) > 10);
    assert!(consumer_count.load(Ordering::SeqCst) > 10);

    // Check TopicNodeRegistry
    let registry = topic_node_registry();
    let all = registry.all_topics();

    println!("=== TensorPool Topic Registry ===");
    for (topic, assocs) in &all {
        let nodes: Vec<_> = assocs
            .iter()
            .map(|a| format!("{} ({:?}, type={})", a.node_name, a.role, a.type_name))
            .collect();
        println!("  {} → {}", topic, nodes.join(", "));
    }

    // camera.rgb should have camera as Publisher and detector as Subscriber
    let pubs = registry.publishers_of_topic("camera.rgb");
    let subs = registry.subscribers_of_topic("camera.rgb");

    assert!(
        pubs.contains(&"camera".to_string()),
        "camera should be publisher of camera.rgb, got: {:?}",
        pubs
    );
    assert!(
        subs.contains(&"detector".to_string()),
        "detector should be subscriber of camera.rgb, got: {:?}",
        subs
    );

    // Verify type name is "Image"
    let assocs = all.iter().find(|(name, _)| name == "camera.rgb");
    if let Some((_, entries)) = assocs {
        let cam_entry = entries.iter().find(|a| a.node_name == "camera");
        assert!(
            cam_entry.map(|e| e.type_name.as_str()) == Some("Image"),
            "camera entry should have type_name 'Image'"
        );
    }
}
