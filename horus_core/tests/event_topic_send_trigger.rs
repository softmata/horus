// Proves that Topic::send() now triggers event node ticks automatically.
//
// This was previously broken — Topic::send() did NOT call notify_event(),
// so event nodes with .on("topic") never ticked when data was published
// via Topic::send(). Fixed by wiring Topic::send() → NodeInfo::notify_event()
// and registering event notifiers under the topic name (not just node name).

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

fn unique(prefix: &str) -> String {
    static CTR: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        CTR.fetch_add(1, Ordering::Relaxed)
    )
}

struct EventCounter {
    name: String,
    ticks: Arc<AtomicU64>,
}
impl Node for EventCounter {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::SeqCst);
    }
}

struct TopicPublisher {
    name: String,
    topic: Option<Topic<u64>>,
    topic_name: String,
    count: u64,
    sent: u64,
}
impl Node for TopicPublisher {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if self.sent < self.count {
            if let Some(ref t) = self.topic {
                t.send(self.sent);
                self.sent += 1;
            }
        }
    }
}

// ============================================================================
// THE KEY TEST: Topic::send() triggers event node tick
// ============================================================================

#[test]
fn test_topic_send_triggers_event_node() {
    cleanup_stale_shm();

    let topic_name = unique("evt_send_trigger");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_recv_{}", std::process::id()),
        ticks: event_ticks.clone(),
    };

    let publisher = TopicPublisher {
        name: format!("evt_send_{}", std::process::id()),
        topic: None,
        topic_name: topic_name.clone(),
        count: 20,
        sent: 0,
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    // Publisher as BestEffort node — uses Topic::send() internally
    sched.add(publisher).order(0).build().unwrap();
    // Event node — should tick when Topic::send() publishes to topic_name
    sched
        .add(event_node)
        .order(1)
        .on(&topic_name)
        .build()
        .unwrap();

    sched.run_for(Duration::from_millis(500)).unwrap();

    let ticks = event_ticks.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Event node should tick when Topic::send() publishes data. Got {} ticks. \
         This proves the Topic::send() → notify_event() wiring works.",
        ticks
    );
}

// ============================================================================
// Verify: no tick without publish (regression check)
// ============================================================================

#[test]
fn test_event_no_tick_without_topic_send() {
    cleanup_stale_shm();

    let topic_name = unique("evt_no_send");
    let event_ticks = Arc::new(AtomicU64::new(0));

    let event_node = EventCounter {
        name: format!("evt_nosend_{}", std::process::id()),
        ticks: event_ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(event_node)
        .order(0)
        .on(&topic_name)
        .build()
        .unwrap();
    // No publisher — event node should not tick
    sched.run_for(Duration::from_millis(300)).unwrap();

    let ticks = event_ticks.load(Ordering::SeqCst);
    assert_eq!(
        ticks, 0,
        "Event node should NOT tick without data, got {}",
        ticks
    );
}
