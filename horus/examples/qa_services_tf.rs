/// QA Test: Services + TransformFrame (runs indefinitely)
///
/// - Service: "AddTwoInts" — adds two i64s
/// - TF: publishes world→base→shoulder→elbow→wrist on "tf" topic at 50Hz
///
/// Terminal 1: cargo run --no-default-features --example qa_services_tf
/// Terminal 2: horus service list, horus frame list, etc.

use horus::prelude::*;
use horus_tf::{
    Transform, TransformFrame, TFMessage, TransformStamped, string_to_frame_id,
};

// --- Service definition ---

service! {
    /// Add two integers
    AddTwoInts {
        request {
            a: i64,
            b: i64,
        }
        response {
            sum: i64,
        }
    }
}

// --- TF broadcaster node ---
// Publishes TFMessage on the "tf" topic so `horus frame list/tree/echo` can see it.
// Also maintains a local TransformFrame for in-process lookups.

struct TfBroadcaster {
    tf: TransformFrame,
    tf_topic: Topic<TFMessage>,
    tick_count: u64,
}

impl TfBroadcaster {
    fn new() -> Result<Self> {
        let tf = TransformFrame::new();

        // Register frame hierarchy: world → base → shoulder → elbow → wrist
        tf.register_frame("world", None)?;
        tf.register_frame("base", Some("world"))?;
        tf.register_frame("shoulder", Some("base"))?;
        tf.register_frame("elbow", Some("shoulder"))?;
        tf.register_frame("wrist", Some("elbow"))?;

        let tf_topic = Topic::new("tf")?;

        Ok(Self {
            tf,
            tf_topic,
            tick_count: 0,
        })
    }

    fn make_stamped(parent: &str, child: &str, transform: &Transform, timestamp_ns: u64) -> TransformStamped {
        let mut stamped = TransformStamped::default();
        string_to_frame_id(parent, &mut stamped.parent_frame);
        string_to_frame_id(child, &mut stamped.child_frame);
        stamped.timestamp_ns = timestamp_ns;
        stamped.transform = *transform;
        stamped
    }
}

impl Node for TfBroadcaster {
    fn name(&self) -> &str {
        "tf_broadcaster"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        let t = self.tick_count as f64 * 0.02;
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Build transforms
        let base_tf = Transform::from_translation([0.0, 0.0, 1.0]);
        let shoulder_tf = Transform::from_translation([0.0, 0.0, 0.3]);
        let x = 0.4 * t.cos();
        let z = 0.4 * t.sin();
        let elbow_tf = Transform::from_translation([x, 0.0, z]);
        let wrist_tf = Transform::from_translation([0.2, 0.0, 0.0]);

        // Update local TransformFrame (for in-process lookups)
        let _ = self.tf.update_transform("base", &base_tf, now_ns);
        let _ = self.tf.update_transform("shoulder", &shoulder_tf, now_ns);
        let _ = self.tf.update_transform("elbow", &elbow_tf, now_ns);
        let _ = self.tf.update_transform("wrist", &wrist_tf, now_ns);

        // Publish to SHM topic (for `horus frame list/tree/echo`)
        let mut msg = TFMessage::default();
        msg.transforms[0] = Self::make_stamped("world", "base", &base_tf, now_ns);
        msg.transforms[1] = Self::make_stamped("base", "shoulder", &shoulder_tf, now_ns);
        msg.transforms[2] = Self::make_stamped("shoulder", "elbow", &elbow_tf, now_ns);
        msg.transforms[3] = Self::make_stamped("elbow", "wrist", &wrist_tf, now_ns);
        msg.count = 4;
        self.tf_topic.send(msg);
    }
}

fn main() -> Result<()> {
    println!("=== QA Services + TF (Ctrl+C to stop) ===");

    // Start service server (background thread)
    let _server = ServiceServerBuilder::<AddTwoInts>::new()
        .on_request(|req| {
            let sum = req.a + req.b;
            Ok(AddTwoIntsResponse { sum })
        })
        .build()?;

    println!("Service 'AddTwoInts' listening");

    let mut scheduler = Scheduler::new()
        .name("qa_services_tf")
        .tick_rate(50_u64.hz());

    scheduler
        .add(TfBroadcaster::new()?)
        .rate(50_u64.hz())
        .order(0)
        .build()?;

    println!("Scheduler running...");
    scheduler.run()?;

    Ok(())
}
