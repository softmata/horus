/// QA Test: Services + TransformStamped broadcasting (runs indefinitely)
///
/// - Service: "AddTwoInts" — adds two i64s
/// - TF: publishes TransformStamped messages on "tf" topic at 50Hz
///   representing a kinematic chain: world->base->shoulder->elbow->wrist
///
/// Terminal 1: cargo run --no-default-features --example qa_services_tf
/// Terminal 2: horus service list, horus topic echo tf, etc.
use horus::prelude::*;

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

// --- Local TF message type ---
// horus_tf is only available as a dev-dependency in horus_core, not in the
// horus umbrella crate examples. We define a local fixed-size TF message
// for broadcasting transforms over a topic.

message! {
    /// A bundle of up to 8 named transforms for broadcasting.
    TfBundle {
        count: u32,
        // Flat arrays — index i corresponds to one transform entry.
        // Frame name stored as first 31 bytes of a [u8; 32] (null-terminated).
        parent_names: [[u8; 32]; 8],
        child_names:  [[u8; 32]; 8],
        translations: [[f64; 3]; 8],
        rotations:    [[f64; 4]; 8],
        timestamps:   [u64; 8],
    }
}

fn set_name(buf: &mut [u8; 32], name: &str) {
    let bytes = name.as_bytes();
    let len = bytes.len().min(31);
    buf[..len].copy_from_slice(&bytes[..len]);
    buf[len..].fill(0);
}

// --- TF broadcaster node ---
// Publishes a TfBundle on the "tf" topic so tools can observe the kinematic chain.

struct TfBroadcaster {
    tf_topic: Topic<TfBundle>,
    tick_count: u64,
}

impl TfBroadcaster {
    fn new() -> Result<Self> {
        let tf_topic = Topic::new("tf")?;
        Ok(Self {
            tf_topic,
            tick_count: 0,
        })
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

        // Kinematic chain: world -> base -> shoulder -> elbow -> wrist
        let identity_rot = [0.0, 0.0, 0.0, 1.0];

        let mut msg = TfBundle {
            count: 4,
            parent_names: Default::default(),
            child_names: Default::default(),
            translations: Default::default(),
            rotations: Default::default(),
            timestamps: Default::default(),
        };

        // world -> base (fixed, 1m up)
        set_name(&mut msg.parent_names[0], "world");
        set_name(&mut msg.child_names[0], "base");
        msg.translations[0] = [0.0, 0.0, 1.0];
        msg.rotations[0] = identity_rot;
        msg.timestamps[0] = now_ns;

        // base -> shoulder (fixed, 0.3m up)
        set_name(&mut msg.parent_names[1], "base");
        set_name(&mut msg.child_names[1], "shoulder");
        msg.translations[1] = [0.0, 0.0, 0.3];
        msg.rotations[1] = identity_rot;
        msg.timestamps[1] = now_ns;

        // shoulder -> elbow (oscillating)
        let x = 0.4 * t.cos();
        let z = 0.4 * t.sin();
        set_name(&mut msg.parent_names[2], "shoulder");
        set_name(&mut msg.child_names[2], "elbow");
        msg.translations[2] = [x, 0.0, z];
        msg.rotations[2] = identity_rot;
        msg.timestamps[2] = now_ns;

        // elbow -> wrist (fixed, 0.2m forward)
        set_name(&mut msg.parent_names[3], "elbow");
        set_name(&mut msg.child_names[3], "wrist");
        msg.translations[3] = [0.2, 0.0, 0.0];
        msg.rotations[3] = identity_rot;
        msg.timestamps[3] = now_ns;

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
