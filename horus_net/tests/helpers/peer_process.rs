//! Test helper binary — acts as a remote peer process.
//!
//! Usage:
//!   peer_process write_raw <topic_name> <count> [msg_type]
//!   peer_process read_raw <topic_name> <timeout_secs>
//!
//! msg_type: cmdvel (default), imu, jointstate, pose2d, laserscan, odometry

use std::time::{Duration, Instant};

use horus_core::communication::{read_latest_slot_bytes, write_topic_slot_bytes, Topic};
use horus_robotics::{CmdVel, Imu, JointState, LaserScan, Odometry};
use horus_types::Pose2D;
use horus_robotics::messages::vision::CompressedImage;
use horus_sys::shm::shm_topics_dir;

fn shm_path(name: &str) -> std::path::PathBuf {
    shm_topics_dir().join(format!("horus_{name}"))
}

fn write_pod<T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static>(
    name: &str,
    count: u32,
    make_msg: impl Fn(u32) -> T,
) {
    let _topic: Topic<T> = Topic::new(name).expect("create topic");
    let path = shm_path(name);
    for i in 0..count {
        let msg = make_msg(i);
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(
                &msg as *const T as *const u8,
                std::mem::size_of::<T>(),
            )
        };
        write_topic_slot_bytes(&path, bytes);
    }
    println!("WROTE_RAW {count}");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: peer_process <write_raw|read_raw> <topic> [count|timeout] [msg_type]");
        std::process::exit(1);
    }

    let mode = &args[1];
    let topic_name = &args[2];

    match mode.as_str() {
        "write_raw" => {
            let count: u32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(100);
            let msg_type = args.get(4).map(|s| s.as_str()).unwrap_or("cmdvel");

            match msg_type {
                "cmdvel" => write_pod::<CmdVel>(topic_name, count, |i| {
                    CmdVel::new(i as f32, (i as f32) * 0.01)
                }),
                "imu" => write_pod::<Imu>(topic_name, count, |i| {
                    let mut imu = Imu::default();
                    imu.linear_acceleration[0] = i as f64 * 0.01;
                    imu.timestamp_ns = i as u64 * 1000;
                    imu
                }),
                "jointstate" => write_pod::<JointState>(topic_name, count, |i| {
                    let mut js = JointState::default();
                    js.joint_count = 6;
                    js.positions[0] = i as f64 * 0.1;
                    js.timestamp_ns = i as u64 * 1000;
                    js
                }),
                "pose2d" => write_pod::<Pose2D>(topic_name, count, |i| {
                    Pose2D {
                        x: i as f64,
                        y: i as f64 * 0.5,
                        theta: (i as f64) * 0.01,
                        timestamp_ns: i as u64 * 1000,
                    }
                }),
                "laserscan" => write_pod::<LaserScan>(topic_name, count, |i| {
                    let mut scan = LaserScan::default();
                    scan.ranges[0] = i as f32 * 0.1;
                    scan.timestamp_ns = i as u64 * 1000;
                    scan
                }),
                "odometry" => write_pod::<Odometry>(topic_name, count, |i| {
                    let mut odom = Odometry::default();
                    odom.pose.x = i as f64 * 0.01;
                    odom.timestamp_ns = i as u64 * 1000;
                    odom
                }),
                other => {
                    eprintln!("Unknown msg_type: {other}");
                    std::process::exit(1);
                }
            }
        }

        "publish" => {
            // Use Topic::send() — the REAL user API
            let count: u32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(100);
            let msg_type = args.get(4).map(|s| s.as_str()).unwrap_or("cmdvel");

            match msg_type {
                "cmdvel" => {
                    let topic: Topic<CmdVel> = Topic::new(topic_name).expect("create topic");
                    for i in 0..count {
                        topic.send(CmdVel::new(i as f32, (i as f32) * 0.01));
                    }
                }
                "imu" => {
                    let topic: Topic<Imu> = Topic::new(topic_name).expect("create topic");
                    for i in 0..count {
                        let mut imu = Imu::default();
                        imu.linear_acceleration[0] = i as f64 * 0.1;
                        imu.timestamp_ns = i as u64 * 1000;
                        topic.send(imu);
                    }
                }
                "pose2d" => {
                    let topic: Topic<Pose2D> = Topic::new(topic_name).expect("create topic");
                    for i in 0..count {
                        topic.send(Pose2D {
                            x: i as f64,
                            y: i as f64 * 0.5,
                            theta: (i as f64) * 0.01,
                            timestamp_ns: i as u64 * 1000,
                        });
                    }
                }
                "compressed_image" => {
                    let topic: Topic<CompressedImage> = Topic::new(topic_name).expect("create topic");
                    for i in 0..count {
                        let mut img_data = vec![0xFFu8; 50_000];
                        img_data[0] = (i % 256) as u8;
                        img_data[1] = ((i / 256) % 256) as u8;
                        let img = CompressedImage::new("jpeg", img_data);
                        topic.send(img);
                    }
                }
                other => {
                    eprintln!("Unknown msg_type for publish: {other}");
                    std::process::exit(1);
                }
            }
            println!("PUBLISHED {count}");
        }

        "subscribe" => {
            // Use Topic::recv() — the REAL user API
            let timeout_secs: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(5.0);
            let msg_type = args.get(4).map(|s| s.as_str()).unwrap_or("cmdvel");
            let deadline = Instant::now() + Duration::from_secs_f64(timeout_secs);
            let mut count = 0u32;

            match msg_type {
                "cmdvel" => {
                    let topic: Topic<CmdVel> = Topic::new(topic_name).expect("create topic");
                    while Instant::now() < deadline {
                        if let Some(cmd) = topic.recv() {
                            assert!(cmd.linear.is_finite(), "corrupt CmdVel linear");
                            count += 1;
                        }
                        std::thread::yield_now();
                    }
                }
                "imu" => {
                    let topic: Topic<Imu> = Topic::new(topic_name).expect("create topic");
                    while Instant::now() < deadline {
                        if let Some(imu) = topic.recv() {
                            assert!(imu.linear_acceleration[0].is_finite(), "corrupt Imu");
                            count += 1;
                        }
                        std::thread::yield_now();
                    }
                }
                "pose2d" => {
                    let topic: Topic<Pose2D> = Topic::new(topic_name).expect("create topic");
                    while Instant::now() < deadline {
                        if let Some(pose) = topic.recv() {
                            assert!(pose.x.is_finite(), "corrupt Pose2D");
                            count += 1;
                        }
                        std::thread::yield_now();
                    }
                }
                "compressed_image" => {
                    let topic: Topic<CompressedImage> = Topic::new(topic_name).expect("create topic");
                    while Instant::now() < deadline {
                        if let Some(img) = topic.recv() {
                            assert_eq!(img.data.len(), 50_000, "image data size mismatch");
                            count += 1;
                        }
                        std::thread::yield_now();
                    }
                }
                other => {
                    eprintln!("Unknown msg_type for subscribe: {other}");
                    std::process::exit(1);
                }
            }
            println!("SUBSCRIBED {count}");
        }

        "read_raw" => {
            let timeout_secs: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(5.0);
            let path = shm_path(topic_name);

            let deadline = Instant::now() + Duration::from_secs_f64(timeout_secs);
            let mut last_idx = 0u64;
            let mut count = 0u32;
            let mut corrupt = 0u32;

            while !path.exists() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(10));
            }

            while Instant::now() < deadline {
                if let Some(slot) = read_latest_slot_bytes(&path, last_idx) {
                    last_idx = slot.write_idx;
                    count += 1;
                    // Basic integrity: check payload isn't all zeros (unlikely for real data)
                    if slot.payload.iter().all(|&b| b == 0) && count > 1 {
                        corrupt += 1;
                    }
                }
                std::thread::yield_now();
            }
            println!("READ_RAW {count} CORRUPT {corrupt}");
        }

        other => {
            eprintln!("Unknown mode: {other}");
            std::process::exit(1);
        }
    }
}
