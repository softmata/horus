//! Production Message Benchmarks
//!
//! Benchmarks using actual HORUS robotics message types to measure
//! real-world performance with production data structures.
//!
//! ## Message Types
//!
//! | Message      | Size       | Use Case                    |
//! |--------------|------------|-----------------------------|
//! | CmdVel       | 16B        | Velocity commands           |
//! | BatteryState | ~64B       | Status monitoring           |
//! | IMU          | ~500B      | Inertial measurement        |
//! | Odometry     | ~700B      | Pose + velocity             |
//! | LaserScan    | ~1.5KB     | 2D lidar (360 rays)         |
//! | PointCloud   | Variable   | 3D perception               |
//!
//! ## Running Benchmarks
//!
//! ```bash
//! cargo bench --bench production_messages
//! cargo bench --bench production_messages -- "small_messages"
//! cargo bench --bench production_messages -- "pointcloud"
//! ```

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use horus::communication::Topic;
use horus_library::messages::{
    cmd_vel::CmdVel,
    geometry::Point3,
    perception::PointCloud,
    sensor::{BatteryState, Imu, LaserScan, Odometry},
};
use std::time::Duration;

// =============================================================================
// Section 1: Small Messages (Control & Status)
// =============================================================================

/// Benchmark small control messages (CmdVel: 16 bytes)
fn bench_small_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("small_messages");
    group.measurement_time(Duration::from_secs(5));

    // CmdVel - velocity command (16 bytes)
    group.throughput(Throughput::Bytes(std::mem::size_of::<CmdVel>() as u64));
    group.bench_function("CmdVel_16B", |b| {
        let topic = format!("bench_cmdvel_{}", std::process::id());
        let sender: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let receiver: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            sender.send(black_box(msg), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // BatteryState - status monitoring
    group.throughput(Throughput::Bytes(std::mem::size_of::<BatteryState>() as u64));
    group.bench_function("BatteryState", |b| {
        let topic = format!("bench_battery_{}", std::process::id());
        let sender: Topic<BatteryState> = Topic::new(&topic).unwrap();
        let receiver: Topic<BatteryState> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let battery = BatteryState::new(12.6, 75.0);
            sender.send(black_box(battery), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 2: Medium Messages (Sensor Data)
// =============================================================================

/// Benchmark medium-sized sensor messages
fn bench_medium_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("medium_messages");
    group.measurement_time(Duration::from_secs(5));

    // IMU - inertial measurement (~500 bytes)
    group.throughput(Throughput::Bytes(std::mem::size_of::<Imu>() as u64));
    group.bench_function("IMU_500B", |b| {
        let topic = format!("bench_imu_{}", std::process::id());
        let sender: Topic<Imu> = Topic::new(&topic).unwrap();
        let receiver: Topic<Imu> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let mut imu = Imu::new();
            imu.set_orientation_from_euler(0.1, 0.2, 0.3);
            imu.angular_velocity = [0.01, 0.02, 0.03];
            imu.linear_acceleration = [9.8, 0.1, 0.1];
            sender.send(black_box(imu), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    // Odometry - pose + velocity (~700 bytes)
    group.throughput(Throughput::Bytes(std::mem::size_of::<Odometry>() as u64));
    group.bench_function("Odometry_700B", |b| {
        let topic = format!("bench_odom_{}", std::process::id());
        let sender: Topic<Odometry> = Topic::new(&topic).unwrap();
        let receiver: Topic<Odometry> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let mut odom = Odometry::new();
            odom.pose.x = 1.5;
            odom.pose.y = 2.3;
            odom.pose.theta = 0.8;
            odom.twist.linear[0] = 0.5;
            odom.twist.angular[2] = 0.1;
            sender.send(black_box(odom), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 3: Large Messages (Perception)
// =============================================================================

/// Benchmark large perception messages
fn bench_large_messages(c: &mut Criterion) {
    let mut group = c.benchmark_group("large_messages");
    group.measurement_time(Duration::from_secs(5));

    // LaserScan - 2D lidar (~1.5KB for 360 rays)
    group.throughput(Throughput::Bytes(std::mem::size_of::<LaserScan>() as u64));
    group.bench_function("LaserScan_1.5KB", |b| {
        let topic = format!("bench_laser_{}", std::process::id());
        let sender: Topic<LaserScan> = Topic::new(&topic).unwrap();
        let receiver: Topic<LaserScan> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let mut scan = LaserScan::new();
            for i in 0..360 {
                scan.ranges[i] = 5.0 + (i as f32 * 0.01);
            }
            sender.send(black_box(scan), &mut None).unwrap();
            black_box(receiver.recv(&mut None))
        });
    });

    group.finish();
}

// =============================================================================
// Section 4: PointCloud Scaling (Variable Size)
// =============================================================================

/// Benchmark PointCloud with different point counts
fn bench_pointcloud_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("pointcloud_scaling");
    group.measurement_time(Duration::from_secs(5));

    let point_counts = [100, 1000, 10000];

    for num_points in point_counts {
        let bytes = num_points * 12; // 3 * f32 per point
        group.throughput(Throughput::Bytes(bytes as u64));

        group.bench_with_input(
            BenchmarkId::new("points", num_points),
            &num_points,
            |b, &num_points| {
                let topic = format!("bench_cloud_{}_{}", num_points, std::process::id());
                let sender: Topic<PointCloud> = Topic::new(&topic).unwrap();
                let receiver: Topic<PointCloud> = Topic::new(&topic).unwrap();

                b.iter(|| {
                    let points: Vec<Point3> = (0..num_points)
                        .map(|i| {
                            let t = i as f64 * 0.1;
                            Point3::new(t.sin(), t.cos(), t * 0.1)
                        })
                        .collect();

                    let cloud = PointCloud::xyz(&points);
                    sender.send(black_box(cloud), &mut None).unwrap();
                    black_box(receiver.recv(&mut None))
                });
            },
        );
    }

    group.finish();
}

// =============================================================================
// Section 5: Throughput at Target Frequencies
// =============================================================================

/// Benchmark sustained throughput at typical robotics frequencies
fn bench_frequency_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("frequency_throughput");
    group.measurement_time(Duration::from_secs(5));

    // CmdVel at 100Hz (typical control rate)
    group.bench_function("CmdVel_100Hz", |b| {
        let topic = format!("bench_cmdvel_100hz_{}", std::process::id());
        let sender: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let receiver: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for i in 0..100 {
                let msg = CmdVel::new(1.0 + (i as f32 * 0.01), 0.5);
                sender.send(black_box(msg), &mut None).unwrap();
                let _ = black_box(receiver.recv(&mut None));
            }
        });
    });

    // IMU at 200Hz (typical IMU rate)
    group.bench_function("IMU_200Hz", |b| {
        let topic = format!("bench_imu_200hz_{}", std::process::id());
        let sender: Topic<Imu> = Topic::new(&topic).unwrap();
        let receiver: Topic<Imu> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for _ in 0..200 {
                let mut imu = Imu::new();
                imu.angular_velocity = [0.01, 0.02, 0.03];
                sender.send(black_box(imu), &mut None).unwrap();
                let _ = black_box(receiver.recv(&mut None));
            }
        });
    });

    // LaserScan at 10Hz (typical lidar rate)
    group.bench_function("LaserScan_10Hz", |b| {
        let topic = format!("bench_laser_10hz_{}", std::process::id());
        let sender: Topic<LaserScan> = Topic::new(&topic).unwrap();
        let receiver: Topic<LaserScan> = Topic::new(&topic).unwrap();

        b.iter(|| {
            for i in 0..10 {
                let mut scan = LaserScan::new();
                for j in 0..360 {
                    scan.ranges[j] = 5.0 + ((i + j) as f32 * 0.01);
                }
                sender.send(black_box(scan), &mut None).unwrap();
                let _ = black_box(receiver.recv(&mut None));
            }
        });
    });

    group.finish();
}

// =============================================================================
// Section 6: Mixed Message Scenarios
// =============================================================================

/// Benchmark realistic robot control loop with multiple message types
fn bench_robot_control_loop(c: &mut Criterion) {
    let mut group = c.benchmark_group("robot_control_loop");
    group.measurement_time(Duration::from_secs(5));

    group.bench_function("100Hz_mixed", |b| {
        let cmd_topic = format!("bench_mix_cmd_{}", std::process::id());
        let imu_topic = format!("bench_mix_imu_{}", std::process::id());
        let odom_topic = format!("bench_mix_odom_{}", std::process::id());

        let cmd_tx: Topic<CmdVel> = Topic::new(&cmd_topic).unwrap();
        let cmd_rx: Topic<CmdVel> = Topic::new(&cmd_topic).unwrap();
        let imu_tx: Topic<Imu> = Topic::new(&imu_topic).unwrap();
        let imu_rx: Topic<Imu> = Topic::new(&imu_topic).unwrap();
        let odom_tx: Topic<Odometry> = Topic::new(&odom_topic).unwrap();
        let odom_rx: Topic<Odometry> = Topic::new(&odom_topic).unwrap();

        b.iter(|| {
            // Simulate one control cycle
            // 1. Read IMU
            let mut imu = Imu::new();
            imu.angular_velocity = [0.01, 0.02, 0.03];
            imu_tx.send(black_box(imu), &mut None).unwrap();
            let _ = black_box(imu_rx.recv(&mut None));

            // 2. Read Odometry
            let mut odom = Odometry::new();
            odom.pose.x = 1.0;
            odom.pose.y = 2.0;
            odom_tx.send(black_box(odom), &mut None).unwrap();
            let _ = black_box(odom_rx.recv(&mut None));

            // 3. Send Command
            let cmd = CmdVel::new(1.0, 0.5);
            cmd_tx.send(black_box(cmd), &mut None).unwrap();
            let _ = black_box(cmd_rx.recv(&mut None));
        });
    });

    group.finish();
}

// =============================================================================
// Section 7: Burst Traffic Patterns
// =============================================================================

/// Benchmark burst traffic handling (simulates sensor data bursts)
fn bench_burst_traffic(c: &mut Criterion) {
    let mut group = c.benchmark_group("burst_traffic");
    group.measurement_time(Duration::from_secs(5));

    // Burst of 10 LaserScans
    group.bench_function("LaserScan_burst_10", |b| {
        let topic = format!("bench_burst_laser_{}", std::process::id());
        let sender: Topic<LaserScan> = Topic::new(&topic).unwrap();
        let receiver: Topic<LaserScan> = Topic::new(&topic).unwrap();

        b.iter(|| {
            // Send burst
            for i in 0..10 {
                let mut scan = LaserScan::new();
                for j in 0..360 {
                    scan.ranges[j] = 5.0 + ((i + j) as f32 * 0.01);
                }
                sender.send(black_box(scan), &mut None).unwrap();
            }
            // Drain burst
            for _ in 0..10 {
                let _ = black_box(receiver.recv(&mut None));
            }
        });
    });

    // Burst of 100 CmdVel
    group.bench_function("CmdVel_burst_100", |b| {
        let topic = format!("bench_burst_cmd_{}", std::process::id());
        let sender: Topic<CmdVel> = Topic::new(&topic).unwrap();
        let receiver: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            // Send burst
            for i in 0..100 {
                let msg = CmdVel::new(1.0 + (i as f32 * 0.01), 0.5);
                sender.send(black_box(msg), &mut None).unwrap();
            }
            // Drain burst
            for _ in 0..100 {
                let _ = black_box(receiver.recv(&mut None));
            }
        });
    });

    group.finish();
}

// =============================================================================
// Section 8: Send-Only Latency (Syscall Overhead)
// =============================================================================

/// Measure raw send latency without recv (isolates send overhead)
fn bench_send_only(c: &mut Criterion) {
    let mut group = c.benchmark_group("send_only");
    group.measurement_time(Duration::from_secs(5));

    group.bench_function("CmdVel", |b| {
        let topic = format!("bench_send_cmdvel_{}", std::process::id());
        let sender: Topic<CmdVel> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let msg = CmdVel::new(1.5, 0.8);
            sender.send(black_box(msg), &mut None).unwrap();
        });
    });

    group.bench_function("LaserScan", |b| {
        let topic = format!("bench_send_laser_{}", std::process::id());
        let sender: Topic<LaserScan> = Topic::new(&topic).unwrap();

        b.iter(|| {
            let mut scan = LaserScan::new();
            for i in 0..360 {
                scan.ranges[i] = 5.0 + (i as f32 * 0.01);
            }
            sender.send(black_box(scan), &mut None).unwrap();
        });
    });

    group.finish();
}

// =============================================================================
// Criterion Configuration
// =============================================================================

criterion_group!(
    name = benches;
    config = Criterion::default()
        .sample_size(100)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(5));
    targets =
        bench_small_messages,
        bench_medium_messages,
        bench_large_messages,
        bench_pointcloud_scaling,
        bench_frequency_throughput,
        bench_robot_control_loop,
        bench_burst_traffic,
        bench_send_only,
);

criterion_main!(benches);
