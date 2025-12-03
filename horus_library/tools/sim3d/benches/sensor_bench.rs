// Benchmark for Sensor performance
// Run with: cargo bench --bench sensor_bench

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::{vector, Vector3};
use rapier3d::prelude::*;

/// Create a physics world with obstacles for sensor testing
fn create_sensor_test_world() -> (RigidBodySet, ColliderSet, QueryPipeline) {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Add ground
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);
    let ground_collider = ColliderBuilder::cuboid(50.0, 0.1, 50.0).build();
    collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

    // Add obstacles
    for i in 0..20 {
        for j in 0..20 {
            let x = (i as f32 - 10.0) * 3.0;
            let z = (j as f32 - 10.0) * 3.0;
            let height = 0.5 + (i * j % 5) as f32 * 0.5;

            let rb = RigidBodyBuilder::fixed()
                .translation(vector![x, height / 2.0, z])
                .build();
            let handle = rigid_body_set.insert(rb);

            let collider = ColliderBuilder::cuboid(0.5, height / 2.0, 0.5).build();
            collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);
        }
    }

    let mut query_pipeline = QueryPipeline::new();
    query_pipeline.update(&collider_set);

    (rigid_body_set, collider_set, query_pipeline)
}

/// Simulate GPS position calculation with noise
fn simulate_gps_update(
    true_position: Vector3<f32>,
    noise_std: f32,
) -> Vector3<f32> {
    // Simple noise model (in real impl would use rand)
    let noise = vector![
        (true_position.x * 0.01).sin() * noise_std,
        0.0, // GPS doesn't typically provide altitude with same accuracy
        (true_position.z * 0.01).cos() * noise_std,
    ];
    true_position + noise
}

/// Simulate GPS velocity calculation using position difference
fn calculate_gps_velocity_simple(
    prev_pos: Vector3<f32>,
    curr_pos: Vector3<f32>,
    dt: f32,
) -> Vector3<f32> {
    (curr_pos - prev_pos) / dt
}

/// Simulate GPS velocity with weighted average filter
fn calculate_gps_velocity_weighted(
    velocities: &[Vector3<f32>],
) -> Vector3<f32> {
    if velocities.is_empty() {
        return Vector3::zeros();
    }

    let weights: Vec<f32> = (0..velocities.len())
        .map(|i| (i + 1) as f32)
        .collect();
    let total_weight: f32 = weights.iter().sum();

    let weighted_sum: Vector3<f32> = velocities
        .iter()
        .zip(weights.iter())
        .map(|(v, w)| v * w)
        .fold(Vector3::zeros(), |acc, v| acc + v);

    weighted_sum / total_weight
}

/// Simulate IMU update with accelerometer and gyroscope readings
fn simulate_imu_update(
    linear_accel: Vector3<f32>,
    angular_vel: Vector3<f32>,
    dt: f32,
) -> (Vector3<f32>, Vector3<f32>) {
    // Integrate accelerometer (with gravity compensation)
    let gravity = vector![0.0, -9.81, 0.0];
    let corrected_accel = linear_accel - gravity;

    // Simple integration (real IMU would use quaternions)
    let velocity_delta = corrected_accel * dt;
    let orientation_delta = angular_vel * dt;

    (velocity_delta, orientation_delta)
}

/// Simulate force/torque sensor reading
fn simulate_force_torque_update(
    contact_points: &[(Vector3<f32>, Vector3<f32>)], // (position, force)
    sensor_frame: Vector3<f32>,
) -> (Vector3<f32>, Vector3<f32>) {
    let mut total_force = Vector3::zeros();
    let mut total_torque = Vector3::zeros();

    for (pos, force) in contact_points {
        total_force += force;
        let moment_arm = pos - sensor_frame;
        total_torque += moment_arm.cross(force);
    }

    (total_force, total_torque)
}

fn benchmark_gps_update(c: &mut Criterion) {
    let positions: Vec<Vector3<f32>> = (0..1000)
        .map(|i| vector![i as f32 * 0.1, 0.0, (i as f32 * 0.1).sin() * 10.0])
        .collect();

    c.bench_function("gps_sensor_update", |b| {
        let mut idx = 0;
        b.iter(|| {
            let pos = positions[idx % positions.len()];
            let noisy_pos = simulate_gps_update(pos, 0.5);
            idx += 1;
            black_box(noisy_pos)
        });
    });
}

fn benchmark_gps_velocity_computation(c: &mut Criterion) {
    let mut group = c.benchmark_group("gps_velocity_methods");

    // Pre-generate position samples
    let positions: Vec<Vector3<f32>> = (0..100)
        .map(|i| vector![i as f32 * 0.1, 0.0, (i as f32 * 0.05).sin() * 5.0])
        .collect();

    let velocities: Vec<Vector3<f32>> = positions
        .windows(2)
        .map(|w| calculate_gps_velocity_simple(w[0], w[1], 0.1))
        .collect();

    group.bench_function("simple_diff", |b| {
        let mut idx = 0;
        b.iter(|| {
            let p1 = positions[idx % (positions.len() - 1)];
            let p2 = positions[(idx + 1) % positions.len()];
            idx += 1;
            black_box(calculate_gps_velocity_simple(p1, p2, 0.1))
        });
    });

    group.bench_function("weighted_avg", |b| {
        b.iter(|| {
            black_box(calculate_gps_velocity_weighted(&velocities[..10]))
        });
    });

    group.finish();
}

fn benchmark_imu_update(c: &mut Criterion) {
    let accel = vector![0.1, -9.7, 0.05]; // Slightly tilted
    let gyro = vector![0.01, 0.0, 0.02];
    let dt = 1.0 / 200.0; // 200 Hz IMU

    c.bench_function("imu_sensor_update", |b| {
        b.iter(|| {
            black_box(simulate_imu_update(accel, gyro, dt))
        });
    });
}

fn benchmark_force_torque_update(c: &mut Criterion) {
    // Simulate 6 contact points (like a 6-axis F/T sensor)
    let contact_points: Vec<(Vector3<f32>, Vector3<f32>)> = vec![
        (vector![0.01, 0.0, 0.0], vector![10.0, 0.0, 0.0]),
        (vector![-0.01, 0.0, 0.0], vector![-5.0, 0.0, 0.0]),
        (vector![0.0, 0.01, 0.0], vector![0.0, 15.0, 0.0]),
        (vector![0.0, -0.01, 0.0], vector![0.0, -10.0, 0.0]),
        (vector![0.0, 0.0, 0.01], vector![0.0, 0.0, 8.0]),
        (vector![0.0, 0.0, -0.01], vector![0.0, 0.0, -3.0]),
    ];
    let sensor_frame = vector![0.0, 0.0, 0.0];

    c.bench_function("force_torque_sensor_update", |b| {
        b.iter(|| {
            black_box(simulate_force_torque_update(&contact_points, sensor_frame))
        });
    });
}

fn benchmark_lidar_raycast(c: &mut Criterion) {
    let mut group = c.benchmark_group("lidar_ray_count");

    for ray_count in [100, 360, 720, 1080].iter() {
        group.bench_with_input(
            BenchmarkId::from_parameter(ray_count),
            ray_count,
            |b, &count| {
                let (rigid_body_set, collider_set, query_pipeline) = create_sensor_test_world();

                // LiDAR at 1m height, scanning horizontally
                let lidar_origin = point![0.0, 1.0, 0.0];

                // Pre-generate ray directions (horizontal scan)
                let rays: Vec<Ray> = (0..count)
                    .map(|i| {
                        let angle = (i as f32 / count as f32) * std::f32::consts::TAU;
                        let dir = vector![angle.cos(), 0.0, angle.sin()];
                        Ray::new(lidar_origin, dir)
                    })
                    .collect();

                b.iter(|| {
                    let mut distances = Vec::with_capacity(count);
                    for ray in &rays {
                        let hit = query_pipeline.cast_ray(
                            &rigid_body_set,
                            &collider_set,
                            ray,
                            30.0, // 30m max range
                            true,
                            QueryFilter::default(),
                        );
                        distances.push(hit.map(|(_, toi)| toi).unwrap_or(30.0));
                    }
                    black_box(distances)
                });
            },
        );
    }

    group.finish();
}

fn benchmark_lidar_3d_raycast(c: &mut Criterion) {
    let mut group = c.benchmark_group("lidar_3d");

    // Typical 3D LiDAR configurations (horizontal x vertical beams)
    for (h_beams, v_beams) in [(360, 16), (720, 32), (1080, 64)].iter() {
        let total = h_beams * v_beams;
        group.bench_with_input(
            BenchmarkId::from_parameter(format!("{}x{}", h_beams, v_beams)),
            &(*h_beams, *v_beams),
            |b, &(h, v)| {
                let (rigid_body_set, collider_set, query_pipeline) = create_sensor_test_world();

                let lidar_origin = point![0.0, 1.5, 0.0];

                // Pre-generate 3D ray directions
                let rays: Vec<Ray> = (0..h)
                    .flat_map(|hi| {
                        let h_angle = (hi as f32 / h as f32) * std::f32::consts::TAU;
                        (0..v).map(move |vi| {
                            // Vertical FOV: -15 to +15 degrees
                            let v_angle = -0.26 + (vi as f32 / v as f32) * 0.52;
                            let dir = vector![
                                h_angle.cos() * v_angle.cos(),
                                v_angle.sin(),
                                h_angle.sin() * v_angle.cos()
                            ];
                            Ray::new(lidar_origin, dir)
                        })
                    })
                    .collect();

                b.iter(|| {
                    let mut point_cloud = Vec::with_capacity(total);
                    for ray in &rays {
                        if let Some((_, toi)) = query_pipeline.cast_ray(
                            &rigid_body_set,
                            &collider_set,
                            ray,
                            100.0,
                            true,
                            QueryFilter::default(),
                        ) {
                            let point = ray.origin + ray.dir * toi;
                            point_cloud.push(point);
                        }
                    }
                    black_box(point_cloud)
                });
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_gps_update,
    benchmark_gps_velocity_computation,
    benchmark_imu_update,
    benchmark_force_torque_update,
    benchmark_lidar_raycast,
    benchmark_lidar_3d_raycast
);
criterion_main!(benches);
