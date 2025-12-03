// Benchmark for IK Solver performance
// Run with: cargo bench --bench ik_solver_bench

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

/// Joint configuration for a kinematic chain
#[derive(Clone, Debug)]
struct JointConfig {
    /// Current joint angles (radians)
    angles: Vec<f32>,
    /// Joint axis (unit vector in local frame)
    axes: Vec<Vector3<f32>>,
    /// Link lengths
    link_lengths: Vec<f32>,
    /// Joint limits (min, max) in radians
    limits: Vec<(f32, f32)>,
}

impl JointConfig {
    fn new(dof: usize) -> Self {
        let angles = vec![0.0; dof];
        let axes: Vec<_> = (0..dof)
            .map(|i| {
                if i % 2 == 0 {
                    Vector3::z() // Yaw/roll joints
                } else {
                    Vector3::y() // Pitch joints
                }
            })
            .collect();
        let link_lengths = vec![0.2; dof]; // 20cm links
        let limits = vec![(-std::f32::consts::PI, std::f32::consts::PI); dof];

        Self {
            angles,
            axes,
            link_lengths,
            limits,
        }
    }
}

/// Forward kinematics: compute end-effector position from joint angles
fn forward_kinematics(config: &JointConfig) -> Vector3<f32> {
    let mut position = Vector3::zeros();
    let mut orientation = UnitQuaternion::identity();

    for i in 0..config.angles.len() {
        // Rotate by joint angle around joint axis
        let rotation = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(config.axes[i]),
            config.angles[i],
        );
        orientation = orientation * rotation;

        // Translate along the link (in local frame, then transformed to world)
        let link_offset = orientation * Vector3::new(config.link_lengths[i], 0.0, 0.0);
        position += link_offset;
    }

    position
}

/// Compute Jacobian matrix numerically
fn compute_jacobian(config: &JointConfig) -> nalgebra::DMatrix<f32> {
    let dof = config.angles.len();
    let epsilon = 1e-6;

    let mut jacobian = nalgebra::DMatrix::zeros(3, dof);

    // Current end-effector position
    let current_pos = forward_kinematics(config);

    for i in 0..dof {
        // Perturb joint i
        let mut perturbed = config.clone();
        perturbed.angles[i] += epsilon;

        let perturbed_pos = forward_kinematics(&perturbed);

        // Partial derivative
        let col = (perturbed_pos - current_pos) / epsilon;
        jacobian.set_column(i, &col);
    }

    jacobian
}

/// Inverse kinematics using Jacobian pseudo-inverse (Damped Least Squares)
fn solve_ik_dls(
    config: &mut JointConfig,
    target: Vector3<f32>,
    max_iterations: usize,
    tolerance: f32,
    damping: f32,
) -> bool {
    for _ in 0..max_iterations {
        let current_pos = forward_kinematics(config);
        let error = target - current_pos;

        // Check if we've reached the target
        if error.norm() < tolerance {
            return true;
        }

        // Compute Jacobian
        let jacobian = compute_jacobian(config);

        // Damped least squares: dq = J^T * (J * J^T + λ^2 * I)^-1 * e
        let jjt = &jacobian * jacobian.transpose();
        let damped = jjt + nalgebra::DMatrix::identity(3, 3) * (damping * damping);

        if let Some(inv) = damped.try_inverse() {
            let delta_theta = jacobian.transpose() * inv * error;

            // Apply joint angle update with limits
            for i in 0..config.angles.len() {
                config.angles[i] += delta_theta[i] * 0.5; // Step factor
                config.angles[i] = config.angles[i].clamp(config.limits[i].0, config.limits[i].1);
            }
        } else {
            return false; // Singular matrix
        }
    }

    false
}

/// Analytical 2-DOF IK for planar arm
fn solve_ik_2dof_analytical(
    l1: f32,
    l2: f32,
    target_x: f32,
    target_y: f32,
) -> Option<(f32, f32)> {
    let dist_sq = target_x * target_x + target_y * target_y;
    let dist = dist_sq.sqrt();

    // Check reachability
    if dist > l1 + l2 || dist < (l1 - l2).abs() {
        return None;
    }

    // Law of cosines for elbow angle
    let cos_angle2 = (dist_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    let angle2 = cos_angle2.clamp(-1.0, 1.0).acos();

    // Shoulder angle
    let k1 = l1 + l2 * cos_angle2;
    let k2 = l2 * angle2.sin();
    let angle1 = target_y.atan2(target_x) - k2.atan2(k1);

    Some((angle1, angle2))
}

/// Cyclic Coordinate Descent (CCD) IK
fn solve_ik_ccd(
    config: &mut JointConfig,
    target: Vector3<f32>,
    max_iterations: usize,
    tolerance: f32,
) -> bool {
    for _ in 0..max_iterations {
        let end_pos = forward_kinematics(config);

        if (end_pos - target).norm() < tolerance {
            return true;
        }

        // Iterate through joints from end to base
        for i in (0..config.angles.len()).rev() {
            // Compute joint position
            let mut joint_config = config.clone();
            joint_config.angles.truncate(i);
            joint_config.axes.truncate(i);
            joint_config.link_lengths.truncate(i);
            let joint_pos = forward_kinematics(&joint_config);

            // Vectors from joint to current end-effector and target
            let to_end = end_pos - joint_pos;
            let to_target = target - joint_pos;

            // Project onto rotation plane (perpendicular to joint axis)
            let axis = config.axes[i];

            // Calculate rotation angle
            let to_end_proj = to_end - axis * to_end.dot(&axis);
            let to_target_proj = to_target - axis * to_target.dot(&axis);

            if to_end_proj.norm() > 1e-6 && to_target_proj.norm() > 1e-6 {
                let angle = to_end_proj
                    .normalize()
                    .dot(&to_target_proj.normalize())
                    .clamp(-1.0, 1.0)
                    .acos();

                // Determine rotation direction
                let cross = to_end_proj.cross(&to_target_proj);
                let sign = if cross.dot(&axis) >= 0.0 { 1.0 } else { -1.0 };

                config.angles[i] += sign * angle * 0.5;
                config.angles[i] = config.angles[i].clamp(config.limits[i].0, config.limits[i].1);
            }
        }
    }

    (forward_kinematics(config) - target).norm() < tolerance
}

fn benchmark_ik_2dof(c: &mut Criterion) {
    c.bench_function("ik_solver_2dof_analytical", |b| {
        b.iter(|| {
            let target_x = 0.3;
            let target_y = 0.2;
            black_box(solve_ik_2dof_analytical(0.2, 0.2, target_x, target_y))
        });
    });
}

fn benchmark_ik_6dof(c: &mut Criterion) {
    c.bench_function("ik_solver_6dof_jacobian", |b| {
        let mut config = JointConfig::new(6);
        let target = Vector3::new(0.5, 0.3, 0.2);

        b.iter(|| {
            // Reset config
            config.angles = vec![0.0; 6];
            black_box(solve_ik_dls(&mut config, target, 50, 0.001, 0.1))
        });
    });
}

fn benchmark_ik_ccd(c: &mut Criterion) {
    c.bench_function("ik_solver_6dof_ccd", |b| {
        let mut config = JointConfig::new(6);
        let target = Vector3::new(0.5, 0.3, 0.2);

        b.iter(|| {
            config.angles = vec![0.0; 6];
            black_box(solve_ik_ccd(&mut config, target, 50, 0.001))
        });
    });
}

fn benchmark_ik_varying_dof(c: &mut Criterion) {
    let mut group = c.benchmark_group("ik_solver_dof_comparison");

    for dof in [2, 3, 4, 6, 7].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(dof), dof, |b, &dof| {
            let mut config = JointConfig::new(dof);

            // Target that's reachable for all DOF configurations
            let max_reach = config.link_lengths.iter().sum::<f32>() * 0.6;
            let target = Vector3::new(max_reach * 0.5, max_reach * 0.3, max_reach * 0.2);

            b.iter(|| {
                config.angles = vec![0.0; dof];
                black_box(solve_ik_dls(&mut config, target, 100, 0.001, 0.1))
            });
        });
    }

    group.finish();
}

fn benchmark_forward_kinematics(c: &mut Criterion) {
    let mut group = c.benchmark_group("forward_kinematics");

    for dof in [3, 6, 7, 10].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(dof), dof, |b, &dof| {
            let mut config = JointConfig::new(dof);
            // Random-ish joint angles
            config.angles = (0..dof).map(|i| (i as f32 * 0.5).sin() * 0.5).collect();

            b.iter(|| {
                black_box(forward_kinematics(&config))
            });
        });
    }

    group.finish();
}

fn benchmark_jacobian_computation(c: &mut Criterion) {
    let mut group = c.benchmark_group("jacobian_computation");

    for dof in [3, 6, 7].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(dof), dof, |b, &dof| {
            let mut config = JointConfig::new(dof);
            config.angles = (0..dof).map(|i| (i as f32 * 0.3).sin() * 0.3).collect();

            b.iter(|| {
                black_box(compute_jacobian(&config))
            });
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_ik_2dof,
    benchmark_ik_6dof,
    benchmark_ik_ccd,
    benchmark_ik_varying_dof,
    benchmark_forward_kinematics,
    benchmark_jacobian_computation
);
criterion_main!(benches);
