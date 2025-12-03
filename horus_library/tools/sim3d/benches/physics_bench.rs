// Benchmark for Physics simulation performance
// Run with: cargo bench --bench physics_bench

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::{vector, Vector3};
use rapier3d::prelude::*;

/// Create a physics world for benchmarking
fn create_physics_world() -> (
    RigidBodySet,
    ColliderSet,
    PhysicsPipeline,
    IslandManager,
    DefaultBroadPhase,
    NarrowPhase,
    ImpulseJointSet,
    MultibodyJointSet,
    CCDSolver,
    IntegrationParameters,
    Vector3<f32>,
) {
    let rigid_body_set = RigidBodySet::new();
    let collider_set = ColliderSet::new();
    let physics_pipeline = PhysicsPipeline::new();
    let island_manager = IslandManager::new();
    let broad_phase = DefaultBroadPhase::new();
    let narrow_phase = NarrowPhase::new();
    let impulse_joint_set = ImpulseJointSet::new();
    let multibody_joint_set = MultibodyJointSet::new();
    let ccd_solver = CCDSolver::new();
    let integration_parameters = IntegrationParameters {
        dt: 1.0 / 240.0, // 240 Hz physics
        ..Default::default()
    };
    let gravity = vector![0.0, -9.81, 0.0];

    (
        rigid_body_set,
        collider_set,
        physics_pipeline,
        island_manager,
        broad_phase,
        narrow_phase,
        impulse_joint_set,
        multibody_joint_set,
        ccd_solver,
        integration_parameters,
        gravity,
    )
}

/// Add N rigid bodies to the physics world
fn populate_world(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    count: usize,
) {
    for i in 0..count {
        // Create a dynamic rigid body at random positions
        let x = (i % 10) as f32 * 2.0;
        let y = (i / 10) as f32 * 2.0 + 5.0;
        let z = ((i * 7) % 10) as f32 * 2.0;

        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y, z])
            .build();

        let handle = rigid_body_set.insert(rigid_body);

        // Add a box collider
        let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .restitution(0.3)
            .friction(0.5)
            .build();

        collider_set.insert_with_parent(collider, handle, rigid_body_set);
    }

    // Add a ground plane
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -1.0, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);

    let ground_collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    collider_set.insert_with_parent(ground_collider, ground_handle, rigid_body_set);
}

fn benchmark_physics_step(c: &mut Criterion) {
    let (
        mut rigid_body_set,
        mut collider_set,
        mut physics_pipeline,
        mut island_manager,
        mut broad_phase,
        mut narrow_phase,
        mut impulse_joint_set,
        mut multibody_joint_set,
        mut ccd_solver,
        integration_parameters,
        gravity,
    ) = create_physics_world();

    // Add 50 bodies for the benchmark
    populate_world(&mut rigid_body_set, &mut collider_set, 50);

    c.bench_function("physics_step_240hz_50_bodies", |b| {
        b.iter(|| {
            physics_pipeline.step(
                &gravity,
                &integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                None,
                &(),
                &(),
            );
            black_box(())
        });
    });
}

fn benchmark_collision_detection(c: &mut Criterion) {
    let (
        mut rigid_body_set,
        mut collider_set,
        _physics_pipeline,
        _island_manager,
        mut broad_phase,
        mut narrow_phase,
        _impulse_joint_set,
        _multibody_joint_set,
        _ccd_solver,
        _integration_parameters,
        _gravity,
    ) = create_physics_world();

    // Add bodies
    populate_world(&mut rigid_body_set, &mut collider_set, 100);

    let mut query_pipeline = QueryPipeline::new();
    query_pipeline.update(&collider_set);

    c.bench_function("collision_detection_100_bodies", |b| {
        b.iter(|| {
            // Update broad phase
            broad_phase.update(&rigid_body_set, &collider_set);

            // Update narrow phase
            narrow_phase.find_intersections(&broad_phase, &collider_set, &mut |_| {});

            black_box(())
        });
    });
}

fn benchmark_rigid_body_count(c: &mut Criterion) {
    let mut group = c.benchmark_group("physics_body_scaling");

    for body_count in [10, 50, 100, 200].iter() {
        group.bench_with_input(
            BenchmarkId::from_parameter(body_count),
            body_count,
            |b, &count| {
                let (
                    mut rigid_body_set,
                    mut collider_set,
                    mut physics_pipeline,
                    mut island_manager,
                    mut broad_phase,
                    mut narrow_phase,
                    mut impulse_joint_set,
                    mut multibody_joint_set,
                    mut ccd_solver,
                    integration_parameters,
                    gravity,
                ) = create_physics_world();

                populate_world(&mut rigid_body_set, &mut collider_set, count);

                b.iter(|| {
                    physics_pipeline.step(
                        &gravity,
                        &integration_parameters,
                        &mut island_manager,
                        &mut broad_phase,
                        &mut narrow_phase,
                        &mut rigid_body_set,
                        &mut collider_set,
                        &mut impulse_joint_set,
                        &mut multibody_joint_set,
                        &mut ccd_solver,
                        None,
                        &(),
                        &(),
                    );
                    black_box(())
                });
            },
        );
    }

    group.finish();
}

fn benchmark_force_application(c: &mut Criterion) {
    let (mut rigid_body_set, mut collider_set, ..) = create_physics_world();

    populate_world(&mut rigid_body_set, &mut collider_set, 100);

    // Get handles to apply forces to
    let handles: Vec<_> = rigid_body_set
        .iter()
        .filter(|(_, rb)| rb.is_dynamic())
        .map(|(h, _)| h)
        .collect();

    c.bench_function("force_application_100_bodies", |b| {
        b.iter(|| {
            for handle in &handles {
                if let Some(rb) = rigid_body_set.get_mut(*handle) {
                    rb.apply_force(vector![0.0, 10.0, 0.0], true);
                    rb.apply_torque(vector![0.1, 0.0, 0.0], true);
                }
            }
            black_box(())
        });
    });
}

fn benchmark_raycast(c: &mut Criterion) {
    let (mut rigid_body_set, mut collider_set, ..) = create_physics_world();

    populate_world(&mut rigid_body_set, &mut collider_set, 100);

    let mut query_pipeline = QueryPipeline::new();
    query_pipeline.update(&collider_set);

    c.bench_function("raycast_single", |b| {
        b.iter(|| {
            let ray = Ray::new(point![0.0, 10.0, 0.0], vector![0.0, -1.0, 0.0]);
            let max_toi = 100.0;
            let solid = true;
            let filter = QueryFilter::default();

            let hit = query_pipeline.cast_ray(&rigid_body_set, &collider_set, &ray, max_toi, solid, filter);
            black_box(hit)
        });
    });
}

fn benchmark_raycast_batch(c: &mut Criterion) {
    let mut group = c.benchmark_group("raycast_batch");

    for ray_count in [10, 100, 500, 1000].iter() {
        group.bench_with_input(
            BenchmarkId::from_parameter(ray_count),
            ray_count,
            |b, &count| {
                let (mut rigid_body_set, mut collider_set, ..) = create_physics_world();
                populate_world(&mut rigid_body_set, &mut collider_set, 100);

                let mut query_pipeline = QueryPipeline::new();
                query_pipeline.update(&collider_set);

                // Pre-generate rays
                let rays: Vec<_> = (0..count)
                    .map(|i| {
                        let angle = (i as f32 / count as f32) * std::f32::consts::TAU;
                        let dir = vector![angle.cos(), -0.5, angle.sin()].normalize();
                        Ray::new(point![0.0, 10.0, 0.0], dir)
                    })
                    .collect();

                b.iter(|| {
                    let mut hits = 0;
                    for ray in &rays {
                        if query_pipeline
                            .cast_ray(&rigid_body_set, &collider_set, ray, 100.0, true, QueryFilter::default())
                            .is_some()
                        {
                            hits += 1;
                        }
                    }
                    black_box(hits)
                });
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_physics_step,
    benchmark_collision_detection,
    benchmark_rigid_body_count,
    benchmark_force_application,
    benchmark_raycast,
    benchmark_raycast_batch
);
criterion_main!(benches);
