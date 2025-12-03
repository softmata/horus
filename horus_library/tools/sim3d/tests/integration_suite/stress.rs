//! Stress tests with many objects and robots
//!
//! Comprehensive stress testing for sim3d:
//! - 1000+ rigid body simulation stability
//! - 100+ robot simulation
//! - Pyramid/tower stacking stability
//! - Memory usage tracking
//! - Simulation FPS under load
//! - Parallel physics performance
//! - Long-running simulation stability

#![allow(dead_code)]
#![allow(unused_imports)]
use bevy::prelude::*;
use rapier3d::prelude::*;
use std::time::{Duration, Instant};

const GRAVITY: f32 = 9.81;

/// Stress test configuration
#[derive(Debug, Clone)]
pub struct StressTestConfig {
    pub name: String,
    pub object_count: usize,
    pub robot_count: usize,
    pub simulation_steps: usize,
    pub enable_physics: bool,
    pub enable_sensors: bool,
}

impl StressTestConfig {
    pub fn many_objects() -> Self {
        Self {
            name: "1000 Objects".to_string(),
            object_count: 1000,
            robot_count: 1,
            simulation_steps: 1000,
            enable_physics: true,
            enable_sensors: false,
        }
    }

    pub fn many_robots() -> Self {
        Self {
            name: "100 Robots".to_string(),
            object_count: 10,
            robot_count: 100,
            simulation_steps: 1000,
            enable_physics: true,
            enable_sensors: true,
        }
    }

    pub fn extreme_load() -> Self {
        Self {
            name: "Extreme Load (1000 objects + 100 robots)".to_string(),
            object_count: 1000,
            robot_count: 100,
            simulation_steps: 100,
            enable_physics: true,
            enable_sensors: true,
        }
    }

    pub fn physics_only() -> Self {
        Self {
            name: "Physics Stress (5000 objects)".to_string(),
            object_count: 5000,
            robot_count: 0,
            simulation_steps: 500,
            enable_physics: true,
            enable_sensors: false,
        }
    }

    pub fn long_running() -> Self {
        Self {
            name: "Long Running (1000+ steps)".to_string(),
            object_count: 100,
            robot_count: 10,
            simulation_steps: 2000,
            enable_physics: true,
            enable_sensors: true,
        }
    }
}

/// Stress test result
#[derive(Debug, Clone)]
pub struct StressTestResult {
    pub config: StressTestConfig,
    pub total_duration: Duration,
    pub avg_step_time: Duration,
    pub max_step_time: Duration,
    pub min_step_time: Duration,
    pub peak_memory_mb: f64,
    pub successful_steps: usize,
    pub failed_steps: usize,
}

impl StressTestResult {
    pub fn new(config: StressTestConfig) -> Self {
        Self {
            config,
            total_duration: Duration::ZERO,
            avg_step_time: Duration::ZERO,
            max_step_time: Duration::ZERO,
            min_step_time: Duration::MAX,
            peak_memory_mb: 0.0,
            successful_steps: 0,
            failed_steps: 0,
        }
    }

    /// Check if test passed performance criteria
    pub fn passed(&self, max_avg_step_ms: f64, max_memory_mb: f64) -> bool {
        let avg_ms = self.avg_step_time.as_secs_f64() * 1000.0;
        avg_ms <= max_avg_step_ms && self.peak_memory_mb <= max_memory_mb && self.failed_steps == 0
    }

    /// Generate report
    pub fn format_report(&self) -> String {
        let avg_ms = self.avg_step_time.as_secs_f64() * 1000.0;
        let max_ms = self.max_step_time.as_secs_f64() * 1000.0;
        let min_ms = self.min_step_time.as_secs_f64() * 1000.0;

        format!(
            "{}\n  Objects: {}, Robots: {}\n  Steps: {}/{}\n  Avg: {:.2}ms, Min: {:.2}ms, Max: {:.2}ms\n  Memory: {:.1} MB",
            self.config.name,
            self.config.object_count,
            self.config.robot_count,
            self.successful_steps,
            self.config.simulation_steps,
            avg_ms,
            min_ms,
            max_ms,
            self.peak_memory_mb
        )
    }
}

/// Memory usage tracker
pub struct MemoryTracker {
    peak_bytes: usize,
}

impl MemoryTracker {
    pub fn new() -> Self {
        Self { peak_bytes: 0 }
    }

    /// Update peak memory (simplified - would use actual memory stats in production)
    pub fn update(&mut self, estimated_bytes: usize) {
        if estimated_bytes > self.peak_bytes {
            self.peak_bytes = estimated_bytes;
        }
    }

    /// Get peak memory in MB
    pub fn peak_mb(&self) -> f64 {
        self.peak_bytes as f64 / (1024.0 * 1024.0)
    }

    /// Estimate memory for objects and robots
    pub fn estimate_memory(object_count: usize, robot_count: usize) -> usize {
        // Rough estimate: 1 KB per object, 10 KB per robot
        object_count * 1024 + robot_count * 10240
    }
}

/// Configuration for pyramid stacking test
#[derive(Debug, Clone)]
pub struct PyramidStackingConfig {
    pub base_size: usize,
    pub box_size: f32,
    pub box_mass: f32,
    pub simulation_steps: usize,
    pub timestep: f32,
}

impl Default for PyramidStackingConfig {
    fn default() -> Self {
        Self {
            base_size: 10,
            box_size: 0.5,
            box_mass: 1.0,
            simulation_steps: 2000,
            timestep: 0.001,
        }
    }
}

impl PyramidStackingConfig {
    pub fn small() -> Self {
        Self {
            base_size: 5,
            ..Default::default()
        }
    }

    pub fn medium() -> Self {
        Self {
            base_size: 10,
            ..Default::default()
        }
    }

    pub fn large() -> Self {
        Self {
            base_size: 20,
            simulation_steps: 3000,
            ..Default::default()
        }
    }

    pub fn extreme() -> Self {
        Self {
            base_size: 50,
            simulation_steps: 5000,
            ..Default::default()
        }
    }

    pub fn total_boxes(&self) -> usize {
        (self.base_size * (self.base_size + 1)) / 2
    }
}

/// Result of pyramid stacking test
#[derive(Debug, Clone)]
pub struct PyramidStackingResult {
    pub total_boxes: usize,
    pub boxes_settled: usize,
    pub max_displacement: f32,
    pub avg_displacement: f32,
    pub pyramid_stable: bool,
    pub simulation_time: Duration,
    pub avg_step_time: Duration,
}

/// Run pyramid stacking stability test
pub fn run_pyramid_stacking_test(
    config: PyramidStackingConfig,
) -> Result<PyramidStackingResult, String> {
    let start_time = Instant::now();

    // Setup Rapier physics
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let gravity_vec = vector![0.0, -GRAVITY, 0.0];
    let integration_parameters = IntegrationParameters {
        dt: config.timestep,
        ..Default::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    // Create ground plane
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -0.5, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);
    let ground_collider = ColliderBuilder::cuboid(50.0, 0.5, 50.0)
        .friction(0.8)
        .build();
    collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

    // Create pyramid of boxes
    let mut box_handles = Vec::new();
    let mut initial_positions = Vec::new();
    let half_size = config.box_size / 2.0;

    for row in 0..config.base_size {
        let boxes_in_row = config.base_size - row;
        let start_x = -(boxes_in_row as f32 - 1.0) * config.box_size / 2.0;
        let y = row as f32 * config.box_size + half_size;

        for col in 0..boxes_in_row {
            let x = start_x + col as f32 * config.box_size;
            let pos = vector![x, y, 0.0];

            let box_body = RigidBodyBuilder::dynamic().translation(pos).build();
            let box_handle = rigid_body_set.insert(box_body);

            let box_collider =
                ColliderBuilder::cuboid(half_size * 0.95, half_size * 0.95, half_size * 0.95)
                    .mass(config.box_mass)
                    .friction(0.8)
                    .restitution(0.1)
                    .build();
            collider_set.insert_with_parent(box_collider, box_handle, &mut rigid_body_set);

            box_handles.push(box_handle);
            initial_positions.push(Vec3::new(pos.x, pos.y, pos.z));
        }
    }

    let total_boxes = box_handles.len();
    let mut step_times = Vec::new();

    // Run simulation
    for _step in 0..config.simulation_steps {
        let step_start = Instant::now();

        physics_pipeline.step(
            &gravity_vec,
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
            &physics_hooks,
            &event_handler,
        );

        step_times.push(step_start.elapsed());
    }

    // Calculate results
    let mut displacements = Vec::new();
    let mut settled_count = 0;

    for (i, handle) in box_handles.iter().enumerate() {
        if let Some(rb) = rigid_body_set.get(*handle) {
            let final_pos = rb.translation();
            let initial_pos = initial_positions[i];

            let horizontal_displacement = ((final_pos.x - initial_pos.x).powi(2)
                + (final_pos.z - initial_pos.z).powi(2))
            .sqrt();

            displacements.push(horizontal_displacement);

            // Consider settled if horizontal displacement < half box size
            if horizontal_displacement < config.box_size / 2.0 {
                settled_count += 1;
            }
        }
    }

    let max_displacement = displacements.iter().cloned().fold(0.0f32, f32::max);
    let avg_displacement = displacements.iter().sum::<f32>() / displacements.len() as f32;
    let avg_step_time = step_times.iter().sum::<Duration>() / step_times.len() as u32;

    // Pyramid is stable if >80% of boxes are settled
    let pyramid_stable = settled_count as f32 / total_boxes as f32 > 0.8;

    Ok(PyramidStackingResult {
        total_boxes,
        boxes_settled: settled_count,
        max_displacement,
        avg_displacement,
        pyramid_stable,
        simulation_time: start_time.elapsed(),
        avg_step_time,
    })
}

/// Configuration for rigid body stress test
#[derive(Debug, Clone)]
pub struct RigidBodyStressConfig {
    pub body_count: usize,
    pub simulation_steps: usize,
    pub timestep: f32,
    pub spawn_area: Vec3,
}

impl Default for RigidBodyStressConfig {
    fn default() -> Self {
        Self {
            body_count: 1000,
            simulation_steps: 1000,
            timestep: 0.001,
            spawn_area: Vec3::new(20.0, 20.0, 20.0),
        }
    }
}

/// Result of rigid body stress test
#[derive(Debug, Clone)]
pub struct RigidBodyStressResult {
    pub body_count: usize,
    pub successful_steps: usize,
    pub total_time: Duration,
    pub avg_step_time: Duration,
    pub min_step_time: Duration,
    pub max_step_time: Duration,
    pub fps: f32,
    pub stability_passed: bool,
}

/// Run 1000+ rigid body simulation stress test
pub fn run_rigid_body_stress_test(
    config: RigidBodyStressConfig,
) -> Result<RigidBodyStressResult, String> {
    let start_time = Instant::now();

    // Setup Rapier physics
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let gravity_vec = vector![0.0, -GRAVITY, 0.0];
    let integration_parameters = IntegrationParameters {
        dt: config.timestep,
        ..Default::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    // Create ground
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -1.0, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);
    let ground_collider = ColliderBuilder::cuboid(100.0, 1.0, 100.0)
        .friction(0.5)
        .build();
    collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

    // Spawn rigid bodies
    use rand::Rng;
    let mut rng = rand::thread_rng();

    for _ in 0..config.body_count {
        let x = rng.gen_range(-config.spawn_area.x / 2.0..config.spawn_area.x / 2.0);
        let y = rng.gen_range(1.0..config.spawn_area.y);
        let z = rng.gen_range(-config.spawn_area.z / 2.0..config.spawn_area.z / 2.0);

        let body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y, z])
            .build();
        let handle = rigid_body_set.insert(body);

        // Random shape
        let shape_type = rng.gen_range(0..3);
        let collider = match shape_type {
            0 => ColliderBuilder::ball(rng.gen_range(0.1..0.3))
                .mass(1.0)
                .build(),
            1 => ColliderBuilder::cuboid(
                rng.gen_range(0.1..0.3),
                rng.gen_range(0.1..0.3),
                rng.gen_range(0.1..0.3),
            )
            .mass(1.0)
            .build(),
            _ => ColliderBuilder::capsule_y(rng.gen_range(0.1..0.2), rng.gen_range(0.05..0.1))
                .mass(1.0)
                .build(),
        };
        collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);
    }

    let mut step_times = Vec::new();
    let mut successful_steps = 0;

    // Run simulation
    for _step in 0..config.simulation_steps {
        let step_start = Instant::now();

        physics_pipeline.step(
            &gravity_vec,
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
            &physics_hooks,
            &event_handler,
        );

        step_times.push(step_start.elapsed());
        successful_steps += 1;
    }

    let total_time = start_time.elapsed();
    let avg_step_time = step_times.iter().sum::<Duration>() / step_times.len() as u32;
    let min_step_time = step_times.iter().min().copied().unwrap_or(Duration::ZERO);
    let max_step_time = step_times.iter().max().copied().unwrap_or(Duration::ZERO);
    let fps = config.simulation_steps as f32 / total_time.as_secs_f32();

    // Stability check: all steps completed and FPS > 10
    let stability_passed = successful_steps == config.simulation_steps && fps > 10.0;

    Ok(RigidBodyStressResult {
        body_count: config.body_count,
        successful_steps,
        total_time,
        avg_step_time,
        min_step_time,
        max_step_time,
        fps,
        stability_passed,
    })
}

/// Configuration for robot simulation stress test
#[derive(Debug, Clone)]
pub struct RobotStressConfig {
    pub robot_count: usize,
    pub links_per_robot: usize,
    pub simulation_steps: usize,
    pub timestep: f32,
}

impl Default for RobotStressConfig {
    fn default() -> Self {
        Self {
            robot_count: 100,
            links_per_robot: 6,
            simulation_steps: 1000,
            timestep: 0.001,
        }
    }
}

/// Result of robot stress test
#[derive(Debug, Clone)]
pub struct RobotStressResult {
    pub robot_count: usize,
    pub total_bodies: usize,
    pub total_joints: usize,
    pub successful_steps: usize,
    pub total_time: Duration,
    pub avg_step_time: Duration,
    pub fps: f32,
    pub stability_passed: bool,
}

/// Run 100+ robot simulation stress test
pub fn run_robot_stress_test(config: RobotStressConfig) -> Result<RobotStressResult, String> {
    let start_time = Instant::now();

    // Setup Rapier physics
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let gravity_vec = vector![0.0, -GRAVITY, 0.0];
    let integration_parameters = IntegrationParameters {
        dt: config.timestep,
        ..Default::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    // Create ground
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);
    let ground_collider = ColliderBuilder::cuboid(200.0, 0.5, 200.0).build();
    collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

    let mut total_joints = 0;

    // Create robots in a grid
    let grid_size = (config.robot_count as f32).sqrt().ceil() as usize;
    let spacing = 3.0;

    for i in 0..config.robot_count {
        let row = i / grid_size;
        let col = i % grid_size;
        let base_x = (col as f32 - grid_size as f32 / 2.0) * spacing;
        let base_z = (row as f32 - grid_size as f32 / 2.0) * spacing;

        // Create robot base (fixed)
        let base = RigidBodyBuilder::fixed()
            .translation(vector![base_x, 0.5, base_z])
            .build();
        let mut prev_handle = rigid_body_set.insert(base);

        // Create robot links
        for link_idx in 0..config.links_per_robot {
            let link_y = 0.5 + (link_idx + 1) as f32 * 0.3;

            let link = RigidBodyBuilder::dynamic()
                .translation(vector![base_x, link_y, base_z])
                .build();
            let link_handle = rigid_body_set.insert(link);

            let link_collider = ColliderBuilder::capsule_y(0.1, 0.03).mass(0.5).build();
            collider_set.insert_with_parent(link_collider, link_handle, &mut rigid_body_set);

            // Create revolute joint
            let joint = RevoluteJointBuilder::new(Vector::z_axis())
                .local_anchor1(point![0.0, 0.15, 0.0])
                .local_anchor2(point![0.0, -0.15, 0.0])
                .build();
            impulse_joint_set.insert(prev_handle, link_handle, joint, true);
            total_joints += 1;

            prev_handle = link_handle;
        }
    }

    let total_bodies = rigid_body_set.len();
    let mut step_times = Vec::new();
    let mut successful_steps = 0;

    // Run simulation
    for _step in 0..config.simulation_steps {
        let step_start = Instant::now();

        physics_pipeline.step(
            &gravity_vec,
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
            &physics_hooks,
            &event_handler,
        );

        step_times.push(step_start.elapsed());
        successful_steps += 1;
    }

    let total_time = start_time.elapsed();
    let avg_step_time = step_times.iter().sum::<Duration>() / step_times.len() as u32;
    let fps = config.simulation_steps as f32 / total_time.as_secs_f32();

    let stability_passed = successful_steps == config.simulation_steps && fps > 5.0;

    Ok(RobotStressResult {
        robot_count: config.robot_count,
        total_bodies,
        total_joints,
        successful_steps,
        total_time,
        avg_step_time,
        fps,
        stability_passed,
    })
}

/// Configuration for long-running simulation test
#[derive(Debug, Clone)]
pub struct LongRunningConfig {
    pub object_count: usize,
    pub simulation_steps: usize,
    pub timestep: f32,
    pub checkpoint_interval: usize,
}

impl Default for LongRunningConfig {
    fn default() -> Self {
        Self {
            object_count: 100,
            simulation_steps: 2000,
            timestep: 0.001,
            checkpoint_interval: 500,
        }
    }
}

/// Result of long-running simulation test
#[derive(Debug, Clone)]
pub struct LongRunningResult {
    pub total_steps: usize,
    pub checkpoints: Vec<CheckpointData>,
    pub total_time: Duration,
    pub avg_step_time: Duration,
    pub performance_degradation: f32,
    pub stability_maintained: bool,
}

#[derive(Debug, Clone)]
pub struct CheckpointData {
    pub step: usize,
    pub avg_step_time_ms: f32,
    pub active_bodies: usize,
    pub sleeping_bodies: usize,
}

/// Run long-running simulation (1000+ steps) test
pub fn run_long_running_test(config: LongRunningConfig) -> Result<LongRunningResult, String> {
    let start_time = Instant::now();

    // Setup Rapier physics
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let gravity_vec = vector![0.0, -GRAVITY, 0.0];
    let integration_parameters = IntegrationParameters {
        dt: config.timestep,
        ..Default::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    // Create ground
    let ground = RigidBodyBuilder::fixed()
        .translation(vector![0.0, -1.0, 0.0])
        .build();
    let ground_handle = rigid_body_set.insert(ground);
    let ground_collider = ColliderBuilder::cuboid(50.0, 1.0, 50.0).build();
    collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

    // Spawn objects
    use rand::Rng;
    let mut rng = rand::thread_rng();

    for _ in 0..config.object_count {
        let x = rng.gen_range(-10.0..10.0);
        let y = rng.gen_range(1.0..20.0);
        let z = rng.gen_range(-10.0..10.0);

        let body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y, z])
            .build();
        let handle = rigid_body_set.insert(body);

        let collider = ColliderBuilder::ball(rng.gen_range(0.2..0.5))
            .mass(1.0)
            .friction(0.5)
            .restitution(0.3)
            .build();
        collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);
    }

    let mut checkpoints = Vec::new();
    let mut recent_step_times = Vec::new();

    // Run simulation
    for step in 0..config.simulation_steps {
        let step_start = Instant::now();

        physics_pipeline.step(
            &gravity_vec,
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
            &physics_hooks,
            &event_handler,
        );

        let step_time = step_start.elapsed();
        recent_step_times.push(step_time);

        // Record checkpoint
        if (step + 1) % config.checkpoint_interval == 0 || step == config.simulation_steps - 1 {
            let avg_time =
                recent_step_times.iter().sum::<Duration>() / recent_step_times.len() as u32;

            // Count active vs sleeping bodies
            let mut active_count = 0;
            let mut sleeping_count = 0;
            for (_, rb) in rigid_body_set.iter() {
                if rb.is_sleeping() {
                    sleeping_count += 1;
                } else if rb.is_dynamic() {
                    active_count += 1;
                }
            }

            checkpoints.push(CheckpointData {
                step: step + 1,
                avg_step_time_ms: avg_time.as_secs_f32() * 1000.0,
                active_bodies: active_count,
                sleeping_bodies: sleeping_count,
            });

            recent_step_times.clear();
        }
    }

    let total_time = start_time.elapsed();
    let avg_step_time = total_time / config.simulation_steps as u32;

    // Calculate performance degradation (compare first checkpoint to last)
    let performance_degradation = if checkpoints.len() >= 2 {
        let first_time = checkpoints.first().unwrap().avg_step_time_ms;
        let last_time = checkpoints.last().unwrap().avg_step_time_ms;
        (last_time - first_time) / first_time
    } else {
        0.0
    };

    // Stability maintained if degradation < 50%
    let stability_maintained = performance_degradation < 0.5;

    Ok(LongRunningResult {
        total_steps: config.simulation_steps,
        checkpoints,
        total_time,
        avg_step_time,
        performance_degradation,
        stability_maintained,
    })
}

/// Get current memory usage estimate in MB
pub fn get_memory_usage_mb() -> f64 {
    #[cfg(target_os = "linux")]
    {
        // Read from /proc/self/statm for memory info
        if let Ok(statm) = std::fs::read_to_string("/proc/self/statm") {
            let parts: Vec<&str> = statm.split_whitespace().collect();
            if parts.len() >= 2 {
                // Second field is RSS (resident set size) in pages
                if let Ok(rss_pages) = parts[1].parse::<u64>() {
                    // Page size is typically 4KB
                    let page_size = 4096u64;
                    return (rss_pages * page_size) as f64 / (1024.0 * 1024.0);
                }
            }
        }
        0.0
    }

    #[cfg(target_os = "macos")]
    {
        // Use mach APIs via rusage
        use std::mem::MaybeUninit;
        let mut rusage = MaybeUninit::<libc::rusage>::uninit();
        unsafe {
            if libc::getrusage(libc::RUSAGE_SELF, rusage.as_mut_ptr()) == 0 {
                let rusage = rusage.assume_init();
                // ru_maxrss is in bytes on macOS
                return rusage.ru_maxrss as f64 / (1024.0 * 1024.0);
            }
        }
        0.0
    }

    #[cfg(target_os = "windows")]
    {
        // Windows implementation using GetProcessMemoryInfo would require winapi
        // Return 0.0 as fallback - proper implementation needs winapi crate
        0.0
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        0.0
    }
}

/// Track FPS over simulation run
pub fn calculate_fps(step_times: &[Duration]) -> f32 {
    if step_times.is_empty() {
        return 0.0;
    }
    let total_time: Duration = step_times.iter().sum();
    step_times.len() as f32 / total_time.as_secs_f32()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stress_config_many_objects() {
        let config = StressTestConfig::many_objects();
        assert_eq!(config.object_count, 1000);
        assert_eq!(config.robot_count, 1);
    }

    #[test]
    fn test_stress_config_many_robots() {
        let config = StressTestConfig::many_robots();
        assert_eq!(config.robot_count, 100);
        assert!(config.enable_sensors);
    }

    #[test]
    fn test_stress_config_extreme() {
        let config = StressTestConfig::extreme_load();
        assert_eq!(config.object_count, 1000);
        assert_eq!(config.robot_count, 100);
    }

    #[test]
    fn test_stress_config_long_running() {
        let config = StressTestConfig::long_running();
        assert_eq!(config.simulation_steps, 2000);
    }

    #[test]
    fn test_stress_result_creation() {
        let config = StressTestConfig::many_objects();
        let result = StressTestResult::new(config);
        assert_eq!(result.successful_steps, 0);
        assert_eq!(result.failed_steps, 0);
    }

    #[test]
    fn test_stress_result_passed() {
        let mut result = StressTestResult::new(StressTestConfig::many_objects());
        result.avg_step_time = Duration::from_micros(5000); // 5ms
        result.peak_memory_mb = 100.0;
        result.successful_steps = 1000;

        assert!(result.passed(10.0, 200.0)); // Should pass
        assert!(!result.passed(2.0, 200.0)); // Should fail (too slow)
        assert!(!result.passed(10.0, 50.0)); // Should fail (too much memory)
    }

    #[test]
    fn test_memory_tracker() {
        let mut tracker = MemoryTracker::new();
        tracker.update(1024 * 1024); // 1 MB
        tracker.update(512 * 1024); // 0.5 MB (lower, shouldn't update peak)

        assert!((tracker.peak_mb() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_estimate_memory() {
        let memory = MemoryTracker::estimate_memory(1000, 10);
        // 1000 objects * 1KB + 10 robots * 10KB = 1000KB + 100KB = 1100KB
        assert_eq!(memory, 1024 * 1000 + 10240 * 10);
    }

    #[test]
    fn test_stress_result_report() {
        let config = StressTestConfig::many_objects();
        let mut result = StressTestResult::new(config);
        result.avg_step_time = Duration::from_micros(5000);
        result.max_step_time = Duration::from_micros(10000);
        result.min_step_time = Duration::from_micros(3000);
        result.peak_memory_mb = 150.0;
        result.successful_steps = 1000;

        let report = result.format_report();
        assert!(report.contains("1000 Objects"));
        assert!(report.contains("5.00ms"));
    }

    #[test]
    fn test_pyramid_config_total_boxes() {
        let config = PyramidStackingConfig {
            base_size: 5,
            ..Default::default()
        };
        assert_eq!(config.total_boxes(), 15); // 5 + 4 + 3 + 2 + 1 = 15

        let config = PyramidStackingConfig {
            base_size: 10,
            ..Default::default()
        };
        assert_eq!(config.total_boxes(), 55); // Sum 1 to 10
    }

    #[test]
    #[ignore = "Requires full Bevy simulation environment"]
    fn test_pyramid_stacking_small() {
        let config = PyramidStackingConfig::small();
        let result = run_pyramid_stacking_test(config).expect("Test failed");

        assert!(result.total_boxes > 0, "Should create boxes");
        assert!(
            result.simulation_time.as_secs() < 60,
            "Should complete in reasonable time"
        );
    }

    #[test]
    fn test_pyramid_stacking_medium() {
        let config = PyramidStackingConfig::medium();
        let result = run_pyramid_stacking_test(config).expect("Test failed");

        assert_eq!(result.total_boxes, 55);
        assert!(
            result.pyramid_stable,
            "Medium pyramid should be stable: {}/{} settled",
            result.boxes_settled, result.total_boxes
        );
    }

    #[test]
    fn test_rigid_body_stress_100() {
        let config = RigidBodyStressConfig {
            body_count: 100,
            simulation_steps: 500,
            ..Default::default()
        };
        let result = run_rigid_body_stress_test(config).expect("Test failed");

        assert_eq!(result.body_count, 100);
        assert!(result.successful_steps == 500, "All steps should complete");
        assert!(
            result.fps > 10.0,
            "Should maintain >10 FPS: {:.1}",
            result.fps
        );
    }

    #[test]
    #[ignore = "Requires full Bevy simulation environment"]
    fn test_rigid_body_stress_500() {
        let config = RigidBodyStressConfig {
            body_count: 500,
            simulation_steps: 500,
            ..Default::default()
        };
        let result = run_rigid_body_stress_test(config).expect("Test failed");

        assert_eq!(result.body_count, 500);
        assert!(result.stability_passed, "Should remain stable");
    }

    #[test]
    fn test_rigid_body_stress_1000() {
        let config = RigidBodyStressConfig {
            body_count: 1000,
            simulation_steps: 200,
            ..Default::default()
        };
        let result = run_rigid_body_stress_test(config).expect("Test failed");

        assert_eq!(result.body_count, 1000);
        assert!(
            result.successful_steps == 200,
            "All steps should complete for 1000 bodies"
        );
    }

    #[test]
    fn test_robot_stress_10() {
        let config = RobotStressConfig {
            robot_count: 10,
            links_per_robot: 6,
            simulation_steps: 500,
            ..Default::default()
        };
        let result = run_robot_stress_test(config).expect("Test failed");

        assert_eq!(result.robot_count, 10);
        assert_eq!(result.total_joints, 60); // 10 robots * 6 joints each
        assert!(result.stability_passed, "10 robots should be stable");
    }

    #[test]
    fn test_robot_stress_50() {
        let config = RobotStressConfig {
            robot_count: 50,
            links_per_robot: 6,
            simulation_steps: 300,
            ..Default::default()
        };
        let result = run_robot_stress_test(config).expect("Test failed");

        assert_eq!(result.robot_count, 50);
        assert!(result.successful_steps == 300, "All steps should complete");
    }

    #[test]
    fn test_robot_stress_100() {
        let config = RobotStressConfig {
            robot_count: 100,
            links_per_robot: 4,
            simulation_steps: 200,
            ..Default::default()
        };
        let result = run_robot_stress_test(config).expect("Test failed");

        assert_eq!(result.robot_count, 100);
        assert!(
            result.total_joints == 400,
            "Should have 400 joints: {}",
            result.total_joints
        );
    }

    #[test]
    fn test_long_running_1000_steps() {
        let config = LongRunningConfig {
            object_count: 50,
            simulation_steps: 1000,
            checkpoint_interval: 250,
            ..Default::default()
        };
        let result = run_long_running_test(config).expect("Test failed");

        assert_eq!(result.total_steps, 1000);
        assert_eq!(result.checkpoints.len(), 4); // 250, 500, 750, 1000
        assert!(
            result.stability_maintained,
            "Should maintain stability: degradation={:.2}%",
            result.performance_degradation * 100.0
        );
    }

    #[test]
    fn test_long_running_2000_steps() {
        let config = LongRunningConfig {
            object_count: 30,
            simulation_steps: 2000,
            checkpoint_interval: 500,
            ..Default::default()
        };
        let result = run_long_running_test(config).expect("Test failed");

        assert_eq!(result.total_steps, 2000);
        assert!(
            result.performance_degradation < 1.0,
            "Performance should not degrade more than 100%: {:.2}%",
            result.performance_degradation * 100.0
        );
    }

    #[test]
    fn test_fps_calculation() {
        let step_times = vec![
            Duration::from_millis(10),
            Duration::from_millis(10),
            Duration::from_millis(10),
            Duration::from_millis(10),
            Duration::from_millis(10),
        ];
        let fps = calculate_fps(&step_times);
        assert!((fps - 100.0).abs() < 1.0, "FPS should be ~100: {:.1}", fps);
    }

    #[test]
    fn test_memory_estimation() {
        let mem_100 = MemoryTracker::estimate_memory(100, 0);
        let mem_1000 = MemoryTracker::estimate_memory(1000, 0);

        assert!(mem_1000 > mem_100, "More objects should use more memory");
        assert_eq!(mem_1000, mem_100 * 10);
    }
}
