use crate::cli::Cli;
use crate::config::robot::DiffDrivePresets;
use crate::hframe::HFrameTree;
use crate::physics::diff_drive::CmdVel;
use crate::physics::PhysicsWorld;
use crate::rendering::camera_controller::OrbitCamera;
use crate::robot::urdf_loader::URDFLoader;
use crate::scene::loader::SceneLoader;
use crate::scene::spawner::{ObjectSpawnConfig, ObjectSpawner, SpawnShape, SpawnedObjects};
use bevy::prelude::*;
use std::path::Path;

pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics_world: ResMut<PhysicsWorld>,
    mut spawned_objects: ResMut<SpawnedObjects>,
    mut hframe_tree: ResMut<HFrameTree>,
    cli: Res<Cli>,
) {
    // Always spawn camera with orbit controller
    // Position camera higher and further back for better overview of scenes
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 15.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera::default(),
    ));

    // Always spawn directional light (may be overridden by scene)
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, -0.5, 0.0)),
    ));

    // Track whether robots were loaded from the world file
    let mut world_has_robots = false;

    // Load world file if provided, otherwise create default world
    if let Some(world_path) = &cli.world {
        info!("Loading world from: {:?}", world_path);
        match SceneLoader::load_scene(
            world_path,
            &mut commands,
            &mut physics_world,
            &mut meshes,
            &mut materials,
            &mut spawned_objects,
            &mut hframe_tree,
        ) {
            Ok(loaded_scene) => {
                info!(
                    "Successfully loaded scene: {}",
                    loaded_scene.definition.name
                );
                // Check if the world file included robots
                world_has_robots = !loaded_scene.definition.robots.is_empty();
                if world_has_robots {
                    info!(
                        "World file contains {} robot(s), skipping default robot spawn",
                        loaded_scene.definition.robots.len()
                    );
                }
                commands.insert_resource(loaded_scene);
            }
            Err(e) => {
                error!("Failed to load world file: {}", e);
                warn!("Falling back to default scene");
                spawn_default_world(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut physics_world,
                    &mut spawned_objects,
                );
            }
        }
    } else {
        info!("No world file specified, creating default world with obstacles");
        spawn_default_world(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut physics_world,
            &mut spawned_objects,
        );
    }

    // Load robot file if provided, otherwise spawn default robot (only if world didn't include robots)
    if let Some(robot_path) = &cli.robot {
        info!("Loading robot from: {:?}", robot_path);

        // Get the directory containing the robot file for relative mesh resolution
        let robot_file = Path::new(robot_path);
        let base_path = robot_file.parent().unwrap_or(Path::new("."));

        // Determine file type by extension and load accordingly
        let extension = robot_file
            .extension()
            .and_then(|ext| ext.to_str())
            .unwrap_or("")
            .to_lowercase();

        match extension.as_str() {
            "urdf" | "xacro" => {
                // Load URDF/Xacro robot file
                let mut urdf_loader = URDFLoader::new().with_base_path(base_path);
                match urdf_loader.load(
                    robot_file,
                    &mut commands,
                    &mut physics_world,
                    &mut hframe_tree,
                    &mut meshes,
                    &mut materials,
                ) {
                    Ok(robot_entity) => {
                        spawned_objects.add(robot_entity);
                        info!("Successfully loaded robot from URDF: {:?}", robot_path);
                    }
                    Err(e) => {
                        error!("Failed to load robot from URDF: {}", e);
                        warn!("Falling back to default robot");
                        spawn_default_robot(
                            &mut commands,
                            &mut meshes,
                            &mut materials,
                            &mut physics_world,
                            &mut spawned_objects,
                        );
                    }
                }
            }
            "sdf" => {
                // SDF robot files can be loaded through scene loader
                warn!(
                    "Direct SDF robot loading not yet implemented, falling back to default robot"
                );
                spawn_default_robot(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut physics_world,
                    &mut spawned_objects,
                );
            }
            _ => {
                error!("Unsupported robot file format: {}", extension);
                warn!("Supported formats: .urdf, .xacro");
                warn!("Falling back to default robot");
                spawn_default_robot(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut physics_world,
                    &mut spawned_objects,
                );
            }
        }
    } else if !world_has_robots {
        // Only spawn default robot if no robots were loaded from world file
        info!("No robot file specified, spawning default TurtleBot3-style robot");
        spawn_default_robot(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut physics_world,
            &mut spawned_objects,
        );
    }

    info!("Scene setup complete");
}

/// Spawn the default world with ground plane and some obstacles
fn spawn_default_world(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    physics_world: &mut PhysicsWorld,
    spawned_objects: &mut SpawnedObjects,
) {
    // Spawn ground plane with physics
    let ground =
        ObjectSpawner::spawn_ground(50.0, 50.0, commands, physics_world, meshes, materials);
    spawned_objects.add(ground);

    // Spawn some default obstacles for navigation testing
    let obstacles = [
        // Position (x, y, z), Size (w, h, d), Color
        (
            Vec3::new(3.0, 0.5, 2.0),
            Vec3::new(1.0, 1.0, 1.0),
            Color::srgb(0.8, 0.3, 0.3),
        ),
        (
            Vec3::new(-2.0, 0.5, 3.0),
            Vec3::new(0.8, 1.0, 0.8),
            Color::srgb(0.3, 0.3, 0.8),
        ),
        (
            Vec3::new(4.0, 0.5, -3.0),
            Vec3::new(1.2, 1.0, 0.6),
            Color::srgb(0.3, 0.8, 0.3),
        ),
        (
            Vec3::new(-3.0, 0.5, -2.0),
            Vec3::new(0.6, 1.0, 1.2),
            Color::srgb(0.8, 0.8, 0.3),
        ),
        (
            Vec3::new(0.0, 0.5, 5.0),
            Vec3::new(2.0, 1.0, 0.5),
            Color::srgb(0.5, 0.5, 0.5),
        ),
    ];

    for (pos, size, color) in obstacles {
        let config = ObjectSpawnConfig::new("obstacle", SpawnShape::Box { size })
            .at_position(pos)
            .as_static()
            .with_color(color)
            .with_friction(0.7);
        let entity =
            ObjectSpawner::spawn_object(config, commands, physics_world, meshes, materials);
        spawned_objects.add(entity);
    }

    info!(
        "Default world spawned with {} objects",
        spawned_objects.len()
    );
}

/// Spawn a default differential drive robot (TurtleBot3-style)
fn spawn_default_robot(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    physics_world: &mut PhysicsWorld,
    spawned_objects: &mut SpawnedObjects,
) {
    // Use TurtleBot3 Burger preset for realistic parameters
    let (_robot_config, diff_drive) = DiffDrivePresets::turtlebot3_burger();

    // Store values for logging before moving diff_drive
    let wheel_separation = diff_drive.wheel_separation;
    let wheel_radius = diff_drive.wheel_radius;
    let max_linear_velocity = diff_drive.max_linear_velocity;
    let max_angular_velocity = diff_drive.max_angular_velocity;

    // Robot body dimensions (TurtleBot3 Burger-like)
    let body_size = Vec3::new(0.14, 0.08, 0.14); // width, height, depth
    let body_position = Vec3::new(0.0, 0.1, 0.0); // Slightly above ground

    // Spawn robot body with physics
    let robot_config = ObjectSpawnConfig::new("robot", SpawnShape::Box { size: body_size })
        .at_position(body_position)
        .as_dynamic()
        .with_mass(1.0) // 1 kg (TurtleBot3 Burger is ~1 kg)
        .with_color(Color::srgb(0.2, 0.7, 0.2)) // Green robot
        .with_friction(0.5)
        .with_damping(0.5, 0.5); // Add damping to prevent sliding

    let robot_entity =
        ObjectSpawner::spawn_object(robot_config, commands, physics_world, meshes, materials);

    // Add differential drive and velocity command components
    commands
        .entity(robot_entity)
        .insert((diff_drive, CmdVel::default()));

    spawned_objects.add(robot_entity);

    // Spawn visual wheels (cosmetic only, physics uses body collider)
    // Use the same wheel parameters from diff_drive preset
    let wheel_width = 0.02;
    // Wheels are children of body, so Y is relative to body center
    // Body height is 0.08, so bottom of body is at -0.04 relative to center
    // Place wheels at bottom of robot
    let wheel_y = -0.04 + wheel_radius; // Bottom of body + wheel radius

    // Left wheel - rotate around X axis to lay flat (cylinder default is vertical)
    let left_wheel = commands
        .spawn((
            Name::new("left_wheel"),
            Mesh3d(meshes.add(Cylinder {
                radius: wheel_radius,
                half_height: wheel_width / 2.0,
            })),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.1, 0.1, 0.1),
                ..default()
            })),
            Transform::from_xyz(-wheel_separation / 2.0, wheel_y, 0.0)
                .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
        ))
        .id();

    // Right wheel
    let right_wheel = commands
        .spawn((
            Name::new("right_wheel"),
            Mesh3d(meshes.add(Cylinder {
                radius: wheel_radius,
                half_height: wheel_width / 2.0,
            })),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.1, 0.1, 0.1),
                ..default()
            })),
            Transform::from_xyz(wheel_separation / 2.0, wheel_y, 0.0)
                .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
        ))
        .id();

    // Parent wheels to robot body
    commands
        .entity(robot_entity)
        .add_children(&[left_wheel, right_wheel]);

    // Add a small "direction indicator" on front (relative to body center)
    let indicator = commands
        .spawn((
            Name::new("direction_indicator"),
            Mesh3d(meshes.add(Cuboid::new(0.02, 0.02, 0.04))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.0, 0.0), // Red indicator
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.09), // Front of robot (Z+ is forward)
        ))
        .id();
    commands.entity(robot_entity).add_children(&[indicator]);

    info!("Default robot spawned at {:?}", body_position);
    info!(
        "  - Differential drive: wheel_separation={:.3}m, wheel_radius={:.3}m",
        wheel_separation, wheel_radius
    );
    info!(
        "  - Max velocities: linear={:.2}m/s, angular={:.2}rad/s",
        max_linear_velocity, max_angular_velocity
    );
    info!("  - Control topic: robot.cmd_vel (use HORUS Hub to send CmdVel messages)");
}
