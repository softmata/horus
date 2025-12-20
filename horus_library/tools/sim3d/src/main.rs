// Bevy ECS standard patterns - these are idiomatic for Bevy systems
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
// Public API: Many types in this binary's modules are public API for the sim3d library,
// exported for external users but not all are used within the binary itself.
#![allow(dead_code)]
// Some sensor types use IMU as the standard acronym
#![allow(clippy::upper_case_acronyms)]
// Methods on Copy types using &self are intentional for API consistency
#![allow(clippy::wrong_self_convention)]
// EnhancedError is intentionally large (136 bytes) to carry rich error context
// (file path, line/column, hints, suggestions). Error paths are not hot paths.
#![allow(clippy::result_large_err)]
// Simulator code often uses test assertions with length comparisons
#![allow(clippy::len_zero)]
// Simulator uses manual range checks for clarity in physics code
#![allow(clippy::manual_range_contains)]
// Field initialization patterns are common in test fixtures
#![allow(clippy::field_reassign_with_default)]
// Private interfaces in Bevy system signatures are acceptable
#![allow(private_interfaces)]
// Test code uses approximate PI values intentionally in tests
#![allow(clippy::approx_constant)]
// Unused variables in tests are fine
#![allow(unused_variables)]
// Reference patterns in Bevy queries
#![allow(clippy::needless_borrow)]

use bevy::prelude::*;

mod assets;
mod cli;
mod config;
mod editor;
mod error;
mod gpu;
mod hframe;
mod horus_native;
mod multi_robot;
mod physics;
mod plugins;
mod procedural;
mod recording;
mod rendering;
mod robot;
mod scene;
mod sensors;
mod systems;
mod ui;
mod utils;
mod view_modes;

#[cfg(feature = "python")]
mod rl;

use cli::{Cli, Command, Mode};
use hframe::HFrameTree;
use physics::PhysicsWorld;
use scene::spawner::SpawnedObjects;
use systems::sensor_update::{SensorSystemSet, SensorUpdatePlugin};

// Import all plugins for integration
use gpu::GPUAccelerationPlugin;
use multi_robot::MultiRobotPlugin;
use physics::soft_body::SoftBodyPlugin;
use physics::AdvancedPhysicsPlugin;
use plugins::PluginSystemPlugin;
use procedural::ProceduralGenerationPlugin;
use recording::RecordingPlugin;
use rendering::{
    ambient_occlusion::AmbientOcclusionPlugin, area_lights::AreaLightsPlugin,
    atmosphere::AtmospherePlugin, environment::EnvironmentPlugin, gizmos::GizmoPlugin,
    materials::MaterialPlugin, post_processing::PostProcessingPlugin, shadows::ShadowsPlugin,
};
use robot::{articulated::ArticulatedRobotPlugin, state::JointStatePlugin};
use sensors::{
    depth::DepthCameraPlugin, imu::IMUPlugin, rgbd::RGBDCameraPlugin,
    segmentation::SegmentationCameraPlugin, tactile::TactileSensorPlugin,
    thermal::ThermalCameraPlugin,
};
use systems::{
    hframe_update::HFrameUpdatePlugin, horus_comm::HorusCommPlugin, horus_sync::HorusSyncPlugin,
    topic_discovery::TopicDiscoveryPlugin,
};
use view_modes::{
    collision_mode::CollisionVisualizationPlugin, hframe_mode::HFrameVisualizationPlugin,
    physics_mode::PhysicsVisualizationPlugin,
};

/// System sets for organizing update order
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum SimSystemSet {
    /// Physics simulation and force application
    Physics,
    /// HFrame transform updates
    HFrame,
}

fn main() {
    let cli = Cli::parse();

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    // Handle subcommands first
    if let Some(command) = &cli.command {
        match command {
            Command::Validate {
                files,
                validation_type,
                format,
                check_meshes,
                verbose: _,
            } => {
                run_validate_command(files, *validation_type, *format, *check_meshes);
                return;
            }
        }
    }

    info!("Starting sim3d");
    info!("Mode: {:?}", cli.mode);

    match cli.mode {
        Mode::Visual => run_visual_mode(cli),
        Mode::Headless => run_headless_mode(cli),
    }
}

fn run_validate_command(
    files: &[std::path::PathBuf],
    validation_type: Option<cli::CliValidationType>,
    format: cli::CliOutputFormat,
    check_meshes: bool,
) {
    use cli::{format_batch_report, validate_file};
    use cli::{BatchValidationReport, OutputFormat};

    let mut results = Vec::new();
    let mut valid_count = 0;
    let mut invalid_count = 0;

    for file in files {
        let vtype = validation_type.map(|v| v.into());
        match validate_file(file, vtype, check_meshes) {
            Ok(result) => {
                if result.valid {
                    valid_count += 1;
                } else {
                    invalid_count += 1;
                }
                results.push(result);
            }
            Err(e) => {
                eprintln!("Error validating {}: {}", file.display(), e);
                invalid_count += 1;
            }
        }
    }

    let report = BatchValidationReport {
        total_files: files.len(),
        valid_files: valid_count,
        invalid_files: invalid_count,
        results,
    };

    let output_format: OutputFormat = format.into();
    match format_batch_report(&report, output_format) {
        Ok(output) => println!("{}", output),
        Err(e) => eprintln!("Error formatting output: {}", e),
    }

    // Exit with error code if any files failed validation
    if invalid_count > 0 {
        std::process::exit(1);
    }
}

/// Resource for automated screenshot capture
#[derive(Resource)]
struct AutoScreenshot {
    output_path: std::path::PathBuf,
    frames_to_wait: u32,
    current_frame: u32,
    captured: bool,
}

/// Resource to track when we should exit (after screenshot is captured)
#[derive(Resource, Default)]
struct ScreenshotExitTimer {
    frames_until_exit: Option<u32>,
}

/// Resource to track screenshot camera settings
#[derive(Resource)]
struct ScreenshotCameraConfig {
    positioned: bool,
    angle: String,
    distance_multiplier: f32,
    world_view: bool,
}

impl Default for ScreenshotCameraConfig {
    fn default() -> Self {
        Self {
            positioned: false,
            angle: "isometric".to_string(),
            distance_multiplier: 1.0,
            world_view: false,
        }
    }
}

/// System to reposition camera for screenshot verification
/// Supports multiple angles: front, back, left, right, top, isometric
/// Can focus on robot close-up or world overview
fn screenshot_camera_setup_system(
    mut config: ResMut<ScreenshotCameraConfig>,
    mut camera_query: Query<&mut Transform, With<Camera3d>>,
    robot_query: Query<
        &Transform,
        (
            With<physics::diff_drive::DifferentialDrive>,
            Without<Camera3d>,
        ),
    >,
) {
    if config.positioned {
        return;
    }

    // Find the robot position
    let robot_pos = if let Ok(robot_transform) = robot_query.get_single() {
        robot_transform.translation
    } else {
        // Default robot position if not found yet
        Vec3::new(0.0, 0.1, 0.0)
    };

    if let Ok(mut camera_transform) = camera_query.get_single_mut() {
        let (camera_offset, look_target) = if config.world_view {
            // World overview - higher and further back to see entire scene
            let dist = 8.0 * config.distance_multiplier;
            (
                Vec3::new(dist * 0.5, dist * 0.6, dist * 0.5),
                Vec3::new(0.0, 0.0, 0.0), // Look at world center
            )
        } else {
            // Robot close-up with various angles
            let base_dist = 0.8 * config.distance_multiplier;
            let height = 0.4 * config.distance_multiplier;

            let offset = match config.angle.as_str() {
                "front" => Vec3::new(0.0, height, base_dist), // Front view (Z+)
                "back" => Vec3::new(0.0, height, -base_dist), // Back view (Z-)
                "left" => Vec3::new(-base_dist, height, 0.0), // Left view (X-)
                "right" => Vec3::new(base_dist, height, 0.0), // Right view (X+)
                "top" => Vec3::new(0.0, base_dist * 1.5, 0.1), // Top-down view
                "isometric" | _ => Vec3::new(
                    // 3/4 isometric view
                    base_dist * 0.7,
                    height,
                    base_dist * 0.7,
                ),
            };
            (offset, robot_pos + Vec3::new(0.0, 0.05, 0.0))
        };

        camera_transform.translation = robot_pos + camera_offset;
        camera_transform.look_at(look_target, Vec3::Y);

        info!(
            "Screenshot camera: angle={}, world_view={}, position={:?}, target={:?}",
            config.angle, config.world_view, camera_transform.translation, look_target
        );
        config.positioned = true;
    }
}

/// System to capture screenshot after N frames using Bevy 0.15 observer pattern
fn auto_screenshot_system(mut commands: Commands, mut screenshot: ResMut<AutoScreenshot>) {
    if screenshot.captured {
        return;
    }

    screenshot.current_frame += 1;

    if screenshot.current_frame >= screenshot.frames_to_wait {
        let path = screenshot.output_path.clone();
        info!("Capturing screenshot to: {:?}", path);

        // Use Bevy 0.15's observer-based screenshot API
        use bevy::render::view::screenshot::{save_to_disk, Screenshot};
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));

        screenshot.captured = true;
        info!("Screenshot capture initiated");
    }
}

/// System to exit after screenshot is saved
fn screenshot_exit_system(
    mut exit_timer: ResMut<ScreenshotExitTimer>,
    screenshot: Res<AutoScreenshot>,
    mut exit: EventWriter<bevy::app::AppExit>,
) {
    if screenshot.captured {
        // Start or decrement exit timer
        match exit_timer.frames_until_exit {
            None => {
                // Give 10 frames for screenshot to be written
                exit_timer.frames_until_exit = Some(10);
            }
            Some(0) => {
                info!("Screenshot saved, exiting...");
                exit.send(bevy::app::AppExit::Success);
            }
            Some(ref mut frames) => {
                *frames -= 1;
            }
        }
    }
}

fn run_visual_mode(cli: Cli) {
    let screenshot_mode = cli.screenshot.is_some();
    let screenshot_path = cli.screenshot.clone();
    let screenshot_frames = cli.screenshot_frames;

    let mut app = App::new();

    // Adjust window size for screenshot mode
    let (width, height) = if screenshot_mode {
        (1280.0, 720.0)
    } else {
        (1920.0, 1080.0)
    };

    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "sim3d - HORUS 3D Simulator".into(),
                    resolution: (width, height).into(),
                    ..default()
                }),
                ..default()
            })
            .set(AssetPlugin { ..default() })
            .disable::<bevy::log::LogPlugin>(), // Disable since we init tracing manually
    );

    // Add screenshot automation if --screenshot was specified
    if let Some(path) = screenshot_path {
        app.insert_resource(AutoScreenshot {
            output_path: path,
            frames_to_wait: screenshot_frames,
            current_frame: 0,
            captured: false,
        });
        app.init_resource::<ScreenshotExitTimer>();
        app.insert_resource(ScreenshotCameraConfig {
            positioned: false,
            angle: cli.screenshot_angle.clone(),
            distance_multiplier: cli.screenshot_distance,
            world_view: cli.screenshot_world,
        });
        // Camera setup runs first, then screenshot capture, then exit check
        app.add_systems(
            Update,
            (
                screenshot_camera_setup_system,
                auto_screenshot_system,
                screenshot_exit_system,
            )
                .chain(),
        );
        info!(
            "Screenshot mode: will capture after {} frames",
            screenshot_frames
        );
    }

    // Only add UI plugins if not in screenshot mode
    if !screenshot_mode {
        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiPlugin;
            app.add_plugins(EguiPlugin);
        }

        #[cfg(feature = "editor")]
        {
            use bevy_editor_pls::EditorPlugin;
            app.add_plugins(EditorPlugin::default());
        }
    }

    // Add sensor update plugin
    app.add_plugins(SensorUpdatePlugin);

    // === INTEGRATED PLUGINS ===

    // Native HORUS integration - auto-wires all topics
    app.add_plugins(horus_native::HorusNativePlugin::new(&cli.robot_name));
    app.add_plugins(HorusSyncPlugin);
    app.add_plugins(HorusCommPlugin);
    app.add_plugins(TopicDiscoveryPlugin);
    app.add_plugins(HFrameUpdatePlugin);

    // Physics plugins
    app.add_plugins(SoftBodyPlugin);
    app.add_plugins(GPUAccelerationPlugin);
    app.add_plugins(AdvancedPhysicsPlugin);

    // Rendering plugins
    app.add_plugins(MaterialPlugin);
    app.add_plugins(GizmoPlugin);
    app.add_plugins(EnvironmentPlugin);
    app.add_plugins(ShadowsPlugin::default());
    app.add_plugins(AmbientOcclusionPlugin::default());
    app.add_plugins(AtmospherePlugin);
    app.add_plugins(AreaLightsPlugin);
    app.add_plugins(PostProcessingPlugin::default());

    // Robot plugins
    app.add_plugins(ArticulatedRobotPlugin);
    app.add_plugins(JointStatePlugin);
    app.add_plugins(MultiRobotPlugin::default());

    // Sensor plugins
    app.add_plugins(DepthCameraPlugin);
    app.add_plugins(RGBDCameraPlugin);
    app.add_plugins(SegmentationCameraPlugin);
    app.add_plugins(ThermalCameraPlugin);
    app.add_plugins(IMUPlugin);
    app.add_plugins(TactileSensorPlugin);

    // View mode plugins (debug visualization) - skip in screenshot mode
    if !screenshot_mode {
        app.add_plugins(CollisionVisualizationPlugin);
        app.add_plugins(PhysicsVisualizationPlugin);
        app.add_plugins(HFrameVisualizationPlugin);
    }

    // Utility plugins
    app.add_plugins(ProceduralGenerationPlugin);
    app.add_plugins(RecordingPlugin);

    // Plugin system (for dynamic plugin loading and example plugins)
    app.add_plugins(PluginSystemPlugin);

    app.insert_resource(PhysicsWorld::default())
        .insert_resource(HFrameTree::with_root("world"))
        .insert_resource(SpawnedObjects::default())
        .insert_resource(cli)
        .init_resource::<systems::physics_step::PhysicsAccumulator>();

    // Configure system set ordering: Physics -> Sensors -> TF
    app.configure_sets(
        Update,
        (
            SimSystemSet::Physics,
            SensorSystemSet::Update,
            SensorSystemSet::Visualization,
            SimSystemSet::HFrame,
        )
            .chain(),
    );

    app.add_systems(Startup, (rendering::setup::setup_scene,));

    // Physics systems
    app.add_systems(
        Update,
        (
            systems::physics_step::physics_step_system,
            systems::sync_visual::apply_external_forces_system,
            systems::sync_visual::apply_external_impulses_system,
            systems::sync_visual::apply_differential_drive_system,
            systems::sync_visual::sync_physics_to_visual_system,
            systems::sync_visual::sync_velocities_from_physics_system,
        )
            .chain()
            .in_set(SimSystemSet::Physics),
    );

    // TF update systems
    app.add_systems(
        Update,
        systems::hframe_update::hframe_update_system.in_set(SimSystemSet::HFrame),
    );

    #[cfg(feature = "visual")]
    if !screenshot_mode {
        // UI plugins (skip in screenshot mode for cleaner output)
        app.add_plugins(ui::layouts::LayoutPlugin);
        app.add_plugins(ui::keybindings::KeyBindingsPlugin::default());
        app.add_plugins(ui::view_modes::ViewModePlugin);
        app.add_plugins(ui::hframe_panel::HFramePanelPlugin);
        app.add_plugins(ui::stats_panel::StatsPanelPlugin);
        app.add_plugins(ui::status_bar::StatusBarPlugin);
        app.add_plugins(ui::controls::ControlsPlugin);
        app.add_plugins(ui::theme::ThemePlugin);
        app.add_plugins(ui::tooltips::TooltipsPlugin);
        app.add_plugins(ui::notifications::NotificationsPlugin);
        app.add_plugins(ui::recent_files::RecentFilesPlugin::default());
        app.add_plugins(ui::crash_recovery::CrashRecoveryPlugin::default());
        app.add_plugins(ui::dock::DockPlugin);
        app.add_plugins(ui::plugin_panel::PluginPanelPlugin);

        // New simulation control panels
        app.add_plugins(ui::physics_panel::PhysicsPanelPlugin);
        app.add_plugins(ui::sensor_panel::SensorPanelPlugin);
        app.add_plugins(ui::rendering_panel::RenderingPanelPlugin);
        app.add_plugins(ui::recording_panel::RecordingPanelPlugin);
        app.add_plugins(ui::horus_panel::HorusPanelPlugin);

        // Editor plugins (EditorPlugin already includes undo internally)
        app.add_plugins(editor::EditorPlugin);

        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                ui::debug_panel::debug_panel_system.after(EguiSet::InitContexts),
            );
        }

        app.add_systems(
            Update,
            (
                rendering::camera_controller::camera_controller_system,
                hframe::render_hframe_frames,
            ),
        );
    }

    info!("Starting visual simulation");
    app.run();
}

fn run_headless_mode(cli: Cli) {
    info!("Starting headless mode for RL training");

    let robot_name = cli.robot_name.clone();
    let mut app = App::new();

    // Use minimal plugins (no rendering, no input, no audio)
    app.add_plugins(MinimalPlugins);

    // Add asset plugin for headless mode (needed for mesh/material assets even without rendering)
    app.add_plugins(AssetPlugin::default());
    app.init_resource::<Assets<Mesh>>();
    app.init_resource::<Assets<StandardMaterial>>();

    // Add essential resources
    app.insert_resource(PhysicsWorld::default())
        .insert_resource(HFrameTree::with_root("world"))
        .insert_resource(SpawnedObjects::default())
        .insert_resource(cli)
        .init_resource::<systems::physics_step::PhysicsAccumulator>();

    // Configure system set ordering: Physics -> Sensors -> TF
    app.configure_sets(
        Update,
        (
            SimSystemSet::Physics,
            SensorSystemSet::Update,
            SimSystemSet::HFrame,
        )
            .chain(),
    );

    // Add sensor update plugin (without visualization)
    app.add_plugins(SensorUpdatePlugin);

    // === HEADLESS MODE PLUGINS ===

    // Native HORUS integration - auto-wires all topics
    app.add_plugins(horus_native::HorusNativePlugin::new(&robot_name));
    app.add_plugins(HorusSyncPlugin);
    app.add_plugins(HorusCommPlugin);
    app.add_plugins(TopicDiscoveryPlugin);
    app.add_plugins(HFrameUpdatePlugin);

    // Physics plugins
    app.add_plugins(SoftBodyPlugin);
    app.add_plugins(GPUAccelerationPlugin);
    app.add_plugins(AdvancedPhysicsPlugin);

    // Robot plugins
    app.add_plugins(ArticulatedRobotPlugin);
    app.add_plugins(JointStatePlugin);
    app.add_plugins(MultiRobotPlugin::default());

    // Sensor plugins (headless mode - for data processing)
    app.add_plugins(SegmentationCameraPlugin);
    app.add_plugins(ThermalCameraPlugin);
    app.add_plugins(IMUPlugin);
    app.add_plugins(TactileSensorPlugin);

    // Utility plugins
    app.add_plugins(ProceduralGenerationPlugin);
    app.add_plugins(RecordingPlugin);

    // Plugin system (for dynamic plugin loading)
    app.add_plugins(PluginSystemPlugin);

    // Physics systems (same as visual mode)
    app.add_systems(
        Update,
        (
            systems::physics_step::physics_step_system,
            systems::sync_visual::apply_external_forces_system,
            systems::sync_visual::apply_external_impulses_system,
            systems::sync_visual::apply_differential_drive_system,
            systems::sync_visual::sync_physics_to_visual_system,
            systems::sync_visual::sync_velocities_from_physics_system,
        )
            .chain()
            .in_set(SimSystemSet::Physics),
    );

    // TF update systems
    app.add_systems(
        Update,
        systems::hframe_update::hframe_update_system.in_set(SimSystemSet::HFrame),
    );

    #[cfg(feature = "python")]
    {
        use rl::RLTaskManager;

        // Add RL task manager
        app.init_resource::<RLTaskManager>();

        // Add RL rendering system (for debug gizmos if needed)
        app.add_systems(Update, rl::rl_task_render_system);
    }

    // Setup initial scene (without rendering)
    app.add_systems(Startup, setup_headless_scene);

    info!("Running headless simulation at maximum speed");
    info!("Press Ctrl+C to stop");

    app.run();
}

/// Setup scene for headless mode (no rendering components)
fn setup_headless_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics_world: ResMut<PhysicsWorld>,
    mut spawned_objects: ResMut<SpawnedObjects>,
    mut hframe_tree: ResMut<HFrameTree>,
    cli: Res<Cli>,
) {
    info!("Setting up headless scene");

    // Load world file if provided, otherwise create default ground
    if let Some(world_path) = &cli.world {
        info!("Loading world from: {:?}", world_path);
        match scene::loader::SceneLoader::load_scene(
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
                commands.insert_resource(loaded_scene);
            }
            Err(e) => {
                error!("Failed to load world file: {}", e);
                warn!("Falling back to default ground plane");
                create_default_ground(&mut physics_world);
            }
        }
    } else {
        info!("No world file specified, creating default ground plane");
        create_default_ground(&mut physics_world);
    }

    info!("Headless scene setup complete");
}

fn create_default_ground(physics_world: &mut PhysicsWorld) {
    use physics::collider::{ColliderBuilder, ColliderShape};
    use rapier3d::prelude::*;

    // Ground plane
    let ground_rb = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0, 0.0])
        .build();

    let ground_handle = physics_world.rigid_body_set.insert(ground_rb);

    let ground_collider = ColliderBuilder::new(ColliderShape::Box {
        half_extents: Vec3::new(50.0, 0.1, 50.0),
    })
    .friction(0.7)
    .build();

    // Use reborrow pattern to split mutable borrows
    let PhysicsWorld {
        collider_set,
        rigid_body_set,
        ..
    } = &mut *physics_world;

    collider_set.insert_with_parent(ground_collider, ground_handle, rigid_body_set);
}
