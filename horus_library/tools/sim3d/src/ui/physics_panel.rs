//! Physics Control Panel
//!
//! Provides runtime controls for physics simulation parameters including
//! gravity, damping, solver iterations, and material properties.

use bevy::prelude::*;
use bevy_egui::egui;
use std::num::NonZeroUsize;

/// Physics panel configuration resource
#[derive(Resource)]
pub struct PhysicsPanelConfig {
    /// Show advanced parameters
    pub show_advanced: bool,
    /// Show debug stats
    pub show_debug_stats: bool,
}

impl Default for PhysicsPanelConfig {
    fn default() -> Self {
        Self {
            show_advanced: false,
            show_debug_stats: true,
        }
    }
}

/// Editable physics parameters (separate from PhysicsWorld to allow UI editing)
#[derive(Resource)]
pub struct PhysicsParams {
    /// Gravity vector (m/s²)
    pub gravity: [f32; 3],
    /// Physics timestep (seconds)
    pub dt: f32,
    /// Linear damping (0.0 = no damping)
    pub linear_damping: f32,
    /// Angular damping (0.0 = no damping)
    pub angular_damping: f32,
    /// Number of velocity solver iterations
    pub num_solver_iterations: usize,
    /// Number of additional friction solver iterations
    pub num_additional_friction_iterations: usize,
    /// Enable continuous collision detection
    pub ccd_enabled: bool,
    /// Maximum CCD substeps
    pub max_ccd_substeps: usize,
    /// Contact natural frequency for soft contacts
    pub contact_natural_frequency: f32,
    /// Contact damping ratio
    pub contact_damping_ratio: f32,
    /// Whether params have been modified and need sync
    pub dirty: bool,
}

impl Default for PhysicsParams {
    fn default() -> Self {
        Self {
            gravity: [0.0, -9.81, 0.0],
            dt: 1.0 / 240.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            num_solver_iterations: 4,
            num_additional_friction_iterations: 4,
            ccd_enabled: true,
            max_ccd_substeps: 1,
            contact_natural_frequency: 30.0,
            contact_damping_ratio: 0.5,
            dirty: false,
        }
    }
}

impl PhysicsParams {
    /// Create from current PhysicsWorld state
    pub fn from_physics_world(world: &crate::physics::PhysicsWorld) -> Self {
        let ip = &world.integration_parameters;
        Self {
            gravity: [world.gravity.x, world.gravity.y, world.gravity.z],
            dt: ip.dt,
            linear_damping: 0.0, // Not stored in integration params
            angular_damping: 0.0,
            num_solver_iterations: ip.num_solver_iterations.get(),
            num_additional_friction_iterations: ip.num_additional_friction_iterations,
            ccd_enabled: true,
            max_ccd_substeps: ip.max_ccd_substeps,
            contact_natural_frequency: ip.contact_natural_frequency,
            contact_damping_ratio: ip.contact_damping_ratio,
            dirty: false,
        }
    }

    /// Apply changes to PhysicsWorld
    pub fn apply_to_physics_world(&self, world: &mut crate::physics::PhysicsWorld) {
        world.gravity = nalgebra::Vector3::new(self.gravity[0], self.gravity[1], self.gravity[2]);
        world.integration_parameters.dt = self.dt;
        // NonZero requires at least 1 iteration
        if let Some(nz) = NonZeroUsize::new(self.num_solver_iterations.max(1)) {
            world.integration_parameters.num_solver_iterations = nz;
        }
        world.integration_parameters.num_additional_friction_iterations = self.num_additional_friction_iterations;
        world.integration_parameters.max_ccd_substeps = self.max_ccd_substeps;
        world.integration_parameters.contact_natural_frequency = self.contact_natural_frequency;
        world.integration_parameters.contact_damping_ratio = self.contact_damping_ratio;
    }

    /// Preset: Earth gravity
    pub fn preset_earth(&mut self) {
        self.gravity = [0.0, -9.81, 0.0];
        self.dirty = true;
    }

    /// Preset: Moon gravity
    pub fn preset_moon(&mut self) {
        self.gravity = [0.0, -1.62, 0.0];
        self.dirty = true;
    }

    /// Preset: Mars gravity
    pub fn preset_mars(&mut self) {
        self.gravity = [0.0, -3.72, 0.0];
        self.dirty = true;
    }

    /// Preset: Zero gravity (space)
    pub fn preset_zero_g(&mut self) {
        self.gravity = [0.0, 0.0, 0.0];
        self.dirty = true;
    }

    /// Preset: High precision (slow but accurate)
    pub fn preset_high_precision(&mut self) {
        self.dt = 1.0 / 480.0;
        self.num_solver_iterations = 8;
        self.num_additional_friction_iterations = 8;
        self.dirty = true;
    }

    /// Preset: Performance (fast but less accurate)
    pub fn preset_performance(&mut self) {
        self.dt = 1.0 / 120.0;
        self.num_solver_iterations = 2;
        self.num_additional_friction_iterations = 2;
        self.dirty = true;
    }

    /// Preset: Default balanced
    pub fn preset_default(&mut self) {
        self.dt = 1.0 / 240.0;
        self.num_solver_iterations = 4;
        self.num_additional_friction_iterations = 4;
        self.dirty = true;
    }
}

/// Render the physics control panel UI
pub fn render_physics_panel_ui(
    ui: &mut egui::Ui,
    params: &mut PhysicsParams,
    config: &mut PhysicsPanelConfig,
    physics_world: Option<&crate::physics::PhysicsWorld>,
) {
    ui.heading("⚙ Physics");
    ui.separator();

    // Gravity section
    ui.collapsing("🌍 Gravity", |ui| {
        ui.horizontal(|ui| {
            ui.label("X:");
            if ui.add(egui::DragValue::new(&mut params.gravity[0]).speed(0.1).suffix(" m/s²")).changed() {
                params.dirty = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Y:");
            if ui.add(egui::DragValue::new(&mut params.gravity[1]).speed(0.1).suffix(" m/s²")).changed() {
                params.dirty = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Z:");
            if ui.add(egui::DragValue::new(&mut params.gravity[2]).speed(0.1).suffix(" m/s²")).changed() {
                params.dirty = true;
            }
        });

        ui.separator();
        ui.label("Presets:");
        ui.horizontal(|ui| {
            if ui.button("Earth").clicked() {
                params.preset_earth();
            }
            if ui.button("Moon").clicked() {
                params.preset_moon();
            }
            if ui.button("Mars").clicked() {
                params.preset_mars();
            }
            if ui.button("Zero-G").clicked() {
                params.preset_zero_g();
            }
        });
    });

    // Timestep section
    ui.collapsing("⏱ Timestep", |ui| {
        let hz = (1.0 / params.dt).round() as i32;
        ui.horizontal(|ui| {
            ui.label("Rate:");
            let mut hz_edit = hz;
            if ui.add(egui::DragValue::new(&mut hz_edit).range(30..=1000).suffix(" Hz")).changed() {
                params.dt = 1.0 / hz_edit as f32;
                params.dirty = true;
            }
        });
        ui.label(format!("dt = {:.6} s", params.dt));

        ui.separator();
        ui.label("Presets:");
        ui.horizontal(|ui| {
            if ui.button("120 Hz").clicked() {
                params.dt = 1.0 / 120.0;
                params.dirty = true;
            }
            if ui.button("240 Hz").clicked() {
                params.dt = 1.0 / 240.0;
                params.dirty = true;
            }
            if ui.button("480 Hz").clicked() {
                params.dt = 1.0 / 480.0;
                params.dirty = true;
            }
        });
    });

    // Solver section
    ui.collapsing("🔧 Solver", |ui| {
        ui.horizontal(|ui| {
            ui.label("Velocity iterations:");
            if ui.add(egui::DragValue::new(&mut params.num_solver_iterations).range(1..=20)).changed() {
                params.dirty = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Friction iterations:");
            if ui.add(egui::DragValue::new(&mut params.num_additional_friction_iterations).range(0..=20)).changed() {
                params.dirty = true;
            }
        });

        ui.separator();
        ui.label("Presets:");
        ui.horizontal(|ui| {
            if ui.button("Performance").clicked() {
                params.preset_performance();
            }
            if ui.button("Default").clicked() {
                params.preset_default();
            }
            if ui.button("High Precision").clicked() {
                params.preset_high_precision();
            }
        });
    });

    // CCD section
    if config.show_advanced {
        ui.collapsing("💥 Continuous Collision Detection", |ui| {
            if ui.checkbox(&mut params.ccd_enabled, "Enable CCD").changed() {
                params.dirty = true;
            }
            ui.horizontal(|ui| {
                ui.label("Max substeps:");
                if ui.add(egui::DragValue::new(&mut params.max_ccd_substeps).range(1..=10)).changed() {
                    params.dirty = true;
                }
            });
        });

        // Contact parameters
        ui.collapsing("📞 Contact Parameters", |ui| {
            ui.horizontal(|ui| {
                ui.label("Natural frequency:");
                if ui.add(egui::DragValue::new(&mut params.contact_natural_frequency).speed(1.0).range(1.0..=100.0).suffix(" Hz")).changed() {
                    params.dirty = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Damping ratio:");
                if ui.add(egui::DragValue::new(&mut params.contact_damping_ratio).speed(0.01).range(0.0..=1.0)).changed() {
                    params.dirty = true;
                }
            });
        });
    }

    // Debug stats
    if config.show_debug_stats {
        if let Some(world) = physics_world {
            ui.separator();
            ui.collapsing("📊 Debug Stats", |ui| {
                ui.label(format!("Rigid bodies: {}", world.rigid_body_set.len()));
                ui.label(format!("Colliders: {}", world.collider_set.len()));
                ui.label(format!("Impulse joints: {}", world.impulse_joint_set.len()));
                // Note: MultibodyJointSet and IslandManager don't expose count methods publicly
            });
        }
    }

    // Options
    ui.separator();
    ui.horizontal(|ui| {
        ui.checkbox(&mut config.show_advanced, "Advanced");
        ui.checkbox(&mut config.show_debug_stats, "Debug Stats");
    });

    // Apply button
    if params.dirty {
        ui.separator();
        ui.colored_label(egui::Color32::YELLOW, "⚠ Parameters modified - click Apply");
        if ui.button("Apply Changes").clicked() {
            params.dirty = false;
            // The actual application happens in sync_physics_params_system
        }
    }
}

/// System to sync physics params to world when dirty flag is cleared
pub fn sync_physics_params_system(
    params: Res<PhysicsParams>,
    mut physics_world: Option<ResMut<crate::physics::PhysicsWorld>>,
) {
    // Only sync when Apply was clicked (dirty just became false)
    if !params.dirty {
        if let Some(ref mut world) = physics_world {
            params.apply_to_physics_world(world);
        }
    }
}

/// System to initialize physics params from world on startup
pub fn init_physics_params_system(
    mut params: ResMut<PhysicsParams>,
    physics_world: Option<Res<crate::physics::PhysicsWorld>>,
) {
    if let Some(world) = physics_world {
        *params = PhysicsParams::from_physics_world(&world);
    }
}

/// Physics panel plugin
pub struct PhysicsPanelPlugin;

impl Plugin for PhysicsPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsPanelConfig>()
            .init_resource::<PhysicsParams>()
            .add_systems(Startup, init_physics_params_system)
            .add_systems(Update, sync_physics_params_system);
    }
}
