#[cfg(feature = "visual")]
use bevy::prelude::*;
#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

#[cfg(feature = "visual")]
use crate::robot::Robot;
#[cfg(feature = "visual")]
use crate::sensors::encoder::Encoder;
#[cfg(feature = "visual")]
use crate::sensors::force_torque::ForceTorqueSensor;
#[cfg(feature = "visual")]
use crate::sensors::gps::GPS;
#[cfg(feature = "visual")]
use crate::sensors::imu::IMU;
#[cfg(feature = "visual")]
use crate::sensors::lidar3d::Lidar3D;
#[cfg(feature = "visual")]
use crate::systems::horus_sync::HorusSyncStats;

/// Resource to track frame time breakdown
#[derive(Resource, Default)]
pub struct FrameTimeBreakdown {
    pub physics_time_ms: f32,
    pub sensor_time_ms: f32,
    pub rendering_time_ms: f32,
    pub total_time_ms: f32,
}

/// Resource to track simulation statistics
#[derive(Resource, Default)]
pub struct SimulationStats {
    pub total_entities: usize,
    pub robot_count: usize,
    pub sensor_count: usize,
    pub obstacle_count: usize,
    pub rigid_body_count: usize,
    pub collider_count: usize,
    pub joint_count: usize,
    pub contact_count: usize,
    pub simulation_time: f32,
    pub real_time: f32,
}

impl SimulationStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn time_ratio(&self) -> f32 {
        if self.real_time > 0.0 {
            self.simulation_time / self.real_time
        } else {
            0.0
        }
    }

    pub fn estimated_memory_mb(&self) -> f32 {
        // Rough estimates:
        // Entity: ~200 bytes
        // RigidBody: ~500 bytes
        // Collider: ~300 bytes
        // Joint: ~400 bytes
        let entity_mem = self.total_entities as f32 * 200.0;
        let rb_mem = self.rigid_body_count as f32 * 500.0;
        let collider_mem = self.collider_count as f32 * 300.0;
        let joint_mem = self.joint_count as f32 * 400.0;

        (entity_mem + rb_mem + collider_mem + joint_mem) / 1_048_576.0 // Convert to MB
    }
}

/// System to collect simulation statistics
pub fn collect_stats_system(
    mut stats: ResMut<SimulationStats>,
    time: Res<Time>,
    entities: Query<Entity>,
    robots: Query<&Robot>,
    lidars: Query<&Lidar3D>,
    imus: Query<&IMU>,
    gps: Query<&GPS>,
    encoders: Query<&Encoder>,
    force_torque: Query<&ForceTorqueSensor>,
    physics_world: Option<Res<crate::physics::PhysicsWorld>>,
) {
    stats.total_entities = entities.iter().count();
    stats.robot_count = robots.iter().count();

    // Count all sensor types
    stats.sensor_count = lidars.iter().count()
        + imus.iter().count()
        + gps.iter().count()
        + encoders.iter().count()
        + force_torque.iter().count();

    stats.simulation_time = time.elapsed_secs();
    stats.real_time = time.elapsed_secs(); // Real time tracking would need separate timer

    // Get physics stats from PhysicsWorld
    if let Some(physics) = physics_world {
        stats.rigid_body_count = physics.rigid_body_set.len();
        stats.collider_count = physics.collider_set.len();
        stats.joint_count = physics.impulse_joint_set.len();
        // Count active contacts from narrow phase
        stats.contact_count = physics.narrow_phase.contact_pairs().count();
    }
}

#[cfg(feature = "visual")]
use crate::ui::dock::DockConfig;

#[cfg(feature = "visual")]
/// Main stats panel system - only shown when dock mode is disabled
pub fn stats_panel_system(
    mut contexts: EguiContexts,
    time: Res<Time>,
    stats: Res<SimulationStats>,
    frame_time: Res<FrameTimeBreakdown>,
    horus_stats: Option<Res<HorusSyncStats>>,
    dock_config: Option<Res<DockConfig>>,
    mut show_panel: Local<bool>,
) {
    // Skip if dock mode is enabled (dock renders its own stats tab)
    if let Some(dock) = dock_config {
        if dock.enabled {
            return;
        }
    }

    // Safely get context, return early if not initialized
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    // Toggle with 'S' key (handled elsewhere)
    if !*show_panel {
        *show_panel = true; // Default to showing
    }

    egui::Window::new("Statistics")
        .default_width(320.0)
        .default_pos([10.0, 100.0])
        .show(ctx, |ui| {
            ui.heading("Performance");
            ui.separator();

            // FPS and frame time
            let fps = if time.delta_secs() > 0.0 {
                1.0 / time.delta_secs()
            } else {
                0.0
            };

            ui.horizontal(|ui| {
                ui.label("FPS:");
                ui.label(format!("{:.1}", fps));

                // Color code FPS
                if fps >= 60.0 {
                    ui.colored_label(egui::Color32::GREEN, "[OK]");
                } else if fps >= 30.0 {
                    ui.colored_label(egui::Color32::YELLOW, "[WARNING]");
                } else {
                    ui.colored_label(egui::Color32::RED, "[FAIL]");
                }
            });

            ui.label(format!("Frame Time: {:.2}ms", time.delta_secs() * 1000.0));

            ui.add_space(5.0);
            ui.label("Frame Time Breakdown:");
            ui.indent("frame_breakdown", |ui| {
                ui.label(format!("  Physics: {:.2}ms", frame_time.physics_time_ms));
                ui.label(format!("  Sensors: {:.2}ms", frame_time.sensor_time_ms));
                ui.label(format!(
                    "  Rendering: {:.2}ms",
                    frame_time.rendering_time_ms
                ));
            });

            ui.add_space(10.0);
            ui.heading("Entities");
            ui.separator();

            ui.label(format!("Total Entities: {}", stats.total_entities));
            ui.label(format!("Robots: {}", stats.robot_count));
            ui.label(format!("Sensors: {}", stats.sensor_count));
            ui.label(format!("Obstacles: {}", stats.obstacle_count));

            ui.add_space(10.0);
            ui.heading("Physics");
            ui.separator();

            ui.label(format!("Rigid Bodies: {}", stats.rigid_body_count));
            ui.label(format!("Colliders: {}", stats.collider_count));
            ui.label(format!("Joints: {}", stats.joint_count));
            ui.label(format!("Active Contacts: {}", stats.contact_count));

            ui.add_space(10.0);
            ui.heading("Simulation");
            ui.separator();

            ui.label(format!("Sim Time: {:.2}s", stats.simulation_time));
            ui.label(format!("Real Time: {:.2}s", stats.real_time));

            let time_ratio = stats.time_ratio();
            ui.horizontal(|ui| {
                ui.label("Time Ratio:");
                let ratio_text = format!("{:.2}x", time_ratio);
                if time_ratio >= 0.95 {
                    ui.colored_label(egui::Color32::GREEN, ratio_text);
                } else if time_ratio >= 0.5 {
                    ui.colored_label(egui::Color32::YELLOW, ratio_text);
                } else {
                    ui.colored_label(egui::Color32::RED, ratio_text);
                }
            });

            ui.add_space(10.0);
            ui.heading("Memory");
            ui.separator();

            ui.label(format!("Est. Usage: {:.2} MB", stats.estimated_memory_mb()));

            // HORUS sync statistics
            if let Some(horus) = horus_stats {
                ui.add_space(10.0);
                ui.heading("HORUS Sync");
                ui.separator();

                ui.label(format!("Published: {}", horus.messages_published));
                ui.label(format!("Received: {}", horus.messages_received));
                ui.label(format!("Errors: {}", horus.publish_errors));
                ui.label(format!("Last Pub: {:.2}s", horus.last_publish_time));
                ui.label(format!("Last Recv: {:.2}s", horus.last_receive_time));
            }
        });
}

#[cfg(not(feature = "visual"))]
pub fn stats_panel_system() {}

#[cfg(not(feature = "visual"))]
pub fn collect_stats_system() {}

/// Plugin to register stats panel systems
pub struct StatsPanelPlugin;

impl Plugin for StatsPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimulationStats>()
            .init_resource::<FrameTimeBreakdown>()
            .add_systems(Update, collect_stats_system);

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                stats_panel_system
                    .after(collect_stats_system)
                    .after(EguiSet::InitContexts),
            );
        }

        #[cfg(not(feature = "visual"))]
        {
            app.add_systems(Update, stats_panel_system.after(collect_stats_system));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_stats() {
        let mut stats = SimulationStats::new();
        assert_eq!(stats.total_entities, 0);
        assert_eq!(stats.robot_count, 0);

        stats.total_entities = 100;
        stats.rigid_body_count = 50;
        stats.collider_count = 75;

        let mem = stats.estimated_memory_mb();
        assert!(mem > 0.0);
    }

    #[test]
    fn test_time_ratio() {
        let mut stats = SimulationStats::new();
        stats.simulation_time = 10.0;
        stats.real_time = 10.0;

        assert_eq!(stats.time_ratio(), 1.0);

        stats.real_time = 20.0;
        assert_eq!(stats.time_ratio(), 0.5); // Running at half speed
    }

    #[test]
    fn test_memory_estimation() {
        let mut stats = SimulationStats::new();
        stats.total_entities = 1000;
        stats.rigid_body_count = 500;
        stats.collider_count = 500;
        stats.joint_count = 100;

        let mem = stats.estimated_memory_mb();
        assert!(mem > 0.5); // Should be reasonable estimate
        assert!(mem < 100.0); // Shouldn't be crazy
    }
}
