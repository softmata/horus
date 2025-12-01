#[cfg(feature = "visual")]
use bevy::prelude::*;
#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

/// Resource to control simulation state
#[derive(Resource, Clone)]
pub struct SimulationControls {
    pub paused: bool,
    pub time_scale: f32,
    pub show_debug_info: bool,
    pub show_physics_debug: bool,
    pub show_sensor_rays: bool,
    pub show_tf_frames: bool,
    pub show_collision_shapes: bool,
}

impl Default for SimulationControls {
    fn default() -> Self {
        Self {
            paused: false,
            time_scale: 1.0,
            show_debug_info: true,
            show_physics_debug: false,
            show_sensor_rays: false,
            show_tf_frames: true,
            show_collision_shapes: false,
        }
    }
}

impl SimulationControls {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn toggle_pause(&mut self) {
        self.paused = !self.paused;
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn set_time_scale(&mut self, scale: f32) {
        self.time_scale = scale.clamp(0.0, 10.0);
    }
}

/// Events for simulation control
#[derive(Event, Clone, Debug)]
pub enum SimulationEvent {
    Reset,
    Pause,
    Resume,
    SetTimeScale(f32),
    TakeScreenshot,
    ResetCamera,
    SetCameraView(CameraView),
}

/// Predefined camera views
#[derive(Clone, Debug, PartialEq)]
pub enum CameraView {
    Front,
    Back,
    Left,
    Right,
    Top,
    Isometric,
}

impl CameraView {
    pub fn to_position(&self, target: Vec3, distance: f32) -> Vec3 {
        let offset = match self {
            CameraView::Front => Vec3::new(0.0, 0.0, distance),
            CameraView::Back => Vec3::new(0.0, 0.0, -distance),
            CameraView::Left => Vec3::new(-distance, 0.0, 0.0),
            CameraView::Right => Vec3::new(distance, 0.0, 0.0),
            CameraView::Top => Vec3::new(0.0, distance, 0.0),
            CameraView::Isometric => Vec3::new(distance * 0.7, distance * 0.7, distance * 0.7),
        };
        target + offset
    }
}

#[cfg(feature = "visual")]
use crate::ui::dock::DockConfig;

#[cfg(feature = "visual")]
/// Main controls panel system - only shown when dock mode is disabled
pub fn controls_panel_system(
    mut contexts: EguiContexts,
    mut controls: ResMut<SimulationControls>,
    mut events: EventWriter<SimulationEvent>,
    dock_config: Option<Res<DockConfig>>,
    mut show_panel: Local<bool>,
) {
    // Skip if dock mode is enabled (dock renders its own controls tab)
    if let Some(dock) = dock_config {
        if dock.enabled {
            return;
        }
    }

    if !*show_panel {
        *show_panel = true; // Default to showing
    }

    // Safely get context
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::Window::new("Controls")
        .default_width(280.0)
        .default_pos([10.0, 400.0])
        .show(ctx, |ui| {
            ui.heading("Simulation");
            ui.separator();

            // Pause/Resume button
            ui.horizontal(|ui| {
                let button_text = if controls.paused {
                    "▶ Resume"
                } else {
                    "⏸ Pause"
                };
                if ui.button(button_text).clicked() {
                    controls.toggle_pause();
                    if controls.paused {
                        events.send(SimulationEvent::Pause);
                    } else {
                        events.send(SimulationEvent::Resume);
                    }
                }

                if ui.button("↻ Reset").clicked() {
                    events.send(SimulationEvent::Reset);
                }
            });

            ui.add_space(5.0);

            // Time scale control
            ui.label("Time Scale:");
            let mut time_scale = controls.time_scale;
            ui.horizontal(|ui| {
                if ui.button("0.25x").clicked() {
                    time_scale = 0.25;
                }
                if ui.button("0.5x").clicked() {
                    time_scale = 0.5;
                }
                if ui.button("1x").clicked() {
                    time_scale = 1.0;
                }
                if ui.button("2x").clicked() {
                    time_scale = 2.0;
                }
                if ui.button("5x").clicked() {
                    time_scale = 5.0;
                }
            });

            ui.add(egui::Slider::new(&mut time_scale, 0.0..=10.0).text("Custom"));

            if time_scale != controls.time_scale {
                controls.set_time_scale(time_scale);
                events.send(SimulationEvent::SetTimeScale(time_scale));
            }

            ui.add_space(10.0);
            ui.heading("Camera");
            ui.separator();

            ui.label("View Presets:");
            ui.horizontal(|ui| {
                if ui.button("Front").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Front));
                }
                if ui.button("Back").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Back));
                }
                if ui.button("Left").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Left));
                }
            });

            ui.horizontal(|ui| {
                if ui.button("Right").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Right));
                }
                if ui.button("Top").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Top));
                }
                if ui.button("Iso").clicked() {
                    events.send(SimulationEvent::SetCameraView(CameraView::Isometric));
                }
            });

            ui.add_space(5.0);
            if ui.button("↺ Reset Camera").clicked() {
                events.send(SimulationEvent::ResetCamera);
            }

            ui.add_space(10.0);
            ui.heading("Visualization");
            ui.separator();

            ui.checkbox(&mut controls.show_debug_info, "Debug Info");
            ui.checkbox(&mut controls.show_physics_debug, "Physics Debug");
            ui.checkbox(&mut controls.show_sensor_rays, "Sensor Rays");
            ui.checkbox(&mut controls.show_tf_frames, "TF Frames");
            ui.checkbox(&mut controls.show_collision_shapes, "Collision Shapes");

            ui.add_space(10.0);
            ui.heading("Actions");
            ui.separator();

            if ui.button("Take Screenshot").clicked() {
                events.send(SimulationEvent::TakeScreenshot);
            }
        });
}

#[cfg(not(feature = "visual"))]
pub fn controls_panel_system() {}

/// System to handle simulation control events
pub fn handle_simulation_events(
    mut events: EventReader<SimulationEvent>,
    mut controls: ResMut<SimulationControls>,
    mut time: ResMut<Time<Virtual>>,
) {
    for event in events.read() {
        match event {
            SimulationEvent::Pause => {
                controls.paused = true;
                time.pause();
            }
            SimulationEvent::Resume => {
                controls.paused = false;
                time.unpause();
            }
            SimulationEvent::SetTimeScale(scale) => {
                controls.time_scale = *scale;
                time.set_relative_speed(*scale);
            }
            SimulationEvent::Reset => {
                // Reset handled by specific reset systems
                info!("Simulation reset requested");
            }
            SimulationEvent::TakeScreenshot => {
                // Screenshot handling would be done by a separate system
                info!("Screenshot requested");
            }
            SimulationEvent::ResetCamera => {
                // Camera reset handled by camera system
                info!("Camera reset requested");
            }
            SimulationEvent::SetCameraView(view) => {
                info!("Camera view changed to {:?}", view);
            }
        }
    }
}

/// Keyboard shortcuts for simulation controls
pub fn keyboard_controls_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut controls: ResMut<SimulationControls>,
    mut events: EventWriter<SimulationEvent>,
) {
    // Space: Toggle pause
    if keyboard.just_pressed(KeyCode::Space) {
        controls.toggle_pause();
        if controls.paused {
            events.send(SimulationEvent::Pause);
        } else {
            events.send(SimulationEvent::Resume);
        }
    }

    // R: Reset simulation
    if keyboard.just_pressed(KeyCode::KeyR) {
        events.send(SimulationEvent::Reset);
    }

    // Number keys for time scale
    if keyboard.just_pressed(KeyCode::Digit1) {
        controls.set_time_scale(0.25);
        events.send(SimulationEvent::SetTimeScale(0.25));
    }
    if keyboard.just_pressed(KeyCode::Digit2) {
        controls.set_time_scale(0.5);
        events.send(SimulationEvent::SetTimeScale(0.5));
    }
    if keyboard.just_pressed(KeyCode::Digit3) {
        controls.set_time_scale(1.0);
        events.send(SimulationEvent::SetTimeScale(1.0));
    }
    if keyboard.just_pressed(KeyCode::Digit4) {
        controls.set_time_scale(2.0);
        events.send(SimulationEvent::SetTimeScale(2.0));
    }
    if keyboard.just_pressed(KeyCode::Digit5) {
        controls.set_time_scale(5.0);
        events.send(SimulationEvent::SetTimeScale(5.0));
    }

    // F12: Take screenshot
    if keyboard.just_pressed(KeyCode::F12) {
        events.send(SimulationEvent::TakeScreenshot);
    }

    // Home: Reset camera
    if keyboard.just_pressed(KeyCode::Home) {
        events.send(SimulationEvent::ResetCamera);
    }

    // Toggle visualization options
    if keyboard.just_pressed(KeyCode::KeyD) {
        controls.show_debug_info = !controls.show_debug_info;
    }
    if keyboard.just_pressed(KeyCode::KeyP) {
        controls.show_physics_debug = !controls.show_physics_debug;
    }
    if keyboard.just_pressed(KeyCode::KeyS) {
        controls.show_sensor_rays = !controls.show_sensor_rays;
    }
    if keyboard.just_pressed(KeyCode::KeyT) {
        controls.show_tf_frames = !controls.show_tf_frames;
    }
    if keyboard.just_pressed(KeyCode::KeyC) {
        controls.show_collision_shapes = !controls.show_collision_shapes;
    }
}

/// Plugin to register control systems
pub struct ControlsPlugin;

impl Plugin for ControlsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimulationControls>()
            .add_event::<SimulationEvent>()
            .add_systems(
                Update,
                (keyboard_controls_system, handle_simulation_events).chain(),
            );

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                controls_panel_system.after(EguiSet::InitContexts),
            );
        }

        #[cfg(not(feature = "visual"))]
        {
            app.add_systems(Update, controls_panel_system);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_controls() {
        let mut controls = SimulationControls::new();
        assert!(!controls.paused);
        assert_eq!(controls.time_scale, 1.0);

        controls.toggle_pause();
        assert!(controls.paused);

        controls.set_time_scale(2.0);
        assert_eq!(controls.time_scale, 2.0);

        // Test clamping
        controls.set_time_scale(100.0);
        assert_eq!(controls.time_scale, 10.0); // Clamped to max

        controls.set_time_scale(-5.0);
        assert_eq!(controls.time_scale, 0.0); // Clamped to min
    }

    #[test]
    fn test_camera_views() {
        let target = Vec3::ZERO;
        let distance = 10.0;

        let front = CameraView::Front.to_position(target, distance);
        assert_eq!(front, Vec3::new(0.0, 0.0, 10.0));

        let top = CameraView::Top.to_position(target, distance);
        assert_eq!(top, Vec3::new(0.0, 10.0, 0.0));

        let iso = CameraView::Isometric.to_position(target, distance);
        assert!(iso.x > 0.0 && iso.y > 0.0 && iso.z > 0.0);
    }

    #[test]
    fn test_controls_reset() {
        let mut controls = SimulationControls::new();
        controls.paused = true;
        controls.time_scale = 5.0;
        controls.show_debug_info = false;

        controls.reset();

        assert!(!controls.paused);
        assert_eq!(controls.time_scale, 1.0);
        assert!(controls.show_debug_info);
    }
}
