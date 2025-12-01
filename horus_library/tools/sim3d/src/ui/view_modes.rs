use bevy::prelude::*;

#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

use crate::rendering::camera_controller::OrbitCamera;

/// Different view modes for the camera
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ViewMode {
    /// Free orbit camera (default)
    #[default]
    Orbit,
    /// Top-down orthographic view
    TopDown,
    /// Front view
    Front,
    /// Side view
    Side,
    /// Follow a specific entity
    Follow,
    /// First-person view
    FirstPerson,
}

impl ViewMode {
    /// Get human-readable name
    pub fn name(&self) -> &'static str {
        match self {
            ViewMode::Orbit => "Orbit",
            ViewMode::TopDown => "Top-Down",
            ViewMode::Front => "Front",
            ViewMode::Side => "Side",
            ViewMode::Follow => "Follow",
            ViewMode::FirstPerson => "First Person",
        }
    }

    /// Get all available view modes
    pub fn all() -> &'static [ViewMode] {
        &[
            ViewMode::Orbit,
            ViewMode::TopDown,
            ViewMode::Front,
            ViewMode::Side,
            ViewMode::Follow,
            ViewMode::FirstPerson,
        ]
    }

    /// Get hotkey for this view mode
    pub fn hotkey(&self) -> KeyCode {
        match self {
            ViewMode::Orbit => KeyCode::Digit1,
            ViewMode::TopDown => KeyCode::Digit2,
            ViewMode::Front => KeyCode::Digit3,
            ViewMode::Side => KeyCode::Digit4,
            ViewMode::Follow => KeyCode::Digit5,
            ViewMode::FirstPerson => KeyCode::Digit6,
        }
    }
}

/// Resource to track current view mode
#[derive(Resource, Default)]
pub struct CurrentViewMode {
    pub mode: ViewMode,
    pub follow_entity: Option<Entity>,
}

impl CurrentViewMode {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the current view mode
    pub fn set_mode(&mut self, mode: ViewMode) {
        self.mode = mode;
    }

    /// Get the current view mode
    pub fn get_mode(&self) -> ViewMode {
        self.mode
    }

    /// Set entity to follow in Follow mode
    pub fn set_follow_entity(&mut self, entity: Entity) {
        self.follow_entity = Some(entity);
        self.mode = ViewMode::Follow;
    }

    /// Clear follow entity
    pub fn clear_follow_entity(&mut self) {
        self.follow_entity = None;
    }
}

/// System to handle view mode hotkeys
pub fn view_mode_hotkey_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut current_mode: ResMut<CurrentViewMode>,
) {
    for mode in ViewMode::all() {
        if keyboard.just_pressed(mode.hotkey()) {
            current_mode.set_mode(*mode);
            info!("Switched to {} view mode", mode.name());
        }
    }
}

/// System to apply view mode transformations to camera
pub fn apply_view_mode_system(
    current_mode: Res<CurrentViewMode>,
    mut camera_query: Query<(&mut Transform, &mut OrbitCamera), With<Camera>>,
    follow_query: Query<&GlobalTransform>,
) {
    if !current_mode.is_changed() {
        return;
    }

    for (mut transform, mut orbit) in camera_query.iter_mut() {
        match current_mode.mode {
            ViewMode::Orbit => {
                // Default orbit mode - no changes needed
                orbit.radius = 10.0;
                orbit.yaw = 0.0;
                orbit.pitch = 0.3;
            }
            ViewMode::TopDown => {
                // Top-down view
                orbit.radius = 15.0;
                orbit.yaw = 0.0;
                orbit.pitch = std::f32::consts::PI / 2.0 - 0.1;
                transform.translation = orbit.focus + Vec3::new(0.0, orbit.radius, 0.0);
                transform.look_at(orbit.focus, Vec3::Z);
            }
            ViewMode::Front => {
                // Front view
                orbit.radius = 10.0;
                orbit.yaw = 0.0;
                orbit.pitch = 0.0;
                transform.translation = orbit.focus + Vec3::new(0.0, 0.0, orbit.radius);
                transform.look_at(orbit.focus, Vec3::Y);
            }
            ViewMode::Side => {
                // Side view
                orbit.radius = 10.0;
                orbit.yaw = std::f32::consts::PI / 2.0;
                orbit.pitch = 0.0;
                transform.translation = orbit.focus + Vec3::new(orbit.radius, 0.0, 0.0);
                transform.look_at(orbit.focus, Vec3::Y);
            }
            ViewMode::Follow => {
                // Follow entity mode
                if let Some(entity) = current_mode.follow_entity {
                    if let Ok(target_transform) = follow_query.get(entity) {
                        orbit.focus = target_transform.translation();
                        orbit.radius = 5.0;
                        orbit.pitch = 0.3;
                    }
                }
            }
            ViewMode::FirstPerson => {
                // First person view (close to focus point)
                orbit.radius = 0.1;
                orbit.pitch = 0.0;
            }
        }
    }
}

/// System to update follow camera continuously
pub fn follow_mode_update_system(
    current_mode: Res<CurrentViewMode>,
    mut camera_query: Query<&mut OrbitCamera, With<Camera>>,
    follow_query: Query<&GlobalTransform>,
) {
    if current_mode.mode != ViewMode::Follow {
        return;
    }

    if let Some(entity) = current_mode.follow_entity {
        if let Ok(target_transform) = follow_query.get(entity) {
            for mut orbit in camera_query.iter_mut() {
                orbit.focus = target_transform.translation();
            }
        }
    }
}

/// UI panel for view mode selection - only shown when dock mode is disabled
#[cfg(feature = "visual")]
pub fn view_mode_panel_system(
    mut contexts: EguiContexts,
    mut current_mode: ResMut<CurrentViewMode>,
    dock_config: Option<Res<crate::ui::dock::DockConfig>>,
) {
    // Skip if dock mode is enabled
    if let Some(dock) = dock_config {
        if dock.enabled {
            return;
        }
    }

    // Safely get context
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::Window::new("View Modes")
        .default_pos([10.0, 300.0])
        .show(ctx, |ui| {
            ui.heading("Camera View Modes");
            ui.separator();

            for mode in ViewMode::all() {
                let is_selected = current_mode.mode == *mode;
                let hotkey_text = format!("{} ({:?})", mode.name(), mode.hotkey());

                if ui.selectable_label(is_selected, hotkey_text).clicked() {
                    current_mode.set_mode(*mode);
                }
            }

            ui.separator();
            ui.label("Hotkeys:");
            for mode in ViewMode::all() {
                ui.label(format!("  {:?} - {}", mode.hotkey(), mode.name()));
            }

            ui.separator();
            ui.label(format!("Current: {}", current_mode.mode.name()));
        });
}

#[cfg(not(feature = "visual"))]
pub fn view_mode_panel_system() {}

/// Plugin to register view mode systems
pub struct ViewModePlugin;

impl Plugin for ViewModePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CurrentViewMode>().add_systems(
            Update,
            (
                view_mode_hotkey_system,
                apply_view_mode_system,
                follow_mode_update_system,
            ),
        );

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                view_mode_panel_system.after(EguiSet::InitContexts),
            );
        }
    }
}

/// Utility functions for view mode management
pub struct ViewModeUtils;

impl ViewModeUtils {
    /// Calculate camera position for a specific view mode
    pub fn calculate_camera_position(mode: ViewMode, focus: Vec3, radius: f32) -> Vec3 {
        match mode {
            ViewMode::Orbit => {
                let yaw = Quat::from_rotation_y(0.0);
                let pitch = Quat::from_rotation_x(0.3);
                focus + (yaw * pitch) * Vec3::new(0.0, 0.0, radius)
            }
            ViewMode::TopDown => focus + Vec3::new(0.0, radius, 0.0),
            ViewMode::Front => focus + Vec3::new(0.0, 0.0, radius),
            ViewMode::Side => focus + Vec3::new(radius, 0.0, 0.0),
            ViewMode::Follow => focus + Vec3::new(0.0, 2.0, -5.0),
            ViewMode::FirstPerson => focus,
        }
    }

    /// Get camera up vector for a view mode
    pub fn get_up_vector(mode: ViewMode) -> Vec3 {
        match mode {
            ViewMode::TopDown => Vec3::Z,
            _ => Vec3::Y,
        }
    }

    /// Check if a view mode allows manual camera control
    pub fn allows_manual_control(mode: ViewMode) -> bool {
        matches!(mode, ViewMode::Orbit | ViewMode::Follow)
    }

    /// Get recommended radius for a view mode
    pub fn get_recommended_radius(mode: ViewMode) -> f32 {
        match mode {
            ViewMode::Orbit => 10.0,
            ViewMode::TopDown => 15.0,
            ViewMode::Front | ViewMode::Side => 10.0,
            ViewMode::Follow => 5.0,
            ViewMode::FirstPerson => 0.1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_view_mode_names() {
        assert_eq!(ViewMode::Orbit.name(), "Orbit");
        assert_eq!(ViewMode::TopDown.name(), "Top-Down");
        assert_eq!(ViewMode::Front.name(), "Front");
    }

    #[test]
    fn test_view_mode_all() {
        let modes = ViewMode::all();
        assert_eq!(modes.len(), 6);
        assert!(modes.contains(&ViewMode::Orbit));
        assert!(modes.contains(&ViewMode::TopDown));
    }

    #[test]
    fn test_current_view_mode() {
        let mut current = CurrentViewMode::new();
        assert_eq!(current.get_mode(), ViewMode::Orbit);

        current.set_mode(ViewMode::TopDown);
        assert_eq!(current.get_mode(), ViewMode::TopDown);
    }

    #[test]
    fn test_view_mode_hotkeys() {
        assert_eq!(ViewMode::Orbit.hotkey(), KeyCode::Digit1);
        assert_eq!(ViewMode::TopDown.hotkey(), KeyCode::Digit2);
        assert_eq!(ViewMode::Front.hotkey(), KeyCode::Digit3);
    }

    #[test]
    fn test_view_mode_utils_radius() {
        assert_eq!(ViewModeUtils::get_recommended_radius(ViewMode::Orbit), 10.0);
        assert_eq!(
            ViewModeUtils::get_recommended_radius(ViewMode::TopDown),
            15.0
        );
        assert_eq!(ViewModeUtils::get_recommended_radius(ViewMode::Follow), 5.0);
    }

    #[test]
    fn test_view_mode_utils_manual_control() {
        assert!(ViewModeUtils::allows_manual_control(ViewMode::Orbit));
        assert!(ViewModeUtils::allows_manual_control(ViewMode::Follow));
        assert!(!ViewModeUtils::allows_manual_control(ViewMode::TopDown));
        assert!(!ViewModeUtils::allows_manual_control(ViewMode::Front));
    }

    #[test]
    fn test_view_mode_utils_up_vector() {
        assert_eq!(ViewModeUtils::get_up_vector(ViewMode::Orbit), Vec3::Y);
        assert_eq!(ViewModeUtils::get_up_vector(ViewMode::TopDown), Vec3::Z);
        assert_eq!(ViewModeUtils::get_up_vector(ViewMode::Front), Vec3::Y);
    }

    #[test]
    fn test_follow_entity() {
        let mut current = CurrentViewMode::new();
        assert!(current.follow_entity.is_none());

        let entity = Entity::from_raw(42);
        current.set_follow_entity(entity);
        assert_eq!(current.follow_entity, Some(entity));
        assert_eq!(current.get_mode(), ViewMode::Follow);

        current.clear_follow_entity();
        assert!(current.follow_entity.is_none());
    }
}
