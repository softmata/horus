#[cfg(feature = "visual")]
use bevy::prelude::*;
#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

#[cfg(feature = "visual")]
use crate::ui::dock::DockConfig;

/// Debug panel system - only shown when dock mode is disabled
/// When dock mode is enabled, the docked panels provide all the info
#[cfg(feature = "visual")]
pub fn debug_panel_system(
    mut contexts: EguiContexts,
    time: Res<Time>,
    dock_config: Option<Res<DockConfig>>,
) {
    // Skip if dock mode is enabled (dock provides unified UI)
    if let Some(config) = dock_config {
        if config.enabled {
            return;
        }
    }

    // Safely get context, return early if not initialized
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    egui::Window::new("Debug Panel").show(ctx, |ui| {
        ui.heading("sim3d - HORUS 3D Simulator");
        ui.separator();

        ui.label(format!("FPS: {:.1}", 1.0 / time.delta_secs()));
        ui.label(format!("Delta: {:.3}ms", time.delta_secs() * 1000.0));

        ui.separator();
        ui.label("Controls:");
        ui.label("  Right Mouse: Rotate camera");
        ui.label("  Middle Mouse: Pan camera");
        ui.label("  Mouse Wheel: Zoom");
        ui.label("  WASD/Arrows: Move camera focus");
        ui.label("  Q/E: Move focus up/down");
        ui.separator();
        ui.label("Press F7 to enable dock mode");
    });
}

#[cfg(not(feature = "visual"))]
pub fn debug_panel_system() {}
