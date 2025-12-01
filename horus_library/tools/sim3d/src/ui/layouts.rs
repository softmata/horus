//! Layout system for preset UI arrangements in sim3d.
//!
//! This module provides a flexible layout system that allows users to switch between
//! different UI arrangements optimized for various workflows like coding, debugging,
//! presentation, and minimal views.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

/// Layout presets for different workflows
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum LayoutPreset {
    /// Full inspector, hierarchy, and console for coding workflow
    #[default]
    Coding,
    /// Stats prominent, console visible, debug overlays for debugging
    Debugging,
    /// Clean viewport, minimal UI, large viewport for presentations
    Presentation,
    /// Just viewport and essential controls
    Minimal,
    /// User-defined custom layout
    Custom,
}

impl LayoutPreset {
    /// Get human-readable name for the preset
    pub fn name(&self) -> &'static str {
        match self {
            LayoutPreset::Coding => "Coding",
            LayoutPreset::Debugging => "Debugging",
            LayoutPreset::Presentation => "Presentation",
            LayoutPreset::Minimal => "Minimal",
            LayoutPreset::Custom => "Custom",
        }
    }

    /// Get description for the preset
    pub fn description(&self) -> &'static str {
        match self {
            LayoutPreset::Coding => "Full inspector, hierarchy, and console for development",
            LayoutPreset::Debugging => "Stats and debug overlays for troubleshooting",
            LayoutPreset::Presentation => "Clean viewport for demonstrations",
            LayoutPreset::Minimal => "Essential controls only",
            LayoutPreset::Custom => "User-defined layout",
        }
    }

    /// Get all available presets
    pub fn all() -> &'static [LayoutPreset] {
        &[
            LayoutPreset::Coding,
            LayoutPreset::Debugging,
            LayoutPreset::Presentation,
            LayoutPreset::Minimal,
            LayoutPreset::Custom,
        ]
    }

    /// Get hotkey for this layout preset
    pub fn hotkey(&self) -> Option<KeyCode> {
        match self {
            LayoutPreset::Coding => Some(KeyCode::F2),
            LayoutPreset::Debugging => Some(KeyCode::F3),
            LayoutPreset::Presentation => Some(KeyCode::F4),
            LayoutPreset::Minimal => Some(KeyCode::F5),
            LayoutPreset::Custom => Some(KeyCode::F6),
        }
    }
}

/// Position anchor for UI panels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum PanelAnchor {
    #[default]
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    TopCenter,
    BottomCenter,
    LeftCenter,
    RightCenter,
    Center,
}

impl PanelAnchor {
    /// Convert anchor to screen position
    pub fn to_position(
        &self,
        screen_width: f32,
        screen_height: f32,
        panel_width: f32,
        panel_height: f32,
    ) -> [f32; 2] {
        let margin = 10.0;
        match self {
            PanelAnchor::TopLeft => [margin, margin],
            PanelAnchor::TopRight => [screen_width - panel_width - margin, margin],
            PanelAnchor::BottomLeft => [margin, screen_height - panel_height - margin],
            PanelAnchor::BottomRight => [
                screen_width - panel_width - margin,
                screen_height - panel_height - margin,
            ],
            PanelAnchor::TopCenter => [(screen_width - panel_width) / 2.0, margin],
            PanelAnchor::BottomCenter => [
                (screen_width - panel_width) / 2.0,
                screen_height - panel_height - margin,
            ],
            PanelAnchor::LeftCenter => [margin, (screen_height - panel_height) / 2.0],
            PanelAnchor::RightCenter => [
                screen_width - panel_width - margin,
                (screen_height - panel_height) / 2.0,
            ],
            PanelAnchor::Center => [
                (screen_width - panel_width) / 2.0,
                (screen_height - panel_height) / 2.0,
            ],
        }
    }
}

/// Configuration for a single panel
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelConfig {
    /// Whether the panel is visible
    pub visible: bool,
    /// Anchor position for the panel
    pub anchor: PanelAnchor,
    /// Custom position offset from anchor (optional)
    pub position_offset: [f32; 2],
    /// Width of the panel (None for auto-width)
    pub width: Option<f32>,
    /// Height of the panel (None for auto-height)
    pub height: Option<f32>,
    /// Whether the panel is collapsible
    pub collapsible: bool,
    /// Whether the panel starts collapsed
    pub collapsed: bool,
    /// Opacity of the panel (0.0 - 1.0)
    pub opacity: f32,
}

impl Default for PanelConfig {
    fn default() -> Self {
        Self {
            visible: true,
            anchor: PanelAnchor::TopLeft,
            position_offset: [0.0, 0.0],
            width: None,
            height: None,
            collapsible: true,
            collapsed: false,
            opacity: 1.0,
        }
    }
}

impl PanelConfig {
    /// Create a new panel config with visibility
    pub fn new(visible: bool) -> Self {
        Self {
            visible,
            ..Default::default()
        }
    }

    /// Builder pattern: set visibility
    pub fn with_visibility(mut self, visible: bool) -> Self {
        self.visible = visible;
        self
    }

    /// Builder pattern: set anchor
    pub fn with_anchor(mut self, anchor: PanelAnchor) -> Self {
        self.anchor = anchor;
        self
    }

    /// Builder pattern: set position offset
    pub fn with_offset(mut self, x: f32, y: f32) -> Self {
        self.position_offset = [x, y];
        self
    }

    /// Builder pattern: set width
    pub fn with_width(mut self, width: f32) -> Self {
        self.width = Some(width);
        self
    }

    /// Builder pattern: set height
    pub fn with_height(mut self, height: f32) -> Self {
        self.height = Some(height);
        self
    }

    /// Builder pattern: set opacity
    pub fn with_opacity(mut self, opacity: f32) -> Self {
        self.opacity = opacity.clamp(0.0, 1.0);
        self
    }

    /// Builder pattern: set collapsible
    pub fn with_collapsible(mut self, collapsible: bool) -> Self {
        self.collapsible = collapsible;
        self
    }

    /// Builder pattern: set collapsed state
    pub fn with_collapsed(mut self, collapsed: bool) -> Self {
        self.collapsed = collapsed;
        self
    }

    /// Get the position for this panel
    pub fn get_position(&self, screen_width: f32, screen_height: f32) -> [f32; 2] {
        let panel_width = self.width.unwrap_or(280.0);
        let panel_height = self.height.unwrap_or(400.0);
        let base_pos =
            self.anchor
                .to_position(screen_width, screen_height, panel_width, panel_height);
        [
            base_pos[0] + self.position_offset[0],
            base_pos[1] + self.position_offset[1],
        ]
    }
}

/// Viewport configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ViewportConfig {
    /// Position of the viewport (x, y)
    pub position: [f32; 2],
    /// Size of the viewport (width, height) - None for fullscreen
    pub size: Option<[f32; 2]>,
    /// Margin from edges when other panels are visible
    pub margins: ViewportMargins,
}

impl Default for ViewportConfig {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0],
            size: None,
            margins: ViewportMargins::default(),
        }
    }
}

/// Margins for the viewport when panels are visible
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ViewportMargins {
    pub left: f32,
    pub right: f32,
    pub top: f32,
    pub bottom: f32,
}

impl Default for ViewportMargins {
    fn default() -> Self {
        Self {
            left: 0.0,
            right: 0.0,
            top: 0.0,
            bottom: 0.0,
        }
    }
}

impl ViewportMargins {
    pub fn new(left: f32, right: f32, top: f32, bottom: f32) -> Self {
        Self {
            left,
            right,
            top,
            bottom,
        }
    }

    /// Create uniform margins
    pub fn uniform(margin: f32) -> Self {
        Self {
            left: margin,
            right: margin,
            top: margin,
            bottom: margin,
        }
    }
}

/// Complete layout configuration
#[derive(Debug, Clone, Serialize, Deserialize, Resource)]
pub struct LayoutConfig {
    /// Name of the layout
    pub name: String,
    /// Description of the layout
    pub description: String,
    /// Inspector panel configuration
    pub inspector: PanelConfig,
    /// Hierarchy panel configuration
    pub hierarchy: PanelConfig,
    /// Console panel configuration
    pub console: PanelConfig,
    /// Stats panel configuration
    pub stats: PanelConfig,
    /// Controls panel configuration
    pub controls: PanelConfig,
    /// TF panel configuration
    pub tf_panel: PanelConfig,
    /// View modes panel configuration
    pub view_modes: PanelConfig,
    /// Viewport configuration
    pub viewport: ViewportConfig,
    /// Show debug overlays
    pub show_debug_overlays: bool,
    /// Show grid
    pub show_grid: bool,
    /// Show axes
    pub show_axes: bool,
}

impl Default for LayoutConfig {
    fn default() -> Self {
        Self::coding()
    }
}

impl LayoutConfig {
    /// Create a new layout configuration
    pub fn new(name: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            inspector: PanelConfig::default(),
            hierarchy: PanelConfig::default(),
            console: PanelConfig::default(),
            stats: PanelConfig::default(),
            controls: PanelConfig::default(),
            tf_panel: PanelConfig::default(),
            view_modes: PanelConfig::default(),
            viewport: ViewportConfig::default(),
            show_debug_overlays: false,
            show_grid: true,
            show_axes: true,
        }
    }

    /// Create Coding layout preset
    pub fn coding() -> Self {
        Self {
            name: "Coding".to_string(),
            description: "Full inspector, hierarchy, and console for development".to_string(),
            inspector: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopRight)
                .with_width(320.0),
            hierarchy: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopLeft)
                .with_width(280.0),
            console: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomLeft)
                .with_width(600.0)
                .with_height(200.0),
            stats: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopLeft)
                .with_offset(0.0, 300.0)
                .with_width(280.0),
            controls: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopRight)
                .with_offset(0.0, 400.0)
                .with_width(280.0),
            tf_panel: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomRight)
                .with_width(300.0),
            view_modes: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopCenter)
                .with_collapsed(true),
            viewport: ViewportConfig {
                position: [0.0, 0.0],
                size: None,
                margins: ViewportMargins::new(290.0, 330.0, 10.0, 210.0),
            },
            show_debug_overlays: false,
            show_grid: true,
            show_axes: true,
        }
    }

    /// Create Debugging layout preset
    pub fn debugging() -> Self {
        Self {
            name: "Debugging".to_string(),
            description: "Stats and debug overlays for troubleshooting".to_string(),
            inspector: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopRight)
                .with_width(350.0),
            hierarchy: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopLeft)
                .with_width(280.0),
            console: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomLeft)
                .with_width(800.0)
                .with_height(250.0),
            stats: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopLeft)
                .with_offset(290.0, 0.0)
                .with_width(320.0),
            controls: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomRight)
                .with_width(280.0),
            tf_panel: PanelConfig::new(true)
                .with_anchor(PanelAnchor::RightCenter)
                .with_width(320.0),
            view_modes: PanelConfig::new(false),
            viewport: ViewportConfig {
                position: [0.0, 0.0],
                size: None,
                margins: ViewportMargins::new(290.0, 360.0, 10.0, 260.0),
            },
            show_debug_overlays: true,
            show_grid: true,
            show_axes: true,
        }
    }

    /// Create Presentation layout preset
    pub fn presentation() -> Self {
        Self {
            name: "Presentation".to_string(),
            description: "Clean viewport for demonstrations".to_string(),
            inspector: PanelConfig::new(false),
            hierarchy: PanelConfig::new(false),
            console: PanelConfig::new(false),
            stats: PanelConfig::new(false),
            controls: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomRight)
                .with_width(200.0)
                .with_opacity(0.7)
                .with_collapsed(true),
            tf_panel: PanelConfig::new(false),
            view_modes: PanelConfig::new(false),
            viewport: ViewportConfig {
                position: [0.0, 0.0],
                size: None,
                margins: ViewportMargins::uniform(0.0),
            },
            show_debug_overlays: false,
            show_grid: false,
            show_axes: false,
        }
    }

    /// Create Minimal layout preset
    pub fn minimal() -> Self {
        Self {
            name: "Minimal".to_string(),
            description: "Essential controls only".to_string(),
            inspector: PanelConfig::new(false),
            hierarchy: PanelConfig::new(false),
            console: PanelConfig::new(false),
            stats: PanelConfig::new(true)
                .with_anchor(PanelAnchor::TopLeft)
                .with_width(200.0)
                .with_opacity(0.8)
                .with_collapsed(true),
            controls: PanelConfig::new(true)
                .with_anchor(PanelAnchor::BottomRight)
                .with_width(200.0)
                .with_opacity(0.8),
            tf_panel: PanelConfig::new(false),
            view_modes: PanelConfig::new(false),
            viewport: ViewportConfig {
                position: [0.0, 0.0],
                size: None,
                margins: ViewportMargins::uniform(10.0),
            },
            show_debug_overlays: false,
            show_grid: true,
            show_axes: true,
        }
    }

    /// Create Custom layout preset (starts as copy of Coding)
    pub fn custom() -> Self {
        let mut layout = Self::coding();
        layout.name = "Custom".to_string();
        layout.description = "User-defined layout".to_string();
        layout
    }

    /// Get layout for a preset
    pub fn from_preset(preset: LayoutPreset) -> Self {
        match preset {
            LayoutPreset::Coding => Self::coding(),
            LayoutPreset::Debugging => Self::debugging(),
            LayoutPreset::Presentation => Self::presentation(),
            LayoutPreset::Minimal => Self::minimal(),
            LayoutPreset::Custom => Self::custom(),
        }
    }

    /// Check if a specific panel is visible
    pub fn is_panel_visible(&self, panel_name: &str) -> bool {
        match panel_name {
            "inspector" => self.inspector.visible,
            "hierarchy" => self.hierarchy.visible,
            "console" => self.console.visible,
            "stats" => self.stats.visible,
            "controls" => self.controls.visible,
            "tf_panel" => self.tf_panel.visible,
            "view_modes" => self.view_modes.visible,
            _ => false,
        }
    }

    /// Toggle visibility of a specific panel
    pub fn toggle_panel(&mut self, panel_name: &str) {
        match panel_name {
            "inspector" => self.inspector.visible = !self.inspector.visible,
            "hierarchy" => self.hierarchy.visible = !self.hierarchy.visible,
            "console" => self.console.visible = !self.console.visible,
            "stats" => self.stats.visible = !self.stats.visible,
            "controls" => self.controls.visible = !self.controls.visible,
            "tf_panel" => self.tf_panel.visible = !self.tf_panel.visible,
            "view_modes" => self.view_modes.visible = !self.view_modes.visible,
            _ => {}
        }
    }
}

/// Layout manager resource for managing and switching layouts
#[derive(Resource)]
pub struct LayoutManager {
    /// Current active preset
    pub current_preset: LayoutPreset,
    /// Stored custom layouts
    pub custom_layouts: HashMap<String, LayoutConfig>,
    /// Whether layout changes should be animated
    pub animate_transitions: bool,
    /// Transition duration in seconds
    pub transition_duration: f32,
    /// Path to save custom layouts
    pub save_path: Option<PathBuf>,
    /// Current layout config (derived from preset or custom)
    current_config: LayoutConfig,
}

impl Default for LayoutManager {
    fn default() -> Self {
        Self {
            current_preset: LayoutPreset::Coding,
            custom_layouts: HashMap::new(),
            animate_transitions: true,
            transition_duration: 0.3,
            save_path: None,
            current_config: LayoutConfig::coding(),
        }
    }
}

impl LayoutManager {
    /// Create a new layout manager
    pub fn new() -> Self {
        Self::default()
    }

    /// Get the current layout configuration
    pub fn current_layout(&self) -> &LayoutConfig {
        &self.current_config
    }

    /// Get mutable reference to current layout configuration
    pub fn current_layout_mut(&mut self) -> &mut LayoutConfig {
        &mut self.current_config
    }

    /// Switch to a layout preset
    pub fn switch_to_preset(&mut self, preset: LayoutPreset) {
        self.current_preset = preset;
        self.current_config = if preset == LayoutPreset::Custom {
            self.custom_layouts
                .get("default")
                .cloned()
                .unwrap_or_else(LayoutConfig::custom)
        } else {
            LayoutConfig::from_preset(preset)
        };
    }

    /// Switch to a named custom layout
    pub fn switch_to_custom(&mut self, name: &str) -> bool {
        if let Some(layout) = self.custom_layouts.get(name) {
            self.current_preset = LayoutPreset::Custom;
            self.current_config = layout.clone();
            true
        } else {
            false
        }
    }

    /// Save current layout as a custom layout
    pub fn save_custom_layout(&mut self, name: impl Into<String>) {
        let name = name.into();
        let mut layout = self.current_config.clone();
        layout.name = name.clone();
        self.custom_layouts.insert(name, layout);
    }

    /// Delete a custom layout
    pub fn delete_custom_layout(&mut self, name: &str) -> bool {
        self.custom_layouts.remove(name).is_some()
    }

    /// Get list of custom layout names
    pub fn custom_layout_names(&self) -> Vec<&String> {
        self.custom_layouts.keys().collect()
    }

    /// Save custom layouts to file
    pub fn save_to_file(&self, path: &std::path::Path) -> Result<(), std::io::Error> {
        let json = serde_json::to_string_pretty(&self.custom_layouts)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, json)
    }

    /// Load custom layouts from file
    pub fn load_from_file(&mut self, path: &std::path::Path) -> Result<(), std::io::Error> {
        let json = std::fs::read_to_string(path)?;
        self.custom_layouts = serde_json::from_str(&json)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        Ok(())
    }

    /// Cycle to the next preset
    pub fn next_preset(&mut self) {
        let presets = LayoutPreset::all();
        let current_idx = presets
            .iter()
            .position(|p| *p == self.current_preset)
            .unwrap_or(0);
        let next_idx = (current_idx + 1) % presets.len();
        self.switch_to_preset(presets[next_idx]);
    }

    /// Cycle to the previous preset
    pub fn previous_preset(&mut self) {
        let presets = LayoutPreset::all();
        let current_idx = presets
            .iter()
            .position(|p| *p == self.current_preset)
            .unwrap_or(0);
        let prev_idx = if current_idx == 0 {
            presets.len() - 1
        } else {
            current_idx - 1
        };
        self.switch_to_preset(presets[prev_idx]);
    }
}

/// Event for layout changes
#[derive(Event, Clone, Debug)]
pub enum LayoutEvent {
    /// Switch to a preset layout
    SwitchPreset(LayoutPreset),
    /// Switch to a custom layout by name
    SwitchCustom(String),
    /// Save current layout as custom
    SaveCustom(String),
    /// Delete a custom layout
    DeleteCustom(String),
    /// Toggle a specific panel
    TogglePanel(String),
    /// Reset to default layout
    ResetLayout,
    /// Export layouts to file
    ExportLayouts(PathBuf),
    /// Import layouts from file
    ImportLayouts(PathBuf),
}

/// System to handle layout hotkeys
pub fn layout_hotkey_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut events: EventWriter<LayoutEvent>,
) {
    // Check for preset hotkeys
    for preset in LayoutPreset::all() {
        if let Some(hotkey) = preset.hotkey() {
            if keyboard.just_pressed(hotkey) {
                events.send(LayoutEvent::SwitchPreset(*preset));
            }
        }
    }

    // Tab: Cycle through presets (when holding Ctrl)
    if keyboard.pressed(KeyCode::ControlLeft) && keyboard.just_pressed(KeyCode::Tab) {
        events.send(LayoutEvent::SwitchPreset(LayoutPreset::Coding));
    }
}

/// System to handle layout events
pub fn handle_layout_events(
    mut events: EventReader<LayoutEvent>,
    mut manager: ResMut<LayoutManager>,
) {
    for event in events.read() {
        match event {
            LayoutEvent::SwitchPreset(preset) => {
                manager.switch_to_preset(*preset);
                info!("Switched to layout preset: {}", preset.name());
            }
            LayoutEvent::SwitchCustom(name) => {
                if manager.switch_to_custom(name) {
                    info!("Switched to custom layout: {}", name);
                } else {
                    warn!("Custom layout not found: {}", name);
                }
            }
            LayoutEvent::SaveCustom(name) => {
                manager.save_custom_layout(name.clone());
                info!("Saved custom layout: {}", name);
            }
            LayoutEvent::DeleteCustom(name) => {
                if manager.delete_custom_layout(name) {
                    info!("Deleted custom layout: {}", name);
                } else {
                    warn!("Custom layout not found: {}", name);
                }
            }
            LayoutEvent::TogglePanel(panel_name) => {
                manager.current_layout_mut().toggle_panel(panel_name);
                info!("Toggled panel: {}", panel_name);
            }
            LayoutEvent::ResetLayout => {
                let preset = manager.current_preset;
                manager.switch_to_preset(preset);
                info!("Reset layout to preset defaults");
            }
            LayoutEvent::ExportLayouts(path) => {
                if let Err(e) = manager.save_to_file(path) {
                    error!("Failed to export layouts: {}", e);
                } else {
                    info!("Exported layouts to: {:?}", path);
                }
            }
            LayoutEvent::ImportLayouts(path) => {
                if let Err(e) = manager.load_from_file(path) {
                    error!("Failed to import layouts: {}", e);
                } else {
                    info!("Imported layouts from: {:?}", path);
                }
            }
        }
    }
}

#[cfg(feature = "visual")]
use crate::ui::dock::DockConfig;

#[cfg(feature = "visual")]
/// UI panel for layout management - only shown when dock mode is disabled
pub fn layout_panel_system(
    mut contexts: EguiContexts,
    mut manager: ResMut<LayoutManager>,
    mut events: EventWriter<LayoutEvent>,
    mut save_name: Local<String>,
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

    egui::Window::new("Layouts")
        .default_pos([10.0, 500.0])
        .default_width(250.0)
        .show(ctx, |ui| {
            ui.heading("Layout Presets");
            ui.separator();

            // Preset buttons
            for preset in LayoutPreset::all() {
                let is_selected = manager.current_preset == *preset;
                let label = if let Some(hotkey) = preset.hotkey() {
                    format!("{} ({:?})", preset.name(), hotkey)
                } else {
                    preset.name().to_string()
                };

                if ui.selectable_label(is_selected, label).clicked() {
                    events.send(LayoutEvent::SwitchPreset(*preset));
                }
            }

            ui.separator();
            ui.heading("Custom Layouts");

            // List custom layouts
            let custom_names: Vec<String> = manager
                .custom_layout_names()
                .iter()
                .map(|s| (*s).clone())
                .collect();
            for name in custom_names {
                ui.horizontal(|ui| {
                    if ui.button(&name).clicked() {
                        events.send(LayoutEvent::SwitchCustom(name.clone()));
                    }
                    if ui.small_button("X").clicked() {
                        events.send(LayoutEvent::DeleteCustom(name));
                    }
                });
            }

            // Save new custom layout
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Name:");
                ui.text_edit_singleline(&mut *save_name);
            });
            if ui.button("Save Current Layout").clicked() && !save_name.is_empty() {
                events.send(LayoutEvent::SaveCustom(save_name.clone()));
                save_name.clear();
            }

            ui.separator();
            ui.heading("Panel Visibility");

            let layout = manager.current_layout_mut();

            ui.checkbox(&mut layout.inspector.visible, "Inspector");
            ui.checkbox(&mut layout.hierarchy.visible, "Hierarchy");
            ui.checkbox(&mut layout.console.visible, "Console");
            ui.checkbox(&mut layout.stats.visible, "Stats");
            ui.checkbox(&mut layout.controls.visible, "Controls");
            ui.checkbox(&mut layout.tf_panel.visible, "TF Panel");
            ui.checkbox(&mut layout.view_modes.visible, "View Modes");

            ui.separator();
            ui.heading("Display Options");

            ui.checkbox(&mut layout.show_debug_overlays, "Debug Overlays");
            ui.checkbox(&mut layout.show_grid, "Grid");
            ui.checkbox(&mut layout.show_axes, "Axes");

            ui.separator();
            if ui.button("Reset to Preset Defaults").clicked() {
                events.send(LayoutEvent::ResetLayout);
            }
        });
}

#[cfg(not(feature = "visual"))]
pub fn layout_panel_system() {}

/// Plugin to register layout systems
pub struct LayoutPlugin;

impl Plugin for LayoutPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LayoutManager>()
            .init_resource::<LayoutConfig>()
            .add_event::<LayoutEvent>()
            .add_systems(Update, (layout_hotkey_system, handle_layout_events).chain());

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                layout_panel_system.after(EguiSet::InitContexts),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layout_preset_names() {
        assert_eq!(LayoutPreset::Coding.name(), "Coding");
        assert_eq!(LayoutPreset::Debugging.name(), "Debugging");
        assert_eq!(LayoutPreset::Presentation.name(), "Presentation");
        assert_eq!(LayoutPreset::Minimal.name(), "Minimal");
        assert_eq!(LayoutPreset::Custom.name(), "Custom");
    }

    #[test]
    fn test_layout_preset_all() {
        let presets = LayoutPreset::all();
        assert_eq!(presets.len(), 5);
        assert!(presets.contains(&LayoutPreset::Coding));
        assert!(presets.contains(&LayoutPreset::Debugging));
        assert!(presets.contains(&LayoutPreset::Presentation));
        assert!(presets.contains(&LayoutPreset::Minimal));
        assert!(presets.contains(&LayoutPreset::Custom));
    }

    #[test]
    fn test_layout_preset_hotkeys() {
        assert_eq!(LayoutPreset::Coding.hotkey(), Some(KeyCode::F2));
        assert_eq!(LayoutPreset::Debugging.hotkey(), Some(KeyCode::F3));
        assert_eq!(LayoutPreset::Presentation.hotkey(), Some(KeyCode::F4));
        assert_eq!(LayoutPreset::Minimal.hotkey(), Some(KeyCode::F5));
        assert_eq!(LayoutPreset::Custom.hotkey(), Some(KeyCode::F6));
    }

    #[test]
    fn test_panel_anchor_position() {
        let anchor = PanelAnchor::TopLeft;
        let pos = anchor.to_position(1920.0, 1080.0, 280.0, 400.0);
        assert_eq!(pos, [10.0, 10.0]);

        let anchor = PanelAnchor::TopRight;
        let pos = anchor.to_position(1920.0, 1080.0, 280.0, 400.0);
        assert_eq!(pos, [1630.0, 10.0]);

        let anchor = PanelAnchor::BottomLeft;
        let pos = anchor.to_position(1920.0, 1080.0, 280.0, 400.0);
        assert_eq!(pos, [10.0, 670.0]);

        let anchor = PanelAnchor::Center;
        let pos = anchor.to_position(1920.0, 1080.0, 280.0, 400.0);
        assert_eq!(pos, [820.0, 340.0]);
    }

    #[test]
    fn test_panel_config_builder() {
        let config = PanelConfig::new(true)
            .with_anchor(PanelAnchor::TopRight)
            .with_width(300.0)
            .with_height(400.0)
            .with_opacity(0.8);

        assert!(config.visible);
        assert_eq!(config.anchor, PanelAnchor::TopRight);
        assert_eq!(config.width, Some(300.0));
        assert_eq!(config.height, Some(400.0));
        assert_eq!(config.opacity, 0.8);
    }

    #[test]
    fn test_panel_config_opacity_clamping() {
        let config = PanelConfig::new(true).with_opacity(1.5);
        assert_eq!(config.opacity, 1.0);

        let config = PanelConfig::new(true).with_opacity(-0.5);
        assert_eq!(config.opacity, 0.0);
    }

    #[test]
    fn test_layout_config_from_preset() {
        let coding = LayoutConfig::from_preset(LayoutPreset::Coding);
        assert_eq!(coding.name, "Coding");
        assert!(coding.inspector.visible);
        assert!(coding.hierarchy.visible);
        assert!(coding.console.visible);

        let presentation = LayoutConfig::from_preset(LayoutPreset::Presentation);
        assert_eq!(presentation.name, "Presentation");
        assert!(!presentation.inspector.visible);
        assert!(!presentation.hierarchy.visible);
        assert!(!presentation.console.visible);

        let minimal = LayoutConfig::from_preset(LayoutPreset::Minimal);
        assert_eq!(minimal.name, "Minimal");
        assert!(!minimal.inspector.visible);
        assert!(minimal.controls.visible);
    }

    #[test]
    fn test_layout_config_toggle_panel() {
        let mut config = LayoutConfig::coding();
        assert!(config.inspector.visible);

        config.toggle_panel("inspector");
        assert!(!config.inspector.visible);

        config.toggle_panel("inspector");
        assert!(config.inspector.visible);
    }

    #[test]
    fn test_layout_config_is_panel_visible() {
        let config = LayoutConfig::coding();
        assert!(config.is_panel_visible("inspector"));
        assert!(config.is_panel_visible("hierarchy"));
        assert!(config.is_panel_visible("console"));
        assert!(!config.is_panel_visible("unknown_panel"));
    }

    #[test]
    fn test_layout_manager_switch_preset() {
        let mut manager = LayoutManager::new();
        assert_eq!(manager.current_preset, LayoutPreset::Coding);

        manager.switch_to_preset(LayoutPreset::Debugging);
        assert_eq!(manager.current_preset, LayoutPreset::Debugging);
        assert_eq!(manager.current_layout().name, "Debugging");

        manager.switch_to_preset(LayoutPreset::Presentation);
        assert_eq!(manager.current_preset, LayoutPreset::Presentation);
        assert!(!manager.current_layout().inspector.visible);
    }

    #[test]
    fn test_layout_manager_custom_layouts() {
        let mut manager = LayoutManager::new();

        // Save current layout as custom
        manager.save_custom_layout("my_layout");
        assert!(manager.custom_layouts.contains_key("my_layout"));

        // Switch to custom layout
        assert!(manager.switch_to_custom("my_layout"));
        assert_eq!(manager.current_preset, LayoutPreset::Custom);

        // Delete custom layout
        assert!(manager.delete_custom_layout("my_layout"));
        assert!(!manager.custom_layouts.contains_key("my_layout"));
    }

    #[test]
    fn test_layout_manager_cycle_presets() {
        let mut manager = LayoutManager::new();
        manager.switch_to_preset(LayoutPreset::Coding);

        manager.next_preset();
        assert_eq!(manager.current_preset, LayoutPreset::Debugging);

        manager.next_preset();
        assert_eq!(manager.current_preset, LayoutPreset::Presentation);

        manager.previous_preset();
        assert_eq!(manager.current_preset, LayoutPreset::Debugging);
    }

    #[test]
    fn test_viewport_margins() {
        let margins = ViewportMargins::uniform(10.0);
        assert_eq!(margins.left, 10.0);
        assert_eq!(margins.right, 10.0);
        assert_eq!(margins.top, 10.0);
        assert_eq!(margins.bottom, 10.0);

        let margins = ViewportMargins::new(5.0, 10.0, 15.0, 20.0);
        assert_eq!(margins.left, 5.0);
        assert_eq!(margins.right, 10.0);
        assert_eq!(margins.top, 15.0);
        assert_eq!(margins.bottom, 20.0);
    }

    #[test]
    fn test_panel_config_get_position() {
        let config = PanelConfig::new(true)
            .with_anchor(PanelAnchor::TopLeft)
            .with_offset(20.0, 30.0)
            .with_width(280.0)
            .with_height(400.0);

        let pos = config.get_position(1920.0, 1080.0);
        assert_eq!(pos, [30.0, 40.0]); // 10 (margin) + 20 (offset), 10 + 30
    }

    #[test]
    fn test_debugging_layout_has_debug_overlays() {
        let debugging = LayoutConfig::debugging();
        assert!(debugging.show_debug_overlays);
        assert!(debugging.stats.visible);
        assert!(debugging.console.visible);
    }

    #[test]
    fn test_presentation_layout_is_clean() {
        let presentation = LayoutConfig::presentation();
        assert!(!presentation.show_debug_overlays);
        assert!(!presentation.show_grid);
        assert!(!presentation.show_axes);
        assert!(!presentation.inspector.visible);
        assert!(!presentation.hierarchy.visible);
    }

    #[test]
    fn test_layout_manager_switch_to_nonexistent_custom() {
        let mut manager = LayoutManager::new();
        assert!(!manager.switch_to_custom("nonexistent"));
        // Should stay on current preset
        assert_eq!(manager.current_preset, LayoutPreset::Coding);
    }

    #[test]
    fn test_layout_config_serialization() {
        let config = LayoutConfig::coding();
        let json = serde_json::to_string(&config).unwrap();
        let deserialized: LayoutConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.name, config.name);
        assert_eq!(deserialized.inspector.visible, config.inspector.visible);
    }
}
