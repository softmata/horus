//! Dockable Panel System using egui_dock
//!
//! Provides a unified dockable workspace similar to Gazebo and Isaac Sim.
//! Users can drag, dock, and arrange panels as needed.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use egui_dock::{DockArea, DockState, NodeIndex, Style, TabViewer};

use crate::hframe::HFrameTree;
use crate::physics::PhysicsWorld;
use crate::systems::hframe_update::HFramePublisher;
use crate::systems::horus_sync::HorusSyncStats;
use crate::ui::controls::{render_controls_ui, SimulationControls, SimulationEvent};
use crate::ui::hframe_panel::{render_hframe_ui, HFramePanelConfig};
use crate::ui::stats_panel::{render_stats_ui, FrameTimeBreakdown, SimulationStats};
use crate::ui::view_modes::{render_view_modes_ui, CurrentViewMode};

/// Tab identifiers for the dock system
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DockTab {
    /// Simulation controls
    Controls,
    /// Statistics and performance
    Stats,
    /// Console/Log output
    Console,
    /// HFrame (Transform Frame) tree
    HFrameTree,
    /// Camera view modes
    ViewModes,
    /// Custom plugin tab
    Plugin(String),
}

impl DockTab {
    pub fn title(&self) -> &str {
        match self {
            DockTab::Controls => "Controls",
            DockTab::Stats => "Statistics",
            DockTab::Console => "Console",
            DockTab::HFrameTree => "HFrame Tree",
            DockTab::ViewModes => "View Modes",
            DockTab::Plugin(name) => name.as_str(),
        }
    }

    pub fn closeable(&self) -> bool {
        true // All tabs can be closed
    }
}

/// Resource to control dock system behavior
#[derive(Resource)]
pub struct DockConfig {
    /// Whether the dock system is enabled (vs. floating windows)
    pub enabled: bool,
    /// Show the menu bar
    pub show_menu_bar: bool,
}

impl Default for DockConfig {
    fn default() -> Self {
        Self {
            enabled: true, // Enabled by default - unified dock panel on startup
            show_menu_bar: true,
        }
    }
}

/// Context for rendering dock tabs - holds references to world resources
pub struct DockRenderContext<'a> {
    pub time: &'a Time,
    pub stats: &'a SimulationStats,
    pub frame_time: &'a FrameTimeBreakdown,
    pub controls: &'a mut SimulationControls,
    pub hframe_tree: &'a HFrameTree,
    pub physics_world: Option<&'a PhysicsWorld>,
    pub horus_stats: Option<&'a HorusSyncStats>,
    pub hframe_publishers: Vec<&'a HFramePublisher>,
    pub hframe_panel_config: &'a mut HFramePanelConfig,
    pub view_mode: &'a mut CurrentViewMode,
    pub current_time: f32,
}

/// Tab viewer implementation for our dock system
pub struct SimDockViewer<'a> {
    pub ctx: DockRenderContext<'a>,
    /// Console log messages (stored separately for persistence)
    pub console_messages: &'a mut Vec<String>,
    /// Collected events from controls tab that need to be sent
    pub pending_events: Vec<SimulationEvent>,
}

impl TabViewer for SimDockViewer<'_> {
    type Tab = DockTab;

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        tab.title().into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        match tab {
            DockTab::Controls => {
                // Use the full controls UI from controls.rs
                let events = render_controls_ui(ui, self.ctx.controls);
                self.pending_events.extend(events.events);
            }
            DockTab::Stats => {
                // Use the full stats UI from stats_panel.rs
                render_stats_ui(
                    ui,
                    self.ctx.time,
                    self.ctx.stats,
                    self.ctx.frame_time,
                    self.ctx.horus_stats,
                );
            }
            DockTab::Console => {
                render_console_content(ui, self.console_messages);
            }
            DockTab::HFrameTree => {
                // Use the full HFrame tree UI from hframe_panel.rs
                render_hframe_ui(
                    ui,
                    self.ctx.hframe_tree,
                    self.ctx.hframe_panel_config,
                    &self.ctx.hframe_publishers,
                    self.ctx.current_time,
                );
            }
            DockTab::ViewModes => {
                // Use the full view modes UI from view_modes.rs
                render_view_modes_ui(ui, self.ctx.view_mode);
            }
            DockTab::Plugin(name) => {
                ui.heading(format!("Plugin: {}", name));
                ui.separator();
                ui.label("Plugin-specific content goes here");
            }
        }
    }

    fn closeable(&mut self, tab: &mut Self::Tab) -> bool {
        tab.closeable()
    }

    fn on_close(&mut self, _tab: &mut Self::Tab) -> bool {
        true // Allow closing
    }
}

// ============================================================================
// Tab Content Renderers (only for Console which has no external UI function)
// ============================================================================

fn render_console_content(ui: &mut egui::Ui, messages: &mut Vec<String>) {
    ui.heading("Console");
    ui.separator();

    // Add clear button
    ui.horizontal(|ui| {
        if ui.button("Clear").clicked() {
            messages.clear();
        }
        ui.label(format!("{} messages", messages.len()));
    });

    ui.separator();

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for msg in messages.iter() {
                // Color code by log level
                if msg.contains("[ERROR]") {
                    ui.colored_label(egui::Color32::RED, msg);
                } else if msg.contains("[WARN]") {
                    ui.colored_label(egui::Color32::YELLOW, msg);
                } else if msg.contains("[INFO]") {
                    ui.label(msg);
                } else {
                    ui.colored_label(egui::Color32::GRAY, msg);
                }
            }
        });
}

// ============================================================================
// Dock System Resources and Systems
// ============================================================================

/// Create a dark theme style for egui_dock (Catppuccin Mocha inspired)
fn create_dark_dock_style() -> Style {
    let mut style = Style::from_egui(&egui::Style::default());

    // Catppuccin Mocha colors
    let surface0 = egui::Color32::from_rgb(0x31, 0x32, 0x44); // Slightly lighter
    let surface1 = egui::Color32::from_rgb(0x45, 0x47, 0x5A); // Even lighter
    let text = egui::Color32::from_rgb(0xCD, 0xD6, 0xF4); // Light text
    let blue = egui::Color32::from_rgb(0x89, 0xB4, 0xFA); // Accent blue

    // Tab bar styling
    style.tab_bar.bg_fill = surface0;
    style.tab_bar.hline_color = surface1;

    // Active tab (Catppuccin blue accent)
    style.tab_bar.fill_tab_bar = true;
    style.tab_bar.height = 28.0;
    style.tab_bar.rounding = egui::Rounding::same(4.0);

    // Tab styling - use TabBarStyle fields
    // The tab_bar contains the tab visual settings

    // Dock area background
    style.dock_area_padding = Some(egui::Margin::same(0.0));

    // Separator styling
    style.separator.width = 2.0;
    style.separator.color_idle = surface1;
    style.separator.color_hovered = blue;
    style.separator.color_dragged = blue;

    // Tab body (content area) - use buttons field for tab styling
    style.buttons.close_tab_bg_fill = surface0;
    style.buttons.close_tab_color = text;
    style.buttons.close_tab_active_color = egui::Color32::from_rgb(0xF3, 0x8B, 0xA8); // Red on hover

    style
}

/// Dock state resource for persistence
#[derive(Resource)]
pub struct DockWorkspace {
    pub state: DockState<DockTab>,
    pub style: Style,
}

impl Default for DockWorkspace {
    fn default() -> Self {
        Self::new_default_layout()
    }
}

impl DockWorkspace {
    /// Create workspace with default layout
    pub fn new_default_layout() -> Self {
        // Start with Stats + Controls + ViewModes as tabs
        let mut state = DockState::new(vec![DockTab::Stats, DockTab::Controls, DockTab::ViewModes]);
        let tree = state.main_surface_mut();

        // Split: Bottom panel (Console + TfTree as tabs)
        let [_main, _bottom] = tree.split_below(
            NodeIndex::root(),
            0.65,
            vec![DockTab::Console, DockTab::HFrameTree],
        );

        Self {
            state,
            style: create_dark_dock_style(),
        }
    }

    /// Create a minimal layout (just stats)
    pub fn new_minimal_layout() -> Self {
        let state = DockState::new(vec![DockTab::Stats, DockTab::Controls]);

        Self {
            state,
            style: create_dark_dock_style(),
        }
    }

    /// Create a development layout (all panels)
    pub fn new_dev_layout() -> Self {
        // Left: Stats + Controls + ViewModes
        let mut state = DockState::new(vec![DockTab::Stats, DockTab::Controls, DockTab::ViewModes]);
        let tree = state.main_surface_mut();

        // Right: TF Tree
        let [_left, _right] = tree.split_right(NodeIndex::root(), 0.65, vec![DockTab::HFrameTree]);

        // Bottom: Console
        let [_main, _bottom] = tree.split_below(NodeIndex::root(), 0.7, vec![DockTab::Console]);

        Self {
            state,
            style: create_dark_dock_style(),
        }
    }

    /// Add a custom plugin tab
    pub fn add_plugin_tab(&mut self, plugin_name: String) {
        self.state
            .main_surface_mut()
            .push_to_focused_leaf(DockTab::Plugin(plugin_name));
    }

    /// Reset to default layout
    pub fn reset_layout(&mut self) {
        *self = Self::new_default_layout();
    }
}

/// Console messages storage
#[derive(Resource, Default)]
pub struct ConsoleMessages {
    pub messages: Vec<String>,
}

impl ConsoleMessages {
    pub fn add(&mut self, message: String) {
        self.messages.push(message);
        // Keep last 1000 messages
        if self.messages.len() > 1000 {
            self.messages.remove(0);
        }
    }

    pub fn info(&mut self, msg: &str) {
        self.add(format!("[INFO] {}", msg));
    }

    pub fn warn(&mut self, msg: &str) {
        self.add(format!("[WARN] {}", msg));
    }

    pub fn error(&mut self, msg: &str) {
        self.add(format!("[ERROR] {}", msg));
    }
}

/// Layout preset enum
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DockLayoutPreset {
    #[default]
    Default,
    Minimal,
    Development,
}

impl DockLayoutPreset {
    pub fn label(&self) -> &'static str {
        match self {
            DockLayoutPreset::Default => "Default",
            DockLayoutPreset::Minimal => "Minimal",
            DockLayoutPreset::Development => "Development",
        }
    }

    pub fn apply(&self) -> DockWorkspace {
        match self {
            DockLayoutPreset::Default => DockWorkspace::new_default_layout(),
            DockLayoutPreset::Minimal => DockWorkspace::new_minimal_layout(),
            DockLayoutPreset::Development => DockWorkspace::new_dev_layout(),
        }
    }
}

/// Event to change dock layout
#[derive(Event)]
pub struct ChangeDockLayoutEvent {
    pub preset: DockLayoutPreset,
}

/// Event to add a plugin tab
#[derive(Event)]
pub struct AddPluginTabEvent {
    pub plugin_name: String,
}

/// Event to toggle dock mode
#[derive(Event)]
pub struct ToggleDockModeEvent;

/// System to handle layout changes
pub fn handle_layout_change(
    mut events: EventReader<ChangeDockLayoutEvent>,
    mut workspace: ResMut<DockWorkspace>,
) {
    for event in events.read() {
        *workspace = event.preset.apply();
        tracing::info!("Dock layout changed to: {}", event.preset.label());
    }
}

/// System to handle adding plugin tabs
pub fn handle_add_plugin_tab(
    mut events: EventReader<AddPluginTabEvent>,
    mut workspace: ResMut<DockWorkspace>,
) {
    for event in events.read() {
        workspace.add_plugin_tab(event.plugin_name.clone());
        tracing::info!("Added plugin tab: {}", event.plugin_name);
    }
}

/// System to toggle dock mode
pub fn handle_toggle_dock_mode(
    mut events: EventReader<ToggleDockModeEvent>,
    mut config: ResMut<DockConfig>,
) {
    for _ in events.read() {
        config.enabled = !config.enabled;
        tracing::info!(
            "Dock mode: {}",
            if config.enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
    }
}

/// Keyboard shortcuts for dock system
pub fn dock_keyboard_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut layout_events: EventWriter<ChangeDockLayoutEvent>,
    mut toggle_events: EventWriter<ToggleDockModeEvent>,
) {
    // F7: Toggle dock mode
    if keyboard.just_pressed(KeyCode::F7) {
        toggle_events.send(ToggleDockModeEvent);
    }

    // F8: Reset to default layout
    if keyboard.just_pressed(KeyCode::F8) {
        layout_events.send(ChangeDockLayoutEvent {
            preset: DockLayoutPreset::Default,
        });
    }

    // F9: Minimal layout
    if keyboard.just_pressed(KeyCode::F9) {
        layout_events.send(ChangeDockLayoutEvent {
            preset: DockLayoutPreset::Minimal,
        });
    }

    // F10: Development layout
    if keyboard.just_pressed(KeyCode::F10) {
        layout_events.send(ChangeDockLayoutEvent {
            preset: DockLayoutPreset::Development,
        });
    }
}

/// Main dock UI rendering system
#[cfg(feature = "visual")]
pub fn dock_ui_system(
    mut contexts: EguiContexts,
    config: Res<DockConfig>,
    mut workspace: ResMut<DockWorkspace>,
    mut console: ResMut<ConsoleMessages>,
    time: Res<Time>,
    stats: Res<SimulationStats>,
    frame_time: Res<FrameTimeBreakdown>,
    mut controls: ResMut<SimulationControls>,
    hframe_tree: Res<HFrameTree>,
    physics_world: Option<Res<PhysicsWorld>>,
    horus_stats: Option<Res<HorusSyncStats>>,
    hframe_publishers: Query<&HFramePublisher>,
    mut hframe_panel_config: ResMut<HFramePanelConfig>,
    mut view_mode: ResMut<CurrentViewMode>,
    mut layout_events: EventWriter<ChangeDockLayoutEvent>,
    mut sim_events: EventWriter<SimulationEvent>,
) {
    if !config.enabled {
        return;
    }

    let egui_ctx = contexts.ctx_mut();

    // Menu bar
    if config.show_menu_bar {
        egui::TopBottomPanel::top("dock_menu_bar").show(egui_ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("View", |ui| {
                    if ui.button("Default Layout (F8)").clicked() {
                        layout_events.send(ChangeDockLayoutEvent {
                            preset: DockLayoutPreset::Default,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Minimal Layout (F9)").clicked() {
                        layout_events.send(ChangeDockLayoutEvent {
                            preset: DockLayoutPreset::Minimal,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Development Layout (F10)").clicked() {
                        layout_events.send(ChangeDockLayoutEvent {
                            preset: DockLayoutPreset::Development,
                        });
                        ui.close_menu();
                    }
                    ui.separator();
                    ui.label("F7: Toggle dock mode");
                });

                ui.menu_button("Panels", |ui| {
                    if ui.button("Add Controls").clicked() {
                        workspace
                            .state
                            .main_surface_mut()
                            .push_to_focused_leaf(DockTab::Controls);
                        ui.close_menu();
                    }
                    if ui.button("Add Statistics").clicked() {
                        workspace
                            .state
                            .main_surface_mut()
                            .push_to_focused_leaf(DockTab::Stats);
                        ui.close_menu();
                    }
                    if ui.button("Add Console").clicked() {
                        workspace
                            .state
                            .main_surface_mut()
                            .push_to_focused_leaf(DockTab::Console);
                        ui.close_menu();
                    }
                    if ui.button("Add HFrame Tree").clicked() {
                        workspace
                            .state
                            .main_surface_mut()
                            .push_to_focused_leaf(DockTab::HFrameTree);
                        ui.close_menu();
                    }
                    if ui.button("Add View Modes").clicked() {
                        workspace
                            .state
                            .main_surface_mut()
                            .push_to_focused_leaf(DockTab::ViewModes);
                        ui.close_menu();
                    }
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label("sim3d - Dockable UI");
                });
            });
        });
    }

    // Collect HFrame publishers
    let hframe_pubs: Vec<&HFramePublisher> = hframe_publishers.iter().collect();

    // Build render context
    let render_ctx = DockRenderContext {
        time: &time,
        stats: &stats,
        frame_time: &frame_time,
        controls: &mut controls,
        hframe_tree: &hframe_tree,
        physics_world: physics_world.as_deref(),
        horus_stats: horus_stats.as_deref(),
        hframe_publishers: hframe_pubs,
        hframe_panel_config: &mut hframe_panel_config,
        view_mode: &mut view_mode,
        current_time: time.elapsed_secs(),
    };

    // Create tab viewer
    let mut viewer = SimDockViewer {
        ctx: render_ctx,
        console_messages: &mut console.messages,
        pending_events: Vec::new(),
    };

    // Render dock area in a side panel (left side) to preserve viewport
    // Clone style first to avoid borrow conflict with mutable state borrow
    let style = workspace.style.clone();
    egui::SidePanel::left("dock_panel")
        .default_width(350.0)
        .min_width(200.0)
        .max_width(600.0)
        .resizable(true)
        .show(egui_ctx, |ui| {
            DockArea::new(&mut workspace.state)
                .style(style)
                .show_inside(ui, &mut viewer);
        });

    // Send any pending events from the controls UI
    for event in viewer.pending_events {
        sim_events.send(event);
    }
}

#[cfg(not(feature = "visual"))]
pub fn dock_ui_system() {}

/// Bevy plugin for the dock system
pub struct DockPlugin;

impl Plugin for DockPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DockConfig>()
            .init_resource::<DockWorkspace>()
            .init_resource::<ConsoleMessages>()
            .add_event::<ChangeDockLayoutEvent>()
            .add_event::<AddPluginTabEvent>()
            .add_event::<ToggleDockModeEvent>()
            .add_systems(
                Update,
                (
                    handle_layout_change,
                    handle_add_plugin_tab,
                    handle_toggle_dock_mode,
                    dock_keyboard_system,
                )
                    .chain(),
            );

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(Update, dock_ui_system.after(EguiSet::InitContexts));
        }

        #[cfg(not(feature = "visual"))]
        {
            app.add_systems(Update, dock_ui_system);
        }

        tracing::info!(
            "Dock system initialized (enabled by default) - Press F7 to toggle, F8-F10 for layouts"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dock_tab_titles() {
        assert_eq!(DockTab::Controls.title(), "Controls");
        assert_eq!(DockTab::Stats.title(), "Statistics");
        assert_eq!(DockTab::Plugin("Test".to_string()).title(), "Test");
    }

    #[test]
    fn test_dock_tab_closeable() {
        assert!(DockTab::Controls.closeable());
        assert!(DockTab::Stats.closeable());
    }

    #[test]
    fn test_workspace_default() {
        let workspace = DockWorkspace::default();
        assert!(!workspace.state.main_surface().is_empty());
    }

    #[test]
    fn test_layout_presets() {
        let _default = DockLayoutPreset::Default.apply();
        let _minimal = DockLayoutPreset::Minimal.apply();
        let _dev = DockLayoutPreset::Development.apply();
    }

    #[test]
    fn test_console_messages() {
        let mut console = ConsoleMessages::default();
        console.info("Test message");
        console.warn("Warning message");
        console.error("Error message");

        assert_eq!(console.messages.len(), 3);
        assert!(console.messages[0].contains("[INFO]"));
        assert!(console.messages[1].contains("[WARN]"));
        assert!(console.messages[2].contains("[ERROR]"));
    }
}
