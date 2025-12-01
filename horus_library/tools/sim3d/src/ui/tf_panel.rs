#[cfg(feature = "visual")]
use bevy::prelude::*;
#[cfg(feature = "visual")]
use bevy_egui::{egui, EguiContexts};

#[cfg(feature = "visual")]
use crate::hframe::TFTree;
#[cfg(feature = "visual")]
use crate::systems::tf_update::TFPublisher;

/// Resource to control TF panel display options
#[derive(Resource, Clone)]
pub struct TFPanelConfig {
    pub show_panel: bool,
    pub show_world_frame: bool,
    pub show_robot_frames: bool,
    pub show_sensor_frames: bool,
    pub show_link_frames: bool,
    pub highlight_selected: bool,
    pub selected_frame: Option<String>,
}

impl Default for TFPanelConfig {
    fn default() -> Self {
        Self {
            show_panel: true,
            show_world_frame: true,
            show_robot_frames: true,
            show_sensor_frames: true,
            show_link_frames: true,
            highlight_selected: true,
            selected_frame: None,
        }
    }
}

impl TFPanelConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn select_frame(&mut self, frame_name: impl Into<String>) {
        self.selected_frame = Some(frame_name.into());
    }

    pub fn clear_selection(&mut self) {
        self.selected_frame = None;
    }

    pub fn is_selected(&self, frame_name: &str) -> bool {
        self.selected_frame
            .as_ref()
            .is_some_and(|f| f == frame_name)
    }
}

/// Statistics about TF frames
#[derive(Default)]
pub struct TFStats {
    pub total_frames: usize,
    pub world_frames: usize,
    pub robot_frames: usize,
    pub sensor_frames: usize,
    pub link_frames: usize,
}

impl TFStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, publishers: &Vec<&TFPublisher>) {
        self.total_frames = publishers.len();
        self.world_frames = 0;
        self.robot_frames = 0;
        self.sensor_frames = 0;
        self.link_frames = 0;

        for publisher in publishers {
            if publisher.frame_name.contains("world") || publisher.frame_name.contains("map") {
                self.world_frames += 1;
            } else if publisher.frame_name.contains("base")
                || publisher.frame_name.contains("robot")
            {
                self.robot_frames += 1;
            } else if publisher.frame_name.contains("sensor")
                || publisher.frame_name.contains("camera")
                || publisher.frame_name.contains("lidar")
            {
                self.sensor_frames += 1;
            } else if publisher.frame_name.contains("link")
                || publisher.frame_name.contains("joint")
            {
                self.link_frames += 1;
            }
        }
    }
}

#[cfg(feature = "visual")]
use crate::ui::dock::DockConfig;

#[cfg(feature = "visual")]
/// Main TF panel system - only shown when dock mode is disabled
pub fn tf_panel_system(
    mut contexts: EguiContexts,
    tf_tree: Res<TFTree>,
    mut config: ResMut<TFPanelConfig>,
    publishers: Query<&TFPublisher>,
    time: Res<Time>,
    dock_config: Option<Res<DockConfig>>,
) {
    // Skip if dock mode is enabled (dock renders its own TF tree tab)
    if let Some(dock) = dock_config {
        if dock.enabled {
            return;
        }
    }

    if !config.show_panel {
        return;
    }

    // Safely get context
    let Some(ctx) = contexts.try_ctx_mut() else {
        return;
    };

    // Collect all publishers
    let all_publishers: Vec<&TFPublisher> = publishers.iter().collect();
    let mut stats = TFStats::new();
    stats.update(&all_publishers);

    egui::Window::new("TF Tree")
        .default_width(320.0)
        .default_pos([10.0, 700.0])
        .resizable(true)
        .show(ctx, |ui| {
            ui.heading("Transform Frames");
            ui.separator();

            // Statistics
            ui.label(format!("Total Frames: {}", stats.total_frames));
            ui.label(format!(
                "World: {} | Robot: {} | Sensor: {} | Link: {}",
                stats.world_frames, stats.robot_frames, stats.sensor_frames, stats.link_frames
            ));

            ui.add_space(10.0);

            // Filter options
            ui.heading("Filters");
            ui.separator();
            ui.checkbox(&mut config.show_world_frame, "World Frames");
            ui.checkbox(&mut config.show_robot_frames, "Robot Frames");
            ui.checkbox(&mut config.show_sensor_frames, "Sensor Frames");
            ui.checkbox(&mut config.show_link_frames, "Link Frames");
            ui.checkbox(&mut config.highlight_selected, "Highlight Selected");

            ui.add_space(10.0);

            // Frame tree
            ui.heading("Frame Hierarchy");
            ui.separator();

            egui::ScrollArea::vertical()
                .max_height(400.0)
                .show(ui, |ui| {
                    // Group frames by type
                    let mut world_frames = Vec::new();
                    let mut robot_frames = Vec::new();
                    let mut sensor_frames = Vec::new();
                    let mut link_frames = Vec::new();
                    let mut other_frames = Vec::new();

                    for publisher in &all_publishers {
                        let frame_name = &publisher.frame_name;
                        if frame_name.contains("world") || frame_name.contains("map") {
                            world_frames.push(publisher);
                        } else if frame_name.contains("base") || frame_name.contains("robot") {
                            robot_frames.push(publisher);
                        } else if frame_name.contains("sensor")
                            || frame_name.contains("camera")
                            || frame_name.contains("lidar")
                        {
                            sensor_frames.push(publisher);
                        } else if frame_name.contains("link") || frame_name.contains("joint") {
                            link_frames.push(publisher);
                        } else {
                            other_frames.push(publisher);
                        }
                    }

                    // Display world frames
                    if config.show_world_frame && !world_frames.is_empty() {
                        ui.collapsing("World Frames", |ui| {
                            for publisher in world_frames {
                                display_frame_item(
                                    ui,
                                    publisher,
                                    &config,
                                    &tf_tree,
                                    time.elapsed_secs(),
                                );
                            }
                        });
                    }

                    // Display robot frames
                    if config.show_robot_frames && !robot_frames.is_empty() {
                        ui.collapsing("Robot Frames", |ui| {
                            for publisher in robot_frames {
                                display_frame_item(
                                    ui,
                                    publisher,
                                    &config,
                                    &tf_tree,
                                    time.elapsed_secs(),
                                );
                            }
                        });
                    }

                    // Display sensor frames
                    if config.show_sensor_frames && !sensor_frames.is_empty() {
                        ui.collapsing("Sensor Frames", |ui| {
                            for publisher in sensor_frames {
                                display_frame_item(
                                    ui,
                                    publisher,
                                    &config,
                                    &tf_tree,
                                    time.elapsed_secs(),
                                );
                            }
                        });
                    }

                    // Display link frames
                    if config.show_link_frames && !link_frames.is_empty() {
                        ui.collapsing("Link Frames", |ui| {
                            for publisher in link_frames {
                                display_frame_item(
                                    ui,
                                    publisher,
                                    &config,
                                    &tf_tree,
                                    time.elapsed_secs(),
                                );
                            }
                        });
                    }

                    // Display other frames
                    if !other_frames.is_empty() {
                        ui.collapsing("Other Frames", |ui| {
                            for publisher in other_frames {
                                display_frame_item(
                                    ui,
                                    publisher,
                                    &config,
                                    &tf_tree,
                                    time.elapsed_secs(),
                                );
                            }
                        });
                    }
                });

            ui.add_space(10.0);

            // Selected frame details
            if let Some(selected) = &config.selected_frame {
                ui.heading("Selected Frame");
                ui.separator();

                ui.label(format!("Frame: {}", selected));

                // Try to get transform from TF tree
                if let Ok(transform) = tf_tree.lookup_transform("world", selected) {
                    ui.label(format!(
                        "Position: ({:.3}, {:.3}, {:.3})",
                        transform.translation.x, transform.translation.y, transform.translation.z
                    ));

                    let (roll, pitch, yaw) = transform.rotation.euler_angles();
                    ui.label(format!(
                        "Rotation: R:{:.2}° P:{:.2}° Y:{:.2}°",
                        roll.to_degrees(),
                        pitch.to_degrees(),
                        yaw.to_degrees()
                    ));
                } else {
                    ui.label("Transform not available");
                }

                if ui.button("Clear Selection").clicked() {
                    config.clear_selection();
                }
            }
        });
}

#[cfg(feature = "visual")]
/// Helper function to display a single frame item
fn display_frame_item(
    ui: &mut egui::Ui,
    publisher: &TFPublisher,
    config: &TFPanelConfig,
    tf_tree: &TFTree,
    current_time: f32,
) {
    let is_selected = config.is_selected(&publisher.frame_name);

    ui.horizontal(|ui| {
        // Highlight selected frame
        if is_selected && config.highlight_selected {
            ui.visuals_mut().override_text_color = Some(egui::Color32::YELLOW);
        }

        // Frame name (clickable)
        if ui
            .selectable_label(is_selected, &publisher.frame_name)
            .clicked()
        {
            // Selection is handled by the mut config, but we can't modify it here
            // This would need to be refactored to send an event
        }

        // Parent frame
        ui.label(format!("← {}", publisher.parent_frame));

        // Update rate indicator
        if publisher.rate_hz > 0.0 {
            let time_since_update = current_time - publisher.last_update;
            let update_period = 1.0 / publisher.rate_hz;

            if time_since_update > update_period * 2.0 {
                ui.colored_label(egui::Color32::RED, "[WARNING]");
            } else if time_since_update > update_period * 1.5 {
                ui.colored_label(egui::Color32::YELLOW, "[WARNING]");
            } else {
                ui.colored_label(egui::Color32::GREEN, "[OK]");
            }
        }
    });

    // Show transform info on hover
    ui.horizontal(|ui| {
        if let Ok(transform) = tf_tree.lookup_transform("world", &publisher.frame_name) {
            ui.small(format!(
                "  pos: ({:.2}, {:.2}, {:.2})",
                transform.translation.x, transform.translation.y, transform.translation.z
            ));
        }
    });
}

#[cfg(not(feature = "visual"))]
pub fn tf_panel_system() {}

/// Event for TF frame selection
#[derive(Event, Clone, Debug)]
pub struct TFFrameSelectEvent {
    pub frame_name: String,
}

/// System to handle TF frame selection events
pub fn handle_tf_select_events(
    mut events: EventReader<TFFrameSelectEvent>,
    mut config: ResMut<TFPanelConfig>,
) {
    for event in events.read() {
        config.select_frame(event.frame_name.clone());
    }
}

/// Keyboard shortcuts for TF panel
pub fn tf_panel_keyboard_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut config: ResMut<TFPanelConfig>,
) {
    // T: Toggle TF panel
    if keyboard.just_pressed(KeyCode::KeyT) {
        config.show_panel = !config.show_panel;
    }

    // Escape: Clear selection
    if keyboard.just_pressed(KeyCode::Escape) {
        config.clear_selection();
    }
}

/// Plugin to register TF panel systems
pub struct TFPanelPlugin;

impl Plugin for TFPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TFPanelConfig>()
            .add_event::<TFFrameSelectEvent>()
            .add_systems(
                Update,
                (tf_panel_keyboard_system, handle_tf_select_events).chain(),
            );

        #[cfg(feature = "visual")]
        {
            use bevy_egui::EguiSet;
            app.add_systems(
                Update,
                tf_panel_system.after(EguiSet::InitContexts),
            );
        }

        #[cfg(not(feature = "visual"))]
        {
            app.add_systems(Update, tf_panel_system);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tf_panel_config() {
        let mut config = TFPanelConfig::new();
        assert!(config.show_panel);
        assert!(config.show_world_frame);
        assert!(config.selected_frame.is_none());

        config.select_frame("robot_base");
        assert_eq!(config.selected_frame, Some("robot_base".to_string()));
        assert!(config.is_selected("robot_base"));
        assert!(!config.is_selected("world"));

        config.clear_selection();
        assert!(config.selected_frame.is_none());
    }

    #[test]
    fn test_tf_stats() {
        let mut stats = TFStats::new();
        assert_eq!(stats.total_frames, 0);

        // Create some mock publishers
        let publishers = vec![
            TFPublisher::new("world", ""),
            TFPublisher::new("robot_base", "world"),
            TFPublisher::new("sensor_lidar", "robot_base"),
            TFPublisher::new("link_1", "robot_base"),
        ];

        let refs: Vec<&TFPublisher> = publishers.iter().collect();
        stats.update(&refs);

        assert_eq!(stats.total_frames, 4);
        assert_eq!(stats.world_frames, 1);
        assert_eq!(stats.robot_frames, 1);
        assert_eq!(stats.sensor_frames, 1);
        assert_eq!(stats.link_frames, 1);
    }
}
