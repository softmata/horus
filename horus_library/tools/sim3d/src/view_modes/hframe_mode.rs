use bevy::prelude::*;

use crate::systems::hframe_update::HFramePublisher;
use crate::ui::controls::SimulationControls;
use crate::ui::hframe_panel::HFramePanelConfig;

/// Resource to control HFrame visualization
#[derive(Resource, Clone)]
pub struct HFrameVisualization {
    pub enabled: bool,
    pub show_frame_axes: bool,
    pub show_frame_labels: bool,
    pub show_parent_links: bool,
    pub show_world_frame: bool,
    pub axis_length: f32,
    pub axis_thickness: f32,
    pub highlight_selected: bool,
}

impl Default for HFrameVisualization {
    fn default() -> Self {
        Self {
            enabled: true,
            show_frame_axes: true,
            show_frame_labels: false, // Text rendering would need additional setup
            show_parent_links: true,
            show_world_frame: true,
            axis_length: 0.2,
            axis_thickness: 0.01,
            highlight_selected: true,
        }
    }
}

impl HFrameVisualization {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    pub fn with_axis_length(mut self, length: f32) -> Self {
        self.axis_length = length;
        self
    }
}

/// System to visualize HFrame frames
pub fn hframe_visualization_system(
    mut gizmos: Gizmos,
    hframe_viz: Res<HFrameVisualization>,
    controls: Res<SimulationControls>,
    hframe_panel: Option<Res<HFramePanelConfig>>,
    publishers: Query<(&HFramePublisher, &Transform)>,
) {
    if !hframe_viz.enabled && !controls.show_tf_frames {
        return;
    }

    // Draw world frame
    if hframe_viz.show_world_frame {
        let world_axis_length = hframe_viz.axis_length * 2.0; // Larger for world frame

        // X axis - Red
        gizmos.arrow(
            Vec3::ZERO,
            Vec3::new(world_axis_length, 0.0, 0.0),
            Color::srgb(1.0, 0.0, 0.0),
        );

        // Y axis - Green
        gizmos.arrow(
            Vec3::ZERO,
            Vec3::new(0.0, world_axis_length, 0.0),
            Color::srgb(0.0, 1.0, 0.0),
        );

        // Z axis - Blue
        gizmos.arrow(
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, world_axis_length),
            Color::srgb(0.0, 0.0, 1.0),
        );
    }

    // Draw each HFrame frame
    for (publisher, transform) in publishers.iter() {
        let position = transform.translation;

        // Check if this frame is selected
        let is_selected = if let Some(panel) = &hframe_panel {
            hframe_viz.highlight_selected && panel.is_selected(&publisher.frame_name)
        } else {
            false
        };

        // Scale and brighten axes if selected
        let axis_length = if is_selected {
            hframe_viz.axis_length * 1.5
        } else {
            hframe_viz.axis_length
        };

        let alpha = if is_selected { 1.0 } else { 0.8 };

        // Draw frame axes
        if hframe_viz.show_frame_axes {
            // X axis - Red
            let x_axis = transform.rotation * Vec3::X * axis_length;
            gizmos.arrow(
                position,
                position + x_axis,
                Color::srgb(1.0, 0.0, 0.0).with_alpha(alpha),
            );

            // Y axis - Green
            let y_axis = transform.rotation * Vec3::Y * axis_length;
            gizmos.arrow(
                position,
                position + y_axis,
                Color::srgb(0.0, 1.0, 0.0).with_alpha(alpha),
            );

            // Z axis - Blue
            let z_axis = transform.rotation * Vec3::Z * axis_length;
            gizmos.arrow(
                position,
                position + z_axis,
                Color::srgb(0.0, 0.0, 1.0).with_alpha(alpha),
            );
        }

        // Draw selection indicator
        if is_selected {
            // Draw a sphere at the frame origin
            gizmos.sphere(
                Isometry3d::new(position, Quat::IDENTITY),
                hframe_viz.axis_length * 0.3,
                Color::srgb(1.0, 1.0, 0.0), // Yellow
            );
        }
    }
}

/// System to visualize parent-child links in HFrame tree
pub fn hframe_links_visualization_system(
    mut gizmos: Gizmos,
    hframe_viz: Res<HFrameVisualization>,
    publishers: Query<(&HFramePublisher, &Transform)>,
) {
    if !hframe_viz.enabled || !hframe_viz.show_parent_links {
        return;
    }

    // Build a map of frame names to positions
    let mut frame_positions: std::collections::HashMap<String, Vec3> =
        std::collections::HashMap::new();

    for (publisher, transform) in publishers.iter() {
        frame_positions.insert(publisher.frame_name.clone(), transform.translation);
    }

    // Draw links from each frame to its parent
    for (publisher, transform) in publishers.iter() {
        if publisher.parent_frame.is_empty() || publisher.parent_frame == "world" {
            // Link to world origin
            if hframe_viz.show_world_frame {
                gizmos.line(
                    Vec3::ZERO,
                    transform.translation,
                    Color::srgb(0.5, 0.5, 0.5).with_alpha(0.3),
                );
            }
        } else if let Some(&parent_pos) = frame_positions.get(&publisher.parent_frame) {
            // Link to parent frame
            gizmos.line(
                parent_pos,
                transform.translation,
                Color::srgb(0.3, 0.6, 0.9).with_alpha(0.5),
            );
        }
    }
}

/// System to visualize HFrame frame trails (history of positions)
#[derive(Component)]
pub struct HFrameTrail {
    pub frame_name: String,
    pub positions: Vec<Vec3>,
    pub max_length: usize,
    pub color: Color,
}

impl HFrameTrail {
    pub fn new(frame_name: impl Into<String>, max_length: usize) -> Self {
        Self {
            frame_name: frame_name.into(),
            positions: Vec::new(),
            max_length,
            color: Color::srgb(0.5, 0.5, 1.0),
        }
    }

    pub fn add_position(&mut self, position: Vec3) {
        self.positions.push(position);
        if self.positions.len() > self.max_length {
            self.positions.remove(0);
        }
    }
}

/// System to update and visualize HFrame trails
pub fn hframe_trail_system(
    mut gizmos: Gizmos,
    hframe_viz: Res<HFrameVisualization>,
    mut trails: Query<&mut HFrameTrail>,
    publishers: Query<(&HFramePublisher, &Transform)>,
) {
    if !hframe_viz.enabled {
        return;
    }

    // Update trails with current positions
    for mut trail in trails.iter_mut() {
        for (publisher, transform) in publishers.iter() {
            if publisher.frame_name == trail.frame_name {
                trail.add_position(transform.translation);
                break;
            }
        }

        // Draw the trail
        if trail.positions.len() >= 2 {
            for i in 0..trail.positions.len() - 1 {
                // Fade older positions
                let alpha = (i as f32 / trail.positions.len() as f32) * 0.8;
                gizmos.line(
                    trail.positions[i],
                    trail.positions[i + 1],
                    trail.color.with_alpha(alpha),
                );
            }
        }
    }
}

/// System to visualize HFrame transform chains
pub fn hframe_chain_visualization_system(
    mut gizmos: Gizmos,
    hframe_viz: Res<HFrameVisualization>,
    hframe_panel: Option<Res<HFramePanelConfig>>,
    publishers: Query<(&HFramePublisher, &Transform)>,
) {
    if !hframe_viz.enabled {
        return;
    }

    // If a frame is selected, highlight its entire chain to root
    if let Some(panel) = hframe_panel {
        if let Some(selected_frame) = &panel.selected_frame {
            // Build parent chain
            let mut current_frame = selected_frame.clone();
            let mut chain_positions = Vec::new();

            // Build frame map
            let mut frame_map: std::collections::HashMap<String, (String, Vec3)> =
                std::collections::HashMap::new();
            for (publisher, transform) in publishers.iter() {
                frame_map.insert(
                    publisher.frame_name.clone(),
                    (publisher.parent_frame.clone(), transform.translation),
                );
            }

            // Traverse up to root
            chain_positions.push(
                frame_map
                    .get(&current_frame)
                    .map(|(_, pos)| *pos)
                    .unwrap_or(Vec3::ZERO),
            );

            for _ in 0..20 {
                // Max depth limit
                if let Some((parent, pos)) = frame_map.get(&current_frame) {
                    if parent.is_empty() || parent == "world" {
                        chain_positions.push(Vec3::ZERO);
                        break;
                    }
                    current_frame = parent.clone();
                    chain_positions.push(*pos);
                } else {
                    break;
                }
            }

            // Draw highlighted chain
            for i in 0..chain_positions.len() - 1 {
                gizmos.line(
                    chain_positions[i],
                    chain_positions[i + 1],
                    Color::srgb(1.0, 1.0, 0.0).with_alpha(0.8), // Yellow
                );
            }
        }
    }
}

/// Keyboard shortcuts for HFrame visualization
pub fn hframe_viz_keyboard_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut hframe_viz: ResMut<HFrameVisualization>,
) {
    // T: Toggle HFrame visualization
    if keyboard.just_pressed(KeyCode::KeyT) && keyboard.pressed(KeyCode::ControlLeft) {
        hframe_viz.toggle();
    }

    // Shift+T: Toggle frame axes
    if keyboard.just_pressed(KeyCode::KeyT) && keyboard.pressed(KeyCode::ShiftLeft) {
        hframe_viz.show_frame_axes = !hframe_viz.show_frame_axes;
    }

    // Shift+L: Toggle parent links
    if keyboard.just_pressed(KeyCode::KeyL) && keyboard.pressed(KeyCode::ControlLeft) {
        hframe_viz.show_parent_links = !hframe_viz.show_parent_links;
    }

    // Shift+W: Toggle world frame
    if keyboard.just_pressed(KeyCode::KeyW) && keyboard.pressed(KeyCode::ControlLeft) {
        hframe_viz.show_world_frame = !hframe_viz.show_world_frame;
    }
}

/// Plugin to register HFrame visualization systems
pub struct HFrameVisualizationPlugin;

impl Plugin for HFrameVisualizationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HFrameVisualization>().add_systems(
            Update,
            (
                hframe_visualization_system,
                hframe_links_visualization_system,
                hframe_trail_system,
                hframe_chain_visualization_system,
                hframe_viz_keyboard_system,
            )
                .chain(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hframe_visualization() {
        let mut viz = HFrameVisualization::new();
        assert!(viz.enabled);

        viz.toggle();
        assert!(!viz.enabled);

        viz.disable();
        assert!(!viz.enabled);

        viz.enable();
        assert!(viz.enabled);
    }

    #[test]
    fn test_hframe_trail() {
        let mut trail = HFrameTrail::new("test_frame", 10);
        assert_eq!(trail.positions.len(), 0);

        for i in 0..15 {
            trail.add_position(Vec3::new(i as f32, 0.0, 0.0));
        }

        // Should be limited to max_length
        assert_eq!(trail.positions.len(), 10);
        assert_eq!(trail.positions[0], Vec3::new(5.0, 0.0, 0.0)); // Oldest kept
        assert_eq!(trail.positions[9], Vec3::new(14.0, 0.0, 0.0)); // Newest
    }

    #[test]
    fn test_default_settings() {
        let viz = HFrameVisualization::default();
        assert!(viz.show_frame_axes);
        assert!(viz.show_parent_links);
        assert!(viz.show_world_frame);
        assert_eq!(viz.axis_length, 0.2);
    }
}
