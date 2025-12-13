//! Sensor Control Panel
//!
//! Provides runtime controls for enabling/disabling sensors, adjusting noise
//! parameters, update rates, and visualization options.

use bevy::prelude::*;
use bevy_egui::egui;
use std::collections::HashMap;

/// Sensor types supported by the panel
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorType {
    Camera,
    DepthCamera,
    RGBDCamera,
    ThermalCamera,
    Lidar3D,
    Lidar2D,
    IMU,
    GPS,
    ForceTorque,
    Tactile,
    Sonar,
    Radar,
    Encoder,
    EventCamera,
}

impl SensorType {
    pub fn name(&self) -> &'static str {
        match self {
            SensorType::Camera => "RGB Camera",
            SensorType::DepthCamera => "Depth Camera",
            SensorType::RGBDCamera => "RGB-D Camera",
            SensorType::ThermalCamera => "Thermal Camera",
            SensorType::Lidar3D => "3D LiDAR",
            SensorType::Lidar2D => "2D LiDAR",
            SensorType::IMU => "IMU",
            SensorType::GPS => "GPS",
            SensorType::ForceTorque => "Force/Torque",
            SensorType::Tactile => "Tactile",
            SensorType::Sonar => "Sonar",
            SensorType::Radar => "Radar",
            SensorType::Encoder => "Encoder",
            SensorType::EventCamera => "Event Camera",
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            SensorType::Camera => "📷",
            SensorType::DepthCamera => "📐",
            SensorType::RGBDCamera => "🎥",
            SensorType::ThermalCamera => "🌡",
            SensorType::Lidar3D => "📡",
            SensorType::Lidar2D => "〰",
            SensorType::IMU => "🧭",
            SensorType::GPS => "🛰",
            SensorType::ForceTorque => "💪",
            SensorType::Tactile => "✋",
            SensorType::Sonar => "🔊",
            SensorType::Radar => "📻",
            SensorType::Encoder => "🔄",
            SensorType::EventCamera => "⚡",
        }
    }

    pub fn all() -> &'static [SensorType] {
        &[
            SensorType::Camera,
            SensorType::DepthCamera,
            SensorType::RGBDCamera,
            SensorType::ThermalCamera,
            SensorType::Lidar3D,
            SensorType::Lidar2D,
            SensorType::IMU,
            SensorType::GPS,
            SensorType::ForceTorque,
            SensorType::Tactile,
            SensorType::Sonar,
            SensorType::Radar,
            SensorType::Encoder,
            SensorType::EventCamera,
        ]
    }
}

/// Configuration for a single sensor instance
#[derive(Debug, Clone)]
pub struct SensorConfig {
    pub enabled: bool,
    pub update_rate_hz: f32,
    pub noise_enabled: bool,
    pub noise_stddev: f32,
    pub visualize: bool,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            update_rate_hz: 30.0,
            noise_enabled: true,
            noise_stddev: 0.01,
            visualize: false,
        }
    }
}

/// Sensor panel configuration resource
#[derive(Resource)]
pub struct SensorPanelConfig {
    /// Show advanced noise parameters
    pub show_advanced: bool,
    /// Filter to show only specific sensor types
    pub type_filter: Option<SensorType>,
    /// Expand all sensors in tree view
    pub expand_all: bool,
}

impl Default for SensorPanelConfig {
    fn default() -> Self {
        Self {
            show_advanced: false,
            type_filter: None,
            expand_all: false,
        }
    }
}

/// Global sensor settings resource
#[derive(Resource)]
pub struct SensorSettings {
    /// Per-sensor-type default configurations
    pub type_defaults: HashMap<SensorType, SensorConfig>,
    /// Global enable/disable all sensors
    pub global_enabled: bool,
    /// Global visualization toggle
    pub global_visualize: bool,
    /// Global noise enable
    pub global_noise: bool,
}

impl Default for SensorSettings {
    fn default() -> Self {
        let mut type_defaults = HashMap::new();
        for sensor_type in SensorType::all() {
            let mut config = SensorConfig::default();
            // Set type-specific defaults
            match sensor_type {
                SensorType::Camera | SensorType::DepthCamera | SensorType::RGBDCamera => {
                    config.update_rate_hz = 30.0;
                }
                SensorType::ThermalCamera => {
                    config.update_rate_hz = 10.0;
                }
                SensorType::Lidar3D | SensorType::Lidar2D => {
                    config.update_rate_hz = 20.0;
                    config.noise_stddev = 0.02;
                }
                SensorType::IMU => {
                    config.update_rate_hz = 200.0;
                    config.noise_stddev = 0.001;
                }
                SensorType::GPS => {
                    config.update_rate_hz = 10.0;
                    config.noise_stddev = 0.5; // meters
                }
                SensorType::ForceTorque | SensorType::Tactile => {
                    config.update_rate_hz = 100.0;
                }
                SensorType::Encoder => {
                    config.update_rate_hz = 100.0;
                    config.noise_stddev = 0.0;
                }
                _ => {}
            }
            type_defaults.insert(*sensor_type, config);
        }
        Self {
            type_defaults,
            global_enabled: true,
            global_visualize: false,
            global_noise: true,
        }
    }
}

/// Render the sensor control panel UI
pub fn render_sensor_panel_ui(
    ui: &mut egui::Ui,
    settings: &mut SensorSettings,
    config: &mut SensorPanelConfig,
) {
    ui.heading("📡 Sensors");
    ui.separator();

    // Global controls
    ui.horizontal(|ui| {
        if ui.checkbox(&mut settings.global_enabled, "All Enabled").changed() {
            for cfg in settings.type_defaults.values_mut() {
                cfg.enabled = settings.global_enabled;
            }
        }
        if ui.checkbox(&mut settings.global_visualize, "Visualize").changed() {
            for cfg in settings.type_defaults.values_mut() {
                cfg.visualize = settings.global_visualize;
            }
        }
        if ui.checkbox(&mut settings.global_noise, "Noise").changed() {
            for cfg in settings.type_defaults.values_mut() {
                cfg.noise_enabled = settings.global_noise;
            }
        }
    });

    ui.separator();

    // Type filter
    ui.horizontal(|ui| {
        ui.label("Filter:");
        egui::ComboBox::from_id_salt("sensor_filter")
            .selected_text(config.type_filter.map_or("All", |t| t.name()))
            .show_ui(ui, |ui| {
                if ui.selectable_label(config.type_filter.is_none(), "All").clicked() {
                    config.type_filter = None;
                }
                ui.separator();
                for sensor_type in SensorType::all() {
                    let label = format!("{} {}", sensor_type.icon(), sensor_type.name());
                    if ui.selectable_label(config.type_filter == Some(*sensor_type), label).clicked() {
                        config.type_filter = Some(*sensor_type);
                    }
                }
            });
    });

    ui.separator();

    // Sensor list
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for sensor_type in SensorType::all() {
                // Apply filter
                if let Some(filter) = config.type_filter {
                    if *sensor_type != filter {
                        continue;
                    }
                }

                let cfg = settings.type_defaults.get_mut(sensor_type).unwrap();
                let header = format!("{} {}", sensor_type.icon(), sensor_type.name());

                egui::CollapsingHeader::new(header)
                    .default_open(config.expand_all)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut cfg.enabled, "Enabled");
                            ui.checkbox(&mut cfg.visualize, "Visualize");
                        });

                        ui.horizontal(|ui| {
                            ui.label("Rate:");
                            ui.add(egui::DragValue::new(&mut cfg.update_rate_hz)
                                .range(1.0..=1000.0)
                                .speed(1.0)
                                .suffix(" Hz"));
                        });

                        ui.horizontal(|ui| {
                            ui.checkbox(&mut cfg.noise_enabled, "Noise");
                            if cfg.noise_enabled {
                                ui.add(egui::DragValue::new(&mut cfg.noise_stddev)
                                    .range(0.0..=10.0)
                                    .speed(0.001)
                                    .prefix("σ="));
                            }
                        });

                        if config.show_advanced {
                            render_advanced_sensor_options(ui, *sensor_type, cfg);
                        }
                    });
            }
        });

    // Options
    ui.separator();
    ui.horizontal(|ui| {
        ui.checkbox(&mut config.show_advanced, "Advanced");
        ui.checkbox(&mut config.expand_all, "Expand All");
    });
}

fn render_advanced_sensor_options(ui: &mut egui::Ui, sensor_type: SensorType, _cfg: &mut SensorConfig) {
    ui.separator();
    ui.label("Advanced options:");

    match sensor_type {
        SensorType::Camera | SensorType::DepthCamera | SensorType::RGBDCamera => {
            ui.label("Resolution: (read-only)");
            ui.label("FOV: (read-only)");
        }
        SensorType::Lidar3D | SensorType::Lidar2D => {
            ui.label("Beams: (read-only)");
            ui.label("Range: (read-only)");
        }
        SensorType::IMU => {
            ui.label("Accelerometer bias: (read-only)");
            ui.label("Gyroscope bias: (read-only)");
        }
        SensorType::GPS => {
            ui.label("Horizontal accuracy: (read-only)");
            ui.label("Vertical accuracy: (read-only)");
        }
        _ => {
            ui.label("No advanced options");
        }
    }
}

/// Sensor panel plugin
pub struct SensorPanelPlugin;

impl Plugin for SensorPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SensorPanelConfig>()
            .init_resource::<SensorSettings>();
    }
}
