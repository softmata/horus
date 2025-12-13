//! Recording Control Panel
//!
//! Provides runtime controls for recording simulation data including
//! video capture, trajectory recording, sensor data logging, and playback.

use bevy::prelude::*;
use bevy_egui::egui;
use std::path::PathBuf;

/// Recording state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RecordingState {
    #[default]
    Idle,
    Recording,
    Paused,
    Playing,
}

impl RecordingState {
    pub fn icon(&self) -> &'static str {
        match self {
            RecordingState::Idle => "⏹",
            RecordingState::Recording => "🔴",
            RecordingState::Paused => "⏸",
            RecordingState::Playing => "▶",
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            RecordingState::Idle => "Idle",
            RecordingState::Recording => "Recording",
            RecordingState::Paused => "Paused",
            RecordingState::Playing => "Playing",
        }
    }
}

/// Video format options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VideoFormat {
    #[default]
    MP4,
    WebM,
    GIF,
    ImageSequence,
}

impl VideoFormat {
    pub fn name(&self) -> &'static str {
        match self {
            VideoFormat::MP4 => "MP4 (H.264)",
            VideoFormat::WebM => "WebM (VP9)",
            VideoFormat::GIF => "GIF",
            VideoFormat::ImageSequence => "Image Sequence (PNG)",
        }
    }

    pub fn extension(&self) -> &'static str {
        match self {
            VideoFormat::MP4 => "mp4",
            VideoFormat::WebM => "webm",
            VideoFormat::GIF => "gif",
            VideoFormat::ImageSequence => "png",
        }
    }

    pub fn all() -> &'static [VideoFormat] {
        &[VideoFormat::MP4, VideoFormat::WebM, VideoFormat::GIF, VideoFormat::ImageSequence]
    }
}

/// Data export format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataFormat {
    #[default]
    MCAP,
    CSV,
    JSON,
    ROS2Bag,
}

impl DataFormat {
    pub fn name(&self) -> &'static str {
        match self {
            DataFormat::MCAP => "MCAP",
            DataFormat::CSV => "CSV",
            DataFormat::JSON => "JSON",
            DataFormat::ROS2Bag => "ROS2 Bag",
        }
    }

    pub fn all() -> &'static [DataFormat] {
        &[DataFormat::MCAP, DataFormat::CSV, DataFormat::JSON, DataFormat::ROS2Bag]
    }
}

/// Recording settings resource
#[derive(Resource)]
pub struct RecordingSettings {
    /// Current recording state
    pub state: RecordingState,

    // Video settings
    pub video_enabled: bool,
    pub video_format: VideoFormat,
    pub video_fps: u32,
    pub video_resolution: [u32; 2],
    pub video_quality: u32, // 1-100

    // Trajectory recording
    pub trajectory_enabled: bool,
    pub trajectory_rate_hz: f32,
    pub record_transforms: bool,
    pub record_velocities: bool,
    pub record_forces: bool,

    // Sensor data recording
    pub sensor_data_enabled: bool,
    pub data_format: DataFormat,
    pub record_cameras: bool,
    pub record_lidar: bool,
    pub record_imu: bool,
    pub record_other_sensors: bool,

    // Output settings
    pub output_directory: PathBuf,
    pub auto_timestamp: bool,
    pub prefix: String,

    // Playback settings
    pub playback_speed: f32,
    pub loop_playback: bool,

    // Statistics
    pub frames_recorded: u64,
    pub data_size_bytes: u64,
    pub recording_duration_secs: f32,
}

impl Default for RecordingSettings {
    fn default() -> Self {
        Self {
            state: RecordingState::Idle,
            video_enabled: true,
            video_format: VideoFormat::MP4,
            video_fps: 30,
            video_resolution: [1920, 1080],
            video_quality: 80,
            trajectory_enabled: true,
            trajectory_rate_hz: 100.0,
            record_transforms: true,
            record_velocities: true,
            record_forces: false,
            sensor_data_enabled: false,
            data_format: DataFormat::MCAP,
            record_cameras: true,
            record_lidar: true,
            record_imu: true,
            record_other_sensors: false,
            output_directory: PathBuf::from("recordings"),
            auto_timestamp: true,
            prefix: "sim3d".to_string(),
            playback_speed: 1.0,
            loop_playback: false,
            frames_recorded: 0,
            data_size_bytes: 0,
            recording_duration_secs: 0.0,
        }
    }
}

/// Recording panel configuration
#[derive(Resource, Default)]
pub struct RecordingPanelConfig {
    pub show_advanced: bool,
    pub show_playback: bool,
}

/// Events for recording control
#[derive(Event)]
pub enum RecordingEvent {
    StartRecording,
    StopRecording,
    PauseRecording,
    ResumeRecording,
    TakeScreenshot,
    StartPlayback { file: PathBuf },
    StopPlayback,
}

/// Render the recording control panel UI
pub fn render_recording_panel_ui(
    ui: &mut egui::Ui,
    settings: &mut RecordingSettings,
    config: &mut RecordingPanelConfig,
) -> Vec<RecordingEvent> {
    let mut events = Vec::new();

    ui.heading("🎬 Recording");
    ui.separator();

    // Status display
    ui.horizontal(|ui| {
        ui.label("Status:");
        let color = match settings.state {
            RecordingState::Idle => egui::Color32::GRAY,
            RecordingState::Recording => egui::Color32::RED,
            RecordingState::Paused => egui::Color32::YELLOW,
            RecordingState::Playing => egui::Color32::GREEN,
        };
        ui.colored_label(color, format!("{} {}", settings.state.icon(), settings.state.label()));
    });

    // Main controls
    ui.horizontal(|ui| {
        match settings.state {
            RecordingState::Idle => {
                if ui.button("🔴 Record").clicked() {
                    events.push(RecordingEvent::StartRecording);
                    settings.state = RecordingState::Recording;
                    settings.frames_recorded = 0;
                    settings.data_size_bytes = 0;
                    settings.recording_duration_secs = 0.0;
                }
            }
            RecordingState::Recording => {
                if ui.button("⏸ Pause").clicked() {
                    events.push(RecordingEvent::PauseRecording);
                    settings.state = RecordingState::Paused;
                }
                if ui.button("⏹ Stop").clicked() {
                    events.push(RecordingEvent::StopRecording);
                    settings.state = RecordingState::Idle;
                }
            }
            RecordingState::Paused => {
                if ui.button("▶ Resume").clicked() {
                    events.push(RecordingEvent::ResumeRecording);
                    settings.state = RecordingState::Recording;
                }
                if ui.button("⏹ Stop").clicked() {
                    events.push(RecordingEvent::StopRecording);
                    settings.state = RecordingState::Idle;
                }
            }
            RecordingState::Playing => {
                if ui.button("⏹ Stop").clicked() {
                    events.push(RecordingEvent::StopPlayback);
                    settings.state = RecordingState::Idle;
                }
            }
        }

        if ui.button("📷 Screenshot").clicked() {
            events.push(RecordingEvent::TakeScreenshot);
        }
    });

    // Recording stats (when recording or just finished)
    if settings.state == RecordingState::Recording || settings.state == RecordingState::Paused {
        ui.separator();
        ui.label(format!("Frames: {}", settings.frames_recorded));
        ui.label(format!("Duration: {:.1}s", settings.recording_duration_secs));
        ui.label(format!("Size: {}", format_bytes(settings.data_size_bytes)));
    }

    ui.separator();

    // Video settings
    ui.collapsing("🎥 Video", |ui| {
        ui.checkbox(&mut settings.video_enabled, "Enable video recording");

        if settings.video_enabled {
            ui.horizontal(|ui| {
                ui.label("Format:");
                egui::ComboBox::from_id_salt("video_format")
                    .selected_text(settings.video_format.name())
                    .show_ui(ui, |ui| {
                        for format in VideoFormat::all() {
                            if ui.selectable_label(settings.video_format == *format, format.name()).clicked() {
                                settings.video_format = *format;
                            }
                        }
                    });
            });

            ui.horizontal(|ui| {
                ui.label("FPS:");
                ui.add(egui::DragValue::new(&mut settings.video_fps).range(1..=120));
            });

            ui.horizontal(|ui| {
                ui.label("Resolution:");
                ui.add(egui::DragValue::new(&mut settings.video_resolution[0]).prefix("W:").range(320..=3840));
                ui.add(egui::DragValue::new(&mut settings.video_resolution[1]).prefix("H:").range(240..=2160));
            });

            ui.horizontal(|ui| {
                ui.label("Quality:");
                ui.add(egui::Slider::new(&mut settings.video_quality, 1..=100).suffix("%"));
            });
        }
    });

    // Trajectory settings
    ui.collapsing("📈 Trajectory", |ui| {
        ui.checkbox(&mut settings.trajectory_enabled, "Enable trajectory recording");

        if settings.trajectory_enabled {
            ui.horizontal(|ui| {
                ui.label("Rate:");
                ui.add(egui::DragValue::new(&mut settings.trajectory_rate_hz).range(1.0..=1000.0).suffix(" Hz"));
            });

            ui.checkbox(&mut settings.record_transforms, "Record transforms");
            ui.checkbox(&mut settings.record_velocities, "Record velocities");
            ui.checkbox(&mut settings.record_forces, "Record forces");
        }
    });

    // Sensor data settings
    ui.collapsing("📡 Sensor Data", |ui| {
        ui.checkbox(&mut settings.sensor_data_enabled, "Enable sensor recording");

        if settings.sensor_data_enabled {
            ui.horizontal(|ui| {
                ui.label("Format:");
                egui::ComboBox::from_id_salt("data_format")
                    .selected_text(settings.data_format.name())
                    .show_ui(ui, |ui| {
                        for format in DataFormat::all() {
                            if ui.selectable_label(settings.data_format == *format, format.name()).clicked() {
                                settings.data_format = *format;
                            }
                        }
                    });
            });

            ui.label("Record:");
            ui.horizontal(|ui| {
                ui.checkbox(&mut settings.record_cameras, "Cameras");
                ui.checkbox(&mut settings.record_lidar, "LiDAR");
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut settings.record_imu, "IMU");
                ui.checkbox(&mut settings.record_other_sensors, "Other");
            });
        }
    });

    // Output settings
    if config.show_advanced {
        ui.collapsing("📁 Output", |ui| {
            ui.horizontal(|ui| {
                ui.label("Directory:");
                let dir_str = settings.output_directory.to_string_lossy().to_string();
                let mut dir_edit = dir_str.clone();
                if ui.text_edit_singleline(&mut dir_edit).changed() {
                    settings.output_directory = PathBuf::from(dir_edit);
                }
            });

            ui.horizontal(|ui| {
                ui.label("Prefix:");
                ui.text_edit_singleline(&mut settings.prefix);
            });

            ui.checkbox(&mut settings.auto_timestamp, "Auto-add timestamp");
        });

        // Playback settings
        if config.show_playback {
            ui.collapsing("▶ Playback", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Speed:");
                    ui.add(egui::Slider::new(&mut settings.playback_speed, 0.1..=4.0).suffix("x"));
                });
                ui.checkbox(&mut settings.loop_playback, "Loop playback");

                if ui.button("Load Recording...").clicked() {
                    // Would trigger file dialog
                }
            });
        }
    }

    // Options
    ui.separator();
    ui.horizontal(|ui| {
        ui.checkbox(&mut config.show_advanced, "Advanced");
        ui.checkbox(&mut config.show_playback, "Playback");
    });

    events
}

fn format_bytes(bytes: u64) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else if bytes < 1024 * 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else {
        format!("{:.2} GB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    }
}

/// Recording panel plugin
pub struct RecordingPanelPlugin;

impl Plugin for RecordingPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RecordingSettings>()
            .init_resource::<RecordingPanelConfig>()
            .add_event::<RecordingEvent>();
    }
}
