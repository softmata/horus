//! Rendering Control Panel
//!
//! Provides runtime controls for rendering quality settings including
//! bloom, HDR, shadows, ambient occlusion, and post-processing effects.

use bevy::prelude::*;
use bevy_egui::egui;

/// Rendering quality preset
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RenderingPreset {
    Low,
    #[default]
    Medium,
    High,
    Ultra,
    Custom,
}

impl RenderingPreset {
    pub fn name(&self) -> &'static str {
        match self {
            RenderingPreset::Low => "Low",
            RenderingPreset::Medium => "Medium",
            RenderingPreset::High => "High",
            RenderingPreset::Ultra => "Ultra",
            RenderingPreset::Custom => "Custom",
        }
    }
}

/// Shadow quality settings
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ShadowQuality {
    Off,
    Low,
    #[default]
    Medium,
    High,
    Ultra,
}

impl ShadowQuality {
    pub fn name(&self) -> &'static str {
        match self {
            ShadowQuality::Off => "Off",
            ShadowQuality::Low => "Low (512)",
            ShadowQuality::Medium => "Medium (1024)",
            ShadowQuality::High => "High (2048)",
            ShadowQuality::Ultra => "Ultra (4096)",
        }
    }

    pub fn resolution(&self) -> u32 {
        match self {
            ShadowQuality::Off => 0,
            ShadowQuality::Low => 512,
            ShadowQuality::Medium => 1024,
            ShadowQuality::High => 2048,
            ShadowQuality::Ultra => 4096,
        }
    }
}

/// Rendering settings resource
#[derive(Resource)]
pub struct RenderingSettings {
    /// Current preset
    pub preset: RenderingPreset,

    // Bloom settings
    pub bloom_enabled: bool,
    pub bloom_intensity: f32,
    pub bloom_threshold: f32,

    // HDR settings
    pub hdr_enabled: bool,
    pub exposure: f32,
    pub tonemapping: TonemappingMode,

    // Shadow settings
    pub shadow_quality: ShadowQuality,
    pub shadow_distance: f32,

    // Ambient occlusion
    pub ao_enabled: bool,
    pub ao_intensity: f32,
    pub ao_radius: f32,

    // Atmosphere/Fog
    pub fog_enabled: bool,
    pub fog_color: [f32; 3],
    pub fog_start: f32,
    pub fog_end: f32,

    // Anti-aliasing
    pub aa_mode: AAMode,

    // Post-processing
    pub vignette_enabled: bool,
    pub vignette_intensity: f32,
    pub film_grain_enabled: bool,
    pub film_grain_intensity: f32,
    pub chromatic_aberration_enabled: bool,
    pub chromatic_aberration_intensity: f32,

    /// Dirty flag for applying changes
    pub dirty: bool,
}

/// Tonemapping modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TonemappingMode {
    #[default]
    TonyMcMapface,
    Reinhard,
    ReinhardLuminance,
    AcesFitted,
    AgX,
    SomewhatBoringDisplayTransform,
    Blender,
    None,
}

impl TonemappingMode {
    pub fn name(&self) -> &'static str {
        match self {
            TonemappingMode::TonyMcMapface => "TonyMcMapface",
            TonemappingMode::Reinhard => "Reinhard",
            TonemappingMode::ReinhardLuminance => "Reinhard Luminance",
            TonemappingMode::AcesFitted => "ACES Fitted",
            TonemappingMode::AgX => "AgX",
            TonemappingMode::SomewhatBoringDisplayTransform => "Somewhat Boring",
            TonemappingMode::Blender => "Blender",
            TonemappingMode::None => "None",
        }
    }

    pub fn all() -> &'static [TonemappingMode] {
        &[
            TonemappingMode::TonyMcMapface,
            TonemappingMode::Reinhard,
            TonemappingMode::ReinhardLuminance,
            TonemappingMode::AcesFitted,
            TonemappingMode::AgX,
            TonemappingMode::SomewhatBoringDisplayTransform,
            TonemappingMode::Blender,
            TonemappingMode::None,
        ]
    }
}

/// Anti-aliasing modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AAMode {
    Off,
    #[default]
    FXAA,
    SMAA,
    TAA,
    MSAA2x,
    MSAA4x,
    MSAA8x,
}

impl AAMode {
    pub fn name(&self) -> &'static str {
        match self {
            AAMode::Off => "Off",
            AAMode::FXAA => "FXAA",
            AAMode::SMAA => "SMAA",
            AAMode::TAA => "TAA",
            AAMode::MSAA2x => "MSAA 2x",
            AAMode::MSAA4x => "MSAA 4x",
            AAMode::MSAA8x => "MSAA 8x",
        }
    }

    pub fn all() -> &'static [AAMode] {
        &[
            AAMode::Off,
            AAMode::FXAA,
            AAMode::SMAA,
            AAMode::TAA,
            AAMode::MSAA2x,
            AAMode::MSAA4x,
            AAMode::MSAA8x,
        ]
    }
}

impl Default for RenderingSettings {
    fn default() -> Self {
        Self {
            preset: RenderingPreset::Medium,
            bloom_enabled: true,
            bloom_intensity: 0.3,
            bloom_threshold: 1.0,
            hdr_enabled: true,
            exposure: 1.0,
            tonemapping: TonemappingMode::TonyMcMapface,
            shadow_quality: ShadowQuality::Medium,
            shadow_distance: 100.0,
            ao_enabled: true,
            ao_intensity: 0.5,
            ao_radius: 0.5,
            fog_enabled: false,
            fog_color: [0.5, 0.6, 0.7],
            fog_start: 50.0,
            fog_end: 500.0,
            aa_mode: AAMode::FXAA,
            vignette_enabled: false,
            vignette_intensity: 0.3,
            film_grain_enabled: false,
            film_grain_intensity: 0.1,
            chromatic_aberration_enabled: false,
            chromatic_aberration_intensity: 0.02,
            dirty: false,
        }
    }
}

impl RenderingSettings {
    pub fn apply_preset(&mut self, preset: RenderingPreset) {
        self.preset = preset;
        match preset {
            RenderingPreset::Low => {
                self.bloom_enabled = false;
                self.ao_enabled = false;
                self.shadow_quality = ShadowQuality::Low;
                self.aa_mode = AAMode::Off;
                self.vignette_enabled = false;
                self.film_grain_enabled = false;
                self.chromatic_aberration_enabled = false;
            }
            RenderingPreset::Medium => {
                self.bloom_enabled = true;
                self.bloom_intensity = 0.3;
                self.ao_enabled = true;
                self.ao_intensity = 0.5;
                self.shadow_quality = ShadowQuality::Medium;
                self.aa_mode = AAMode::FXAA;
                self.vignette_enabled = false;
                self.film_grain_enabled = false;
                self.chromatic_aberration_enabled = false;
            }
            RenderingPreset::High => {
                self.bloom_enabled = true;
                self.bloom_intensity = 0.4;
                self.ao_enabled = true;
                self.ao_intensity = 0.6;
                self.shadow_quality = ShadowQuality::High;
                self.aa_mode = AAMode::TAA;
                self.vignette_enabled = true;
                self.vignette_intensity = 0.2;
                self.film_grain_enabled = false;
                self.chromatic_aberration_enabled = false;
            }
            RenderingPreset::Ultra => {
                self.bloom_enabled = true;
                self.bloom_intensity = 0.5;
                self.ao_enabled = true;
                self.ao_intensity = 0.8;
                self.shadow_quality = ShadowQuality::Ultra;
                self.aa_mode = AAMode::MSAA4x;
                self.vignette_enabled = true;
                self.vignette_intensity = 0.3;
                self.film_grain_enabled = true;
                self.film_grain_intensity = 0.05;
                self.chromatic_aberration_enabled = true;
                self.chromatic_aberration_intensity = 0.01;
            }
            RenderingPreset::Custom => {}
        }
        self.dirty = true;
    }
}

/// Rendering panel configuration
#[derive(Resource, Default)]
pub struct RenderingPanelConfig {
    pub show_advanced: bool,
}

/// Render the rendering control panel UI
pub fn render_rendering_panel_ui(
    ui: &mut egui::Ui,
    settings: &mut RenderingSettings,
    config: &mut RenderingPanelConfig,
) {
    ui.heading("🎨 Rendering");
    ui.separator();

    // Preset selector
    ui.horizontal(|ui| {
        ui.label("Preset:");
        egui::ComboBox::from_id_salt("render_preset")
            .selected_text(settings.preset.name())
            .show_ui(ui, |ui| {
                for preset in [RenderingPreset::Low, RenderingPreset::Medium, RenderingPreset::High, RenderingPreset::Ultra, RenderingPreset::Custom] {
                    if ui.selectable_label(settings.preset == preset, preset.name()).clicked() {
                        settings.apply_preset(preset);
                    }
                }
            });
    });

    ui.separator();

    // Bloom
    ui.collapsing("✨ Bloom", |ui| {
        if ui.checkbox(&mut settings.bloom_enabled, "Enabled").changed() {
            settings.dirty = true;
            settings.preset = RenderingPreset::Custom;
        }
        if settings.bloom_enabled {
            ui.horizontal(|ui| {
                ui.label("Intensity:");
                if ui.add(egui::Slider::new(&mut settings.bloom_intensity, 0.0..=1.0)).changed() {
                    settings.dirty = true;
                    settings.preset = RenderingPreset::Custom;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Threshold:");
                if ui.add(egui::Slider::new(&mut settings.bloom_threshold, 0.0..=3.0)).changed() {
                    settings.dirty = true;
                    settings.preset = RenderingPreset::Custom;
                }
            });
        }
    });

    // HDR & Tonemapping
    ui.collapsing("☀ HDR & Tonemapping", |ui| {
        if ui.checkbox(&mut settings.hdr_enabled, "HDR").changed() {
            settings.dirty = true;
        }
        ui.horizontal(|ui| {
            ui.label("Exposure:");
            if ui.add(egui::Slider::new(&mut settings.exposure, 0.1..=5.0)).changed() {
                settings.dirty = true;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Tonemapping:");
            egui::ComboBox::from_id_salt("tonemapping")
                .selected_text(settings.tonemapping.name())
                .show_ui(ui, |ui| {
                    for mode in TonemappingMode::all() {
                        if ui.selectable_label(settings.tonemapping == *mode, mode.name()).clicked() {
                            settings.tonemapping = *mode;
                            settings.dirty = true;
                        }
                    }
                });
        });
    });

    // Shadows
    ui.collapsing("🌑 Shadows", |ui| {
        ui.horizontal(|ui| {
            ui.label("Quality:");
            egui::ComboBox::from_id_salt("shadow_quality")
                .selected_text(settings.shadow_quality.name())
                .show_ui(ui, |ui| {
                    for quality in [ShadowQuality::Off, ShadowQuality::Low, ShadowQuality::Medium, ShadowQuality::High, ShadowQuality::Ultra] {
                        if ui.selectable_label(settings.shadow_quality == quality, quality.name()).clicked() {
                            settings.shadow_quality = quality;
                            settings.dirty = true;
                            settings.preset = RenderingPreset::Custom;
                        }
                    }
                });
        });
        ui.horizontal(|ui| {
            ui.label("Distance:");
            if ui.add(egui::Slider::new(&mut settings.shadow_distance, 10.0..=500.0).suffix(" m")).changed() {
                settings.dirty = true;
            }
        });
    });

    // Ambient Occlusion
    ui.collapsing("🌫 Ambient Occlusion", |ui| {
        if ui.checkbox(&mut settings.ao_enabled, "Enabled").changed() {
            settings.dirty = true;
            settings.preset = RenderingPreset::Custom;
        }
        if settings.ao_enabled {
            ui.horizontal(|ui| {
                ui.label("Intensity:");
                if ui.add(egui::Slider::new(&mut settings.ao_intensity, 0.0..=1.0)).changed() {
                    settings.dirty = true;
                }
            });
            ui.horizontal(|ui| {
                ui.label("Radius:");
                if ui.add(egui::Slider::new(&mut settings.ao_radius, 0.1..=2.0)).changed() {
                    settings.dirty = true;
                }
            });
        }
    });

    // Anti-aliasing
    ui.collapsing("🔲 Anti-Aliasing", |ui| {
        ui.horizontal(|ui| {
            ui.label("Mode:");
            egui::ComboBox::from_id_salt("aa_mode")
                .selected_text(settings.aa_mode.name())
                .show_ui(ui, |ui| {
                    for mode in AAMode::all() {
                        if ui.selectable_label(settings.aa_mode == *mode, mode.name()).clicked() {
                            settings.aa_mode = *mode;
                            settings.dirty = true;
                            settings.preset = RenderingPreset::Custom;
                        }
                    }
                });
        });
    });

    // Advanced: Fog and post-processing
    if config.show_advanced {
        ui.collapsing("🌁 Fog", |ui| {
            if ui.checkbox(&mut settings.fog_enabled, "Enabled").changed() {
                settings.dirty = true;
            }
            if settings.fog_enabled {
                ui.horizontal(|ui| {
                    ui.label("Color:");
                    if ui.color_edit_button_rgb(&mut settings.fog_color).changed() {
                        settings.dirty = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Start:");
                    if ui.add(egui::DragValue::new(&mut settings.fog_start).suffix(" m")).changed() {
                        settings.dirty = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("End:");
                    if ui.add(egui::DragValue::new(&mut settings.fog_end).suffix(" m")).changed() {
                        settings.dirty = true;
                    }
                });
            }
        });

        ui.collapsing("🎬 Post-Processing", |ui| {
            // Vignette
            ui.horizontal(|ui| {
                if ui.checkbox(&mut settings.vignette_enabled, "Vignette").changed() {
                    settings.dirty = true;
                }
                if settings.vignette_enabled {
                    if ui.add(egui::Slider::new(&mut settings.vignette_intensity, 0.0..=1.0).text("")).changed() {
                        settings.dirty = true;
                    }
                }
            });

            // Film grain
            ui.horizontal(|ui| {
                if ui.checkbox(&mut settings.film_grain_enabled, "Film Grain").changed() {
                    settings.dirty = true;
                }
                if settings.film_grain_enabled {
                    if ui.add(egui::Slider::new(&mut settings.film_grain_intensity, 0.0..=0.5).text("")).changed() {
                        settings.dirty = true;
                    }
                }
            });

            // Chromatic aberration
            ui.horizontal(|ui| {
                if ui.checkbox(&mut settings.chromatic_aberration_enabled, "Chromatic Aberration").changed() {
                    settings.dirty = true;
                }
                if settings.chromatic_aberration_enabled {
                    if ui.add(egui::Slider::new(&mut settings.chromatic_aberration_intensity, 0.0..=0.1).text("")).changed() {
                        settings.dirty = true;
                    }
                }
            });
        });
    }

    // Options
    ui.separator();
    ui.checkbox(&mut config.show_advanced, "Advanced");

    // Apply indicator
    if settings.dirty {
        ui.separator();
        ui.colored_label(egui::Color32::YELLOW, "⚠ Settings modified");
        if ui.button("Apply Changes").clicked() {
            settings.dirty = false;
        }
    }
}

/// Rendering panel plugin
pub struct RenderingPanelPlugin;

impl Plugin for RenderingPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RenderingSettings>()
            .init_resource::<RenderingPanelConfig>();
    }
}
