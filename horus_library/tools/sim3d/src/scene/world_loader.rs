//! Unified World Loader
//!
//! Provides a single entry point for loading worlds/scenes from various file formats.
//! Automatically detects the format and dispatches to the appropriate loader.
//!
//! Supported formats:
//! - URDF (.urdf) - Robot description format
//! - Xacro (.xacro) - URDF with macros
//! - SDF (.sdf, .world) - Gazebo simulation description format
//! - MJCF (.xml) - MuJoCo model format
//! - USD (.usd, .usda, .usdc, .usdz) - Universal Scene Description
//! - OpenDRIVE (.xodr) - Road network format
//! - OpenSCENARIO (.xosc) - Driving scenario format
//! - Scene JSON (.scene, .json) - Native HORUS scene format
//! - Heightmap (.png, .tiff) - Terrain heightmaps

use bevy::prelude::*;
use std::path::{Path, PathBuf};
use thiserror::Error;

use crate::ui::recent_files::RecentFileType;

/// Result type for world loading operations
pub type WorldLoadResult<T> = std::result::Result<T, WorldLoadError>;

/// Errors that can occur during world loading
#[derive(Debug, Error)]
pub enum WorldLoadError {
    #[error("File not found: {0}")]
    FileNotFound(PathBuf),

    #[error("Unsupported file format: {0}")]
    UnsupportedFormat(String),

    #[error("Failed to parse file: {0}")]
    ParseError(String),

    #[error("Failed to load resource: {0}")]
    ResourceError(String),

    #[error("Invalid file content: {0}")]
    InvalidContent(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Missing dependency: {0}")]
    MissingDependency(String),

    #[error("Loader not implemented for format: {0}")]
    NotImplemented(String),
}

/// Information about a loaded world
#[derive(Debug, Clone)]
pub struct LoadedWorld {
    /// Name of the world/scene
    pub name: String,
    /// Source file path
    pub source_path: PathBuf,
    /// File format that was loaded
    pub format: RecentFileType,
    /// Number of objects in the world
    pub object_count: usize,
    /// Number of robots in the world
    pub robot_count: usize,
    /// Whether terrain was loaded
    pub has_terrain: bool,
    /// Whether roads were loaded
    pub has_roads: bool,
    /// Additional metadata
    pub metadata: WorldMetadata,
}

/// Additional metadata about a loaded world
#[derive(Debug, Clone, Default)]
pub struct WorldMetadata {
    /// Description if available
    pub description: Option<String>,
    /// Author/creator if available
    pub author: Option<String>,
    /// Version if available
    pub version: Option<String>,
    /// Units used (meters, centimeters, etc.)
    pub units: Option<String>,
    /// Up axis (Y or Z)
    pub up_axis: UpAxis,
    /// Custom key-value metadata
    pub custom: std::collections::HashMap<String, String>,
}

/// Up axis convention
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UpAxis {
    #[default]
    Y,
    Z,
}

/// Configuration for world loading
#[derive(Debug, Clone)]
pub struct WorldLoadConfig {
    /// Base path for resolving relative paths
    pub base_path: Option<PathBuf>,
    /// Whether to load physics properties
    pub load_physics: bool,
    /// Whether to load visual meshes
    pub load_visuals: bool,
    /// Whether to load collision meshes
    pub load_collisions: bool,
    /// Whether to spawn robots immediately
    pub spawn_robots: bool,
    /// Scale factor to apply
    pub scale: f32,
    /// Position offset
    pub position_offset: Vec3,
    /// Rotation offset (euler angles in degrees)
    pub rotation_offset: Vec3,
}

impl Default for WorldLoadConfig {
    fn default() -> Self {
        Self {
            base_path: None,
            load_physics: true,
            load_visuals: true,
            load_collisions: true,
            spawn_robots: true,
            scale: 1.0,
            position_offset: Vec3::ZERO,
            rotation_offset: Vec3::ZERO,
        }
    }
}

/// Unified world loader that dispatches to format-specific loaders
pub struct WorldLoader {
    config: WorldLoadConfig,
}

impl Default for WorldLoader {
    fn default() -> Self {
        Self::new()
    }
}

impl WorldLoader {
    /// Create a new world loader with default configuration
    pub fn new() -> Self {
        Self {
            config: WorldLoadConfig::default(),
        }
    }

    /// Create a world loader with custom configuration
    pub fn with_config(config: WorldLoadConfig) -> Self {
        Self { config }
    }

    /// Get file type from path
    pub fn detect_format(path: &Path) -> RecentFileType {
        RecentFileType::from_path(path)
    }

    /// Check if a file format is supported for world loading
    pub fn is_supported(path: &Path) -> bool {
        let file_type = Self::detect_format(path);
        matches!(
            file_type,
            RecentFileType::Scene
                | RecentFileType::Urdf
                | RecentFileType::Xacro
                | RecentFileType::Sdf
                | RecentFileType::Mjcf
                | RecentFileType::Usd
                | RecentFileType::OpenDrive
                | RecentFileType::OpenScenario
                | RecentFileType::Heightmap
        )
    }

    /// Get supported file extensions for file dialogs
    pub fn supported_extensions() -> Vec<&'static str> {
        vec![
            // Scene formats
            "scene", "scn", // Robot formats
            "urdf", "xacro", "sdf", "world", "srdf", // MJCF
            "xml",  // USD
            "usd", "usda", "usdc", "usdz", // Automotive
            "xodr", "xosc", // Heightmaps
            "png", "tiff", "tif", // Config (for scene definitions)
            "json", "yaml", "yml",
        ]
    }

    /// Get file filter string for file dialogs
    pub fn file_filter() -> String {
        format!(
            "World Files (*.{})",
            Self::supported_extensions().join(", *.")
        )
    }

    /// Load a world from a file path
    ///
    /// This is the main entry point - it detects the format and dispatches
    /// to the appropriate loader.
    pub fn load(&self, path: impl AsRef<Path>) -> WorldLoadResult<LoadedWorld> {
        let path = path.as_ref();

        // Check file exists
        if !path.exists() {
            return Err(WorldLoadError::FileNotFound(path.to_path_buf()));
        }

        // Detect format
        let format = Self::detect_format(path);

        // For ambiguous formats (XML), do content inspection
        let format = if format == RecentFileType::Mjcf
            && path.extension().map(|e| e == "xml").unwrap_or(false)
        {
            self.detect_xml_format(path)?
        } else {
            format
        };

        // Dispatch to appropriate loader
        match format {
            RecentFileType::Scene => self.load_scene(path),
            RecentFileType::Urdf => self.load_urdf(path),
            RecentFileType::Xacro => self.load_xacro(path),
            RecentFileType::Sdf => self.load_sdf(path),
            RecentFileType::Mjcf => self.load_mjcf(path),
            RecentFileType::Srdf => self.load_srdf(path),
            RecentFileType::Usd => self.load_usd(path),
            RecentFileType::OpenDrive => self.load_opendrive(path),
            RecentFileType::OpenScenario => self.load_openscenario(path),
            RecentFileType::Heightmap => self.load_heightmap(path),
            _ => Err(WorldLoadError::UnsupportedFormat(format!("{:?}", format))),
        }
    }

    /// Detect XML format by inspecting content
    fn detect_xml_format(&self, path: &Path) -> WorldLoadResult<RecentFileType> {
        let content = std::fs::read_to_string(path)?;
        let content_lower = content.to_lowercase();

        // Check for MuJoCo
        if content_lower.contains("<mujoco") {
            return Ok(RecentFileType::Mjcf);
        }

        // Check for URDF
        if content_lower.contains("<robot") && !content_lower.contains("xmlns:xacro")
            || content_lower.contains("<robot") && !content_lower.contains("xacro:")
        {
            // Pure URDF without xacro macros
            if content_lower.contains("<link") && content_lower.contains("<joint") {
                return Ok(RecentFileType::Urdf);
            }
        }

        // Check for SDF
        if content_lower.contains("<sdf")
            || content_lower.contains("<world") && content_lower.contains("<model")
        {
            return Ok(RecentFileType::Sdf);
        }

        // Check for SRDF
        if content_lower.contains("<robot") && content_lower.contains("<group") {
            return Ok(RecentFileType::Srdf);
        }

        // Default to MJCF for unknown XML in robotics context
        Ok(RecentFileType::Mjcf)
    }

    // =========================================================================
    // Format-specific loaders
    // =========================================================================

    fn load_scene(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::loader::SceneDefinition;

        // Determine format from extension and load
        let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");
        let scene = match ext {
            "yaml" | "yml" => SceneDefinition::from_yaml_file(path)
                .map_err(|e| WorldLoadError::ParseError(e.to_string()))?,
            "json" | "scene" => SceneDefinition::from_json_file(path)
                .map_err(|e| WorldLoadError::ParseError(e.to_string()))?,
            _ => return Err(WorldLoadError::UnsupportedFormat(ext.to_string())),
        };

        Ok(LoadedWorld {
            name: scene.name.clone(),
            source_path: path.to_path_buf(),
            format: RecentFileType::Scene,
            object_count: scene.objects.len(),
            robot_count: scene.robots.len(),
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata {
                description: scene.description.clone(),
                ..Default::default()
            },
        })
    }

    fn load_urdf(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        // Parse URDF XML to extract metadata without spawning entities
        let content = std::fs::read_to_string(path)?;
        let doc = roxmltree::Document::parse(&content)
            .map_err(|e| WorldLoadError::ParseError(format!("Invalid URDF XML: {}", e)))?;

        // Extract robot name from <robot name="...">
        let robot_name = doc
            .root_element()
            .attribute("name")
            .unwrap_or("Unnamed Robot")
            .to_string();

        // Count links and joints
        let link_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "link")
            .count();
        let joint_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "joint")
            .count();

        Ok(LoadedWorld {
            name: robot_name,
            source_path: path.to_path_buf(),
            format: RecentFileType::Urdf,
            object_count: link_count,
            robot_count: 1,
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata {
                custom: [
                    ("links".to_string(), link_count.to_string()),
                    ("joints".to_string(), joint_count.to_string()),
                ]
                .into_iter()
                .collect(),
                ..Default::default()
            },
        })
    }

    fn load_xacro(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::robot::xacro_loader::XacroPreprocessor;

        // Process xacro to URDF
        let mut preprocessor = XacroPreprocessor::new();
        let urdf_content = preprocessor
            .process(path)
            .map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        // Parse the processed URDF XML
        let doc = roxmltree::Document::parse(&urdf_content)
            .map_err(|e| WorldLoadError::ParseError(format!("Invalid URDF from xacro: {}", e)))?;

        let robot_name = doc
            .root_element()
            .attribute("name")
            .unwrap_or("Unnamed Robot")
            .to_string();

        let link_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "link")
            .count();
        let joint_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "joint")
            .count();

        Ok(LoadedWorld {
            name: robot_name,
            source_path: path.to_path_buf(),
            format: RecentFileType::Xacro,
            object_count: link_count,
            robot_count: 1,
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata {
                custom: [
                    ("links".to_string(), link_count.to_string()),
                    ("joints".to_string(), joint_count.to_string()),
                ]
                .into_iter()
                .collect(),
                ..Default::default()
            },
        })
    }

    fn load_sdf(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::sdf_importer::SDFImporter;

        let world = SDFImporter::load_file(path).map_err(WorldLoadError::ParseError)?;

        Ok(LoadedWorld {
            name: world.name.clone(),
            source_path: path.to_path_buf(),
            format: RecentFileType::Sdf,
            object_count: world.models.len(),
            robot_count: 0, // Would need to analyze model contents
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata::default(),
        })
    }

    fn load_mjcf(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        // Parse MJCF XML to extract metadata
        let content = std::fs::read_to_string(path)?;
        let doc = roxmltree::Document::parse(&content)
            .map_err(|e| WorldLoadError::ParseError(format!("Invalid MJCF XML: {}", e)))?;

        // Find <mujoco> root and get model name
        let mujoco_elem = doc.root_element();
        let model_name = mujoco_elem
            .attribute("model")
            .unwrap_or_else(|| {
                path.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("Unnamed Model")
            })
            .to_string();

        // Count bodies
        let body_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "body")
            .count();
        let joint_count = doc
            .descendants()
            .filter(|n| n.tag_name().name() == "joint")
            .count();
        let actuator_count = doc
            .descendants()
            .filter(|n| {
                n.tag_name().name() == "motor"
                    || n.tag_name().name() == "position"
                    || n.tag_name().name() == "velocity"
            })
            .count();

        Ok(LoadedWorld {
            name: model_name,
            source_path: path.to_path_buf(),
            format: RecentFileType::Mjcf,
            object_count: body_count,
            robot_count: 1, // MJCF typically describes one robot/model
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata {
                up_axis: UpAxis::Z, // MuJoCo uses Z-up by default
                custom: [
                    ("bodies".to_string(), body_count.to_string()),
                    ("joints".to_string(), joint_count.to_string()),
                    ("actuators".to_string(), actuator_count.to_string()),
                ]
                .into_iter()
                .collect(),
                ..Default::default()
            },
        })
    }

    fn load_srdf(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::robot::srdf_loader::SRDFLoader;

        let srdf =
            SRDFLoader::load_file(path).map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        Ok(LoadedWorld {
            name: srdf.name.clone(),
            source_path: path.to_path_buf(),
            format: RecentFileType::Srdf,
            object_count: 0,
            robot_count: 1,
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata::default(),
        })
    }

    fn load_usd(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::usd_importer::USDImporter;

        let mut importer = USDImporter::new();
        let stage = importer
            .load_file(path)
            .map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        let name = stage.default_prim.clone().unwrap_or_else(|| {
            path.file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("USD Scene")
                .to_string()
        });

        Ok(LoadedWorld {
            name,
            source_path: path.to_path_buf(),
            format: RecentFileType::Usd,
            object_count: count_usd_prims(&stage.root_prims),
            robot_count: 0, // Would need to scan for articulation roots
            has_terrain: false,
            has_roads: false,
            metadata: WorldMetadata {
                up_axis: match stage.up_axis {
                    crate::scene::usd_importer::UpAxis::Y => UpAxis::Y,
                    crate::scene::usd_importer::UpAxis::Z => UpAxis::Z,
                },
                units: Some(format!("{} meters per unit", stage.meters_per_unit)),
                ..Default::default()
            },
        })
    }

    fn load_opendrive(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::opendrive_loader::OpenDRIVELoader;

        let loader = OpenDRIVELoader::new();
        let network = loader
            .load(path)
            .map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        Ok(LoadedWorld {
            name: network.header.name.clone(),
            source_path: path.to_path_buf(),
            format: RecentFileType::OpenDrive,
            object_count: 0,
            robot_count: 0,
            has_terrain: false,
            has_roads: true,
            metadata: WorldMetadata {
                version: Some(network.header.version.clone()),
                ..Default::default()
            },
        })
    }

    fn load_openscenario(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::openscenario_loader::{EntityType, OpenSCENARIOLoader};

        let loader = OpenSCENARIOLoader::new();
        let scenario = loader
            .load(path)
            .map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        // Get name from file header description or fallback to filename
        let name = if !scenario.file_header.description.is_empty() {
            scenario.file_header.description.clone()
        } else {
            path.file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("OpenSCENARIO")
                .to_string()
        };

        // Count vehicles in entities
        let vehicle_count = scenario
            .entities
            .iter()
            .filter(|e| e.entity_type == EntityType::Vehicle)
            .count();

        // Check if road network is defined (has logic_file or scene_graph_file)
        let has_roads = scenario.road_network.logic_file.is_some()
            || scenario.road_network.scene_graph_file.is_some();

        Ok(LoadedWorld {
            name,
            source_path: path.to_path_buf(),
            format: RecentFileType::OpenScenario,
            object_count: scenario.entities.len(),
            robot_count: vehicle_count,
            has_terrain: false,
            has_roads,
            metadata: WorldMetadata {
                description: Some(scenario.file_header.description.clone()),
                author: Some(scenario.file_header.author.clone()),
                ..Default::default()
            },
        })
    }

    fn load_heightmap(&self, path: &Path) -> WorldLoadResult<LoadedWorld> {
        use crate::scene::heightmap_loader::HeightmapLoader;

        let loader = HeightmapLoader::new();
        let heightmap = loader
            .load_heightmap(path)
            .map_err(|e| WorldLoadError::ParseError(e.to_string()))?;

        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("Terrain")
            .to_string();

        Ok(LoadedWorld {
            name,
            source_path: path.to_path_buf(),
            format: RecentFileType::Heightmap,
            object_count: 1, // The terrain itself
            robot_count: 0,
            has_terrain: true,
            has_roads: false,
            metadata: WorldMetadata {
                custom: [
                    ("width".to_string(), heightmap.width.to_string()),
                    ("height".to_string(), heightmap.height.to_string()),
                ]
                .into_iter()
                .collect(),
                ..Default::default()
            },
        })
    }
}

// Helper functions

fn count_usd_prims(prims: &[crate::scene::usd_importer::USDPrim]) -> usize {
    prims.iter().map(|p| 1 + count_usd_prims(&p.children)).sum()
}

/// Bevy event for world loading requests
#[derive(Event, Debug, Clone)]
pub struct LoadWorldEvent {
    /// Path to the world file
    pub path: PathBuf,
    /// Optional configuration overrides
    pub config: Option<WorldLoadConfig>,
}

impl LoadWorldEvent {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self {
            path: path.into(),
            config: None,
        }
    }

    pub fn with_config(path: impl Into<PathBuf>, config: WorldLoadConfig) -> Self {
        Self {
            path: path.into(),
            config: Some(config),
        }
    }
}

/// Bevy event emitted when world loading completes
#[derive(Event, Debug, Clone)]
pub struct WorldLoadedEvent {
    /// The loaded world info
    pub world: LoadedWorld,
}

/// Bevy event emitted when world loading fails
#[derive(Event, Debug, Clone)]
pub struct WorldLoadErrorEvent {
    /// Path that failed to load
    pub path: PathBuf,
    /// Error message
    pub error: String,
}

/// Bevy resource tracking the currently loaded world
#[derive(Resource, Default)]
pub struct CurrentWorld {
    pub world: Option<LoadedWorld>,
}

/// System to handle world loading events
pub fn handle_load_world_events(
    mut load_events: EventReader<LoadWorldEvent>,
    mut loaded_events: EventWriter<WorldLoadedEvent>,
    mut error_events: EventWriter<WorldLoadErrorEvent>,
    mut current_world: ResMut<CurrentWorld>,
) {
    for event in load_events.read() {
        let config = event.config.clone().unwrap_or_default();
        let loader = WorldLoader::with_config(config);

        match loader.load(&event.path) {
            Ok(world) => {
                info!("Loaded world: {} from {:?}", world.name, world.source_path);
                current_world.world = Some(world.clone());
                loaded_events.send(WorldLoadedEvent { world });
            }
            Err(e) => {
                error!("Failed to load world from {:?}: {}", event.path, e);
                error_events.send(WorldLoadErrorEvent {
                    path: event.path.clone(),
                    error: e.to_string(),
                });
            }
        }
    }
}

/// Plugin for world loading functionality
pub struct WorldLoaderPlugin;

impl Plugin for WorldLoaderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CurrentWorld>()
            .add_event::<LoadWorldEvent>()
            .add_event::<WorldLoadedEvent>()
            .add_event::<WorldLoadErrorEvent>()
            .add_systems(Update, handle_load_world_events);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_detection() {
        assert_eq!(
            WorldLoader::detect_format(Path::new("robot.urdf")),
            RecentFileType::Urdf
        );
        assert_eq!(
            WorldLoader::detect_format(Path::new("scene.usd")),
            RecentFileType::Usd
        );
        assert_eq!(
            WorldLoader::detect_format(Path::new("road.xodr")),
            RecentFileType::OpenDrive
        );
    }

    #[test]
    fn test_supported_check() {
        assert!(WorldLoader::is_supported(Path::new("robot.urdf")));
        assert!(WorldLoader::is_supported(Path::new("world.sdf")));
        assert!(WorldLoader::is_supported(Path::new("scene.usd")));
        assert!(!WorldLoader::is_supported(Path::new("document.pdf")));
    }

    #[test]
    fn test_config_default() {
        let config = WorldLoadConfig::default();
        assert!(config.load_physics);
        assert!(config.load_visuals);
        assert_eq!(config.scale, 1.0);
    }
}
