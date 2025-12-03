//! Gazebo Model Packages Loader
//!
//! Supports loading models from Gazebo model repository format:
//! - model.config XML metadata
//! - model.sdf (SDF model definition)
//! - meshes/ directory with visual/collision meshes
//! - materials/ directory with textures and scripts
//!
//! Compatible with:
//! - Gazebo classic models (model database)
//! - Ignition/Gazebo Fuel models
//! - Custom model packages

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::error::{EnhancedError, ErrorCategory, Result};
use crate::scene::sdf_importer::{SDFImporter, SDFModel};

// ============================================================================
// Model Configuration (model.config)
// ============================================================================

/// Model configuration from model.config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelConfig {
    /// Model name
    pub name: String,
    /// Model version
    pub version: String,
    /// SDF file path
    pub sdf: SDFReference,
    /// Model author
    pub author: Option<Author>,
    /// Model description
    pub description: String,
    /// Thumbnail image
    pub thumbnail: Option<String>,
    /// Dependencies on other models
    #[serde(default)]
    pub depends: Vec<Dependency>,
}

/// SDF file reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SDFReference {
    /// SDF version
    #[serde(rename = "@version")]
    pub version: Option<String>,
    /// SDF file path
    #[serde(rename = "$text")]
    pub path: String,
}

/// Model author
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Author {
    pub name: String,
    pub email: Option<String>,
}

/// Model dependency
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dependency {
    pub uri: String,
}

// ============================================================================
// Model Database
// ============================================================================

/// Database of available models
#[derive(Debug, Clone, Default)]
pub struct ModelDatabase {
    /// Model paths by name
    models: HashMap<String, ModelEntry>,
    /// Search paths for models
    search_paths: Vec<PathBuf>,
}

/// Entry for a model in the database
#[derive(Debug, Clone)]
pub struct ModelEntry {
    /// Model name
    pub name: String,
    /// Path to model directory
    pub path: PathBuf,
    /// Model configuration (if loaded)
    pub config: Option<ModelConfig>,
    /// Model category (if known)
    pub category: Option<String>,
}

impl ModelDatabase {
    /// Create a new empty model database
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a search path for models
    pub fn add_search_path<P: AsRef<Path>>(&mut self, path: P) {
        self.search_paths.push(path.as_ref().to_path_buf());
    }

    /// Add default Gazebo model paths
    pub fn add_default_paths(&mut self) {
        // Gazebo classic default paths
        if let Some(home) = dirs::home_dir() {
            self.add_search_path(home.join(".gazebo/models"));
            self.add_search_path(home.join(".ignition/fuel"));
        }

        // System paths
        self.add_search_path("/usr/share/gazebo/models");
        self.add_search_path("/usr/share/gazebo-11/models");

        // Check GAZEBO_MODEL_PATH environment variable
        if let Ok(model_path) = std::env::var("GAZEBO_MODEL_PATH") {
            for path in model_path.split(':') {
                self.add_search_path(path);
            }
        }

        // Check IGN_GAZEBO_RESOURCE_PATH environment variable
        if let Ok(resource_path) = std::env::var("IGN_GAZEBO_RESOURCE_PATH") {
            for path in resource_path.split(':') {
                self.add_search_path(path);
            }
        }

        // Check GZ_SIM_RESOURCE_PATH environment variable
        if let Ok(resource_path) = std::env::var("GZ_SIM_RESOURCE_PATH") {
            for path in resource_path.split(':') {
                self.add_search_path(path);
            }
        }
    }

    /// Scan search paths and populate database
    pub fn scan(&mut self) -> Result<usize> {
        let mut count = 0;

        for search_path in self.search_paths.clone() {
            if !search_path.exists() {
                continue;
            }

            // Scan directory for model packages
            if let Ok(entries) = std::fs::read_dir(&search_path) {
                for entry in entries.filter_map(|e| e.ok()) {
                    let path = entry.path();
                    if path.is_dir() {
                        if let Some(entry) = self.scan_model_directory(&path) {
                            self.models.insert(entry.name.clone(), entry);
                            count += 1;
                        }
                    }
                }
            }
        }

        info!("Model database scanned: {} models found", count);
        Ok(count)
    }

    /// Scan a single model directory
    fn scan_model_directory(&self, path: &Path) -> Option<ModelEntry> {
        let config_path = path.join("model.config");
        let sdf_path = path.join("model.sdf");

        // Check for model.config or model.sdf
        if !config_path.exists() && !sdf_path.exists() {
            return None;
        }

        let name = path.file_name()?.to_str()?.to_string();

        let config = if config_path.exists() {
            self.load_model_config(&config_path).ok()
        } else {
            None
        };

        Some(ModelEntry {
            name: config.as_ref().map(|c| c.name.clone()).unwrap_or(name),
            path: path.to_path_buf(),
            config,
            category: None,
        })
    }

    /// Load model configuration from model.config
    fn load_model_config(&self, path: &Path) -> Result<ModelConfig> {
        let content = std::fs::read_to_string(path).map_err(|e| {
            EnhancedError::new(format!("Failed to read model.config: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        self.parse_model_config(&content, path)
    }

    /// Parse model.config XML
    fn parse_model_config(&self, content: &str, source: &Path) -> Result<ModelConfig> {
        let doc = roxmltree::Document::parse(content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse model.config: {}", e))
                .with_file(source)
                .with_category(ErrorCategory::ParseError)
        })?;

        let root = doc.root_element();
        if root.tag_name().name() != "model" {
            return Err(
                EnhancedError::new("Invalid model.config: root must be <model>").with_file(source),
            );
        }

        let name = root
            .children()
            .find(|n| n.tag_name().name() == "name")
            .and_then(|n| n.text())
            .unwrap_or("unknown")
            .to_string();

        let version = root
            .children()
            .find(|n| n.tag_name().name() == "version")
            .and_then(|n| n.text())
            .unwrap_or("1.0")
            .to_string();

        let description = root
            .children()
            .find(|n| n.tag_name().name() == "description")
            .and_then(|n| n.text())
            .unwrap_or("")
            .to_string();

        let sdf = root
            .children()
            .find(|n| n.tag_name().name() == "sdf")
            .map(|n| SDFReference {
                version: n.attribute("version").map(String::from),
                path: n.text().unwrap_or("model.sdf").to_string(),
            })
            .unwrap_or(SDFReference {
                version: None,
                path: "model.sdf".to_string(),
            });

        let author = root
            .children()
            .find(|n| n.tag_name().name() == "author")
            .map(|n| Author {
                name: n
                    .children()
                    .find(|c| c.tag_name().name() == "name")
                    .and_then(|c| c.text())
                    .unwrap_or("")
                    .to_string(),
                email: n
                    .children()
                    .find(|c| c.tag_name().name() == "email")
                    .and_then(|c| c.text())
                    .map(String::from),
            });

        let thumbnail = root
            .children()
            .find(|n| n.tag_name().name() == "thumbnail")
            .and_then(|n| n.text())
            .map(String::from);

        let depends: Vec<Dependency> = root
            .children()
            .filter(|n| n.tag_name().name() == "depend")
            .filter_map(|n| {
                n.children()
                    .find(|c| c.tag_name().name() == "uri")
                    .and_then(|c| c.text())
                    .map(|uri| Dependency {
                        uri: uri.to_string(),
                    })
            })
            .collect();

        Ok(ModelConfig {
            name,
            version,
            sdf,
            author,
            description,
            thumbnail,
            depends,
        })
    }

    /// Find a model by name
    pub fn find(&self, name: &str) -> Option<&ModelEntry> {
        self.models.get(name)
    }

    /// Find a model by URI (model://name or fuel://...)
    pub fn find_by_uri(&self, uri: &str) -> Option<&ModelEntry> {
        if uri.starts_with("model://") {
            let name = uri.trim_start_matches("model://");
            // Handle model://name or model://name/path
            let model_name = name.split('/').next()?;
            self.models.get(model_name)
        } else if uri.starts_with("fuel://") {
            // Ignition Fuel format: fuel://server/owner/type/name/version
            let parts: Vec<&str> = uri.trim_start_matches("fuel://").split('/').collect();
            if parts.len() >= 4 {
                let name = parts[3];
                self.models.get(name)
            } else {
                None
            }
        } else {
            // Try as direct name
            self.models.get(uri)
        }
    }

    /// Get all models in the database
    pub fn all_models(&self) -> impl Iterator<Item = &ModelEntry> {
        self.models.values()
    }

    /// Get models by category
    pub fn models_by_category<'a>(
        &'a self,
        category: &'a str,
    ) -> impl Iterator<Item = &'a ModelEntry> + 'a {
        self.models
            .values()
            .filter(move |e| e.category.as_deref() == Some(category))
    }

    /// Number of models in database
    pub fn len(&self) -> usize {
        self.models.len()
    }

    /// Check if database is empty
    pub fn is_empty(&self) -> bool {
        self.models.is_empty()
    }
}

// ============================================================================
// Model Loader
// ============================================================================

/// Loader for Gazebo model packages
pub struct GazeboModelLoader {
    /// Model database
    database: ModelDatabase,
}

impl GazeboModelLoader {
    /// Create a new model loader
    pub fn new() -> Self {
        let mut database = ModelDatabase::new();
        database.add_default_paths();

        Self { database }
    }

    /// Create with custom database
    pub fn with_database(database: ModelDatabase) -> Self {
        Self { database }
    }

    /// Add a search path
    pub fn add_search_path<P: AsRef<Path>>(&mut self, path: P) {
        self.database.add_search_path(path);
    }

    /// Scan for available models
    pub fn scan(&mut self) -> Result<usize> {
        self.database.scan()
    }

    /// Get the model database
    pub fn database(&self) -> &ModelDatabase {
        &self.database
    }

    /// Load a model by name or URI
    pub fn load(&self, name_or_uri: &str) -> Result<LoadedGazeboModel> {
        let entry = self
            .database
            .find_by_uri(name_or_uri)
            .or_else(|| self.database.find(name_or_uri))
            .ok_or_else(|| {
                EnhancedError::new(format!("Model not found: {}", name_or_uri))
                    .with_hint("Run scan() to populate model database or check search paths")
            })?;

        self.load_from_path(&entry.path)
    }

    /// Load a model from a specific path
    pub fn load_from_path<P: AsRef<Path>>(&self, path: P) -> Result<LoadedGazeboModel> {
        let path = path.as_ref();

        // Load model.config if present
        let config = {
            let config_path = path.join("model.config");
            if config_path.exists() {
                Some(self.database.load_model_config(&config_path)?)
            } else {
                None
            }
        };

        // Determine SDF file path
        let sdf_path = if let Some(ref cfg) = config {
            path.join(&cfg.sdf.path)
        } else {
            let default_sdf = path.join("model.sdf");
            if default_sdf.exists() {
                default_sdf
            } else {
                // Try to find any .sdf file
                std::fs::read_dir(path)
                    .ok()
                    .and_then(|entries| {
                        entries
                            .filter_map(|e| e.ok())
                            .find(|e| {
                                e.path()
                                    .extension()
                                    .map(|ext| ext == "sdf")
                                    .unwrap_or(false)
                            })
                            .map(|e| e.path())
                    })
                    .ok_or_else(|| {
                        EnhancedError::new("No SDF file found in model package")
                            .with_file(path)
                            .with_hint("Model package must contain model.sdf or specify SDF in model.config")
                    })?
            }
        };

        // Load SDF
        let sdf_world = SDFImporter::load_file(&sdf_path).map_err(|e| {
            EnhancedError::new(format!("Failed to load SDF file: {}", e)).with_file(&sdf_path)
        })?;

        // Extract first model from SDF
        let sdf_model =
            sdf_world.models.into_iter().next().ok_or_else(|| {
                EnhancedError::new("No model found in SDF file").with_file(&sdf_path)
            })?;

        // Collect mesh paths
        let meshes = self.collect_meshes(path);

        // Collect material paths
        let materials = self.collect_materials(path);

        Ok(LoadedGazeboModel {
            name: config.as_ref().map(|c| c.name.clone()).unwrap_or_else(|| {
                path.file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or("unknown")
                    .to_string()
            }),
            path: path.to_path_buf(),
            config,
            sdf_model,
            meshes,
            materials,
        })
    }

    /// Collect mesh files from model directory
    fn collect_meshes(&self, model_path: &Path) -> Vec<MeshFile> {
        let mut meshes = Vec::new();

        let mesh_dir = model_path.join("meshes");
        if mesh_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(&mesh_dir) {
                for entry in entries.filter_map(|e| e.ok()) {
                    let path = entry.path();
                    if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                        let mesh_type = match ext.to_lowercase().as_str() {
                            "dae" => Some(MeshType::Collada),
                            "obj" => Some(MeshType::Obj),
                            "stl" => Some(MeshType::Stl),
                            "gltf" | "glb" => Some(MeshType::Gltf),
                            _ => None,
                        };

                        if let Some(mesh_type) = mesh_type {
                            meshes.push(MeshFile { path, mesh_type });
                        }
                    }
                }
            }
        }

        // Also check root directory for meshes
        if let Ok(entries) = std::fs::read_dir(model_path) {
            for entry in entries.filter_map(|e| e.ok()) {
                let path = entry.path();
                if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                    let mesh_type = match ext.to_lowercase().as_str() {
                        "dae" => Some(MeshType::Collada),
                        "obj" => Some(MeshType::Obj),
                        "stl" => Some(MeshType::Stl),
                        "gltf" | "glb" => Some(MeshType::Gltf),
                        _ => None,
                    };

                    if let Some(mesh_type) = mesh_type {
                        if !meshes.iter().any(|m| m.path == path) {
                            meshes.push(MeshFile { path, mesh_type });
                        }
                    }
                }
            }
        }

        meshes
    }

    /// Collect material files from model directory
    fn collect_materials(&self, model_path: &Path) -> Vec<MaterialFile> {
        let mut materials = Vec::new();

        let materials_dir = model_path.join("materials");
        if materials_dir.exists() {
            // Check textures subdirectory
            let textures_dir = materials_dir.join("textures");
            if textures_dir.exists() {
                if let Ok(entries) = std::fs::read_dir(&textures_dir) {
                    for entry in entries.filter_map(|e| e.ok()) {
                        let path = entry.path();
                        if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                            let texture_type = match ext.to_lowercase().as_str() {
                                "png" | "jpg" | "jpeg" | "tga" | "bmp" => Some(TextureType::Image),
                                _ => None,
                            };

                            if let Some(texture_type) = texture_type {
                                materials.push(MaterialFile {
                                    path,
                                    file_type: MaterialFileType::Texture(texture_type),
                                });
                            }
                        }
                    }
                }
            }

            // Check scripts subdirectory for Ogre materials
            let scripts_dir = materials_dir.join("scripts");
            if scripts_dir.exists() {
                if let Ok(entries) = std::fs::read_dir(&scripts_dir) {
                    for entry in entries.filter_map(|e| e.ok()) {
                        let path = entry.path();
                        if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                            if ext == "material" {
                                materials.push(MaterialFile {
                                    path,
                                    file_type: MaterialFileType::OgreMaterial,
                                });
                            }
                        }
                    }
                }
            }
        }

        materials
    }

    /// Resolve a model URI to a path
    pub fn resolve_uri(&self, uri: &str, base_path: &Path) -> Option<PathBuf> {
        if uri.starts_with("model://") {
            let model_name = uri.trim_start_matches("model://");
            // Handle model://name/path/to/file
            let parts: Vec<&str> = model_name.split('/').collect();
            if parts.is_empty() {
                return None;
            }

            let model = self.database.find(parts[0])?;
            if parts.len() == 1 {
                Some(model.path.clone())
            } else {
                Some(model.path.join(parts[1..].join("/")))
            }
        } else if uri.starts_with("file://") {
            Some(PathBuf::from(uri.trim_start_matches("file://")))
        } else if uri.starts_with('/') {
            Some(PathBuf::from(uri))
        } else {
            // Relative path
            Some(base_path.join(uri))
        }
    }
}

impl Default for GazeboModelLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// Loaded Gazebo model
#[derive(Debug, Clone)]
pub struct LoadedGazeboModel {
    /// Model name
    pub name: String,
    /// Model directory path
    pub path: PathBuf,
    /// Model configuration
    pub config: Option<ModelConfig>,
    /// SDF model definition
    pub sdf_model: SDFModel,
    /// Available mesh files
    pub meshes: Vec<MeshFile>,
    /// Available material files
    pub materials: Vec<MaterialFile>,
}

/// Mesh file entry
#[derive(Debug, Clone)]
pub struct MeshFile {
    pub path: PathBuf,
    pub mesh_type: MeshType,
}

/// Mesh file types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshType {
    Collada,
    Obj,
    Stl,
    Gltf,
}

/// Material file entry
#[derive(Debug, Clone)]
pub struct MaterialFile {
    pub path: PathBuf,
    pub file_type: MaterialFileType,
}

/// Material file types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialFileType {
    Texture(TextureType),
    OgreMaterial,
}

/// Texture types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TextureType {
    Image,
    NormalMap,
    SpecularMap,
}

// ============================================================================
// Fuel API Client (for downloading models)
// ============================================================================

/// Client for Gazebo Fuel API
pub struct FuelClient {
    /// API base URL
    base_url: String,
    /// Cache directory
    cache_dir: PathBuf,
}

impl FuelClient {
    /// Create a new Fuel client
    pub fn new() -> Self {
        let cache_dir = dirs::home_dir()
            .map(|h| h.join(".ignition/fuel"))
            .unwrap_or_else(|| PathBuf::from("/tmp/gazebo_fuel"));

        Self {
            base_url: "https://fuel.gazebosim.org".to_string(),
            cache_dir,
        }
    }

    /// Set custom API URL
    pub fn with_url(mut self, url: &str) -> Self {
        self.base_url = url.to_string();
        self
    }

    /// Set cache directory
    pub fn with_cache_dir<P: AsRef<Path>>(mut self, path: P) -> Self {
        self.cache_dir = path.as_ref().to_path_buf();
        self
    }

    /// Get cache directory
    pub fn cache_dir(&self) -> &Path {
        &self.cache_dir
    }

    /// Check if a model is cached
    pub fn is_cached(&self, owner: &str, name: &str, version: Option<&str>) -> bool {
        let path = self.model_cache_path(owner, name, version);
        path.exists()
    }

    /// Get cache path for a model
    pub fn model_cache_path(&self, owner: &str, name: &str, version: Option<&str>) -> PathBuf {
        let version = version.unwrap_or("tip");
        self.cache_dir
            .join(owner)
            .join("models")
            .join(name)
            .join(version)
    }

    /// Parse a Fuel URI
    pub fn parse_uri(uri: &str) -> Option<FuelUri> {
        if !uri.starts_with("fuel://") {
            return None;
        }

        let path = uri.trim_start_matches("fuel://");
        let parts: Vec<&str> = path.split('/').collect();

        if parts.len() < 4 {
            return None;
        }

        Some(FuelUri {
            server: parts[0].to_string(),
            owner: parts[1].to_string(),
            resource_type: parts[2].to_string(),
            name: parts[3].to_string(),
            version: parts.get(4).map(|s| s.to_string()),
        })
    }

    /// Download a model (async version would use reqwest)
    /// For now, this returns the expected cache path
    pub fn get_model_path(&self, uri: &FuelUri) -> PathBuf {
        self.model_cache_path(&uri.owner, &uri.name, uri.version.as_deref())
    }
}

impl Default for FuelClient {
    fn default() -> Self {
        Self::new()
    }
}

/// Parsed Fuel URI
#[derive(Debug, Clone)]
pub struct FuelUri {
    pub server: String,
    pub owner: String,
    pub resource_type: String,
    pub name: String,
    pub version: Option<String>,
}

// ============================================================================
// Bevy Plugin
// ============================================================================

/// Plugin for Gazebo model support
pub struct GazeboModelsPlugin;

impl Plugin for GazeboModelsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GazeboModelDatabase>()
            .add_event::<LoadGazeboModelEvent>()
            .add_event::<ScanModelsEvent>()
            .add_systems(Startup, initialize_model_database)
            .add_systems(Update, (handle_scan_models, handle_load_model));
    }
}

/// Resource for model database
#[derive(Resource, Default)]
pub struct GazeboModelDatabase {
    pub loader: Option<GazeboModelLoader>,
}

/// Event to trigger model database scan
#[derive(Event)]
pub struct ScanModelsEvent {
    pub additional_paths: Vec<PathBuf>,
}

/// Event to load a model
#[derive(Event)]
pub struct LoadGazeboModelEvent {
    pub name_or_uri: String,
    pub position: Vec3,
    pub rotation: Quat,
}

fn initialize_model_database(mut database: ResMut<GazeboModelDatabase>) {
    let mut loader = GazeboModelLoader::new();
    if let Err(e) = loader.scan() {
        warn!("Failed to scan model database: {}", e);
    }
    database.loader = Some(loader);
    info!("Gazebo model database initialized");
}

fn handle_scan_models(
    mut events: EventReader<ScanModelsEvent>,
    mut database: ResMut<GazeboModelDatabase>,
) {
    for event in events.read() {
        if let Some(ref mut loader) = database.loader {
            for path in &event.additional_paths {
                loader.add_search_path(path);
            }
            if let Err(e) = loader.scan() {
                error!("Failed to scan models: {}", e);
            }
        }
    }
}

fn handle_load_model(
    mut events: EventReader<LoadGazeboModelEvent>,
    database: Res<GazeboModelDatabase>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for event in events.read() {
        if let Some(ref loader) = database.loader {
            match loader.load(&event.name_or_uri) {
                Ok(model) => {
                    info!(
                        "Loaded Gazebo model '{}' with {} meshes, {} links",
                        model.name,
                        model.meshes.len(),
                        model.sdf_model.links.len()
                    );

                    // Spawn the model into the scene
                    match GazeboModelSpawner::spawn(
                        &model,
                        &mut commands,
                        event.position,
                        event.rotation,
                        &mut meshes,
                        &mut materials,
                    ) {
                        Ok(entity) => {
                            info!(
                                "Successfully spawned Gazebo model '{}' as entity {:?}",
                                model.name, entity
                            );
                        }
                        Err(e) => {
                            error!("Failed to spawn Gazebo model '{}': {}", model.name, e);
                        }
                    }
                }
                Err(e) => {
                    error!("Failed to load Gazebo model '{}': {}", event.name_or_uri, e);
                }
            }
        }
    }
}

// ============================================================================
// Model Spawner
// ============================================================================

/// Spawner for Gazebo models
pub struct GazeboModelSpawner;

impl GazeboModelSpawner {
    /// Spawn a loaded model into the scene
    pub fn spawn(
        model: &LoadedGazeboModel,
        commands: &mut Commands,
        position: Vec3,
        rotation: Quat,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        #[allow(unused_imports)]
        use crate::scene::sdf_importer::{SDFGeometry, SDFLink, SDFVisual};

        // Create parent entity for the model
        let parent_entity = commands
            .spawn((
                Transform::from_translation(position).with_rotation(rotation),
                GlobalTransform::default(),
                Name::new(model.name.clone()),
                GazeboModelComponent {
                    name: model.name.clone(),
                    sdf_model: model.sdf_model.clone(),
                },
            ))
            .id();

        // Spawn child entities for each link in the SDF model
        for link in &model.sdf_model.links {
            Self::spawn_link(
                commands,
                parent_entity,
                link,
                &model.path,
                meshes,
                materials,
            );
        }

        info!(
            "Spawned Gazebo model '{}' at {:?} with {} links",
            model.name,
            position,
            model.sdf_model.links.len()
        );

        Ok(parent_entity)
    }

    /// Spawn a single link as a child entity
    fn spawn_link(
        commands: &mut Commands,
        parent_entity: Entity,
        link: &crate::scene::sdf_importer::SDFLink,
        model_path: &Path,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) {
        #[allow(unused_imports)]
        use crate::scene::sdf_importer::SDFGeometry;

        // Convert SDF pose to Bevy transform
        let link_transform = link.pose.to_transform();

        // Create link entity
        let link_entity = commands
            .spawn((
                link_transform,
                GlobalTransform::default(),
                Name::new(format!("link_{}", link.name)),
                GazeboLinkComponent {
                    name: link.name.clone(),
                    mass: link.inertial.mass,
                },
            ))
            .id();

        // Make link a child of the parent model
        commands.entity(parent_entity).add_child(link_entity);

        // Spawn visual geometries for this link
        for visual in &link.visuals {
            Self::spawn_visual(commands, link_entity, visual, model_path, meshes, materials);
        }
    }

    /// Spawn a visual geometry as a child entity
    fn spawn_visual(
        commands: &mut Commands,
        link_entity: Entity,
        visual: &crate::scene::sdf_importer::SDFVisual,
        model_path: &Path,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) {
        #[allow(unused_imports)]
        use crate::scene::sdf_importer::SDFGeometry;

        // Convert SDF pose to Bevy transform
        let visual_transform = visual.pose.to_transform();

        // Create material from SDF material
        let color = Color::srgba(
            visual.material.diffuse[0],
            visual.material.diffuse[1],
            visual.material.diffuse[2],
            visual.material.diffuse[3],
        );

        let material_handle = materials.add(StandardMaterial {
            base_color: color,
            emissive: LinearRgba::new(
                visual.material.emissive[0],
                visual.material.emissive[1],
                visual.material.emissive[2],
                visual.material.emissive[3],
            ),
            ..default()
        });

        // Create mesh based on geometry type
        let mesh_handle = match &visual.geometry {
            SDFGeometry::Box { size } => {
                // Convert from SDF Z-up to Bevy Y-up
                let bevy_size = Vec3::new(size.x, size.z, size.y);
                meshes.add(Mesh::from(bevy::prelude::Cuboid::new(
                    bevy_size.x,
                    bevy_size.y,
                    bevy_size.z,
                )))
            }
            SDFGeometry::Cylinder { radius, length } => {
                meshes.add(Mesh::from(bevy::prelude::Cylinder {
                    radius: *radius,
                    half_height: *length / 2.0,
                }))
            }
            SDFGeometry::Sphere { radius } => {
                meshes.add(Mesh::from(bevy::prelude::Sphere { radius: *radius }))
            }
            SDFGeometry::Plane { normal: _, size } => {
                // Create a flat cuboid for plane
                meshes.add(Mesh::from(bevy::prelude::Cuboid::new(size.x, 0.01, size.y)))
            }
            SDFGeometry::Mesh { uri, scale } => {
                // For mesh URIs, try to resolve and load
                // For now, spawn a placeholder cube - mesh loading would require asset server
                warn!(
                    "Mesh geometry '{}' - using placeholder (full mesh loading requires AssetServer)",
                    uri
                );
                let placeholder_size = *scale * 0.1; // Small cube as placeholder
                meshes.add(Mesh::from(bevy::prelude::Cuboid::new(
                    placeholder_size.x.max(0.05),
                    placeholder_size.y.max(0.05),
                    placeholder_size.z.max(0.05),
                )))
            }
        };

        // Spawn the visual entity
        commands.entity(link_entity).with_children(|parent| {
            parent.spawn((
                Mesh3d(mesh_handle),
                MeshMaterial3d(material_handle),
                visual_transform,
                Name::new(format!("visual_{}", visual.name)),
            ));
        });
    }
}

/// Component for spawned Gazebo links
#[derive(Component)]
pub struct GazeboLinkComponent {
    pub name: String,
    pub mass: f32,
}

/// Component for spawned Gazebo models
#[derive(Component)]
pub struct GazeboModelComponent {
    pub name: String,
    pub sdf_model: SDFModel,
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_model_config() {
        let xml = r#"<?xml version="1.0"?>
<model>
    <name>test_robot</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>Test Author</name>
        <email>test@example.com</email>
    </author>
    <description>A test robot model</description>
    <thumbnail>thumbnail.png</thumbnail>
</model>"#;

        let db = ModelDatabase::new();
        let config = db.parse_model_config(xml, Path::new("test")).unwrap();

        assert_eq!(config.name, "test_robot");
        assert_eq!(config.version, "1.0");
        assert_eq!(config.sdf.path, "model.sdf");
        assert_eq!(config.description, "A test robot model");
        assert!(config.author.is_some());
    }

    #[test]
    fn test_fuel_uri_parsing() {
        let uri = "fuel://fuel.gazebosim.org/OpenRobotics/models/ground_plane/1";
        let parsed = FuelClient::parse_uri(uri).unwrap();

        assert_eq!(parsed.server, "fuel.gazebosim.org");
        assert_eq!(parsed.owner, "OpenRobotics");
        assert_eq!(parsed.resource_type, "models");
        assert_eq!(parsed.name, "ground_plane");
        assert_eq!(parsed.version, Some("1".to_string()));
    }

    #[test]
    fn test_model_database() {
        let mut db = ModelDatabase::new();
        db.add_search_path("/nonexistent/path");
        assert!(db.is_empty());

        // scan() should handle nonexistent paths gracefully
        let count = db.scan().unwrap();
        assert_eq!(count, 0);
    }
}
