//! Recent Files Management Module for sim3d
//!
//! Provides a comprehensive system for tracking and managing recently opened files,
//! including scenes, URDF/SDF files, and configuration files. Features include:
//! - Persistence to JSON
//! - Pinned favorites
//! - Filtering by file type
//! - Automatic handling of missing files

#![allow(dead_code)]

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

// ============================================================================
// File Types
// ============================================================================

/// Type of file tracked in the recent files list
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum RecentFileType {
    /// Scene file (.scene, .json)
    #[default]
    Scene,
    /// URDF robot description file
    Urdf,
    /// Xacro macro file for URDF (.xacro)
    Xacro,
    /// SDF simulation description format file (.sdf, .world)
    Sdf,
    /// MJCF MuJoCo XML format (.xml with mujoco root)
    Mjcf,
    /// SRDF semantic robot description (.srdf)
    Srdf,
    /// USD/USDA/USDC/USDZ scene files
    Usd,
    /// OpenDRIVE road network (.xodr)
    OpenDrive,
    /// OpenSCENARIO scenario files (.xosc)
    OpenScenario,
    /// Configuration file (.toml, .yaml)
    Config,
    /// GLTF/GLB 3D model file
    Gltf,
    /// FBX 3D model file
    Fbx,
    /// OBJ 3D model file
    Obj,
    /// STL 3D model file
    Stl,
    /// Collada DAE 3D model file
    Dae,
    /// Heightmap/terrain file (.png, .tiff for heightmaps)
    Heightmap,
    /// Other/unknown file type
    Other,
}

impl RecentFileType {
    /// Returns the icon representation for this file type
    pub fn icon(&self) -> &'static str {
        match self {
            RecentFileType::Scene => "[S]",
            RecentFileType::Urdf => "[U]",
            RecentFileType::Xacro => "[X]",
            RecentFileType::Sdf => "[D]",
            RecentFileType::Mjcf => "[M]",
            RecentFileType::Srdf => "[R]",
            RecentFileType::Usd => "[P]", // Pixar USD
            RecentFileType::OpenDrive => "[O]",
            RecentFileType::OpenScenario => "[N]",
            RecentFileType::Config => "[C]",
            RecentFileType::Gltf => "[G]",
            RecentFileType::Fbx => "[F]",
            RecentFileType::Obj => "[B]",
            RecentFileType::Stl => "[T]",
            RecentFileType::Dae => "[A]",
            RecentFileType::Heightmap => "[H]",
            RecentFileType::Other => "[?]",
        }
    }

    /// Returns a human-readable label for this file type
    pub fn label(&self) -> &'static str {
        match self {
            RecentFileType::Scene => "Scene",
            RecentFileType::Urdf => "URDF",
            RecentFileType::Xacro => "Xacro",
            RecentFileType::Sdf => "SDF",
            RecentFileType::Mjcf => "MJCF",
            RecentFileType::Srdf => "SRDF",
            RecentFileType::Usd => "USD",
            RecentFileType::OpenDrive => "OpenDRIVE",
            RecentFileType::OpenScenario => "OpenSCENARIO",
            RecentFileType::Config => "Config",
            RecentFileType::Gltf => "GLTF",
            RecentFileType::Fbx => "FBX",
            RecentFileType::Obj => "OBJ",
            RecentFileType::Stl => "STL",
            RecentFileType::Dae => "DAE",
            RecentFileType::Heightmap => "Heightmap",
            RecentFileType::Other => "Other",
        }
    }

    /// Infers the file type from a path's extension
    pub fn from_path(path: &Path) -> Self {
        match path
            .extension()
            .and_then(|e| e.to_str())
            .map(|e| e.to_lowercase())
        {
            Some(ext) => match ext.as_str() {
                // Scene formats
                "scene" | "scn" => RecentFileType::Scene,

                // Robot description formats
                "urdf" => RecentFileType::Urdf,
                "xacro" => RecentFileType::Xacro,
                "sdf" | "world" => RecentFileType::Sdf,
                "srdf" => RecentFileType::Srdf,

                // USD formats (Pixar Universal Scene Description)
                "usd" | "usda" | "usdc" | "usdz" => RecentFileType::Usd,

                // Automotive simulation formats
                "xodr" => RecentFileType::OpenDrive,
                "xosc" => RecentFileType::OpenScenario,

                // 3D model formats
                "gltf" | "glb" => RecentFileType::Gltf,
                "fbx" => RecentFileType::Fbx,
                "obj" => RecentFileType::Obj,
                "stl" => RecentFileType::Stl,
                "dae" => RecentFileType::Dae,

                // Configuration formats
                "toml" | "yaml" | "yml" | "cfg" => RecentFileType::Config,

                // Heightmap/terrain (image-based)
                "tiff" | "tif" => RecentFileType::Heightmap,

                // Ambiguous formats that need content inspection
                "xml" => {
                    // XML could be MJCF, URDF, SDF, etc. - check filename hints
                    let filename = path
                        .file_name()
                        .and_then(|n| n.to_str())
                        .map(|n| n.to_lowercase())
                        .unwrap_or_default();

                    if filename.contains("mujoco") || filename.contains("mjcf") {
                        RecentFileType::Mjcf
                    } else if filename.contains("urdf") {
                        RecentFileType::Urdf
                    } else if filename.contains("sdf") || filename.contains("world") {
                        RecentFileType::Sdf
                    } else {
                        // Default to MJCF for .xml in robotics context
                        RecentFileType::Mjcf
                    }
                }
                "json" => {
                    // Check if it's a scene JSON
                    if path
                        .file_name()
                        .and_then(|n| n.to_str())
                        .map(|n| n.contains("scene"))
                        .unwrap_or(false)
                    {
                        RecentFileType::Scene
                    } else {
                        RecentFileType::Config
                    }
                }
                "png" => {
                    // PNG could be heightmap or texture - check filename hints
                    let filename = path
                        .file_name()
                        .and_then(|n| n.to_str())
                        .map(|n| n.to_lowercase())
                        .unwrap_or_default();

                    if filename.contains("height")
                        || filename.contains("terrain")
                        || filename.contains("elevation")
                        || filename.contains("dem")
                    {
                        RecentFileType::Heightmap
                    } else {
                        RecentFileType::Other
                    }
                }
                _ => RecentFileType::Other,
            },
            None => RecentFileType::Other,
        }
    }

    /// Returns all file type variants
    pub fn all() -> &'static [RecentFileType] {
        &[
            RecentFileType::Scene,
            RecentFileType::Urdf,
            RecentFileType::Xacro,
            RecentFileType::Sdf,
            RecentFileType::Mjcf,
            RecentFileType::Srdf,
            RecentFileType::Usd,
            RecentFileType::OpenDrive,
            RecentFileType::OpenScenario,
            RecentFileType::Config,
            RecentFileType::Gltf,
            RecentFileType::Fbx,
            RecentFileType::Obj,
            RecentFileType::Stl,
            RecentFileType::Dae,
            RecentFileType::Heightmap,
            RecentFileType::Other,
        ]
    }

    /// Returns file extensions associated with this type
    pub fn extensions(&self) -> &'static [&'static str] {
        match self {
            RecentFileType::Scene => &["scene", "scn"],
            RecentFileType::Urdf => &["urdf"],
            RecentFileType::Xacro => &["xacro"],
            RecentFileType::Sdf => &["sdf", "world"],
            RecentFileType::Mjcf => &["xml"],
            RecentFileType::Srdf => &["srdf"],
            RecentFileType::Usd => &["usd", "usda", "usdc", "usdz"],
            RecentFileType::OpenDrive => &["xodr"],
            RecentFileType::OpenScenario => &["xosc"],
            RecentFileType::Config => &["toml", "yaml", "yml", "cfg", "json"],
            RecentFileType::Gltf => &["gltf", "glb"],
            RecentFileType::Fbx => &["fbx"],
            RecentFileType::Obj => &["obj"],
            RecentFileType::Stl => &["stl"],
            RecentFileType::Dae => &["dae"],
            RecentFileType::Heightmap => &["png", "tiff", "tif"],
            RecentFileType::Other => &[],
        }
    }

    /// Returns whether this file type represents a robot description
    pub fn is_robot_description(&self) -> bool {
        matches!(
            self,
            RecentFileType::Urdf
                | RecentFileType::Xacro
                | RecentFileType::Sdf
                | RecentFileType::Mjcf
                | RecentFileType::Srdf
        )
    }

    /// Returns whether this file type represents a 3D model
    pub fn is_3d_model(&self) -> bool {
        matches!(
            self,
            RecentFileType::Gltf
                | RecentFileType::Fbx
                | RecentFileType::Obj
                | RecentFileType::Stl
                | RecentFileType::Dae
        )
    }

    /// Returns whether this file type represents a world/scene
    pub fn is_world(&self) -> bool {
        matches!(
            self,
            RecentFileType::Scene
                | RecentFileType::Sdf
                | RecentFileType::Usd
                | RecentFileType::OpenDrive
                | RecentFileType::OpenScenario
        )
    }
}

// ============================================================================
// Recent File Entry
// ============================================================================

/// Represents a single recent file entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecentFile {
    /// Full path to the file
    pub path: PathBuf,
    /// Display name (usually filename without extension)
    pub name: String,
    /// Unix timestamp when last opened
    pub last_opened: u64,
    /// Type of file
    pub file_type: RecentFileType,
    /// Optional path to thumbnail image
    pub thumbnail_path: Option<PathBuf>,
    /// Whether this file is pinned as a favorite
    pub pinned: bool,
    /// Whether the file exists on disk (cached status)
    #[serde(skip)]
    pub available: bool,
    /// Optional metadata tags
    #[serde(default)]
    pub tags: Vec<String>,
}

impl RecentFile {
    /// Creates a new RecentFile entry from a path
    pub fn new(path: impl Into<PathBuf>) -> Self {
        let path = path.into();
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("Unknown")
            .to_string();
        let file_type = RecentFileType::from_path(&path);
        let available = path.exists();
        let last_opened = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);

        Self {
            path,
            name,
            last_opened,
            file_type,
            thumbnail_path: None,
            pinned: false,
            available,
            tags: Vec::new(),
        }
    }

    /// Creates a RecentFile with a custom name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Sets the file type explicitly
    pub fn with_type(mut self, file_type: RecentFileType) -> Self {
        self.file_type = file_type;
        self
    }

    /// Sets the thumbnail path
    pub fn with_thumbnail(mut self, thumbnail: impl Into<PathBuf>) -> Self {
        self.thumbnail_path = Some(thumbnail.into());
        self
    }

    /// Marks the file as pinned
    pub fn pinned(mut self) -> Self {
        self.pinned = true;
        self
    }

    /// Adds a tag to the file
    pub fn with_tag(mut self, tag: impl Into<String>) -> Self {
        self.tags.push(tag.into());
        self
    }

    /// Updates the last opened timestamp to now
    pub fn touch(&mut self) {
        self.last_opened = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(self.last_opened);
    }

    /// Checks if the file exists and updates availability status
    pub fn check_availability(&mut self) -> bool {
        self.available = self.path.exists();
        self.available
    }

    /// Returns a formatted timestamp string
    pub fn formatted_time(&self) -> String {
        use chrono::{Local, TimeZone};

        if let Some(dt) = Local.timestamp_opt(self.last_opened as i64, 0).single() {
            dt.format("%Y-%m-%d %H:%M").to_string()
        } else {
            "Unknown".to_string()
        }
    }

    /// Returns a relative time description (e.g., "2 hours ago")
    pub fn relative_time(&self) -> String {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);

        let diff = now.saturating_sub(self.last_opened);

        if diff < 60 {
            "Just now".to_string()
        } else if diff < 3600 {
            let mins = diff / 60;
            format!("{} minute{} ago", mins, if mins == 1 { "" } else { "s" })
        } else if diff < 86400 {
            let hours = diff / 3600;
            format!("{} hour{} ago", hours, if hours == 1 { "" } else { "s" })
        } else if diff < 604800 {
            let days = diff / 86400;
            format!("{} day{} ago", days, if days == 1 { "" } else { "s" })
        } else {
            let weeks = diff / 604800;
            format!("{} week{} ago", weeks, if weeks == 1 { "" } else { "s" })
        }
    }
}

impl PartialEq for RecentFile {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path
    }
}

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for the recent files system
#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct RecentFilesConfig {
    /// Maximum number of recent files to track
    pub max_recent_files: usize,
    /// Whether to show pinned files first in the list
    pub show_pinned_first: bool,
    /// Whether to group files by type
    pub group_by_type: bool,
    /// Whether to clear non-pinned files on exit
    pub clear_on_exit: bool,
    /// Path to the recent files storage file
    pub storage_path: PathBuf,
    /// Whether to automatically save changes
    pub auto_save: bool,
    /// Whether to show thumbnails (if available)
    pub show_thumbnails: bool,
    /// Whether to show unavailable (deleted) files grayed out
    pub show_unavailable: bool,
    /// Maximum items to show in the File > Recent menu
    pub max_menu_items: usize,
}

impl Default for RecentFilesConfig {
    fn default() -> Self {
        Self {
            max_recent_files: 20,
            show_pinned_first: true,
            group_by_type: false,
            clear_on_exit: false,
            storage_path: Self::default_storage_path(),
            auto_save: true,
            show_thumbnails: true,
            show_unavailable: true,
            max_menu_items: 10,
        }
    }
}

impl RecentFilesConfig {
    /// Creates a new configuration with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the default storage path for recent files data
    pub fn default_storage_path() -> PathBuf {
        dirs::data_local_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("sim3d")
            .join("recent_files.json")
    }

    /// Sets the maximum recent files count
    pub fn with_max_files(mut self, max: usize) -> Self {
        self.max_recent_files = max.max(1);
        self
    }

    /// Sets whether to show pinned first
    pub fn with_pinned_first(mut self, enabled: bool) -> Self {
        self.show_pinned_first = enabled;
        self
    }

    /// Sets whether to group by type
    pub fn with_group_by_type(mut self, enabled: bool) -> Self {
        self.group_by_type = enabled;
        self
    }

    /// Sets the storage path
    pub fn with_storage_path(mut self, path: impl Into<PathBuf>) -> Self {
        self.storage_path = path.into();
        self
    }

    /// Sets clear on exit behavior
    pub fn with_clear_on_exit(mut self, enabled: bool) -> Self {
        self.clear_on_exit = enabled;
        self
    }
}

// ============================================================================
// Recent Files Manager
// ============================================================================

/// Manages the list of recently opened files
#[derive(Resource, Debug, Default)]
pub struct RecentFilesManager {
    /// List of recent files
    files: Vec<RecentFile>,
    /// Whether there are unsaved changes
    dirty: bool,
    /// Cache of file paths for quick lookup
    path_index: HashMap<PathBuf, usize>,
}

impl RecentFilesManager {
    /// Creates a new empty manager
    pub fn new() -> Self {
        Self {
            files: Vec::new(),
            dirty: false,
            path_index: HashMap::new(),
        }
    }

    /// Rebuilds the path index cache
    fn rebuild_index(&mut self) {
        self.path_index.clear();
        for (i, file) in self.files.iter().enumerate() {
            self.path_index.insert(file.path.clone(), i);
        }
    }

    /// Adds a file to the recent list, moving it to front if already present
    pub fn add_recent(&mut self, file: RecentFile) {
        // Check if file already exists
        if let Some(&idx) = self.path_index.get(&file.path) {
            // Move to front and update timestamp
            let mut existing = self.files.remove(idx);
            existing.touch();
            // Preserve pinned status
            let was_pinned = existing.pinned;
            self.files.insert(0, existing);
            if was_pinned {
                self.files[0].pinned = true;
            }
        } else {
            // Add new file at front
            self.files.insert(0, file);
        }

        self.dirty = true;
        self.rebuild_index();
    }

    /// Adds a file by path, inferring type and name
    pub fn add_path(&mut self, path: impl Into<PathBuf>) {
        let file = RecentFile::new(path);
        self.add_recent(file);
    }

    /// Removes a file from the recent list
    pub fn remove_recent(&mut self, path: &Path) -> Option<RecentFile> {
        if let Some(&idx) = self.path_index.get(path) {
            let file = self.files.remove(idx);
            self.dirty = true;
            self.rebuild_index();
            Some(file)
        } else {
            None
        }
    }

    /// Clears all non-pinned files
    pub fn clear_recent(&mut self) {
        self.files.retain(|f| f.pinned);
        self.dirty = true;
        self.rebuild_index();
    }

    /// Clears all files including pinned ones
    pub fn clear_all(&mut self) {
        self.files.clear();
        self.dirty = true;
        self.path_index.clear();
    }

    /// Pins a file (marks as favorite)
    pub fn pin_file(&mut self, path: &Path) -> bool {
        if let Some(&idx) = self.path_index.get(path) {
            self.files[idx].pinned = true;
            self.dirty = true;
            true
        } else {
            false
        }
    }

    /// Unpins a file
    pub fn unpin_file(&mut self, path: &Path) -> bool {
        if let Some(&idx) = self.path_index.get(path) {
            self.files[idx].pinned = false;
            self.dirty = true;
            true
        } else {
            false
        }
    }

    /// Toggles the pinned state of a file
    pub fn toggle_pin(&mut self, path: &Path) -> Option<bool> {
        if let Some(&idx) = self.path_index.get(path) {
            self.files[idx].pinned = !self.files[idx].pinned;
            self.dirty = true;
            Some(self.files[idx].pinned)
        } else {
            None
        }
    }

    /// Gets all recent files, optionally filtered by type
    pub fn get_recent(&self, file_type: Option<RecentFileType>) -> Vec<&RecentFile> {
        self.files
            .iter()
            .filter(|f| file_type.is_none() || f.file_type == file_type.unwrap())
            .collect()
    }

    /// Gets recent files sorted according to config settings
    pub fn get_sorted(&self, config: &RecentFilesConfig) -> Vec<&RecentFile> {
        let mut files: Vec<&RecentFile> = self.files.iter().collect();

        if config.show_pinned_first {
            files.sort_by(|a, b| {
                // First sort by pinned (pinned first)
                b.pinned
                    .cmp(&a.pinned)
                    // Then by timestamp (most recent first)
                    .then(b.last_opened.cmp(&a.last_opened))
            });
        } else {
            files.sort_by(|a, b| b.last_opened.cmp(&a.last_opened));
        }

        if !config.show_unavailable {
            files.retain(|f| f.available);
        }

        files.truncate(config.max_recent_files);
        files
    }

    /// Gets only pinned files
    pub fn get_pinned(&self) -> Vec<&RecentFile> {
        self.files.iter().filter(|f| f.pinned).collect()
    }

    /// Gets files grouped by type
    pub fn get_grouped(&self) -> HashMap<RecentFileType, Vec<&RecentFile>> {
        let mut groups: HashMap<RecentFileType, Vec<&RecentFile>> = HashMap::new();

        for file in &self.files {
            groups.entry(file.file_type).or_default().push(file);
        }

        // Sort each group by last opened
        for group in groups.values_mut() {
            group.sort_by(|a, b| b.last_opened.cmp(&a.last_opened));
        }

        groups
    }

    /// Gets a specific file by path
    pub fn get(&self, path: &Path) -> Option<&RecentFile> {
        self.path_index.get(path).map(|&idx| &self.files[idx])
    }

    /// Gets a mutable reference to a specific file by path
    pub fn get_mut(&mut self, path: &Path) -> Option<&mut RecentFile> {
        if let Some(&idx) = self.path_index.get(path) {
            Some(&mut self.files[idx])
        } else {
            None
        }
    }

    /// Returns the number of recent files
    pub fn count(&self) -> usize {
        self.files.len()
    }

    /// Returns the number of pinned files
    pub fn pinned_count(&self) -> usize {
        self.files.iter().filter(|f| f.pinned).count()
    }

    /// Checks if the list is empty
    pub fn is_empty(&self) -> bool {
        self.files.is_empty()
    }

    /// Checks if there are unsaved changes
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Marks as clean (after saving)
    pub fn mark_clean(&mut self) {
        self.dirty = false;
    }

    /// Checks availability of all files and removes unavailable ones if requested
    pub fn check_all_availability(&mut self, remove_unavailable: bool) {
        for file in &mut self.files {
            file.check_availability();
        }

        if remove_unavailable {
            self.files.retain(|f| f.available);
            self.rebuild_index();
        }

        self.dirty = true;
    }

    /// Trims the list to a maximum size, keeping pinned files
    pub fn trim_to(&mut self, max_count: usize) {
        if self.files.len() <= max_count {
            return;
        }

        // Sort by pinned first, then by last_opened
        self.files.sort_by(|a, b| {
            b.pinned
                .cmp(&a.pinned)
                .then(b.last_opened.cmp(&a.last_opened))
        });

        self.files.truncate(max_count);
        self.dirty = true;
        self.rebuild_index();
    }

    /// Saves the recent files list to a JSON file
    pub fn save(&mut self, path: &Path) -> Result<(), RecentFilesError> {
        // Ensure parent directory exists
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| RecentFilesError::IoError(e.to_string()))?;
        }

        let data = RecentFilesData {
            version: 1,
            files: self.files.clone(),
        };

        let json = serde_json::to_string_pretty(&data)
            .map_err(|e| RecentFilesError::SerializationError(e.to_string()))?;

        fs::write(path, json).map_err(|e| RecentFilesError::IoError(e.to_string()))?;

        self.dirty = false;
        Ok(())
    }

    /// Loads the recent files list from a JSON file
    pub fn load(path: &Path) -> Result<Self, RecentFilesError> {
        if !path.exists() {
            return Ok(Self::new());
        }

        let json =
            fs::read_to_string(path).map_err(|e| RecentFilesError::IoError(e.to_string()))?;

        let data: RecentFilesData = serde_json::from_str(&json)
            .map_err(|e| RecentFilesError::DeserializationError(e.to_string()))?;

        let mut manager = Self {
            files: data.files,
            dirty: false,
            path_index: HashMap::new(),
        };

        // Check availability of all files
        for file in &mut manager.files {
            file.check_availability();
        }

        manager.rebuild_index();
        Ok(manager)
    }
}

/// Serializable data structure for recent files persistence
#[derive(Debug, Serialize, Deserialize)]
struct RecentFilesData {
    version: u32,
    files: Vec<RecentFile>,
}

// ============================================================================
// Errors
// ============================================================================

/// Errors that can occur in the recent files system
#[derive(Debug, Clone, thiserror::Error)]
pub enum RecentFilesError {
    #[error("I/O error: {0}")]
    IoError(String),
    #[error("Serialization error: {0}")]
    SerializationError(String),
    #[error("Deserialization error: {0}")]
    DeserializationError(String),
    #[error("File not found: {0}")]
    FileNotFound(String),
}

// ============================================================================
// Events
// ============================================================================

/// Event fired when a recent file is selected
#[derive(Event, Debug, Clone)]
pub struct RecentFileSelectedEvent {
    /// Path to the selected file
    pub path: PathBuf,
    /// Type of file
    pub file_type: RecentFileType,
}

/// Event to add a file to recent list
#[derive(Event, Debug, Clone)]
pub struct AddRecentFileEvent {
    /// The file to add
    pub file: RecentFile,
}

impl AddRecentFileEvent {
    /// Creates an event from a path
    pub fn from_path(path: impl Into<PathBuf>) -> Self {
        Self {
            file: RecentFile::new(path),
        }
    }
}

/// Event to clear recent files
#[derive(Event, Debug, Clone)]
pub struct ClearRecentFilesEvent {
    /// Whether to also clear pinned files
    pub include_pinned: bool,
}

// ============================================================================
// Systems
// ============================================================================

/// System to handle add recent file events
pub fn handle_add_recent_file_events(
    mut events: EventReader<AddRecentFileEvent>,
    mut manager: ResMut<RecentFilesManager>,
    config: Res<RecentFilesConfig>,
) {
    for event in events.read() {
        manager.add_recent(event.file.clone());

        // Trim to max if needed
        if manager.count() > config.max_recent_files {
            manager.trim_to(config.max_recent_files);
        }

        // Auto-save if enabled
        if config.auto_save {
            if let Err(e) = manager.save(&config.storage_path) {
                tracing::warn!("Failed to save recent files: {}", e);
            }
        }
    }
}

/// System to handle clear recent files events
pub fn handle_clear_recent_files_events(
    mut events: EventReader<ClearRecentFilesEvent>,
    mut manager: ResMut<RecentFilesManager>,
    config: Res<RecentFilesConfig>,
) {
    for event in events.read() {
        if event.include_pinned {
            manager.clear_all();
        } else {
            manager.clear_recent();
        }

        // Auto-save if enabled
        if config.auto_save {
            if let Err(e) = manager.save(&config.storage_path) {
                tracing::warn!("Failed to save recent files: {}", e);
            }
        }
    }
}

/// System to auto-save recent files on change
pub fn auto_save_recent_files_system(
    mut manager: ResMut<RecentFilesManager>,
    config: Res<RecentFilesConfig>,
) {
    if config.auto_save && manager.is_dirty() {
        if let Err(e) = manager.save(&config.storage_path) {
            tracing::warn!("Failed to auto-save recent files: {}", e);
        }
    }
}

/// System to load recent files on startup
fn startup_load_recent_files(mut commands: Commands, config: Res<RecentFilesConfig>) {
    let manager = match RecentFilesManager::load(&config.storage_path) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!("Failed to load recent files: {}", e);
            RecentFilesManager::new()
        }
    };

    commands.insert_resource(manager);
}

/// System to save recent files on exit (if clear_on_exit is false)
pub fn on_exit_save_recent_files(
    mut manager: ResMut<RecentFilesManager>,
    config: Res<RecentFilesConfig>,
) {
    if config.clear_on_exit {
        manager.clear_recent();
    }

    if let Err(e) = manager.save(&config.storage_path) {
        tracing::error!("Failed to save recent files on exit: {}", e);
    }
}

// ============================================================================
// UI Rendering (Feature-gated)
// ============================================================================

#[cfg(feature = "visual")]
use bevy_egui::egui;

#[cfg(feature = "visual")]
/// Renders the recent files menu
pub fn render_recent_files_menu(
    ui: &mut egui::Ui,
    manager: &RecentFilesManager,
    config: &RecentFilesConfig,
) -> Option<PathBuf> {
    let mut selected_path: Option<PathBuf> = None;
    let files = manager.get_sorted(config);

    if files.is_empty() {
        ui.label("No recent files");
        return None;
    }

    let display_files: Vec<_> = files.iter().take(config.max_menu_items).collect();

    if config.group_by_type {
        // Group by type
        let grouped = manager.get_grouped();

        for file_type in RecentFileType::all() {
            if let Some(type_files) = grouped.get(file_type) {
                if !type_files.is_empty() {
                    ui.menu_button(file_type.label(), |ui| {
                        for file in type_files.iter().take(config.max_menu_items) {
                            let label = format_file_label(file, config);
                            if ui.button(&label).clicked() {
                                selected_path = Some(file.path.clone());
                                ui.close_menu();
                            }
                        }
                    });
                }
            }
        }
    } else {
        // Flat list
        for file in display_files {
            let label = format_file_label(file, config);
            let enabled = file.available || config.show_unavailable;

            if ui.add_enabled(enabled, egui::Button::new(&label)).clicked() {
                if file.available {
                    selected_path = Some(file.path.clone());
                }
                ui.close_menu();
            }
        }
    }

    ui.separator();

    if ui.button("Clear Recent Files").clicked() {
        // This should trigger a ClearRecentFilesEvent
        ui.close_menu();
    }

    selected_path
}

#[cfg(feature = "visual")]
fn format_file_label(file: &RecentFile, _config: &RecentFilesConfig) -> String {
    let pin_marker = if file.pinned { "*" } else { "" };
    let avail_marker = if !file.available { " (missing)" } else { "" };
    format!(
        "{}{} {}{}",
        pin_marker,
        file.file_type.icon(),
        file.name,
        avail_marker
    )
}

// ============================================================================
// Plugin
// ============================================================================

/// Plugin for recent files management
#[derive(Default)]
pub struct RecentFilesPlugin {
    /// Initial configuration
    config: RecentFilesConfig,
}

impl RecentFilesPlugin {
    /// Creates a new plugin with default configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a new plugin with custom configuration
    pub fn with_config(config: RecentFilesConfig) -> Self {
        Self { config }
    }
}

impl Plugin for RecentFilesPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone())
            .init_resource::<RecentFilesManager>()
            .add_event::<RecentFileSelectedEvent>()
            .add_event::<AddRecentFileEvent>()
            .add_event::<ClearRecentFilesEvent>()
            .add_systems(Startup, startup_load_recent_files)
            .add_systems(
                Update,
                (
                    handle_add_recent_file_events,
                    handle_clear_recent_files_events,
                ),
            );
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn create_test_file(name: &str, file_type: RecentFileType) -> RecentFile {
        let path = PathBuf::from(format!(
            "/test/{}.{}",
            name,
            match file_type {
                RecentFileType::Scene => "scene",
                RecentFileType::Urdf => "urdf",
                RecentFileType::Xacro => "xacro",
                RecentFileType::Sdf => "sdf",
                RecentFileType::Mjcf => "xml",
                RecentFileType::Srdf => "srdf",
                RecentFileType::Usd => "usd",
                RecentFileType::OpenDrive => "xodr",
                RecentFileType::OpenScenario => "xosc",
                RecentFileType::Config => "toml",
                RecentFileType::Gltf => "gltf",
                RecentFileType::Fbx => "fbx",
                RecentFileType::Obj => "obj",
                RecentFileType::Stl => "stl",
                RecentFileType::Dae => "dae",
                RecentFileType::Heightmap => "tiff",
                RecentFileType::Other => "txt",
            }
        ));
        RecentFile::new(path).with_type(file_type)
    }

    #[test]
    fn test_recent_file_creation() {
        let file = RecentFile::new("/test/scene.scene");

        assert_eq!(file.name, "scene");
        assert_eq!(file.file_type, RecentFileType::Scene);
        assert!(!file.pinned);
        assert!(file.last_opened > 0);
    }

    #[test]
    fn test_recent_file_type_inference() {
        let cases = vec![
            // Scene formats
            ("test.scene", RecentFileType::Scene),
            ("level.scn", RecentFileType::Scene),
            // Robot description formats
            ("robot.urdf", RecentFileType::Urdf),
            ("robot.xacro", RecentFileType::Xacro),
            ("world.sdf", RecentFileType::Sdf),
            ("environment.world", RecentFileType::Sdf),
            ("semantic.srdf", RecentFileType::Srdf),
            // USD formats
            ("stage.usd", RecentFileType::Usd),
            ("stage.usda", RecentFileType::Usd),
            ("stage.usdc", RecentFileType::Usd),
            ("archive.usdz", RecentFileType::Usd),
            // Automotive formats
            ("road_network.xodr", RecentFileType::OpenDrive),
            ("scenario.xosc", RecentFileType::OpenScenario),
            // 3D model formats
            ("model.gltf", RecentFileType::Gltf),
            ("model.glb", RecentFileType::Gltf),
            ("mesh.fbx", RecentFileType::Fbx),
            ("mesh.obj", RecentFileType::Obj),
            ("part.stl", RecentFileType::Stl),
            ("collada.dae", RecentFileType::Dae),
            // Config formats
            ("config.toml", RecentFileType::Config),
            ("settings.yaml", RecentFileType::Config),
            // Heightmap formats
            ("terrain.tiff", RecentFileType::Heightmap),
            ("heightmap_data.png", RecentFileType::Heightmap),
            // Unknown
            ("unknown.xyz", RecentFileType::Other),
        ];

        for (path, expected_type) in cases {
            let inferred = RecentFileType::from_path(Path::new(path));
            assert_eq!(inferred, expected_type, "Failed for path: {}", path);
        }
    }

    #[test]
    fn test_recent_file_with_modifiers() {
        let file = RecentFile::new("/test/file.scene")
            .with_name("Custom Name")
            .with_type(RecentFileType::Config)
            .with_tag("important")
            .pinned();

        assert_eq!(file.name, "Custom Name");
        assert_eq!(file.file_type, RecentFileType::Config);
        assert!(file.pinned);
        assert_eq!(file.tags, vec!["important"]);
    }

    #[test]
    fn test_manager_add_recent() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Urdf);

        manager.add_recent(file1.clone());
        manager.add_recent(file2.clone());

        assert_eq!(manager.count(), 2);

        // Most recent should be first
        let recent = manager.get_recent(None);
        assert_eq!(recent[0].name, "file2");
        assert_eq!(recent[1].name, "file1");
    }

    #[test]
    fn test_manager_add_existing_moves_to_front() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Scene);

        manager.add_recent(file1.clone());
        manager.add_recent(file2.clone());

        // Add file1 again
        manager.add_recent(file1.clone());

        // Should still have 2 files, with file1 now first
        assert_eq!(manager.count(), 2);
        let recent = manager.get_recent(None);
        assert_eq!(recent[0].name, "file1");
    }

    #[test]
    fn test_manager_remove_recent() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Scene);

        manager.add_recent(file1.clone());
        manager.add_recent(file2.clone());

        let removed = manager.remove_recent(&file1.path);

        assert!(removed.is_some());
        assert_eq!(removed.unwrap().name, "file1");
        assert_eq!(manager.count(), 1);
    }

    #[test]
    fn test_manager_clear_preserves_pinned() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Scene).pinned();

        manager.add_recent(file1);
        manager.add_recent(file2);

        manager.clear_recent();

        assert_eq!(manager.count(), 1);
        assert_eq!(manager.files[0].name, "file2");
    }

    #[test]
    fn test_manager_clear_all() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Scene).pinned();

        manager.add_recent(file1);
        manager.add_recent(file2);

        manager.clear_all();

        assert!(manager.is_empty());
    }

    #[test]
    fn test_manager_pin_unpin() {
        let mut manager = RecentFilesManager::new();

        let file = create_test_file("file1", RecentFileType::Scene);
        manager.add_recent(file.clone());

        assert!(!manager.get(&file.path).unwrap().pinned);

        manager.pin_file(&file.path);
        assert!(manager.get(&file.path).unwrap().pinned);

        manager.unpin_file(&file.path);
        assert!(!manager.get(&file.path).unwrap().pinned);
    }

    #[test]
    fn test_manager_toggle_pin() {
        let mut manager = RecentFilesManager::new();

        let file = create_test_file("file1", RecentFileType::Scene);
        manager.add_recent(file.clone());

        let result = manager.toggle_pin(&file.path);
        assert_eq!(result, Some(true));

        let result = manager.toggle_pin(&file.path);
        assert_eq!(result, Some(false));
    }

    #[test]
    fn test_manager_get_pinned() {
        let mut manager = RecentFilesManager::new();

        let file1 = create_test_file("file1", RecentFileType::Scene);
        let file2 = create_test_file("file2", RecentFileType::Scene).pinned();
        let file3 = create_test_file("file3", RecentFileType::Scene).pinned();

        manager.add_recent(file1);
        manager.add_recent(file2);
        manager.add_recent(file3);

        let pinned = manager.get_pinned();
        assert_eq!(pinned.len(), 2);
    }

    #[test]
    fn test_manager_filter_by_type() {
        let mut manager = RecentFilesManager::new();

        manager.add_recent(create_test_file("scene1", RecentFileType::Scene));
        manager.add_recent(create_test_file("scene2", RecentFileType::Scene));
        manager.add_recent(create_test_file("robot", RecentFileType::Urdf));

        let scenes = manager.get_recent(Some(RecentFileType::Scene));
        assert_eq!(scenes.len(), 2);

        let urdfs = manager.get_recent(Some(RecentFileType::Urdf));
        assert_eq!(urdfs.len(), 1);
    }

    #[test]
    fn test_manager_get_grouped() {
        let mut manager = RecentFilesManager::new();

        manager.add_recent(create_test_file("scene1", RecentFileType::Scene));
        manager.add_recent(create_test_file("robot", RecentFileType::Urdf));
        manager.add_recent(create_test_file("scene2", RecentFileType::Scene));

        let grouped = manager.get_grouped();

        assert_eq!(grouped.get(&RecentFileType::Scene).unwrap().len(), 2);
        assert_eq!(grouped.get(&RecentFileType::Urdf).unwrap().len(), 1);
    }

    #[test]
    fn test_manager_trim_to() {
        let mut manager = RecentFilesManager::new();

        for i in 0..10 {
            manager.add_recent(create_test_file(
                &format!("file{}", i),
                RecentFileType::Scene,
            ));
        }

        assert_eq!(manager.count(), 10);

        manager.trim_to(5);

        assert_eq!(manager.count(), 5);
    }

    #[test]
    fn test_manager_trim_preserves_pinned() {
        let mut manager = RecentFilesManager::new();

        // Add files - oldest first (they get pushed to back)
        manager.add_recent(create_test_file("old_pinned", RecentFileType::Scene).pinned());

        for i in 0..9 {
            manager.add_recent(create_test_file(
                &format!("file{}", i),
                RecentFileType::Scene,
            ));
        }

        manager.trim_to(5);

        // Pinned file should still be there
        assert!(manager.get(Path::new("/test/old_pinned.scene")).is_some());
    }

    #[test]
    fn test_manager_save_load() {
        let temp_dir = TempDir::new().unwrap();
        let save_path = temp_dir.path().join("recent_files.json");

        let mut manager = RecentFilesManager::new();
        manager.add_recent(create_test_file("file1", RecentFileType::Scene).pinned());
        manager.add_recent(create_test_file("file2", RecentFileType::Urdf));

        // Save
        manager.save(&save_path).unwrap();

        // Load
        let loaded = RecentFilesManager::load(&save_path).unwrap();

        assert_eq!(loaded.count(), 2);
        assert!(loaded.get(Path::new("/test/file1.scene")).unwrap().pinned);
    }

    #[test]
    fn test_manager_load_nonexistent() {
        let result = RecentFilesManager::load(Path::new("/nonexistent/path.json"));

        // Should return empty manager, not error
        assert!(result.is_ok());
        assert!(result.unwrap().is_empty());
    }

    #[test]
    fn test_manager_dirty_tracking() {
        let mut manager = RecentFilesManager::new();

        assert!(!manager.is_dirty());

        manager.add_recent(create_test_file("file", RecentFileType::Scene));
        assert!(manager.is_dirty());

        manager.mark_clean();
        assert!(!manager.is_dirty());

        manager.pin_file(Path::new("/test/file.scene"));
        assert!(manager.is_dirty());
    }

    #[test]
    fn test_config_default() {
        let config = RecentFilesConfig::default();

        assert_eq!(config.max_recent_files, 20);
        assert!(config.show_pinned_first);
        assert!(!config.group_by_type);
        assert!(!config.clear_on_exit);
        assert!(config.auto_save);
    }

    #[test]
    fn test_config_with_modifiers() {
        let config = RecentFilesConfig::new()
            .with_max_files(50)
            .with_pinned_first(false)
            .with_group_by_type(true)
            .with_clear_on_exit(true);

        assert_eq!(config.max_recent_files, 50);
        assert!(!config.show_pinned_first);
        assert!(config.group_by_type);
        assert!(config.clear_on_exit);
    }

    #[test]
    fn test_config_max_files_minimum() {
        let config = RecentFilesConfig::new().with_max_files(0);

        // Should be clamped to at least 1
        assert_eq!(config.max_recent_files, 1);
    }

    #[test]
    fn test_file_type_icons() {
        assert_eq!(RecentFileType::Scene.icon(), "[S]");
        assert_eq!(RecentFileType::Urdf.icon(), "[U]");
        assert_eq!(RecentFileType::Xacro.icon(), "[X]");
        assert_eq!(RecentFileType::Sdf.icon(), "[D]");
        assert_eq!(RecentFileType::Mjcf.icon(), "[M]");
        assert_eq!(RecentFileType::Srdf.icon(), "[R]");
        assert_eq!(RecentFileType::Usd.icon(), "[P]");
        assert_eq!(RecentFileType::OpenDrive.icon(), "[O]");
        assert_eq!(RecentFileType::OpenScenario.icon(), "[N]");
        assert_eq!(RecentFileType::Config.icon(), "[C]");
        assert_eq!(RecentFileType::Gltf.icon(), "[G]");
        assert_eq!(RecentFileType::Fbx.icon(), "[F]");
        assert_eq!(RecentFileType::Other.icon(), "[?]");
    }

    #[test]
    fn test_file_type_labels() {
        assert_eq!(RecentFileType::Scene.label(), "Scene");
        assert_eq!(RecentFileType::Urdf.label(), "URDF");
        assert_eq!(RecentFileType::Xacro.label(), "Xacro");
        assert_eq!(RecentFileType::Sdf.label(), "SDF");
        assert_eq!(RecentFileType::Mjcf.label(), "MJCF");
        assert_eq!(RecentFileType::Usd.label(), "USD");
        assert_eq!(RecentFileType::OpenDrive.label(), "OpenDRIVE");
        assert_eq!(RecentFileType::OpenScenario.label(), "OpenSCENARIO");
        assert_eq!(RecentFileType::Gltf.label(), "GLTF");
    }

    #[test]
    fn test_file_type_category_helpers() {
        // Robot descriptions
        assert!(RecentFileType::Urdf.is_robot_description());
        assert!(RecentFileType::Xacro.is_robot_description());
        assert!(RecentFileType::Mjcf.is_robot_description());
        assert!(!RecentFileType::Gltf.is_robot_description());

        // 3D models
        assert!(RecentFileType::Gltf.is_3d_model());
        assert!(RecentFileType::Fbx.is_3d_model());
        assert!(RecentFileType::Obj.is_3d_model());
        assert!(!RecentFileType::Urdf.is_3d_model());

        // World/scene formats
        assert!(RecentFileType::Scene.is_world());
        assert!(RecentFileType::Usd.is_world());
        assert!(RecentFileType::OpenDrive.is_world());
        assert!(!RecentFileType::Gltf.is_world());
    }

    #[test]
    fn test_recent_file_touch() {
        let mut file = RecentFile::new("/test/file.scene");
        let original_time = file.last_opened;

        // Small delay to ensure timestamp changes
        std::thread::sleep(std::time::Duration::from_millis(10));

        file.touch();

        assert!(file.last_opened >= original_time);
    }

    #[test]
    fn test_recent_file_relative_time() {
        let mut file = RecentFile::new("/test/file.scene");

        // Just now
        file.last_opened = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        assert_eq!(file.relative_time(), "Just now");

        // Minutes ago
        file.last_opened = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs()
            - 120;
        assert!(file.relative_time().contains("minute"));

        // Hours ago
        file.last_opened = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs()
            - 7200;
        assert!(file.relative_time().contains("hour"));
    }

    #[test]
    fn test_manager_get_sorted_with_config() {
        let mut manager = RecentFilesManager::new();

        // Add files with different timestamps
        let mut file1 = create_test_file("old", RecentFileType::Scene);
        file1.last_opened = 1000;

        let mut file2 = create_test_file("new", RecentFileType::Scene);
        file2.last_opened = 2000;

        let mut file3 = create_test_file("pinned", RecentFileType::Scene).pinned();
        file3.last_opened = 500; // Oldest but pinned

        manager.add_recent(file1);
        manager.add_recent(file2);
        manager.add_recent(file3);

        // With show_pinned_first = true
        let config = RecentFilesConfig::new().with_pinned_first(true);
        let sorted = manager.get_sorted(&config);

        // Pinned should be first even though it's oldest
        assert_eq!(sorted[0].name, "pinned");
    }

    #[test]
    fn test_add_recent_file_event() {
        let event = AddRecentFileEvent::from_path("/test/scene.scene");

        assert_eq!(event.file.path, PathBuf::from("/test/scene.scene"));
        assert_eq!(event.file.file_type, RecentFileType::Scene);
    }

    #[test]
    fn test_pinned_count() {
        let mut manager = RecentFilesManager::new();

        manager.add_recent(create_test_file("file1", RecentFileType::Scene));
        manager.add_recent(create_test_file("file2", RecentFileType::Scene).pinned());
        manager.add_recent(create_test_file("file3", RecentFileType::Scene).pinned());

        assert_eq!(manager.count(), 3);
        assert_eq!(manager.pinned_count(), 2);
    }

    #[test]
    fn test_add_path_convenience() {
        let mut manager = RecentFilesManager::new();

        manager.add_path("/test/robot.urdf");

        assert_eq!(manager.count(), 1);
        assert_eq!(manager.files[0].file_type, RecentFileType::Urdf);
    }

    #[test]
    fn test_get_mut() {
        let mut manager = RecentFilesManager::new();

        let file = create_test_file("file", RecentFileType::Scene);
        let path = file.path.clone();
        manager.add_recent(file);

        // Modify via get_mut
        if let Some(f) = manager.get_mut(&path) {
            f.name = "Modified Name".to_string();
        }

        assert_eq!(manager.get(&path).unwrap().name, "Modified Name");
    }
}
