//! File Dialog Integration for World/Scene Loading
//!
//! Provides a unified file dialog for loading various world and scene formats.
//! Integrates with the WorldLoader and RecentFiles systems.

use bevy::prelude::*;
use std::path::PathBuf;

use crate::scene::world_loader::{LoadWorldEvent, WorldLoader};
use crate::ui::recent_files::{AddRecentFileEvent, RecentFile, RecentFileType};

/// File dialog state
#[derive(Resource, Default)]
pub struct FileDialogState {
    /// Whether a file dialog is currently open
    pub is_open: bool,
    /// The selected file path (if any)
    pub selected_path: Option<PathBuf>,
    /// Current filter category
    pub filter: FileFilter,
    /// Error message from last operation
    pub error: Option<String>,
}

/// File filter categories for the dialog
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FileFilter {
    /// Show all supported files
    #[default]
    All,
    /// Show only scene/world files
    Worlds,
    /// Show only robot description files
    Robots,
    /// Show only 3D model files
    Models,
    /// Show only road/scenario files
    Automotive,
}

impl FileFilter {
    /// Get the filter name for display
    pub fn name(&self) -> &'static str {
        match self {
            FileFilter::All => "All Supported Files",
            FileFilter::Worlds => "Worlds & Scenes",
            FileFilter::Robots => "Robot Descriptions",
            FileFilter::Models => "3D Models",
            FileFilter::Automotive => "Road & Scenarios",
        }
    }

    /// Get file extensions for this filter
    pub fn extensions(&self) -> Vec<&'static str> {
        match self {
            FileFilter::All => WorldLoader::supported_extensions(),
            FileFilter::Worlds => vec![
                "scene", "scn", "sdf", "world", "usd", "usda", "usdc", "usdz", "json", "yaml",
                "yml",
            ],
            FileFilter::Robots => vec!["urdf", "xacro", "srdf", "xml"],
            FileFilter::Models => vec!["gltf", "glb", "fbx", "obj", "stl", "dae"],
            FileFilter::Automotive => vec!["xodr", "xosc"],
        }
    }

    /// Get all filter variants
    pub fn all() -> &'static [FileFilter] {
        &[
            FileFilter::All,
            FileFilter::Worlds,
            FileFilter::Robots,
            FileFilter::Models,
            FileFilter::Automotive,
        ]
    }
}

/// Event to request opening a file dialog
#[derive(Event, Debug, Clone)]
pub struct OpenFileDialogEvent {
    /// Filter to use for the dialog
    pub filter: FileFilter,
    /// Title for the dialog
    pub title: Option<String>,
}

impl Default for OpenFileDialogEvent {
    fn default() -> Self {
        Self {
            filter: FileFilter::All,
            title: None,
        }
    }
}

impl OpenFileDialogEvent {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_filter(filter: FileFilter) -> Self {
        Self {
            filter,
            title: None,
        }
    }

    pub fn with_title(mut self, title: impl Into<String>) -> Self {
        self.title = Some(title.into());
        self
    }
}

/// Event emitted when a file is selected from the dialog
#[derive(Event, Debug, Clone)]
pub struct FileSelectedEvent {
    /// Path to the selected file
    pub path: PathBuf,
    /// Detected file type
    pub file_type: RecentFileType,
}

/// Event to request saving a file
#[derive(Event, Debug, Clone)]
pub struct SaveFileDialogEvent {
    /// Default filename
    pub default_name: Option<String>,
    /// Default extension
    pub default_extension: Option<String>,
    /// Title for the dialog
    pub title: Option<String>,
}

/// Event emitted when a save location is selected
#[derive(Event, Debug, Clone)]
pub struct SaveLocationSelectedEvent {
    /// Path where to save
    pub path: PathBuf,
}

/// System to handle file dialog opening
#[cfg(feature = "rfd")]
pub fn handle_open_file_dialog(
    mut events: EventReader<OpenFileDialogEvent>,
    mut dialog_state: ResMut<FileDialogState>,
    mut file_selected: EventWriter<FileSelectedEvent>,
) {
    use rfd::FileDialog;

    for event in events.read() {
        if dialog_state.is_open {
            warn!("File dialog already open");
            continue;
        }

        dialog_state.is_open = true;
        dialog_state.error = None;

        let title = event
            .title
            .clone()
            .unwrap_or_else(|| format!("Open {}", event.filter.name()));

        // Build file dialog with filters
        let mut dialog = FileDialog::new().set_title(&title);

        // Add file type filters
        let extensions = event.filter.extensions();
        if !extensions.is_empty() {
            dialog = dialog.add_filter(event.filter.name(), &extensions);
        }

        // Set starting directory to common locations
        if let Some(home) = dirs::home_dir() {
            dialog = dialog.set_directory(home);
        }

        // Show the dialog (blocking on main thread - spawns native dialog)
        match dialog.pick_file() {
            Some(path) => {
                info!("File selected via dialog: {:?}", path);

                // Detect file type
                let file_type = detect_file_type(&path);

                dialog_state.selected_path = Some(path.clone());
                dialog_state.is_open = false;

                // Send file selected event
                file_selected.send(FileSelectedEvent { path, file_type });
            }
            None => {
                info!("File dialog cancelled");
                dialog_state.is_open = false;
            }
        }
    }
}

/// System to handle file dialog opening (stub when rfd is not available)
#[cfg(not(feature = "rfd"))]
pub fn handle_open_file_dialog(
    mut events: EventReader<OpenFileDialogEvent>,
    mut _dialog_state: ResMut<FileDialogState>,
) {
    for _event in events.read() {
        warn!("Native file dialog not available - build with 'visual' feature for rfd support");
        warn!("Use manual file path input, drag-and-drop, or command line arguments instead");
    }
}

/// Detect the file type based on extension
fn detect_file_type(path: &std::path::Path) -> RecentFileType {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase();

    match ext.as_str() {
        // World/Scene formats
        "sdf" | "world" => RecentFileType::Sdf,
        "scene" | "scn" => RecentFileType::Scene,
        "usd" | "usda" | "usdc" | "usdz" => RecentFileType::Usd,

        // Robot descriptions
        "urdf" => RecentFileType::Urdf,
        "xacro" => RecentFileType::Xacro,
        "srdf" => RecentFileType::Srdf,

        // 3D Models
        "gltf" | "glb" => RecentFileType::Gltf,
        "fbx" => RecentFileType::Fbx,
        "obj" => RecentFileType::Obj,
        "stl" => RecentFileType::Stl,
        "dae" => RecentFileType::Dae,

        // Automotive formats
        "xodr" => RecentFileType::OpenDrive,
        "xosc" => RecentFileType::OpenScenario,

        // Config/data
        "yaml" | "yml" | "json" => RecentFileType::Config,

        _ => RecentFileType::Other,
    }
}

/// System to handle save file dialog
#[cfg(feature = "rfd")]
pub fn handle_save_file_dialog(
    mut events: EventReader<SaveFileDialogEvent>,
    mut save_selected: EventWriter<SaveLocationSelectedEvent>,
) {
    use rfd::FileDialog;

    for event in events.read() {
        let title = event
            .title
            .clone()
            .unwrap_or_else(|| "Save As".to_string());

        let mut dialog = FileDialog::new().set_title(&title);

        // Set default filename if provided
        if let Some(ref name) = event.default_name {
            dialog = dialog.set_file_name(name);
        }

        // Add extension filter if provided
        if let Some(ref ext) = event.default_extension {
            dialog = dialog.add_filter("Default", &[ext.as_str()]);
        }

        // Set starting directory
        if let Some(home) = dirs::home_dir() {
            dialog = dialog.set_directory(home);
        }

        match dialog.save_file() {
            Some(path) => {
                info!("Save location selected: {:?}", path);
                save_selected.send(SaveLocationSelectedEvent { path });
            }
            None => {
                info!("Save dialog cancelled");
            }
        }
    }
}

/// System to handle save file dialog (stub when rfd is not available)
#[cfg(not(feature = "rfd"))]
pub fn handle_save_file_dialog(
    mut events: EventReader<SaveFileDialogEvent>,
) {
    for _event in events.read() {
        warn!("Native save dialog not available - build with 'visual' feature for rfd support");
    }
}

/// System to handle file selection and trigger world loading
pub fn handle_file_selected(
    mut events: EventReader<FileSelectedEvent>,
    mut load_world: EventWriter<LoadWorldEvent>,
    mut add_recent: EventWriter<AddRecentFileEvent>,
) {
    for event in events.read() {
        info!(
            "File selected: {:?} (type: {:?})",
            event.path, event.file_type
        );

        // Check if file type is loadable as a world
        if WorldLoader::is_supported(&event.path) {
            // Request world loading
            load_world.send(LoadWorldEvent::new(event.path.clone()));

            // Add to recent files
            let recent_file = RecentFile::new(&event.path);
            add_recent.send(AddRecentFileEvent { file: recent_file });
        } else {
            warn!(
                "Selected file type {:?} is not supported for world loading",
                event.file_type
            );
        }
    }
}

/// Plugin for file dialog functionality
pub struct FileDialogPlugin;

impl Plugin for FileDialogPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FileDialogState>()
            .add_event::<OpenFileDialogEvent>()
            .add_event::<FileSelectedEvent>()
            .add_event::<SaveFileDialogEvent>()
            .add_event::<SaveLocationSelectedEvent>()
            .add_systems(
                Update,
                (
                    handle_file_selected,
                    handle_open_file_dialog,
                    handle_save_file_dialog,
                ),
            );
    }
}

/// Helper function to build file filter string for native dialogs
pub fn build_filter_string(filter: FileFilter) -> String {
    let extensions = filter.extensions();
    let ext_str = extensions
        .iter()
        .map(|e| format!("*.{}", e))
        .collect::<Vec<_>>()
        .join(";");
    format!("{} ({})", filter.name(), ext_str)
}

/// Check if a file extension is supported
pub fn is_extension_supported(ext: &str) -> bool {
    let ext_lower = ext.to_lowercase();
    WorldLoader::supported_extensions()
        .iter()
        .any(|e| *e == ext_lower)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_extensions() {
        // All filter should have all extensions
        let all_ext = FileFilter::All.extensions();
        assert!(all_ext.len() > 10);

        // Worlds filter
        let world_ext = FileFilter::Worlds.extensions();
        assert!(world_ext.contains(&"sdf"));
        assert!(world_ext.contains(&"usd"));

        // Robots filter
        let robot_ext = FileFilter::Robots.extensions();
        assert!(robot_ext.contains(&"urdf"));
        assert!(robot_ext.contains(&"xacro"));

        // Models filter
        let model_ext = FileFilter::Models.extensions();
        assert!(model_ext.contains(&"gltf"));
        assert!(model_ext.contains(&"fbx"));

        // Automotive filter
        let auto_ext = FileFilter::Automotive.extensions();
        assert!(auto_ext.contains(&"xodr"));
        assert!(auto_ext.contains(&"xosc"));
    }

    #[test]
    fn test_is_extension_supported() {
        assert!(is_extension_supported("urdf"));
        assert!(is_extension_supported("URDF")); // Case insensitive
        assert!(is_extension_supported("sdf"));
        assert!(is_extension_supported("usd"));
        assert!(!is_extension_supported("pdf"));
        assert!(!is_extension_supported("exe"));
    }

    #[test]
    fn test_filter_names() {
        assert_eq!(FileFilter::All.name(), "All Supported Files");
        assert_eq!(FileFilter::Worlds.name(), "Worlds & Scenes");
        assert_eq!(FileFilter::Robots.name(), "Robot Descriptions");
    }
}
