//! Plugin loader for HORUS drivers
//!
//! This module provides the [`PluginLoader`] for discovering and loading driver plugins.
//! It supports both static (compiled-in) and dynamic (runtime) plugin loading.
//!
//! # Static Plugins
//!
//! Static plugins are registered at compile time and have zero runtime overhead:
//!
//! ```rust,ignore
//! let mut loader = PluginLoader::new();
//! loader.register_static(Box::new(Mpu6050Plugin));
//! ```
//!
//! # Dynamic Plugins
//!
//! Dynamic plugins are loaded at runtime from shared libraries (.so/.dylib).
//! Requires the `dynamic-plugins` feature:
//!
//! ```rust,ignore
//! let mut loader = PluginLoader::new();
//! loader.add_search_path("~/.horus/drivers");
//! loader.discover()?;  // Scans for plugins
//! loader.load("horus-imu-mpu6050")?;  // Loads specific plugin
//! ```

use super::traits::DriverPlugin;

#[cfg(feature = "dynamic-plugins")]
use super::traits::PLUGIN_ENTRY_SYMBOL;
use super::types::{PluginError, PluginId, PluginManifest, PluginResult};

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

#[cfg(feature = "dynamic-plugins")]
use libloading::Library;

/// Information about a discovered plugin (before loading)
#[derive(Debug, Clone)]
pub struct DiscoveredPlugin {
    /// Plugin ID (from filename or manifest)
    pub id: PluginId,

    /// Path to the plugin file
    pub path: PathBuf,

    /// Whether this is a static (compiled-in) plugin
    pub is_static: bool,

    /// Manifest if already loaded/parsed
    pub manifest: Option<PluginManifest>,
}

/// A loaded plugin with its library handle (for dynamic plugins)
struct LoadedPlugin {
    /// The plugin instance
    plugin: Box<dyn DriverPlugin>,

    /// Library handle (only for dynamic plugins, keeps library alive)
    #[cfg(feature = "dynamic-plugins")]
    _library: Option<Library>,
}

/// Plugin loader for discovering and managing driver plugins
///
/// The loader maintains a registry of available plugins and handles
/// loading them on demand. It supports both static registration and
/// dynamic discovery.
///
/// # Thread Safety
///
/// The loader uses `RwLock` for interior mutability and is safe to use
/// from multiple threads.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::plugin::PluginLoader;
///
/// let mut loader = PluginLoader::new();
///
/// // Add search paths for dynamic plugins
/// loader.add_search_path("~/.horus/drivers");
/// loader.add_search_path("/usr/local/lib/horus/drivers");
///
/// // Discover available plugins
/// let discovered = loader.discover()?;
/// println!("Found {} plugins", discovered.len());
///
/// // Load a specific plugin
/// let plugin = loader.load("horus-imu-mpu6050")?;
/// let manifest = plugin.manifest();
/// ```
pub struct PluginLoader {
    /// Search paths for dynamic plugin discovery
    search_paths: Vec<PathBuf>,

    /// Discovered plugins (not yet loaded)
    discovered: HashMap<PluginId, DiscoveredPlugin>,

    /// Loaded plugins
    loaded: RwLock<HashMap<PluginId, Arc<LoadedPlugin>>>,

    /// Static plugins registered at compile time
    static_plugins: HashMap<PluginId, Box<dyn DriverPlugin>>,

    /// Current HORUS version for compatibility checking
    horus_version: String,
}

impl PluginLoader {
    /// Create a new plugin loader
    pub fn new() -> Self {
        Self::with_version(env!("CARGO_PKG_VERSION"))
    }

    /// Create a new plugin loader with a specific HORUS version
    pub fn with_version(horus_version: &str) -> Self {
        Self {
            search_paths: Vec::new(),
            discovered: HashMap::new(),
            loaded: RwLock::new(HashMap::new()),
            static_plugins: HashMap::new(),
            horus_version: horus_version.to_string(),
        }
    }

    /// Add a search path for dynamic plugin discovery
    ///
    /// Paths are searched in order. Supports `~` for home directory.
    pub fn add_search_path(&mut self, path: impl AsRef<Path>) {
        let path = path.as_ref();

        // Expand ~ to home directory
        let expanded = if let Some(path_str) = path.to_str() {
            if let Some(rest) = path_str.strip_prefix("~/") {
                if let Some(home) = dirs::home_dir() {
                    home.join(rest)
                } else {
                    path.to_path_buf()
                }
            } else {
                path.to_path_buf()
            }
        } else {
            path.to_path_buf()
        };

        if !self.search_paths.contains(&expanded) {
            self.search_paths.push(expanded);
        }
    }

    /// Add default search paths
    ///
    /// Adds the following paths in order:
    /// 1. `~/.horus/drivers/` (user plugins)
    /// 2. `/usr/local/lib/horus/drivers/` (system plugins)
    /// 3. `/usr/lib/horus/drivers/` (distribution plugins)
    pub fn add_default_search_paths(&mut self) {
        self.add_search_path("~/.horus/drivers");

        #[cfg(unix)]
        {
            self.add_search_path("/usr/local/lib/horus/drivers");
            self.add_search_path("/usr/lib/horus/drivers");
        }

        #[cfg(windows)]
        {
            if let Some(program_data) = std::env::var_os("ProgramData") {
                let path = PathBuf::from(program_data).join("horus").join("drivers");
                self.add_search_path(path);
            }
        }
    }

    /// Register a static (compiled-in) plugin
    ///
    /// Static plugins are always available and have zero runtime loading overhead.
    pub fn register_static(&mut self, plugin: Box<dyn DriverPlugin>) {
        let manifest = plugin.manifest();
        let id = manifest.id.clone();

        // Add to discovered list
        self.discovered.insert(
            id.clone(),
            DiscoveredPlugin {
                id: id.clone(),
                path: PathBuf::new(),
                is_static: true,
                manifest: Some(manifest),
            },
        );

        // Store the plugin
        self.static_plugins.insert(id, plugin);
    }

    /// Discover available plugins in search paths
    ///
    /// Scans all search paths for plugin files (.so on Linux, .dylib on macOS,
    /// .dll on Windows). Returns a list of discovered plugins.
    ///
    /// Note: This only discovers plugins; it doesn't load them. Use [`Self::load`]
    /// to actually load a plugin.
    pub fn discover(&mut self) -> PluginResult<Vec<DiscoveredPlugin>> {
        let mut discovered = Vec::new();

        // Include static plugins
        for info in self.discovered.values() {
            if info.is_static {
                discovered.push(info.clone());
            }
        }

        // Scan search paths for dynamic plugins
        #[cfg(feature = "dynamic-plugins")]
        {
            let extension = if cfg!(target_os = "linux") {
                "so"
            } else if cfg!(target_os = "macos") {
                "dylib"
            } else if cfg!(target_os = "windows") {
                "dll"
            } else {
                "so"
            };

            for search_path in &self.search_paths {
                if !search_path.exists() {
                    continue;
                }

                let entries = match std::fs::read_dir(search_path) {
                    Ok(entries) => entries,
                    Err(_) => continue,
                };

                for entry in entries.filter_map(|e| e.ok()) {
                    let path = entry.path();

                    // Check if it's a plugin file
                    if path.extension().and_then(|s| s.to_str()) != Some(extension) {
                        continue;
                    }

                    // Extract plugin ID from filename
                    // Format: libhorus_<id>.so or horus-<id>.so
                    let filename = match path.file_stem().and_then(|s| s.to_str()) {
                        Some(name) => name,
                        None => continue,
                    };

                    let id = if let Some(stripped) = filename.strip_prefix("libhorus_") {
                        stripped.replace('_', "-")
                    } else if let Some(stripped) = filename.strip_prefix("horus-") {
                        stripped.to_string()
                    } else if let Some(stripped) = filename.strip_prefix("lib") {
                        stripped.replace('_', "-")
                    } else {
                        filename.replace('_', "-")
                    };

                    // Skip if already discovered
                    if self.discovered.contains_key(&id) {
                        continue;
                    }

                    let info = DiscoveredPlugin {
                        id: id.clone(),
                        path: path.clone(),
                        is_static: false,
                        manifest: None, // Will be populated on load
                    };

                    self.discovered.insert(id, info.clone());
                    discovered.push(info);
                }
            }
        }

        Ok(discovered)
    }

    /// Get list of discovered plugin IDs
    pub fn discovered_ids(&self) -> Vec<&str> {
        self.discovered.keys().map(|s| s.as_str()).collect()
    }

    /// Check if a plugin is discovered
    pub fn is_discovered(&self, id: &str) -> bool {
        self.discovered.contains_key(id)
    }

    /// Check if a plugin is loaded
    pub fn is_loaded(&self, id: &str) -> bool {
        self.loaded.read().unwrap().contains_key(id)
    }

    /// Load a plugin by ID
    ///
    /// If the plugin is already loaded, returns the cached instance.
    /// For static plugins, this is essentially free.
    /// For dynamic plugins, this loads the shared library.
    pub fn load(&self, id: &str) -> PluginResult<Arc<dyn DriverPlugin>> {
        // Check if already loaded
        {
            let loaded = self.loaded.read().unwrap();
            if let Some(plugin) = loaded.get(id) {
                return Ok(Arc::new(PluginWrapper(Arc::clone(plugin))));
            }
        }

        // Find the plugin info
        let info = self
            .discovered
            .get(id)
            .ok_or_else(|| PluginError::NotFound(id.to_string()))?;

        // Load based on type
        let loaded_plugin = if info.is_static {
            self.load_static(id)?
        } else {
            #[cfg(feature = "dynamic-plugins")]
            {
                self.load_dynamic(&info.path)?
            }
            #[cfg(not(feature = "dynamic-plugins"))]
            {
                return Err(PluginError::LoadError(
                    "Dynamic plugins require the 'dynamic-plugins' feature".into(),
                ));
            }
        };

        // Validate compatibility
        let manifest = loaded_plugin.plugin.manifest();
        if !manifest.is_compatible_with(&self.horus_version) {
            return Err(PluginError::VersionMismatch {
                required: manifest.horus_version.clone(),
                found: self.horus_version.clone(),
            });
        }

        if !manifest.supports_current_platform() {
            return Err(PluginError::PlatformNotSupported(
                std::env::consts::OS.to_string(),
            ));
        }

        // Call on_load hook
        loaded_plugin.plugin.on_load()?;

        // Cache and return
        let arc = Arc::new(loaded_plugin);
        {
            let mut loaded = self.loaded.write().unwrap();
            loaded.insert(id.to_string(), Arc::clone(&arc));
        }

        Ok(Arc::new(PluginWrapper(arc)))
    }

    /// Load a static plugin
    fn load_static(&self, id: &str) -> PluginResult<LoadedPlugin> {
        // For static plugins, we need to clone the Box<dyn DriverPlugin>
        // This is a limitation - static plugins need to implement Clone
        // For now, we return an error suggesting re-registration
        Err(PluginError::LoadError(format!(
            "Static plugin '{}' should be accessed via register_static(). \
             Consider using get_static() instead.",
            id
        )))
    }

    /// Load a dynamic plugin from a shared library
    #[cfg(feature = "dynamic-plugins")]
    fn load_dynamic(&self, path: &Path) -> PluginResult<LoadedPlugin> {
        // SAFETY: We trust that plugin libraries export the correct symbol
        // with the correct signature. This is inherently unsafe as we're
        // loading arbitrary code from a user-specified .so/.dll/.dylib path.
        unsafe {
            let library = Library::new(path).map_err(|e| {
                PluginError::LoadError(format!("Failed to load library {:?}: {}", path, e))
            })?;

            // Look up the entry point
            let entry_fn: libloading::Symbol<super::traits::PluginEntryFn> =
                library.get(PLUGIN_ENTRY_SYMBOL.as_bytes()).map_err(|e| {
                    PluginError::InvalidPlugin(format!(
                        "Missing entry point '{}': {}",
                        PLUGIN_ENTRY_SYMBOL, e
                    ))
                })?;

            // Call entry point to get plugin instance
            let plugin = entry_fn();

            Ok(LoadedPlugin {
                plugin,
                _library: Some(library),
            })
        }
    }

    /// Unload a plugin
    ///
    /// Removes the plugin from the loaded cache. For dynamic plugins,
    /// this will unload the shared library if no other references exist.
    pub fn unload(&self, id: &str) -> PluginResult<()> {
        let mut loaded = self.loaded.write().unwrap();

        if let Some(plugin) = loaded.remove(id) {
            // Call on_unload if this is the last reference
            if Arc::strong_count(&plugin) == 1 {
                plugin.plugin.on_unload();
            }
        }

        Ok(())
    }

    /// Get all loaded plugins
    pub fn loaded_plugins(&self) -> Vec<Arc<dyn DriverPlugin>> {
        let loaded = self.loaded.read().unwrap();
        loaded
            .values()
            .map(|p| Arc::new(PluginWrapper(Arc::clone(p))) as Arc<dyn DriverPlugin>)
            .collect()
    }

    /// Get a loaded plugin by ID
    pub fn get(&self, id: &str) -> Option<Arc<dyn DriverPlugin>> {
        let loaded = self.loaded.read().unwrap();
        loaded
            .get(id)
            .map(|p| Arc::new(PluginWrapper(Arc::clone(p))) as Arc<dyn DriverPlugin>)
    }

    /// Get plugin info by ID
    pub fn get_info(&self, id: &str) -> Option<&DiscoveredPlugin> {
        self.discovered.get(id)
    }

    /// Get the search paths
    pub fn search_paths(&self) -> &[PathBuf] {
        &self.search_paths
    }
}

impl Default for PluginLoader {
    fn default() -> Self {
        Self::new()
    }
}

// Wrapper to implement DriverPlugin for Arc<LoadedPlugin>
struct PluginWrapper(Arc<LoadedPlugin>);

impl DriverPlugin for PluginWrapper {
    fn manifest(&self) -> PluginManifest {
        self.0.plugin.manifest()
    }

    fn probe(&self, backend_id: &str) -> Vec<super::types::ProbeResult> {
        self.0.plugin.probe(backend_id)
    }

    fn create(
        &self,
        backend_id: &str,
        config: &crate::driver::SingleDriverConfig,
    ) -> crate::HorusResult<Box<dyn std::any::Any + Send + Sync>> {
        self.0.plugin.create(backend_id, config)
    }

    fn health(&self) -> super::types::PluginHealth {
        self.0.plugin.health()
    }

    fn on_load(&self) -> crate::HorusResult<()> {
        // Already loaded
        Ok(())
    }

    fn on_unload(&self) {
        // Handled by drop
    }
}

// Make PluginWrapper Send + Sync
unsafe impl Send for PluginWrapper {}
unsafe impl Sync for PluginWrapper {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_loader_new() {
        let loader = PluginLoader::new();
        assert!(loader.search_paths.is_empty());
        assert!(loader.discovered.is_empty());
    }

    #[test]
    fn test_add_search_path() {
        let mut loader = PluginLoader::new();
        loader.add_search_path("/test/path");
        assert_eq!(loader.search_paths.len(), 1);
        assert_eq!(loader.search_paths[0], PathBuf::from("/test/path"));

        // Adding same path again should not duplicate
        loader.add_search_path("/test/path");
        assert_eq!(loader.search_paths.len(), 1);
    }

    #[test]
    fn test_discover_empty() {
        let mut loader = PluginLoader::new();
        let discovered = loader.discover().unwrap();
        assert!(discovered.is_empty());
    }

    #[test]
    fn test_is_discovered() {
        let loader = PluginLoader::new();
        assert!(!loader.is_discovered("nonexistent"));
    }

    #[test]
    fn test_is_loaded() {
        let loader = PluginLoader::new();
        assert!(!loader.is_loaded("nonexistent"));
    }
}
