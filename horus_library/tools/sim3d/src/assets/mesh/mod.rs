//! Mesh loading module
//!
//! Supports multiple mesh formats:
//! - OBJ (Wavefront)
//! - STL (ASCII and binary)
//! - COLLADA (.dae)
//! - glTF/GLB
//! - FBX (Filmbox - ASCII and binary)

mod collada_loader;
mod fbx_loader;
mod gltf_loader;
mod obj_loader;
mod optimization;
mod processing;
mod stl_loader;
mod validation;

pub use collada_loader::load_collada;
pub use fbx_loader::load_fbx;
pub use gltf_loader::load_gltf;
pub use obj_loader::load_obj;
pub use stl_loader::load_stl;
pub use validation::validate_mesh;

use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy::render::primitives::Aabb;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Mesh loading options
#[derive(Debug, Clone)]
pub struct MeshLoadOptions {
    /// Scale to apply to mesh vertices
    pub scale: Vec3,
    /// Generate normals if missing
    pub generate_normals: bool,
    /// Generate tangents for normal mapping
    pub generate_tangents: bool,
    /// Flip UVs vertically
    pub flip_uvs: bool,
    /// Validate mesh geometry
    pub validate: bool,
    /// Maximum triangle count (triggers decimation if exceeded)
    pub max_triangles: Option<usize>,
}

impl Default for MeshLoadOptions {
    fn default() -> Self {
        Self {
            scale: Vec3::ONE,
            generate_normals: true,
            generate_tangents: false,
            flip_uvs: false,
            validate: true,
            max_triangles: None,
        }
    }
}

impl MeshLoadOptions {
    /// Create with custom scale
    pub fn with_scale(mut self, scale: Vec3) -> Self {
        self.scale = scale;
        self
    }

    /// Enable normal generation
    pub fn generate_normals(mut self, generate: bool) -> Self {
        self.generate_normals = generate;
        self
    }

    /// Enable tangent generation
    pub fn generate_tangents(mut self, generate: bool) -> Self {
        self.generate_tangents = generate;
        self
    }

    /// Enable UV flipping
    pub fn flip_uvs(mut self, flip: bool) -> Self {
        self.flip_uvs = flip;
        self
    }

    /// Enable validation
    pub fn validate(mut self, validate: bool) -> Self {
        self.validate = validate;
        self
    }

    /// Set maximum triangle count
    pub fn max_triangles(mut self, max: usize) -> Self {
        self.max_triangles = Some(max);
        self
    }
}

/// Embedded texture data extracted from GLB/glTF files
#[derive(Clone, Debug)]
pub struct EmbeddedTexture {
    /// Unique identifier for this texture (e.g., "embedded_0")
    pub id: String,
    /// MIME type of the image (e.g., "image/png", "image/jpeg")
    pub mime_type: String,
    /// Raw image data bytes
    pub data: Vec<u8>,
}

/// Loaded mesh data with metadata
#[derive(Clone)]
pub struct LoadedMesh {
    /// The Bevy mesh
    pub mesh: Mesh,
    /// Material information (if available from file)
    pub materials: Vec<MaterialInfo>,
    /// Texture paths referenced by materials
    pub texture_paths: Vec<PathBuf>,
    /// Embedded textures extracted from the mesh file (GLB/glTF)
    pub embedded_textures: Vec<EmbeddedTexture>,
    /// Axis-aligned bounding box
    pub bounds: Aabb,
    /// Triangle count
    pub triangle_count: usize,
    /// Vertex count
    pub vertex_count: usize,
}

/// Material information from mesh file
#[derive(Clone, Debug)]
pub struct MaterialInfo {
    pub name: String,
    pub diffuse_color: Option<Color>,
    pub diffuse_texture: Option<PathBuf>,
    pub normal_texture: Option<PathBuf>,
    pub metallic: f32,
    pub roughness: f32,
}

impl Default for MaterialInfo {
    fn default() -> Self {
        Self {
            name: "default".to_string(),
            diffuse_color: Some(Color::srgb(0.8, 0.8, 0.8)),
            diffuse_texture: None,
            normal_texture: None,
            metallic: 0.0,
            roughness: 0.5,
        }
    }
}

/// Universal mesh loader
pub struct MeshLoader {
    base_paths: Vec<PathBuf>,
    cache: HashMap<PathBuf, LoadedMesh>,
}

impl MeshLoader {
    /// Create a new mesh loader
    pub fn new() -> Self {
        Self {
            base_paths: vec![PathBuf::from("assets/models")],
            cache: HashMap::new(),
        }
    }

    /// Create with custom base paths
    pub fn with_base_paths(base_paths: Vec<PathBuf>) -> Self {
        Self {
            base_paths,
            cache: HashMap::new(),
        }
    }

    /// Add a base path to search
    pub fn add_base_path(&mut self, path: PathBuf) {
        if !self.base_paths.contains(&path) {
            self.base_paths.push(path);
        }
    }

    /// Load mesh with automatic format detection
    pub fn load(&mut self, path: &Path, options: MeshLoadOptions) -> Result<LoadedMesh> {
        // Check cache first
        if let Some(cached) = self.cache.get(path) {
            tracing::info!("Mesh loader cache hit: {}", path.display());
            return Ok(cached.clone());
        }

        // Resolve path
        let resolved_path = self
            .resolve_path(path)
            .context(format!("Failed to resolve mesh path: {}", path.display()))?;

        // Detect format and load
        let extension = resolved_path
            .extension()
            .and_then(|s| s.to_str())
            .context(format!(
                "Invalid file extension for: {}",
                resolved_path.display()
            ))?
            .to_lowercase();

        tracing::info!(
            "Loading mesh: {} (format: {})",
            resolved_path.display(),
            extension
        );

        let mut loaded = match extension.as_str() {
            "obj" => load_obj(&resolved_path, &options)?,
            "stl" => load_stl(&resolved_path, &options)?,
            "dae" => load_collada(&resolved_path, &options)?,
            "gltf" | "glb" => load_gltf(&resolved_path, &options)?,
            "fbx" => load_fbx(&resolved_path, &options)?,
            ext => anyhow::bail!("Unsupported mesh format: {}", ext),
        };

        // Validate if requested
        if options.validate {
            validate_mesh(&loaded.mesh).context(format!(
                "Mesh validation failed for: {}",
                resolved_path.display()
            ))?;
        }

        // Apply decimation if max_triangles is exceeded
        if let Some(max_tris) = options.max_triangles {
            if loaded.triangle_count > max_tris {
                tracing::info!(
                    "Mesh exceeds max_triangles ({} > {}), applying decimation",
                    loaded.triangle_count,
                    max_tris
                );

                let decim_options =
                    optimization::DecimationOptions::default().with_target_triangles(max_tris);

                optimization::decimate_mesh(&mut loaded.mesh, decim_options)
                    .context("Failed to decimate mesh")?;

                // Update triangle count
                loaded.triangle_count = processing::count_triangles(loaded.mesh.indices());
                tracing::info!("Mesh decimated to {} triangles", loaded.triangle_count);
            }
        }

        // Cache and return
        self.cache.insert(path.to_path_buf(), loaded.clone());
        tracing::info!(
            "Loaded mesh: {} vertices, {} triangles",
            loaded.vertex_count,
            loaded.triangle_count
        );

        Ok(loaded)
    }

    /// Clear the cache
    pub fn clear_cache(&mut self) {
        self.cache.clear();
    }

    /// Get cache size
    pub fn cache_size(&self) -> usize {
        self.cache.len()
    }

    /// Resolve mesh path (search in base paths)
    fn resolve_path(&self, path: &Path) -> Result<PathBuf> {
        // Try absolute path first
        if path.is_absolute() && path.exists() {
            return Ok(path.to_path_buf());
        }

        // Try relative to base paths
        for base in &self.base_paths {
            let candidate = base.join(path);
            if candidate.exists() {
                return Ok(candidate);
            }
        }

        // Try current directory
        if path.exists() {
            return Ok(path.to_path_buf());
        }

        anyhow::bail!(
            "Mesh file not found: {}\nSearched in {} locations:\n{}",
            path.display(),
            self.base_paths.len() + 1,
            self.base_paths
                .iter()
                .map(|p| format!("  - {}", p.display()))
                .collect::<Vec<_>>()
                .join("\n")
        )
    }
}

impl Default for MeshLoader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_options_default() {
        let options = MeshLoadOptions::default();
        assert_eq!(options.scale, Vec3::ONE);
        assert!(options.generate_normals);
        assert!(!options.generate_tangents);
        assert!(!options.flip_uvs);
        assert!(options.validate);
        assert!(options.max_triangles.is_none());
    }

    #[test]
    fn test_load_options_builder() {
        let options = MeshLoadOptions::default()
            .with_scale(Vec3::new(2.0, 2.0, 2.0))
            .generate_tangents(true)
            .flip_uvs(true)
            .max_triangles(10000);

        assert_eq!(options.scale, Vec3::new(2.0, 2.0, 2.0));
        assert!(options.generate_tangents);
        assert!(options.flip_uvs);
        assert_eq!(options.max_triangles, Some(10000));
    }

    #[test]
    fn test_mesh_loader_creation() {
        let loader = MeshLoader::new();
        assert_eq!(loader.cache_size(), 0);
        assert_eq!(loader.base_paths.len(), 1);
    }
}
