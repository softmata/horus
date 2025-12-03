//! Heightmap/Terrain Loader for Outdoor Robot Simulation
//!
//! Supports loading terrain from:
//! - PNG/TIFF grayscale images (8-bit, 16-bit)
//! - RAW heightmap files (common in game engines)
//! - GeoTIFF for real-world terrain data
//! - Terrain configuration files (YAML/JSON)
//!
//! Features:
//! - Multi-resolution terrain with LOD support
//! - Physics collision mesh generation
//! - Texture splatting based on height/slope
//! - Erosion and detail map support

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

use crate::error::{EnhancedError, ErrorCategory, Result};

// ============================================================================
// Terrain Configuration
// ============================================================================

/// Main terrain configuration structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainConfig {
    /// Name of the terrain
    pub name: String,
    /// Path to heightmap image
    pub heightmap: String,
    /// Terrain dimensions in world units
    pub size: TerrainSize,
    /// Height scaling factor
    #[serde(default = "default_height_scale")]
    pub height_scale: f32,
    /// Height offset (base height)
    #[serde(default)]
    pub height_offset: f32,
    /// Physics configuration
    #[serde(default)]
    pub physics: TerrainPhysics,
    /// Texture layers for splatting
    #[serde(default)]
    pub texture_layers: Vec<TextureLayer>,
    /// Level of detail configuration
    #[serde(default)]
    pub lod: Option<TerrainLOD>,
    /// Optional detail maps
    #[serde(default)]
    pub detail_maps: Vec<DetailMap>,
}

fn default_height_scale() -> f32 {
    100.0
}

/// Terrain dimensions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainSize {
    /// Width in world units (X axis)
    pub width: f32,
    /// Length in world units (Z axis)
    pub length: f32,
    /// Maximum height value
    #[serde(default = "default_max_height")]
    pub max_height: f32,
}

fn default_max_height() -> f32 {
    100.0
}

impl Default for TerrainSize {
    fn default() -> Self {
        Self {
            width: 1000.0,
            length: 1000.0,
            max_height: 100.0,
        }
    }
}

/// Physics properties for terrain
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainPhysics {
    /// Static friction coefficient
    #[serde(default = "default_terrain_friction")]
    pub friction: f32,
    /// Restitution (bounciness)
    #[serde(default)]
    pub restitution: f32,
    /// Collision margin for physics
    #[serde(default = "default_collision_margin")]
    pub collision_margin: f32,
    /// Resolution of physics collision mesh (lower = more detailed)
    #[serde(default = "default_physics_resolution")]
    pub physics_resolution: u32,
}

fn default_terrain_friction() -> f32 {
    0.7
}

fn default_collision_margin() -> f32 {
    0.01
}

fn default_physics_resolution() -> u32 {
    4
}

impl Default for TerrainPhysics {
    fn default() -> Self {
        Self {
            friction: 0.7,
            restitution: 0.0,
            collision_margin: 0.01,
            physics_resolution: 4,
        }
    }
}

/// Texture layer for terrain splatting
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextureLayer {
    /// Layer name
    pub name: String,
    /// Diffuse/albedo texture path
    pub diffuse: String,
    /// Optional normal map path
    #[serde(default)]
    pub normal: Option<String>,
    /// Minimum height for this layer (0.0-1.0 normalized)
    #[serde(default)]
    pub min_height: f32,
    /// Maximum height for this layer (0.0-1.0 normalized)
    #[serde(default = "default_max_layer_height")]
    pub max_height: f32,
    /// Minimum slope for this layer (degrees)
    #[serde(default)]
    pub min_slope: f32,
    /// Maximum slope for this layer (degrees)
    #[serde(default = "default_max_slope")]
    pub max_slope: f32,
    /// Texture tiling scale
    #[serde(default = "default_tile_scale")]
    pub tile_scale: f32,
}

fn default_max_layer_height() -> f32 {
    1.0
}

fn default_max_slope() -> f32 {
    90.0
}

fn default_tile_scale() -> f32 {
    10.0
}

/// Level of detail configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainLOD {
    /// Number of LOD levels
    #[serde(default = "default_lod_levels")]
    pub levels: u32,
    /// Distance multiplier between LOD levels
    #[serde(default = "default_lod_distance")]
    pub distance_multiplier: f32,
    /// Base distance for LOD switching
    #[serde(default = "default_base_distance")]
    pub base_distance: f32,
}

fn default_lod_levels() -> u32 {
    4
}

fn default_lod_distance() -> f32 {
    2.0
}

fn default_base_distance() -> f32 {
    50.0
}

impl Default for TerrainLOD {
    fn default() -> Self {
        Self {
            levels: 4,
            distance_multiplier: 2.0,
            base_distance: 50.0,
        }
    }
}

/// Detail map for additional terrain features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetailMap {
    /// Detail map type
    pub map_type: DetailMapType,
    /// Path to detail map image
    pub path: String,
    /// Intensity/influence of the detail map
    #[serde(default = "default_detail_intensity")]
    pub intensity: f32,
}

fn default_detail_intensity() -> f32 {
    1.0
}

/// Types of detail maps
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DetailMapType {
    /// Erosion patterns
    Erosion,
    /// Cavity/occlusion
    Cavity,
    /// Vegetation density
    Vegetation,
    /// Rock distribution
    Rocks,
    /// Snow coverage
    Snow,
    /// Custom detail type
    Custom(String),
}

// ============================================================================
// Heightmap Data
// ============================================================================

/// Loaded heightmap data
#[derive(Debug, Clone)]
pub struct HeightmapData {
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Normalized height values (0.0-1.0)
    pub heights: Vec<f32>,
    /// Bits per pixel of source (8, 16, 32)
    pub bits_per_pixel: u8,
}

impl HeightmapData {
    /// Get height at pixel coordinates
    pub fn get_height(&self, x: u32, y: u32) -> f32 {
        if x >= self.width || y >= self.height {
            return 0.0;
        }
        let idx = (y * self.width + x) as usize;
        self.heights.get(idx).copied().unwrap_or(0.0)
    }

    /// Get interpolated height at normalized coordinates (0.0-1.0)
    pub fn sample(&self, u: f32, v: f32) -> f32 {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        let x = u * (self.width - 1) as f32;
        let y = v * (self.height - 1) as f32;

        let x0 = x.floor() as u32;
        let y0 = y.floor() as u32;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);

        let fx = x.fract();
        let fy = y.fract();

        // Bilinear interpolation
        let h00 = self.get_height(x0, y0);
        let h10 = self.get_height(x1, y0);
        let h01 = self.get_height(x0, y1);
        let h11 = self.get_height(x1, y1);

        let h0 = h00 * (1.0 - fx) + h10 * fx;
        let h1 = h01 * (1.0 - fx) + h11 * fx;

        h0 * (1.0 - fy) + h1 * fy
    }

    /// Calculate normal at pixel coordinates
    pub fn get_normal(&self, x: u32, y: u32, scale: f32) -> Vec3 {
        let h_left = if x > 0 {
            self.get_height(x - 1, y)
        } else {
            self.get_height(x, y)
        };
        let h_right = if x < self.width - 1 {
            self.get_height(x + 1, y)
        } else {
            self.get_height(x, y)
        };
        let h_down = if y > 0 {
            self.get_height(x, y - 1)
        } else {
            self.get_height(x, y)
        };
        let h_up = if y < self.height - 1 {
            self.get_height(x, y + 1)
        } else {
            self.get_height(x, y)
        };

        let dx = (h_right - h_left) * scale;
        let dz = (h_up - h_down) * scale;

        Vec3::new(-dx, 2.0, -dz).normalize()
    }

    /// Calculate slope angle at pixel (in degrees)
    pub fn get_slope(&self, x: u32, y: u32, scale: f32) -> f32 {
        let normal = self.get_normal(x, y, scale);
        let up = Vec3::Y;
        normal.dot(up).acos().to_degrees()
    }
}

// ============================================================================
// Heightmap Loader
// ============================================================================

/// Loader for terrain heightmaps
pub struct HeightmapLoader {
    base_path: PathBuf,
}

impl HeightmapLoader {
    /// Create a new heightmap loader
    pub fn new() -> Self {
        Self {
            base_path: PathBuf::new(),
        }
    }

    /// Set the base path for resolving relative paths
    pub fn with_base_path<P: AsRef<Path>>(mut self, path: P) -> Self {
        self.base_path = path.as_ref().to_path_buf();
        self
    }

    /// Load terrain configuration from YAML file
    pub fn load_config<P: AsRef<Path>>(&self, path: P) -> Result<TerrainConfig> {
        let path = self.resolve_path(path.as_ref());
        let content = std::fs::read_to_string(&path).map_err(|e| {
            EnhancedError::new(format!("Failed to read terrain config: {}", e))
                .with_file(&path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        serde_yaml::from_str(&content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse terrain config: {}", e))
                .with_file(&path)
                .with_category(ErrorCategory::ParseError)
        })
    }

    /// Load heightmap from image file
    pub fn load_heightmap<P: AsRef<Path>>(&self, path: P) -> Result<HeightmapData> {
        let path = self.resolve_path(path.as_ref());
        let extension = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_lowercase();

        match extension.as_str() {
            "png" => self.load_png(&path),
            "raw" | "r16" | "r32" => self.load_raw(&path),
            "tif" | "tiff" => self.load_tiff(&path),
            _ => Err(
                EnhancedError::new(format!("Unsupported heightmap format: {}", extension))
                    .with_file(&path)
                    .with_hint("Supported formats: PNG, RAW, R16, R32, TIFF"),
            ),
        }
    }

    /// Load PNG heightmap
    fn load_png(&self, path: &Path) -> Result<HeightmapData> {
        let file = std::fs::File::open(path).map_err(|e| {
            EnhancedError::new(format!("Failed to open heightmap: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        let decoder = png::Decoder::new(file);
        let mut reader = decoder.read_info().map_err(|e| {
            EnhancedError::new(format!("Failed to read PNG info: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::ParseError)
        })?;

        let mut buf = vec![0; reader.output_buffer_size()];
        let info = reader.next_frame(&mut buf).map_err(|e| {
            EnhancedError::new(format!("Failed to decode PNG frame: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::ParseError)
        })?;

        let width = info.width;
        let height = info.height;
        let bit_depth = info.bit_depth as u8;
        let color_type = info.color_type;

        // Convert to normalized heights
        let heights = match (color_type, bit_depth) {
            (png::ColorType::Grayscale, 8) => buf[..width as usize * height as usize]
                .iter()
                .map(|&v| v as f32 / 255.0)
                .collect(),
            (png::ColorType::Grayscale, 16) => {
                let mut heights = Vec::with_capacity(width as usize * height as usize);
                for chunk in buf.chunks(2) {
                    if chunk.len() == 2 {
                        let value = u16::from_be_bytes([chunk[0], chunk[1]]);
                        heights.push(value as f32 / 65535.0);
                    }
                }
                heights
            }
            (png::ColorType::Rgb, 8) | (png::ColorType::Rgba, 8) => {
                // Use red channel or average for color images
                let channels = if color_type == png::ColorType::Rgba {
                    4
                } else {
                    3
                };
                buf.chunks(channels).map(|c| c[0] as f32 / 255.0).collect()
            }
            _ => {
                return Err(EnhancedError::new(format!(
                    "Unsupported PNG format: {:?} {}bit",
                    color_type, bit_depth
                ))
                .with_file(path)
                .with_hint("Use grayscale 8-bit or 16-bit PNG for heightmaps"))
            }
        };

        Ok(HeightmapData {
            width,
            height,
            heights,
            bits_per_pixel: bit_depth * color_type.samples() as u8,
        })
    }

    /// Load RAW heightmap (headerless binary)
    fn load_raw(&self, path: &Path) -> Result<HeightmapData> {
        let data = std::fs::read(path).map_err(|e| {
            EnhancedError::new(format!("Failed to read RAW heightmap: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        let extension = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("raw")
            .to_lowercase();

        // Determine format from extension or file size
        let (bits_per_pixel, heights) = match extension.as_str() {
            "r16" => {
                let heights: Vec<f32> = data
                    .chunks(2)
                    .filter_map(|c| {
                        if c.len() == 2 {
                            Some(u16::from_le_bytes([c[0], c[1]]) as f32 / 65535.0)
                        } else {
                            None
                        }
                    })
                    .collect();
                (16u8, heights)
            }
            "r32" => {
                let heights: Vec<f32> = data
                    .chunks(4)
                    .filter_map(|c| {
                        if c.len() == 4 {
                            Some(f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                        } else {
                            None
                        }
                    })
                    .collect();
                (32u8, heights)
            }
            _ => {
                // Assume 8-bit grayscale
                let heights: Vec<f32> = data.iter().map(|&v| v as f32 / 255.0).collect();
                (8u8, heights)
            }
        };

        // Assume square heightmap
        let size = (heights.len() as f32).sqrt() as u32;
        if size * size != heights.len() as u32 {
            return Err(
                EnhancedError::new("RAW heightmap must be square (width == height)")
                    .with_file(path)
                    .with_hint("Common sizes: 257x257, 513x513, 1025x1025, 2049x2049"),
            );
        }

        Ok(HeightmapData {
            width: size,
            height: size,
            heights,
            bits_per_pixel,
        })
    }

    /// Load TIFF/GeoTIFF heightmap
    fn load_tiff(&self, path: &Path) -> Result<HeightmapData> {
        // Basic TIFF loading - for production would use tiff crate
        let data = std::fs::read(path).map_err(|e| {
            EnhancedError::new(format!("Failed to read TIFF heightmap: {}", e))
                .with_file(path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        // Check TIFF magic bytes
        if data.len() < 8 {
            return Err(EnhancedError::new("Invalid TIFF file: too short")
                .with_file(path)
                .with_category(ErrorCategory::ParseError));
        }

        let is_little_endian = &data[0..2] == b"II";
        let is_big_endian = &data[0..2] == b"MM";

        if !is_little_endian && !is_big_endian {
            return Err(EnhancedError::new("Invalid TIFF magic bytes")
                .with_file(path)
                .with_category(ErrorCategory::ParseError));
        }

        // Simple TIFF parser for uncompressed grayscale
        let read_u16 = |offset: usize| -> u16 {
            if is_little_endian {
                u16::from_le_bytes([data[offset], data[offset + 1]])
            } else {
                u16::from_be_bytes([data[offset], data[offset + 1]])
            }
        };

        let read_u32 = |offset: usize| -> u32 {
            if is_little_endian {
                u32::from_le_bytes([
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ])
            } else {
                u32::from_be_bytes([
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ])
            }
        };

        // Check TIFF version
        let version = read_u16(2);
        if version != 42 {
            return Err(
                EnhancedError::new(format!("Unsupported TIFF version: {}", version))
                    .with_file(path)
                    .with_hint("Only standard TIFF (version 42) is supported"),
            );
        }

        // Get IFD offset
        let ifd_offset = read_u32(4) as usize;
        if ifd_offset >= data.len() {
            return Err(EnhancedError::new("Invalid IFD offset")
                .with_file(path)
                .with_category(ErrorCategory::ParseError));
        }

        // Parse IFD entries
        let num_entries = read_u16(ifd_offset);
        let mut width: u32 = 0;
        let mut height: u32 = 0;
        let mut bits_per_sample: u16 = 8;
        let mut strip_offsets: Vec<u32> = Vec::new();
        let mut strip_byte_counts: Vec<u32> = Vec::new();
        let mut rows_per_strip: u32 = 0;

        for i in 0..num_entries as usize {
            let entry_offset = ifd_offset + 2 + i * 12;
            if entry_offset + 12 > data.len() {
                break;
            }

            let tag = read_u16(entry_offset);
            let field_type = read_u16(entry_offset + 2);
            let count = read_u32(entry_offset + 4);
            let value_offset = entry_offset + 8;

            let get_value = || -> u32 {
                match field_type {
                    3 => read_u16(value_offset) as u32, // SHORT
                    4 => read_u32(value_offset),        // LONG
                    _ => 0,
                }
            };

            match tag {
                256 => width = get_value(),                  // ImageWidth
                257 => height = get_value(),                 // ImageLength
                258 => bits_per_sample = get_value() as u16, // BitsPerSample
                273 => {
                    // StripOffsets
                    if count == 1 {
                        strip_offsets.push(get_value());
                    } else {
                        let offset = read_u32(value_offset) as usize;
                        for j in 0..count as usize {
                            if offset + j * 4 + 4 <= data.len() {
                                strip_offsets.push(read_u32(offset + j * 4));
                            }
                        }
                    }
                }
                278 => rows_per_strip = get_value(), // RowsPerStrip
                279 => {
                    // StripByteCounts
                    if count == 1 {
                        strip_byte_counts.push(get_value());
                    } else {
                        let offset = read_u32(value_offset) as usize;
                        for j in 0..count as usize {
                            if offset + j * 4 + 4 <= data.len() {
                                strip_byte_counts.push(read_u32(offset + j * 4));
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        if width == 0 || height == 0 {
            return Err(EnhancedError::new("Could not read TIFF dimensions")
                .with_file(path)
                .with_category(ErrorCategory::ParseError));
        }

        let _rows_per_strip = if rows_per_strip == 0 {
            height
        } else {
            rows_per_strip
        };

        // Read strip data
        let mut heights = Vec::with_capacity((width * height) as usize);

        for (strip_idx, &strip_offset) in strip_offsets.iter().enumerate() {
            let byte_count = strip_byte_counts.get(strip_idx).copied().unwrap_or(0) as usize;
            let strip_offset = strip_offset as usize;

            if strip_offset + byte_count > data.len() {
                continue;
            }

            let strip_data = &data[strip_offset..strip_offset + byte_count];

            match bits_per_sample {
                8 => {
                    for &byte in strip_data {
                        heights.push(byte as f32 / 255.0);
                    }
                }
                16 => {
                    for chunk in strip_data.chunks(2) {
                        if chunk.len() == 2 {
                            let value = if is_little_endian {
                                u16::from_le_bytes([chunk[0], chunk[1]])
                            } else {
                                u16::from_be_bytes([chunk[0], chunk[1]])
                            };
                            heights.push(value as f32 / 65535.0);
                        }
                    }
                }
                32 => {
                    for chunk in strip_data.chunks(4) {
                        if chunk.len() == 4 {
                            let value = if is_little_endian {
                                f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]])
                            } else {
                                f32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]])
                            };
                            // Normalize assuming typical DEM range
                            heights.push((value.clamp(-500.0, 9000.0) + 500.0) / 9500.0);
                        }
                    }
                }
                _ => {
                    return Err(EnhancedError::new(format!(
                        "Unsupported TIFF bit depth: {}",
                        bits_per_sample
                    ))
                    .with_file(path)
                    .with_hint("Supported: 8, 16, or 32 bits per sample"))
                }
            }
        }

        // Pad if needed
        let expected = (width * height) as usize;
        heights.resize(expected, 0.0);

        Ok(HeightmapData {
            width,
            height,
            heights,
            bits_per_pixel: bits_per_sample as u8,
        })
    }

    /// Resolve a path relative to base path
    fn resolve_path(&self, path: &Path) -> PathBuf {
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.base_path.join(path)
        }
    }
}

impl Default for HeightmapLoader {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Terrain Mesh Generator
// ============================================================================

/// Generates terrain meshes from heightmap data
pub struct TerrainMeshGenerator;

impl TerrainMeshGenerator {
    /// Generate a terrain mesh from heightmap data
    pub fn generate(heightmap: &HeightmapData, config: &TerrainConfig, lod_level: u32) -> Mesh {
        // Calculate step size based on LOD
        let step = 1u32 << lod_level;
        let vertices_x = ((heightmap.width - 1) / step + 1) as usize;
        let vertices_z = ((heightmap.height - 1) / step + 1) as usize;

        let scale_x = config.size.width / (heightmap.width - 1) as f32;
        let scale_z = config.size.length / (heightmap.height - 1) as f32;
        let scale_y = config.size.max_height * config.height_scale;

        let mut positions = Vec::with_capacity(vertices_x * vertices_z);
        let mut normals = Vec::with_capacity(vertices_x * vertices_z);
        let mut uvs = Vec::with_capacity(vertices_x * vertices_z);

        // Generate vertices
        for z in 0..vertices_z {
            for x in 0..vertices_x {
                let hx = (x as u32 * step).min(heightmap.width - 1);
                let hz = (z as u32 * step).min(heightmap.height - 1);

                let height = heightmap.get_height(hx, hz);
                let world_x = x as f32 * step as f32 * scale_x - config.size.width / 2.0;
                let world_y = height * scale_y + config.height_offset;
                let world_z = z as f32 * step as f32 * scale_z - config.size.length / 2.0;

                positions.push([world_x, world_y, world_z]);

                let normal = heightmap.get_normal(hx, hz, scale_y / scale_x);
                normals.push([normal.x, normal.y, normal.z]);

                uvs.push([
                    x as f32 / (vertices_x - 1) as f32,
                    z as f32 / (vertices_z - 1) as f32,
                ]);
            }
        }

        // Generate indices
        let mut indices = Vec::with_capacity((vertices_x - 1) * (vertices_z - 1) * 6);
        for z in 0..(vertices_z - 1) {
            for x in 0..(vertices_x - 1) {
                let i00 = (z * vertices_x + x) as u32;
                let i10 = i00 + 1;
                let i01 = i00 + vertices_x as u32;
                let i11 = i01 + 1;

                // Two triangles per quad
                indices.extend_from_slice(&[i00, i01, i10, i10, i01, i11]);
            }
        }

        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, default());
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }

    /// Generate a collision heightfield for physics
    pub fn generate_collision_data(
        heightmap: &HeightmapData,
        config: &TerrainConfig,
    ) -> CollisionHeightfield {
        let step = config.physics.physics_resolution;
        let cols = ((heightmap.width - 1) / step + 1) as usize;
        let rows = ((heightmap.height - 1) / step + 1) as usize;

        let scale_y = config.size.max_height * config.height_scale;

        let mut heights = Vec::with_capacity(rows * cols);
        for z in 0..rows {
            for x in 0..cols {
                let hx = (x as u32 * step).min(heightmap.width - 1);
                let hz = (z as u32 * step).min(heightmap.height - 1);
                let height = heightmap.get_height(hx, hz) * scale_y + config.height_offset;
                heights.push(height);
            }
        }

        CollisionHeightfield {
            heights,
            rows,
            cols,
            scale_x: config.size.width / (cols - 1) as f32,
            scale_z: config.size.length / (rows - 1) as f32,
        }
    }
}

/// Collision heightfield data for physics
#[derive(Debug, Clone)]
pub struct CollisionHeightfield {
    /// Height values in row-major order
    pub heights: Vec<f32>,
    /// Number of rows (Z direction)
    pub rows: usize,
    /// Number of columns (X direction)
    pub cols: usize,
    /// Scale in X direction
    pub scale_x: f32,
    /// Scale in Z direction
    pub scale_z: f32,
}

impl CollisionHeightfield {
    /// Get height at grid position
    pub fn get_height(&self, row: usize, col: usize) -> f32 {
        if row >= self.rows || col >= self.cols {
            return 0.0;
        }
        self.heights[row * self.cols + col]
    }

    /// Sample height at world position
    pub fn sample_at(&self, world_x: f32, world_z: f32) -> f32 {
        let half_width = (self.cols - 1) as f32 * self.scale_x / 2.0;
        let half_length = (self.rows - 1) as f32 * self.scale_z / 2.0;

        let local_x = world_x + half_width;
        let local_z = world_z + half_length;

        let col_f = local_x / self.scale_x;
        let row_f = local_z / self.scale_z;

        let col0 = (col_f.floor() as usize).min(self.cols.saturating_sub(1));
        let row0 = (row_f.floor() as usize).min(self.rows.saturating_sub(1));
        let col1 = (col0 + 1).min(self.cols - 1);
        let row1 = (row0 + 1).min(self.rows - 1);

        let fx = col_f.fract();
        let fz = row_f.fract();

        let h00 = self.get_height(row0, col0);
        let h10 = self.get_height(row0, col1);
        let h01 = self.get_height(row1, col0);
        let h11 = self.get_height(row1, col1);

        let h0 = h00 * (1.0 - fx) + h10 * fx;
        let h1 = h01 * (1.0 - fx) + h11 * fx;

        h0 * (1.0 - fz) + h1 * fz
    }

    /// Convert to nalgebra DMatrix for Rapier3D
    #[allow(dead_code)]
    pub fn to_nalgebra_matrix(&self) -> nalgebra::DMatrix<f32> {
        nalgebra::DMatrix::from_row_slice(self.rows, self.cols, &self.heights)
    }
}

// ============================================================================
// Terrain Component
// ============================================================================

/// Component marking an entity as terrain
#[derive(Component, Debug)]
pub struct Terrain {
    /// Terrain configuration
    pub config: TerrainConfig,
    /// Heightmap data
    pub heightmap: HeightmapData,
    /// Collision data
    pub collision: CollisionHeightfield,
}

impl Terrain {
    /// Get height at world position
    pub fn get_height_at(&self, x: f32, z: f32) -> f32 {
        self.collision.sample_at(x, z)
    }

    /// Get normal at world position
    pub fn get_normal_at(&self, x: f32, z: f32) -> Vec3 {
        let half_width = self.config.size.width / 2.0;
        let half_length = self.config.size.length / 2.0;

        let u = (x + half_width) / self.config.size.width;
        let v = (z + half_length) / self.config.size.length;

        let hx = (u * (self.heightmap.width - 1) as f32) as u32;
        let hz = (v * (self.heightmap.height - 1) as f32) as u32;

        let scale = self.config.size.max_height * self.config.height_scale
            / (self.config.size.width / self.heightmap.width as f32);

        self.heightmap.get_normal(hx, hz, scale)
    }
}

// ============================================================================
// Bevy Plugin
// ============================================================================

/// Plugin for terrain loading and rendering
pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<LoadTerrainEvent>()
            .add_systems(Update, handle_load_terrain);
    }
}

/// Event to load terrain
#[derive(Event)]
pub struct LoadTerrainEvent {
    /// Path to terrain config or heightmap
    pub path: PathBuf,
    /// Optional inline configuration
    pub config: Option<TerrainConfig>,
}

fn handle_load_terrain(
    mut events: EventReader<LoadTerrainEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for event in events.read() {
        let loader =
            HeightmapLoader::new().with_base_path(event.path.parent().unwrap_or(Path::new(".")));

        // Load or use provided config
        let config = match &event.config {
            Some(c) => c.clone(),
            None => match loader.load_config(&event.path) {
                Ok(c) => c,
                Err(e) => {
                    error!("Failed to load terrain config: {}", e);
                    continue;
                }
            },
        };

        // Load heightmap
        let heightmap_path = if event.config.is_some() {
            event.path.clone()
        } else {
            event
                .path
                .parent()
                .unwrap_or(Path::new("."))
                .join(&config.heightmap)
        };

        let heightmap = match loader.load_heightmap(&heightmap_path) {
            Ok(h) => h,
            Err(e) => {
                error!("Failed to load heightmap: {}", e);
                continue;
            }
        };

        // Generate mesh
        let mesh = TerrainMeshGenerator::generate(&heightmap, &config, 0);
        let mesh_handle = meshes.add(mesh);

        // Generate collision data
        let collision = TerrainMeshGenerator::generate_collision_data(&heightmap, &config);

        // Create material
        let material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        });

        // Spawn terrain entity
        commands.spawn((
            Terrain {
                config: config.clone(),
                heightmap,
                collision,
            },
            Mesh3d(mesh_handle),
            MeshMaterial3d(material),
            Transform::default(),
            GlobalTransform::default(),
            Name::new(config.name),
        ));

        info!("Terrain loaded successfully");
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heightmap_data() {
        let heightmap = HeightmapData {
            width: 3,
            height: 3,
            heights: vec![0.0, 0.5, 1.0, 0.25, 0.5, 0.75, 0.5, 0.5, 0.5],
            bits_per_pixel: 8,
        };

        assert_eq!(heightmap.get_height(0, 0), 0.0);
        assert_eq!(heightmap.get_height(1, 0), 0.5);
        assert_eq!(heightmap.get_height(2, 0), 1.0);

        // Test sampling
        let center = heightmap.sample(0.5, 0.5);
        assert!((center - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_terrain_config_serde() {
        let config = TerrainConfig {
            name: "test_terrain".to_string(),
            heightmap: "heightmap.png".to_string(),
            size: TerrainSize {
                width: 1000.0,
                length: 1000.0,
                max_height: 100.0,
            },
            height_scale: 1.0,
            height_offset: 0.0,
            physics: TerrainPhysics::default(),
            texture_layers: vec![],
            lod: Some(TerrainLOD::default()),
            detail_maps: vec![],
        };

        let yaml = serde_yaml::to_string(&config).unwrap();
        let loaded: TerrainConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(loaded.name, "test_terrain");
    }

    #[test]
    fn test_collision_heightfield() {
        let collision = CollisionHeightfield {
            heights: vec![0.0, 1.0, 2.0, 3.0],
            rows: 2,
            cols: 2,
            scale_x: 10.0,
            scale_z: 10.0,
        };

        // Test corners
        assert_eq!(collision.get_height(0, 0), 0.0);
        assert_eq!(collision.get_height(0, 1), 1.0);
        assert_eq!(collision.get_height(1, 0), 2.0);
        assert_eq!(collision.get_height(1, 1), 3.0);
    }
}
