//! USD (Universal Scene Description) importer
//!
//! USD is Pixar's open-source scene description format, increasingly used in robotics
//! through NVIDIA Isaac Sim and Omniverse. This module provides parsing and loading
//! of USD/USDA/USDC files for robotics simulation.
//!
//! Supports:
//! - USDA (ASCII) and USDC (binary crate) formats
//! - USD physics schema (rigid bodies, joints, colliders)
//! - USD geometry (meshes, primitives)
//! - Articulation root and joint drive
//!
//! Note: Full USD support requires the `usd-rs` crate or OpenUSD bindings.
//! This implementation provides a lightweight parser for common robotics use cases.

use anyhow::{Context, Result};
use bevy::prelude::*;
use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use tracing::{debug, info, warn};

/// USD Stage (scene graph root)
#[derive(Debug, Clone)]
pub struct USDStage {
    /// Stage metadata
    pub metadata: USDMetadata,
    /// Root prims (top-level objects)
    pub root_prims: Vec<USDPrim>,
    /// Default prim path
    pub default_prim: Option<String>,
    /// Up axis (Y or Z)
    pub up_axis: UpAxis,
    /// Meters per unit
    pub meters_per_unit: f32,
}

impl Default for USDStage {
    fn default() -> Self {
        Self {
            metadata: USDMetadata::default(),
            root_prims: Vec::new(),
            default_prim: None,
            up_axis: UpAxis::Y,
            meters_per_unit: 0.01, // cm by default (USD convention)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpAxis {
    Y,
    Z,
}

/// USD metadata
#[derive(Debug, Clone, Default)]
pub struct USDMetadata {
    pub doc: Option<String>,
    pub comment: Option<String>,
    pub custom_layer_data: HashMap<String, String>,
}

/// USD Prim (primitive - base scene graph node)
#[derive(Debug, Clone)]
pub struct USDPrim {
    /// Prim path (e.g., "/World/Robot/base_link")
    pub path: String,
    /// Prim name (last component of path)
    pub name: String,
    /// Prim type (Xform, Mesh, Cube, etc.)
    pub prim_type: USDPrimType,
    /// Transform
    pub transform: USDTransform,
    /// Properties/attributes
    pub properties: HashMap<String, USDValue>,
    /// Physics properties
    pub physics: Option<USDPhysics>,
    /// Geometry data
    pub geometry: Option<USDGeometry>,
    /// Material binding
    pub material: Option<String>,
    /// Child prims
    pub children: Vec<USDPrim>,
    /// References to other USD files
    pub references: Vec<USDReference>,
    /// Variants
    pub variants: HashMap<String, String>,
    /// Whether this prim is active
    pub active: bool,
    /// Custom properties
    pub custom: HashMap<String, USDValue>,
}

impl Default for USDPrim {
    fn default() -> Self {
        Self {
            path: String::new(),
            name: String::new(),
            prim_type: USDPrimType::Xform,
            transform: USDTransform::default(),
            properties: HashMap::new(),
            physics: None,
            geometry: None,
            material: None,
            children: Vec::new(),
            references: Vec::new(),
            variants: HashMap::new(),
            active: true,
            custom: HashMap::new(),
        }
    }
}

/// USD prim types
#[derive(Debug, Clone, PartialEq)]
pub enum USDPrimType {
    Xform,
    Scope,
    Mesh,
    Cube,
    Sphere,
    Cylinder,
    Capsule,
    Cone,
    Plane,
    Camera,
    Light,
    Material,
    Shader,
    // Physics types
    PhysicsScene,
    RigidBodyAPI,
    CollisionAPI,
    ArticulationRootAPI,
    // Joint types
    PhysicsRevoluteJoint,
    PhysicsPrismaticJoint,
    PhysicsFixedJoint,
    PhysicsSphericalJoint,
    PhysicsDistanceJoint,
    PhysicsD6Joint,
    // Custom/unknown
    Custom(String),
}

impl From<&str> for USDPrimType {
    fn from(s: &str) -> Self {
        match s {
            "Xform" => USDPrimType::Xform,
            "Scope" => USDPrimType::Scope,
            "Mesh" => USDPrimType::Mesh,
            "Cube" => USDPrimType::Cube,
            "Sphere" => USDPrimType::Sphere,
            "Cylinder" => USDPrimType::Cylinder,
            "Capsule" => USDPrimType::Capsule,
            "Cone" => USDPrimType::Cone,
            "Plane" => USDPrimType::Plane,
            "Camera" => USDPrimType::Camera,
            "DistantLight" | "DomeLight" | "SphereLight" | "RectLight" => USDPrimType::Light,
            "Material" => USDPrimType::Material,
            "Shader" => USDPrimType::Shader,
            "PhysicsScene" => USDPrimType::PhysicsScene,
            "PhysicsRevoluteJoint" => USDPrimType::PhysicsRevoluteJoint,
            "PhysicsPrismaticJoint" => USDPrimType::PhysicsPrismaticJoint,
            "PhysicsFixedJoint" => USDPrimType::PhysicsFixedJoint,
            "PhysicsSphericalJoint" => USDPrimType::PhysicsSphericalJoint,
            "PhysicsDistanceJoint" => USDPrimType::PhysicsDistanceJoint,
            "PhysicsD6Joint" => USDPrimType::PhysicsD6Joint,
            other => USDPrimType::Custom(other.to_string()),
        }
    }
}

/// USD transform
#[derive(Debug, Clone)]
pub struct USDTransform {
    pub translate: Vec3,
    pub rotate: Quat,
    pub scale: Vec3,
    /// Full 4x4 transform matrix (if specified directly)
    pub matrix: Option<[[f32; 4]; 4]>,
}

impl Default for USDTransform {
    fn default() -> Self {
        Self {
            translate: Vec3::ZERO,
            rotate: Quat::IDENTITY,
            scale: Vec3::ONE,
            matrix: None,
        }
    }
}

impl USDTransform {
    /// Convert to Bevy Transform
    pub fn to_transform(&self) -> Transform {
        if let Some(matrix) = &self.matrix {
            // Extract TRS from matrix
            let mat = Mat4::from_cols_array_2d(matrix);
            let (scale, rotation, translation) = mat.to_scale_rotation_translation();
            Transform {
                translation,
                rotation,
                scale,
            }
        } else {
            Transform {
                translation: self.translate,
                rotation: self.rotate,
                scale: self.scale,
            }
        }
    }
}

/// USD physics properties
#[derive(Debug, Clone)]
pub struct USDPhysics {
    /// Rigid body properties
    pub rigid_body: Option<USDRigidBody>,
    /// Collision properties
    pub collision: Option<USDCollision>,
    /// Articulation root
    pub articulation_root: bool,
    /// Joint drive settings
    pub joint_drive: Option<USDJointDrive>,
}

#[derive(Debug, Clone)]
pub struct USDRigidBody {
    pub enabled: bool,
    pub kinematic: bool,
    pub mass: Option<f32>,
    pub density: Option<f32>,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl Default for USDRigidBody {
    fn default() -> Self {
        Self {
            enabled: true,
            kinematic: false,
            mass: None,
            density: None,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

#[derive(Debug, Clone)]
pub struct USDCollision {
    pub enabled: bool,
    pub collision_group: u32,
    pub simulation_owner: Option<String>,
}

impl Default for USDCollision {
    fn default() -> Self {
        Self {
            enabled: true,
            collision_group: 0,
            simulation_owner: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct USDJointDrive {
    pub drive_type: String, // "force" or "acceleration"
    pub stiffness: f32,
    pub damping: f32,
    pub max_force: f32,
    pub target_position: Option<f32>,
    pub target_velocity: Option<f32>,
}

/// USD geometry data
#[derive(Debug, Clone)]
pub struct USDGeometry {
    pub geom_type: USDGeomType,
    /// For meshes: vertex positions
    pub points: Vec<Vec3>,
    /// For meshes: face vertex counts
    pub face_vertex_counts: Vec<u32>,
    /// For meshes: face vertex indices
    pub face_vertex_indices: Vec<u32>,
    /// Normals
    pub normals: Vec<Vec3>,
    /// UVs
    pub uvs: Vec<Vec2>,
    /// Extent (bounding box)
    pub extent: Option<[Vec3; 2]>,
    /// Primitive-specific parameters
    pub params: HashMap<String, f32>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum USDGeomType {
    Mesh,
    Cube,
    Sphere,
    Cylinder,
    Capsule,
    Cone,
    Plane,
}

impl Default for USDGeometry {
    fn default() -> Self {
        Self {
            geom_type: USDGeomType::Mesh,
            points: Vec::new(),
            face_vertex_counts: Vec::new(),
            face_vertex_indices: Vec::new(),
            normals: Vec::new(),
            uvs: Vec::new(),
            extent: None,
            params: HashMap::new(),
        }
    }
}

/// USD value types
#[derive(Debug, Clone)]
pub enum USDValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    Double(f64),
    String(String),
    Token(String),
    Asset(String),
    Vec2f([f32; 2]),
    Vec3f([f32; 3]),
    Vec4f([f32; 4]),
    Quatf([f32; 4]),
    Matrix4d([[f64; 4]; 4]),
    IntArray(Vec<i64>),
    FloatArray(Vec<f64>),
    Vec3fArray(Vec<[f32; 3]>),
    TokenArray(Vec<String>),
    Relationship(String),
}

/// USD reference
#[derive(Debug, Clone)]
pub struct USDReference {
    pub asset_path: String,
    pub prim_path: Option<String>,
    pub layer_offset: Option<f64>,
}

/// USD importer
pub struct USDImporter {
    base_path: PathBuf,
}

impl Default for USDImporter {
    fn default() -> Self {
        Self::new()
    }
}

impl USDImporter {
    pub fn new() -> Self {
        Self {
            base_path: PathBuf::from("."),
        }
    }

    /// Load USD file (auto-detects USDA vs USDC)
    pub fn load_file(&mut self, path: impl AsRef<Path>) -> Result<USDStage> {
        let path = path.as_ref();
        self.base_path = path.parent().unwrap_or(Path::new(".")).to_path_buf();

        let extension = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_lowercase();

        match extension.as_str() {
            "usda" | "usd" => self.load_usda(path),
            "usdc" => self.load_usdc(path),
            "usdz" => self.load_usdz(path),
            _ => anyhow::bail!("Unknown USD file format: {}", extension),
        }
    }

    /// Load USDA (ASCII) format
    fn load_usda(&self, path: &Path) -> Result<USDStage> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read USDA file: {}", path.display()))?;

        self.parse_usda(&content)
    }

    /// Load USDC (binary crate) format
    fn load_usdc(&self, path: &Path) -> Result<USDStage> {
        // USDC is a binary format - for full support would need OpenUSD bindings
        // For now, try to read it as USDA (some tools output text with .usdc extension)
        // or return an error suggesting conversion

        let mut file = File::open(path)
            .with_context(|| format!("Failed to open USDC file: {}", path.display()))?;

        let mut header = [0u8; 8];
        file.read_exact(&mut header).ok();

        // Check for "PXR-USDC" magic header
        if &header == b"PXR-USDC" {
            // This is binary USDC
            warn!("Binary USDC format detected. For full support, convert to USDA using: usdcat {} -o output.usda", path.display());
            anyhow::bail!(
                "Binary USDC format not fully supported. Please convert to USDA using: usdcat {} -o output.usda",
                path.display()
            );
        }

        // Try parsing as text anyway
        self.load_usda(path)
    }

    /// Load USDZ (zipped USD) format
    fn load_usdz(&self, path: &Path) -> Result<USDStage> {
        // USDZ is a zip archive containing USD files and assets
        warn!("USDZ format support is limited. Extract the archive for full support.");

        // Try to extract and find the root USD file
        let file = File::open(path)
            .with_context(|| format!("Failed to open USDZ file: {}", path.display()))?;

        let mut archive =
            zip::ZipArchive::new(file).with_context(|| "Failed to read USDZ as zip archive")?;

        // Find the root USD file (usually first .usd/.usda/.usdc file)
        for i in 0..archive.len() {
            let mut file = archive.by_index(i)?;
            let name = file.name().to_lowercase();

            if name.ends_with(".usda") || name.ends_with(".usd") {
                let mut content = String::new();
                file.read_to_string(&mut content)?;
                return self.parse_usda(&content);
            }
        }

        anyhow::bail!("No USD file found in USDZ archive")
    }

    /// Parse USDA content
    fn parse_usda(&self, content: &str) -> Result<USDStage> {
        let mut stage = USDStage::default();
        let lines: Vec<&str> = content.lines().collect();
        let mut index = 0;

        // Parse header/metadata
        while index < lines.len() {
            let line = lines[index].trim();

            if line.starts_with("#usda") {
                // Version line
                index += 1;
                continue;
            }

            if line.starts_with('(') {
                // Stage metadata block
                index = self.parse_metadata_block(&lines, index, &mut stage)?;
                continue;
            }

            if line.starts_with("def ") || line.starts_with("over ") || line.starts_with("class ") {
                // Prim definition
                let (prim, new_index) = self.parse_prim(&lines, index, "")?;
                stage.root_prims.push(prim);
                index = new_index;
                continue;
            }

            index += 1;
        }

        info!(
            "Parsed USD stage with {} root prims, up_axis={:?}, meters_per_unit={}",
            stage.root_prims.len(),
            stage.up_axis,
            stage.meters_per_unit
        );

        Ok(stage)
    }

    fn parse_metadata_block(
        &self,
        lines: &[&str],
        start: usize,
        stage: &mut USDStage,
    ) -> Result<usize> {
        let mut index = start;
        let mut depth = 0;

        while index < lines.len() {
            let line = lines[index].trim();

            if line.contains('(') {
                depth += line.matches('(').count();
            }
            if line.contains(')') {
                depth -= line.matches(')').count();
            }

            // Parse metadata attributes
            if let Some(value) = self.extract_attribute(line, "defaultPrim") {
                stage.default_prim = Some(value.trim_matches('"').to_string());
            }
            if let Some(value) = self.extract_attribute(line, "upAxis") {
                stage.up_axis = match value.trim_matches('"') {
                    "Z" => UpAxis::Z,
                    _ => UpAxis::Y,
                };
            }
            if let Some(value) = self.extract_attribute(line, "metersPerUnit") {
                stage.meters_per_unit = value.parse().unwrap_or(0.01);
            }
            if let Some(value) = self.extract_attribute(line, "doc") {
                stage.metadata.doc = Some(value.trim_matches('"').to_string());
            }

            index += 1;

            if depth == 0 {
                break;
            }
        }

        Ok(index)
    }

    fn parse_prim(
        &self,
        lines: &[&str],
        start: usize,
        parent_path: &str,
    ) -> Result<(USDPrim, usize)> {
        let mut prim = USDPrim::default();
        let mut index = start;

        // Parse def/over/class line
        let def_line = lines[index].trim();

        // Extract prim type and name: def Type "name" or def "name"
        let parts: Vec<&str> = def_line.split_whitespace().collect();
        if parts.len() >= 2 {
            let keyword = parts[0]; // def, over, class

            if parts.len() >= 3 && !parts[1].starts_with('"') {
                // def Type "name"
                prim.prim_type = USDPrimType::from(parts[1]);
                prim.name = parts[2].trim_matches('"').to_string();
            } else if parts.len() >= 2 {
                // def "name"
                prim.name = parts[1].trim_matches('"').to_string();
            }
        }

        // Build full path
        prim.path = if parent_path.is_empty() {
            format!("/{}", prim.name)
        } else {
            format!("{}/{}", parent_path, prim.name)
        };

        debug!("Parsing prim: {} ({})", prim.path, def_line);

        index += 1;
        let mut depth = 1; // Started with opening brace

        // Find the opening brace if not on same line
        while index < lines.len() && !lines[index].contains('{') {
            // Parse attributes before opening brace
            let line = lines[index].trim();
            self.parse_prim_attribute(line, &mut prim);
            index += 1;
        }

        if index < lines.len() && lines[index].contains('{') {
            index += 1;
        }

        // Parse prim body
        while index < lines.len() && depth > 0 {
            let line = lines[index].trim();

            // Track brace depth
            for c in line.chars() {
                match c {
                    '{' => depth += 1,
                    '}' => depth -= 1,
                    _ => {}
                }
            }

            if depth <= 0 {
                index += 1;
                break;
            }

            // Parse child prim
            if line.starts_with("def ") || line.starts_with("over ") || line.starts_with("class ") {
                let (child, new_index) = self.parse_prim(lines, index, &prim.path)?;
                prim.children.push(child);
                index = new_index;
                continue;
            }

            // Parse attributes
            self.parse_prim_attribute(line, &mut prim);
            index += 1;
        }

        Ok((prim, index))
    }

    fn parse_prim_attribute(&self, line: &str, prim: &mut USDPrim) {
        // Skip comments and empty lines
        if line.starts_with('#') || line.is_empty() || line == "{" || line == "}" {
            return;
        }

        // Parse transform attributes
        if let Some(value) = self.extract_attribute(line, "xformOp:translate") {
            if let Some(vec) = self.parse_vec3(&value) {
                prim.transform.translate = vec;
            }
        }
        if let Some(value) = self.extract_attribute(line, "xformOp:rotateXYZ") {
            if let Some(vec) = self.parse_vec3(&value) {
                prim.transform.rotate = Quat::from_euler(
                    EulerRot::XYZ,
                    vec.x.to_radians(),
                    vec.y.to_radians(),
                    vec.z.to_radians(),
                );
            }
        }
        if let Some(value) = self.extract_attribute(line, "xformOp:orient") {
            if let Some(quat) = self.parse_quat(&value) {
                prim.transform.rotate = quat;
            }
        }
        if let Some(value) = self.extract_attribute(line, "xformOp:scale") {
            if let Some(vec) = self.parse_vec3(&value) {
                prim.transform.scale = vec;
            }
        }

        // Parse physics attributes
        if (line.contains("PhysicsRigidBodyAPI") || line.contains("physics:rigidBodyEnabled"))
            && prim.physics.is_none()
        {
            prim.physics = Some(USDPhysics {
                rigid_body: Some(USDRigidBody::default()),
                collision: None,
                articulation_root: false,
                joint_drive: None,
            });
        }
        if line.contains("PhysicsCollisionAPI") || line.contains("physics:collisionEnabled") {
            if let Some(physics) = &mut prim.physics {
                physics.collision = Some(USDCollision::default());
            } else {
                prim.physics = Some(USDPhysics {
                    rigid_body: None,
                    collision: Some(USDCollision::default()),
                    articulation_root: false,
                    joint_drive: None,
                });
            }
        }
        if line.contains("PhysicsArticulationRootAPI") {
            if let Some(physics) = &mut prim.physics {
                physics.articulation_root = true;
            }
        }
        if let Some(value) = self.extract_attribute(line, "physics:mass") {
            if let Some(physics) = &mut prim.physics {
                if let Some(rb) = &mut physics.rigid_body {
                    rb.mass = value.parse().ok();
                }
            }
        }

        // Parse geometry attributes
        if let Some(value) = self.extract_attribute(line, "size") {
            if prim.geometry.is_none() {
                prim.geometry = Some(USDGeometry::default());
            }
            if let Some(geom) = &mut prim.geometry {
                if let Ok(size) = value.parse::<f32>() {
                    geom.params.insert("size".to_string(), size);
                }
            }
        }
        if let Some(value) = self.extract_attribute(line, "radius") {
            if prim.geometry.is_none() {
                prim.geometry = Some(USDGeometry::default());
            }
            if let Some(geom) = &mut prim.geometry {
                if let Ok(radius) = value.parse::<f32>() {
                    geom.params.insert("radius".to_string(), radius);
                }
            }
        }
        if let Some(value) = self.extract_attribute(line, "height") {
            if prim.geometry.is_none() {
                prim.geometry = Some(USDGeometry::default());
            }
            if let Some(geom) = &mut prim.geometry {
                if let Ok(height) = value.parse::<f32>() {
                    geom.params.insert("height".to_string(), height);
                }
            }
        }

        // Parse material binding
        if let Some(value) = self.extract_attribute(line, "material:binding") {
            prim.material = Some(value.trim_matches(|c| c == '<' || c == '>').to_string());
        }

        // Parse references
        if line.contains("references") {
            if let Some(asset) = self.extract_asset_path(line) {
                prim.references.push(USDReference {
                    asset_path: asset,
                    prim_path: None,
                    layer_offset: None,
                });
            }
        }

        // Parse active state
        if let Some(value) = self.extract_attribute(line, "active") {
            prim.active = value != "false" && value != "0";
        }
    }

    fn extract_attribute(&self, line: &str, attr_name: &str) -> Option<String> {
        // Match patterns like: attr_name = value or attr_name = "value"
        let patterns = [
            format!("{} = ", attr_name),
            format!("{}=", attr_name),
            format!("{} (", attr_name),
        ];

        for pattern in &patterns {
            if let Some(pos) = line.find(pattern) {
                let after_eq = &line[pos + pattern.len()..];
                // Extract value (handle quotes, parens, etc.)
                let value = after_eq
                    .trim()
                    .trim_matches(|c| c == '"' || c == '\'' || c == '(' || c == ')')
                    .trim()
                    .to_string();
                return Some(value);
            }
        }
        None
    }

    fn extract_asset_path(&self, line: &str) -> Option<String> {
        // Match @path@ pattern
        if let Some(start) = line.find('@') {
            if let Some(end) = line[start + 1..].find('@') {
                return Some(line[start + 1..start + 1 + end].to_string());
            }
        }
        None
    }

    fn parse_vec3(&self, value: &str) -> Option<Vec3> {
        // Parse (x, y, z) or x, y, z
        let cleaned = value
            .trim()
            .trim_matches(|c| c == '(' || c == ')' || c == '[' || c == ']');
        let parts: Vec<f32> = cleaned
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();

        if parts.len() >= 3 {
            Some(Vec3::new(parts[0], parts[1], parts[2]))
        } else {
            None
        }
    }

    fn parse_quat(&self, value: &str) -> Option<Quat> {
        // Parse (w, x, y, z) - USD uses wxyz
        let cleaned = value
            .trim()
            .trim_matches(|c| c == '(' || c == ')' || c == '[' || c == ']');
        let parts: Vec<f32> = cleaned
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();

        if parts.len() >= 4 {
            Some(Quat::from_xyzw(parts[1], parts[2], parts[3], parts[0]))
        } else {
            None
        }
    }

    /// Convert USD stage to list of spawn-ready entities
    pub fn to_spawn_data(&self, stage: &USDStage) -> Vec<USDSpawnData> {
        let mut data = Vec::new();
        let scale = stage.meters_per_unit;
        let up_conversion = match stage.up_axis {
            UpAxis::Z => Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            UpAxis::Y => Quat::IDENTITY,
        };

        for prim in &stage.root_prims {
            self.collect_spawn_data(prim, scale, up_conversion, &mut data);
        }

        data
    }

    #[allow(clippy::only_used_in_recursion)]
    fn collect_spawn_data(
        &self,
        prim: &USDPrim,
        scale: f32,
        up_conversion: Quat,
        data: &mut Vec<USDSpawnData>,
    ) {
        if !prim.active {
            return;
        }

        // Check if this prim should be spawned as an entity
        let spawn = match &prim.prim_type {
            USDPrimType::Mesh
            | USDPrimType::Cube
            | USDPrimType::Sphere
            | USDPrimType::Cylinder
            | USDPrimType::Capsule
            | USDPrimType::Cone
            | USDPrimType::Plane => {
                let mut transform = prim.transform.to_transform();
                transform.translation *= scale;
                transform.rotation = up_conversion * transform.rotation;

                Some(USDSpawnData {
                    path: prim.path.clone(),
                    name: prim.name.clone(),
                    transform,
                    prim_type: prim.prim_type.clone(),
                    geometry: prim.geometry.clone(),
                    physics: prim.physics.clone(),
                    material: prim.material.clone(),
                })
            }
            USDPrimType::Xform if prim.physics.is_some() => {
                // Xform with physics = rigid body
                let mut transform = prim.transform.to_transform();
                transform.translation *= scale;
                transform.rotation = up_conversion * transform.rotation;

                Some(USDSpawnData {
                    path: prim.path.clone(),
                    name: prim.name.clone(),
                    transform,
                    prim_type: prim.prim_type.clone(),
                    geometry: None,
                    physics: prim.physics.clone(),
                    material: None,
                })
            }
            USDPrimType::PhysicsRevoluteJoint
            | USDPrimType::PhysicsPrismaticJoint
            | USDPrimType::PhysicsFixedJoint
            | USDPrimType::PhysicsSphericalJoint => {
                // Joint prims
                Some(USDSpawnData {
                    path: prim.path.clone(),
                    name: prim.name.clone(),
                    transform: prim.transform.to_transform(),
                    prim_type: prim.prim_type.clone(),
                    geometry: None,
                    physics: prim.physics.clone(),
                    material: None,
                })
            }
            _ => None,
        };

        if let Some(s) = spawn {
            data.push(s);
        }

        // Recurse to children
        for child in &prim.children {
            self.collect_spawn_data(child, scale, up_conversion, data);
        }
    }
}

/// Data needed to spawn a USD prim as an entity
#[derive(Debug, Clone)]
pub struct USDSpawnData {
    pub path: String,
    pub name: String,
    pub transform: Transform,
    pub prim_type: USDPrimType,
    pub geometry: Option<USDGeometry>,
    pub physics: Option<USDPhysics>,
    pub material: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_usda() {
        let usda = r#"#usda 1.0
(
    defaultPrim = "World"
    upAxis = "Z"
    metersPerUnit = 0.01
)

def Xform "World"
{
    def Sphere "ball"
    {
        double radius = 0.5
        float3 xformOp:translate = (0, 0, 1)
    }
}
"#;

        let importer = USDImporter::new();
        let stage = importer.parse_usda(usda).unwrap();

        assert_eq!(stage.default_prim, Some("World".to_string()));
        assert_eq!(stage.up_axis, UpAxis::Z);
        assert_eq!(stage.meters_per_unit, 0.01);
        assert_eq!(stage.root_prims.len(), 1);
        assert_eq!(stage.root_prims[0].name, "World");
    }

    #[test]
    fn test_parse_prim_type() {
        assert_eq!(USDPrimType::from("Xform"), USDPrimType::Xform);
        assert_eq!(USDPrimType::from("Mesh"), USDPrimType::Mesh);
        assert_eq!(
            USDPrimType::from("PhysicsRevoluteJoint"),
            USDPrimType::PhysicsRevoluteJoint
        );
    }

    #[test]
    fn test_parse_transform() {
        let importer = USDImporter::new();

        let vec = importer.parse_vec3("(1.0, 2.0, 3.0)").unwrap();
        assert_eq!(vec, Vec3::new(1.0, 2.0, 3.0));

        let quat = importer.parse_quat("(1.0, 0.0, 0.0, 0.0)").unwrap();
        // USD quaternion is wxyz, we convert to xyzw
        assert!((quat.w - 1.0).abs() < 0.001);
    }
}
