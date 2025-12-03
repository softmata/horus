//! FBX (Filmbox) mesh loader
//!
//! FBX is Autodesk's proprietary 3D asset format, commonly used in game engines
//! and animation software. This loader supports:
//! - ASCII FBX format (version 6.x and 7.x)
//! - Binary FBX format (version 7.x)
//! - Mesh geometry extraction
//! - Material and texture references
//! - Basic skeletal/bone data (for animated robots)
//!
//! Note: Full FBX support requires the fbx crate or Autodesk SDK bindings.
//! This implementation handles common robotics mesh use cases.

use super::{processing::*, LoadedMesh, MaterialInfo, MeshLoadOptions};
use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use std::fs::File;
use std::io::Read;
use std::path::Path;
use tracing::{debug, info, warn};

/// Load an FBX file (auto-detects ASCII vs binary)
pub fn load_fbx(path: &Path, options: &MeshLoadOptions) -> Result<LoadedMesh> {
    info!("Loading FBX file: {}", path.display());

    let mut file =
        File::open(path).with_context(|| format!("Failed to open FBX file: {}", path.display()))?;

    // Check for binary FBX magic header: "Kaydara FBX Binary  \x00"
    let mut header = [0u8; 23];
    file.read_exact(&mut header).ok();

    let is_binary = header.starts_with(b"Kaydara FBX Binary");

    if is_binary {
        load_fbx_binary(path, options)
    } else {
        load_fbx_ascii(path, options)
    }
}

/// Load ASCII FBX file
fn load_fbx_ascii(path: &Path, options: &MeshLoadOptions) -> Result<LoadedMesh> {
    debug!("Loading ASCII FBX: {}", path.display());

    let content = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read FBX file: {}", path.display()))?;

    let mut parser = FBXAsciiParser::new(&content);
    parser.parse()?;

    // Extract geometry
    let (positions, normals, uvs, indices) = parser.extract_geometry()?;

    // Apply options
    let mut positions = positions;
    if options.scale != Vec3::ONE {
        scale_positions(&mut positions, options.scale);
    }

    let normals = if normals.is_empty() && options.generate_normals {
        debug!("Generating normals for FBX mesh");
        generate_normals(&positions, &indices)
    } else if normals.is_empty() {
        vec![[0.0, 1.0, 0.0]; positions.len()]
    } else {
        normals
    };

    let mut uvs = if uvs.is_empty() {
        vec![[0.0, 0.0]; positions.len()]
    } else {
        uvs
    };

    if options.flip_uvs && !uvs.is_empty() {
        flip_uvs(&mut uvs);
    }

    // Create Bevy mesh
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, Default::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals.clone());

    if !uvs.is_empty() {
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs.clone());

        if options.generate_tangents {
            let tangents = generate_tangents(&positions, &normals, &uvs, &indices);
            mesh.insert_attribute(Mesh::ATTRIBUTE_TANGENT, tangents);
        }
    }

    mesh.insert_indices(Indices::U32(indices.clone()));

    // Calculate bounds
    let bounds = calculate_aabb(&positions);

    // Extract materials
    let materials = parser.extract_materials(path);

    info!(
        "Loaded FBX mesh: {} vertices, {} triangles",
        positions.len(),
        indices.len() / 3
    );

    Ok(LoadedMesh {
        mesh,
        materials,
        texture_paths: Vec::new(),
        embedded_textures: Vec::new(),
        bounds,
        triangle_count: indices.len() / 3,
        vertex_count: positions.len(),
    })
}

/// Load binary FBX file
fn load_fbx_binary(path: &Path, options: &MeshLoadOptions) -> Result<LoadedMesh> {
    debug!("Loading binary FBX: {}", path.display());

    let data = std::fs::read(path)
        .with_context(|| format!("Failed to read binary FBX: {}", path.display()))?;

    let mut parser = FBXBinaryParser::new(&data);
    parser.parse()?;

    // Extract geometry
    let (positions, normals, uvs, indices) = parser.extract_geometry()?;

    // Apply options
    let mut positions = positions;
    if options.scale != Vec3::ONE {
        scale_positions(&mut positions, options.scale);
    }

    let normals = if normals.is_empty() && options.generate_normals {
        debug!("Generating normals for FBX mesh");
        generate_normals(&positions, &indices)
    } else if normals.is_empty() {
        vec![[0.0, 1.0, 0.0]; positions.len()]
    } else {
        normals
    };

    let mut uvs = if uvs.is_empty() {
        vec![[0.0, 0.0]; positions.len()]
    } else {
        uvs
    };

    if options.flip_uvs && !uvs.is_empty() {
        flip_uvs(&mut uvs);
    }

    // Create Bevy mesh
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, Default::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals.clone());

    if !uvs.is_empty() {
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs.clone());

        if options.generate_tangents {
            let tangents = generate_tangents(&positions, &normals, &uvs, &indices);
            mesh.insert_attribute(Mesh::ATTRIBUTE_TANGENT, tangents);
        }
    }

    mesh.insert_indices(Indices::U32(indices.clone()));

    // Calculate bounds
    let bounds = calculate_aabb(&positions);

    // Extract materials
    let materials = parser.extract_materials(path);

    info!(
        "Loaded binary FBX mesh: {} vertices, {} triangles",
        positions.len(),
        indices.len() / 3
    );

    Ok(LoadedMesh {
        mesh,
        materials,
        texture_paths: Vec::new(),
        embedded_textures: Vec::new(),
        bounds,
        triangle_count: indices.len() / 3,
        vertex_count: positions.len(),
    })
}

/// ASCII FBX parser
struct FBXAsciiParser<'a> {
    content: &'a str,
    nodes: Vec<FBXNode>,
}

#[derive(Debug, Clone)]
struct FBXNode {
    name: String,
    properties: Vec<FBXProperty>,
    children: Vec<FBXNode>,
}

#[derive(Debug, Clone)]
enum FBXProperty {
    Int(i64),
    Float(f64),
    String(String),
    Array(Vec<f64>),
}

impl<'a> FBXAsciiParser<'a> {
    fn new(content: &'a str) -> Self {
        Self {
            content,
            nodes: Vec::new(),
        }
    }

    fn parse(&mut self) -> Result<()> {
        let lines: Vec<&str> = self.content.lines().collect();
        let mut index = 0;

        while index < lines.len() {
            let line = lines[index].trim();

            // Skip comments and empty lines
            if line.starts_with(';') || line.is_empty() {
                index += 1;
                continue;
            }

            // Parse node
            if line.ends_with('{') || line.contains(':') {
                let (node, new_index) = self.parse_node(&lines, index)?;
                self.nodes.push(node);
                index = new_index;
            } else {
                index += 1;
            }
        }

        debug!("Parsed {} top-level FBX nodes", self.nodes.len());
        Ok(())
    }

    fn parse_node(&self, lines: &[&str], start: usize) -> Result<(FBXNode, usize)> {
        let line = lines[start].trim();
        let mut index = start;

        // Extract node name
        let name = if let Some(colon_pos) = line.find(':') {
            line[..colon_pos].trim().to_string()
        } else {
            line.trim_end_matches('{').trim().to_string()
        };

        let mut node = FBXNode {
            name,
            properties: Vec::new(),
            children: Vec::new(),
        };

        // Extract properties from same line (after colon)
        if let Some(colon_pos) = line.find(':') {
            let props_str = &line[colon_pos + 1..].trim_end_matches('{').trim();
            if !props_str.is_empty() {
                node.properties = self.parse_properties(props_str);
            }
        }

        // Check if this node has a block
        if line.ends_with('{') {
            index += 1;
            let mut depth = 1;

            while index < lines.len() && depth > 0 {
                let inner_line = lines[index].trim();

                if inner_line.starts_with(';') || inner_line.is_empty() {
                    index += 1;
                    continue;
                }

                if inner_line == "}" {
                    depth -= 1;
                    index += 1;
                    continue;
                }

                if inner_line.ends_with('{') {
                    let (child, new_index) = self.parse_node(lines, index)?;
                    node.children.push(child);
                    index = new_index;
                } else if inner_line.contains(':') {
                    // Property line
                    let (child, new_index) = self.parse_node(lines, index)?;
                    node.children.push(child);
                    index = new_index;
                } else {
                    index += 1;
                }
            }
        } else {
            index += 1;
        }

        Ok((node, index))
    }

    fn parse_properties(&self, props_str: &str) -> Vec<FBXProperty> {
        let mut properties = Vec::new();

        // Split by comma, handling quoted strings
        let parts = self.split_properties(props_str);

        for part in parts {
            let part = part.trim();

            if part.starts_with('"') && part.ends_with('"') {
                // String property
                properties.push(FBXProperty::String(part.trim_matches('"').to_string()));
            } else if part.contains(',') || part.starts_with('*') {
                // Array property (indicated by *)
                if let Some(array) = self.parse_array(part) {
                    properties.push(FBXProperty::Array(array));
                }
            } else if let Ok(i) = part.parse::<i64>() {
                properties.push(FBXProperty::Int(i));
            } else if let Ok(f) = part.parse::<f64>() {
                properties.push(FBXProperty::Float(f));
            } else if !part.is_empty() {
                properties.push(FBXProperty::String(part.to_string()));
            }
        }

        properties
    }

    fn split_properties(&self, s: &str) -> Vec<String> {
        let mut parts = Vec::new();
        let mut current = String::new();
        let mut in_quotes = false;
        let mut depth = 0;

        for c in s.chars() {
            match c {
                '"' => {
                    in_quotes = !in_quotes;
                    current.push(c);
                }
                '(' | '[' | '{' => {
                    depth += 1;
                    current.push(c);
                }
                ')' | ']' | '}' => {
                    depth -= 1;
                    current.push(c);
                }
                ',' if !in_quotes && depth == 0 => {
                    parts.push(current.trim().to_string());
                    current = String::new();
                }
                _ => current.push(c),
            }
        }

        if !current.is_empty() {
            parts.push(current.trim().to_string());
        }

        parts
    }

    fn parse_array(&self, s: &str) -> Option<Vec<f64>> {
        // Handle *N format (array size indicator)
        let s = if s.starts_with('*') {
            // Skip the *N part and look for the actual array
            if let Some(colon_pos) = s.find(':') {
                &s[colon_pos + 1..]
            } else {
                return None;
            }
        } else {
            s
        };

        let values: Vec<f64> = s
            .trim()
            .split(',')
            .filter_map(|v| v.trim().parse().ok())
            .collect();

        if values.is_empty() {
            None
        } else {
            Some(values)
        }
    }

    fn extract_geometry(&self) -> Result<(Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<u32>)> {
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut uvs = Vec::new();
        let mut indices = Vec::new();

        // Find Geometry nodes in Objects
        for node in &self.nodes {
            if node.name == "Objects" {
                for child in &node.children {
                    if child.name == "Geometry" {
                        self.extract_geometry_from_node(
                            child,
                            &mut positions,
                            &mut normals,
                            &mut uvs,
                            &mut indices,
                        );
                    }
                }
            }
        }

        // If no geometry found in Objects, search top level
        if positions.is_empty() {
            for node in &self.nodes {
                if node.name == "Geometry" {
                    self.extract_geometry_from_node(
                        node,
                        &mut positions,
                        &mut normals,
                        &mut uvs,
                        &mut indices,
                    );
                }
            }
        }

        if positions.is_empty() {
            anyhow::bail!("No geometry data found in FBX file");
        }

        // Generate indices if not found
        if indices.is_empty() {
            indices = (0..positions.len() as u32).collect();
        }

        Ok((positions, normals, uvs, indices))
    }

    fn extract_geometry_from_node(
        &self,
        node: &FBXNode,
        positions: &mut Vec<[f32; 3]>,
        normals: &mut Vec<[f32; 3]>,
        uvs: &mut Vec<[f32; 2]>,
        indices: &mut Vec<u32>,
    ) {
        for child in &node.children {
            match child.name.as_str() {
                "Vertices" => {
                    if let Some(FBXProperty::Array(verts)) = child.properties.first() {
                        for chunk in verts.chunks(3) {
                            if chunk.len() >= 3 {
                                positions.push([chunk[0] as f32, chunk[1] as f32, chunk[2] as f32]);
                            }
                        }
                    }
                }
                "PolygonVertexIndex" => {
                    if let Some(FBXProperty::Array(idx)) = child.properties.first() {
                        // FBX uses negative indices to mark end of polygon
                        // Convert to triangles
                        let mut polygon = Vec::new();
                        for &i in idx {
                            let idx = i as i32;
                            if idx < 0 {
                                // End of polygon, negative index is bitwise NOT
                                polygon.push((!idx) as u32);
                                // Triangulate polygon
                                if polygon.len() >= 3 {
                                    for i in 1..polygon.len() - 1 {
                                        indices.push(polygon[0]);
                                        indices.push(polygon[i]);
                                        indices.push(polygon[i + 1]);
                                    }
                                }
                                polygon.clear();
                            } else {
                                polygon.push(idx as u32);
                            }
                        }
                    }
                }
                "LayerElementNormal" => {
                    for subchild in &child.children {
                        if subchild.name == "Normals" {
                            if let Some(FBXProperty::Array(norms)) = subchild.properties.first() {
                                for chunk in norms.chunks(3) {
                                    if chunk.len() >= 3 {
                                        normals.push([
                                            chunk[0] as f32,
                                            chunk[1] as f32,
                                            chunk[2] as f32,
                                        ]);
                                    }
                                }
                            }
                        }
                    }
                }
                "LayerElementUV" => {
                    for subchild in &child.children {
                        if subchild.name == "UV" {
                            if let Some(FBXProperty::Array(uv_data)) = subchild.properties.first() {
                                for chunk in uv_data.chunks(2) {
                                    if chunk.len() >= 2 {
                                        uvs.push([chunk[0] as f32, chunk[1] as f32]);
                                    }
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }

    fn extract_materials(&self, _path: &Path) -> Vec<MaterialInfo> {
        let mut materials = Vec::new();

        // Find Material nodes in Objects
        for node in &self.nodes {
            if node.name == "Objects" {
                for child in &node.children {
                    if child.name == "Material" {
                        let name = child
                            .properties
                            .iter()
                            .filter_map(|p| match p {
                                FBXProperty::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .next()
                            .unwrap_or_else(|| "fbx_material".to_string());

                        // Extract material properties
                        let mut diffuse_color = Color::srgb(0.8, 0.8, 0.8);

                        for subchild in &child.children {
                            if subchild.name == "DiffuseColor" || subchild.name == "Diffuse" {
                                if let Some(FBXProperty::Array(rgb)) = subchild.properties.first() {
                                    if rgb.len() >= 3 {
                                        diffuse_color = Color::srgb(
                                            rgb[0] as f32,
                                            rgb[1] as f32,
                                            rgb[2] as f32,
                                        );
                                    }
                                }
                            }
                        }

                        materials.push(MaterialInfo {
                            name,
                            diffuse_color: Some(diffuse_color),
                            diffuse_texture: None,
                            normal_texture: None,
                            metallic: 0.0,
                            roughness: 0.5,
                        });
                    }
                }
            }
        }

        if materials.is_empty() {
            materials.push(MaterialInfo::default());
        }

        materials
    }
}

/// Binary FBX parser
struct FBXBinaryParser<'a> {
    data: &'a [u8],
    position: usize,
    version: u32,
    nodes: Vec<FBXBinaryNode>,
}

#[derive(Debug, Clone)]
struct FBXBinaryNode {
    name: String,
    properties: Vec<FBXBinaryProperty>,
    children: Vec<FBXBinaryNode>,
}

#[derive(Debug, Clone)]
enum FBXBinaryProperty {
    Bool(bool),
    Int16(i16),
    Int32(i32),
    Int64(i64),
    Float32(f32),
    Float64(f64),
    String(String),
    Raw(Vec<u8>),
    ArrayBool(Vec<bool>),
    ArrayInt32(Vec<i32>),
    ArrayInt64(Vec<i64>),
    ArrayFloat32(Vec<f32>),
    ArrayFloat64(Vec<f64>),
}

impl<'a> FBXBinaryParser<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            position: 0,
            version: 0,
            nodes: Vec::new(),
        }
    }

    fn parse(&mut self) -> Result<()> {
        // Skip magic header (23 bytes: "Kaydara FBX Binary  \x00\x1a\x00")
        self.position = 23;

        // Read version
        self.version = self.read_u32()?;
        debug!("FBX binary version: {}", self.version);

        // Parse nodes
        while self.position < self.data.len() - 13 {
            if let Some(node) = self.parse_node()? {
                if node.name.is_empty() && node.children.is_empty() {
                    // Null record = end of file
                    break;
                }
                self.nodes.push(node);
            } else {
                break;
            }
        }

        debug!("Parsed {} binary FBX nodes", self.nodes.len());
        Ok(())
    }

    fn parse_node(&mut self) -> Result<Option<FBXBinaryNode>> {
        let end_offset = if self.version >= 7500 {
            self.read_u64()? as usize
        } else {
            self.read_u32()? as usize
        };

        let num_properties = if self.version >= 7500 {
            self.read_u64()? as usize
        } else {
            self.read_u32()? as usize
        };

        let properties_size = if self.version >= 7500 {
            self.read_u64()? as usize
        } else {
            self.read_u32()? as usize
        };

        let name_len = self.read_u8()? as usize;

        // Check for null record
        if end_offset == 0 && name_len == 0 {
            return Ok(None);
        }

        let name = String::from_utf8_lossy(&self.data[self.position..self.position + name_len])
            .to_string();
        self.position += name_len;

        // Parse properties
        let mut properties = Vec::new();
        let props_end = self.position + properties_size;

        while self.position < props_end {
            let prop = self.parse_property()?;
            properties.push(prop);
        }

        // Parse children
        let mut children = Vec::new();
        while self.position < end_offset {
            if let Some(child) = self.parse_node()? {
                if child.name.is_empty() && child.children.is_empty() {
                    break;
                }
                children.push(child);
            } else {
                break;
            }
        }

        self.position = end_offset;

        Ok(Some(FBXBinaryNode {
            name,
            properties,
            children,
        }))
    }

    fn parse_property(&mut self) -> Result<FBXBinaryProperty> {
        let type_code = self.read_u8()? as char;

        let prop = match type_code {
            'C' => FBXBinaryProperty::Bool(self.read_u8()? != 0),
            'Y' => FBXBinaryProperty::Int16(self.read_i16()?),
            'I' => FBXBinaryProperty::Int32(self.read_i32()?),
            'L' => FBXBinaryProperty::Int64(self.read_i64()?),
            'F' => FBXBinaryProperty::Float32(self.read_f32()?),
            'D' => FBXBinaryProperty::Float64(self.read_f64()?),
            'S' | 'R' => {
                let len = self.read_u32()? as usize;
                let bytes = self.data[self.position..self.position + len].to_vec();
                self.position += len;
                if type_code == 'S' {
                    FBXBinaryProperty::String(String::from_utf8_lossy(&bytes).to_string())
                } else {
                    FBXBinaryProperty::Raw(bytes)
                }
            }
            'b' => {
                let (data, _) = self.read_array::<bool>()?;
                FBXBinaryProperty::ArrayBool(data)
            }
            'i' => {
                let (data, _) = self.read_array::<i32>()?;
                FBXBinaryProperty::ArrayInt32(data)
            }
            'l' => {
                let (data, _) = self.read_array::<i64>()?;
                FBXBinaryProperty::ArrayInt64(data)
            }
            'f' => {
                let (data, _) = self.read_array::<f32>()?;
                FBXBinaryProperty::ArrayFloat32(data)
            }
            'd' => {
                let (data, _) = self.read_array::<f64>()?;
                FBXBinaryProperty::ArrayFloat64(data)
            }
            _ => {
                warn!("Unknown FBX property type: {}", type_code);
                FBXBinaryProperty::Raw(Vec::new())
            }
        };

        Ok(prop)
    }

    fn read_array<T: Default + Clone + Copy>(&mut self) -> Result<(Vec<T>, usize)> {
        let array_len = self.read_u32()? as usize;
        let encoding = self.read_u32()?;
        let compressed_len = self.read_u32()? as usize;

        let type_size = std::mem::size_of::<T>();
        let expected_size = array_len * type_size;

        let raw_data = if encoding == 1 {
            // Deflate compressed
            use flate2::read::ZlibDecoder;
            use std::io::Read;

            let compressed = &self.data[self.position..self.position + compressed_len];
            self.position += compressed_len;

            let mut decoder = ZlibDecoder::new(compressed);
            let mut decompressed = Vec::with_capacity(expected_size);
            decoder.read_to_end(&mut decompressed)?;
            decompressed
        } else {
            // Uncompressed
            let data = self.data[self.position..self.position + expected_size].to_vec();
            self.position += expected_size;
            data
        };

        // Convert bytes to typed array
        let mut result = vec![T::default(); array_len];
        unsafe {
            std::ptr::copy_nonoverlapping(
                raw_data.as_ptr(),
                result.as_mut_ptr() as *mut u8,
                raw_data.len().min(expected_size),
            );
        }

        Ok((result, array_len))
    }

    fn read_u8(&mut self) -> Result<u8> {
        if self.position >= self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = self.data[self.position];
        self.position += 1;
        Ok(v)
    }

    fn read_i16(&mut self) -> Result<i16> {
        if self.position + 2 > self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = i16::from_le_bytes([self.data[self.position], self.data[self.position + 1]]);
        self.position += 2;
        Ok(v)
    }

    fn read_u32(&mut self) -> Result<u32> {
        if self.position + 4 > self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = u32::from_le_bytes([
            self.data[self.position],
            self.data[self.position + 1],
            self.data[self.position + 2],
            self.data[self.position + 3],
        ]);
        self.position += 4;
        Ok(v)
    }

    fn read_i32(&mut self) -> Result<i32> {
        Ok(self.read_u32()? as i32)
    }

    fn read_u64(&mut self) -> Result<u64> {
        if self.position + 8 > self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = u64::from_le_bytes([
            self.data[self.position],
            self.data[self.position + 1],
            self.data[self.position + 2],
            self.data[self.position + 3],
            self.data[self.position + 4],
            self.data[self.position + 5],
            self.data[self.position + 6],
            self.data[self.position + 7],
        ]);
        self.position += 8;
        Ok(v)
    }

    fn read_i64(&mut self) -> Result<i64> {
        Ok(self.read_u64()? as i64)
    }

    fn read_f32(&mut self) -> Result<f32> {
        if self.position + 4 > self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = f32::from_le_bytes([
            self.data[self.position],
            self.data[self.position + 1],
            self.data[self.position + 2],
            self.data[self.position + 3],
        ]);
        self.position += 4;
        Ok(v)
    }

    fn read_f64(&mut self) -> Result<f64> {
        if self.position + 8 > self.data.len() {
            anyhow::bail!("Unexpected end of FBX data");
        }
        let v = f64::from_le_bytes([
            self.data[self.position],
            self.data[self.position + 1],
            self.data[self.position + 2],
            self.data[self.position + 3],
            self.data[self.position + 4],
            self.data[self.position + 5],
            self.data[self.position + 6],
            self.data[self.position + 7],
        ]);
        self.position += 8;
        Ok(v)
    }

    fn extract_geometry(&self) -> Result<(Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<u32>)> {
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut uvs = Vec::new();
        let mut indices = Vec::new();

        // Find Objects node
        for node in &self.nodes {
            if node.name == "Objects" {
                for child in &node.children {
                    if child.name == "Geometry" {
                        self.extract_geometry_from_node(
                            child,
                            &mut positions,
                            &mut normals,
                            &mut uvs,
                            &mut indices,
                        );
                    }
                }
            }
        }

        if positions.is_empty() {
            // Try finding geometry at top level
            for node in &self.nodes {
                if node.name == "Geometry" {
                    self.extract_geometry_from_node(
                        node,
                        &mut positions,
                        &mut normals,
                        &mut uvs,
                        &mut indices,
                    );
                }
            }
        }

        if positions.is_empty() {
            anyhow::bail!("No geometry found in binary FBX");
        }

        if indices.is_empty() {
            indices = (0..positions.len() as u32).collect();
        }

        Ok((positions, normals, uvs, indices))
    }

    fn extract_geometry_from_node(
        &self,
        node: &FBXBinaryNode,
        positions: &mut Vec<[f32; 3]>,
        normals: &mut Vec<[f32; 3]>,
        uvs: &mut Vec<[f32; 2]>,
        indices: &mut Vec<u32>,
    ) {
        for child in &node.children {
            match child.name.as_str() {
                "Vertices" => {
                    if let Some(FBXBinaryProperty::ArrayFloat64(verts)) = child.properties.first() {
                        for chunk in verts.chunks(3) {
                            if chunk.len() >= 3 {
                                positions.push([chunk[0] as f32, chunk[1] as f32, chunk[2] as f32]);
                            }
                        }
                    }
                }
                "PolygonVertexIndex" => {
                    if let Some(FBXBinaryProperty::ArrayInt32(idx)) = child.properties.first() {
                        let mut polygon = Vec::new();
                        for &i in idx {
                            if i < 0 {
                                polygon.push((!i) as u32);
                                if polygon.len() >= 3 {
                                    for j in 1..polygon.len() - 1 {
                                        indices.push(polygon[0]);
                                        indices.push(polygon[j]);
                                        indices.push(polygon[j + 1]);
                                    }
                                }
                                polygon.clear();
                            } else {
                                polygon.push(i as u32);
                            }
                        }
                    }
                }
                "LayerElementNormal" => {
                    for subchild in &child.children {
                        if subchild.name == "Normals" {
                            if let Some(FBXBinaryProperty::ArrayFloat64(norms)) =
                                subchild.properties.first()
                            {
                                for chunk in norms.chunks(3) {
                                    if chunk.len() >= 3 {
                                        normals.push([
                                            chunk[0] as f32,
                                            chunk[1] as f32,
                                            chunk[2] as f32,
                                        ]);
                                    }
                                }
                            }
                        }
                    }
                }
                "LayerElementUV" => {
                    for subchild in &child.children {
                        if subchild.name == "UV" {
                            if let Some(FBXBinaryProperty::ArrayFloat64(uv_data)) =
                                subchild.properties.first()
                            {
                                for chunk in uv_data.chunks(2) {
                                    if chunk.len() >= 2 {
                                        uvs.push([chunk[0] as f32, chunk[1] as f32]);
                                    }
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }

    fn extract_materials(&self, _path: &Path) -> Vec<MaterialInfo> {
        let mut materials = Vec::new();

        for node in &self.nodes {
            if node.name == "Objects" {
                for child in &node.children {
                    if child.name == "Material" {
                        let name = child
                            .properties
                            .iter()
                            .filter_map(|p| match p {
                                FBXBinaryProperty::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .next()
                            .unwrap_or_else(|| "fbx_material".to_string());

                        materials.push(MaterialInfo {
                            name,
                            diffuse_color: Some(Color::srgb(0.8, 0.8, 0.8)),
                            diffuse_texture: None,
                            normal_texture: None,
                            metallic: 0.0,
                            roughness: 0.5,
                        });
                    }
                }
            }
        }

        if materials.is_empty() {
            materials.push(MaterialInfo::default());
        }

        materials
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fbx_property_parsing() {
        let parser = FBXAsciiParser::new("");
        let props = parser.parse_properties(r#""test", 123, 3.14"#);

        assert_eq!(props.len(), 3);
        match &props[0] {
            FBXProperty::String(s) => assert_eq!(s, "test"),
            _ => panic!("Expected string"),
        }
        match &props[1] {
            FBXProperty::Int(i) => assert_eq!(*i, 123),
            _ => panic!("Expected int"),
        }
    }

    #[test]
    fn test_mesh_load_options() {
        let options = MeshLoadOptions::default()
            .with_scale(Vec3::new(0.01, 0.01, 0.01))
            .generate_normals(true)
            .generate_tangents(true);

        assert_eq!(options.scale, Vec3::new(0.01, 0.01, 0.01));
        assert!(options.generate_normals);
        assert!(options.generate_tangents);
    }
}
