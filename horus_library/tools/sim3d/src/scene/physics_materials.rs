//! Physics Material Database
//!
//! Provides a standardized library of physical material properties for simulation:
//! - Friction coefficients (static/dynamic)
//! - Restitution (bounciness)
//! - Density
//! - Surface properties for contact dynamics
//!
//! Supports:
//! - Built-in material presets (rubber, steel, wood, etc.)
//! - Custom material definitions
//! - Material combinations for contact pairs
//! - Temperature/condition-dependent properties
//! - Import from various formats

use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

use crate::error::{EnhancedError, ErrorCategory, Result};

// ============================================================================
// Physics Material Definition
// ============================================================================

/// Physical material properties
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Material name
    pub name: String,
    /// Material category
    #[serde(default)]
    pub category: MaterialCategory,
    /// Static friction coefficient (Coulomb friction)
    #[serde(default = "default_static_friction")]
    pub static_friction: f32,
    /// Dynamic/kinetic friction coefficient
    #[serde(default = "default_dynamic_friction")]
    pub dynamic_friction: f32,
    /// Restitution (coefficient of restitution, 0-1)
    #[serde(default)]
    pub restitution: f32,
    /// Density in kg/m³
    #[serde(default = "default_density")]
    pub density: f32,
    /// Young's modulus (stiffness) in Pa
    #[serde(default)]
    pub youngs_modulus: Option<f64>,
    /// Poisson's ratio
    #[serde(default)]
    pub poissons_ratio: Option<f32>,
    /// Rolling friction coefficient
    #[serde(default)]
    pub rolling_friction: f32,
    /// Spinning friction coefficient
    #[serde(default)]
    pub spinning_friction: f32,
    /// Contact stiffness for soft contact model
    #[serde(default)]
    pub contact_stiffness: Option<f32>,
    /// Contact damping for soft contact model
    #[serde(default)]
    pub contact_damping: Option<f32>,
    /// Description
    #[serde(default)]
    pub description: String,
    /// Tags for searching/filtering
    #[serde(default)]
    pub tags: Vec<String>,
    /// Additional properties
    #[serde(default)]
    pub properties: HashMap<String, f64>,
}

fn default_static_friction() -> f32 {
    0.5
}

fn default_dynamic_friction() -> f32 {
    0.4
}

fn default_density() -> f32 {
    1000.0
}

impl PhysicsMaterial {
    /// Create a new material with default properties
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            category: MaterialCategory::Other,
            static_friction: 0.5,
            dynamic_friction: 0.4,
            restitution: 0.0,
            density: 1000.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.0,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: String::new(),
            tags: Vec::new(),
            properties: HashMap::new(),
        }
    }

    /// Set friction coefficients
    pub fn with_friction(mut self, static_f: f32, dynamic_f: f32) -> Self {
        self.static_friction = static_f;
        self.dynamic_friction = dynamic_f;
        self
    }

    /// Set restitution
    pub fn with_restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Set density
    pub fn with_density(mut self, density: f32) -> Self {
        self.density = density;
        self
    }

    /// Set category
    pub fn with_category(mut self, category: MaterialCategory) -> Self {
        self.category = category;
        self
    }

    /// Calculate combined friction with another material
    pub fn combined_friction(&self, other: &PhysicsMaterial, mode: FrictionCombineMode) -> f32 {
        match mode {
            FrictionCombineMode::Average => (self.dynamic_friction + other.dynamic_friction) / 2.0,
            FrictionCombineMode::Minimum => self.dynamic_friction.min(other.dynamic_friction),
            FrictionCombineMode::Maximum => self.dynamic_friction.max(other.dynamic_friction),
            FrictionCombineMode::Multiply => self.dynamic_friction * other.dynamic_friction,
        }
    }

    /// Calculate combined restitution with another material
    pub fn combined_restitution(
        &self,
        other: &PhysicsMaterial,
        mode: RestitutionCombineMode,
    ) -> f32 {
        match mode {
            RestitutionCombineMode::Average => (self.restitution + other.restitution) / 2.0,
            RestitutionCombineMode::Minimum => self.restitution.min(other.restitution),
            RestitutionCombineMode::Maximum => self.restitution.max(other.restitution),
            RestitutionCombineMode::Multiply => self.restitution * other.restitution,
        }
    }
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self::new("default")
    }
}

/// Material categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum MaterialCategory {
    Metal,
    Wood,
    Plastic,
    Rubber,
    Glass,
    Concrete,
    Stone,
    Ceramic,
    Fabric,
    Foam,
    Ice,
    Terrain,
    Organic,
    Composite,
    #[default]
    Other,
}

/// Friction combination mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FrictionCombineMode {
    #[default]
    Average,
    Minimum,
    Maximum,
    Multiply,
}

/// Restitution combination mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum RestitutionCombineMode {
    #[default]
    Average,
    Minimum,
    Maximum,
    Multiply,
}

// ============================================================================
// Material Contact Pair
// ============================================================================

/// Contact properties between two specific materials
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialContactPair {
    /// First material name
    pub material_a: String,
    /// Second material name
    pub material_b: String,
    /// Override friction (if None, use combine mode)
    pub friction: Option<f32>,
    /// Override restitution
    pub restitution: Option<f32>,
    /// Override contact stiffness
    pub contact_stiffness: Option<f32>,
    /// Override contact damping
    pub contact_damping: Option<f32>,
    /// Notes about this contact pair
    #[serde(default)]
    pub notes: String,
}

impl MaterialContactPair {
    /// Create a new contact pair
    pub fn new(material_a: &str, material_b: &str) -> Self {
        Self {
            material_a: material_a.to_string(),
            material_b: material_b.to_string(),
            friction: None,
            restitution: None,
            contact_stiffness: None,
            contact_damping: None,
            notes: String::new(),
        }
    }

    /// Check if this pair matches the given materials
    pub fn matches(&self, a: &str, b: &str) -> bool {
        (self.material_a == a && self.material_b == b)
            || (self.material_a == b && self.material_b == a)
    }
}

// ============================================================================
// Physics Material Database
// ============================================================================

/// Database of physics materials
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PhysicsMaterialDatabase {
    /// Materials by name
    #[serde(default)]
    materials: HashMap<String, PhysicsMaterial>,
    /// Contact pairs with override properties
    #[serde(default)]
    contact_pairs: Vec<MaterialContactPair>,
    /// Default friction combine mode
    #[serde(default)]
    pub friction_combine_mode: FrictionCombineMode,
    /// Default restitution combine mode
    #[serde(default)]
    pub restitution_combine_mode: RestitutionCombineMode,
}

impl PhysicsMaterialDatabase {
    /// Create a new empty database
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a database with built-in materials
    pub fn with_builtins() -> Self {
        let mut db = Self::new();
        db.add_builtin_materials();
        db.add_builtin_contact_pairs();
        db
    }

    /// Add a material to the database
    pub fn add(&mut self, material: PhysicsMaterial) {
        self.materials.insert(material.name.clone(), material);
    }

    /// Get a material by name
    pub fn get(&self, name: &str) -> Option<&PhysicsMaterial> {
        self.materials.get(name)
    }

    /// Get a material, falling back to default if not found
    pub fn get_or_default(&self, name: &str) -> PhysicsMaterial {
        self.materials.get(name).cloned().unwrap_or_default()
    }

    /// Remove a material
    pub fn remove(&mut self, name: &str) -> Option<PhysicsMaterial> {
        self.materials.remove(name)
    }

    /// Get all materials
    pub fn all(&self) -> impl Iterator<Item = &PhysicsMaterial> {
        self.materials.values()
    }

    /// Get materials by category
    pub fn by_category(
        &self,
        category: MaterialCategory,
    ) -> impl Iterator<Item = &PhysicsMaterial> {
        self.materials
            .values()
            .filter(move |m| m.category == category)
    }

    /// Get materials by tag
    pub fn by_tag<'a>(&'a self, tag: &'a str) -> impl Iterator<Item = &'a PhysicsMaterial> + 'a {
        self.materials
            .values()
            .filter(move |m| m.tags.iter().any(|t| t == tag))
    }

    /// Add a contact pair
    pub fn add_contact_pair(&mut self, pair: MaterialContactPair) {
        self.contact_pairs.push(pair);
    }

    /// Get contact pair for two materials
    pub fn get_contact_pair(
        &self,
        material_a: &str,
        material_b: &str,
    ) -> Option<&MaterialContactPair> {
        self.contact_pairs
            .iter()
            .find(|p| p.matches(material_a, material_b))
    }

    /// Calculate contact properties between two materials
    pub fn get_contact_properties(&self, material_a: &str, material_b: &str) -> ContactProperties {
        let mat_a = self.get_or_default(material_a);
        let mat_b = self.get_or_default(material_b);

        // Check for override contact pair
        if let Some(pair) = self.get_contact_pair(material_a, material_b) {
            ContactProperties {
                friction: pair
                    .friction
                    .unwrap_or_else(|| mat_a.combined_friction(&mat_b, self.friction_combine_mode)),
                restitution: pair.restitution.unwrap_or_else(|| {
                    mat_a.combined_restitution(&mat_b, self.restitution_combine_mode)
                }),
                contact_stiffness: pair.contact_stiffness,
                contact_damping: pair.contact_damping,
            }
        } else {
            ContactProperties {
                friction: mat_a.combined_friction(&mat_b, self.friction_combine_mode),
                restitution: mat_a.combined_restitution(&mat_b, self.restitution_combine_mode),
                contact_stiffness: mat_a.contact_stiffness.or(mat_b.contact_stiffness),
                contact_damping: mat_a.contact_damping.or(mat_b.contact_damping),
            }
        }
    }

    /// Number of materials
    pub fn len(&self) -> usize {
        self.materials.len()
    }

    /// Check if database is empty
    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }

    /// Load from YAML file
    pub fn load_yaml<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(path.as_ref()).map_err(|e| {
            EnhancedError::new(format!("Failed to read materials file: {}", e))
                .with_file(path.as_ref())
                .with_category(ErrorCategory::FileNotFound)
        })?;

        serde_yaml::from_str(&content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse materials file: {}", e))
                .with_file(path.as_ref())
                .with_category(ErrorCategory::ParseError)
        })
    }

    /// Save to YAML file
    pub fn save_yaml<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let content = serde_yaml::to_string(self)
            .map_err(|e| EnhancedError::new(format!("Failed to serialize materials: {}", e)))?;

        std::fs::write(path.as_ref(), content).map_err(|e| {
            EnhancedError::new(format!("Failed to write materials file: {}", e))
                .with_file(path.as_ref())
        })?;

        Ok(())
    }

    /// Load from JSON file
    pub fn load_json<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(path.as_ref()).map_err(|e| {
            EnhancedError::new(format!("Failed to read materials file: {}", e))
                .with_file(path.as_ref())
                .with_category(ErrorCategory::FileNotFound)
        })?;

        serde_json::from_str(&content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse materials file: {}", e))
                .with_file(path.as_ref())
                .with_category(ErrorCategory::ParseError)
        })
    }

    /// Merge another database into this one
    pub fn merge(&mut self, other: PhysicsMaterialDatabase) {
        for (name, material) in other.materials {
            self.materials.insert(name, material);
        }
        self.contact_pairs.extend(other.contact_pairs);
    }

    /// Add all built-in material presets
    fn add_builtin_materials(&mut self) {
        // ================== Metals ==================
        self.add(PhysicsMaterial {
            name: "steel".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 0.74,
            dynamic_friction: 0.57,
            restitution: 0.65,
            density: 7850.0,
            youngs_modulus: Some(200e9),
            poissons_ratio: Some(0.3),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Carbon steel".to_string(),
            tags: vec!["metal".to_string(), "common".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "stainless_steel".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 0.78,
            dynamic_friction: 0.63,
            restitution: 0.68,
            density: 8000.0,
            youngs_modulus: Some(193e9),
            poissons_ratio: Some(0.29),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Stainless steel (304)".to_string(),
            tags: vec!["metal".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "aluminum".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 0.61,
            dynamic_friction: 0.47,
            restitution: 0.70,
            density: 2700.0,
            youngs_modulus: Some(69e9),
            poissons_ratio: Some(0.33),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Aluminum alloy (6061)".to_string(),
            tags: vec!["metal".to_string(), "light".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "cast_iron".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 1.10,
            dynamic_friction: 0.15,
            restitution: 0.50,
            density: 7200.0,
            youngs_modulus: Some(165e9),
            poissons_ratio: Some(0.25),
            rolling_friction: 0.002,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Gray cast iron".to_string(),
            tags: vec!["metal".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "brass".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 0.51,
            dynamic_friction: 0.44,
            restitution: 0.60,
            density: 8500.0,
            youngs_modulus: Some(100e9),
            poissons_ratio: Some(0.34),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Brass alloy".to_string(),
            tags: vec!["metal".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "copper".to_string(),
            category: MaterialCategory::Metal,
            static_friction: 1.00,
            dynamic_friction: 0.80,
            restitution: 0.55,
            density: 8960.0,
            youngs_modulus: Some(117e9),
            poissons_ratio: Some(0.34),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Pure copper".to_string(),
            tags: vec!["metal".to_string()],
            properties: HashMap::new(),
        });

        // ================== Rubber ==================
        self.add(PhysicsMaterial {
            name: "rubber".to_string(),
            category: MaterialCategory::Rubber,
            static_friction: 1.0,
            dynamic_friction: 0.8,
            restitution: 0.75,
            density: 1100.0,
            youngs_modulus: Some(0.01e9),
            poissons_ratio: Some(0.49),
            rolling_friction: 0.01,
            spinning_friction: 0.01,
            contact_stiffness: Some(1e4),
            contact_damping: Some(100.0),
            description: "Natural rubber".to_string(),
            tags: vec!["rubber".to_string(), "elastic".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "tire_rubber".to_string(),
            category: MaterialCategory::Rubber,
            static_friction: 0.9,
            dynamic_friction: 0.7,
            restitution: 0.65,
            density: 1150.0,
            youngs_modulus: Some(0.01e9),
            poissons_ratio: Some(0.49),
            rolling_friction: 0.015,
            spinning_friction: 0.01,
            contact_stiffness: Some(2e4),
            contact_damping: Some(150.0),
            description: "Vehicle tire rubber".to_string(),
            tags: vec!["rubber".to_string(), "vehicle".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "silicone".to_string(),
            category: MaterialCategory::Rubber,
            static_friction: 0.75,
            dynamic_friction: 0.65,
            restitution: 0.80,
            density: 1100.0,
            youngs_modulus: Some(0.001e9),
            poissons_ratio: Some(0.48),
            rolling_friction: 0.02,
            spinning_friction: 0.02,
            contact_stiffness: Some(5e3),
            contact_damping: Some(50.0),
            description: "Silicone rubber".to_string(),
            tags: vec!["rubber".to_string(), "soft".to_string()],
            properties: HashMap::new(),
        });

        // ================== Wood ==================
        self.add(PhysicsMaterial {
            name: "wood_oak".to_string(),
            category: MaterialCategory::Wood,
            static_friction: 0.62,
            dynamic_friction: 0.48,
            restitution: 0.40,
            density: 750.0,
            youngs_modulus: Some(12e9),
            poissons_ratio: Some(0.37),
            rolling_friction: 0.005,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Oak hardwood".to_string(),
            tags: vec!["wood".to_string(), "hardwood".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "wood_pine".to_string(),
            category: MaterialCategory::Wood,
            static_friction: 0.50,
            dynamic_friction: 0.40,
            restitution: 0.35,
            density: 520.0,
            youngs_modulus: Some(9e9),
            poissons_ratio: Some(0.42),
            rolling_friction: 0.005,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Pine softwood".to_string(),
            tags: vec!["wood".to_string(), "softwood".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "plywood".to_string(),
            category: MaterialCategory::Wood,
            static_friction: 0.45,
            dynamic_friction: 0.38,
            restitution: 0.30,
            density: 600.0,
            youngs_modulus: Some(7e9),
            poissons_ratio: Some(0.35),
            rolling_friction: 0.004,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Plywood composite".to_string(),
            tags: vec!["wood".to_string(), "composite".to_string()],
            properties: HashMap::new(),
        });

        // ================== Plastic ==================
        self.add(PhysicsMaterial {
            name: "abs".to_string(),
            category: MaterialCategory::Plastic,
            static_friction: 0.50,
            dynamic_friction: 0.35,
            restitution: 0.50,
            density: 1050.0,
            youngs_modulus: Some(2.3e9),
            poissons_ratio: Some(0.35),
            rolling_friction: 0.003,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "ABS plastic".to_string(),
            tags: vec!["plastic".to_string(), "common".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "pla".to_string(),
            category: MaterialCategory::Plastic,
            static_friction: 0.40,
            dynamic_friction: 0.32,
            restitution: 0.45,
            density: 1250.0,
            youngs_modulus: Some(3.5e9),
            poissons_ratio: Some(0.36),
            rolling_friction: 0.003,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "PLA biodegradable plastic".to_string(),
            tags: vec!["plastic".to_string(), "3d_printing".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "nylon".to_string(),
            category: MaterialCategory::Plastic,
            static_friction: 0.35,
            dynamic_friction: 0.30,
            restitution: 0.55,
            density: 1150.0,
            youngs_modulus: Some(2.7e9),
            poissons_ratio: Some(0.40),
            rolling_friction: 0.002,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Nylon/Polyamide".to_string(),
            tags: vec!["plastic".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "polycarbonate".to_string(),
            category: MaterialCategory::Plastic,
            static_friction: 0.45,
            dynamic_friction: 0.38,
            restitution: 0.60,
            density: 1200.0,
            youngs_modulus: Some(2.4e9),
            poissons_ratio: Some(0.37),
            rolling_friction: 0.003,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Polycarbonate".to_string(),
            tags: vec!["plastic".to_string(), "transparent".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "teflon".to_string(),
            category: MaterialCategory::Plastic,
            static_friction: 0.04,
            dynamic_friction: 0.04,
            restitution: 0.45,
            density: 2200.0,
            youngs_modulus: Some(0.4e9),
            poissons_ratio: Some(0.46),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "PTFE/Teflon (very low friction)".to_string(),
            tags: vec!["plastic".to_string(), "low_friction".to_string()],
            properties: HashMap::new(),
        });

        // ================== Concrete & Stone ==================
        self.add(PhysicsMaterial {
            name: "concrete".to_string(),
            category: MaterialCategory::Concrete,
            static_friction: 0.70,
            dynamic_friction: 0.60,
            restitution: 0.20,
            density: 2400.0,
            youngs_modulus: Some(30e9),
            poissons_ratio: Some(0.20),
            rolling_friction: 0.01,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Standard concrete".to_string(),
            tags: vec!["concrete".to_string(), "construction".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "asphalt".to_string(),
            category: MaterialCategory::Concrete,
            static_friction: 0.85,
            dynamic_friction: 0.70,
            restitution: 0.25,
            density: 2350.0,
            youngs_modulus: Some(2e9),
            poissons_ratio: Some(0.35),
            rolling_friction: 0.015,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Road asphalt".to_string(),
            tags: vec!["road".to_string(), "construction".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "granite".to_string(),
            category: MaterialCategory::Stone,
            static_friction: 0.80,
            dynamic_friction: 0.60,
            restitution: 0.30,
            density: 2750.0,
            youngs_modulus: Some(50e9),
            poissons_ratio: Some(0.25),
            rolling_friction: 0.005,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Granite stone".to_string(),
            tags: vec!["stone".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "marble".to_string(),
            category: MaterialCategory::Stone,
            static_friction: 0.65,
            dynamic_friction: 0.50,
            restitution: 0.35,
            density: 2700.0,
            youngs_modulus: Some(55e9),
            poissons_ratio: Some(0.27),
            rolling_friction: 0.004,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Marble stone".to_string(),
            tags: vec!["stone".to_string()],
            properties: HashMap::new(),
        });

        // ================== Glass & Ceramic ==================
        self.add(PhysicsMaterial {
            name: "glass".to_string(),
            category: MaterialCategory::Glass,
            static_friction: 0.94,
            dynamic_friction: 0.40,
            restitution: 0.75,
            density: 2500.0,
            youngs_modulus: Some(70e9),
            poissons_ratio: Some(0.22),
            rolling_friction: 0.001,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Soda-lime glass".to_string(),
            tags: vec!["glass".to_string(), "brittle".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "ceramic".to_string(),
            category: MaterialCategory::Ceramic,
            static_friction: 0.50,
            dynamic_friction: 0.45,
            restitution: 0.60,
            density: 2600.0,
            youngs_modulus: Some(90e9),
            poissons_ratio: Some(0.22),
            rolling_friction: 0.002,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Technical ceramic".to_string(),
            tags: vec!["ceramic".to_string(), "brittle".to_string()],
            properties: HashMap::new(),
        });

        // ================== Terrain ==================
        self.add(PhysicsMaterial {
            name: "dry_sand".to_string(),
            category: MaterialCategory::Terrain,
            static_friction: 0.60,
            dynamic_friction: 0.45,
            restitution: 0.05,
            density: 1600.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.10,
            spinning_friction: 0.05,
            contact_stiffness: Some(1e5),
            contact_damping: Some(500.0),
            description: "Dry sand".to_string(),
            tags: vec!["terrain".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "wet_sand".to_string(),
            category: MaterialCategory::Terrain,
            static_friction: 0.50,
            dynamic_friction: 0.40,
            restitution: 0.03,
            density: 1900.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.08,
            spinning_friction: 0.04,
            contact_stiffness: Some(5e4),
            contact_damping: Some(800.0),
            description: "Wet sand".to_string(),
            tags: vec!["terrain".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "gravel".to_string(),
            category: MaterialCategory::Terrain,
            static_friction: 0.65,
            dynamic_friction: 0.55,
            restitution: 0.15,
            density: 1800.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.08,
            spinning_friction: 0.03,
            contact_stiffness: Some(2e5),
            contact_damping: Some(300.0),
            description: "Gravel".to_string(),
            tags: vec!["terrain".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "mud".to_string(),
            category: MaterialCategory::Terrain,
            static_friction: 0.40,
            dynamic_friction: 0.30,
            restitution: 0.02,
            density: 1700.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.15,
            spinning_friction: 0.08,
            contact_stiffness: Some(2e4),
            contact_damping: Some(1000.0),
            description: "Wet mud".to_string(),
            tags: vec!["terrain".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "grass".to_string(),
            category: MaterialCategory::Terrain,
            static_friction: 0.55,
            dynamic_friction: 0.45,
            restitution: 0.10,
            density: 1400.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.06,
            spinning_friction: 0.03,
            contact_stiffness: Some(8e4),
            contact_damping: Some(400.0),
            description: "Grass on soil".to_string(),
            tags: vec!["terrain".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        // ================== Ice & Snow ==================
        self.add(PhysicsMaterial {
            name: "ice".to_string(),
            category: MaterialCategory::Ice,
            static_friction: 0.05,
            dynamic_friction: 0.03,
            restitution: 0.40,
            density: 917.0,
            youngs_modulus: Some(9e9),
            poissons_ratio: Some(0.33),
            rolling_friction: 0.001,
            spinning_friction: 0.001,
            contact_stiffness: None,
            contact_damping: None,
            description: "Ice".to_string(),
            tags: vec!["ice".to_string(), "slippery".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "snow_packed".to_string(),
            category: MaterialCategory::Ice,
            static_friction: 0.30,
            dynamic_friction: 0.20,
            restitution: 0.10,
            density: 500.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.05,
            spinning_friction: 0.02,
            contact_stiffness: Some(5e4),
            contact_damping: Some(600.0),
            description: "Packed snow".to_string(),
            tags: vec!["snow".to_string(), "outdoor".to_string()],
            properties: HashMap::new(),
        });

        // ================== Fabric ==================
        self.add(PhysicsMaterial {
            name: "canvas".to_string(),
            category: MaterialCategory::Fabric,
            static_friction: 0.70,
            dynamic_friction: 0.55,
            restitution: 0.05,
            density: 300.0,
            youngs_modulus: None,
            poissons_ratio: None,
            rolling_friction: 0.02,
            spinning_friction: 0.01,
            contact_stiffness: Some(1e3),
            contact_damping: Some(50.0),
            description: "Canvas fabric".to_string(),
            tags: vec!["fabric".to_string()],
            properties: HashMap::new(),
        });

        // ================== Foam ==================
        self.add(PhysicsMaterial {
            name: "foam_soft".to_string(),
            category: MaterialCategory::Foam,
            static_friction: 0.80,
            dynamic_friction: 0.70,
            restitution: 0.50,
            density: 50.0,
            youngs_modulus: Some(0.0001e9),
            poissons_ratio: Some(0.40),
            rolling_friction: 0.03,
            spinning_friction: 0.02,
            contact_stiffness: Some(1e3),
            contact_damping: Some(20.0),
            description: "Soft foam".to_string(),
            tags: vec!["foam".to_string(), "soft".to_string()],
            properties: HashMap::new(),
        });

        self.add(PhysicsMaterial {
            name: "foam_rigid".to_string(),
            category: MaterialCategory::Foam,
            static_friction: 0.60,
            dynamic_friction: 0.50,
            restitution: 0.40,
            density: 200.0,
            youngs_modulus: Some(0.01e9),
            poissons_ratio: Some(0.35),
            rolling_friction: 0.02,
            spinning_friction: 0.01,
            contact_stiffness: Some(5e3),
            contact_damping: Some(50.0),
            description: "Rigid foam".to_string(),
            tags: vec!["foam".to_string()],
            properties: HashMap::new(),
        });

        // ================== Carbon Fiber ==================
        self.add(PhysicsMaterial {
            name: "carbon_fiber".to_string(),
            category: MaterialCategory::Composite,
            static_friction: 0.45,
            dynamic_friction: 0.35,
            restitution: 0.55,
            density: 1550.0,
            youngs_modulus: Some(70e9),
            poissons_ratio: Some(0.30),
            rolling_friction: 0.002,
            spinning_friction: 0.0,
            contact_stiffness: None,
            contact_damping: None,
            description: "Carbon fiber composite".to_string(),
            tags: vec!["composite".to_string(), "light".to_string()],
            properties: HashMap::new(),
        });

        info!("Added {} built-in materials", self.len());
    }

    /// Add built-in contact pairs
    fn add_builtin_contact_pairs(&mut self) {
        // Tire on various surfaces
        self.add_contact_pair(MaterialContactPair {
            material_a: "tire_rubber".to_string(),
            material_b: "asphalt".to_string(),
            friction: Some(0.9),
            restitution: Some(0.2),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Tire on dry asphalt".to_string(),
        });

        self.add_contact_pair(MaterialContactPair {
            material_a: "tire_rubber".to_string(),
            material_b: "concrete".to_string(),
            friction: Some(0.85),
            restitution: Some(0.2),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Tire on dry concrete".to_string(),
        });

        self.add_contact_pair(MaterialContactPair {
            material_a: "tire_rubber".to_string(),
            material_b: "ice".to_string(),
            friction: Some(0.15),
            restitution: Some(0.25),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Tire on ice (dangerous!)".to_string(),
        });

        self.add_contact_pair(MaterialContactPair {
            material_a: "tire_rubber".to_string(),
            material_b: "grass".to_string(),
            friction: Some(0.60),
            restitution: Some(0.15),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Tire on grass".to_string(),
        });

        self.add_contact_pair(MaterialContactPair {
            material_a: "tire_rubber".to_string(),
            material_b: "gravel".to_string(),
            friction: Some(0.70),
            restitution: Some(0.10),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Tire on gravel".to_string(),
        });

        // Steel on steel (lubricated vs dry)
        self.add_contact_pair(MaterialContactPair {
            material_a: "steel".to_string(),
            material_b: "steel".to_string(),
            friction: Some(0.40),
            restitution: Some(0.65),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Steel on steel (dry)".to_string(),
        });

        // Teflon (very low friction)
        self.add_contact_pair(MaterialContactPair {
            material_a: "teflon".to_string(),
            material_b: "steel".to_string(),
            friction: Some(0.04),
            restitution: Some(0.50),
            contact_stiffness: None,
            contact_damping: None,
            notes: "Teflon on steel (very low friction)".to_string(),
        });
    }
}

/// Calculated contact properties
#[derive(Debug, Clone, Copy)]
pub struct ContactProperties {
    pub friction: f32,
    pub restitution: f32,
    pub contact_stiffness: Option<f32>,
    pub contact_damping: Option<f32>,
}

// ============================================================================
// Bevy Plugin
// ============================================================================

/// Plugin for physics materials
pub struct PhysicsMaterialsPlugin;

impl Plugin for PhysicsMaterialsPlugin {
    fn build(&self, app: &mut App) {
        let database = PhysicsMaterialDatabase::with_builtins();
        app.insert_resource(PhysicsMaterialDatabaseResource { database });
    }
}

/// Resource wrapper for material database
#[derive(Resource)]
pub struct PhysicsMaterialDatabaseResource {
    pub database: PhysicsMaterialDatabase,
}

// ============================================================================
// Bevy Component
// ============================================================================

/// Component for entity material assignment
#[derive(Component, Debug, Clone)]
pub struct MaterialAssignment {
    pub material_name: String,
}

impl MaterialAssignment {
    pub fn new(name: &str) -> Self {
        Self {
            material_name: name.to_string(),
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builtin_materials() {
        let db = PhysicsMaterialDatabase::with_builtins();
        assert!(db.len() > 20);

        // Check some specific materials
        assert!(db.get("steel").is_some());
        assert!(db.get("rubber").is_some());
        assert!(db.get("concrete").is_some());
        assert!(db.get("ice").is_some());
    }

    #[test]
    fn test_material_combination() {
        let db = PhysicsMaterialDatabase::with_builtins();

        let steel = db.get("steel").unwrap();
        let rubber = db.get("rubber").unwrap();

        let combined = steel.combined_friction(rubber, FrictionCombineMode::Average);
        assert!(combined > 0.0);
        assert!(combined < 1.5);
    }

    #[test]
    fn test_contact_pair() {
        let db = PhysicsMaterialDatabase::with_builtins();

        let props = db.get_contact_properties("tire_rubber", "asphalt");
        assert!((props.friction - 0.9).abs() < 0.01);
    }

    #[test]
    fn test_yaml_serialization() {
        let db = PhysicsMaterialDatabase::with_builtins();
        let yaml = serde_yaml::to_string(&db).unwrap();
        assert!(yaml.contains("steel"));
        assert!(yaml.contains("rubber"));

        // Deserialize and check
        let loaded: PhysicsMaterialDatabase = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(loaded.len(), db.len());
    }

    #[test]
    fn test_material_categories() {
        let db = PhysicsMaterialDatabase::with_builtins();

        let metals: Vec<_> = db.by_category(MaterialCategory::Metal).collect();
        assert!(metals.len() >= 5);

        let terrain: Vec<_> = db.by_category(MaterialCategory::Terrain).collect();
        assert!(terrain.len() >= 4);
    }
}
