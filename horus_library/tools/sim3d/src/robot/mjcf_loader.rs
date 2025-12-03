//! MJCF (MuJoCo XML Format) loader
//!
//! MJCF is the native format for the MuJoCo physics simulator, widely used in
//! reinforcement learning and robotics research. Many popular robot models
//! (Unitree, ANYmal, Franka, etc.) are available in MJCF format.
//!
//! This module provides:
//! - Full MJCF parsing (bodies, joints, geoms, actuators, sensors)
//! - Conversion to Bevy/Rapier physics entities
//! - Support for MuJoCo Menagerie models

use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy::render::mesh::Mesh;
use rapier3d::prelude::*;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use tracing::{debug, info, warn};

use crate::assets::mesh::{MeshLoadOptions, MeshLoader};
use crate::assets::resolver::PathResolver;
use crate::physics::collider::{ColliderBuilder, ColliderShape};
use crate::physics::joints::{
    create_prismatic_joint_with_limits, create_revolute_joint_with_limits, create_spherical_joint,
    JointType,
};
use crate::physics::rigid_body::RigidBodyComponent;
use crate::physics::world::PhysicsWorld;
use crate::robot::robot::Robot;

/// MJCF model loader
pub struct MJCFLoader {
    base_path: PathBuf,
    mesh_loader: MeshLoader,
    path_resolver: PathResolver,
    /// Default class settings (from <default>)
    defaults: MJCFDefaults,
}

/// Parsed MJCF model
#[derive(Debug, Clone)]
pub struct MJCFModel {
    pub name: String,
    pub compiler: MJCFCompiler,
    pub option: MJCFOption,
    pub defaults: MJCFDefaults,
    pub assets: MJCFAssets,
    pub worldbody: MJCFBody,
    pub actuators: Vec<MJCFActuator>,
    pub sensors: Vec<MJCFSensor>,
    pub tendons: Vec<MJCFTendon>,
    pub equalities: Vec<MJCFEquality>,
}

impl Default for MJCFModel {
    fn default() -> Self {
        Self {
            name: "unnamed".to_string(),
            compiler: MJCFCompiler::default(),
            option: MJCFOption::default(),
            defaults: MJCFDefaults::default(),
            assets: MJCFAssets::default(),
            worldbody: MJCFBody::default(),
            actuators: Vec::new(),
            sensors: Vec::new(),
            tendons: Vec::new(),
            equalities: Vec::new(),
        }
    }
}

/// MJCF compiler settings
#[derive(Debug, Clone)]
pub struct MJCFCompiler {
    pub coordinate: CoordinateSystem,
    pub angle: AngleUnit,
    pub meshdir: Option<String>,
    pub texturedir: Option<String>,
    pub autolimits: bool,
    pub eulerseq: String,
}

impl Default for MJCFCompiler {
    fn default() -> Self {
        Self {
            coordinate: CoordinateSystem::Local,
            angle: AngleUnit::Degree,
            meshdir: None,
            texturedir: None,
            autolimits: true,
            eulerseq: "xyz".to_string(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoordinateSystem {
    Local,
    Global,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AngleUnit {
    Degree,
    Radian,
}

/// MJCF simulation options
#[derive(Debug, Clone)]
pub struct MJCFOption {
    pub timestep: f32,
    pub gravity: Vec3,
    pub integrator: String,
    pub collision: String,
    pub cone: String,
    pub jacobian: String,
    pub solver: String,
    pub iterations: u32,
    pub tolerance: f32,
}

impl Default for MJCFOption {
    fn default() -> Self {
        Self {
            timestep: 0.002,
            gravity: Vec3::new(0.0, 0.0, -9.81),
            integrator: "Euler".to_string(),
            collision: "all".to_string(),
            cone: "pyramidal".to_string(),
            jacobian: "dense".to_string(),
            solver: "Newton".to_string(),
            iterations: 100,
            tolerance: 1e-8,
        }
    }
}

/// MJCF default class settings
#[derive(Debug, Clone, Default)]
pub struct MJCFDefaults {
    pub geom: MJCFGeomDefaults,
    pub joint: MJCFJointDefaults,
    pub motor: MJCFMotorDefaults,
    pub classes: HashMap<String, MJCFClassDefaults>,
}

#[derive(Debug, Clone)]
pub struct MJCFGeomDefaults {
    pub contype: u32,
    pub conaffinity: u32,
    pub condim: u32,
    pub friction: [f32; 3],
    pub rgba: [f32; 4],
    pub density: f32,
    pub margin: f32,
}

impl Default for MJCFGeomDefaults {
    fn default() -> Self {
        Self {
            contype: 1,
            conaffinity: 1,
            condim: 3,
            friction: [1.0, 0.005, 0.0001],
            rgba: [0.5, 0.5, 0.5, 1.0],
            density: 1000.0,
            margin: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MJCFJointDefaults {
    pub armature: f32,
    pub damping: f32,
    pub limited: bool,
    pub stiffness: f32,
    pub frictionloss: f32,
}

impl Default for MJCFJointDefaults {
    fn default() -> Self {
        Self {
            armature: 0.0,
            damping: 0.0,
            limited: false,
            stiffness: 0.0,
            frictionloss: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MJCFMotorDefaults {
    pub ctrlrange: [f32; 2],
    pub ctrllimited: bool,
    pub forcerange: [f32; 2],
    pub forcelimited: bool,
    pub gear: f32,
}

impl Default for MJCFMotorDefaults {
    fn default() -> Self {
        Self {
            ctrlrange: [-1.0, 1.0],
            ctrllimited: true,
            forcerange: [-1.0, 1.0],
            forcelimited: false,
            gear: 1.0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MJCFClassDefaults {
    pub geom: Option<MJCFGeomDefaults>,
    pub joint: Option<MJCFJointDefaults>,
    pub motor: Option<MJCFMotorDefaults>,
}

/// MJCF assets (meshes, textures, materials)
#[derive(Debug, Clone, Default)]
pub struct MJCFAssets {
    pub meshes: HashMap<String, MJCFMesh>,
    pub textures: HashMap<String, MJCFTexture>,
    pub materials: HashMap<String, MJCFMaterial>,
}

#[derive(Debug, Clone)]
pub struct MJCFMesh {
    pub name: String,
    pub file: String,
    pub scale: Vec3,
}

#[derive(Debug, Clone)]
pub struct MJCFTexture {
    pub name: String,
    pub texture_type: String,
    pub file: Option<String>,
    pub builtin: Option<String>,
    pub rgb1: Option<[f32; 3]>,
    pub rgb2: Option<[f32; 3]>,
}

#[derive(Debug, Clone)]
pub struct MJCFMaterial {
    pub name: String,
    pub texture: Option<String>,
    pub rgba: [f32; 4],
    pub specular: f32,
    pub shininess: f32,
    pub reflectance: f32,
}

/// MJCF body (recursive structure)
#[derive(Debug, Clone)]
pub struct MJCFBody {
    pub name: String,
    pub pos: Vec3,
    pub quat: Quat,
    pub euler: Option<Vec3>,
    pub inertial: Option<MJCFInertial>,
    pub joints: Vec<MJCFJoint>,
    pub geoms: Vec<MJCFGeom>,
    pub sites: Vec<MJCFSite>,
    pub children: Vec<MJCFBody>,
    pub mocap: bool,
}

impl Default for MJCFBody {
    fn default() -> Self {
        Self {
            name: "world".to_string(),
            pos: Vec3::ZERO,
            quat: Quat::IDENTITY,
            euler: None,
            inertial: None,
            joints: Vec::new(),
            geoms: Vec::new(),
            sites: Vec::new(),
            children: Vec::new(),
            mocap: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MJCFInertial {
    pub pos: Vec3,
    pub mass: f32,
    pub diaginertia: Option<Vec3>,
    pub fullinertia: Option<[f32; 6]>,
}

#[derive(Debug, Clone)]
pub struct MJCFJoint {
    pub name: String,
    pub joint_type: MJCFJointType,
    pub pos: Vec3,
    pub axis: Vec3,
    pub range: Option<[f32; 2]>,
    pub limited: bool,
    pub armature: f32,
    pub damping: f32,
    pub stiffness: f32,
    pub ref_pos: f32,
    pub class: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MJCFJointType {
    Free,
    Ball,
    Slide,
    Hinge,
}

impl Default for MJCFJoint {
    fn default() -> Self {
        Self {
            name: String::new(),
            joint_type: MJCFJointType::Hinge,
            pos: Vec3::ZERO,
            axis: Vec3::Z,
            range: None,
            limited: false,
            armature: 0.0,
            damping: 0.0,
            stiffness: 0.0,
            ref_pos: 0.0,
            class: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MJCFGeom {
    pub name: Option<String>,
    pub geom_type: MJCFGeomType,
    pub pos: Vec3,
    pub quat: Quat,
    pub euler: Option<Vec3>,
    pub size: Vec<f32>,
    pub rgba: [f32; 4],
    pub mesh: Option<String>,
    pub material: Option<String>,
    pub contype: u32,
    pub conaffinity: u32,
    pub density: f32,
    pub friction: [f32; 3],
    pub class: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MJCFGeomType {
    Plane,
    Sphere,
    Capsule,
    Ellipsoid,
    Cylinder,
    Box,
    Mesh,
}

impl Default for MJCFGeom {
    fn default() -> Self {
        Self {
            name: None,
            geom_type: MJCFGeomType::Sphere,
            pos: Vec3::ZERO,
            quat: Quat::IDENTITY,
            euler: None,
            size: vec![0.05],
            rgba: [0.5, 0.5, 0.5, 1.0],
            mesh: None,
            material: None,
            contype: 1,
            conaffinity: 1,
            density: 1000.0,
            friction: [1.0, 0.005, 0.0001],
            class: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MJCFSite {
    pub name: String,
    pub pos: Vec3,
    pub quat: Quat,
    pub size: Vec3,
    pub rgba: [f32; 4],
}

/// MJCF actuator
#[derive(Debug, Clone)]
pub struct MJCFActuator {
    pub name: String,
    pub actuator_type: MJCFActuatorType,
    pub joint: Option<String>,
    pub site: Option<String>,
    pub tendon: Option<String>,
    pub gear: f32,
    pub ctrlrange: [f32; 2],
    pub ctrllimited: bool,
    pub forcerange: [f32; 2],
    pub forcelimited: bool,
    pub kp: f32,
    pub kv: f32,
    pub class: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MJCFActuatorType {
    Motor,
    Position,
    Velocity,
    General,
}

impl Default for MJCFActuator {
    fn default() -> Self {
        Self {
            name: String::new(),
            actuator_type: MJCFActuatorType::Motor,
            joint: None,
            site: None,
            tendon: None,
            gear: 1.0,
            ctrlrange: [-1.0, 1.0],
            ctrllimited: true,
            forcerange: [-1.0, 1.0],
            forcelimited: false,
            kp: 0.0,
            kv: 0.0,
            class: None,
        }
    }
}

/// MJCF sensor
#[derive(Debug, Clone)]
pub struct MJCFSensor {
    pub name: String,
    pub sensor_type: String,
    pub site: Option<String>,
    pub joint: Option<String>,
    pub body: Option<String>,
    pub objtype: Option<String>,
    pub objname: Option<String>,
    pub noise: f32,
    pub cutoff: f32,
}

/// MJCF tendon
#[derive(Debug, Clone)]
pub struct MJCFTendon {
    pub name: String,
    pub tendon_type: String,
    pub stiffness: f32,
    pub damping: f32,
    pub limited: bool,
    pub range: [f32; 2],
}

/// MJCF equality constraint
#[derive(Debug, Clone)]
pub struct MJCFEquality {
    pub name: Option<String>,
    pub equality_type: String,
    pub body1: Option<String>,
    pub body2: Option<String>,
    pub joint1: Option<String>,
    pub joint2: Option<String>,
    pub polycoef: Vec<f32>,
}

impl MJCFLoader {
    /// Create a new MJCF loader
    pub fn new() -> Self {
        let mut mesh_loader = MeshLoader::new();
        let mut path_resolver = PathResolver::new();

        mesh_loader.add_base_path(PathBuf::from("assets/models"));
        mesh_loader.add_base_path(PathBuf::from("assets/robots"));
        path_resolver.add_base_path(PathBuf::from("assets/models"));
        path_resolver.add_base_path(PathBuf::from("assets/robots"));

        Self {
            base_path: PathBuf::from("."),
            mesh_loader,
            path_resolver,
            defaults: MJCFDefaults::default(),
        }
    }

    /// Set base path for resolving assets
    pub fn with_base_path(mut self, path: impl Into<PathBuf>) -> Self {
        let path_buf = path.into();
        self.base_path = path_buf.clone();
        self.mesh_loader.add_base_path(path_buf.clone());
        self.path_resolver.add_base_path(path_buf);
        self
    }

    /// Parse MJCF file
    pub fn parse_file(&self, path: impl AsRef<Path>) -> Result<MJCFModel> {
        let path = path.as_ref();
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read MJCF file: {}", path.display()))?;

        self.parse_string(&content, path.parent())
    }

    /// Parse MJCF from string
    pub fn parse_string(&self, xml: &str, base_dir: Option<&Path>) -> Result<MJCFModel> {
        let doc = roxmltree::Document::parse(xml).with_context(|| "Failed to parse MJCF XML")?;

        let root = doc.root_element();
        if !root.has_tag_name("mujoco") {
            anyhow::bail!("Root element must be <mujoco>");
        }

        let mut model = MJCFModel::default();

        // Parse model name
        if let Some(name) = root.attribute("model") {
            model.name = name.to_string();
        }

        // Parse all sections
        for child in root.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "compiler" => model.compiler = self.parse_compiler(child)?,
                "option" => model.option = self.parse_option(child)?,
                "default" => model.defaults = self.parse_defaults(child)?,
                "asset" => model.assets = self.parse_assets(child, base_dir)?,
                "worldbody" => model.worldbody = self.parse_body(child, &model.compiler)?,
                "actuator" => model.actuators = self.parse_actuators(child)?,
                "sensor" => model.sensors = self.parse_sensors(child)?,
                "tendon" => model.tendons = self.parse_tendons(child)?,
                "equality" => model.equalities = self.parse_equalities(child)?,
                "include" => {
                    // Handle includes
                    if let Some(file) = child.attribute("file") {
                        if let Some(base) = base_dir {
                            let include_path = base.join(file);
                            if include_path.exists() {
                                let included = self.parse_file(&include_path)?;
                                // Merge included model (simplified - just merge assets and bodies)
                                for (k, v) in included.assets.meshes {
                                    model.assets.meshes.insert(k, v);
                                }
                                for (k, v) in included.assets.materials {
                                    model.assets.materials.insert(k, v);
                                }
                                model.worldbody.children.extend(included.worldbody.children);
                            }
                        }
                    }
                }
                _ => debug!("Ignoring unknown MJCF element: {}", child.tag_name().name()),
            }
        }

        Ok(model)
    }

    fn parse_compiler(&self, elem: roxmltree::Node) -> Result<MJCFCompiler> {
        let mut compiler = MJCFCompiler::default();

        if let Some(coord) = elem.attribute("coordinate") {
            compiler.coordinate = match coord {
                "global" => CoordinateSystem::Global,
                _ => CoordinateSystem::Local,
            };
        }

        if let Some(angle) = elem.attribute("angle") {
            compiler.angle = match angle {
                "radian" => AngleUnit::Radian,
                _ => AngleUnit::Degree,
            };
        }

        if let Some(meshdir) = elem.attribute("meshdir") {
            compiler.meshdir = Some(meshdir.to_string());
        }

        if let Some(texturedir) = elem.attribute("texturedir") {
            compiler.texturedir = Some(texturedir.to_string());
        }

        if let Some(autolimits) = elem.attribute("autolimits") {
            compiler.autolimits = autolimits == "true";
        }

        if let Some(eulerseq) = elem.attribute("eulerseq") {
            compiler.eulerseq = eulerseq.to_string();
        }

        Ok(compiler)
    }

    fn parse_option(&self, elem: roxmltree::Node) -> Result<MJCFOption> {
        let mut opt = MJCFOption::default();

        if let Some(ts) = elem.attribute("timestep") {
            opt.timestep = ts.parse().unwrap_or(0.002);
        }

        if let Some(grav) = elem.attribute("gravity") {
            let parts: Vec<f32> = grav
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                opt.gravity = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        if let Some(integrator) = elem.attribute("integrator") {
            opt.integrator = integrator.to_string();
        }

        if let Some(iterations) = elem.attribute("iterations") {
            opt.iterations = iterations.parse().unwrap_or(100);
        }

        Ok(opt)
    }

    fn parse_defaults(&self, elem: roxmltree::Node) -> Result<MJCFDefaults> {
        let mut defaults = MJCFDefaults::default();

        for child in elem.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "geom" => defaults.geom = self.parse_geom_defaults(child)?,
                "joint" => defaults.joint = self.parse_joint_defaults(child)?,
                "motor" => defaults.motor = self.parse_motor_defaults(child)?,
                "class" => {
                    if let Some(name) = child.attribute("name") {
                        let class_defaults = self.parse_class_defaults(child)?;
                        defaults.classes.insert(name.to_string(), class_defaults);
                    }
                }
                _ => {}
            }
        }

        Ok(defaults)
    }

    fn parse_geom_defaults(&self, elem: roxmltree::Node) -> Result<MJCFGeomDefaults> {
        let mut geom = MJCFGeomDefaults::default();

        if let Some(contype) = elem.attribute("contype") {
            geom.contype = contype.parse().unwrap_or(1);
        }

        if let Some(conaffinity) = elem.attribute("conaffinity") {
            geom.conaffinity = conaffinity.parse().unwrap_or(1);
        }

        if let Some(friction) = elem.attribute("friction") {
            let parts: Vec<f32> = friction
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                geom.friction = [parts[0], parts[1], parts[2]];
            }
        }

        if let Some(rgba) = elem.attribute("rgba") {
            let parts: Vec<f32> = rgba
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                geom.rgba = [parts[0], parts[1], parts[2], parts[3]];
            }
        }

        if let Some(density) = elem.attribute("density") {
            geom.density = density.parse().unwrap_or(1000.0);
        }

        Ok(geom)
    }

    fn parse_joint_defaults(&self, elem: roxmltree::Node) -> Result<MJCFJointDefaults> {
        let mut joint = MJCFJointDefaults::default();

        if let Some(armature) = elem.attribute("armature") {
            joint.armature = armature.parse().unwrap_or(0.0);
        }

        if let Some(damping) = elem.attribute("damping") {
            joint.damping = damping.parse().unwrap_or(0.0);
        }

        if let Some(limited) = elem.attribute("limited") {
            joint.limited = limited == "true";
        }

        if let Some(stiffness) = elem.attribute("stiffness") {
            joint.stiffness = stiffness.parse().unwrap_or(0.0);
        }

        Ok(joint)
    }

    fn parse_motor_defaults(&self, elem: roxmltree::Node) -> Result<MJCFMotorDefaults> {
        let mut motor = MJCFMotorDefaults::default();

        if let Some(ctrlrange) = elem.attribute("ctrlrange") {
            let parts: Vec<f32> = ctrlrange
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 2 {
                motor.ctrlrange = [parts[0], parts[1]];
            }
        }

        if let Some(ctrllimited) = elem.attribute("ctrllimited") {
            motor.ctrllimited = ctrllimited == "true";
        }

        if let Some(gear) = elem.attribute("gear") {
            motor.gear = gear.parse().unwrap_or(1.0);
        }

        Ok(motor)
    }

    fn parse_class_defaults(&self, elem: roxmltree::Node) -> Result<MJCFClassDefaults> {
        let mut class = MJCFClassDefaults::default();

        for child in elem.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "geom" => class.geom = Some(self.parse_geom_defaults(child)?),
                "joint" => class.joint = Some(self.parse_joint_defaults(child)?),
                "motor" => class.motor = Some(self.parse_motor_defaults(child)?),
                _ => {}
            }
        }

        Ok(class)
    }

    fn parse_assets(&self, elem: roxmltree::Node, base_dir: Option<&Path>) -> Result<MJCFAssets> {
        let mut assets = MJCFAssets::default();

        for child in elem.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "mesh" => {
                    if let Some(name) = child.attribute("name") {
                        let file = child.attribute("file").unwrap_or("").to_string();
                        let scale = if let Some(s) = child.attribute("scale") {
                            let parts: Vec<f32> = s
                                .split_whitespace()
                                .filter_map(|x| x.parse().ok())
                                .collect();
                            if parts.len() >= 3 {
                                Vec3::new(parts[0], parts[1], parts[2])
                            } else if parts.len() == 1 {
                                Vec3::splat(parts[0])
                            } else {
                                Vec3::ONE
                            }
                        } else {
                            Vec3::ONE
                        };

                        assets.meshes.insert(
                            name.to_string(),
                            MJCFMesh {
                                name: name.to_string(),
                                file,
                                scale,
                            },
                        );
                    }
                }
                "texture" => {
                    if let Some(name) = child.attribute("name") {
                        let texture = MJCFTexture {
                            name: name.to_string(),
                            texture_type: child.attribute("type").unwrap_or("2d").to_string(),
                            file: child.attribute("file").map(|s| s.to_string()),
                            builtin: child.attribute("builtin").map(|s| s.to_string()),
                            rgb1: child.attribute("rgb1").and_then(|s| {
                                let parts: Vec<f32> = s
                                    .split_whitespace()
                                    .filter_map(|x| x.parse().ok())
                                    .collect();
                                if parts.len() >= 3 {
                                    Some([parts[0], parts[1], parts[2]])
                                } else {
                                    None
                                }
                            }),
                            rgb2: child.attribute("rgb2").and_then(|s| {
                                let parts: Vec<f32> = s
                                    .split_whitespace()
                                    .filter_map(|x| x.parse().ok())
                                    .collect();
                                if parts.len() >= 3 {
                                    Some([parts[0], parts[1], parts[2]])
                                } else {
                                    None
                                }
                            }),
                        };
                        assets.textures.insert(name.to_string(), texture);
                    }
                }
                "material" => {
                    if let Some(name) = child.attribute("name") {
                        let material = MJCFMaterial {
                            name: name.to_string(),
                            texture: child.attribute("texture").map(|s| s.to_string()),
                            rgba: child
                                .attribute("rgba")
                                .and_then(|s| {
                                    let parts: Vec<f32> = s
                                        .split_whitespace()
                                        .filter_map(|x| x.parse().ok())
                                        .collect();
                                    if parts.len() >= 4 {
                                        Some([parts[0], parts[1], parts[2], parts[3]])
                                    } else {
                                        None
                                    }
                                })
                                .unwrap_or([0.5, 0.5, 0.5, 1.0]),
                            specular: child
                                .attribute("specular")
                                .and_then(|s| s.parse().ok())
                                .unwrap_or(0.5),
                            shininess: child
                                .attribute("shininess")
                                .and_then(|s| s.parse().ok())
                                .unwrap_or(0.5),
                            reflectance: child
                                .attribute("reflectance")
                                .and_then(|s| s.parse().ok())
                                .unwrap_or(0.0),
                        };
                        assets.materials.insert(name.to_string(), material);
                    }
                }
                _ => {}
            }
        }

        Ok(assets)
    }

    fn parse_body(&self, elem: roxmltree::Node, compiler: &MJCFCompiler) -> Result<MJCFBody> {
        let mut body = MJCFBody::default();

        body.name = elem.attribute("name").unwrap_or("unnamed").to_string();

        // Parse position
        if let Some(pos) = elem.attribute("pos") {
            let parts: Vec<f32> = pos
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                body.pos = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        // Parse orientation (quat or euler)
        if let Some(quat) = elem.attribute("quat") {
            let parts: Vec<f32> = quat
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                // MuJoCo uses wxyz, Bevy uses xyzw
                body.quat = Quat::from_xyzw(parts[1], parts[2], parts[3], parts[0]);
            }
        } else if let Some(euler) = elem.attribute("euler") {
            let parts: Vec<f32> = euler
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                let angles = if compiler.angle == AngleUnit::Degree {
                    Vec3::new(
                        parts[0].to_radians(),
                        parts[1].to_radians(),
                        parts[2].to_radians(),
                    )
                } else {
                    Vec3::new(parts[0], parts[1], parts[2])
                };
                body.euler = Some(angles);
                body.quat = Quat::from_euler(EulerRot::XYZ, angles.x, angles.y, angles.z);
            }
        }

        // Parse mocap
        if let Some(mocap) = elem.attribute("mocap") {
            body.mocap = mocap == "true";
        }

        // Parse children
        for child in elem.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "inertial" => body.inertial = Some(self.parse_inertial(child)?),
                "joint" => body.joints.push(self.parse_joint(child, compiler)?),
                "geom" => body.geoms.push(self.parse_geom(child, compiler)?),
                "site" => body.sites.push(self.parse_site(child)?),
                "body" => body.children.push(self.parse_body(child, compiler)?),
                _ => {}
            }
        }

        Ok(body)
    }

    fn parse_inertial(&self, elem: roxmltree::Node) -> Result<MJCFInertial> {
        let mut inertial = MJCFInertial {
            pos: Vec3::ZERO,
            mass: 1.0,
            diaginertia: None,
            fullinertia: None,
        };

        if let Some(pos) = elem.attribute("pos") {
            let parts: Vec<f32> = pos
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                inertial.pos = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        if let Some(mass) = elem.attribute("mass") {
            inertial.mass = mass.parse().unwrap_or(1.0);
        }

        if let Some(diaginertia) = elem.attribute("diaginertia") {
            let parts: Vec<f32> = diaginertia
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                inertial.diaginertia = Some(Vec3::new(parts[0], parts[1], parts[2]));
            }
        }

        if let Some(fullinertia) = elem.attribute("fullinertia") {
            let parts: Vec<f32> = fullinertia
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 6 {
                inertial.fullinertia =
                    Some([parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]]);
            }
        }

        Ok(inertial)
    }

    fn parse_joint(&self, elem: roxmltree::Node, compiler: &MJCFCompiler) -> Result<MJCFJoint> {
        let mut joint = MJCFJoint::default();

        joint.name = elem
            .attribute("name")
            .unwrap_or("unnamed_joint")
            .to_string();

        if let Some(jtype) = elem.attribute("type") {
            joint.joint_type = match jtype {
                "free" => MJCFJointType::Free,
                "ball" => MJCFJointType::Ball,
                "slide" => MJCFJointType::Slide,
                _ => MJCFJointType::Hinge,
            };
        }

        if let Some(pos) = elem.attribute("pos") {
            let parts: Vec<f32> = pos
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                joint.pos = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        if let Some(axis) = elem.attribute("axis") {
            let parts: Vec<f32> = axis
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                joint.axis = Vec3::new(parts[0], parts[1], parts[2]).normalize();
            }
        }

        if let Some(range) = elem.attribute("range") {
            let parts: Vec<f32> = range
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 2 {
                let (min, max) = if compiler.angle == AngleUnit::Degree {
                    (parts[0].to_radians(), parts[1].to_radians())
                } else {
                    (parts[0], parts[1])
                };
                joint.range = Some([min, max]);
            }
        }

        if let Some(limited) = elem.attribute("limited") {
            joint.limited = limited == "true";
        }

        if let Some(armature) = elem.attribute("armature") {
            joint.armature = armature.parse().unwrap_or(0.0);
        }

        if let Some(damping) = elem.attribute("damping") {
            joint.damping = damping.parse().unwrap_or(0.0);
        }

        if let Some(stiffness) = elem.attribute("stiffness") {
            joint.stiffness = stiffness.parse().unwrap_or(0.0);
        }

        if let Some(class) = elem.attribute("class") {
            joint.class = Some(class.to_string());
        }

        Ok(joint)
    }

    fn parse_geom(&self, elem: roxmltree::Node, compiler: &MJCFCompiler) -> Result<MJCFGeom> {
        let mut geom = MJCFGeom::default();

        geom.name = elem.attribute("name").map(|s| s.to_string());

        if let Some(gtype) = elem.attribute("type") {
            geom.geom_type = match gtype {
                "plane" => MJCFGeomType::Plane,
                "sphere" => MJCFGeomType::Sphere,
                "capsule" => MJCFGeomType::Capsule,
                "ellipsoid" => MJCFGeomType::Ellipsoid,
                "cylinder" => MJCFGeomType::Cylinder,
                "box" => MJCFGeomType::Box,
                "mesh" => MJCFGeomType::Mesh,
                _ => MJCFGeomType::Sphere,
            };
        }

        if let Some(pos) = elem.attribute("pos") {
            let parts: Vec<f32> = pos
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                geom.pos = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        if let Some(quat) = elem.attribute("quat") {
            let parts: Vec<f32> = quat
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                geom.quat = Quat::from_xyzw(parts[1], parts[2], parts[3], parts[0]);
            }
        } else if let Some(euler) = elem.attribute("euler") {
            let parts: Vec<f32> = euler
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                let angles = if compiler.angle == AngleUnit::Degree {
                    Vec3::new(
                        parts[0].to_radians(),
                        parts[1].to_radians(),
                        parts[2].to_radians(),
                    )
                } else {
                    Vec3::new(parts[0], parts[1], parts[2])
                };
                geom.euler = Some(angles);
                geom.quat = Quat::from_euler(EulerRot::XYZ, angles.x, angles.y, angles.z);
            }
        }

        if let Some(size) = elem.attribute("size") {
            geom.size = size
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
        }

        if let Some(rgba) = elem.attribute("rgba") {
            let parts: Vec<f32> = rgba
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                geom.rgba = [parts[0], parts[1], parts[2], parts[3]];
            }
        }

        if let Some(mesh) = elem.attribute("mesh") {
            geom.mesh = Some(mesh.to_string());
        }

        if let Some(material) = elem.attribute("material") {
            geom.material = Some(material.to_string());
        }

        if let Some(contype) = elem.attribute("contype") {
            geom.contype = contype.parse().unwrap_or(1);
        }

        if let Some(conaffinity) = elem.attribute("conaffinity") {
            geom.conaffinity = conaffinity.parse().unwrap_or(1);
        }

        if let Some(density) = elem.attribute("density") {
            geom.density = density.parse().unwrap_or(1000.0);
        }

        if let Some(friction) = elem.attribute("friction") {
            let parts: Vec<f32> = friction
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                geom.friction = [parts[0], parts[1], parts[2]];
            }
        }

        if let Some(class) = elem.attribute("class") {
            geom.class = Some(class.to_string());
        }

        Ok(geom)
    }

    fn parse_site(&self, elem: roxmltree::Node) -> Result<MJCFSite> {
        let mut site = MJCFSite {
            name: elem.attribute("name").unwrap_or("unnamed_site").to_string(),
            pos: Vec3::ZERO,
            quat: Quat::IDENTITY,
            size: Vec3::new(0.01, 0.01, 0.01),
            rgba: [1.0, 0.0, 0.0, 1.0],
        };

        if let Some(pos) = elem.attribute("pos") {
            let parts: Vec<f32> = pos
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                site.pos = Vec3::new(parts[0], parts[1], parts[2]);
            }
        }

        if let Some(quat) = elem.attribute("quat") {
            let parts: Vec<f32> = quat
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                site.quat = Quat::from_xyzw(parts[1], parts[2], parts[3], parts[0]);
            }
        }

        if let Some(size) = elem.attribute("size") {
            let parts: Vec<f32> = size
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                site.size = Vec3::new(parts[0], parts[1], parts[2]);
            } else if parts.len() == 1 {
                site.size = Vec3::splat(parts[0]);
            }
        }

        if let Some(rgba) = elem.attribute("rgba") {
            let parts: Vec<f32> = rgba
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 4 {
                site.rgba = [parts[0], parts[1], parts[2], parts[3]];
            }
        }

        Ok(site)
    }

    fn parse_actuators(&self, elem: roxmltree::Node) -> Result<Vec<MJCFActuator>> {
        let mut actuators = Vec::new();

        for child in elem.children().filter(|n| n.is_element()) {
            let mut actuator = MJCFActuator::default();

            actuator.name = child
                .attribute("name")
                .unwrap_or("unnamed_actuator")
                .to_string();

            actuator.actuator_type = match child.tag_name().name() {
                "motor" => MJCFActuatorType::Motor,
                "position" => MJCFActuatorType::Position,
                "velocity" => MJCFActuatorType::Velocity,
                "general" => MJCFActuatorType::General,
                _ => MJCFActuatorType::Motor,
            };

            if let Some(joint) = child.attribute("joint") {
                actuator.joint = Some(joint.to_string());
            }

            if let Some(site) = child.attribute("site") {
                actuator.site = Some(site.to_string());
            }

            if let Some(tendon) = child.attribute("tendon") {
                actuator.tendon = Some(tendon.to_string());
            }

            if let Some(gear) = child.attribute("gear") {
                actuator.gear = gear.parse().unwrap_or(1.0);
            }

            if let Some(ctrlrange) = child.attribute("ctrlrange") {
                let parts: Vec<f32> = ctrlrange
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if parts.len() >= 2 {
                    actuator.ctrlrange = [parts[0], parts[1]];
                }
            }

            if let Some(ctrllimited) = child.attribute("ctrllimited") {
                actuator.ctrllimited = ctrllimited == "true";
            }

            if let Some(kp) = child.attribute("kp") {
                actuator.kp = kp.parse().unwrap_or(0.0);
            }

            if let Some(kv) = child.attribute("kv") {
                actuator.kv = kv.parse().unwrap_or(0.0);
            }

            if let Some(class) = child.attribute("class") {
                actuator.class = Some(class.to_string());
            }

            actuators.push(actuator);
        }

        Ok(actuators)
    }

    fn parse_sensors(&self, elem: roxmltree::Node) -> Result<Vec<MJCFSensor>> {
        let mut sensors = Vec::new();

        for child in elem.children().filter(|n| n.is_element()) {
            let sensor = MJCFSensor {
                name: child
                    .attribute("name")
                    .unwrap_or("unnamed_sensor")
                    .to_string(),
                sensor_type: child.tag_name().name().to_string(),
                site: child.attribute("site").map(|s| s.to_string()),
                joint: child.attribute("joint").map(|s| s.to_string()),
                body: child.attribute("body").map(|s| s.to_string()),
                objtype: child.attribute("objtype").map(|s| s.to_string()),
                objname: child.attribute("objname").map(|s| s.to_string()),
                noise: child
                    .attribute("noise")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                cutoff: child
                    .attribute("cutoff")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            };

            sensors.push(sensor);
        }

        Ok(sensors)
    }

    fn parse_tendons(&self, elem: roxmltree::Node) -> Result<Vec<MJCFTendon>> {
        let mut tendons = Vec::new();

        for child in elem.children().filter(|n| n.is_element()) {
            let tendon = MJCFTendon {
                name: child
                    .attribute("name")
                    .unwrap_or("unnamed_tendon")
                    .to_string(),
                tendon_type: child.tag_name().name().to_string(),
                stiffness: child
                    .attribute("stiffness")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                damping: child
                    .attribute("damping")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                limited: child
                    .attribute("limited")
                    .map(|s| s == "true")
                    .unwrap_or(false),
                range: child
                    .attribute("range")
                    .and_then(|s| {
                        let parts: Vec<f32> = s
                            .split_whitespace()
                            .filter_map(|x| x.parse().ok())
                            .collect();
                        if parts.len() >= 2 {
                            Some([parts[0], parts[1]])
                        } else {
                            None
                        }
                    })
                    .unwrap_or([0.0, 0.0]),
            };

            tendons.push(tendon);
        }

        Ok(tendons)
    }

    fn parse_equalities(&self, elem: roxmltree::Node) -> Result<Vec<MJCFEquality>> {
        let mut equalities = Vec::new();

        for child in elem.children().filter(|n| n.is_element()) {
            let equality = MJCFEquality {
                name: child.attribute("name").map(|s| s.to_string()),
                equality_type: child.tag_name().name().to_string(),
                body1: child.attribute("body1").map(|s| s.to_string()),
                body2: child.attribute("body2").map(|s| s.to_string()),
                joint1: child.attribute("joint1").map(|s| s.to_string()),
                joint2: child.attribute("joint2").map(|s| s.to_string()),
                polycoef: child
                    .attribute("polycoef")
                    .map(|s| {
                        s.split_whitespace()
                            .filter_map(|x| x.parse().ok())
                            .collect()
                    })
                    .unwrap_or_default(),
            };

            equalities.push(equality);
        }

        Ok(equalities)
    }

    /// Load and spawn MJCF model into the world
    pub fn load(
        &mut self,
        mjcf_path: impl AsRef<Path>,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        self.load_at_position(
            mjcf_path,
            Vec3::ZERO,
            Quat::IDENTITY,
            commands,
            physics_world,
            meshes,
            materials,
        )
    }

    /// Load MJCF at specific position
    pub fn load_at_position(
        &mut self,
        mjcf_path: impl AsRef<Path>,
        position: Vec3,
        rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        let path = mjcf_path.as_ref();

        // Update base path for mesh resolution
        if let Some(parent) = path.parent() {
            self.base_path = parent.to_path_buf();
            self.mesh_loader.add_base_path(parent.to_path_buf());
            self.path_resolver.add_base_path(parent.to_path_buf());
        }

        // Parse the MJCF file
        let model = self.parse_file(path)?;

        info!(
            "Loading MJCF model '{}' with {} bodies",
            model.name,
            count_bodies(&model.worldbody)
        );

        // Apply mesh directory from compiler settings
        if let Some(meshdir) = &model.compiler.meshdir {
            let mesh_path = self.base_path.join(meshdir);
            self.mesh_loader.add_base_path(mesh_path.clone());
            self.path_resolver.add_base_path(mesh_path);
        }

        // Store defaults
        self.defaults = model.defaults.clone();

        // Spawn the body tree
        let root_entity = self.spawn_body_tree(
            &model.worldbody,
            &model,
            None,
            position,
            rotation,
            commands,
            physics_world,
            meshes,
            materials,
        )?;

        // Add Robot component
        commands.entity(root_entity).insert(Robot::new(&model.name));

        info!(
            "Spawned MJCF model '{}' with {} actuators and {} sensors",
            model.name,
            model.actuators.len(),
            model.sensors.len()
        );

        Ok(root_entity)
    }

    fn spawn_body_tree(
        &mut self,
        body: &MJCFBody,
        model: &MJCFModel,
        parent_rb: Option<RigidBodyHandle>,
        offset: Vec3,
        rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        // Calculate world position
        let world_pos = offset + rotation * body.pos;
        let world_rot = rotation * body.quat;

        // Calculate mass from inertial or geoms
        let mass = body.inertial.as_ref().map(|i| i.mass).unwrap_or_else(|| {
            // Estimate from geoms
            body.geoms
                .iter()
                .map(|g| self.estimate_geom_mass(g))
                .sum::<f32>()
                .max(0.1)
        });

        // Create rigid body
        let na_position = nalgebra::Vector3::new(world_pos.x, world_pos.y, world_pos.z);
        let na_rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            world_rot.w,
            world_rot.x,
            world_rot.y,
            world_rot.z,
        ));
        let isometry =
            nalgebra::Isometry3::from_parts(nalgebra::Translation3::from(na_position), na_rotation);

        // Determine if body should be dynamic or fixed (worldbody and static bodies are fixed)
        let rigid_body = if parent_rb.is_none() || body.mocap {
            RigidBodyBuilder::fixed().position(isometry).build()
        } else {
            RigidBodyBuilder::dynamic()
                .position(isometry)
                .additional_mass(mass)
                .linear_damping(0.5)
                .angular_damping(0.5)
                .build()
        };

        // Create entity
        let entity = commands
            .spawn((
                Name::new(body.name.clone()),
                Transform::from_translation(world_pos).with_rotation(world_rot),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
            ))
            .id();

        let rb_handle = physics_world.spawn_rigid_body(rigid_body, entity);
        commands
            .entity(entity)
            .insert(RigidBodyComponent::new(rb_handle));

        // Add colliders from geoms
        for geom in &body.geoms {
            if let Some(collider) = self.create_collider_from_geom(geom, model)? {
                physics_world.spawn_collider(collider, rb_handle);
            }

            // Add visual mesh
            self.spawn_geom_visual(geom, model, entity, commands, meshes, materials)?;
        }

        // Create joints to parent
        if let Some(parent_handle) = parent_rb {
            for joint in &body.joints {
                self.create_physics_joint(
                    joint,
                    parent_handle,
                    rb_handle,
                    commands,
                    physics_world,
                )?;
            }
        }

        // Recursively spawn children
        for child_body in &body.children {
            self.spawn_body_tree(
                child_body,
                model,
                Some(rb_handle),
                world_pos,
                world_rot,
                commands,
                physics_world,
                meshes,
                materials,
            )?;
        }

        Ok(entity)
    }

    fn estimate_geom_mass(&self, geom: &MJCFGeom) -> f32 {
        let volume = match geom.geom_type {
            MJCFGeomType::Sphere => {
                let r = geom.size.first().copied().unwrap_or(0.05);
                (4.0 / 3.0) * std::f32::consts::PI * r.powi(3)
            }
            MJCFGeomType::Box => {
                if geom.size.len() >= 3 {
                    8.0 * geom.size[0] * geom.size[1] * geom.size[2]
                } else {
                    0.001
                }
            }
            MJCFGeomType::Cylinder | MJCFGeomType::Capsule => {
                let r = geom.size.first().copied().unwrap_or(0.05);
                let h = geom.size.get(1).copied().unwrap_or(0.1);
                std::f32::consts::PI * r.powi(2) * h
            }
            _ => 0.001,
        };

        volume * geom.density
    }

    fn create_collider_from_geom(
        &mut self,
        geom: &MJCFGeom,
        model: &MJCFModel,
    ) -> Result<Option<Collider>> {
        let collider = match geom.geom_type {
            MJCFGeomType::Sphere => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                Some(
                    ColliderBuilder::new(ColliderShape::Sphere { radius })
                        .position(geom.pos)
                        .rotation(geom.quat)
                        .friction(geom.friction[0])
                        .build(),
                )
            }
            MJCFGeomType::Box => {
                let half_extents = if geom.size.len() >= 3 {
                    Vec3::new(geom.size[0], geom.size[1], geom.size[2])
                } else {
                    Vec3::splat(geom.size.first().copied().unwrap_or(0.05))
                };
                Some(
                    ColliderBuilder::new(ColliderShape::Box { half_extents })
                        .position(geom.pos)
                        .rotation(geom.quat)
                        .friction(geom.friction[0])
                        .build(),
                )
            }
            MJCFGeomType::Capsule => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_height = geom.size.get(1).copied().unwrap_or(0.1);
                Some(
                    ColliderBuilder::new(ColliderShape::Capsule {
                        half_height,
                        radius,
                    })
                    .position(geom.pos)
                    .rotation(geom.quat)
                    .friction(geom.friction[0])
                    .build(),
                )
            }
            MJCFGeomType::Cylinder => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_height = geom.size.get(1).copied().unwrap_or(0.1);
                Some(
                    ColliderBuilder::new(ColliderShape::Cylinder {
                        half_height,
                        radius,
                    })
                    .position(geom.pos)
                    .rotation(geom.quat)
                    .friction(geom.friction[0])
                    .build(),
                )
            }
            MJCFGeomType::Plane => {
                // Planes are handled as ground plane, use a large thin box
                let half_extents = Vec3::new(100.0, 0.01, 100.0);
                Some(
                    ColliderBuilder::new(ColliderShape::Box { half_extents })
                        .position(geom.pos)
                        .rotation(geom.quat)
                        .friction(geom.friction[0])
                        .build(),
                )
            }
            MJCFGeomType::Mesh => {
                // Skip mesh colliders for now (would need trimesh)
                warn!("Mesh colliders not fully supported, using bounding sphere");
                let radius = geom.size.first().copied().unwrap_or(0.1);
                Some(
                    ColliderBuilder::new(ColliderShape::Sphere { radius })
                        .position(geom.pos)
                        .rotation(geom.quat)
                        .friction(geom.friction[0])
                        .build(),
                )
            }
            MJCFGeomType::Ellipsoid => {
                // Approximate ellipsoid with sphere using average radius
                let avg_radius = geom.size.iter().sum::<f32>() / geom.size.len().max(1) as f32;
                Some(
                    ColliderBuilder::new(ColliderShape::Sphere { radius: avg_radius })
                        .position(geom.pos)
                        .rotation(geom.quat)
                        .friction(geom.friction[0])
                        .build(),
                )
            }
        };

        Ok(collider)
    }

    fn spawn_geom_visual(
        &mut self,
        geom: &MJCFGeom,
        model: &MJCFModel,
        parent_entity: Entity,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<()> {
        let transform = Transform::from_translation(geom.pos).with_rotation(geom.quat);

        // Get color from material or geom rgba
        let color = if let Some(mat_name) = &geom.material {
            if let Some(mat) = model.assets.materials.get(mat_name) {
                Color::srgba(mat.rgba[0], mat.rgba[1], mat.rgba[2], mat.rgba[3])
            } else {
                Color::srgba(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3])
            }
        } else {
            Color::srgba(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3])
        };

        let material = materials.add(StandardMaterial {
            base_color: color,
            ..default()
        });

        match geom.geom_type {
            MJCFGeomType::Sphere => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let mesh = meshes.add(Mesh::from(bevy::prelude::Sphere { radius }));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
            MJCFGeomType::Box => {
                let half = if geom.size.len() >= 3 {
                    Vec3::new(geom.size[0], geom.size[1], geom.size[2])
                } else {
                    Vec3::splat(geom.size.first().copied().unwrap_or(0.05))
                };
                let mesh = meshes.add(Mesh::from(bevy::prelude::Cuboid::new(
                    half.x * 2.0,
                    half.y * 2.0,
                    half.z * 2.0,
                )));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
            MJCFGeomType::Capsule => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_length = geom.size.get(1).copied().unwrap_or(0.1);
                let mesh = meshes.add(Mesh::from(bevy::prelude::Capsule3d {
                    radius,
                    half_length,
                }));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
            MJCFGeomType::Cylinder => {
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let half_height = geom.size.get(1).copied().unwrap_or(0.1);
                let mesh = meshes.add(Mesh::from(bevy::prelude::Cylinder {
                    radius,
                    half_height,
                }));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
            MJCFGeomType::Plane => {
                let size = if geom.size.len() >= 2 {
                    (geom.size[0] * 2.0, geom.size[1] * 2.0)
                } else {
                    (100.0, 100.0)
                };
                let mesh = meshes.add(Mesh::from(
                    bevy::prelude::Plane3d::default()
                        .mesh()
                        .size(size.0, size.1),
                ));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
            MJCFGeomType::Mesh => {
                if let Some(mesh_name) = &geom.mesh {
                    if let Some(mesh_asset) = model.assets.meshes.get(mesh_name) {
                        // Try to load the mesh file
                        let mesh_path = self.base_path.join(&mesh_asset.file);
                        let options = MeshLoadOptions::default()
                            .with_scale(mesh_asset.scale)
                            .generate_normals(true);

                        match self.mesh_loader.load(&mesh_path, options) {
                            Ok(loaded) => {
                                let mesh = meshes.add(loaded.mesh);
                                commands.entity(parent_entity).with_children(|parent| {
                                    parent.spawn((
                                        Mesh3d(mesh),
                                        MeshMaterial3d(material),
                                        transform,
                                    ));
                                });
                            }
                            Err(e) => {
                                warn!("Failed to load MJCF mesh '{}': {}", mesh_path.display(), e);
                                // Fallback to sphere
                                let radius = geom.size.first().copied().unwrap_or(0.05);
                                let mesh = meshes.add(Mesh::from(bevy::prelude::Sphere { radius }));
                                commands.entity(parent_entity).with_children(|parent| {
                                    parent.spawn((
                                        Mesh3d(mesh),
                                        MeshMaterial3d(material),
                                        transform,
                                    ));
                                });
                            }
                        }
                    }
                }
            }
            MJCFGeomType::Ellipsoid => {
                // Approximate with sphere
                let radius = geom.size.first().copied().unwrap_or(0.05);
                let mesh = meshes.add(Mesh::from(bevy::prelude::Sphere { radius }));
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((Mesh3d(mesh), MeshMaterial3d(material), transform));
                });
            }
        }

        Ok(())
    }

    fn create_physics_joint(
        &self,
        joint: &MJCFJoint,
        parent_rb: RigidBodyHandle,
        child_rb: RigidBodyHandle,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
    ) -> Result<()> {
        let anchor = joint.pos;
        let axis = joint.axis;

        let (physics_joint, joint_type) = match joint.joint_type {
            MJCFJointType::Free => {
                // Free joint = no constraint
                return Ok(());
            }
            MJCFJointType::Ball => (
                create_spherical_joint(anchor, Vec3::ZERO),
                JointType::Spherical,
            ),
            MJCFJointType::Slide => {
                if let Some(range) = joint.range {
                    (
                        create_prismatic_joint_with_limits(
                            anchor,
                            Vec3::ZERO,
                            axis,
                            range[0],
                            range[1],
                        ),
                        JointType::Prismatic,
                    )
                } else {
                    (
                        create_prismatic_joint_with_limits(anchor, Vec3::ZERO, axis, -1.0, 1.0),
                        JointType::Prismatic,
                    )
                }
            }
            MJCFJointType::Hinge => {
                if let Some(range) = joint.range {
                    (
                        create_revolute_joint_with_limits(
                            anchor,
                            Vec3::ZERO,
                            axis,
                            range[0],
                            range[1],
                        ),
                        JointType::Revolute,
                    )
                } else {
                    (
                        create_revolute_joint_with_limits(
                            anchor,
                            Vec3::ZERO,
                            axis,
                            -std::f32::consts::PI,
                            std::f32::consts::PI,
                        ),
                        JointType::Revolute,
                    )
                }
            }
        };

        let joint_handle =
            physics_world
                .impulse_joint_set
                .insert(parent_rb, child_rb, physics_joint, true);

        // The child entity would need to be looked up properly
        // For now just log the joint creation
        debug!(
            "Created {:?} joint '{}' at {:?} with axis {:?}",
            joint_type, joint.name, anchor, axis
        );

        Ok(())
    }
}

impl Default for MJCFLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// Count total bodies in tree
fn count_bodies(body: &MJCFBody) -> usize {
    1 + body.children.iter().map(count_bodies).sum::<usize>()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_mjcf() {
        let xml = r#"
        <mujoco model="test">
            <option timestep="0.001" gravity="0 0 -10"/>
            <worldbody>
                <body name="body1" pos="0 0 1">
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let loader = MJCFLoader::new();
        let model = loader.parse_string(xml, None).unwrap();

        assert_eq!(model.name, "test");
        assert_eq!(model.option.timestep, 0.001);
        assert_eq!(model.option.gravity, Vec3::new(0.0, 0.0, -10.0));
        assert_eq!(model.worldbody.children.len(), 1);
        assert_eq!(model.worldbody.children[0].name, "body1");
    }

    #[test]
    fn test_parse_joint() {
        let xml = r#"
        <mujoco>
            <compiler angle="degree"/>
            <worldbody>
                <body name="link1">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-90 90"/>
                    <geom type="box" size="0.1 0.1 0.1"/>
                </body>
            </worldbody>
        </mujoco>
        "#;

        let loader = MJCFLoader::new();
        let model = loader.parse_string(xml, None).unwrap();

        let link1 = &model.worldbody.children[0];
        assert_eq!(link1.joints.len(), 1);
        assert_eq!(link1.joints[0].name, "joint1");
        assert_eq!(link1.joints[0].joint_type, MJCFJointType::Hinge);

        // Check range converted from degrees to radians
        let range = link1.joints[0].range.unwrap();
        assert!((range[0] - (-std::f32::consts::PI / 2.0)).abs() < 0.01);
        assert!((range[1] - (std::f32::consts::PI / 2.0)).abs() < 0.01);
    }

    #[test]
    fn test_parse_actuators() {
        let xml = r#"
        <mujoco>
            <worldbody>
                <body name="link1">
                    <joint name="joint1" type="hinge"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="joint1" gear="100"/>
                <position name="pos1" joint="joint1" kp="10"/>
            </actuator>
        </mujoco>
        "#;

        let loader = MJCFLoader::new();
        let model = loader.parse_string(xml, None).unwrap();

        assert_eq!(model.actuators.len(), 2);
        assert_eq!(model.actuators[0].name, "motor1");
        assert_eq!(model.actuators[0].actuator_type, MJCFActuatorType::Motor);
        assert_eq!(model.actuators[0].gear, 100.0);
        assert_eq!(model.actuators[1].name, "pos1");
        assert_eq!(model.actuators[1].actuator_type, MJCFActuatorType::Position);
        assert_eq!(model.actuators[1].kp, 10.0);
    }
}
