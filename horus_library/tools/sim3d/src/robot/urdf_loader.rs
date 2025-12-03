use anyhow::{Context, Result};
use bevy::prelude::*;
use bevy::render::mesh::Mesh;
use rapier3d::prelude::*;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use urdf_rs::{Geometry, Joint, Link, Robot as URDFRobot, Visual};

use crate::assets::mesh::{MeshLoadOptions, MeshLoader};
use crate::assets::resolver::PathResolver;
use crate::hframe::HFrameTree;
use crate::physics::collider::{ColliderBuilder, ColliderShape};
use crate::physics::joints::{
    create_fixed_joint, create_prismatic_joint, create_prismatic_joint_with_limits,
    create_revolute_joint, create_revolute_joint_with_limits, create_spherical_joint, JointType,
    PhysicsJoint,
};
use crate::physics::world::PhysicsWorld;
use crate::robot::robot::Robot;
use bevy::render::mesh::{Indices, VertexAttributeValues};
use tracing::info;

/// Pass through URDF position directly - no coordinate conversion
/// The entire robot will be rotated at the root level to handle Z-up to Y-up
fn urdf_to_bevy_position(urdf_pos: urdf_rs::Vec3) -> Vec3 {
    Vec3::new(urdf_pos[0] as f32, urdf_pos[1] as f32, urdf_pos[2] as f32)
}

/// Pass through URDF RPY directly as Bevy quaternion
fn urdf_to_bevy_rotation(urdf_rpy: urdf_rs::Vec3) -> Quat {
    Quat::from_euler(
        EulerRot::XYZ,
        urdf_rpy[0] as f32,
        urdf_rpy[1] as f32,
        urdf_rpy[2] as f32,
    )
}

/// Pass through URDF box size directly
fn urdf_to_bevy_box_size(urdf_size: urdf_rs::Vec3) -> [f32; 3] {
    [
        urdf_size[0] as f32,
        urdf_size[1] as f32,
        urdf_size[2] as f32,
    ]
}

/// Pass through URDF axis vector directly
fn urdf_to_bevy_axis(urdf_axis: urdf_rs::Vec3) -> Vec3 {
    Vec3::new(
        urdf_axis[0] as f32,
        urdf_axis[1] as f32,
        urdf_axis[2] as f32,
    )
}

/// Rotation to convert from URDF Z-up coordinate system to Bevy Y-up
/// This should be applied to the robot root entity
pub fn urdf_to_bevy_root_rotation() -> Quat {
    // Rotate +90 degrees around X axis to convert Z-up to Y-up
    Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)
}

pub struct URDFLoader {
    base_path: PathBuf,
    mesh_loader: MeshLoader,
    path_resolver: PathResolver,
}

impl URDFLoader {
    pub fn new() -> Self {
        let mut mesh_loader = MeshLoader::new();
        let mut path_resolver = PathResolver::new();

        // Add common robot model paths
        mesh_loader.add_base_path(PathBuf::from("assets/models"));
        mesh_loader.add_base_path(PathBuf::from("assets/robots"));
        path_resolver.add_base_path(PathBuf::from("assets/models"));
        path_resolver.add_base_path(PathBuf::from("assets/robots"));

        Self {
            base_path: PathBuf::from("."),
            mesh_loader,
            path_resolver,
        }
    }

    pub fn with_base_path(mut self, path: impl Into<PathBuf>) -> Self {
        let path_buf = path.into();
        self.base_path = path_buf.clone();
        self.mesh_loader.add_base_path(path_buf.clone());
        self.path_resolver.add_base_path(path_buf);
        self
    }

    pub fn load(
        &mut self,
        urdf_path: impl AsRef<Path>,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        hframe_tree: &mut HFrameTree,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        self.load_at_position(
            urdf_path,
            Vec3::ZERO,
            Quat::IDENTITY,
            commands,
            physics_world,
            hframe_tree,
            meshes,
            materials,
        )
    }

    /// Load URDF robot at a specific position and rotation
    pub fn load_at_position(
        &mut self,
        urdf_path: impl AsRef<Path>,
        position: Vec3,
        rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        hframe_tree: &mut HFrameTree,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        let urdf_path = urdf_path.as_ref();
        let urdf_str = std::fs::read_to_string(urdf_path)
            .with_context(|| format!("Failed to read URDF file: {}", urdf_path.display()))?;

        let urdf = urdf_rs::read_from_string(&urdf_str)
            .with_context(|| format!("Failed to parse URDF file: {}", urdf_path.display()))?;

        self.spawn_robot_at_position(
            urdf,
            position,
            rotation,
            commands,
            physics_world,
            hframe_tree,
            meshes,
            materials,
        )
    }

    pub fn spawn_robot(
        &mut self,
        urdf: URDFRobot,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        hframe_tree: &mut HFrameTree,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        self.spawn_robot_at_position(
            urdf,
            Vec3::ZERO,
            Quat::IDENTITY,
            commands,
            physics_world,
            hframe_tree,
            meshes,
            materials,
        )
    }

    pub fn spawn_robot_at_position(
        &mut self,
        urdf: URDFRobot,
        initial_position: Vec3,
        initial_rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        hframe_tree: &mut HFrameTree,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        let robot_name = urdf.name.clone();

        // Find the base link (root of the kinematic tree - not a child in any joint)
        let base_link_name = urdf
            .links
            .iter()
            .find(|link| !urdf.joints.iter().any(|j| j.child.link == link.name))
            .map(|link| link.name.clone())
            .unwrap_or_else(|| {
                urdf.links
                    .first()
                    .map(|l| l.name.clone())
                    .unwrap_or_default()
            });

        info!("URDF base link identified: {}", base_link_name);

        // Map link names to entity IDs and rigid body handles
        let mut link_entities: HashMap<String, Entity> = HashMap::new();
        let mut link_rb_handles: HashMap<String, RigidBodyHandle> = HashMap::new();

        // Build a map of joint origins for positioning child links
        let mut joint_origins: HashMap<String, (Vec3, Quat)> = HashMap::new();
        for joint in &urdf.joints {
            let origin = &joint.origin;
            let translation = urdf_to_bevy_position(origin.xyz);
            let rotation = urdf_to_bevy_rotation(origin.rpy);
            joint_origins.insert(joint.child.link.clone(), (translation, rotation));
        }

        // Apply URDF-to-Bevy coordinate system conversion (Z-up to Y-up)
        let coord_conversion = urdf_to_bevy_root_rotation();
        let base_rotation = initial_rotation * coord_conversion;

        // Spawn all links with physics rigid bodies
        for link in &urdf.links {
            let is_base_link = link.name == base_link_name;

            // For base link, use initial position and rotation
            // For child links, calculate their position from joint origin to avoid overlap
            let (link_position, link_rotation) = if is_base_link {
                (initial_position, base_rotation)
            } else if let Some((joint_offset, joint_rot)) = joint_origins.get(&link.name) {
                // Transform the joint offset from URDF coords to world coords
                // The joint origin is in parent frame (base_link), so rotate it by base_rotation
                let world_offset = base_rotation * *joint_offset;
                let world_position = initial_position + world_offset;
                let world_rotation = base_rotation * *joint_rot;
                (world_position, world_rotation)
            } else {
                // Fallback: position at base (shouldn't happen for valid URDFs)
                (initial_position, base_rotation)
            };

            let (link_entity, rb_handle) = self.spawn_articulated_link(
                link,
                &robot_name,
                is_base_link,
                link_position,
                link_rotation,
                commands,
                physics_world,
                meshes,
                materials,
            )?;

            link_entities.insert(link.name.clone(), link_entity);
            if let Some(handle) = rb_handle {
                link_rb_handles.insert(link.name.clone(), handle);
            }
        }

        // Build HFrame tree from URDF
        *hframe_tree = HFrameTree::from_urdf(&urdf);

        // Create physics joints to connect links
        for joint in &urdf.joints {
            self.spawn_physics_joint(
                joint,
                &link_entities,
                &link_rb_handles,
                commands,
                physics_world,
            )?;
        }

        // The base link entity is the robot root
        let root_entity = *link_entities
            .get(&base_link_name)
            .context("Base link entity not found")?;

        // Add Robot component to the base link
        commands.entity(root_entity).insert(Robot::new(&robot_name));

        info!(
            "Spawned articulated URDF robot '{}' with {} links and {} joints",
            robot_name,
            urdf.links.len(),
            urdf.joints.len()
        );

        Ok(root_entity)
    }

    /// Spawn a URDF link - v2 approach where only the base link gets physics
    /// Child links are visual-only and follow the parent via Bevy's transform hierarchy
    #[allow(dead_code)]
    fn spawn_link_v2(
        &mut self,
        link: &Link,
        robot_name: &str,
        is_base_link: bool,
        initial_position: Vec3,
        initial_rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        if is_base_link {
            // Base link gets a physics rigid body
            self.spawn_base_link(
                link,
                robot_name,
                initial_position,
                initial_rotation,
                commands,
                physics_world,
                meshes,
                materials,
            )
        } else {
            // Child links are visual-only (no physics body)
            self.spawn_visual_link(link, robot_name, commands, meshes, materials)
        }
    }

    /// Spawn an articulated link with physics rigid body
    /// ALL links get physics bodies, connected via joints
    fn spawn_articulated_link(
        &mut self,
        link: &Link,
        robot_name: &str,
        is_base_link: bool,
        initial_position: Vec3,
        initial_rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<(Entity, Option<RigidBodyHandle>)> {
        // Calculate mass from the link's inertial properties
        let mass = if !link.inertial.mass.value.is_nan() && link.inertial.mass.value > 0.0 {
            link.inertial.mass.value as f32
        } else {
            0.1 // Default small mass for links without inertia
        };

        // Convert Bevy types to nalgebra types for Rapier
        let na_position =
            nalgebra::Vector3::new(initial_position.x, initial_position.y, initial_position.z);
        let na_rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            initial_rotation.w,
            initial_rotation.x,
            initial_rotation.y,
            initial_rotation.z,
        ));
        let isometry =
            nalgebra::Isometry3::from_parts(nalgebra::Translation3::from(na_position), na_rotation);

        // Create dynamic rigid body for all links
        let rigid_body = RigidBodyBuilder::dynamic()
            .position(isometry)
            .additional_mass(mass)
            .linear_damping(0.5)
            .angular_damping(0.5)
            .build();

        // Create entity with required components
        let entity = commands
            .spawn((
                Name::new(format!("{}::{}", robot_name, link.name)),
                Transform::from_translation(initial_position).with_rotation(initial_rotation),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
            ))
            .id();

        let rb_handle = physics_world.spawn_rigid_body(rigid_body, entity);

        // Add colliders from collision geometry
        for collision in &link.collision {
            if let Some(collider) =
                self.create_collider_from_geometry(&collision.geometry, &collision.origin)?
            {
                physics_world.spawn_collider(collider, rb_handle);
            }
        }

        // Add RigidBodyComponent so physics sync works
        use crate::physics::rigid_body::RigidBodyComponent;
        commands
            .entity(entity)
            .insert(RigidBodyComponent::new(rb_handle));

        // Add visual meshes as children
        for visual in &link.visual {
            self.spawn_visual_mesh(visual, entity, commands, meshes, materials)?;
        }

        info!(
            "Spawned articulated link '{}' with mass={:.3}kg (base={})",
            link.name, mass, is_base_link
        );

        Ok((entity, Some(rb_handle)))
    }

    /// Create physics joint connecting parent and child links
    fn spawn_physics_joint(
        &self,
        joint: &Joint,
        link_entities: &HashMap<String, Entity>,
        link_rb_handles: &HashMap<String, RigidBodyHandle>,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
    ) -> Result<()> {
        let child_entity = link_entities
            .get(&joint.child.link)
            .context(format!("Child link '{}' not found", joint.child.link))?;

        let parent_rb = link_rb_handles.get(&joint.parent.link).context(format!(
            "Parent rigid body for '{}' not found",
            joint.parent.link
        ))?;

        let child_rb = link_rb_handles.get(&joint.child.link).context(format!(
            "Child rigid body for '{}' not found",
            joint.child.link
        ))?;

        // Joint anchor in parent frame (from URDF origin)
        let origin = &joint.origin;
        let anchor = urdf_to_bevy_position(origin.xyz);

        // Joint axis
        let axis = urdf_to_bevy_axis(joint.axis.xyz);

        // Create appropriate joint type based on URDF
        let (physics_joint, joint_type) = match joint.joint_type {
            urdf_rs::JointType::Revolute => {
                let limit = &joint.limit;
                if limit.upper != 0.0 || limit.lower != 0.0 {
                    let min_angle = limit.lower as f32;
                    let max_angle = limit.upper as f32;
                    (
                        create_revolute_joint_with_limits(
                            anchor,
                            Vec3::ZERO,
                            axis,
                            min_angle,
                            max_angle,
                        ),
                        JointType::Revolute,
                    )
                } else {
                    (
                        create_revolute_joint(anchor, Vec3::ZERO, axis),
                        JointType::Revolute,
                    )
                }
            }
            urdf_rs::JointType::Continuous => (
                create_revolute_joint(anchor, Vec3::ZERO, axis),
                JointType::Revolute,
            ),
            urdf_rs::JointType::Prismatic => {
                let limit = &joint.limit;
                if limit.upper != 0.0 || limit.lower != 0.0 {
                    let min_dist = limit.lower as f32;
                    let max_dist = limit.upper as f32;
                    (
                        create_prismatic_joint_with_limits(
                            anchor,
                            Vec3::ZERO,
                            axis,
                            min_dist,
                            max_dist,
                        ),
                        JointType::Prismatic,
                    )
                } else {
                    (
                        create_prismatic_joint(anchor, Vec3::ZERO, axis),
                        JointType::Prismatic,
                    )
                }
            }
            urdf_rs::JointType::Fixed => (create_fixed_joint(anchor, Vec3::ZERO), JointType::Fixed),
            urdf_rs::JointType::Floating => {
                // Floating joints are represented by no joint constraint
                info!("Skipping floating joint '{}'", joint.name);
                return Ok(());
            }
            urdf_rs::JointType::Planar => {
                // Planar joints not directly supported, use fixed for now
                info!("Using fixed joint for planar joint '{}'", joint.name);
                (create_fixed_joint(anchor, Vec3::ZERO), JointType::Fixed)
            }
            urdf_rs::JointType::Spherical => {
                // Spherical joint (ball joint)
                (
                    create_spherical_joint(anchor, Vec3::ZERO),
                    JointType::Spherical,
                )
            }
        };

        // Insert joint into physics world
        let joint_handle =
            physics_world
                .impulse_joint_set
                .insert(*parent_rb, *child_rb, physics_joint, true);

        // Add PhysicsJoint component to child entity
        commands.entity(*child_entity).insert(PhysicsJoint {
            handle: joint_handle,
            joint_type,
        });

        info!(
            "Created {:?} joint '{}': {} -> {}",
            joint_type, joint.name, joint.parent.link, joint.child.link
        );

        Ok(())
    }

    /// Spawn the base link with physics rigid body
    fn spawn_base_link(
        &mut self,
        link: &Link,
        robot_name: &str,
        initial_position: Vec3,
        initial_rotation: Quat,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        // Calculate total mass from the link's inertial properties
        let mass = if !link.inertial.mass.value.is_nan() && link.inertial.mass.value > 0.0 {
            link.inertial.mass.value as f32
        } else {
            1.0 // Default mass
        };

        // Apply URDF-to-Bevy coordinate system conversion (Z-up to Y-up)
        // This rotation converts the URDF coordinate system to Bevy's
        let coord_conversion = urdf_to_bevy_root_rotation();
        let final_rotation = initial_rotation * coord_conversion;

        // Convert Bevy types to nalgebra types for Rapier
        let na_position =
            nalgebra::Vector3::new(initial_position.x, initial_position.y, initial_position.z);
        let na_rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            final_rotation.w,
            final_rotation.x,
            final_rotation.y,
            final_rotation.z,
        ));
        let isometry =
            nalgebra::Isometry3::from_parts(nalgebra::Translation3::from(na_position), na_rotation);

        // Create dynamic rigid body at the specified position
        let rigid_body = RigidBodyBuilder::dynamic()
            .position(isometry)
            .additional_mass(mass)
            .linear_damping(0.5)
            .angular_damping(0.5)
            .build();

        // Create entity with required components
        // Must include GlobalTransform for transform hierarchy to work
        let entity = commands
            .spawn((
                Name::new(format!("{}::{}", robot_name, link.name)),
                Transform::from_translation(initial_position).with_rotation(final_rotation),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
            ))
            .id();

        let rb_handle = physics_world.spawn_rigid_body(rigid_body, entity);

        // Add colliders from collision geometry
        for collision in &link.collision {
            if let Some(collider) =
                self.create_collider_from_geometry(&collision.geometry, &collision.origin)?
            {
                physics_world.spawn_collider(collider, rb_handle);
            }
        }

        // Add RigidBodyComponent so physics sync works
        use crate::physics::rigid_body::RigidBodyComponent;
        commands
            .entity(entity)
            .insert(RigidBodyComponent::new(rb_handle));

        // Add visual meshes as children
        for visual in &link.visual {
            self.spawn_visual_mesh(visual, entity, commands, meshes, materials)?;
        }

        info!("Spawned base link '{}' with mass={:.2}kg", link.name, mass);

        Ok(entity)
    }

    /// Spawn a visual-only link (no physics body)
    fn spawn_visual_link(
        &mut self,
        link: &Link,
        robot_name: &str,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        // Create entity without physics (visual only)
        // Must include GlobalTransform for transform hierarchy to work
        let entity = commands
            .spawn((
                Name::new(format!("{}::{}", robot_name, link.name)),
                Transform::default(),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
            ))
            .id();

        // Add visual meshes as children
        for visual in &link.visual {
            self.spawn_visual_mesh(visual, entity, commands, meshes, materials)?;
        }

        Ok(entity)
    }

    #[allow(dead_code)]
    fn spawn_link(
        &mut self,
        link: &Link,
        robot_name: &str,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<Entity> {
        // Create rigid body at origin - position will be set later based on joint hierarchy
        // The inertial origin is relative to the link frame, used for mass properties
        let rigid_body = if !link.inertial.mass.value.is_nan() && link.inertial.mass.value > 0.0 {
            let mass = link.inertial.mass.value as f32;

            // Create rigid body at origin (will be positioned later by joint hierarchy)
            // The center of mass offset from inertial.origin is handled by Rapier internally
            RigidBodyBuilder::dynamic().additional_mass(mass).build()
        } else {
            // Static/fixed body for links without inertia
            RigidBodyBuilder::fixed().build()
        };

        // Create entity and spawn rigid body
        let entity = commands
            .spawn((
                Name::new(format!("{}::{}", robot_name, link.name)),
                Transform::default(),
                Visibility::default(),
            ))
            .id();

        let rb_handle = physics_world.spawn_rigid_body(rigid_body, entity);

        // Add colliders from collision geometry
        for collision in &link.collision {
            if let Some(collider) =
                self.create_collider_from_geometry(&collision.geometry, &collision.origin)?
            {
                physics_world.spawn_collider(collider, rb_handle);
            }
        }

        // Add visual meshes
        for visual in &link.visual {
            self.spawn_visual_mesh(visual, entity, commands, meshes, materials)?;
        }

        Ok(entity)
    }

    #[allow(dead_code)]
    fn spawn_joint(
        &self,
        joint: &Joint,
        link_entities: &HashMap<String, Entity>,
        link_rb_handles: &HashMap<String, RigidBodyHandle>,
        commands: &mut Commands,
        physics_world: &mut PhysicsWorld,
    ) -> Result<()> {
        let _parent_entity = link_entities
            .get(&joint.parent.link)
            .context(format!("Parent link '{}' not found", joint.parent.link))?;

        let child_entity = link_entities
            .get(&joint.child.link)
            .context(format!("Child link '{}' not found", joint.child.link))?;

        let parent_rb = link_rb_handles.get(&joint.parent.link).context(format!(
            "Parent rigid body for '{}' not found",
            joint.parent.link
        ))?;

        let child_rb = link_rb_handles.get(&joint.child.link).context(format!(
            "Child rigid body for '{}' not found",
            joint.child.link
        ))?;

        // Convert origin to anchor points with URDF->Bevy coordinate conversion
        let origin = &joint.origin;
        let anchor = urdf_to_bevy_position(origin.xyz);

        let axis = urdf_to_bevy_axis(joint.axis.xyz);

        // Create appropriate joint type
        let (physics_joint, joint_type) = match joint.joint_type {
            urdf_rs::JointType::Revolute => {
                let limit = &joint.limit;
                if limit.upper != 0.0 || limit.lower != 0.0 {
                    let min_angle = limit.lower as f32;
                    let max_angle = limit.upper as f32;
                    (
                        create_revolute_joint_with_limits(
                            anchor, anchor, axis, min_angle, max_angle,
                        ),
                        JointType::Revolute,
                    )
                } else {
                    (
                        create_revolute_joint(anchor, anchor, axis),
                        JointType::Revolute,
                    )
                }
            }
            urdf_rs::JointType::Continuous => (
                create_revolute_joint(anchor, anchor, axis),
                JointType::Revolute,
            ),
            urdf_rs::JointType::Prismatic => {
                let limit = &joint.limit;
                if limit.upper != 0.0 || limit.lower != 0.0 {
                    let min_dist = limit.lower as f32;
                    let max_dist = limit.upper as f32;
                    (
                        create_prismatic_joint_with_limits(
                            anchor, anchor, axis, min_dist, max_dist,
                        ),
                        JointType::Prismatic,
                    )
                } else {
                    (
                        create_prismatic_joint(anchor, anchor, axis),
                        JointType::Prismatic,
                    )
                }
            }
            urdf_rs::JointType::Fixed => (create_fixed_joint(anchor, anchor), JointType::Fixed),
            urdf_rs::JointType::Floating => {
                // Floating joints are represented by no joint constraint
                return Ok(());
            }
            urdf_rs::JointType::Planar => {
                // Planar joints not directly supported, use fixed for now
                (create_fixed_joint(anchor, anchor), JointType::Fixed)
            }
            urdf_rs::JointType::Spherical => {
                // Spherical joint (ball joint)
                (create_spherical_joint(anchor, anchor), JointType::Spherical)
            }
        };

        // Insert joint into physics world
        let joint_handle =
            physics_world
                .impulse_joint_set
                .insert(*parent_rb, *child_rb, physics_joint, true);

        // Add PhysicsJoint component to child entity
        commands.entity(*child_entity).insert(PhysicsJoint {
            handle: joint_handle,
            joint_type,
        });

        Ok(())
    }

    fn create_collider_from_geometry(
        &mut self,
        geometry: &Geometry,
        origin: &urdf_rs::Pose,
    ) -> Result<Option<Collider>> {
        // Keep collision origin in URDF coordinates (not converted)
        // The root rotation handles the Z-up to Y-up conversion for the whole robot
        let collision_pos = Vec3::new(
            origin.xyz[0] as f32,
            origin.xyz[1] as f32,
            origin.xyz[2] as f32,
        );
        let collision_rot = Quat::from_euler(
            EulerRot::XYZ,
            origin.rpy[0] as f32,
            origin.rpy[1] as f32,
            origin.rpy[2] as f32,
        );

        let collider = match geometry {
            Geometry::Box { size } => {
                let half_extents = Vec3::new(
                    size[0] as f32 / 2.0,
                    size[1] as f32 / 2.0,
                    size[2] as f32 / 2.0,
                );
                Some(
                    ColliderBuilder::new(ColliderShape::Box { half_extents })
                        .position(collision_pos)
                        .rotation(collision_rot)
                        .build(),
                )
            }
            Geometry::Cylinder { radius, length } => {
                let half_height = *length as f32 / 2.0;
                let radius = *radius as f32;
                Some(
                    ColliderBuilder::new(ColliderShape::Cylinder {
                        half_height,
                        radius,
                    })
                    .position(collision_pos)
                    .rotation(collision_rot)
                    .build(),
                )
            }
            Geometry::Capsule { radius, length } => {
                let half_height = *length as f32 / 2.0;
                let radius = *radius as f32;
                Some(
                    ColliderBuilder::new(ColliderShape::Capsule {
                        half_height,
                        radius,
                    })
                    .position(collision_pos)
                    .rotation(collision_rot)
                    .build(),
                )
            }
            Geometry::Sphere { radius } => {
                let radius = *radius as f32;
                Some(
                    ColliderBuilder::new(ColliderShape::Sphere { radius })
                        .position(collision_pos)
                        .rotation(collision_rot)
                        .build(),
                )
            }
            Geometry::Mesh { filename, scale } => {
                // Load mesh file and create trimesh collider
                let mesh_path = self.resolve_mesh_path(filename);

                // Apply scale from URDF
                let scale_vec = if let Some(s) = scale {
                    Vec3::new(s[0] as f32, s[1] as f32, s[2] as f32)
                } else {
                    Vec3::ONE
                };

                // Try to load the mesh
                let load_options = MeshLoadOptions::default()
                    .with_scale(scale_vec)
                    .generate_normals(false) // Don't need normals for collision
                    .validate(false); // Skip validation for collision meshes

                match self.mesh_loader.load(&mesh_path, load_options) {
                    Ok(loaded_mesh) => {
                        // Extract vertices and indices from the loaded mesh
                        if let Some((vertices, indices)) =
                            extract_mesh_data_for_collider(&loaded_mesh.mesh)
                        {
                            info!(
                                "Created mesh collider from {} ({} vertices, {} triangles)",
                                mesh_path.display(),
                                vertices.len(),
                                indices.len()
                            );
                            Some(
                                ColliderBuilder::new(ColliderShape::Mesh { vertices, indices })
                                    .position(collision_pos)
                                    .rotation(collision_rot)
                                    .build(),
                            )
                        } else {
                            warn!(
                                "Failed to extract collision data from mesh: {}",
                                mesh_path.display()
                            );
                            None
                        }
                    }
                    Err(e) => {
                        warn!(
                            "Failed to load mesh for collider {}: {}. Using bounding box approximation.",
                            mesh_path.display(),
                            e
                        );
                        // Fall back to a bounding box approximation if mesh loading fails
                        None
                    }
                }
            }
        };

        Ok(collider)
    }

    fn spawn_visual_mesh(
        &mut self,
        visual: &Visual,
        parent_entity: Entity,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) -> Result<()> {
        // Create transform from visual origin with URDF->Bevy coordinate conversion
        let origin = &visual.origin;
        let translation = urdf_to_bevy_position(origin.xyz);
        let rotation = urdf_to_bevy_rotation(origin.rpy);

        let transform = Transform::from_translation(translation).with_rotation(rotation);

        // Create material
        let material = if let Some(mat) = &visual.material {
            if let Some(color) = &mat.color {
                Color::srgba(
                    color.rgba[0] as f32,
                    color.rgba[1] as f32,
                    color.rgba[2] as f32,
                    color.rgba[3] as f32,
                )
            } else {
                Color::srgb(0.8, 0.8, 0.8)
            }
        } else {
            Color::srgb(0.8, 0.8, 0.8)
        };

        // Spawn visual based on geometry type
        match &visual.geometry {
            Geometry::Box { size } => {
                // Convert URDF box size to Bevy coordinates
                let bevy_size = urdf_to_bevy_box_size(*size);
                let cuboid = bevy::prelude::Cuboid::new(bevy_size[0], bevy_size[1], bevy_size[2]);
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((
                        Mesh3d(meshes.add(Mesh::from(cuboid))),
                        MeshMaterial3d(materials.add(StandardMaterial {
                            base_color: material,
                            ..default()
                        })),
                        transform,
                    ));
                });
            }
            Geometry::Cylinder { radius, length } => {
                let cylinder = bevy::prelude::Cylinder {
                    radius: *radius as f32,
                    half_height: *length as f32 / 2.0,
                };
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((
                        Mesh3d(meshes.add(Mesh::from(cylinder))),
                        MeshMaterial3d(materials.add(StandardMaterial {
                            base_color: material,
                            ..default()
                        })),
                        transform,
                    ));
                });
            }
            Geometry::Sphere { radius } => {
                let sphere = bevy::prelude::Sphere {
                    radius: *radius as f32,
                };
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((
                        Mesh3d(meshes.add(Mesh::from(sphere))),
                        MeshMaterial3d(materials.add(StandardMaterial {
                            base_color: material,
                            ..default()
                        })),
                        transform,
                    ));
                });
            }
            Geometry::Capsule { radius, length } => {
                let capsule = bevy::prelude::Capsule3d {
                    radius: *radius as f32,
                    half_length: *length as f32 / 2.0,
                };
                commands.entity(parent_entity).with_children(|parent| {
                    parent.spawn((
                        Mesh3d(meshes.add(Mesh::from(capsule))),
                        MeshMaterial3d(materials.add(StandardMaterial {
                            base_color: material,
                            ..default()
                        })),
                        transform,
                    ));
                });
            }
            Geometry::Mesh { filename, scale } => {
                // Load mesh file using the mesh loader
                let resolved_path = match self.resolve_mesh_path_full(filename) {
                    Ok(path) => path,
                    Err(e) => {
                        warn!("Failed to resolve mesh path '{}': {}", filename, e);
                        warn!("Spawning fallback cube geometry instead");
                        // Spawn fallback cube when mesh path cannot be resolved
                        let cuboid = bevy::prelude::Cuboid::new(0.1, 0.1, 0.1);
                        commands.entity(parent_entity).with_children(|parent| {
                            parent.spawn((
                                Mesh3d(meshes.add(Mesh::from(cuboid))),
                                MeshMaterial3d(materials.add(StandardMaterial {
                                    base_color: Color::srgb(1.0, 0.0, 1.0), // Magenta for "missing mesh"
                                    ..default()
                                })),
                                transform,
                            ));
                        });
                        return Ok(());
                    }
                };

                // Set up mesh load options with scale
                let mesh_scale = scale
                    .as_ref()
                    .map(|s| Vec3::new(s[0] as f32, s[1] as f32, s[2] as f32))
                    .unwrap_or(Vec3::ONE);

                let load_options = MeshLoadOptions::default()
                    .with_scale(mesh_scale)
                    .generate_normals(true)
                    .generate_tangents(false)
                    .validate(true);

                // Load the mesh
                match self.mesh_loader.load(&resolved_path, load_options) {
                    Ok(loaded_mesh) => {
                        info!(
                            "Successfully loaded mesh: {} ({} vertices, {} triangles)",
                            resolved_path.display(),
                            loaded_mesh.vertex_count,
                            loaded_mesh.triangle_count
                        );

                        // Add mesh to assets
                        let mesh_handle = meshes.add(loaded_mesh.mesh);

                        // Use material from URDF if available, otherwise use mesh material
                        let material_handle = materials.add(StandardMaterial {
                            base_color: material,
                            ..default()
                        });

                        // Spawn the loaded mesh
                        commands.entity(parent_entity).with_children(|parent| {
                            parent.spawn((
                                Mesh3d(mesh_handle),
                                MeshMaterial3d(material_handle),
                                transform,
                            ));
                        });
                    }
                    Err(e) => {
                        warn!("Failed to load mesh '{}': {}", resolved_path.display(), e);
                        warn!("Spawning fallback cube geometry instead");
                        // Spawn fallback cube when mesh loading fails
                        let cuboid = bevy::prelude::Cuboid::new(0.1, 0.1, 0.1);
                        commands.entity(parent_entity).with_children(|parent| {
                            parent.spawn((
                                Mesh3d(meshes.add(Mesh::from(cuboid))),
                                MeshMaterial3d(materials.add(StandardMaterial {
                                    base_color: Color::srgb(1.0, 0.0, 1.0), // Magenta for "missing mesh"
                                    ..default()
                                })),
                                transform,
                            ));
                        });
                    }
                }
            }
        }

        Ok(())
    }

    fn resolve_mesh_path(&self, filename: &str) -> PathBuf {
        // Simple fallback for collider mesh paths (not used for visual meshes)
        if filename.starts_with("package://") {
            let relative_path = filename.strip_prefix("package://").unwrap();
            self.base_path.join(relative_path)
        } else if filename.starts_with("file://") {
            PathBuf::from(filename.strip_prefix("file://").unwrap())
        } else {
            self.base_path.join(filename)
        }
    }

    fn resolve_mesh_path_full(&self, filename: &str) -> Result<PathBuf> {
        // Use the PathResolver for comprehensive URI resolution
        self.path_resolver
            .resolve(filename)
            .with_context(|| format!("Failed to resolve mesh URI: {}", filename))
    }

    #[allow(dead_code)]
    fn get_rb_handle_for_entity(
        &self,
        entity: Entity,
        physics_world: &PhysicsWorld,
    ) -> Option<RigidBodyHandle> {
        // Search through rigid body set to find handle matching entity
        physics_world
            .rigid_body_set
            .iter()
            .find(|(_, rb)| Entity::from_bits(rb.user_data as u64) == entity)
            .map(|(handle, _)| handle)
    }
}

impl Default for URDFLoader {
    fn default() -> Self {
        Self::new()
    }
}

/// Extract vertex and index data from a Bevy Mesh for use in physics collision
///
/// Returns (vertices, indices) suitable for creating a trimesh collider
fn extract_mesh_data_for_collider(mesh: &Mesh) -> Option<(Vec<Vec3>, Vec<[u32; 3]>)> {
    // Extract vertex positions
    let positions = mesh.attribute(Mesh::ATTRIBUTE_POSITION)?;
    let vertices: Vec<Vec3> = match positions {
        VertexAttributeValues::Float32x3(positions) => positions
            .iter()
            .map(|p| Vec3::new(p[0], p[1], p[2]))
            .collect(),
        _ => return None,
    };

    if vertices.is_empty() {
        return None;
    }

    // Extract indices
    let indices = mesh.indices()?;
    let index_vec: Vec<u32> = match indices {
        Indices::U16(idx) => idx.iter().map(|i| *i as u32).collect(),
        Indices::U32(idx) => idx.clone(),
    };

    // Convert to triangle triplets
    if index_vec.len() % 3 != 0 {
        return None;
    }

    let triangles: Vec<[u32; 3]> = index_vec
        .chunks(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect();

    if triangles.is_empty() {
        return None;
    }

    Some((vertices, triangles))
}
