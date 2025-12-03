//! SRDF (Semantic Robot Description Format) loader
//!
//! SRDF is used by MoveIt and other motion planning systems to provide
//! semantic information about robots that supplements URDF:
//! - Planning groups (kinematic chains, joint groups)
//! - End effectors
//! - Virtual joints (for mobile bases)
//! - Disabled collision pairs (self-collision filtering)
//! - Named robot poses/configurations
//! - Passive joints
//!
//! This module parses SRDF files and provides the data structures needed
//! for motion planning integration.

use anyhow::{Context, Result};
use std::collections::{HashMap, HashSet};
use std::path::Path;
use tracing::{debug, info};

/// Semantic robot description
#[derive(Debug, Clone, Default)]
pub struct SRDF {
    /// Robot name (should match URDF)
    pub name: String,
    /// Planning groups
    pub groups: Vec<PlanningGroup>,
    /// Group states (named poses)
    pub group_states: Vec<GroupState>,
    /// End effectors
    pub end_effectors: Vec<EndEffector>,
    /// Virtual joints (e.g., floating base)
    pub virtual_joints: Vec<VirtualJoint>,
    /// Disabled collision pairs (self-collision filtering)
    pub disabled_collisions: Vec<DisabledCollision>,
    /// Passive joints (non-actuated)
    pub passive_joints: Vec<PassiveJoint>,
    /// Link sphere approximations (for collision checking)
    pub link_sphere_approximations: Vec<LinkSphereApproximation>,
}

/// A planning group (kinematic chain or joint collection)
#[derive(Debug, Clone, Default)]
pub struct PlanningGroup {
    /// Group name (e.g., "arm", "gripper", "manipulator")
    pub name: String,
    /// Base link of kinematic chain
    pub base_link: Option<String>,
    /// Tip link of kinematic chain
    pub tip_link: Option<String>,
    /// Individual joints in the group
    pub joints: Vec<String>,
    /// Individual links in the group
    pub links: Vec<String>,
    /// Kinematic chains (base_link -> tip_link)
    pub chains: Vec<KinematicChain>,
    /// Sub-groups included in this group
    pub subgroups: Vec<String>,
}

/// A kinematic chain within a planning group
#[derive(Debug, Clone)]
pub struct KinematicChain {
    pub base_link: String,
    pub tip_link: String,
}

/// A named robot pose/configuration
#[derive(Debug, Clone)]
pub struct GroupState {
    /// Name of the state (e.g., "home", "ready", "extended")
    pub name: String,
    /// Group this state belongs to
    pub group: String,
    /// Joint positions in this state
    pub joint_values: HashMap<String, f64>,
}

/// End effector definition
#[derive(Debug, Clone)]
pub struct EndEffector {
    /// End effector name
    pub name: String,
    /// Planning group for the end effector
    pub group: String,
    /// Parent link (where end effector attaches)
    pub parent_link: String,
    /// Parent group (the arm/manipulator)
    pub parent_group: Option<String>,
}

/// Virtual joint (connection between robot and world)
#[derive(Debug, Clone)]
pub struct VirtualJoint {
    /// Joint name
    pub name: String,
    /// Joint type (fixed, floating, planar)
    pub joint_type: VirtualJointType,
    /// Parent frame (usually "world")
    pub parent_frame: String,
    /// Child link (robot base)
    pub child_link: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VirtualJointType {
    Fixed,
    Floating,
    Planar,
}

impl From<&str> for VirtualJointType {
    fn from(s: &str) -> Self {
        match s.to_lowercase().as_str() {
            "floating" => VirtualJointType::Floating,
            "planar" => VirtualJointType::Planar,
            _ => VirtualJointType::Fixed,
        }
    }
}

/// Disabled collision pair (for self-collision filtering)
#[derive(Debug, Clone)]
pub struct DisabledCollision {
    /// First link name
    pub link1: String,
    /// Second link name
    pub link2: String,
    /// Reason for disabling
    pub reason: DisabledCollisionReason,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DisabledCollisionReason {
    /// Links are adjacent in the kinematic chain
    Adjacent,
    /// Links are always in collision (e.g., nested geometry)
    Default,
    /// Links can never collide due to joint limits
    Never,
    /// User specified to disable
    User,
    /// Unknown reason
    Unknown,
}

impl From<&str> for DisabledCollisionReason {
    fn from(s: &str) -> Self {
        match s.to_lowercase().as_str() {
            "adjacent" => DisabledCollisionReason::Adjacent,
            "default" => DisabledCollisionReason::Default,
            "never" => DisabledCollisionReason::Never,
            "user" => DisabledCollisionReason::User,
            _ => DisabledCollisionReason::Unknown,
        }
    }
}

/// Passive joint (non-actuated, moves freely)
#[derive(Debug, Clone)]
pub struct PassiveJoint {
    pub name: String,
}

/// Sphere approximation for collision checking
#[derive(Debug, Clone)]
pub struct LinkSphereApproximation {
    pub link: String,
    pub spheres: Vec<CollisionSphere>,
}

/// A sphere used for collision approximation
#[derive(Debug, Clone)]
pub struct CollisionSphere {
    pub center: [f64; 3],
    pub radius: f64,
}

/// SRDF loader
pub struct SRDFLoader;

impl SRDFLoader {
    /// Load SRDF from file
    pub fn load_file(path: impl AsRef<Path>) -> Result<SRDF> {
        let path = path.as_ref();
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read SRDF file: {}", path.display()))?;

        Self::parse_string(&content)
    }

    /// Parse SRDF from string
    pub fn parse_string(xml: &str) -> Result<SRDF> {
        let doc = roxmltree::Document::parse(xml).with_context(|| "Failed to parse SRDF XML")?;

        let root = doc.root_element();
        if !root.has_tag_name("robot") {
            anyhow::bail!("SRDF root element must be <robot>");
        }

        let mut srdf = SRDF::default();

        // Get robot name
        if let Some(name) = root.attribute("name") {
            srdf.name = name.to_string();
        }

        // Parse all elements
        for child in root.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "group" => {
                    if let Ok(group) = Self::parse_group(child) {
                        srdf.groups.push(group);
                    }
                }
                "group_state" => {
                    if let Ok(state) = Self::parse_group_state(child) {
                        srdf.group_states.push(state);
                    }
                }
                "end_effector" => {
                    if let Ok(ee) = Self::parse_end_effector(child) {
                        srdf.end_effectors.push(ee);
                    }
                }
                "virtual_joint" => {
                    if let Ok(vj) = Self::parse_virtual_joint(child) {
                        srdf.virtual_joints.push(vj);
                    }
                }
                "disable_collisions" => {
                    if let Ok(dc) = Self::parse_disabled_collision(child) {
                        srdf.disabled_collisions.push(dc);
                    }
                }
                "passive_joint" => {
                    if let Ok(pj) = Self::parse_passive_joint(child) {
                        srdf.passive_joints.push(pj);
                    }
                }
                "link_sphere_approximation" => {
                    if let Ok(lsa) = Self::parse_link_sphere_approximation(child) {
                        srdf.link_sphere_approximations.push(lsa);
                    }
                }
                _ => {
                    debug!("Ignoring unknown SRDF element: {}", child.tag_name().name());
                }
            }
        }

        info!(
            "Loaded SRDF '{}': {} groups, {} group_states, {} end_effectors, {} virtual_joints, {} disabled_collisions",
            srdf.name,
            srdf.groups.len(),
            srdf.group_states.len(),
            srdf.end_effectors.len(),
            srdf.virtual_joints.len(),
            srdf.disabled_collisions.len()
        );

        Ok(srdf)
    }

    fn parse_group(elem: roxmltree::Node) -> Result<PlanningGroup> {
        let mut group = PlanningGroup::default();

        group.name = elem
            .attribute("name")
            .ok_or_else(|| anyhow::anyhow!("Group missing name attribute"))?
            .to_string();

        for child in elem.children().filter(|n| n.is_element()) {
            match child.tag_name().name() {
                "joint" => {
                    if let Some(name) = child.attribute("name") {
                        group.joints.push(name.to_string());
                    }
                }
                "link" => {
                    if let Some(name) = child.attribute("name") {
                        group.links.push(name.to_string());
                    }
                }
                "chain" => {
                    let base = child
                        .attribute("base_link")
                        .map(|s| s.to_string())
                        .unwrap_or_default();
                    let tip = child
                        .attribute("tip_link")
                        .map(|s| s.to_string())
                        .unwrap_or_default();

                    if !base.is_empty() && !tip.is_empty() {
                        group.chains.push(KinematicChain {
                            base_link: base.clone(),
                            tip_link: tip.clone(),
                        });

                        // Set base and tip if not already set
                        if group.base_link.is_none() {
                            group.base_link = Some(base);
                        }
                        if group.tip_link.is_none() {
                            group.tip_link = Some(tip);
                        }
                    }
                }
                "group" => {
                    if let Some(name) = child.attribute("name") {
                        group.subgroups.push(name.to_string());
                    }
                }
                _ => {}
            }
        }

        debug!(
            "Parsed group '{}': {} joints, {} links, {} chains",
            group.name,
            group.joints.len(),
            group.links.len(),
            group.chains.len()
        );

        Ok(group)
    }

    fn parse_group_state(elem: roxmltree::Node) -> Result<GroupState> {
        let name = elem
            .attribute("name")
            .ok_or_else(|| anyhow::anyhow!("Group state missing name"))?
            .to_string();
        let group = elem
            .attribute("group")
            .ok_or_else(|| anyhow::anyhow!("Group state missing group"))?
            .to_string();

        let mut joint_values = HashMap::new();

        for child in elem.children().filter(|n| n.is_element()) {
            if child.tag_name().name() == "joint" {
                if let (Some(joint_name), Some(value)) =
                    (child.attribute("name"), child.attribute("value"))
                {
                    if let Ok(v) = value.parse::<f64>() {
                        joint_values.insert(joint_name.to_string(), v);
                    }
                }
            }
        }

        debug!(
            "Parsed group state '{}' for group '{}': {} joint values",
            name,
            group,
            joint_values.len()
        );

        Ok(GroupState {
            name,
            group,
            joint_values,
        })
    }

    fn parse_end_effector(elem: roxmltree::Node) -> Result<EndEffector> {
        let name = elem
            .attribute("name")
            .ok_or_else(|| anyhow::anyhow!("End effector missing name"))?
            .to_string();
        let group = elem
            .attribute("group")
            .ok_or_else(|| anyhow::anyhow!("End effector missing group"))?
            .to_string();
        let parent_link = elem
            .attribute("parent_link")
            .ok_or_else(|| anyhow::anyhow!("End effector missing parent_link"))?
            .to_string();
        let parent_group = elem.attribute("parent_group").map(|s| s.to_string());

        debug!(
            "Parsed end effector '{}': group='{}', parent_link='{}'",
            name, group, parent_link
        );

        Ok(EndEffector {
            name,
            group,
            parent_link,
            parent_group,
        })
    }

    fn parse_virtual_joint(elem: roxmltree::Node) -> Result<VirtualJoint> {
        let name = elem
            .attribute("name")
            .ok_or_else(|| anyhow::anyhow!("Virtual joint missing name"))?
            .to_string();
        let joint_type = elem
            .attribute("type")
            .map(VirtualJointType::from)
            .unwrap_or(VirtualJointType::Fixed);
        let parent_frame = elem
            .attribute("parent_frame")
            .unwrap_or("world")
            .to_string();
        let child_link = elem
            .attribute("child_link")
            .ok_or_else(|| anyhow::anyhow!("Virtual joint missing child_link"))?
            .to_string();

        debug!(
            "Parsed virtual joint '{}': type={:?}, parent='{}', child='{}'",
            name, joint_type, parent_frame, child_link
        );

        Ok(VirtualJoint {
            name,
            joint_type,
            parent_frame,
            child_link,
        })
    }

    fn parse_disabled_collision(elem: roxmltree::Node) -> Result<DisabledCollision> {
        let link1 = elem
            .attribute("link1")
            .ok_or_else(|| anyhow::anyhow!("Disabled collision missing link1"))?
            .to_string();
        let link2 = elem
            .attribute("link2")
            .ok_or_else(|| anyhow::anyhow!("Disabled collision missing link2"))?
            .to_string();
        let reason = elem
            .attribute("reason")
            .map(DisabledCollisionReason::from)
            .unwrap_or(DisabledCollisionReason::Unknown);

        Ok(DisabledCollision {
            link1,
            link2,
            reason,
        })
    }

    fn parse_passive_joint(elem: roxmltree::Node) -> Result<PassiveJoint> {
        let name = elem
            .attribute("name")
            .ok_or_else(|| anyhow::anyhow!("Passive joint missing name"))?
            .to_string();

        Ok(PassiveJoint { name })
    }

    fn parse_link_sphere_approximation(elem: roxmltree::Node) -> Result<LinkSphereApproximation> {
        let link = elem
            .attribute("link")
            .ok_or_else(|| anyhow::anyhow!("Link sphere approximation missing link"))?
            .to_string();

        let mut spheres = Vec::new();

        for child in elem.children().filter(|n| n.is_element()) {
            if child.tag_name().name() == "sphere" {
                let center_str = child.attribute("center").unwrap_or("0 0 0");
                let center_parts: Vec<f64> = center_str
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();

                let center = if center_parts.len() >= 3 {
                    [center_parts[0], center_parts[1], center_parts[2]]
                } else {
                    [0.0, 0.0, 0.0]
                };

                let radius = child
                    .attribute("radius")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.01);

                spheres.push(CollisionSphere { center, radius });
            }
        }

        Ok(LinkSphereApproximation { link, spheres })
    }
}

/// SRDF utilities
impl SRDF {
    /// Get a planning group by name
    pub fn get_group(&self, name: &str) -> Option<&PlanningGroup> {
        self.groups.iter().find(|g| g.name == name)
    }

    /// Get all joint names in a group (including from chains)
    pub fn get_group_joints(&self, group_name: &str) -> Vec<String> {
        if let Some(group) = self.get_group(group_name) {
            let mut joints = group.joints.clone();

            // Add joints from subgroups
            for subgroup in &group.subgroups {
                joints.extend(self.get_group_joints(subgroup));
            }

            joints
        } else {
            Vec::new()
        }
    }

    /// Get a group state by name
    pub fn get_group_state(&self, group: &str, state: &str) -> Option<&GroupState> {
        self.group_states
            .iter()
            .find(|s| s.group == group && s.name == state)
    }

    /// Get all states for a group
    pub fn get_group_states(&self, group: &str) -> Vec<&GroupState> {
        self.group_states
            .iter()
            .filter(|s| s.group == group)
            .collect()
    }

    /// Check if collision between two links is disabled
    pub fn is_collision_disabled(&self, link1: &str, link2: &str) -> bool {
        self.disabled_collisions.iter().any(|dc| {
            (dc.link1 == link1 && dc.link2 == link2) || (dc.link1 == link2 && dc.link2 == link1)
        })
    }

    /// Get all disabled collision pairs as a set for fast lookup
    pub fn get_disabled_collision_set(&self) -> HashSet<(String, String)> {
        let mut set = HashSet::new();
        for dc in &self.disabled_collisions {
            // Add both orderings for symmetric lookup
            set.insert((dc.link1.clone(), dc.link2.clone()));
            set.insert((dc.link2.clone(), dc.link1.clone()));
        }
        set
    }

    /// Check if a joint is passive
    pub fn is_passive_joint(&self, joint: &str) -> bool {
        self.passive_joints.iter().any(|pj| pj.name == joint)
    }

    /// Get the virtual joint for a given child link
    pub fn get_virtual_joint_for_link(&self, link: &str) -> Option<&VirtualJoint> {
        self.virtual_joints.iter().find(|vj| vj.child_link == link)
    }

    /// Get end effector by name
    pub fn get_end_effector(&self, name: &str) -> Option<&EndEffector> {
        self.end_effectors.iter().find(|ee| ee.name == name)
    }

    /// Get sphere approximation for a link
    pub fn get_link_spheres(&self, link: &str) -> Option<&Vec<CollisionSphere>> {
        self.link_sphere_approximations
            .iter()
            .find(|lsa| lsa.link == link)
            .map(|lsa| &lsa.spheres)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_srdf() {
        let srdf_xml = r#"
        <robot name="test_robot">
            <group name="arm">
                <chain base_link="base_link" tip_link="end_effector"/>
                <joint name="joint1"/>
                <joint name="joint2"/>
            </group>
            <group_state name="home" group="arm">
                <joint name="joint1" value="0.0"/>
                <joint name="joint2" value="0.0"/>
            </group_state>
            <end_effector name="gripper" parent_link="end_effector" group="gripper_group"/>
            <virtual_joint name="world_joint" type="floating" parent_frame="world" child_link="base_link"/>
            <disable_collisions link1="link1" link2="link2" reason="adjacent"/>
        </robot>
        "#;

        let srdf = SRDFLoader::parse_string(srdf_xml).unwrap();

        assert_eq!(srdf.name, "test_robot");
        assert_eq!(srdf.groups.len(), 1);
        assert_eq!(srdf.groups[0].name, "arm");
        assert_eq!(srdf.groups[0].joints.len(), 2);
        assert_eq!(srdf.groups[0].chains.len(), 1);

        assert_eq!(srdf.group_states.len(), 1);
        assert_eq!(srdf.group_states[0].name, "home");
        assert_eq!(srdf.group_states[0].joint_values.len(), 2);

        assert_eq!(srdf.end_effectors.len(), 1);
        assert_eq!(srdf.end_effectors[0].name, "gripper");

        assert_eq!(srdf.virtual_joints.len(), 1);
        assert_eq!(
            srdf.virtual_joints[0].joint_type,
            VirtualJointType::Floating
        );

        assert_eq!(srdf.disabled_collisions.len(), 1);
        assert_eq!(
            srdf.disabled_collisions[0].reason,
            DisabledCollisionReason::Adjacent
        );
    }

    #[test]
    fn test_srdf_utilities() {
        let srdf_xml = r#"
        <robot name="test">
            <group name="arm">
                <joint name="j1"/>
                <joint name="j2"/>
            </group>
            <group_state name="home" group="arm">
                <joint name="j1" value="0"/>
                <joint name="j2" value="1.57"/>
            </group_state>
            <group_state name="extended" group="arm">
                <joint name="j1" value="1.0"/>
                <joint name="j2" value="0.5"/>
            </group_state>
            <disable_collisions link1="l1" link2="l2" reason="adjacent"/>
            <passive_joint name="wheel"/>
        </robot>
        "#;

        let srdf = SRDFLoader::parse_string(srdf_xml).unwrap();

        // Test get_group
        assert!(srdf.get_group("arm").is_some());
        assert!(srdf.get_group("nonexistent").is_none());

        // Test get_group_joints
        let joints = srdf.get_group_joints("arm");
        assert_eq!(joints.len(), 2);
        assert!(joints.contains(&"j1".to_string()));

        // Test get_group_state
        let state = srdf.get_group_state("arm", "home").unwrap();
        assert_eq!(state.joint_values.get("j2"), Some(&1.57));

        // Test get_group_states
        let states = srdf.get_group_states("arm");
        assert_eq!(states.len(), 2);

        // Test collision disabled
        assert!(srdf.is_collision_disabled("l1", "l2"));
        assert!(srdf.is_collision_disabled("l2", "l1")); // Symmetric
        assert!(!srdf.is_collision_disabled("l1", "l3"));

        // Test passive joint
        assert!(srdf.is_passive_joint("wheel"));
        assert!(!srdf.is_passive_joint("j1"));
    }

    #[test]
    fn test_virtual_joint_types() {
        assert_eq!(VirtualJointType::from("fixed"), VirtualJointType::Fixed);
        assert_eq!(
            VirtualJointType::from("floating"),
            VirtualJointType::Floating
        );
        assert_eq!(VirtualJointType::from("planar"), VirtualJointType::Planar);
        assert_eq!(
            VirtualJointType::from("FLOATING"),
            VirtualJointType::Floating
        );
        assert_eq!(VirtualJointType::from("unknown"), VirtualJointType::Fixed);
    }

    #[test]
    fn test_disabled_collision_reasons() {
        assert_eq!(
            DisabledCollisionReason::from("adjacent"),
            DisabledCollisionReason::Adjacent
        );
        assert_eq!(
            DisabledCollisionReason::from("default"),
            DisabledCollisionReason::Default
        );
        assert_eq!(
            DisabledCollisionReason::from("never"),
            DisabledCollisionReason::Never
        );
        assert_eq!(
            DisabledCollisionReason::from("user"),
            DisabledCollisionReason::User
        );
        assert_eq!(
            DisabledCollisionReason::from("other"),
            DisabledCollisionReason::Unknown
        );
    }
}
