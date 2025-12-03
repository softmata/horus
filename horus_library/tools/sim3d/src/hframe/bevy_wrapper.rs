//! Bevy-compatible wrapper for HFrame
//!
//! Provides Bevy Resource and conversion utilities for HFrame integration.

use bevy::prelude::*;
use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform as HFrameTransform};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use std::collections::HashMap;

/// Bevy Resource wrapper for HFrame
///
/// This provides the same API as the old TFTree but uses HFrame internally
/// for high-performance lock-free transforms.
#[derive(Resource)]
pub struct HFrameResource {
    inner: HFrame,
    /// Cache of frame children for tree traversal
    children_cache: HashMap<String, Vec<String>>,
}

impl Default for HFrameResource {
    fn default() -> Self {
        Self::new()
    }
}

impl HFrameResource {
    /// Create a new HFrame resource with default configuration
    pub fn new() -> Self {
        let hf = HFrame::new();
        // Register world frame as root
        hf.register_frame("world", None).ok();
        Self {
            inner: hf,
            children_cache: HashMap::new(),
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: HFrameConfig) -> Self {
        let hf = HFrame::with_config(config);
        hf.register_frame("world", None).ok();
        Self {
            inner: hf,
            children_cache: HashMap::new(),
        }
    }

    /// Create with root frame name
    pub fn with_root(root: impl Into<String>) -> Self {
        let hf = HFrame::new();
        let root = root.into();
        hf.register_frame(&root, None).ok();
        Self {
            inner: hf,
            children_cache: HashMap::new(),
        }
    }

    /// Register a new frame
    pub fn add_frame(
        &mut self,
        name: impl Into<String>,
        parent: impl Into<String>,
        transform: Isometry3<f32>,
    ) -> Result<(), String> {
        let name = name.into();
        let parent = parent.into();

        // Convert nalgebra Isometry3<f32> to HFrame Transform
        let hf_transform = isometry_to_hframe(&transform);

        // Register the frame
        self.inner
            .register_frame(&name, Some(&parent))
            .map_err(|e| e.to_string())?;

        // Update transform
        self.inner
            .update_transform(&name, &hf_transform, timestamp_now())
            .map_err(|e| e.to_string())?;

        // Update children cache
        self.children_cache.entry(parent).or_default().push(name);

        Ok(())
    }

    /// Update a frame's transform
    pub fn update_frame(&mut self, name: &str, transform: Isometry3<f32>) -> Result<(), String> {
        let hf_transform = isometry_to_hframe(&transform);
        self.inner
            .update_transform(name, &hf_transform, timestamp_now())
            .map_err(|e| e.to_string())
    }

    /// Get transform from source frame to target frame
    pub fn get_transform(&self, from: &str, to: &str) -> Result<Isometry3<f32>, String> {
        let hf_transform = self.inner.tf(from, to).map_err(|e| e.to_string())?;
        Ok(hframe_to_isometry(&hf_transform))
    }

    /// Alias for get_transform
    pub fn lookup_transform(&self, from: &str, to: &str) -> Result<Isometry3<f32>, String> {
        self.get_transform(from, to)
    }

    /// Check if a frame exists
    pub fn has_frame(&self, name: &str) -> bool {
        self.inner.has_frame(name)
    }

    /// Get all frame names
    pub fn get_all_frames(&self) -> Vec<String> {
        self.inner.all_frames()
    }

    /// Get frame count
    pub fn frame_count(&self) -> usize {
        self.inner.frame_count()
    }

    /// Get parent of a frame
    pub fn get_parent(&self, name: &str) -> Option<String> {
        self.inner.parent(name)
    }

    /// Get children of a frame
    pub fn get_children(&self, parent: &str) -> Vec<String> {
        self.children_cache.get(parent).cloned().unwrap_or_default()
    }

    /// Get direct access to inner HFrame
    pub fn inner(&self) -> &HFrame {
        &self.inner
    }

    /// Get mutable access to inner HFrame
    pub fn inner_mut(&mut self) -> &mut HFrame {
        &mut self.inner
    }

    /// Check if transform is possible between two frames
    pub fn can_transform(&self, from: &str, to: &str) -> bool {
        self.inner.can_transform(from, to)
    }

    /// Backwards-compatible frames accessor
    /// Allows code like hframe_tree.frames.contains_key() and hframe_tree.frames.len()
    pub fn frames(&self) -> FramesAccessor<'_> {
        FramesAccessor { inner: self }
    }

    /// Transform a point from one frame to another
    pub fn transform_point(&self, from: &str, to: &str, point: Vec3) -> Result<Vec3, String> {
        let point_f64 = [point.x as f64, point.y as f64, point.z as f64];
        let result = self
            .inner
            .transform_point(from, to, point_f64)
            .map_err(|e| e.to_string())?;
        Ok(Vec3::new(
            result[0] as f32,
            result[1] as f32,
            result[2] as f32,
        ))
    }

    /// Create from URDF robot description
    pub fn from_urdf(urdf: &urdf_rs::Robot) -> Self {
        let mut hf = Self::with_root("world");

        // Add base link
        if let Some(base_link) = urdf.links.first() {
            hf.add_frame(&base_link.name, "world", Isometry3::identity())
                .ok();

            // Add frames from joints
            for joint in &urdf.joints {
                if hf.has_frame(&joint.parent.link) {
                    let transform = urdf_origin_to_isometry(&joint.origin);
                    hf.add_frame(&joint.child.link, &joint.parent.link, transform)
                        .ok();
                }
            }
        }

        hf
    }
}

/// Dummy frames accessor for backwards compatibility
/// This wraps HFrameResource to provide .frames.len() and .frames.contains_key() access
pub struct FramesAccessor<'a> {
    inner: &'a HFrameResource,
}

impl FramesAccessor<'_> {
    pub fn len(&self) -> usize {
        self.inner.frame_count()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.frame_count() == 0
    }

    pub fn contains_key(&self, key: &str) -> bool {
        self.inner.has_frame(key)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, FrameInfo)> + '_ {
        self.inner.get_all_frames().into_iter().map(|name| {
            // Return dummy frame info - just enough for iteration
            let info = FrameInfo {
                name: name.clone(),
                transform: Isometry3::identity(),
            };
            // We need to leak the string to get a &String - this is a workaround
            // In practice, the old code iterated over frames, so we provide compatible iteration
            let leaked: &'static String = Box::leak(Box::new(name));
            (leaked, info)
        })
    }
}

/// Minimal frame info for iteration compatibility
pub struct FrameInfo {
    pub name: String,
    pub transform: Isometry3<f32>,
}

// Type alias for convenience
pub type BevyHFrame = HFrameResource;

// Primary type alias
pub type HFrameTree = HFrameResource;

/// Wrapper for HFrame Transform with Bevy compatibility
#[derive(Clone, Copy, Debug)]
pub struct BevyTransform {
    pub inner: HFrameTransform,
}

impl BevyTransform {
    pub fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        Self {
            inner: HFrameTransform::new(translation, rotation),
        }
    }

    pub fn identity() -> Self {
        Self {
            inner: HFrameTransform::identity(),
        }
    }

    pub fn from_translation(translation: [f64; 3]) -> Self {
        Self {
            inner: HFrameTransform::from_translation(translation),
        }
    }

    pub fn from_bevy(bevy_transform: &Transform) -> Self {
        Self {
            inner: bevy_transform_to_hframe(bevy_transform),
        }
    }

    pub fn to_bevy(&self) -> Transform {
        hframe_to_bevy_transform(&self.inner)
    }

    pub fn translation(&self) -> Vec3 {
        Vec3::new(
            self.inner.translation[0] as f32,
            self.inner.translation[1] as f32,
            self.inner.translation[2] as f32,
        )
    }

    pub fn rotation(&self) -> Quat {
        Quat::from_xyzw(
            self.inner.rotation[0] as f32,
            self.inner.rotation[1] as f32,
            self.inner.rotation[2] as f32,
            self.inner.rotation[3] as f32,
        )
    }
}

/// Convert nalgebra Isometry3<f32> to HFrame Transform
pub fn isometry_to_hframe(iso: &Isometry3<f32>) -> HFrameTransform {
    HFrameTransform::new(
        [
            iso.translation.x as f64,
            iso.translation.y as f64,
            iso.translation.z as f64,
        ],
        [
            iso.rotation.i as f64,
            iso.rotation.j as f64,
            iso.rotation.k as f64,
            iso.rotation.w as f64,
        ],
    )
}

/// Convert HFrame Transform to nalgebra Isometry3<f32>
pub fn hframe_to_isometry(tf: &HFrameTransform) -> Isometry3<f32> {
    let translation = Translation3::new(
        tf.translation[0] as f32,
        tf.translation[1] as f32,
        tf.translation[2] as f32,
    );
    let rotation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        tf.rotation[3] as f32, // w
        tf.rotation[0] as f32, // i
        tf.rotation[1] as f32, // j
        tf.rotation[2] as f32, // k
    ));
    Isometry3::from_parts(translation, rotation)
}

/// Convert HFrame Transform to Bevy Transform
pub fn hframe_to_bevy_transform(tf: &HFrameTransform) -> Transform {
    let translation = Vec3::new(
        tf.translation[0] as f32,
        tf.translation[1] as f32,
        tf.translation[2] as f32,
    );
    let rotation = Quat::from_xyzw(
        tf.rotation[0] as f32,
        tf.rotation[1] as f32,
        tf.rotation[2] as f32,
        tf.rotation[3] as f32,
    );
    Transform::from_translation(translation).with_rotation(rotation)
}

/// Convert Bevy Transform to HFrame Transform
pub fn bevy_transform_to_hframe(bevy_tf: &Transform) -> HFrameTransform {
    HFrameTransform::new(
        [
            bevy_tf.translation.x as f64,
            bevy_tf.translation.y as f64,
            bevy_tf.translation.z as f64,
        ],
        [
            bevy_tf.rotation.x as f64,
            bevy_tf.rotation.y as f64,
            bevy_tf.rotation.z as f64,
            bevy_tf.rotation.w as f64,
        ],
    )
}

/// Convert URDF origin to HFrame Transform
pub fn urdf_origin_to_hframe(origin: &urdf_rs::Pose) -> HFrameTransform {
    HFrameTransform::from_euler(
        [origin.xyz[0], origin.xyz[1], origin.xyz[2]],
        [origin.rpy[0], origin.rpy[1], origin.rpy[2]],
    )
}

/// Convert URDF origin to nalgebra Isometry3<f32> (for backwards compatibility)
pub fn urdf_origin_to_isometry(origin: &urdf_rs::Pose) -> Isometry3<f32> {
    let translation = Translation3::new(
        origin.xyz[0] as f32,
        origin.xyz[1] as f32,
        origin.xyz[2] as f32,
    );
    let rotation = UnitQuaternion::from_euler_angles(
        origin.rpy[0] as f32,
        origin.rpy[1] as f32,
        origin.rpy[2] as f32,
    );
    Isometry3::from_parts(translation, rotation)
}

/// HFrame visualizer configuration
#[derive(Resource, Clone)]
pub struct HFrameVisualizerConfig {
    pub enabled: bool,
    pub axis_length: f32,
    pub show_labels: bool,
    pub filter: HFrameFilter,
}

impl Default for HFrameVisualizerConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            axis_length: 0.3,
            show_labels: true,
            filter: HFrameFilter::All,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum HFrameFilter {
    All,
    OnlyRobot,
    OnlySensors,
    Custom(Vec<String>),
}

/// HFrame visualizer system
pub struct HFrameVisualizer;

impl HFrameVisualizer {
    /// Render HFrame frames as gizmos
    pub fn render_frames(
        mut gizmos: Gizmos,
        hframe: Res<HFrameResource>,
        config: Option<Res<HFrameVisualizerConfig>>,
    ) {
        let config = match config {
            Some(cfg) => cfg,
            None => return,
        };

        if !config.enabled {
            return;
        }

        for frame_name in hframe.get_all_frames() {
            if !Self::should_show_frame(&frame_name, &config.filter) {
                continue;
            }

            // Get transform from world
            if let Ok(iso) = hframe.get_transform(&frame_name, "world") {
                let pos = Vec3::new(iso.translation.x, iso.translation.y, iso.translation.z);
                let rot = Quat::from_xyzw(
                    iso.rotation.i,
                    iso.rotation.j,
                    iso.rotation.k,
                    iso.rotation.w,
                );

                let axis_len = config.axis_length;

                // X axis - Red
                gizmos.arrow(
                    pos,
                    pos + rot * Vec3::X * axis_len,
                    Color::srgb(1.0, 0.0, 0.0),
                );

                // Y axis - Green
                gizmos.arrow(
                    pos,
                    pos + rot * Vec3::Y * axis_len,
                    Color::srgb(0.0, 1.0, 0.0),
                );

                // Z axis - Blue
                gizmos.arrow(
                    pos,
                    pos + rot * Vec3::Z * axis_len,
                    Color::srgb(0.0, 0.0, 1.0),
                );
            }
        }
    }

    fn should_show_frame(name: &str, filter: &HFrameFilter) -> bool {
        match filter {
            HFrameFilter::All => true,
            HFrameFilter::OnlyRobot => !name.contains("sensor") && name != "world",
            HFrameFilter::OnlySensors => name.contains("sensor") || name.contains("camera"),
            HFrameFilter::Custom(names) => names.contains(&name.to_string()),
        }
    }
}

/// Render function for HFrame visualization
pub fn render_hframe_frames(
    gizmos: Gizmos,
    hframe: Res<HFrameResource>,
    config: Option<Res<HFrameVisualizerConfig>>,
) {
    HFrameVisualizer::render_frames(gizmos, hframe, config);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hframe_resource() {
        let mut hf = HFrameResource::new();
        assert!(hf.has_frame("world"));

        let iso = Isometry3::identity();
        hf.add_frame("base_link", "world", iso).unwrap();
        assert!(hf.has_frame("base_link"));

        let tf = hf.get_transform("base_link", "world").unwrap();
        assert!((tf.translation.x).abs() < 1e-6);
    }

    #[test]
    fn test_isometry_conversion() {
        let iso =
            Isometry3::from_parts(Translation3::new(1.0, 2.0, 3.0), UnitQuaternion::identity());

        let hf_tf = isometry_to_hframe(&iso);
        let back = hframe_to_isometry(&hf_tf);

        assert!((iso.translation.x - back.translation.x).abs() < 1e-6);
        assert!((iso.translation.y - back.translation.y).abs() < 1e-6);
        assert!((iso.translation.z - back.translation.z).abs() < 1e-6);
    }

    #[test]
    fn test_bevy_transform_conversion() {
        let bevy_tf = Transform::from_translation(Vec3::new(1.0, 2.0, 3.0));
        let hf_tf = bevy_transform_to_hframe(&bevy_tf);
        let back = hframe_to_bevy_transform(&hf_tf);

        assert!((bevy_tf.translation.x - back.translation.x).abs() < 1e-6);
        assert!((bevy_tf.translation.y - back.translation.y).abs() < 1e-6);
        assert!((bevy_tf.translation.z - back.translation.z).abs() < 1e-6);
    }
}
