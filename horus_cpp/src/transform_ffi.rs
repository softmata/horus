//! FFI wrappers for TransformFrame coordinate system.
//!
//! Wraps horus_library::TransformFrame for C++ access to the TF tree.

use horus_library::transform_frame::{Transform, TransformFrame, TransformFrameConfig};

/// Opaque TransformFrame wrapper.
pub struct FfiTransformFrame {
    inner: TransformFrame,
}

/// Create a new TransformFrame with default config (256 frames).
pub fn transform_frame_new() -> Box<FfiTransformFrame> {
    Box::new(FfiTransformFrame {
        inner: TransformFrame::new(),
    })
}

/// Create with custom max frames.
pub fn transform_frame_with_capacity(max_frames: usize) -> Box<FfiTransformFrame> {
    let config = TransformFrameConfig::rt_fixed(max_frames);
    Box::new(FfiTransformFrame {
        inner: TransformFrame::with_config(config),
    })
}

/// Register a frame in the TF tree.
/// `parent` is null/empty for root frames.
pub fn transform_frame_register(
    tf: &FfiTransformFrame,
    name: &str,
    parent: &str,
) -> Result<u32, String> {
    let parent_opt = if parent.is_empty() {
        None
    } else {
        Some(parent)
    };
    tf.inner
        .register_frame(name, parent_opt)
        .map_err(|e| e.to_string())
}

/// Update a transform between parent and child frame.
pub fn transform_frame_update(
    tf: &FfiTransformFrame,
    frame_name: &str,
    translation: [f64; 3],
    rotation: [f64; 4],
    timestamp_ns: u64,
) -> Result<(), String> {
    let transform = Transform {
        translation,
        rotation,
    };
    tf.inner
        .update_transform(frame_name, &transform, timestamp_ns)
        .map_err(|e| e.to_string())
}

/// Look up transform from source to destination frame.
/// Returns [tx, ty, tz, qx, qy, qz, qw] (7 doubles).
pub fn transform_frame_lookup(
    tf: &FfiTransformFrame,
    source: &str,
    target: &str,
) -> Result<[f64; 7], String> {
    let t = tf.inner.tf(source, target).map_err(|e| e.to_string())?;
    Ok([
        t.translation[0],
        t.translation[1],
        t.translation[2],
        t.rotation[0],
        t.rotation[1],
        t.rotation[2],
        t.rotation[3],
    ])
}

/// Look up transform at a specific timestamp.
pub fn transform_frame_lookup_at(
    tf: &FfiTransformFrame,
    source: &str,
    target: &str,
    timestamp_ns: u64,
) -> Result<[f64; 7], String> {
    let t = tf
        .inner
        .tf_at(source, target, timestamp_ns)
        .map_err(|e| e.to_string())?;
    Ok([
        t.translation[0],
        t.translation[1],
        t.translation[2],
        t.rotation[0],
        t.rotation[1],
        t.rotation[2],
        t.rotation[3],
    ])
}

/// Check if a transform path exists between two frames.
pub fn transform_frame_can_transform(tf: &FfiTransformFrame, source: &str, target: &str) -> bool {
    tf.inner.can_transform(source, target)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_transform_frame() {
        let _tf = transform_frame_new();
    }

    #[test]
    fn register_root_frame() {
        let tf = transform_frame_new();
        let id = transform_frame_register(&tf, "world", "");
        assert!(id.is_ok(), "register root: {:?}", id);
    }

    #[test]
    fn register_child_frame() {
        let tf = transform_frame_new();
        transform_frame_register(&tf, "world", "").unwrap();
        let id = transform_frame_register(&tf, "base_link", "world");
        assert!(id.is_ok(), "register child: {:?}", id);
    }

    #[test]
    fn update_and_lookup() {
        let tf = transform_frame_new();
        transform_frame_register(&tf, "world", "").unwrap();
        transform_frame_register(&tf, "base", "world").unwrap();

        transform_frame_update(
            &tf,
            "base",
            [1.0, 2.0, 3.0],      // translation
            [0.0, 0.0, 0.0, 1.0], // identity rotation
            1000,
        )
        .unwrap();

        let result = transform_frame_lookup(&tf, "base", "world");
        assert!(result.is_ok(), "lookup: {:?}", result);
        let vals = result.unwrap();
        assert!((vals[0] - 1.0).abs() < 1e-10, "tx={}", vals[0]);
        assert!((vals[1] - 2.0).abs() < 1e-10, "ty={}", vals[1]);
        assert!((vals[2] - 3.0).abs() < 1e-10, "tz={}", vals[2]);
    }

    #[test]
    fn can_transform_true() {
        let tf = transform_frame_new();
        transform_frame_register(&tf, "world", "").unwrap();
        transform_frame_register(&tf, "base", "world").unwrap();
        transform_frame_update(&tf, "base", [0.0; 3], [0.0, 0.0, 0.0, 1.0], 0).unwrap();
        assert!(transform_frame_can_transform(&tf, "base", "world"));
    }

    #[test]
    fn can_transform_false() {
        let tf = transform_frame_new();
        transform_frame_register(&tf, "world", "").unwrap();
        assert!(!transform_frame_can_transform(&tf, "world", "nonexistent"));
    }
}
