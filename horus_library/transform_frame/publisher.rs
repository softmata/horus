//! TransformFrame SHM Publisher — Broadcasts frames to shared memory topics
//!
//! Bridges the gap between process-local `TransformFrame` storage and
//! inter-process visibility via HORUS shared-memory topics.
//!
//! # Problem
//!
//! `TransformFrame` stores all transform data in process-local memory.
//! When a robot process publishes TF frames (world -> base_link -> camera_link),
//! the `horus frame list/echo/tree` CLI commands — running in a separate
//! process — cannot see them because there is no shared memory backing.
//!
//! # Solution
//!
//! `TransformFramePublisher` snapshots the frame tree from a `TransformFrame`
//! instance and publishes `TFMessage` batches to the standard `"tf"` and
//! `"tf_static"` SHM topics. The CLI (and any other process) can then read
//! those topics to discover and monitor the frame tree.
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_library::transform_frame::{TransformFrame, TransformFramePublisher};
//!
//! let tf = TransformFrame::new();
//! tf.register_static_frame("world", None, &Transform::identity())?;
//! tf.register_static_frame("base_link", Some("world"), &Transform::xyz(1.0, 0.0, 0.0))?;
//!
//! // One-shot publish (e.g., in your tick loop)
//! let publisher = TransformFramePublisher::new(&tf)?;
//! publisher.publish()?;
//!
//! // Or run as a background thread at 10 Hz
//! let handle = TransformFramePublisher::spawn(&tf, 10.0)?;
//! // ... later ...
//! handle.stop();
//! ```

use horus_core::communication::Topic;
use horus_core::error::HorusResult;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use super::messages::{TFMessage, TransformStamped, MAX_TRANSFORMS_PER_MESSAGE};
use super::transform::Transform;
use super::TransformFrame;

/// Standard topic name for dynamic transforms
const TF_TOPIC: &str = "tf";
/// Standard topic name for static transforms
const TF_STATIC_TOPIC: &str = "tf_static";

/// Publishes `TransformFrame` data to SHM topics for cross-process visibility.
///
/// Creates `Topic<TFMessage>` handles for `"tf"` and `"tf_static"` and provides
/// methods to snapshot the current frame tree and publish it.
pub struct TransformFramePublisher {
    tf: TransformFrame,
    tf_topic: Topic<TFMessage>,
    tf_static_topic: Topic<TFMessage>,
}

impl TransformFramePublisher {
    /// Create a new publisher for the given `TransformFrame`.
    ///
    /// Opens (or creates) the `"tf"` and `"tf_static"` SHM topics.
    pub fn new(tf: &TransformFrame) -> HorusResult<Self> {
        let tf_topic = Topic::<TFMessage>::new(TF_TOPIC)?;
        let tf_static_topic = Topic::<TFMessage>::new(TF_STATIC_TOPIC)?;
        Ok(Self {
            tf: tf.clone(),
            tf_topic,
            tf_static_topic,
        })
    }

    /// Publish a snapshot of the current frame tree to SHM.
    ///
    /// Collects all registered frames and their transforms, then sends them
    /// as `TFMessage` batches on the appropriate topic (`"tf"` for dynamic
    /// frames, `"tf_static"` for static frames).
    ///
    /// Call this periodically (e.g., every tick) to keep the SHM topics
    /// up-to-date for external consumers like the CLI.
    pub fn publish(&self) -> HorusResult<()> {
        let mut dynamic_transforms = Vec::new();
        let mut static_transforms = Vec::new();

        let frames = self.tf.all_frames();
        for name in &frames {
            let Some(id) = self.tf.frame_id(name) else {
                continue;
            };
            let parent_name = match self.tf.parent(name) {
                Some(p) => p,
                None => continue, // Root frame with no parent — nothing to publish
            };

            let is_static = self.tf.core().is_static(id);

            // Read the latest transform entry from the core storage
            let entry = self.tf.core().read_latest(id);
            let (transform, timestamp_ns) = match entry {
                Some(e) => (e.transform, e.timestamp_ns),
                None => {
                    // Frame registered but never updated — publish identity
                    (Transform::identity(), 0)
                }
            };

            let stamped = TransformStamped::new(&parent_name, name, timestamp_ns, transform);

            if is_static {
                static_transforms.push(stamped);
            } else {
                dynamic_transforms.push(stamped);
            }
        }

        // Publish dynamic transforms in batches
        for chunk in dynamic_transforms.chunks(MAX_TRANSFORMS_PER_MESSAGE) {
            let msg = TFMessage::from_vec(chunk.to_vec());
            self.tf_topic.send(msg);
        }

        // Publish static transforms in batches
        for chunk in static_transforms.chunks(MAX_TRANSFORMS_PER_MESSAGE) {
            let msg = TFMessage::from_vec(chunk.to_vec());
            self.tf_static_topic.send(msg);
        }

        Ok(())
    }

    /// Publish only dynamic frames.
    ///
    /// Use this in your tick loop if static frames are published separately
    /// (e.g., once at startup via `publish_static()`).
    pub fn publish_dynamic(&self) -> HorusResult<()> {
        let mut transforms = Vec::new();

        let frames = self.tf.all_frames();
        for name in &frames {
            let Some(id) = self.tf.frame_id(name) else {
                continue;
            };
            if self.tf.core().is_static(id) {
                continue;
            }
            let parent_name = match self.tf.parent(name) {
                Some(p) => p,
                None => continue,
            };

            let entry = self.tf.core().read_latest(id);
            let (transform, timestamp_ns) = match entry {
                Some(e) => (e.transform, e.timestamp_ns),
                None => (Transform::identity(), 0),
            };

            let stamped = TransformStamped::new(&parent_name, name, timestamp_ns, transform);
            transforms.push(stamped);
        }

        for chunk in transforms.chunks(MAX_TRANSFORMS_PER_MESSAGE) {
            let msg = TFMessage::from_vec(chunk.to_vec());
            self.tf_topic.send(msg);
        }

        Ok(())
    }

    /// Publish only static frames.
    ///
    /// Call this once at startup or whenever static frames change.
    /// Static transforms are published on the `"tf_static"` topic which
    /// the CLI reads via `read_latest()` (latched semantics).
    pub fn publish_static(&self) -> HorusResult<()> {
        let mut transforms = Vec::new();

        let frames = self.tf.all_frames();
        for name in &frames {
            let Some(id) = self.tf.frame_id(name) else {
                continue;
            };
            if !self.tf.core().is_static(id) {
                continue;
            }
            let parent_name = match self.tf.parent(name) {
                Some(p) => p,
                None => continue,
            };

            let entry = self.tf.core().read_latest(id);
            let (transform, timestamp_ns) = match entry {
                Some(e) => (e.transform, e.timestamp_ns),
                None => (Transform::identity(), 0),
            };

            let stamped = TransformStamped::new(&parent_name, name, timestamp_ns, transform);
            transforms.push(stamped);
        }

        for chunk in transforms.chunks(MAX_TRANSFORMS_PER_MESSAGE) {
            let msg = TFMessage::from_vec(chunk.to_vec());
            self.tf_static_topic.send(msg);
        }

        Ok(())
    }

    /// Spawn a background thread that publishes the frame tree at the given rate.
    ///
    /// Returns a handle that can be used to stop the background thread.
    /// The thread publishes static frames once on startup, then publishes
    /// dynamic frames at the configured rate.
    pub fn spawn(tf: &TransformFrame, rate_hz: f64) -> HorusResult<TransformFramePublisherHandle> {
        let publisher = Self::new(tf)?;

        // Publish static frames once at startup
        publisher.publish_static()?;

        let stop = Arc::new(AtomicBool::new(false));
        let stop_clone = stop.clone();

        let interval = if rate_hz > 0.0 {
            std::time::Duration::from_secs_f64(1.0 / rate_hz)
        } else {
            std::time::Duration::from_millis(100)
        };

        let thread = std::thread::Builder::new()
            .name("tf-publisher".into())
            .spawn(move || {
                while !stop_clone.load(Ordering::Relaxed) {
                    // Publish all frames (dynamic get new data, static are latched)
                    let _ = publisher.publish();
                    std::thread::sleep(interval);
                }
            })
            .map_err(|e| {
                horus_core::error::HorusError::Communication(
                    format!("Failed to spawn tf-publisher thread: {}", e).into(),
                )
            })?;

        Ok(TransformFramePublisherHandle {
            stop,
            thread: Some(thread),
        })
    }
}

/// Handle for a background `TransformFramePublisher` thread.
///
/// The publisher thread stops when this handle is dropped or `stop()` is called.
pub struct TransformFramePublisherHandle {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl TransformFramePublisherHandle {
    /// Signal the publisher thread to stop.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Check if the publisher thread is still running.
    pub fn is_running(&self) -> bool {
        self.thread
            .as_ref()
            .map(|t| !t.is_finished())
            .unwrap_or(false)
    }
}

impl Drop for TransformFramePublisherHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_publisher_creation() {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_static_frame(
            "base_link",
            Some("world"),
            &Transform::from_translation([1.0, 0.0, 0.0]),
        )
        .unwrap();

        let publisher = TransformFramePublisher::new(&tf);
        assert!(publisher.is_ok());
    }

    #[test]
    fn test_publish_roundtrip() {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_static_frame(
            "base_link",
            Some("world"),
            &Transform::from_translation([1.0, 0.0, 0.0]),
        )
        .unwrap();
        tf.register_frame("camera", Some("base_link")).unwrap();
        tf.update_transform(
            "camera",
            &Transform::from_translation([0.0, 0.0, 0.5]),
            1000,
        )
        .unwrap();

        let publisher = TransformFramePublisher::new(&tf).unwrap();
        assert!(publisher.publish().is_ok());

        // Read back from the SHM topics
        let static_topic = Topic::<TFMessage>::new(TF_STATIC_TOPIC).unwrap();
        if let Some(msg) = static_topic.read_latest() {
            assert!(msg.len() > 0);
            let names: Vec<String> = msg.iter().map(|t| t.child_frame_id()).collect();
            assert!(names.contains(&"base_link".to_string()));
        }

        let dynamic_topic = Topic::<TFMessage>::new(TF_TOPIC).unwrap();
        if let Some(msg) = dynamic_topic.read_latest() {
            assert!(msg.len() > 0);
            let names: Vec<String> = msg.iter().map(|t| t.child_frame_id()).collect();
            assert!(names.contains(&"camera".to_string()));
        }
    }

    #[test]
    fn test_publish_static_only() {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_static_frame(
            "sensor",
            Some("world"),
            &Transform::from_translation([0.5, 0.0, 0.2]),
        )
        .unwrap();

        let publisher = TransformFramePublisher::new(&tf).unwrap();
        assert!(publisher.publish_static().is_ok());
    }

    #[test]
    fn test_publish_dynamic_only() {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_frame("robot", Some("world")).unwrap();
        tf.update_transform(
            "robot",
            &Transform::from_translation([2.0, 1.0, 0.0]),
            5000,
        )
        .unwrap();

        let publisher = TransformFramePublisher::new(&tf).unwrap();
        assert!(publisher.publish_dynamic().is_ok());
    }

    #[test]
    fn test_spawn_and_stop() {
        let tf = TransformFrame::new();
        tf.register_frame("world", None).unwrap();
        tf.register_static_frame(
            "base_link",
            Some("world"),
            &Transform::from_translation([1.0, 0.0, 0.0]),
        )
        .unwrap();

        let handle = TransformFramePublisher::spawn(&tf, 10.0).unwrap();
        assert!(handle.is_running());
        handle.stop();
        std::thread::sleep(std::time::Duration::from_millis(200));
        assert!(!handle.is_running());
    }

    #[test]
    fn test_empty_tree_publish() {
        let tf = TransformFrame::new();
        let publisher = TransformFramePublisher::new(&tf).unwrap();
        // Publishing an empty tree should succeed without error
        assert!(publisher.publish().is_ok());
    }
}
