/// Camera perception pipeline: capture → detect → track.
///
/// Demonstrates the #1 computer vision workflow in robotics:
/// a camera publishes images, a detector finds objects, and a
/// tracker maintains object identity across frames.
///
/// Nodes:
///   - CameraSimulator (30Hz): Publishes simulated camera frames
///   - ObjectDetector  (10Hz): Detects objects in frames → BoundingBox2D
///   - ObjectTracker   (10Hz): Tracks detections across frames → TrackedObject
///   - PerceptionLogger (2Hz): Logs perception results
///
/// Topics:
///   - camera/image (CompressedImage): Raw camera frames
///   - detections (Detection[]): Per-frame object detections
///   - tracked_objects (TrackedObject[]): Tracked objects with IDs
///
/// ROS2 equivalent: darknet_ros, yolov5_ros, image_proc, sort_tracker

use horus::prelude::*;
use horus::DurationExt;

message! {
    /// Simulated camera frame metadata
    CameraFrame {
        width: u32,
        height: u32,
        frame_id: u64,
        timestamp_ns: u64,
    }

    /// A detected object in a camera frame
    ObjectDetection {
        /// Bounding box: [x, y, width, height] in pixels
        x: f32,
        y: f32,
        w: f32,
        h: f32,
        /// Detection confidence (0.0 to 1.0)
        confidence: f32,
        /// Class label encoded as integer (0=person, 1=car, 2=box)
        class_id: u32,
        /// Source frame ID
        frame_id: u64,
    }

    /// A tracked object with persistent ID across frames
    TrackedObject {
        /// Unique track ID (persists across frames)
        track_id: u32,
        /// Current bounding box
        x: f32,
        y: f32,
        w: f32,
        h: f32,
        /// Estimated velocity (pixels/second)
        vx: f32,
        vy: f32,
        /// Frames since last detection match
        age: u32,
        /// Class label
        class_id: u32,
    }
}

// ============================================================================
// Camera Simulator — generates synthetic detectable scenes
// ============================================================================

struct CameraSimulator {
    frame_pub: Topic<CameraFrame>,
    frame_count: u64,
}

impl CameraSimulator {
    fn new() -> Result<Self> {
        Ok(Self {
            frame_pub: Topic::new("camera.image")?,
            frame_count: 0,
        })
    }
}

impl Node for CameraSimulator {
    fn name(&self) -> &str { "camera_simulator" }

    fn tick(&mut self) {
        self.frame_count += 1;
        self.frame_pub.send(CameraFrame {
            width: 640,
            height: 480,
            frame_id: self.frame_count,
            timestamp_ns: 0, // filled by transport
        });

        if self.frame_count % 30 == 0 {
            hlog!(info, "Camera: published {} frames", self.frame_count);
        }
    }
}

// ============================================================================
// Object Detector — simulates YOLO-like detection on each frame
// ============================================================================

struct ObjectDetector {
    frame_sub: Topic<CameraFrame>,
    det_pub: Topic<ObjectDetection>,
    frames_processed: u64,
}

impl ObjectDetector {
    fn new() -> Result<Self> {
        Ok(Self {
            frame_sub: Topic::new("camera.image")?,
            det_pub: Topic::new("detections")?,
            frames_processed: 0,
        })
    }
}

impl Node for ObjectDetector {
    fn name(&self) -> &str { "object_detector" }

    fn tick(&mut self) {
        // Process latest frame (skip stale ones — common CV pattern)
        if let Some(frame) = self.frame_sub.read_latest() {
            self.frames_processed += 1;

            // Simulate detections: objects move in a pattern
            let t = self.frames_processed as f32 * 0.1;

            // Object 1: person walking left-to-right
            let person_x = 100.0 + (t * 20.0) % 440.0;
            self.det_pub.send(ObjectDetection {
                x: person_x,
                y: 200.0,
                w: 60.0,
                h: 120.0,
                confidence: 0.92,
                class_id: 0, // person
                frame_id: frame.frame_id,
            });

            // Object 2: car moving (appears every 3rd frame)
            if self.frames_processed % 3 == 0 {
                let car_x = 400.0 - (t * 15.0) % 300.0;
                self.det_pub.send(ObjectDetection {
                    x: car_x,
                    y: 300.0,
                    w: 100.0,
                    h: 60.0,
                    confidence: 0.87,
                    class_id: 1, // car
                    frame_id: frame.frame_id,
                });
            }

            // Object 3: box (stationary, low confidence sometimes)
            let box_conf = 0.5 + 0.4 * (t * 0.5).sin();
            if box_conf > 0.6 {
                self.det_pub.send(ObjectDetection {
                    x: 500.0,
                    y: 350.0,
                    w: 40.0,
                    h: 40.0,
                    confidence: box_conf,
                    class_id: 2, // box
                    frame_id: frame.frame_id,
                });
            }

            if self.frames_processed % 10 == 0 {
                hlog!(info, "Detector: {} frames processed", self.frames_processed);
            }
        }
    }
}

// ============================================================================
// Object Tracker — maintains identity across frames (simplified SORT)
// ============================================================================

struct ObjectTrackerNode {
    det_sub: Topic<ObjectDetection>,
    track_pub: Topic<TrackedObject>,
    tracks: Vec<Track>,
    next_id: u32,
}

struct Track {
    id: u32,
    x: f32,
    y: f32,
    w: f32,
    h: f32,
    vx: f32,
    vy: f32,
    class_id: u32,
    age: u32,
    last_seen: u64,
}

impl ObjectTrackerNode {
    fn new() -> Result<Self> {
        Ok(Self {
            det_sub: Topic::new("detections")?,
            track_pub: Topic::new("tracked_objects")?,
            tracks: Vec::new(),
            next_id: 1,
        })
    }

    /// Simple IoU (Intersection over Union) for matching
    fn iou(ax: f32, ay: f32, aw: f32, ah: f32, bx: f32, by: f32, bw: f32, bh: f32) -> f32 {
        let x1 = ax.max(bx);
        let y1 = ay.max(by);
        let x2 = (ax + aw).min(bx + bw);
        let y2 = (ay + ah).min(by + bh);

        if x2 <= x1 || y2 <= y1 {
            return 0.0;
        }

        let intersection = (x2 - x1) * (y2 - y1);
        let union = aw * ah + bw * bh - intersection;

        if union > 0.0 { intersection / union } else { 0.0 }
    }
}

impl Node for ObjectTrackerNode {
    fn name(&self) -> &str { "object_tracker" }

    fn tick(&mut self) {
        // Collect all detections from this tick
        let mut detections = Vec::new();
        while let Some(det) = self.det_sub.recv() {
            if det.confidence > 0.5 {
                detections.push(det);
            }
        }

        // Age all tracks
        for track in &mut self.tracks {
            track.age += 1;
        }

        // Match detections to existing tracks (greedy IoU matching)
        let mut matched = vec![false; detections.len()];
        for track in &mut self.tracks {
            let mut best_iou = 0.3; // minimum IoU threshold
            let mut best_idx = None;

            for (i, det) in detections.iter().enumerate() {
                if matched[i] || det.class_id != track.class_id {
                    continue;
                }
                let iou = Self::iou(
                    track.x, track.y, track.w, track.h,
                    det.x, det.y, det.w, det.h,
                );
                if iou > best_iou {
                    best_iou = iou;
                    best_idx = Some(i);
                }
            }

            if let Some(idx) = best_idx {
                let det = &detections[idx];
                // Update track with new detection
                track.vx = det.x - track.x;
                track.vy = det.y - track.y;
                track.x = det.x;
                track.y = det.y;
                track.w = det.w;
                track.h = det.h;
                track.age = 0;
                track.last_seen = det.frame_id;
                matched[idx] = true;
            }
        }

        // Create new tracks for unmatched detections
        for (i, det) in detections.iter().enumerate() {
            if !matched[i] {
                self.tracks.push(Track {
                    id: self.next_id,
                    x: det.x,
                    y: det.y,
                    w: det.w,
                    h: det.h,
                    vx: 0.0,
                    vy: 0.0,
                    class_id: det.class_id,
                    age: 0,
                    last_seen: det.frame_id,
                });
                hlog!(info, "Tracker: new track #{} (class {})", self.next_id, det.class_id);
                self.next_id += 1;
            }
        }

        // Remove stale tracks (not seen for 30 frames)
        self.tracks.retain(|t| t.age < 30);

        // Publish active tracks
        for track in &self.tracks {
            self.track_pub.send(TrackedObject {
                track_id: track.id,
                x: track.x,
                y: track.y,
                w: track.w,
                h: track.h,
                vx: track.vx,
                vy: track.vy,
                age: track.age,
                class_id: track.class_id,
            });
        }
    }
}

// ============================================================================
// Perception Logger — summarizes tracking results
// ============================================================================

struct PerceptionLogger {
    track_sub: Topic<TrackedObject>,
}

impl PerceptionLogger {
    fn new() -> Result<Self> {
        Ok(Self {
            track_sub: Topic::new("tracked_objects")?,
        })
    }
}

impl Node for PerceptionLogger {
    fn name(&self) -> &str { "perception_logger" }

    fn tick(&mut self) {
        let mut tracks = Vec::new();
        while let Some(track) = self.track_sub.recv() {
            tracks.push(track);
        }

        if !tracks.is_empty() {
            let class_names = ["person", "car", "box"];
            for t in &tracks {
                let name = class_names.get(t.class_id as usize).unwrap_or(&"unknown");
                hlog!(info,
                    "Track #{}: {} at ({:.0},{:.0}) vel=({:.1},{:.1}) age={}",
                    t.track_id, name, t.x, t.y, t.vx, t.vy, t.age
                );
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_rate(30_u64.hz());

    scheduler.add(CameraSimulator::new()?).order(0).rate(30_u64.hz()).build()?;
    scheduler.add(ObjectDetector::new()?).order(10).rate(10_u64.hz()).build()?;
    scheduler.add(ObjectTrackerNode::new()?).order(20).rate(10_u64.hz()).build()?;
    scheduler.add(PerceptionLogger::new()?).order(100).rate(2_u64.hz()).build()?;

    hlog!(info, "Camera Perception Pipeline");
    hlog!(info, "  camera_simulator (30Hz) → object_detector (10Hz) → object_tracker (10Hz)");
    hlog!(info, "  Topics: camera.image, detections, tracked_objects");

    scheduler.run()
}
