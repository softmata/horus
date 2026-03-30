# Camera Perception — Object Detection & Tracking Pipeline

The #1 computer vision workflow in robotics: camera → detection → tracking.

## Architecture

```
CameraSimulator (30Hz) → ObjectDetector (10Hz) → ObjectTracker (10Hz) → Logger (2Hz)
     camera.image              detections            tracked_objects
```

## What This Demonstrates

- **Multi-rate CV pipeline**: Camera at 30Hz, detection at 10Hz (skips stale frames via `read_latest()`), logging at 2Hz
- **Object detection pattern**: Per-frame bounding box generation with confidence filtering
- **Object tracking (simplified SORT)**: IoU-based matching, velocity estimation, track lifecycle (create/update/delete)
- **Message types**: CameraFrame, ObjectDetection, TrackedObject — the standard perception message hierarchy

## ROS2 Equivalent

| ROS2 | Horus |
|------|-------|
| `image_transport` + `cv_bridge` | `Topic<CameraFrame>` + `read_latest()` |
| `darknet_ros` / `yolov5_ros` | ObjectDetector node with confidence filtering |
| `sort_tracker` / `deep_sort` | ObjectTracker with IoU matching |
| `rqt_image_view` | `horus topic echo tracked_objects` |

## Key Patterns

### Skip Stale Frames
```rust
// read_latest() returns the most recent frame, skipping any
// that accumulated while the detector was processing.
if let Some(frame) = self.frame_sub.read_latest() {
    // Process only the freshest frame
}
```

### Confidence Filtering
```rust
// Filter low-confidence detections at the tracker level
if det.confidence > 0.5 {
    detections.push(det);
}
```

### Track Lifecycle
```
New detection (no IoU match) → Create track with unique ID
Detection matches track (IoU > 0.3) → Update position + velocity
No match for 30 frames → Delete track (object left scene)
```

## Run

```bash
horus run main.rs
```

## Introspection

```bash
horus topic echo detections         # Watch raw detections
horus topic echo tracked_objects    # Watch tracked objects with IDs
horus topic hz camera.image         # Verify 30Hz camera rate
```
