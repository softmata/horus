"""
HORUS Perception - Object Detection, Point Clouds, and Tracking

Provides Python-friendly types for perception pipelines:
- Detection / DetectionList - 2D object detection results
- BoundingBox2D - Bounding boxes with IoU, center, area
- PointXYZ / PointXYZRGB - 3D point types
- PointCloudBuffer - Numpy-friendly point cloud
- Landmark - 2D pose keypoints
- TrackedObject - Multi-object tracking
- COCOPose - COCO keypoint index constants

Example:
    from horus.perception import Detection, DetectionList

    det = Detection("person", 0.95, x=100, y=200, width=50, height=80)
    print(det.bbox.area)  # 4000.0

    from horus.perception import PointCloudBuffer
    import numpy as np
    cloud = PointCloudBuffer.from_numpy(np.random.randn(1000, 3).astype("float32"))
"""

from horus._horus.perception import (
    BoundingBox2D,
    Detection,
    DetectionList,
    PointXYZ,
    PointXYZRGB,
    PointCloudBuffer,
    Landmark,
    TrackedObject,
    COCOPose,
)

__all__ = [
    "BoundingBox2D",
    "Detection",
    "DetectionList",
    "PointXYZ",
    "PointXYZRGB",
    "PointCloudBuffer",
    "Landmark",
    "TrackedObject",
    "COCOPose",
]
