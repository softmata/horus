"""
`horus.perception` is a pure re-export shim over `horus._horus.perception`.

Its measured coverage was 0.00% because nothing in the repo imported it. A
shim's only failure mode is precisely the one an import exercises: if the Rust
extension stops exporting a name, `import horus.perception` raises ImportError
at module scope and every downstream user breaks, with no test to catch it.
"""
import numpy as np
import pytest

import horus.perception as perception

EXPORTED = [
    "BoundingBox2D", "Detection", "DetectionList", "PointXYZ",
    "PointXYZRGB", "PointCloudBuffer", "Landmark", "TrackedObject", "COCOPose",
]


@pytest.mark.parametrize("name", EXPORTED)
def test_every_declared_export_actually_exists(name):
    """__all__ must not promise a name the extension does not provide."""
    assert hasattr(perception, name), f"horus.perception.{name} is missing"


def test_all_matches_the_module_namespace():
    assert sorted(perception.__all__) == sorted(EXPORTED)


class TestBoundingBox:
    def test_area_and_center(self):
        det = perception.Detection("person", 0.95, x=100, y=200, width=50, height=80)
        assert det.bbox.area == pytest.approx(4000.0)
        assert det.bbox.center_x == pytest.approx(125.0)
        assert det.bbox.center_y == pytest.approx(240.0)

    def test_iou_of_a_box_with_itself_is_one(self):
        a = perception.BoundingBox2D(0.0, 0.0, 10.0, 10.0)
        assert a.iou(a) == pytest.approx(1.0)

    def test_iou_of_disjoint_boxes_is_zero(self):
        a = perception.BoundingBox2D(0.0, 0.0, 10.0, 10.0)
        b = perception.BoundingBox2D(100.0, 100.0, 10.0, 10.0)
        assert a.iou(b) == pytest.approx(0.0)


class TestPointCloudBuffer:
    def test_from_numpy_round_trips(self):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="float32")
        cloud = perception.PointCloudBuffer.from_numpy(pts)
        assert len(cloud) == 2
        np.testing.assert_allclose(cloud.to_numpy(), pts)

    def test_float64_input_is_converted_not_reinterpreted(self):
        """
        The documented example passes float32. A float64 array must be
        value-converted; reinterpreting its bytes as float32 would silently
        corrupt every coordinate rather than fail.
        """
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="float64")
        cloud = perception.PointCloudBuffer.from_numpy(pts)
        assert len(cloud) == 2
        np.testing.assert_allclose(cloud.to_numpy(), pts)

    def test_empty_cloud_is_representable(self):
        cloud = perception.PointCloudBuffer.from_numpy(
            np.empty((0, 3), dtype="float32")
        )
        assert len(cloud) == 0
