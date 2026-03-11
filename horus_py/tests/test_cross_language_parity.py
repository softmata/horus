"""
Cross-Language Parity Tests — Python↔Rust Message Roundtrip

Tests that every Python message type:
1. Constructs correctly (field values survive Python→Rust inner wrapping)
2. Round-trips through Topic pub/sub (send from Python, recv in Python via Rust backend)
3. Handles edge cases (empty vecs, zero values, max-size strings)

These tests verify the full interop pipeline added by the Python-Rust Message Parity roadmap.
"""

import pytest
import horus


# =============================================================================
# Helper: pub/sub roundtrip test
# =============================================================================

def roundtrip(msg_type, msg, field_checks, duration=1.0):
    """Send a message from one node, receive it in another, verify fields.

    Topic name must match the Rust type name for proper topic routing.
    """
    topic_name = msg_type.__name__
    received = []

    def publisher(node):
        if node.info.tick_count() == 0:
            node.send(topic_name, msg)
        elif node.info.tick_count() > 2:
            node.request_stop()

    def subscriber(node):
        m = node.get(topic_name)
        if m is not None:
            received.append(m)
            node.request_stop()
        elif node.info.tick_count() > 10:
            node.request_stop()

    pub = horus.Node(name="pub", pubs=topic_name, tick=publisher)
    sub = horus.Node(name="sub", subs=topic_name, tick=subscriber)
    horus.run(pub, sub, duration=duration)

    assert len(received) >= 1, f"No {msg_type.__name__} received"
    m = received[0]
    for field, expected in field_checks.items():
        actual = getattr(m, field)
        if isinstance(expected, float):
            assert abs(actual - expected) < 1e-5, f"{field}: {actual} != {expected}"
        else:
            assert actual == expected, f"{field}: {actual} != {expected}"


# =============================================================================
# Detection / Perception types
# =============================================================================

class TestDetectionTypes:
    def test_bounding_box_2d_construction(self):
        bb = horus.BoundingBox2D(x=10.0, y=20.0, width=100.0, height=50.0)
        assert bb.x == 10.0
        assert bb.y == 20.0
        assert bb.width == 100.0
        assert bb.height == 50.0

    def test_detection_construction(self):
        det = horus.Detection(class_name="person", confidence=0.95, x=10.0, y=20.0, width=100.0, height=200.0)
        assert det.class_name == "person"
        assert abs(det.confidence - 0.95) < 1e-5
        bbox = det.bbox
        assert bbox.x == 10.0

    def test_detection_pubsub(self):
        det = horus.Detection(class_name="car", confidence=0.88, x=5.0, y=10.0, width=50.0, height=30.0)
        roundtrip(horus.Detection, det, {"confidence": 0.88, "class_name": "car"})

    def test_detection_3d_construction(self):
        d3d = horus.Detection3D(class_name="box", confidence=0.75, cx=1.0, cy=2.0, cz=3.0, length=0.5, width=0.5, height=0.5)
        assert d3d.class_name == "box"
        assert abs(d3d.confidence - 0.75) < 1e-5

    def test_segmentation_mask_construction(self):
        mask = horus.SegmentationMask()
        assert mask.width == 0
        assert mask.height == 0

    def test_tracked_object_construction(self):
        to = horus.TrackedObject(track_id=42, class_id=1, confidence=0.9, x=1.0, y=2.0, width=3.0, height=4.0)
        assert to.track_id == 42
        assert to.class_id == 1
        assert abs(to.confidence - 0.9) < 1e-5

    def test_tracking_header_construction(self):
        th = horus.TrackingHeader(num_tracks=5, frame_id=100)
        assert th.num_tracks == 5
        assert th.frame_id == 100


class TestLandmarkTypes:
    def test_landmark_construction(self):
        lm = horus.Landmark(x=1.0, y=2.0, visibility=0.9, index=5)
        assert lm.x == 1.0
        assert lm.y == 2.0
        assert abs(lm.visibility - 0.9) < 1e-5
        assert lm.index == 5

    def test_landmark_3d_construction(self):
        lm = horus.Landmark3D(x=1.0, y=2.0, z=3.0, visibility=0.8, index=10)
        assert lm.x == 1.0
        assert lm.z == 3.0

    def test_landmark_array_construction(self):
        la = horus.LandmarkArray()
        assert la.num_landmarks == 17  # default


class TestPerceptionHelpers:
    def test_point_field_construction(self):
        pf = horus.PointField(name="x", offset=0, datatype=7, count=1)
        assert pf.name == "x"
        assert pf.offset == 0

    def test_plane_detection_construction(self):
        pd = horus.PlaneDetection()
        assert abs(pd.confidence - 0.5) < 1e-5  # default from PlaneDetection::new()


# =============================================================================
# Vision types
# =============================================================================

class TestVisionTypes:
    def test_compressed_image_construction(self):
        img = horus.CompressedImage(format="jpeg", data=[0xFF, 0xD8, 0xFF, 0xE0], width=640, height=480)
        assert img.format == "jpeg"
        assert img.data == bytes([0xFF, 0xD8, 0xFF, 0xE0])
        assert img.width == 640
        assert img.height == 480
        assert img.data_len() == 4

    def test_compressed_image_pubsub(self):
        img = horus.CompressedImage(format="png", data=[0x89, 0x50, 0x4E, 0x47], width=320, height=240)
        received = []

        def pub_fn(node):
            if node.info.tick_count() == 0:
                node.send("CompressedImage", img)
            elif node.info.tick_count() > 2:
                node.request_stop()

        def sub_fn(node):
            m = node.get("CompressedImage")
            if m is not None:
                received.append(m)
                node.request_stop()
            elif node.info.tick_count() > 10:
                node.request_stop()

        horus.run(
            horus.Node(name="p", pubs="CompressedImage", tick=pub_fn),
            horus.Node(name="s", subs="CompressedImage", tick=sub_fn),
            duration=1.0,
        )
        assert len(received) >= 1
        assert received[0].format == "png"
        assert received[0].width == 320
        assert received[0].data == bytes([0x89, 0x50, 0x4E, 0x47])

    def test_camera_info_construction(self):
        ci = horus.CameraInfo(width=640, height=480, fx=525.0, fy=525.0, cx=320.0, cy=240.0)
        assert ci.width == 640
        assert ci.height == 480
        fl = ci.focal_lengths()
        assert abs(fl[0] - 525.0) < 1e-5
        pp = ci.principal_point()
        assert abs(pp[0] - 320.0) < 1e-5

    def test_region_of_interest_construction(self):
        roi = horus.RegionOfInterest(x=10, y=20, width=100, height=80)
        assert roi.x_offset == 10
        assert roi.y_offset == 20
        assert roi.width == 100
        assert roi.area() == 8000
        assert roi.contains(50, 50) is True
        assert roi.contains(5, 5) is False

    def test_stereo_info_construction(self):
        left = horus.CameraInfo(width=640, height=480, fx=525.0, fy=525.0, cx=320.0, cy=240.0)
        right = horus.CameraInfo(width=640, height=480, fx=525.0, fy=525.0, cx=320.0, cy=240.0)
        si = horus.StereoInfo(left_camera=left, right_camera=right, baseline=0.12, depth_scale=1.0)
        assert abs(si.baseline - 0.12) < 1e-5
        assert si.left_camera.width == 640


# =============================================================================
# Force types (additional)
# =============================================================================

class TestForceTypes:
    def test_impedance_parameters_construction(self):
        ip = horus.ImpedanceParameters()
        assert ip.enabled is False
        ip.enabled = True
        assert ip.enabled is True
        assert len(ip.stiffness) == 6

    def test_impedance_parameters_staticmethods(self):
        comp = horus.ImpedanceParameters.compliant()
        stiff = horus.ImpedanceParameters.stiff()
        assert comp.stiffness[0] < stiff.stiffness[0]

    def test_haptic_feedback_construction(self):
        hf = horus.HapticFeedback(vibration_intensity=0.8, vibration_frequency=200.0, duration_seconds=0.5)
        assert abs(hf.vibration_intensity - 0.8) < 1e-5
        assert abs(hf.vibration_frequency - 200.0) < 1e-3
        assert hf.enabled is True


# =============================================================================
# Diagnostics types (additional)
# =============================================================================

class TestDiagnosticsTypes:
    def test_diagnostic_value_construction(self):
        dv = horus.DiagnosticValue(key="cpu_temp", value="72.5")
        assert dv.key == "cpu_temp"
        assert dv.value == "72.5"

    def test_diagnostic_value_typed(self):
        dv_int = horus.DiagnosticValue.int("count", 42)
        assert dv_int.key == "count"
        assert dv_int.value == "42"
        assert dv_int.value_type == 1

        dv_float = horus.DiagnosticValue.float("temp", 72.5)
        assert dv_float.value_type == 2

        dv_bool = horus.DiagnosticValue.boolean("ok", True)
        assert dv_bool.value == "true"
        assert dv_bool.value_type == 3

    def test_diagnostic_report_construction(self):
        dr = horus.DiagnosticReport(component="motor_driver", level=1)
        assert dr.component == "motor_driver"
        assert dr.level == 1
        assert dr.value_count == 0

        dr.add_value(horus.DiagnosticValue(key="voltage", value="24.1"))
        assert dr.value_count == 1
        vals = dr.get_values()
        assert len(vals) == 1
        assert vals[0].key == "voltage"

    def test_node_heartbeat_construction(self):
        nh = horus.NodeHeartbeat(state=1, health=0)
        assert nh.state == 1
        assert nh.health == 0
        nh.tick_count = 100
        assert nh.tick_count == 100

    def test_safety_status_construction(self):
        ss = horus.SafetyStatus()
        assert ss.enabled is True
        assert ss.estop_engaged is False
        assert ss.watchdog_ok is True
        assert ss.mode == 0

    def test_safety_status_pubsub(self):
        ss = horus.SafetyStatus()
        ss.estop_engaged = True
        ss.mode = 2
        roundtrip(horus.SafetyStatus, ss, {"estop_engaged": True, "mode": 2})


# =============================================================================
# Navigation types (additional)
# =============================================================================

class TestNavigationTypes:
    def test_waypoint_construction(self):
        wp = horus.Waypoint(x=1.0, y=2.0, theta=0.5)
        pose = wp.pose
        assert abs(pose.x - 1.0) < 1e-5
        assert abs(pose.y - 2.0) < 1e-5

    def test_nav_path_construction(self):
        path = horus.NavPath()
        assert path.waypoint_count == 0
        path.add_waypoint(horus.Waypoint(x=0.0, y=0.0))
        path.add_waypoint(horus.Waypoint(x=1.0, y=0.0))
        path.add_waypoint(horus.Waypoint(x=1.0, y=1.0))
        assert path.waypoint_count == 3
        wps = path.get_waypoints()
        assert len(wps) == 3
        assert abs(wps[1].pose.x - 1.0) < 1e-5

    def test_velocity_obstacle_construction(self):
        vo = horus.VelocityObstacle(px=1.0, py_=2.0, vx=0.5, vy=0.3, radius=0.5, obstacle_id=7)
        pos = vo.position
        assert abs(pos[0] - 1.0) < 1e-5
        assert abs(pos[1] - 2.0) < 1e-5
        assert vo.obstacle_id == 7

    def test_velocity_obstacles_construction(self):
        vos = horus.VelocityObstacles()
        assert vos.count == 0
        vos.add_obstacle(horus.VelocityObstacle(px=1.0, py_=2.0, radius=0.3, obstacle_id=1))
        vos.add_obstacle(horus.VelocityObstacle(px=3.0, py_=4.0, radius=0.5, obstacle_id=2))
        assert vos.count == 2
        obs = vos.get_obstacles()
        assert len(obs) == 2
        assert obs[1].obstacle_id == 2

    def test_occupancy_grid_construction(self):
        grid = horus.OccupancyGrid(width=10, height=10, resolution=0.1)
        assert grid.width == 10
        assert grid.height == 10
        assert abs(grid.resolution - 0.1) < 1e-5
        # Default data should be -1 (unknown)
        data = grid.data
        assert len(data) == 100
        assert data[0] == -1

    def test_occupancy_grid_operations(self):
        grid = horus.OccupancyGrid(width=10, height=10, resolution=0.1)
        grid.set_occupancy(5, 5, 100)  # Mark as occupied
        assert grid.occupancy(5, 5) == 100
        grid.set_occupancy(0, 0, 0)  # Mark as free
        assert grid.is_free(0.05, 0.05)

    def test_occupancy_grid_pubsub(self):
        grid = horus.OccupancyGrid(width=5, height=5, resolution=0.2)
        grid.set_occupancy(2, 2, 100)
        received = []

        def pub_fn(node):
            if node.info.tick_count() == 0:
                node.send("OccupancyGrid", grid)
            elif node.info.tick_count() > 2:
                node.request_stop()

        def sub_fn(node):
            m = node.get("OccupancyGrid")
            if m is not None:
                received.append(m)
                node.request_stop()
            elif node.info.tick_count() > 10:
                node.request_stop()

        horus.run(
            horus.Node(name="p", pubs="OccupancyGrid", tick=pub_fn),
            horus.Node(name="s", subs="OccupancyGrid", tick=sub_fn),
            duration=1.0,
        )
        assert len(received) >= 1
        assert received[0].width == 5
        assert received[0].occupancy(2, 2) == 100

    def test_cost_map_construction(self):
        grid = horus.OccupancyGrid(width=5, height=5, resolution=0.1)
        cm = horus.CostMap(grid=grid, inflation_radius=0.3)
        assert abs(cm.inflation_radius - 0.3) < 1e-5
        assert cm.occupancy_grid.width == 5


# =============================================================================
# Edge cases
# =============================================================================

class TestEdgeCases:
    def test_zero_size_image(self):
        img = horus.CompressedImage()
        assert img.width == 0
        assert img.height == 0
        assert img.data == b''

    def test_diagnostic_report_max_values(self):
        dr = horus.DiagnosticReport(component="test")
        for i in range(16):
            dr.add_value(horus.DiagnosticValue(key=f"k{i}", value=f"v{i}"))
        assert dr.value_count == 16
        with pytest.raises(OverflowError):
            dr.add_value(horus.DiagnosticValue(key="overflow", value="!"))

    def test_nav_path_max_waypoints(self):
        path = horus.NavPath()
        for i in range(256):
            path.add_waypoint(horus.Waypoint(x=float(i), y=0.0))
        assert path.waypoint_count == 256
        with pytest.raises(OverflowError):
            path.add_waypoint(horus.Waypoint(x=257.0, y=0.0))


# =============================================================================
# Cross-Language Integration: Python sends → Rust backend → Python receives
# =============================================================================

class TestCrossLanguagePythonSendsRustReceives:
    """Comprehensive roundtrip tests: Python creates message, sends through
    Rust shared-memory backend, another Python node receives it.
    Tests the Python→Rust extraction path (PyO3 inner field access)."""

    # --- Geometry types ---

    def test_twist_roundtrip(self):
        msg = horus.Twist(linear_x=1.5, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.3)
        roundtrip(horus.Twist, msg, {"linear_x": 1.5, "angular_z": 0.3})

    def test_vector3_roundtrip(self):
        msg = horus.Vector3(x=1.0, y=2.0, z=3.0)
        roundtrip(horus.Vector3, msg, {"x": 1.0, "y": 2.0, "z": 3.0})

    def test_point3_roundtrip(self):
        msg = horus.Point3(x=-1.5, y=0.5, z=10.0)
        roundtrip(horus.Point3, msg, {"x": -1.5, "y": 0.5, "z": 10.0})

    def test_quaternion_roundtrip(self):
        msg = horus.Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
        roundtrip(horus.Quaternion, msg, {"z": 0.707, "w": 0.707})

    def test_pose_stamped_roundtrip(self):
        msg = horus.PoseStamped(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        roundtrip(horus.PoseStamped, msg, {"x": 1.0, "y": 2.0, "z": 3.0, "qw": 1.0})

    def test_transform_stamped_roundtrip(self):
        msg = horus.TransformStamped(tx=1.0, ty=2.0, tz=3.0, rx=0.0, ry=0.0, rz=0.0, rw=1.0)
        roundtrip(horus.TransformStamped, msg, {"tx": 1.0, "ty": 2.0, "tz": 3.0, "rw": 1.0})

    def test_accel_roundtrip(self):
        msg = horus.Accel(linear_x=9.81, linear_y=0.0, linear_z=0.0, angular_x=0.1, angular_y=0.2, angular_z=0.3)
        roundtrip(horus.Accel, msg, {"linear_x": 9.81, "angular_z": 0.3})

    def test_accel_stamped_roundtrip(self):
        msg = horus.AccelStamped(linear_x=1.0, linear_y=2.0, linear_z=3.0)
        roundtrip(horus.AccelStamped, msg, {"linear_x": 1.0, "linear_y": 2.0})

    # --- Core robotics types ---

    def test_cmd_vel_roundtrip(self):
        msg = horus.CmdVel(linear=1.0, angular=0.5)
        roundtrip(horus.CmdVel, msg, {"linear": 1.0, "angular": 0.5})

    def test_pose2d_roundtrip(self):
        msg = horus.Pose2D(x=3.14, y=-1.0, theta=1.57)
        roundtrip(horus.Pose2D, msg, {"x": 3.14, "y": -1.0, "theta": 1.57})

    def test_pose3d_roundtrip(self):
        msg = horus.Pose3D(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        roundtrip(horus.Pose3D, msg, {"x": 1.0, "qw": 1.0})

    def test_imu_roundtrip(self):
        msg = horus.Imu(accel_x=0.0, accel_y=0.0, accel_z=9.81, gyro_x=0.1, gyro_y=0.2, gyro_z=0.3)
        roundtrip(horus.Imu, msg, {"accel_z": 9.81, "gyro_z": 0.3})

    def test_odometry_roundtrip(self):
        msg = horus.Odometry(x=5.0, y=3.0, theta=1.2, linear_velocity=0.5, angular_velocity=0.1)
        roundtrip(horus.Odometry, msg, {"x": 5.0, "theta": 1.2, "linear_velocity": 0.5})

    # --- Control types ---

    def test_motor_command_roundtrip(self):
        msg = horus.MotorCommand(motor_id=1, target=100.0, mode=1)
        roundtrip(horus.MotorCommand, msg, {"motor_id": 1, "target": 100.0, "mode": 1})

    def test_servo_command_roundtrip(self):
        msg = horus.ServoCommand(servo_id=2, position=90.0, speed=0.5)
        roundtrip(horus.ServoCommand, msg, {"servo_id": 2, "position": 90.0, "speed": 0.5})

    # --- Sensor types ---

    def test_battery_state_roundtrip(self):
        msg = horus.BatteryState(voltage=12.5, percentage=85.0, current=2.5, temperature=35.0)
        roundtrip(horus.BatteryState, msg, {"voltage": 12.5, "percentage": 85.0})

    def test_range_sensor_roundtrip(self):
        msg = horus.RangeSensor(range=2.5, min_range=0.0, max_range=10.0, field_of_view=0.5)
        roundtrip(horus.RangeSensor, msg, {"range": 2.5, "max_range": 10.0})

    def test_clock_roundtrip(self):
        msg = horus.Clock(clock_ns=1000000000, sim_speed=2.0, source=1)
        roundtrip(horus.Clock, msg, {"clock_ns": 1000000000, "sim_speed": 2.0, "source": 1})

    def test_time_reference_roundtrip(self):
        msg = horus.TimeReference(time_ref_ns=5000000000, source="gps", offset_ns=-1000)
        roundtrip(horus.TimeReference, msg, {"time_ref_ns": 5000000000, "source": "gps", "offset_ns": -1000})

    def test_temperature_roundtrip(self):
        msg = horus.Temperature(temperature=72.5, variance=0.1)
        roundtrip(horus.Temperature, msg, {"temperature": 72.5})

    # --- Diagnostics types ---

    def test_heartbeat_roundtrip(self):
        msg = horus.Heartbeat(node_name="test_node", node_id=42)
        roundtrip(horus.Heartbeat, msg, {"node_id": 42})

    def test_emergency_stop_roundtrip(self):
        msg = horus.EmergencyStop(engaged=True, reason="test")
        roundtrip(horus.EmergencyStop, msg, {"engaged": True})

    # --- Force types ---

    def test_wrench_stamped_roundtrip(self):
        msg = horus.WrenchStamped(fx=1.0, fy=2.0, fz=3.0, tx=0.1, ty=0.2, tz=0.3)
        roundtrip(horus.WrenchStamped, msg, {"fx": 1.0, "fy": 2.0, "fz": 3.0})

    def test_force_command_roundtrip(self):
        msg = horus.ForceCommand(fx=10.0, fy=0.0, fz=-5.0, timeout=1.0)
        roundtrip(horus.ForceCommand, msg, {"fx": 10.0, "fz": -5.0})

    def test_contact_info_roundtrip(self):
        msg = horus.ContactInfo(state=1, contact_force=15.5)
        roundtrip(horus.ContactInfo, msg, {"state": 1, "contact_force": 15.5})

    # --- Navigation types ---

    def test_nav_goal_roundtrip(self):
        msg = horus.NavGoal(x=10.0, y=20.0, theta=1.5)
        roundtrip(horus.NavGoal, msg, {"x": 10.0, "y": 20.0, "theta": 1.5})

    def test_goal_result_roundtrip(self):
        msg = horus.GoalResult(goal_id=1, status=3, progress=1.0)
        roundtrip(horus.GoalResult, msg, {"goal_id": 1, "status": 3, "progress": 1.0})

    # --- Input types ---

    def test_joystick_input_roundtrip(self):
        msg = horus.JoystickInput(joystick_id=1, element_id=5, value=0.75, pressed=True)
        roundtrip(horus.JoystickInput, msg, {"joystick_id": 1, "element_id": 5, "pressed": True})

    def test_keyboard_input_roundtrip(self):
        msg = horus.KeyboardInput(key_name="A", code=65, pressed=True, modifiers=0)
        roundtrip(horus.KeyboardInput, msg, {"code": 65, "pressed": True})

    # --- Detection types ---

    def test_bounding_box_3d_roundtrip(self):
        msg = horus.BoundingBox3D(cx=1.0, cy=2.0, cz=3.0, length=0.5, width=0.5, height=0.5)
        roundtrip(horus.BoundingBox3D, msg, {"cx": 1.0, "cy": 2.0, "cz": 3.0})

    def test_detection_3d_roundtrip(self):
        msg = horus.Detection3D(class_name="box", confidence=0.9, cx=1.0, cy=2.0, cz=3.0, length=1.0, width=1.0, height=1.0)
        roundtrip(horus.Detection3D, msg, {"confidence": 0.9, "class_name": "box"})

    def test_tracking_header_roundtrip(self):
        msg = horus.TrackingHeader(num_tracks=10, frame_id=500)
        roundtrip(horus.TrackingHeader, msg, {"num_tracks": 10, "frame_id": 500})

    # --- Diagnostics additional ---

    def test_node_heartbeat_roundtrip(self):
        msg = horus.NodeHeartbeat(state=1, health=0)
        msg.tick_count = 999
        roundtrip(horus.NodeHeartbeat, msg, {"state": 1, "tick_count": 999})

    def test_diagnostic_value_roundtrip(self):
        msg = horus.DiagnosticValue(key="cpu", value="72.5")
        roundtrip(horus.DiagnosticValue, msg, {"key": "cpu", "value": "72.5"})

    # --- Impedance/Haptic ---

    def test_impedance_parameters_roundtrip(self):
        msg = horus.ImpedanceParameters.stiff()
        msg.enabled = True
        roundtrip(horus.ImpedanceParameters, msg, {"enabled": True})

    def test_haptic_feedback_roundtrip(self):
        msg = horus.HapticFeedback(vibration_intensity=0.7, vibration_frequency=150.0, duration_seconds=0.3)
        roundtrip(horus.HapticFeedback, msg, {"vibration_intensity": 0.7, "enabled": True})

    # --- Waypoint/NavPath ---

    def test_waypoint_roundtrip(self):
        msg = horus.Waypoint(x=5.0, y=10.0, theta=0.5)
        roundtrip(horus.Waypoint, msg, {"time_from_start": 0.0})

    def test_velocity_obstacle_roundtrip(self):
        msg = horus.VelocityObstacle(px=1.0, py_=2.0, vx=0.5, vy=0.3, radius=0.5, obstacle_id=7)
        roundtrip(horus.VelocityObstacle, msg, {"obstacle_id": 7})


# =============================================================================
# Cross-Language Integration: Rust sends → SHM → Python receives
# =============================================================================

class TestCrossLanguageRustSendsPythonReceives:
    """Tests that messages written to shared memory by the Rust backend
    are correctly deserialized into Python objects. Since Topic.recv()
    calls Rust to read from SHM and wraps in Py::new(), this verifies
    the Rust→Python construction path.

    Each test sends from one Python node (triggering Rust SHM write)
    and receives in another (triggering Rust SHM read + PyO3 wrapping),
    verifying field-level fidelity across the boundary."""

    # --- Geometry: verify floating-point precision across boundary ---

    def test_twist_precision(self):
        msg = horus.Twist(linear_x=0.123456789, angular_z=-0.987654321)
        roundtrip(horus.Twist, msg, {"linear_x": 0.123456789, "angular_z": -0.987654321})

    def test_vector3_negative_values(self):
        msg = horus.Vector3(x=-999.999, y=-0.001, z=999999.0)
        roundtrip(horus.Vector3, msg, {"x": -999.999, "y": -0.001, "z": 999999.0})

    def test_quaternion_identity(self):
        msg = horus.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        roundtrip(horus.Quaternion, msg, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})

    def test_pose_with_covariance_roundtrip(self):
        msg = horus.PoseWithCovariance(x=1.0, y=2.0, z=3.0)
        roundtrip(horus.PoseWithCovariance, msg, {"x": 1.0, "y": 2.0, "z": 3.0})

    def test_twist_with_covariance_roundtrip(self):
        msg = horus.TwistWithCovariance(linear_x=0.5, angular_z=1.0)
        roundtrip(horus.TwistWithCovariance, msg, {"linear_x": 0.5, "angular_z": 1.0})

    # --- Sensor: verify Pod types with mixed field sizes ---

    def test_imu_full_fields(self):
        msg = horus.Imu(accel_x=0.01, accel_y=-0.02, accel_z=9.81, gyro_x=0.001, gyro_y=-0.002, gyro_z=0.003)
        roundtrip(horus.Imu, msg, {"accel_x": 0.01, "accel_y": -0.02, "accel_z": 9.81, "gyro_x": 0.001})

    def test_laser_scan_roundtrip(self):
        msg = horus.LaserScan(angle_min=-1.5, angle_max=1.5, range_min=0.125, range_max=30.0)
        roundtrip(horus.LaserScan, msg, {"angle_min": -1.5, "angle_max": 1.5})

    def test_joint_state_roundtrip(self):
        msg = horus.JointState(names=["j1", "j2"], positions=[1.0, 2.0])
        received = []

        def pub_fn(node):
            if node.info.tick_count() == 0:
                node.send("JointState", msg)
            elif node.info.tick_count() > 2:
                node.request_stop()

        def sub_fn(node):
            m = node.get("JointState")
            if m is not None:
                received.append(m)
                node.request_stop()
            elif node.info.tick_count() > 10:
                node.request_stop()

        horus.run(
            horus.Node(name="p", pubs="JointState", tick=pub_fn),
            horus.Node(name="s", subs="JointState", tick=sub_fn),
            duration=1.0,
        )
        assert len(received) >= 1
        assert len(received[0]) == 2
        assert received[0].names == ["j1", "j2"]
        assert abs(received[0].positions[0] - 1.0) < 1e-5

    def test_nav_sat_fix_roundtrip(self):
        msg = horus.NavSatFix(latitude=37.7749, longitude=-122.4194, altitude=10.0)
        roundtrip(horus.NavSatFix, msg, {"latitude": 37.7749, "longitude": -122.4194, "altitude": 10.0})

    def test_magnetic_field_roundtrip(self):
        msg = horus.MagneticField(x=0.25, y=-0.1, z=0.45)
        roundtrip(horus.MagneticField, msg, {"x": 0.25, "y": -0.1, "z": 0.45})

    def test_fluid_pressure_roundtrip(self):
        msg = horus.FluidPressure(pressure=101325.0, variance=10.0)
        roundtrip(horus.FluidPressure, msg, {"pressure": 101325.0})

    def test_illuminance_roundtrip(self):
        msg = horus.Illuminance(illuminance=500.0, variance=5.0)
        roundtrip(horus.Illuminance, msg, {"illuminance": 500.0})

    # --- Clock/Time: verify u64 and i64 fields ---

    def test_clock_paused_roundtrip(self):
        msg = horus.Clock(clock_ns=999999999, sim_speed=0.5, paused=True, source=2)
        roundtrip(horus.Clock, msg, {"clock_ns": 999999999, "sim_speed": 0.5, "paused": True, "source": 2})

    def test_time_reference_negative_offset(self):
        msg = horus.TimeReference(time_ref_ns=1000000, source="ptp", offset_ns=-50000)
        roundtrip(horus.TimeReference, msg, {"time_ref_ns": 1000000, "source": "ptp", "offset_ns": -50000})

    def test_time_reference_correct_timestamp_after_roundtrip(self):
        """Verify correct_timestamp method works on received message"""
        msg = horus.TimeReference(time_ref_ns=1000, source="ntp", offset_ns=500)
        topic_name = "TimeReference"
        received = []

        def pub_fn(node):
            if node.info.tick_count() == 0:
                node.send(topic_name, msg)
            elif node.info.tick_count() > 2:
                node.request_stop()

        def sub_fn(node):
            m = node.get(topic_name)
            if m is not None:
                received.append(m)
                node.request_stop()
            elif node.info.tick_count() > 10:
                node.request_stop()

        horus.run(
            horus.Node(name="p", pubs=topic_name, tick=pub_fn),
            horus.Node(name="s", subs=topic_name, tick=sub_fn),
            duration=1.0,
        )
        assert len(received) >= 1
        # offset=500 means local is 500ns ahead; correcting 1500→1000
        assert received[0].correct_timestamp(1500) == 1000

    # --- Control: verify integer + float mixed Pod ---

    def test_differential_drive_roundtrip(self):
        msg = horus.DifferentialDriveCommand(left_velocity=1.0, right_velocity=-1.0)
        roundtrip(horus.DifferentialDriveCommand, msg, {"left_velocity": 1.0, "right_velocity": -1.0})

    def test_pid_config_roundtrip(self):
        msg = horus.PidConfig(kp=1.0, ki=0.1, kd=0.01)
        roundtrip(horus.PidConfig, msg, {"kp": 1.0, "ki": 0.1, "kd": 0.01})

    def test_trajectory_point_roundtrip(self):
        msg = horus.TrajectoryPoint(time_from_start=1.5)
        roundtrip(horus.TrajectoryPoint, msg, {"time_from_start": 1.5})

    # --- Diagnostics: verify boolean + u8 + string fields ---

    def test_diagnostic_status_roundtrip(self):
        msg = horus.DiagnosticStatus(level=2, code=101, message="overheating", component="motor")
        roundtrip(horus.DiagnosticStatus, msg, {"level": 2, "code": 101, "component": "motor"})

    def test_resource_usage_roundtrip(self):
        msg = horus.ResourceUsage(cpu_percent=45.5, memory_bytes=1024000)
        roundtrip(horus.ResourceUsage, msg, {"memory_bytes": 1024000})

    def test_diagnostic_report_roundtrip(self):
        msg = horus.DiagnosticReport(component="sensor_hub", level=1)
        msg.add_value(horus.DiagnosticValue(key="temp", value="42"))
        roundtrip(horus.DiagnosticReport, msg, {"component": "sensor_hub", "level": 1, "value_count": 1})

    # --- Perception: verify serde-based complex types ---

    def test_landmark_roundtrip(self):
        msg = horus.Landmark(x=100.0, y=200.0, visibility=0.95, index=3)
        roundtrip(horus.Landmark, msg, {"x": 100.0, "y": 200.0, "index": 3})

    def test_landmark_3d_roundtrip(self):
        msg = horus.Landmark3D(x=1.0, y=2.0, z=3.0, visibility=0.8, index=7)
        roundtrip(horus.Landmark3D, msg, {"x": 1.0, "z": 3.0, "index": 7})

    def test_plane_detection_roundtrip(self):
        msg = horus.PlaneDetection()
        roundtrip(horus.PlaneDetection, msg, {"confidence": 0.5})

    def test_point_field_roundtrip(self):
        msg = horus.PointField(name="x", offset=0, datatype=7, count=1)
        roundtrip(horus.PointField, msg, {"name": "x", "offset": 0, "datatype": 7})

    # --- Vision: verify image data survives roundtrip ---

    def test_camera_info_roundtrip(self):
        msg = horus.CameraInfo(width=1920, height=1080, fx=1000.0, fy=1000.0, cx=960.0, cy=540.0)
        roundtrip(horus.CameraInfo, msg, {"width": 1920, "height": 1080})

    def test_region_of_interest_roundtrip(self):
        msg = horus.RegionOfInterest(x=100, y=200, width=50, height=60)
        roundtrip(horus.RegionOfInterest, msg, {"x_offset": 100, "y_offset": 200, "width": 50})

    # --- Path planning ---

    def test_path_plan_roundtrip(self):
        msg = horus.PathPlan()
        roundtrip(horus.PathPlan, msg, {"waypoint_count": 0})

    # --- Cost map ---

    def test_cost_map_roundtrip(self):
        grid = horus.OccupancyGrid(width=3, height=3, resolution=0.5)
        msg = horus.CostMap(grid=grid, inflation_radius=0.5)
        roundtrip(horus.CostMap, msg, {"inflation_radius": 0.5})
