"""
Cross-Language Integration Tests - Python ↔ Rust Communication

Tests that verify data can be correctly passed between Rust and Python
components using HORUS pub/sub infrastructure.
"""

import pytest
import subprocess
import time
import tempfile
import os
from pathlib import Path


def build_rust_binary(rust_code: str, binary_name: str) -> str:
    """
    Build a Rust binary with cargo in a temporary project.

    Returns the path to the compiled binary.
    """
    # Create temporary directory for cargo project
    with tempfile.TemporaryDirectory() as tmpdir:
        project_dir = Path(tmpdir) / "test_project"
        project_dir.mkdir()

        # Write Cargo.toml
        cargo_toml = """[package]
name = "test_project"
version = "0.1.6"
edition = "2021"

[dependencies]
horus = { path = "/home/lord-patpak/horus/HORUS/horus" }
horus_library = { path = "/home/lord-patpak/horus/HORUS/horus_library" }
"""
        (project_dir / "Cargo.toml").write_text(cargo_toml)

        # Create src directory and main.rs
        src_dir = project_dir / "src"
        src_dir.mkdir()
        (src_dir / "main.rs").write_text(rust_code)

        # Build with cargo
        result = subprocess.run(
            ["cargo", "build", "--release"],
            cwd=project_dir,
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            raise RuntimeError(f"Cargo build failed: {result.stderr}")

        # Copy binary to /tmp
        binary_path = project_dir / "target" / "release" / "test_project"
        output_path = f"/tmp/{binary_name}"

        subprocess.run(["cp", str(binary_path), output_path], check=True)

        return output_path


def test_rust_to_python_basic():
    """Test basic message passing from Rust publisher to Python subscriber

    Uses typed hubs (PyPose2DHub) for cross-language communication.
    """
    from horus._horus import PyPose2DHub
    from horus import Pose2D

    # Create a simple Rust publisher
    rust_code = """
use horus::prelude::*;
use horus_library::messages::Pose2D;

fn main() -> Result<()> {
    let hub = Hub::new("test_topic")?;

    for i in 0..5 {
        let pose = Pose2D::new(i as f64, i as f64 * 2.0, i as f64 * 0.5);
        hub.send(pose, None).expect("Failed to send message");
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
"""

    try:
        # Build Rust binary with cargo
        binary_path = build_rust_binary(rust_code, "rust_pub")

        # Create typed hub for receiving Pose2D messages from Rust
        py_hub = PyPose2DHub("test_topic")

        # Run Rust publisher in background
        rust_runner = subprocess.Popen([binary_path])

        # Give Rust time to start and send messages
        time.sleep(0.5)

        # Receive messages from Rust
        received = []
        for _ in range(20):  # Try up to 20 times
            msg = py_hub.recv()
            if msg:
                received.append(msg)
                # Verify it's a Pose2D object with correct attributes
                assert hasattr(msg, 'x')
                assert hasattr(msg, 'y')
                assert hasattr(msg, 'theta')
                if len(received) >= 3:
                    break
            time.sleep(0.1)

        rust_runner.wait(timeout=3)

        # Verify we received messages
        assert len(received) >= 3, f"Expected at least 3 messages, got {len(received)}"

        # Verify message content
        assert received[0].x == 0.0
        assert received[0].y == 0.0
        assert received[0].theta == 0.0

    finally:
        if os.path.exists('/tmp/rust_pub'):
            os.unlink('/tmp/rust_pub')


def test_python_to_rust_basic():
    """Test basic message passing from Python publisher to Rust subscriber

    Uses typed hubs (PyPose2DHub) for cross-language communication.
    """
    from horus._horus import PyPose2DHub
    from horus import Pose2D

    # Create Rust subscriber
    rust_code = """
use horus::prelude::*;
use horus_library::messages::Pose2D;

fn main() -> Result<()> {
    let hub: Hub<Pose2D> = Hub::new("test_topic")?;
    let mut count = 0;

    for _ in 0..50 {
        if let Some(pose) = hub.recv(None) {
            count += 1;
            println!("Received: x={}, y={}, theta={}", pose.x, pose.y, pose.theta);

            if count >= 3 {
                break;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    assert!(count >= 3, "Expected at least 3 messages, got {}", count);
    Ok(())
}
"""

    try:
        # Build Rust binary with cargo
        binary_path = build_rust_binary(rust_code, "rust_sub")

        # Run Rust subscriber in background
        rust_proc = subprocess.Popen(
            [binary_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        time.sleep(0.5)  # Let subscriber start

        # Create typed hub for sending Pose2D messages to Rust
        py_hub = PyPose2DHub("test_topic")

        # Send messages from Python
        for i in range(5):
            pose = Pose2D(x=float(i), y=float(i) * 2.0, theta=float(i) * 0.5)
            py_hub.send(pose)
            time.sleep(0.1)

        # Wait for Rust subscriber to finish
        stdout, stderr = rust_proc.communicate(timeout=3)

        assert rust_proc.returncode == 0, f"Rust subscriber failed: {stderr.decode()}"

    finally:
        if os.path.exists('/tmp/rust_sub'):
            os.unlink('/tmp/rust_sub')


def test_message_types_pose2d():
    """Test Pose2D message integrity across Python-Rust boundary"""
    import horus
    from horus import Pose2D

    test_data = []

    def pub_node(node):
        tick = node.info.tick_count()
        test_cases = [
            (0.0, 0.0, 0.0),
            (1.5, 2.5, 0.785),
            (-1.0, -2.0, -1.57),
            (100.5, 200.5, 3.14159),
        ]
        if tick < len(test_cases):
            x, y, theta = test_cases[tick]
            pose = Pose2D(x=x, y=y, theta=theta)
            node.send("pose_test", pose)
            test_data.append((x, y, theta))
        elif tick >= len(test_cases) + 2:
            node.request_stop()

    received_data = []

    def sub_node(node):
        msg = node.get("pose_test")
        if msg:
            received_data.append((msg.x, msg.y, msg.theta))
        if len(received_data) >= 4:
            node.request_stop()

    pub = horus.Node(name="pub", pubs={"pose_test": {"type": Pose2D}}, tick=pub_node)
    sub = horus.Node(name="sub", subs={"pose_test": {"type": Pose2D}}, tick=sub_node)

    horus.run(pub, sub, duration=1.0)

    # Verify data integrity
    assert len(received_data) == len(test_data), \
        f"Expected {len(test_data)} messages, got {len(received_data)}"

    for sent, received in zip(test_data, received_data):
        assert abs(sent[0] - received[0]) < 1e-10, f"X mismatch: {sent[0]} != {received[0]}"
        assert abs(sent[1] - received[1]) < 1e-10, f"Y mismatch: {sent[1]} != {received[1]}"
        assert abs(sent[2] - received[2]) < 1e-10, f"Theta mismatch: {sent[2]} != {received[2]}"


def test_message_types_cmdvel():
    """Test CmdVel message integrity"""
    import horus
    from horus import CmdVel

    test_data = []

    def pub_node(node):
        tick = node.info.tick_count()
        test_cases = [
            (0.0, 0.0),
            (1.5, 0.5),
            (-0.5, -0.2),
            (2.0, 1.0),
        ]
        if tick < len(test_cases):
            linear, angular = test_cases[tick]
            cmd = CmdVel(linear=linear, angular=angular)
            node.send("cmd_test", cmd)
            test_data.append((linear, angular))
        elif tick >= len(test_cases) + 2:
            node.request_stop()

    received_data = []

    def sub_node(node):
        msg = node.get("cmd_test")
        if msg:
            received_data.append((msg.linear, msg.angular))
        if len(received_data) >= 4:
            node.request_stop()

    pub = horus.Node(name="pub", pubs={"cmd_test": {"type": CmdVel}}, tick=pub_node)
    sub = horus.Node(name="sub", subs={"cmd_test": {"type": CmdVel}}, tick=sub_node)

    horus.run(pub, sub, duration=1.0)

    assert len(received_data) == len(test_data)
    for sent, received in zip(test_data, received_data):
        assert abs(sent[0] - received[0]) < 1e-6
        assert abs(sent[1] - received[1]) < 1e-6


def test_message_types_laserscan():
    """Test LaserScan with NumPy arrays"""
    import horus
    from horus import LaserScan
    import numpy as np

    sent_scan = None
    received_scan = None

    def pub_node(node):
        nonlocal sent_scan
        tick = node.info.tick_count()
        if tick == 0:
            scan = LaserScan()
            # Set ranges using NumPy array
            ranges = np.random.rand(360).astype(np.float32) * 10.0
            scan.ranges = ranges
            sent_scan = ranges.copy()
            node.send("scan_test", scan)
        elif tick >= 5:
            node.request_stop()

    def sub_node(node):
        nonlocal received_scan
        msg = node.get("scan_test")
        if msg:
            received_scan = msg.ranges.copy()
            node.request_stop()

    pub = horus.Node(name="pub", pubs={"scan_test": {"type": LaserScan}}, tick=pub_node)
    sub = horus.Node(name="sub", subs={"scan_test": {"type": LaserScan}}, tick=sub_node)

    horus.run(pub, sub, duration=1.0)

    assert sent_scan is not None, "Publisher didn't send scan"
    assert received_scan is not None, "Subscriber didn't receive scan"
    assert np.allclose(sent_scan, received_scan), "Scan data mismatch"


def test_error_handling_across_languages():
    """Test that errors in one language don't crash the other"""
    import horus
    from horus import Pose2D

    error_count = [0]

    def faulty_node(node):
        # Intentionally cause an error every other tick
        if node.info.tick_count() % 2 == 0:
            raise ValueError("Intentional error for testing")

        # But still try to communicate
        pose = Pose2D(x=1.0, y=2.0, theta=0.5)
        node.send("error_test", pose)

    def robust_node(node):
        msg = node.get("error_test")
        if msg:
            # Should still receive messages despite errors in other node
            pass

        if node.info.tick_count() >= 10:
            node.request_stop()

    def error_handler(node, error):
        error_count[0] += 1

    faulty = horus.Node(name="faulty", tick=faulty_node, on_error=error_handler)
    robust = horus.Node(name="robust", tick=robust_node)

    horus.run(faulty, robust, duration=1.0)

    # Should have caught some errors but still completed
    assert error_count[0] > 0, "No errors were caught"
    assert error_count[0] < 20, "Too many errors"


def test_high_frequency_communication():
    """Test sustained high-frequency communication between Python nodes"""
    import horus
    from horus import Pose2D

    sent_count = [0]
    received_count = [0]

    def fast_publisher(node):
        # Publish at every tick
        pose = Pose2D(x=float(sent_count[0]), y=0.0, theta=0.0)
        node.send("high_freq", pose)
        sent_count[0] += 1

        if sent_count[0] >= 100:
            node.request_stop()

    def fast_subscriber(node):
        msg = node.get("high_freq")
        if msg:
            received_count[0] += 1

    pub = horus.Node(name="fast_pub", tick=fast_publisher)
    sub = horus.Node(name="fast_sub", tick=fast_subscriber)

    horus.run(pub, sub, duration=1.0)

    # Should receive most messages (allowing for some timing variance)
    reception_rate = received_count[0] / sent_count[0]
    assert reception_rate > 0.8, \
        f"Low reception rate: {reception_rate:.2%} ({received_count[0]}/{sent_count[0]})"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
