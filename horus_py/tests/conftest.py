"""
Pytest configuration for HORUS Python tests.

Note: HORUS now uses a flat namespace (ROS-like global topics), so tests
should use unique topic names to avoid conflicts. Session IDs are no longer
used for isolation.
"""
import os
import uuid
import pytest


@pytest.fixture(autouse=True)
def unique_test_prefix():
    """
    Generate a unique prefix for test topics to prevent conflicts between tests.

    With the flat namespace, tests should use unique topic names like:
    topic = Topic(f"{test_prefix}_my_topic")

    This fixture provides a unique prefix for each test.
    """
    # Generate a unique prefix for this test's topics
    test_prefix = f"test_{uuid.uuid4().hex[:8]}"

    yield test_prefix

    # Cleanup is automatic when process terminates - shared memory
    # files in /dev/shm/horus/topics/ persist until manually cleaned
    # or the next `horus run` clears stale topics
