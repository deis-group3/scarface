"""Unit tests for Scarface node."""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from scarface.scarface_node import ScarfaceNode


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context for tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_node_initialization(ros_context):
    """Test that the Scarface node initializes correctly."""
    node = ScarfaceNode()
    assert node.get_name() == "scarface_node"
    node.destroy_node()


def test_node_has_publisher(ros_context):
    """Test that the node has a status publisher."""
    node = ScarfaceNode()
    publishers = node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace()
    )
    assert any("scarface/status" in pub[0] for pub in publishers)
    node.destroy_node()


def test_node_has_subscriber(ros_context):
    """Test that the node has a command subscriber."""
    node = ScarfaceNode()
    subscriptions = node.get_subscriber_names_and_types_by_node(
        node.get_name(), node.get_namespace()
    )
    assert any("scarface/command" in sub[0] for sub in subscriptions)
    node.destroy_node()
