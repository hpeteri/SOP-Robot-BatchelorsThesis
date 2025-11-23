"""
sensor_ut.py

This module implements Unit Tests for SensorBase and SensorNodeBase modules
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor import SensorNodeBase, SensorBase

class MockSensor(SensorBase):
    """
    Mock sensor to test SensorBase and SensorBaseNode functionality
    """
    #pylint: disable=too-few-public-methods

    def __init__(self, sensor_name: str, node: Node) -> None:
        super().__init__(sensor_name, node)
        self.mock_data = "Mock sensor data"

    def read(self) -> None:
        self.node.get_logger().info(f"Mock published: {self.mock_data}")


#pylint: disable=redefined-outer-name
@pytest.fixture
def ut_node():
    """
    Fixure to initialize a Node
    """
    rclpy.init()
    node = SensorNodeBase("sendor_ut_node")
    yield node
    rclpy.shutdown()

def test_add_sensor(ut_node):
    """
    Test that sensor is added to the node
    """

    ut_node.add_sensor("mock_0", MockSensor)

    assert "mock_0" in ut_node.sensors
    assert isinstance(ut_node.mock_0, MockSensor)
    assert isinstance(ut_node.sensors["mock_0"], MockSensor)
