import pytest
import rclpy
from rclpy.node import Node
from sensors.sensor_node_base import SensorNodeBase
from sensors.sensor_base import BaseSensor

class MockSensor(BaseSensor):
    """
    Mock sensor to test BaseSensor and BaseSensorNode functionality
    """
    def __init__(self, sensor_name: str, robot_node: Node) -> None:
        super().__init__(sensor_name, robot_node)
        self.mock_data = "Mock sensor data"

    def read(self) -> None:
        self.node.get_logger().info(f"Mock published: {self.mock_data}")


@pytest.fixture
def robot_node():
    """
    Fixure to initialize a Node
    """
    rclpy.init()
    node = SensorNodeBase("sendor_ut_node")
    yield node
    rclpy.shutdown()

def sendor_ut_add_sensor(robot_node):
    """
    Test that sensor is added to the node
    """

    robot_node.add_sensor("mock_0", MockSensor)

    assert "mock_0" in robot_node.sensors
    assert isinstance(robot_node.mock_0, MockSensor)
    assert isinstance(robot_node.sensors["mock_0"], MockSensor)
