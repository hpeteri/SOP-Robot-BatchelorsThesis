import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Type, Dict, Any
from abc import ABC, abstractMethod

class SensorBase(ABC):
    """
    Abstract Base Class for individual sensors.
    """
    def __init_(self, sensor_name: str, node: Node, *args, **kwargs) -> None:
        self.name: str = sensor_name
        self.node: Node = node

    @abstractmethod
    def read(self) -> None:
        """
        Read Sensor data and publish it.
        """
        pass
