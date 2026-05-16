"""
i2e_webcam_node.py

This Module implements i2eyes Webcam Sensor Node
"""

from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from core.sensor import SensorBase, SensorNodeBase
from core.util import run_node

bridge = CvBridge()

class I2eCv2Webcam:
    """
    cv2Webcam wrapper
    """

    def __init__(self, video_in: int | str, width: int, height: int, fps: int):

        cv2.setLogLevel(2)

        self.capture = cv2.VideoCapture(video_in)

        if self.is_valid():
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture.set(cv2.CAP_PROP_FPS, fps)
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    def is_valid(self) -> bool:
        return self.capture.isOpened()

    def close(self) -> None:
        if not self.is_valid():
            return

        self.capture.release()
        cv2.destroyAllWindows()

class I2eWebcamSensor(SensorBase):
    """
    i2eyes Sensor for webcam Image
    """
    #pylint: disable=too-few-public-methods

    def __init__(self, sensor_name: str, node: Node) -> None:
        super().__init__(sensor_name, node)

        topic_name = self.node.get_parameter("topic_name").value

        self.publisher = self.node.create_publisher(Image, topic_name, 5)
        self.node.get_logger().info(f"publisher ['{self.publisher.topic_name}'] added.")

        camera_index = self.node.get_parameter("camera_index").value
        camera_width = self.node.get_parameter("camera_width").value
        camera_height = self.node.get_parameter("camera_height").value
        camera_fps = self.node.get_parameter("camera_fps").value

        self.webcam = I2eCv2Webcam(camera_index, camera_width, camera_height, camera_fps)

    def read(self) -> None:
        frame = None

        try:
            success, frame = self.webcam.capture.read()

            if not success or frame is None:
                #pylint: disable=broad-exception-raised
                raise Exception

        #pylint: disable=broad-exception-caught
        except Exception:
            camera_index = self.node.get_parameter("camera_index").value
            camera_width = self.node.get_parameter("camera_width").value
            camera_height = self.node.get_parameter("camera_height").value
            camera_fps = self.node.get_parameter("camera_fps").value

            self.webcam.close()
            self.webcam = I2eCv2Webcam(camera_index, camera_width, camera_height, camera_fps)
            return

        try:
            data = bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(data)

        #pylint: disable=broad-exception-caught
        except Exception:
            pass

class I2eWebcamNode(SensorNodeBase):
    """
    i2eyes Node for handling webcam sensor
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("i2e_webcam", *args, **kwargs)

        self.declare_parameter("topic_name", "/i2e_webcam")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 960)
        self.declare_parameter("camera_fps", 30)

        self.add_sensor("webcam", I2eWebcamSensor)

        camera_fps = self.get_parameter("camera_fps").value
        self.timer = self.create_timer(1 / camera_fps, self.read_sensors)

def main():
    """
    Run I2eWebcamNode
    """
    run_node(I2eWebcamNode)
