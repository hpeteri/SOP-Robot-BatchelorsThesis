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

INDEX      = 0
WIDTH      = 1280
HEIGHT     = 960
FPS        = 30
TOPIC_NAME = "webcam_raw"

bridge = CvBridge()

class I2eCv2Webcam:
    """
    cv2Webcam wrapper
    """

    def __init__(self, video_in: int | str, width: int, height: int, fps: int):

        # set log level
        # CV_LOG_SILENT 0
        # CV_LOG_LEVEL_FATAL 1
        # CV_LOG_LEVEL_ERROR 2
        # CV_LOG_LEVEL_WARN 3
        # CV_LOG_LEVEL_INFO 4
        # CV_LOG_LEVEL_DEBUG 5
        cv2.setLogLevel(2)

        self.capture = cv2.VideoCapture(video_in)

        if self.is_valid():
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture.set(cv2.CAP_PROP_FPS, fps)
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    def is_valid(self) -> bool:
        """
        Check if the webcam is valid
        """
        return self.capture.isOpened()

    def close(self) -> None:
        """
        Close the cv2 handle
        """
        if not self.is_valid():
            return

        self.capture.release()

        # should we destroy all windows when closing a single webcam?
        # What if we have multiple sensors?
        cv2.destroyAllWindows()

class I2eWebcamSensor(SensorBase):
    """
    i2eyes Sensor for webcam Image
    """
    #pylint: disable=too-few-public-methods

    def __init__(self, sensor_name: str, node: Node) -> None:
        super().__init__(sensor_name, node)

        self.publisher = self.node.create_publisher(Image, TOPIC_NAME, 5)
        self.node.get_logger().info(f"publisher ['{self.publisher.topic_name}'] added.")
        self.webcam = I2eCv2Webcam(INDEX, WIDTH, HEIGHT, FPS)

    def read(self) -> None:
        """
        Read and publish webcam Image
        """

        frame = None
        
        # Try reading a frame
        try:
            success, frame = self.webcam.capture.read()
        
            if not success or frame is None:
                # @TODO - LOG
                #pylint: disable=broad-exception-raised
                raise Exception
        
        #pylint: disable=broad-exception-caught
        except Exception:
            self.webcam.close()
            self.webcam = I2eCv2Webcam(INDEX, WIDTH, HEIGHT, FPS)
            return
        
        # Try publishing
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
        self.add_sensor("webcam", I2eWebcamSensor)
        self.timer = self.create_timer(1 / FPS, self.read_sensors)

def main():
    """
    Run I2eWebcamNode
    """
    run_node(I2eWebcamNode)
