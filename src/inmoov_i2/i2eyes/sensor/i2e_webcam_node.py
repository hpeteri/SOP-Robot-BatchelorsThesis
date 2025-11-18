import rclpy
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from core.sensor import SensorBase, SensorNodeBase

INDEX      = 0
WIDTH      = 1280
HEIGHT     = 960
FPS        = 30
TOPIC_NAME = "i2e_webcam_raw/"

class i2e_cv2Webcam:
    """
    cv2Webcam wrapper
    """
    
    def __init__(self, video_in: int | str, width: int, height: int, fps: int):
        self.capture = cv2.VideoCapture(video_in)

        if self.is_valid():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    def is_valid(self) -> bool:
        """
        Check if the webcam is valid
        """
        return self.capture.isOpened():

    def close(self) -> None:
        if self.is_valid():
            return

        self.capture.release()

        # should we destroy all windows when closing a single webcam? What if we have multiple sensors?
        cv2.destroyAllWindows()

class i2e_WebcamSensor(SensorBase):
    """
    i2eyes Sensor for webcam Image
    """

    def __init__(self, sensor_name: str, node: Node, *args, **kwargs) -> None:
        super().__init__(sensor_name, node, args, kwargs)
        
        self.publisher = self.node.create_publisher(Image, TOPIC_NAME, 5)
        self.webcam = i2e_cv2Webcam(INDEX, WIDTH, HEIGHT, FPS)
        
    def read(self) -> None:
        """
        Read and publish webcam Image
        """
        try:
            success, frame = self.webcam.read()

            if not success:
                # i think read anyway throws, so we can throw here aswell to try reopen the webcam
                # @TODO - LOG
                raise Exception

        except Exception as e:
            self.webcam.close()
            self.webcam = i2e_cv2Webcam(INDEX, WIDTH, HEIGHT, FPS)
        
        try:
            data = bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(data)
        except CvBridgeError as e:
            # TODO - LOG
            pass
        
        
class i2e_WebcamNode(SensorNodeBase):
    """
    i2eyes Node for handling webcam sensor
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("i2e_webcam_node", *args, **kwargs);
        self.add_sensor("i2e_webcam", i2e_WebcamSendor)
        self.timer = self.create_timer(1 / FPS, self.read_all_sensors)

        
        
    
        
