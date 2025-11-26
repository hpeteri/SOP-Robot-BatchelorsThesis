import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

from face_tracker_msgs.msg import Faces, Face as FaceMsg, Point2, Occurance
from perception.face_detection.core.face_analyzer import FaceAnalyzer
from perception.face_detection.core.lip_movement_net import LipMovementDetector
from perception.face_detection.util.fps_tracker import FPSTracker
from perception.face_detection.util.ros_utils import CvBridgePublisher
from core.util import run_node

# Subscribed Topics
IMAGE_TOPIC = "/i2e_webcam"

# Published Topics
FACE_IMAGE_TOPIC = "face_detection"
FACE_TOPIC = "faces"


MODEL_NAME = "1_32_False_True_0.25_lip_motion_net_model.h5"
SHAPE_PREDICTOR_NAME = "shape_predictor_68_face_landmarks.dat"

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__("face_detection_node")

        pkg_share = get_package_share_directory("perception")

        model_path = os.path.join(
            pkg_share,
            "face_detection",
            "models",
            MODEL_NAME
        )

        shape_predictor_path = os.path.join(
            pkg_share,
            "face_detection",
            "predictors",
            SHAPE_PREDICTOR_NAME
        )

        # Initialize components
        self.lip_detector = LipMovementDetector(
            model_path=model_path,
            shape_predictor_path=shape_predictor_path
        )
        self.face_analyzer = FaceAnalyzer(
            logger=self.get_logger(),
            lip_movement_detector=self.lip_detector
        )

        #FPS is drawn onto the 
        self.fps_tracker = FPSTracker()

        self.face_img_pub = CvBridgePublisher(self, FACE_IMAGE_TOPIC)
        self.get_logger().info(f"publisher ['{self.face_img_pub.publisher.topic_name}'] added.")

        self.face_pub = self.create_publisher(Faces, FACE_TOPIC, 1)
        self.get_logger().info(f"publisher ['{self.face_pub.topic_name}'] added.")
        
        self.subscriber = self.create_subscription(Image, IMAGE_TOPIC, self.on_frame_received, 1)
        
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.get_logger().info("FaceTrackerNode initialized.")

    def on_frame_received(self, img_msg: Image):
        """Handle incoming camera frames and process faces."""
        frame = self.face_img_pub.to_cv2(img_msg)
        if frame is None:
            return

        # Run analysis
        faces = self.face_analyzer.on_frame_received(frame)

        # Convert to ROS messages
        msg_faces = []
        for face in faces:
            occurances = [
                Occurance(
                    start_time=float(o["start_time"]),
                    end_time=float(o["end_time"]),
                    duration=float(o["duration"])
                )
                for o in face["previous_occurances"]
            ]
            msg_faces.append(
                FaceMsg(
                    top_left=Point2(x=face["left"], y=face["top"]),
                    bottom_right=Point2(x=face["right"], y=face["bottom"]),
                    diagonal=face["diagonal"],
                    face_id=face["face_id"],
                    speaking=face["speaking"],
                    occurances=occurances
                )
            )

        # Draw FPS on frame
        cv2.putText(frame, f"{self.fps_tracker.fps:.2f}", (10, 20), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Publish results
        self.face_img_pub.publish(frame)
        if msg_faces:
            self.face_pub.publish(Faces(faces=msg_faces))

        self.fps_tracker.update_fps()


def main(args=None):
    run_node(FaceDetectionNode)


if __name__ == "__main__":
    main()
