import math
import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from face_tracker_msgs.msg import Faces, Face, Point2
from interface.msg import HeadMovementGoal 
from core.util import run_node

CAMERA_DIAGONAL_FOV = 1.19555054 
CAMERA_RESOLUTION_X = 1280
CAMERA_RESOLUTION_Y = 960

COEFF_HEAD_PAN = -1.04387 
PRIORITY_FACE_TRACK = 2
TRACKING_GOAL_MIN_INTERVAL = 0.1 # min time between publishing goals (seconds)

ANGLE_PER_PIXEL = CAMERA_DIAGONAL_FOV / math.sqrt(CAMERA_RESOLUTION_X**2 + CAMERA_RESOLUTION_Y**2)
MIDDLE_X = CAMERA_RESOLUTION_X / 2
MIDDLE_Y = CAMERA_RESOLUTION_Y / 2

class FaceTrackNode(Node):

    def __init__(self):
        super().__init__("face_track")

        self.goal_publisher = self.create_publisher(HeadMovementGoal, "/head_move_goal", 10)
        self.get_logger().info(f"Publisher [{self.goal_publisher.topic_name}] added.")

        self.create_subscription(Faces, "/faces", self.face_list_callback, 2)


        self.last_published_tracking_goal = 0.0
        self.get_logger().info("Node [face_track] initialized.")

    def __get_face_area(self, face: Face) -> float:
        """
        Calculates the area of the face bounding box using top_left and bottom_right.
        """
        width = face.bottom_right.x - face.top_left.x
        height = face.bottom_right.y - face.top_left.y

        return max(0.0, width * height)

    def __select_face_to_track(self, face_list: list[Face]):
        """
        Finds the largest face in the list to track using the new message fields.
        """
        if not face_list:
            return None
        
        # Select the face with the largest calculated area
        best_face = max(face_list, key=self.__get_face_area)
        
        return best_face

    def face_list_callback(self, msg: Faces):
        """
        Processes the list of detected faces, calculates the movement goal, and publishes.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        # frequency check, dont publish target too often.
        if (current_time - self.last_published_tracking_goal) < TRACKING_GOAL_MIN_INTERVAL:
            return

        face_to_track = self.__select_face_to_track(msg.faces)
        
        if face_to_track is None:
            return

        width = face_to_track.bottom_right.x - face_to_track.top_left.x
        height = face_to_track.bottom_right.y - face_to_track.top_left.y
        
        center_x = face_to_track.top_left.x + (width / 2)
        center_y = face_to_track.top_left.y + (height / 2)

        pixel_error_x = center_x - MIDDLE_X
        pixel_error_y = center_y - MIDDLE_Y
        
        angle_error_x = pixel_error_x * ANGLE_PER_PIXEL
        angle_error_y = pixel_error_y * ANGLE_PER_PIXEL

        # determine movement (eyes vs head)
        HEAD_MOVEMENT_THRESHOLD = 0.15 # radians
        
        head_pan_target = None
        head_tilt_target = None
        eye_horizontal_target = None
        eye_vertical_target = None

        # head movement (for large errors/re-centering)
        if abs(angle_error_x) > HEAD_MOVEMENT_THRESHOLD or abs(angle_error_y) > HEAD_MOVEMENT_THRESHOLD:
            head_pan_target = angle_error_x 
            head_tilt_target = angle_error_y
        # eye movement (for small, precise tracking)
        else:
            eye_horizontal_target = angle_error_x 
            eye_vertical_target = angle_error_y

        # construct and publish the movement goal
        goal_msg = HeadMovementGoal()
        goal_msg.priority = PRIORITY_FACE_TRACK
        
        # Ensure float64 fields are set to a value (0.0 if not used)
        goal_msg.head_yaw_pan_target = head_pan_target if head_pan_target is not None else 0.0
        goal_msg.head_pitch_tilt_target = head_tilt_target if head_tilt_target is not None else 0.0
        goal_msg.eye_shift_horizontal_target = eye_horizontal_target if eye_horizontal_target is not None else 0.0
        goal_msg.eye_shift_vertical_target = eye_vertical_target if eye_vertical_target is not None else 0.0
        
        goal_msg.duration_nanosecs = 0 

        self.get_logger().info(
            f"Head [yaw= {goal_msg.head_yaw_pan_target:.4f}, pitch= {goal_msg.head_pitch_tilt_target:.4f}] | "
            f"Eyes [hor= {goal_msg.eye_shift_horizontal_target:.4f}, vert= {goal_msg.eye_shift_vertical_target:.4f}]"
        )
        
        self.goal_publisher.publish(goal_msg)
        self.last_published_tracking_goal = current_time

def main(args=None):
    run_node(FaceTrackNode)
    
if __name__ == "__main__":
    main()
