import rclpy
from rclpy.node import Node
import math
import time
from typing import Optional, List

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.action import ActionClient

from interface.msg import HeadMovementGoal 

PRIORITY_IDLE = 1
PRIORITY_FACE_TRACK = 2
PRIORITY_GESTURE = 3 
NO_ACTIVE_PRIORITY = 0

COEFF_HEAD_PAN = -1.04387 
COEFF_EYE_HORIZONTAL = -2.67659 
COEFF_EYE_VERTICAL = 4.01489 

# limits
HEAD_PAN_MIN_RAD = -0.7
HEAD_PAN_MAX_RAD = 0.7
HEAD_PITCH_MIN_RAD = -0.3
HEAD_PITCH_MAX_RAD = 0.3
EYE_H_MIN_RAD = -0.2
EYE_H_MAX_RAD = 0.2
EYE_V_MIN_RAD = -0.15
EYE_V_MAX_RAD = 0.15

# Joint Names
HEAD_JOINT_NAMES = ["head_pan_joint", "head_pitch_joint"]
EYE_JOINT_NAMES = ["eye_horizontal_joint", "eye_vertical_joint"]
DEFAULT_TRAJECTORY_DURATION = 0.3

ACTION_TYPE_N_A = 0
ACTION_TYPE_HEAD = 1
ACTION_TYPE_EYES = 2

class HeadMovementNode(Node):

    def __init__(self):
        super().__init__("head_movement")

        # State Variables
        self.head_servo_state: Optional[List[float]] = None # [pan, pitch]
        self.eyes_servo_state: Optional[List[float]] = None # [horizontal, vertical]
        self.active_priority = NO_ACTIVE_PRIORITY
        self.is_gesture_active = False

        self.head_action_client = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self.eyes_action_client = ActionClient(self, FollowJointTrajectory, "/eyes_controller/follow_joint_trajectory")
        
        # Subscriptions for State Feedback (Required for Kinematics/Arbitration)
        self.create_subscription(
            JointTrajectoryControllerState, 
            "/head_controller/controller_state", 
            self.head_state_callback, 
            10
        )
        
        self.create_subscription(
            JointTrajectoryControllerState, 
            "/eyes_controller/controller_state", 
            self.eyes_state_callback, 
            10
        )

        self.create_subscription(
            HeadMovementGoal, 
            "/head_move_goal", 
            self.move_goal_callback, 
            10
        )

        self.create_subscription(
            String, 
            "/head_gesture_command",
            self.head_gesture_callback, 
            10
        )

        self.get_logger().info("Node [head_movement] Initialized.")
        
    def head_state_callback(self, msg: JointTrajectoryControllerState):
        """Updates the current head joint positions."""
        # positions[0] is typically pan/yaw, positions[1] is pitch/tilt
        if msg.actual.positions:
            self.head_servo_state = list(msg.actual.positions)

    def eyes_state_callback(self, msg: JointTrajectoryControllerState):
        """Updates the current eye joint positions."""
        # positions[0] is horizontal shift, positions[1] is vertical shift
        if msg.actual.positions:
            self.eyes_servo_state = list(msg.actual.positions)

    def _transform_camera_angle_to_absolute_target(self, goal: HeadMovementGoal) -> tuple[List[float], List[float], int]:
        """
        Converts the angle error in the HeadMovementGoal into absolute joint targets.
        Returns: (head_targets, eye_targets, action_type)
        """
        head_targets = [0.0, 0.0]
        eye_targets = [0.0, 0.0]
        action_type = ACTION_TYPE_N_A

        # Check if we have current state to calculate the new absolute position
        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Cannot calculate targets: Head or Eye state not yet received.")
            return head_targets, eye_targets, "State Missing"

        current_head_pan, current_head_pitch = self.head_servo_state
        current_eye_h, current_eye_v = self.eyes_servo_state

        # Check if the goal specifies head movement
        if abs(goal.head_yaw_pan_target) > 1e-4 or abs(goal.head_pitch_tilt_target) > 1e-4:
            # HEAD MOVEMENT: 
            head_targets[0] = current_head_pan + (goal.head_yaw_pan_target * COEFF_HEAD_PAN)
            head_targets[1] = current_head_pitch + (goal.head_pitch_tilt_target * COEFF_HEAD_PITCH)

            action_type = ACTION_TYPE_HEAD
            
        # Check if the goal specifies eye movement (P2 Face Tracking or P1 Idle)
        elif abs(goal.eye_shift_horizontal_target) > 1e-4 or abs(goal.eye_shift_vertical_target) > 1e-4:

            eye_targets[0] = current_eye_h + (goal.eye_shift_horizontal_target * COEFF_EYE_HORIZONTAL)
            eye_targets[1] = current_eye_v + (goal.eye_shift_vertical_target * COEFF_EYE_VERTICAL)

            action_type = ACTION_TYPE_EYES

        # Apply Joint Limits to prevent damage
        head_targets[0] = self._apply_joint_limits(head_targets[0], HEAD_PAN_MIN_RAD, HEAD_PAN_MAX_RAD)
        head_targets[1] = self._apply_joint_limits(head_targets[1], HEAD_PITCH_MIN_RAD, HEAD_PITCH_MAX_RAD)
        eye_targets[0] = self._apply_joint_limits(eye_targets[0], EYE_H_MIN_RAD, EYE_H_MAX_RAD)
        eye_targets[1] = self._apply_joint_limits(eye_targets[1], EYE_V_MIN_RAD, EYE_V_MAX_RAD)
        
        return head_targets, eye_targets, action_type


    def _apply_joint_limits(self, position: float, min_limit: float, max_limit: float) -> float:
        """Clamps the target position within the defined physical limits."""
        return max(min_limit, min(max_limit, position))

    def move_goal_callback(self, goal: HeadMovementGoal):
        """
        Receives movement goals and handles arbitration.
        """
        
        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Ignoring goal: State is not initialized.")
            return

        # ignore lower priorities
        if goal.priority < self.active_priority:
            self.get_logger().debug(f"priority [{goal.priority}] ignored. Active priority [{self.active_priority}]")
            return
            
        self.active_priority = goal.priority

        head_targets, eye_targets, action_type = self._transform_camera_angle_to_absolute_target(goal)
        
        if action_type == ACTION_TYPE_N_A:
            # This happens if a goal is empty
            return

        # move
        duration_sec = goal.duration_nanosecs / 1e9 if goal.duration_nanosecs > 0 else DEFAULT_TRAJECTORY_DURATION

        # head movement
        if action_type == ACTION_TYPE_HEAD:
            self.get_logger().info(f"priority [{goal.priority}], head goal [{head_targets[0]:.4f}, {head_targets[1]:.4f}] rad in {duration_sec}s")
            self._create_and_publish_trajectory(
                self.head_action_client, HEAD_JOINT_NAMES, head_targets, duration_sec
            )
        # eye movement
        elif action_type == ACTION_TYPE_EYES:
            self.get_logger().info(f"priority [{goal.priority}], eye goal [{eye_targets[0]:.4f}, {eye_targets[1]:.4f}] rad in {duration_sec}s")
            self._create_and_publish_trajectory(
                self.eyes_action_client, EYE_JOINT_NAMES, eye_targets, duration_sec
            )

        # decay active priority
        if self.active_priority == PRIORITY_FACE_TRACK:
            # go back to idle if faces stop appearing. This should return to face track almost instantly if faces are found
            self.active_priority = PRIORITY_IDLE


    def head_gesture_callback(self, msg: String):
        """
        Receives high-priority gesture commands and executes them directly.
        This blocks lower priority goals until the gesture is complete
        """
        command = msg.data.split(",")[0].strip().lower()
        
        self.active_priority = PRIORITY_GESTURE
        self.get_logger().warn(f"GESTURE ACTIVATED: Blocking all other goal commands: [{command}]")
        
        if command == "shake":
            self._execute_shake(magnitude=0.4, repetitions=3, duration=0.2)
        elif command == "nod":
            self._execute_nod(magnitude=0.3, duration=0.3)
        
        # gesture complete
        self.active_priority = PRIORITY_IDLE # Reset to the default lowest priority
        self.get_logger().warn(f"GESTURE COMPLETED. Arbitration priority reset to [{PRIORITY_IDLE}].")


    def _create_and_publish_trajectory(self, client: ActionClient, joint_names: List[str], positions: List[float], duration_sec: float):
        """
        Creates and sends a FollowJointTrajectory goal to the specified controller.
        """
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"Action server not available for {joint_names[0]} controller.")
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=math.floor(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        client.send_goal_async(goal_msg)


    def _execute_shake(self, magnitude: float, repetitions: int, duration: float):
        """
        blocking shake sequence.
        """
        current_pan = self.head_servo_state[0] if self.head_servo_state else 0.0
        
        for i in range(repetitions):
            # Move Right
            target_r = self._apply_joint_limits(current_pan + magnitude, HEAD_PAN_MIN_RAD, HEAD_PAN_MAX_RAD)
            self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[0]], [target_r], duration)
            time.sleep(duration + 0.1) # Wait for trajectory to complete
            
            # Move Left
            target_l = self._apply_joint_limits(current_pan - magnitude, HEAD_PAN_MIN_RAD, HEAD_PAN_MAX_RAD)
            self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[0]], [target_l], duration)
            time.sleep(duration + 0.1)
            
        # Return to center
        self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[0]], [current_pan], duration)
        time.sleep(duration + 0.1)
        
    def _execute_nod(self, magnitude: float, duration: float):
        """
        blocking nod sequence.
        """
        current_pitch = self.head_servo_state[1] if self.head_servo_state else 0.0
        
        # Move Up
        target_up = self._apply_joint_limits(current_pitch + magnitude, HEAD_PITCH_MIN_RAD, HEAD_PITCH_MAX_RAD)
        self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[1]], [target_up], duration)
        time.sleep(duration + 0.1)
        
        # Move Down
        target_down = self._apply_joint_limits(current_pitch - magnitude, HEAD_PITCH_MIN_RAD, HEAD_PITCH_MAX_RAD)
        self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[1]], [target_down], duration)
        time.sleep(duration + 0.1)

        # Return to center
        self._create_and_publish_trajectory(self.head_action_client, [HEAD_JOINT_NAMES[1]], [current_pitch], duration)
        time.sleep(duration + 0.1)

def main(args=None):
    rclpy.init(args=args)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    node = HeadMovementNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
