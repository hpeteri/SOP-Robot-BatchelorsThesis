import rclpy
from rclpy.node import Node
import math
import random
import time
from typing import Optional, List

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from rclpy.action import ActionClient

from interface.msg import HeadMovementGoal

class HeadMovementNode(Node):

    def __init__(self):
        super().__init__("head_movement")

        self.declare_parameter("priority_idle", 1)
        self.declare_parameter("priority_face_track", 2)
        self.declare_parameter("priority_gesture", 3)
        self.declare_parameter("no_active_priority", 0)

        self.declare_parameter("coeff_head_pan", -1.04387)
        self.declare_parameter("coeff_head_pitch", -2.67659)
        self.declare_parameter("coeff_eye_horizontal", -2.67659)
        self.declare_parameter("coeff_eye_vertical", 4.01489)

        self.declare_parameter("head_pan_min_rad", -0.7)
        self.declare_parameter("head_pan_max_rad", 0.7)
        self.declare_parameter("head_pitch_min_rad", -0.3)
        self.declare_parameter("head_pitch_max_rad", 0.3)
        self.declare_parameter("eye_h_min_rad", -0.2)
        self.declare_parameter("eye_h_max_rad", 0.2)
        self.declare_parameter("eye_v_min_rad", -0.15)
        self.declare_parameter("eye_v_max_rad", 0.15)

        self.declare_parameter("head_joint_names", ["head_pan_joint", "head_pitch_joint"])
        self.declare_parameter("eye_joint_names", ["eye_horizontal_joint", "eye_vertical_joint"])
        self.declare_parameter("default_trajectory_duration", 0.3)

        self.declare_parameter("idle_interval_sec", 5.0)
        self.declare_parameter("idle_min_duration_sec", 0.75)
        self.declare_parameter("idle_max_move_ratio", 0.6667)
        self.declare_parameter("idle_after_no_face_sec", 4.0)
        self.declare_parameter("glance_chance", 0.005)
        self.declare_parameter("eyes_center_h", 0.0)
        self.declare_parameter("eyes_center_v", 0.0)

        self.PRIORITY_IDLE = self.get_parameter("priority_idle").value
        self.PRIORITY_FACE_TRACK = self.get_parameter("priority_face_track").value
        self.PRIORITY_GESTURE = self.get_parameter("priority_gesture").value
        self.NO_ACTIVE_PRIORITY = self.get_parameter("no_active_priority").value

        self.COEFF_HEAD_PAN = self.get_parameter("coeff_head_pan").value
        self.COEFF_HEAD_PITCH = self.get_parameter("coeff_head_pitch").value
        self.COEFF_EYE_HORIZONTAL = self.get_parameter("coeff_eye_horizontal").value
        self.COEFF_EYE_VERTICAL = self.get_parameter("coeff_eye_vertical").value

        self.HEAD_PAN_MIN_RAD = self.get_parameter("head_pan_min_rad").value
        self.HEAD_PAN_MAX_RAD = self.get_parameter("head_pan_max_rad").value
        self.HEAD_PITCH_MIN_RAD = self.get_parameter("head_pitch_min_rad").value
        self.HEAD_PITCH_MAX_RAD = self.get_parameter("head_pitch_max_rad").value
        self.EYE_H_MIN_RAD = self.get_parameter("eye_h_min_rad").value
        self.EYE_H_MAX_RAD = self.get_parameter("eye_h_max_rad").value
        self.EYE_V_MIN_RAD = self.get_parameter("eye_v_min_rad").value
        self.EYE_V_MAX_RAD = self.get_parameter("eye_v_max_rad").value

        self.HEAD_JOINT_NAMES = self.get_parameter("head_joint_names").value
        self.EYE_JOINT_NAMES = self.get_parameter("eye_joint_names").value
        self.DEFAULT_TRAJECTORY_DURATION = self.get_parameter("default_trajectory_duration").value

        self.IDLE_INTERVAL_SEC = self.get_parameter("idle_interval_sec").value
        self.IDLE_MIN_DURATION_SEC = self.get_parameter("idle_min_duration_sec").value
        self.IDLE_MAX_MOVE_RATIO = self.get_parameter("idle_max_move_ratio").value
        self.IDLE_AFTER_NO_FACE_SEC = self.get_parameter("idle_after_no_face_sec").value
        self.GLANCE_CHANCE = self.get_parameter("glance_chance").value
        self.EYES_CENTER_H = self.get_parameter("eyes_center_h").value
        self.EYES_CENTER_V = self.get_parameter("eyes_center_v").value

        self.head_servo_state: Optional[List[float]] = None
        self.eyes_servo_state: Optional[List[float]] = None
        self.active_priority = self.NO_ACTIVE_PRIORITY
        self.is_gesture_active = False
        self.last_face_track_time = 0.0

        self.head_action_client = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self.eyes_action_client = ActionClient(self, FollowJointTrajectory, "/eyes_controller/follow_joint_trajectory")

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

        self.idle_timer = self.create_timer(self.IDLE_INTERVAL_SEC, self.idle_timer_callback)

        self.get_logger().info("Node [head_movement] Initialized.")
        self.get_logger().info(f"  Idle timer: {self.IDLE_INTERVAL_SEC}s interval, expire-after {self.IDLE_AFTER_NO_FACE_SEC}s idle")
        self.get_logger().info("  Gestures: nod, shake")

    def head_state_callback(self, msg: JointTrajectoryControllerState):
        if msg.actual.positions:
            self.head_servo_state = list(msg.actual.positions)

    def eyes_state_callback(self, msg: JointTrajectoryControllerState):
        if msg.actual.positions:
            self.eyes_servo_state = list(msg.actual.positions)

    def _transform_camera_angle_to_absolute_target(self, goal: HeadMovementGoal) -> tuple[List[float], List[float], int]:
        head_targets = [0.0, 0.0]
        eye_targets = [0.0, 0.0]
        action_type = 0

        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Cannot calculate targets: Head or Eye state not yet received.")
            return head_targets, eye_targets, 0

        current_head_pan, current_head_pitch = self.head_servo_state
        current_eye_h, current_eye_v = self.eyes_servo_state

        if abs(goal.head_yaw_pan_target) > 1e-4 or abs(goal.head_pitch_tilt_vertical_target) > 1e-4:
            head_targets[0] = current_head_pan + (goal.head_yaw_pan_target * self.COEFF_HEAD_PAN)
            head_targets[1] = current_head_pitch + (goal.head_pitch_tilt_vertical_target * self.COEFF_HEAD_PITCH)
            action_type = 1

        elif abs(goal.eye_shift_horizontal_target) > 1e-4 or abs(goal.eye_shift_vertical_target) > 1e-4:
            eye_targets[0] = current_eye_h + (goal.eye_shift_horizontal_target * self.COEFF_EYE_HORIZONTAL)
            eye_targets[1] = current_eye_v + (goal.eye_shift_vertical_target * self.COEFF_EYE_VERTICAL)
            action_type = 2

        head_targets[0] = self._apply_joint_limits(head_targets[0], self.HEAD_PAN_MIN_RAD, self.HEAD_PAN_MAX_RAD)
        head_targets[1] = self._apply_joint_limits(head_targets[1], self.HEAD_PITCH_MIN_RAD, self.HEAD_PITCH_MAX_RAD)
        eye_targets[0] = self._apply_joint_limits(eye_targets[0], self.EYE_H_MIN_RAD, self.EYE_H_MAX_RAD)
        eye_targets[1] = self._apply_joint_limits(eye_targets[1], self.EYE_V_MIN_RAD, self.EYE_V_MAX_RAD)

        return head_targets, eye_targets, action_type

    def _apply_joint_limits(self, position: float, min_limit: float, max_limit: float) -> float:
        return max(min_limit, min(max_limit, position))

    def move_goal_callback(self, goal: HeadMovementGoal):

        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Ignoring goal: State is not initialized.")
            return

        if goal.priority < self.active_priority:
            self.get_logger().debug(f"priority [{goal.priority}] ignored. Active priority [{self.active_priority}]")
            return

        self.active_priority = goal.priority

        if goal.priority == self.PRIORITY_FACE_TRACK:
            self.last_face_track_time = self.get_clock().now().nanoseconds / 1e9

        head_targets, eye_targets, action_type = self._transform_camera_angle_to_absolute_target(goal)

        if action_type == 0:
            return

        duration_sec = goal.duration_nanosecs / 1e9 if goal.duration_nanosecs > 0 else self.DEFAULT_TRAJECTORY_DURATION

        if action_type == 1:
            self.get_logger().info(f"priority [{goal.priority}], head goal [{head_targets[0]:.4f}, {head_targets[1]:.4f}] rad in {duration_sec}s")
            self._create_and_publish_trajectory(
                self.head_action_client, self.HEAD_JOINT_NAMES, head_targets, duration_sec
            )
        elif action_type == 2:
            self.get_logger().info(f"priority [{goal.priority}], eye goal [{eye_targets[0]:.4f}, {eye_targets[1]:.4f}] rad in {duration_sec}s")
            self._create_and_publish_trajectory(
                self.eyes_action_client, self.EYE_JOINT_NAMES, eye_targets, duration_sec
            )

        if self.active_priority == self.PRIORITY_FACE_TRACK:
            self.active_priority = self.PRIORITY_IDLE

    def head_gesture_callback(self, msg: String):
        command = msg.data.split(",")[0].strip().lower()

        self.active_priority = self.PRIORITY_GESTURE
        self.get_logger().warn(f"GESTURE ACTIVATED: Blocking all other goal commands: [{command}]")

        if command == "shake":
            self._execute_shake(magnitude=0.4, repetitions=3, duration=0.2)
        elif command == "nod":
            self._execute_nod(magnitude=0.3, duration=0.3)

        self.active_priority = self.PRIORITY_IDLE
        self.get_logger().warn(f"GESTURE COMPLETED. Arbitration priority reset to [{self.PRIORITY_IDLE}].")

    def _create_and_publish_trajectory(self, client: ActionClient, joint_names: List[str], positions: List[float], duration_sec: float):
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

    def idle_timer_callback(self):

        if self.active_priority >= self.PRIORITY_FACE_TRACK:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_face_track_time < self.IDLE_AFTER_NO_FACE_SEC:
            if random.random() < self.GLANCE_CHANCE:
                self._execute_glance()
            return

        if self.head_servo_state is None or self.eyes_servo_state is None:
            return

        current_pan = self.head_servo_state[0]

        max_travel = abs(self.HEAD_PAN_MIN_RAD) + abs(self.HEAD_PAN_MAX_RAD)
        max_idle_move = self.IDLE_MAX_MOVE_RATIO * max_travel

        goal_pan = random.uniform(
            max(self.HEAD_PAN_MIN_RAD, current_pan - max_idle_move),
            min(self.HEAD_PAN_MAX_RAD, current_pan + max_idle_move)
        )

        pan_travel = current_pan - goal_pan
        travel_distance = abs(pan_travel)

        movement_ns = int(max(travel_distance / max_idle_move * 3.0, self.IDLE_MIN_DURATION_SEC) * 1e9)

        eye_v = random.gauss(self.EYES_CENTER_V, (self.EYES_CENTER_V + self.EYE_V_MAX_RAD) / 3)

        if pan_travel > 0:
            eye_h = random.uniform(self.EYE_H_MIN_RAD, self.EYE_H_MIN_RAD / 6)
        elif pan_travel < 0:
            eye_h = random.uniform(self.EYE_H_MAX_RAD / 6, self.EYE_H_MAX_RAD)
        else:
            eye_h = random.uniform(self.EYE_H_MIN_RAD, self.EYE_H_MAX_RAD)

        eye_h = self._apply_joint_limits(eye_h, self.EYE_H_MIN_RAD, self.EYE_H_MAX_RAD)
        eye_v = self._apply_joint_limits(eye_v, self.EYE_V_MIN_RAD, self.EYE_V_MAX_RAD)

        self.get_logger().info(
            f"idle: head pan {goal_pan:.3f} rad, eye [{eye_h:.3f}, {eye_v:.3f}] rad"
        )

        self._create_and_publish_trajectory(
            self.head_action_client, self.HEAD_JOINT_NAMES,
            [goal_pan, self.head_servo_state[1]], movement_ns / 1e9
        )

        self._create_and_publish_trajectory(
            self.eyes_action_client, self.EYE_JOINT_NAMES,
            [eye_h, eye_v], movement_ns / 1e9
        )

        next_delay = movement_ns + random.randint(int(0.75 * 1e9), int(1.5 * 1e9))
        self.idle_timer.timer_period_ns = next_delay
        self.idle_timer.reset()

    def _execute_glance(self, delay: float = 0.5):

        if self.eyes_servo_state is None:
            return

        eye_h, eye_v = self._get_random_eye_location(distance=0.5)

        self._create_and_publish_trajectory(
            self.eyes_action_client, self.EYE_JOINT_NAMES,
            [eye_h, eye_v], 0.3
        )

        self._create_and_publish_trajectory(
            self.eyes_action_client, self.EYE_JOINT_NAMES,
            [self.EYES_CENTER_H, self.EYES_CENTER_V], delay
        )

    def _get_random_eye_location(self, distance: float = 0.0) -> tuple[float, float]:

        current_h = self.eyes_servo_state[0] if self.eyes_servo_state else 0.0
        candidates = []

        if self.EYE_H_MIN_RAD < current_h - distance:
            candidates.append(random.uniform(self.EYE_H_MIN_RAD, current_h - distance))
        if self.EYE_H_MAX_RAD > current_h + distance:
            candidates.append(random.uniform(current_h + distance, self.EYE_H_MAX_RAD))

        if not candidates:
            candidates.append(current_h)

        random_h = random.choice(candidates)
        random_v = random.uniform(self.EYE_V_MIN_RAD, self.EYE_V_MAX_RAD)
        return random_h, random_v

    def _center_eyes(self, duration_sec: Optional[float] = None):

        d = duration_sec if duration_sec else 0.3
        self._create_and_publish_trajectory(
            self.eyes_action_client, self.EYE_JOINT_NAMES,
            [self.EYES_CENTER_H, self.EYES_CENTER_V], d
        )

    def _execute_shake(self, magnitude: float, repetitions: int, duration: float):
        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Cannot shake: servo state unknown")
            return

        current_pan = self.head_servo_state[0]
        current_eye_h = self.eyes_servo_state[0]

        for _ in range(repetitions):
            target_r = self._apply_joint_limits(current_pan + magnitude, self.HEAD_PAN_MIN_RAD, self.HEAD_PAN_MAX_RAD)
            eye_r = self._apply_joint_limits(current_eye_h - magnitude, self.EYE_H_MIN_RAD, self.EYE_H_MAX_RAD)
            self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [target_r, self.head_servo_state[1]], duration)
            self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [eye_r, self.eyes_servo_state[1]], duration)
            time.sleep(duration + 0.1)

            target_l = self._apply_joint_limits(current_pan - magnitude, self.HEAD_PAN_MIN_RAD, self.HEAD_PAN_MAX_RAD)
            eye_l = self._apply_joint_limits(current_eye_h + magnitude, self.EYE_H_MIN_RAD, self.EYE_H_MAX_RAD)
            self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [target_l, self.head_servo_state[1]], duration)
            self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [eye_l, self.eyes_servo_state[1]], duration)
            time.sleep(duration + 0.1)

        self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [current_pan, self.head_servo_state[1]], duration)
        self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [current_eye_h, self.eyes_servo_state[1]], duration)
        time.sleep(duration + 0.1)

    def _execute_nod(self, magnitude: float, duration: float):
        if self.head_servo_state is None or self.eyes_servo_state is None:
            self.get_logger().warn("Cannot nod: servo state unknown")
            return

        current_pitch = self.head_servo_state[1]
        current_eye_v = self.eyes_servo_state[1]

        target_up = self._apply_joint_limits(current_pitch + magnitude, self.HEAD_PITCH_MIN_RAD, self.HEAD_PITCH_MAX_RAD)
        eye_down = self._apply_joint_limits(current_eye_v - magnitude, self.EYE_V_MIN_RAD, self.EYE_V_MAX_RAD)
        self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [self.head_servo_state[0], target_up], duration)
        self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [self.eyes_servo_state[0], eye_down], duration)
        time.sleep(duration + 0.1)

        target_down = self._apply_joint_limits(current_pitch - magnitude, self.HEAD_PITCH_MIN_RAD, self.HEAD_PITCH_MAX_RAD)
        eye_up = self._apply_joint_limits(current_eye_v + magnitude, self.EYE_V_MIN_RAD, self.EYE_V_MAX_RAD)
        self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [self.head_servo_state[0], target_down], duration)
        self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [self.eyes_servo_state[0], eye_up], duration)
        time.sleep(duration + 0.1)

        self._create_and_publish_trajectory(self.head_action_client, self.HEAD_JOINT_NAMES, [self.head_servo_state[0], current_pitch], duration)
        self._create_and_publish_trajectory(self.eyes_action_client, self.EYE_JOINT_NAMES, [self.eyes_servo_state[0], current_eye_v], duration)
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
