import pytest
import math
import random

from behaviour.movement.node.head_movement_node import HeadMovementNode


class TestApplyJointLimits:
    """Tests for _apply_joint_limits."""

    def setup_method(self):
        self.node = HeadMovementNode()

    def test_within_limits(self):
        result = self.node._apply_joint_limits(0.0, -1.0, 1.0)
        assert result == 0.0

    def test_below_min(self):
        result = self.node._apply_joint_limits(-2.0, -1.0, 1.0)
        assert result == -1.0

    def test_above_max(self):
        result = self.node._apply_joint_limits(2.0, -1.0, 1.0)
        assert result == 1.0

    def test_at_min(self):
        result = self.node._apply_joint_limits(-1.0, -1.0, 1.0)
        assert result == -1.0

    def test_at_max(self):
        result = self.node._apply_joint_limits(1.0, -1.0, 1.0)
        assert result == 1.0


class TestGetRandomEyeLocation:
    """Tests for _get_random_eye_location."""

    def setup_method(self):
        self.node = HeadMovementNode()

    def test_default_distance(self):
        self.node.eyes_servo_state = [0.0, 0.0]
        eye_h, eye_v = self.node._get_random_eye_location(distance=0.0)
        assert isinstance(eye_h, float)
        assert isinstance(eye_v, float)
        assert self.node.EYE_H_MIN_RAD <= eye_h <= self.node.EYE_H_MAX_RAD
        assert self.node.EYE_V_MIN_RAD <= eye_v <= self.node.EYE_V_MAX_RAD

    def test_returns_within_limits(self):
        self.node.eyes_servo_state = [0.1, 0.0]
        for _ in range(50):
            eye_h, eye_v = self.node._get_random_eye_location(distance=0.3)
            assert self.node.EYE_H_MIN_RAD <= eye_h <= self.node.EYE_H_MAX_RAD
            assert self.node.EYE_V_MIN_RAD <= eye_v <= self.node.EYE_V_MAX_RAD

    def test_current_position_fallback(self):
        self.node.eyes_servo_state = [0.0, 0.0]
        self.node.EYE_H_MIN_RAD = -0.05
        self.node.EYE_H_MAX_RAD = 0.05
        eye_h, _ = self.node._get_random_eye_location(distance=0.5)
        assert eye_h == 0.0

    def test_eye_v_range(self):
        self.node.eyes_servo_state = [0.0, 0.0]
        for _ in range(100):
            _, eye_v = self.node._get_random_eye_location(distance=0.0)
            assert self.node.EYE_V_MIN_RAD <= eye_v <= self.node.EYE_V_MAX_RAD


class TestPriorityArbitration:
    """Tests for priority-based arbitration."""

    def setup_method(self):
        self.node = HeadMovementNode()
        self.node.head_servo_state = [0.0, 0.0]
        self.node.eyes_servo_state = [0.0, 0.0]

    def test_lower_priority_ignored(self):
        self.node.active_priority = self.node.PRIORITY_FACE_TRACK
        goal = MockGoal(priority=self.node.PRIORITY_IDLE, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_FACE_TRACK

    def test_higher_priority_accepted(self):
        self.node.active_priority = self.node.PRIORITY_IDLE
        goal = MockGoal(priority=self.node.PRIORITY_FACE_TRACK, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_IDLE

    def test_gesture_priority_blocks_lower(self):
        self.node.active_priority = self.node.PRIORITY_GESTURE
        goal = MockGoal(priority=self.node.PRIORITY_FACE_TRACK, head_yaw_pan_target=0.5)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority == self.node.PRIORITY_GESTURE

    def test_no_active_priority_accepts_any(self):
        self.node.active_priority = self.node.NO_ACTIVE_PRIORITY
        goal = MockGoal(priority=self.node.PRIORITY_IDLE, head_yaw_pan_target=0.1)
        self.node.move_goal_callback(goal)
        assert self.node.active_priority != self.node.NO_ACTIVE_PRIORITY


class TestTransformCameraAngle:
    """Tests for _transform_camera_angle_to_absolute_target."""

    def setup_method(self):
        self.node = HeadMovementNode()
        self.node.head_servo_state = [0.5, 0.2]
        self.node.eyes_servo_state = [-0.1, 0.05]

    def test_head_target_calculation(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            head_yaw_pan_target=0.3,
            head_pitch_tilt_vertical_target=0.1,
        )
        head_targets, eye_targets, action_type = self.node._transform_camera_angle_to_absolute_target(goal)
        assert action_type == 1
        assert head_targets[0] == pytest.approx(0.5 + 0.3 * self.node.COEFF_HEAD_PAN)

    def test_eye_target_calculation(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            eye_shift_horizontal_target=0.2,
            eye_shift_vertical_target=0.1,
        )
        head_targets, eye_targets, action_type = self.node._transform_camera_angle_to_absolute_target(goal)
        assert action_type == 2
        assert eye_targets[0] == pytest.approx(-0.1 + 0.2 * self.node.COEFF_EYE_HORIZONTAL)

    def test_no_movement_returns_zero(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
        )
        head_targets, eye_targets, action_type = self.node._transform_camera_angle_to_absolute_target(goal)
        assert action_type == 0

    def test_head_limit_clamping(self):
        goal = MockGoal(
            priority=self.node.PRIORITY_FACE_TRACK,
            head_yaw_pan_target=10.0,
        )
        head_targets, _, _ = self.node._transform_camera_angle_to_absolute_target(goal)
        assert self.node.HEAD_PAN_MIN_RAD <= head_targets[0] <= self.node.HEAD_PAN_MAX_RAD


class MockGoal:
    """Minimal mock for HeadMovementGoal."""

    def __init__(self, priority=1, head_yaw_pan_target=0.0, head_pitch_tilt_vertical_target=0.0,
                 eye_shift_horizontal_target=0.0, eye_shift_vertical_target=0.0,
                 duration_nanosecs=0):
        self.priority = priority
        self.head_yaw_pan_target = head_yaw_pan_target
        self.head_pitch_tilt_vertical_target = head_pitch_tilt_vertical_target
        self.eye_shift_horizontal_target = eye_shift_horizontal_target
        self.eye_shift_vertical_target = eye_shift_vertical_target
        self.duration_nanosecs = duration_nanosecs
