"""
i2head_bridge_node.py

ROS2-to-Arduino serial bridge for InMoov i2Head PCA9685 servo control.

Subscribes to a JointTrajectory topic and forwards joint positions
as serial commands to the Arduino: channel:angle,channel:angle,...

Usage:
  ros2 run i2head_bridge i2head_bridge_node
  ros2 run i2head_bridge i2head_bridge_node --ros-args -p serial_port:=/dev/ttyACM1
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports
import yaml
import os

from ament_index_python.packages import get_package_share_directory


class I2HeadBridgeNode(Node):
    """
    Bridges JointTrajectory commands to Arduino over serial for PCA9685 servo control.
    """

    def __init__(self):
        super().__init__("i2head_bridge")

        # --- Parameters ---
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("joint_config", "")
        self.declare_parameter("topic_name", "/i2head/joint_commands")
        self.declare_parameter("feedback_topic", "/i2head/feedback")

        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        joint_config_path = self.get_parameter("joint_config").value
        topic_name = self.get_parameter("topic_name").value
        feedback_topic = self.get_parameter("feedback_topic").value

        # --- Load joint-to-channel mapping ---
        self.joint_map = {}
        if joint_config_path and os.path.isfile(joint_config_path):
            with open(joint_config_path) as f:
                config = yaml.safe_load(f)
                for joint_name, cfg in config.items():
                    if "channel" in cfg:
                        self.joint_map[joint_name] = cfg["channel"]
            self.get_logger().info(f"Loaded {len(self.joint_map)} joints from {joint_config_path}")
        else:
            self.get_logger().warn("No joint config loaded, will use direct channel:angle format")

        # --- Serial connection ---
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Serial port {serial_port} opened at {baud_rate} baud")
        except serial.SerialException as e:
            self.serial = None
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")

        # --- Publisher (feedback) ---
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)

        # --- Subscriber (joint commands) ---
        self.subscription = self.create_subscription(
            JointTrajectory,
            topic_name,
            self.joint_command_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {topic_name}")

    def joint_command_callback(self, msg: JointTrajectory):
        """Convert JointTrajectory to serial command and send to Arduino."""
        if self.serial is None:
            return

        if not msg.points:
            return

        # Build channel:angle pairs from the trajectory message
        commands = []
        for i, joint_name in enumerate(msg.joint_names):
            channel = self.joint_map.get(joint_name, None)
            if channel is None:
                self.get_logger().warn(f"Unknown joint: {joint_name}, skipping")
                continue

            angle = msg.points[0].positions[i] if i < len(msg.points[0].positions) else 0.0

            # Validate angle range
            angle_deg = int(round(angle))
            angle_deg = max(0, min(180, angle_deg))

            commands.append(f"{channel}:{angle_deg}")

        if not commands:
            return

        command_str = ",".join(commands) + "\n"
        self.get_logger().debug(f"Sending: {command_str.strip()}")

        try:
            self.serial.write(command_str.encode())

            feedback = self.serial.readline().decode().strip()
            if feedback:
                fb_msg = String()
                fb_msg.data = feedback
                self.feedback_pub.publish(fb_msg)
                self.get_logger().info(f"Arduino: {feedback}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = I2HeadBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
