from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="behaviour",
            executable="face_track",
            name="face_track_node",
            output="screen",
        ),
        Node(
            package="behaviour",
            executable="head_movement",
            name="head_movement_node",
            output="screen",
        ),
    ])
