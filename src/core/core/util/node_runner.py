"""
node_runner.py

This module provides a helper function to start rclpy and run a Node
"""
import rclpy

def run_node(node_class):
    """
    Helper function to run a node
    """
    rclpy.init()
    node = node_class()
    
    try:
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
