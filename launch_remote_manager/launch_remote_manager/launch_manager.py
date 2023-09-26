#!/usr/bin/env python3

import rclpy
from platform import node as get_host_name
from rclpy.node import Node

class LaunchManager(Node):
    def __init__(self):
        super().__init__("launch_manager", namespace=get_host_name())

def entry(args=None):
    """Spin the node."""
    rclpy.init(args=args)
    launch_manager_node = LaunchManager()
    rclpy.spin(launch_manager_node)
    rclpy.shutdown()

if __name__ == '__main__':
    entry()