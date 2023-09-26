#!/usr/bin/env python3

import rclpy
from platform import node as get_host_name
from rclpy.node import Node
from launch_remote_interfaces.msg import LaunchFile as LaunchFileMsg

class LaunchFile:
    def __init__(self, msg : LaunchFileMsg):
        self._msg = msg
        self._compose_cmd()

    def _compose_cmd(self):
        self._cmd = 'ros2 launch ' + self._msg.package + ' ' + self._msg.name

        for arg in self._msg.arguments:
            self._cmd += ' ' + arg.name + ':=' + arg.value

        # TODO(nmorales) remove
        print(self._cmd)

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