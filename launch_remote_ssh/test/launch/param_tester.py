#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ParamTester(Node):
    def __init__(self):
        super().__init__("param_tester")
        self.declare_parameter("param", descriptor=ParameterDescriptor(dynamic_typing=True))

def main(args=None):
    """Spin the node."""
    rclpy.init(args=args)
    this_node = ParamTester()
    rclpy.spin(this_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()