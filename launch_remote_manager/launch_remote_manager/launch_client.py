
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

def get_required_parameter_value(node, name):
    if node.has_parameter(name):
        return node.get_parameter(name).get_parameter_value()
    else:
        raise Exception(f'Required parameter "{name}" not provided.')

def entry(args=None):
    rclpy.init(args=args)

    launch_client_node = Node(
        "launch_client",
        automatically_declare_parameters_from_overrides=True
    )

    # Namespace
    if launch_client_node.get_namespace() == '/':
        raise Exception(
            'Remote machine name must be provided as a namespace to the launch client node'
        )

    # Parameters
    package = get_required_parameter_value(launch_client_node, 'package').string_value
    file = get_required_parameter_value(launch_client_node, 'file').string_value

if __name__ == '__main__':
    entry()