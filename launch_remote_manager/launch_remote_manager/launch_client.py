
import os
import signal

import rclpy
from rclpy.node import Node
from launch_remote_interfaces.msg import LaunchArgument as LaunchArgumentMsg
from launch_remote_interfaces.srv import Launch as LaunchSrv
from launch_remote_interfaces.srv import Stop as StopSrv

def get_required_parameter_value(node, name):
    if node.has_parameter(name):
        return node.get_parameter(name).get_parameter_value()
    else:
        raise Exception(f'Required parameter "{name}" not provided.')

def signal_handler(sig, frame):
    print('SIGINT received, shutting down launched nodes')

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
    install_dirs = \
        get_required_parameter_value(launch_client_node, 'install_dirs').string_array_value
    if '' in install_dirs:
        install_dirs.pop(install_dirs.index(''))
    
    arguments = []

    # Get all provided launch arguments
    i = 0
    while(True):
        param_name = f'argname{i}'
        param_val = f'argval{i}'
        if launch_client_node.has_parameter(param_name):
            argname = \
                launch_client_node.get_parameter(param_name).get_parameter_value().string_value

            if not (launch_client_node.has_parameter(param_val)):
                raise Exception(
                    'Mismatch in number of provided parameter name/value pairs:\n'
                    f'{param_name}: {argname} did not have a value provided as {param_val}'
                )
            
            argval = \
                launch_client_node.get_parameter(param_val).get_parameter_value().string_value
            
            msg = LaunchArgumentMsg()
            msg.name = argname
            msg.value = argval
            arguments.append(msg)
        else:
            break
        i += 1

    # Service Clients
    cli_launch_srv = \
        launch_client_node.create_client(LaunchSrv, launch_client_node.get_namespace() + '/launch')
    cli_stop_srv = \
        launch_client_node.create_client(StopSrv, launch_client_node.get_namespace() + '/stop')


    # Construct launch file service request from input parameters
    launch_request = LaunchSrv.Request()
    launch_request.file.package = package
    launch_request.file.file = file
    launch_request.file.arguments = arguments
    launch_request.file.install_dirs = install_dirs

    # Wait for service to exist
    while rclpy.ok() and not cli_launch_srv.wait_for_service(1):
        rclpy.spin(launch_client_node)

    if not rclpy.ok():
        rclpy.shutdown()
        return
    
    # Call service
    launch_future = cli_launch_srv.call_async(launch_request)

    rclpy.spin_until_future_complete(launch_client_node, launch_future)

    if not rclpy.ok():
        rclpy.shutdown()
        return

    launch_response = launch_future.result()

    # TODO(ngmor) handle errors in this response

    launch_client_node.get_logger().info(
        f'Launched following command with PID {launch_response.pid}'
        f' on {launch_client_node.get_namespace()}:\n' + launch_response.command
    )

    # Ignore SIGINTs
    # TODO(anyone) this is untested on Windows
    if os.name == 'nt':
        signal.signal(signal.CTRL_C_EVENT, signal_handler)
        # https://stackoverflow.com/questions/9784972/python-signal-pause-equivalent-on-windows
        os.system('pause')
    else:
        signal.signal(signal.SIGINT, signal_handler)
        signal.pause()

    # Stop process now that this has been interrupted
    stop_request = StopSrv.Request()
    stop_request.pid = launch_response.pid

    stop_future = cli_stop_srv.call_async(stop_request)

    while not stop_future.done():
        rclpy.spin_once(launch_client_node)

    stop_response = stop_future.result()

    if stop_response.result == StopSrv.Response.SUCCESS:
        launch_client_node.get_logger().info('Launch file stopped successfully')
    elif stop_response.result == StopSrv.Response.NOT_FOUND:
        launch_client_node.get_logger().warn('Launch file process not found to stop')
    else:
        launch_client_node.get_logger().warn('Unrecognized return code')

if __name__ == '__main__':
    entry()