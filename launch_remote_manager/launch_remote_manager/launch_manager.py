#!/usr/bin/env python3

import os
from platform import node as get_host_name
from subprocess import Popen

import rclpy
from rclpy.node import Node
from ros2pkg.api import get_prefix_path
from launch_remote_interfaces.msg import LaunchFile as LaunchFileMsg
from launch_remote_interfaces.srv import Launch as LaunchSrv

SOURCE_CMD = 'source ${install_dir}${source_file}'
UNDERLAY_INSTALL_DIR = get_prefix_path('rclpy')

LAUNCH_CMD = 'ros2 launch ${package} ${launch_file}'

SHELL_EXECUTABLE = ''

class LaunchFile:
    def __init__(self, msg : LaunchFileMsg):
        self._msg = msg
        self._compose_cmd()
        self._process = None

    def _compose_cmd(self):
        # Source command
        if len(self._msg.install_dirs) == 0:
            self._source_cmd = SOURCE_CMD.replace('${install_dir}', UNDERLAY_INSTALL_DIR)
        else:
            self._source_cmd = ''

            first = False

            for install_dir in self._msg.install_dirs:
                if not first:
                    self._source_cmd += ' && '

                self._source_cmd += SOURCE_CMD.replace('${install_dir}', install_dir)

        # Launch command
        self._launch_cmd = \
            LAUNCH_CMD.replace(
                '${package}',
                self._msg.package
            ).replace(
                '${launch_file}',
                self._msg.name
            )

        for arg in self._msg.arguments:
            self._launch_cmd += ' ' + arg.name + ':=' + arg.value

        # Full command
        self._cmd = self._source_cmd + ' && ' + self._launch_cmd

    def cmd(self):
        return self._cmd
    
    def run(self):
        self._process = Popen(self._cmd, shell=True, executable=SHELL_EXECUTABLE)

    def pid(self):
        if self._process is None:
            return None
        else:
            return self._process.pid

    def stop(self):
        # TODO(ngmor)
        pass



class LaunchManager(Node):
    def __init__(self):
        super().__init__("launch_manager", namespace=get_host_name())

        # Services
        self._srv_launch = self.create_service(LaunchSrv, 'launch', self._srv_launch_callback)

        self._launch_files = {}

    def _srv_launch_callback(self, request : LaunchSrv.Request, response : LaunchSrv.Response):
        launch_file = LaunchFile(request.file)

        response.command = launch_file.cmd()

        self.get_logger().info('Running command:\n' + launch_file.cmd())

        launch_file.run()

        if launch_file.pid() is None:
            
            self.get_logger().error('Launch file failed to launch:\n' + launch_file.cmd())
            # TODO(ngmor) handle error

        else:
            response.pid = launch_file.pid()

            # Store launch file
            self._launch_files[launch_file.pid()] = launch_file

            # TODO (ngmor) publishing

        return response

def entry(args=None):
    """Spin the node."""
    # TODO(anyone) this is untested on Windows
    # https://stackoverflow.com/questions/1325581/how-do-i-check-if-im-running-on-windows-in-python
    global SOURCE_CMD
    global SHELL_EXECUTABLE
    if os.name == 'nt':
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '\setup.bat')
        SHELL_EXECUTABLE = 'cmd.exe'
    else:
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '/setup.bash')
        SHELL_EXECUTABLE = '/bin/bash'

    # TODO(ngmor) consider try/catch
    # TODO(ngmor) explicitly stop all processes on exit
    rclpy.init(args=args)
    launch_manager_node = LaunchManager()
    rclpy.spin(launch_manager_node)
    rclpy.shutdown()

if __name__ == '__main__':
    entry()