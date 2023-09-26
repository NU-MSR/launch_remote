#!/usr/bin/env python3

import os
from platform import node as get_host_name

import rclpy
from ros2pkg.api import get_prefix_path
from rclpy.node import Node
from launch_remote_interfaces.msg import LaunchFile as LaunchFileMsg
from launch_remote_interfaces.srv import Launch as LaunchSrv

SOURCE_CMD = 'source ${install_dir}${source_file}'
UNDERLAY_INSTALL_DIR = get_prefix_path('rclpy')

LAUNCH_CMD = 'ros2 launch ${package} ${launch_file}'

class LaunchFile:
    def __init__(self, msg : LaunchFileMsg):
        self._msg = msg
        self._compose_cmd()

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

        # TODO(ngmor) remove
        print(SOURCE_CMD)
        print(self._source_cmd)

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

        # TODO(ngmor) remove
        print(LAUNCH_CMD)
        print(self._launch_cmd)

        # Full command
        self._cmd = self._source_cmd + ' && ' + self._launch_cmd

        # TODO(ngmor) remove
        print(self._cmd)


class LaunchManager(Node):
    def __init__(self):
        super().__init__("launch_manager", namespace=get_host_name())


def entry(args=None):
    """Spin the node."""
    # TODO(anyone) this is untested on Windows
    # https://stackoverflow.com/questions/1325581/how-do-i-check-if-im-running-on-windows-in-python
    global SOURCE_CMD
    if os.name == 'nt':
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '\setup.bat')
    else:
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '/setup.bash')

    rclpy.init(args=args)
    launch_manager_node = LaunchManager()
    rclpy.spin(launch_manager_node)
    rclpy.shutdown()

if __name__ == '__main__':
    entry()