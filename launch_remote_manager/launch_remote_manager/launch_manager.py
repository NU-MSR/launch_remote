#!/usr/bin/env python3

import os
from platform import node as get_host_name
from subprocess import Popen
import signal
import time
from psutil import Process

import rclpy
from rclpy.node import Node
from ros2pkg.api import get_prefix_path
from launch_remote_interfaces.msg import LaunchFile as LaunchFileMsg
from launch_remote_interfaces.srv import Launch as LaunchSrv
from launch_remote_interfaces.srv import Stop as StopSrv

SOURCE_CMD = 'source ${install_dir}${source_file}'
UNDERLAY_INSTALL_DIR = get_prefix_path('rclpy')

LAUNCH_CMD = 'ros2 launch ${package} ${launch_file}'

SHELL_EXECUTABLE = ''

SIGINT = 0

INTERRUPT_TIMEOUT = 5.0
TERMINATE_TIMEOUT = 10.0

class SignalTimes:
    interrupt = None
    terminate = None

class LaunchFile:
    def __init__(self, msg : LaunchFileMsg):
        self._msg = msg
        self._compose_cmd()
        self._popen = None
        self._process = None
        self._children = None
        self._parent_stop_times = SignalTimes()
        self._children_stop_times = SignalTimes()

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
        # TODO(ngmor) error handling
        self._popen = Popen(self._cmd, shell=True, executable=SHELL_EXECUTABLE)
        self._process = Process(self._popen.pid)

    def pid(self):
        if self._popen is None:
            return None
        else:
            return self._popen.pid

    def stop(self):
        if self._children is None:
            if self._popen is None:
                return True

            self._children = []
            for child in self._process.children(recursive=True):
                self._children.append(child)
        
        parent_stopped = True
        children_stopped = True

        if self._popen.poll() is None:
            parent_stopped = False

        for child in self._children:
            if child.is_running():
                children_stopped = False
                break

        if parent_stopped and children_stopped:
            return True
        
        # Stop children
        # Escalates if processes survive different termination signals
        if not children_stopped:
            if self._children_stop_times.interrupt is None:
                for child in self._children:
                    if child.is_running():
                        child.send_signal(SIGINT)
                self._children_stop_times.interrupt = time.time()
                return False
            
            if self._children_stop_times.terminate is None:
                if (time.time() - self._children_stop_times.interrupt) < INTERRUPT_TIMEOUT:
                    return False
                else:
                    for child in self._children:
                        if child.is_running():
                            child.terminate()
                    self._children_stop_times.terminate = time.time()
                    return False
                
            if (time.time() - self._children_stop_times.terminate) < TERMINATE_TIMEOUT:
                return False
            else:
                for child in self._children:
                    if child.is_running():
                        child.kill()
                return False

        # Stop parent once children are stopped
        # Escalates if process survives different termination signals
        if not parent_stopped:
            if self._parent_stop_times.interrupt is None:
                print("interrupt")
                self._popen.send_signal(SIGINT)
                self._parent_stop_times.interrupt = time.time()
                return False
            
            if self._parent_stop_times.terminate is None:
                if (time.time() - self._parent_stop_times.interrupt) < INTERRUPT_TIMEOUT:
                    return False
                else:
                    print("terminate")
                    self._popen.terminate()
                    self._parent_stop_times.terminate = time.time()
                    return False
                
            if (time.time() - self._parent_stop_times.terminate) < TERMINATE_TIMEOUT:
                return False
            else:
                print("kill")
                self._popen.kill()
                return False



class LaunchManager(Node):
    def __init__(self):
        super().__init__("launch_manager", namespace=get_host_name())

        # Service Servers
        self._srv_launch = self.create_service(LaunchSrv, 'launch', self._srv_launch_callback)
        self._srv_stop = self.create_service(StopSrv, 'stop', self._srv_stop_callback)

        self._launch_files = {}

    def _srv_launch_callback(self, request : LaunchSrv.Request, response : LaunchSrv.Response):
        # TODO(ngmor) validate input fields

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
    
    def _srv_stop_callback(self, request : StopSrv.Request, response : StopSrv.Response):
        # TODO (ngmor)
        if request.pid not in self._launch_files.keys():
            response.result = StopSrv.Response.NOT_FOUND
            return response
        
        launch_file = self._launch_files.pop(request.pid)

        self.get_logger().info(
            'Stopping process:\nPID: ' + str(launch_file.pid()) + '\nCommand:\n' + launch_file.cmd()
        )

        # This method will return true when the process is stopped
        while not launch_file.stop():
            # TODO(ngmor) timeout?
            pass

        response.result = StopSrv.Response.SUCCESS

        return response

def entry(args=None):
    """Spin the node."""
    # TODO(anyone) this is untested on Windows
    # https://stackoverflow.com/questions/1325581/how-do-i-check-if-im-running-on-windows-in-python
    global SOURCE_CMD
    global SHELL_EXECUTABLE
    global SIGINT
    if os.name == 'nt':
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '\setup.bat')
        SHELL_EXECUTABLE = 'cmd.exe'
        SIGINT = signal.CTRL_C_EVENT
    else:
        SOURCE_CMD = SOURCE_CMD.replace('${source_file}', '/setup.bash')
        SHELL_EXECUTABLE = '/bin/bash'
        SIGINT = signal.SIGINT

    # TODO(ngmor) consider try/catch
    # TODO(ngmor) explicitly stop all processes on exit
    rclpy.init(args=args)
    launch_manager_node = LaunchManager()
    rclpy.spin(launch_manager_node)
    rclpy.shutdown()

if __name__ == '__main__':
    entry()