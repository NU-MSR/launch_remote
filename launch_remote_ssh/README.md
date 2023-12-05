# launch_remote_ssh
Launch launch files (and nodes) remotely via a screen session over SSH. Requires SSH keys to be set up.

## Installation
Releasing this as a Debian package is currently a work in progress. For now, you will have to build this package from source.

Clone it into the `src` directory of your workspace root directory (`ws`). Then from `ws` use `colcon build`, like any other ROS 2 package.

This package is currently supported for ROS 2 Iron and Rolling.

### Basic Overview
This package allows users to execute processes remotely on other machines by starting a screen session. The screen session SSH's into the remote machine and uses the ROS 2 CLI to run commands (for example, launching launch files or running nodes). The output of the remote session is redirected to the terminal that was used to launch the launch file.

For every remote process that is run, a `remote_process_handler` node is also started. This node waits for the launch file to be terminated, then sends a SIGINT to the remote process and terminates the screen session.

## Usage
### launch_remote_ssh Python Module
This module implements actions and substitutions based on [ros2/launch](https://github.com/ros2/launch). It also extends the `launch.frontend.launch_extension` entry points group, meaning that exposed actions/substitutions are available in frontend launch files (XML/YAML) as well as Python.

#### Source Paths
This package assumes that the remote machine already has an install space (or multiple) that can be sourced for any custom packages. The absolute path to the `setup.bash` files for those install spaces should be provided as `source_path` arguments in remote launching actions (as shown below).

If all install spaces are already sourced when SSH'ing into the remote machine, then this `source_path` argument can be omitted.

#### Launching Launch Files Remotely
The most well-tested capability of this module is remotely launching launch files. This can be done with:

Python:
```
LaunchRemoteSSH(
    user='my_user',
    machine='my_machine',
    package='my_package',
    file='my_launch_file.launch',
    source_paths=[
        '/example/path/to/workspace/install/setup.bash',
    ],
    launch_arguments=[
        ('my_param1', 'my_param1_value'),
        ('my_param2', 'my_param2_value'),
    ],
),
```

XML:
```
<launch_remote_ssh
    user="my_user"
    machine="my_machine"
    pkg="my_package"
    file="my_launch_file.launch"
>
    <source_path path="/example/path/to/workspace/install/setup.bash"/>
    <arg name="my_param1" value="my_param1_value"/>
    <arg name="my_param2" value="my_param2_value"/>
</launch_remote_ssh>
```

YAML:
```
- launch_remote_ssh:
    user: 'my_user'
    machine: 'my_machine'
    pkg: 'my_package'
    file: 'my_launch_file.launch'
    source_path:
    -
      path: '/example/path/to/workspace/install/setup.bash'
    arg:
    -
      name: 'my_param1'
      value: 'my_param1_value'
    -
      name: 'my_param2'
      value: 'my_param2_value'
```

If included in launch files of their respective types, these actions will launch the `my_launch_file.launch` file from the `my_package` package by SSH'ing with `my_user@my_machine`. The workspace will be sourced with `source /example/path/to/workspace/install/setup.bash` and launch arguments `my_param1:=my_param1_value` and `my_param2:=my_param2_value` will be passed.

See the `test_launch_entry_point.launch.${format}` files in the [launch test directory](test/launch/) for more examples.

#### Running Nodes Remotely
#### Other Processes

### Flexible Launch Files

### Utilities

## Reference
The original screen workaround that this package was based on is proposed [here](https://answers.ros.org/question/364152/remotely-launch-nodes-in-ros2/). This package builds around and improves on that workaround.

## License
This package is released under the [BSD 3-Clause License](LICENSE).