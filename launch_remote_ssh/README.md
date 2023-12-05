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
Running individual nodes remotely is less well tested, but the capability is still available.

Python:
```
NodeRemoteSSH(
    user='my_user',
    machine='my_machine',
    package='my_package',
    executable='my_node',
    source_paths=[
        '/example/path/to/workspace/install/setup.bash',
    ],
),
```

XML:
```
<node_remote_ssh
    user="my_user"
    machine="my_machine"
    pkg="my_package"
    exec="my_node"
>
    <source_path path="/example/path/to/workspace/install/setup.bash"/>
</node_remote_ssh>
```

YAML:
```
- node_remote_ssh:
    user: "my_user"
    machine: "my_machine"
    pkg: "my_package"
    exec: "my_node"
    source_path:
    -
      path: '/example/path/to/workspace/install/setup.bash'
```

If included in launch files of their respective types, these actions will run the `my_node` node from the `my_package` package by SSH'ing with `my_user@my_machine`. The workspace will be sourced with `source /example/path/to/workspace/install/setup.bash`. Other arguments like parameters, remaps, etc should be supported.

See the `test_node_entry_point.launch.${format}` files in the [node test directory](test/node/) for more examples.

#### Other Processes
The [`LaunchRemoteSSH`](launch_remote_ssh/launch_remote_ssh.py) and [`NodeRemoteSSH`](launch_remote_ssh/node_remote_ssh.py) actions use [`ExecuteProcessRemoteSSH`](launch_remote_ssh/execute_process_remote_ssh.py) under the hood. Theoretically this could be used to launch any remote process, but it's only really been tested with the launch and node actions.

#### Substitutions
Some substitutions are provided for utility.
- [FindPackagePrefixRemote](launch_remote_ssh/find_package_remote.py) (find-pkg-prefix-remote) - doesn't actually find the remote package, since all substitutions occur on the local system before any SSH happens at all. Instead, it's just a nice path join substitution between the source path and the package prefix.
- [FindPackageShareRemote](launch_remote_ssh/find_package_remote.py) (find-pkg-share-remote) - doesn't actually find the remote package, since all substitutions occur on the local system before any SSH happens at all. Instead, it's just a nice path join substitution between the source path and the package share.
- [ReplaceTextSubstitution](launch_remote_ssh/replace_text_substitution.py) (replace-text)- replaces text1 with text2 in an input string. Used internally in this package but also exposed.

### Utilities
#### Copying Install Space
The `launch_remote_ssh` module provides [utilities](launch_remote_ssh/install_remote_ssh.py) for copying the install space to another machine. The `copy_install_space.py` script is provided for this. To get usage info, run:

```
ros2 run launch_remote_ssh copy_install_space.py --help
```

**Note**: It's generally discouraged to run this script in a launch file, because it's a good way to create a system that works well during development, but not deployment. Ignore the fact that I have done this in the tests for this repository...

### Flexible Launch Files
"Flexible launch files" allow a user to generate launch files that can switch between launching locally and remotely simply by passing in different launch arguments. They can be very helpful for a launch file that might need to run in either context.

At this time, flexible launch files are only supported for XML launch files.

#### Core Launch File
The first step to setting up flexible launch files is to define a "core" XML launch file. This is just like any other launch file, and defines what the flexible launch file will launch.

The only difference is that the name of the launch file should contain `.core.`, for example:
```
example.core.launch
example.core.launch.xml
example.launch.core.xml
```

#### CMake Setup
To generate flexible launch files, two things should be included in the `CMakeLists.txt`:

1. Install the core launch files like normal:
```
# Typical CMake for installing a directory of launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

2. Call the generation function:
```
launch_remote_ssh_generate_flexible_launch_xmls(
  DESTINATION share/${PROJECT_NAME}/launch
  PACKAGE ${PROJECT_NAME}
  DIRECTORIES
  launch
)
```
or
```
launch_remote_ssh_generate_flexible_launch_xmls(
  DESTINATION share/${PROJECT_NAME}/launch
  PACKAGE ${PROJECT_NAME}
  FILES
  launch/example.core.launch.xml
  launch/example.core.launch
)
```

See the [CMake source](cmake/generate_flexible_launch_xmls.cmake) for more documentation.

Now whenever the ROS package is built, flexible launch XMLs will be generated!

#### Using Flexible Launch Files
The generated flexible launch files will have the same file name as the core launch file, without the `.core.` (ex: `example.core.launch` -> `example.launch`).

These can be launched like any other launch file, and the arguments from the core launch file are still valid. By default, the flexible launch files will run on the local machine. However, launch arguments can be specified to determine where they run:
- `flexible_launch.user` - sets the user for the SSH command
- `flexible_launch.machine` - sets the machine for the ssh command.

So to run a launch file remotely with `my_user@my_machine`, use:
```
ros2 launch my_package example.launch flexible_launch.user:=my_user flexible_launch.machine:=my_machine
```

#### Flexible Launch Defaults
Defaults can be specified in the core launch file as well. By including the following tag, a default user and source path can be specified:
```
<flexible_frontend_launch_defaults>
    <user default="default_user_name"/>
    <source_path default="default_source_path"/>
</flexible_frontend_launch_defaults>
```

An arbitrary number of source path defaults can be provided, and these will be added as launch arguments that can be overriden as well.

#### Notes
It is recommended that when including other flexible launch files in a core launch file, always include the core versions of other launch files. This prevents bugs with propagation of the `flexible_launch` launch arguments.

## Reference
The original screen workaround that this package was based on is proposed [here](https://answers.ros.org/question/364152/remotely-launch-nodes-in-ros2/). This package builds around and improves on that workaround.

## License
This package is released under the [BSD 3-Clause License](LICENSE).