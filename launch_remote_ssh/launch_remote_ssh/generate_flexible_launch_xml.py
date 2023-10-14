#!/usr/bin/env python3
from typing import Optional

import xml.etree.ElementTree as ET
import argparse
import os

def create_arg_tag(name: str, default: Optional[str]=None):
    out = ET.Element('arg')
    out.attrib['name'] = name
    if default is not None:
        out.attrib['default'] = default
    return out

def create_let_tag(name: str, value: str):
    out = ET.Element('let')
    out.attrib['name'] = name
    out.attrib['value'] = value
    return out

def generate_flexible_launch_xml(package_name, in_file_path, out_file_path):
    launch_file_name = os.path.basename(in_file_path)
    launch_file = ET.parse(in_file_path)
    launch_file_root = launch_file.getroot()

    flexible_launch_root = ET.Element('launch')

    # Create launch_remote_ssh tag
    launch_remote_ssh_tag = ET.Element('launch_remote_ssh')
    launch_remote_ssh_tag.attrib['user'] = '$(var user)'
    launch_remote_ssh_tag.attrib['machine'] = '$(var machine)'
    launch_remote_ssh_tag.attrib['pkg'] = package_name
    launch_remote_ssh_tag.attrib['file'] = launch_file_name
    launch_remote_ssh_tag.attrib['if'] = \
        '$(and $(not $(var run_local)) $(not $(var user_not_specified_condition)))'

    # Create launch_local (include) tag
    launch_local_tag = ET.Element('include')
    launch_local_tag.attrib['file'] = f'$(find-pkg-share {package_name})/launch/{launch_file_name}'
    launch_local_tag.attrib['if'] = '$(var run_local)'

    machine_arg_specified = False
    user_arg_specified = False

    # Process arguments in input launch file
    for arg in launch_file_root.findall('arg'):
        assert 'name' in arg.attrib
        name = arg.attrib['name']

        # Add to flexible launch file
        flexible_launch_root.append(arg)

        if name == 'user':
            user_arg_specified = True
        elif name == 'machine':
            machine_arg_specified = True
        elif 'remote_source_path' in name:
            source_path_tag = ET.SubElement(launch_remote_ssh_tag, 'source_path')
            source_path_tag.attrib['path'] = f'$(var {name}'

        # Create an argument subtag for local and remote launch tags
        arg_subtag = create_let_tag(name, f'$(var {name})')
        arg_subtag.tag = 'arg'

        # Pass through to launch_remote_ssh tag
        launch_remote_ssh_tag.append(arg_subtag)

        # Pass through to launch_local tag
        launch_local_tag.append(arg_subtag)

    # Add user and machine args and logic to handle them
    if not user_arg_specified:  # if custom user arg is not specified, append default
        user_arg = create_arg_tag('user', 'USER_NOT_SPECIFIED')
        flexible_launch_root.append(user_arg)
    if not machine_arg_specified:  # if custom machine arg is not specified, append default
        machine_arg = create_arg_tag('machine', 'localhost')
        flexible_launch_root.append(machine_arg)
    run_local_arg = create_let_tag('run_local', '$(equals $(var machine) localhost)')
    flexible_launch_root.append(run_local_arg)
    user_not_specified_msg = create_let_tag(
        'user_not_specified_msg',
        "If 'machine' argument is not 'localhost', a user must be provided"
    )
    flexible_launch_root.append(user_not_specified_msg)
    user_not_specified_condition = create_let_tag(
        'user_not_specified_condition',
        '$(and $(not $(var run_local)) $(equals $(var user) USER_NOT_SPECIFIED))'
    )
    flexible_launch_root.append(user_not_specified_condition)
    user_not_specified_log = ET.Element('log')
    user_not_specified_log.attrib['message'] = '$(var user_not_specified_msg)'
    user_not_specified_log.attrib['if'] = '$(var user_not_specified_condition)'
    flexible_launch_root.append(user_not_specified_log)
    user_not_specified_shutdown = ET.Element('shutdown')
    user_not_specified_shutdown.attrib['reason'] = '$(var user_not_specified_msg)'
    user_not_specified_shutdown.attrib['if'] = '$(var user_not_specified_condition)'
    flexible_launch_root.append(user_not_specified_shutdown)

    # Append remote and local launch tags
    flexible_launch_root.append(launch_remote_ssh_tag)
    flexible_launch_root.append(launch_local_tag)

    # Read/create output paths
    out_path = os.path.abspath(out_file_path)
    out_dir = os.path.dirname(out_file_path)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # Create tree
    flexible_launch_file = ET.ElementTree(flexible_launch_root)

    # Format nicely
    ET.indent(flexible_launch_file, space="    ", level=0)

    with open(out_path, 'w') as out_file:
        flexible_launch_file.write(out_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Automatically generate a remote XML launch file from an XML launch file'
    )
    parser.add_argument(
        'package_name',
        help='Package name of launch files'
    )
    parser.add_argument(
        'in_file',
        help='Path to input XML launch file to convert to a remote launch file',
    )
    parser.add_argument(
        'out_file',
        help='Path to output XML remote launch file'
    )

    args = parser.parse_args()

    generate_flexible_launch_xml(args.package_name, args.in_file, args.out_file, )