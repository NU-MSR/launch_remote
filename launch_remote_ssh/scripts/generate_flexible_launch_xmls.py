#!/usr/bin/env python3
from typing import Optional

import argparse
import os
import shutil
import xml.etree.ElementTree as ET

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
    ARG_NAMESPACE = 'flexible_launch'

    launch_file_name = os.path.basename(in_file_path)
    launch_file = ET.parse(in_file_path)
    launch_file_root = launch_file.getroot()

    flexible_launch_root = ET.Element('launch')

    # Create launch_remote_ssh tag
    launch_remote_ssh_tag = ET.Element('launch_remote_ssh')
    launch_remote_ssh_tag.attrib['user'] = f'$(var {ARG_NAMESPACE}.user)'
    launch_remote_ssh_tag.attrib['machine'] = f'$(var {ARG_NAMESPACE}.machine)'
    launch_remote_ssh_tag.attrib['pkg'] = package_name
    launch_remote_ssh_tag.attrib['file'] = launch_file_name
    launch_remote_ssh_tag.attrib['if'] = \
        f'$(and $(not $(var {ARG_NAMESPACE}.run_local))' \
        f' $(not $(var {ARG_NAMESPACE}.user_not_specified_condition)))'

    # Create launch_local (include) tag
    launch_local_tag = ET.Element('include')
    launch_local_tag.attrib['file'] = f'$(find-pkg-share {package_name})/launch/{launch_file_name}'
    launch_local_tag.attrib['if'] = f'$(var {ARG_NAMESPACE}.run_local)'

    # Process arguments in input launch file
    flexible_launch_root.append(ET.Comment('User-defined custom launch arguments'))
    for arg in launch_file_root.findall('arg'):
        assert 'name' in arg.attrib
        name = arg.attrib['name']

        # Add to flexible launch file
        flexible_launch_root.append(arg)

        # Create an argument subtag for local and flexible launch tags
        arg_subtag = create_let_tag(name, f'$(var {name})')
        arg_subtag.tag = 'arg'

        # Pass through to launch_remote_ssh tag
        launch_remote_ssh_tag.append(arg_subtag)

        # Pass through to launch_local tag
        launch_local_tag.append(arg_subtag)

    defaults = launch_file_root.findall('flexible_frontend_launch_defaults')
    USER_DEFAULT = 'USER_NOT_SPECIFIED'
    user_default = USER_DEFAULT
    source_path_defaults = []
    if len(defaults) > 1:
        raise Exception(
            f'flexible_frontend_launch_defaults tag found {len(defaults)} times, only 1 expected.'
        )
    elif len(defaults) == 1:
        for default in defaults[0]:
            assert 'default' in default.attrib
            if default.tag == 'user':
                # Error if multiple user defaults are found
                if user_default != USER_DEFAULT:
                    raise Exception(
                        'Multiple \'user\' tags found in flexible_frontend_launch_defaults tag,'
                        ' only 1 expected'
                    )
                user_default = default.attrib['default']
            elif default.tag == 'source_path':
                source_path_defaults.append(default)

    # Add user and machine args and logic to handle them
    flexible_launch_root.append(
        ET.Comment('Boilerplate arguments and logic for handling user/machine arguments')
    )
    flexible_launch_root.append(create_arg_tag(f'{ARG_NAMESPACE}.user', user_default))
    flexible_launch_root.append(create_arg_tag(f'{ARG_NAMESPACE}.machine', 'localhost'))
    flexible_launch_root.append(create_let_tag(
        f'{ARG_NAMESPACE}.run_local',
        f'$(equals $(var {ARG_NAMESPACE}.machine) localhost)'
    ))
    flexible_launch_root.append(create_let_tag(
        f'{ARG_NAMESPACE}.user_not_specified_msg',
        f'If \'{ARG_NAMESPACE}.machine\' argument is not \'localhost\','
        f' \'{ARG_NAMESPACE}.user\' must be provided'
    ))
    flexible_launch_root.append(create_let_tag(
        f'{ARG_NAMESPACE}.user_not_specified_condition',
        f'$(and $(not $(var {ARG_NAMESPACE}.run_local)) '
        f'$(equals $(var {ARG_NAMESPACE}.user) {USER_DEFAULT}))'
    ))
    user_not_specified_log = ET.Element('log')
    user_not_specified_log.attrib['message'] = f'$(var {ARG_NAMESPACE}.user_not_specified_msg)'
    user_not_specified_log.attrib['if'] = f'$(var {ARG_NAMESPACE}.user_not_specified_condition)'
    flexible_launch_root.append(user_not_specified_log)
    user_not_specified_shutdown = ET.Element('shutdown')
    user_not_specified_shutdown.attrib['reason'] = f'$(var {ARG_NAMESPACE}.user_not_specified_msg)'
    user_not_specified_shutdown.attrib['if'] = \
        f'$(var {ARG_NAMESPACE}.user_not_specified_condition)'
    flexible_launch_root.append(user_not_specified_shutdown)

    # Add source path arguments
    for i, source_path in enumerate(source_path_defaults):
        name = f'{ARG_NAMESPACE}.source_path{i}'
        flexible_launch_root.append(create_arg_tag(name, source_path.attrib['default']))
        source_path_tag = ET.SubElement(launch_remote_ssh_tag, 'source_path')
        source_path_tag.attrib['path'] = f'$(var {name})'

    # Append remote and local launch tags
    flexible_launch_root.append(ET.Comment(
        f'Remote launch action (runs if \'{ARG_NAMESPACE}.machine\' argument is not \'localhost\')'
    ))
    flexible_launch_root.append(launch_remote_ssh_tag)
    flexible_launch_root.append(ET.Comment(
        f'Local launch action (runs if \'{ARG_NAMESPACE}.machine\' argument is \'localhost\')'
    ))
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

parser = argparse.ArgumentParser(
    description='Automatically generate flexible XML launch files from a series of core XML launch files'
)

parser.add_argument(
    'package_name',
    help='Package name of launch files'
)

parser.add_argument(
    'out_dir',
    help='Directory to output XML remote launch files'
)
parser.add_argument(
    '-f', '--files',
    help='Input files: path to input core XML launch files to convert to flexible launch files',
    nargs='*'
)

parser.add_argument(
    '-d', '--dirs',
    help='Input Directories: path to directory that contains core XML launch files'
         ' to convert to flexible launch files',
    nargs='*'
)

args = parser.parse_args()

# Collect files
file_paths = []
if args.files is not None:
    for file in args.files:
        file_paths.append(os.path.abspath(file))

if args.dirs is not None:
    for dir in args.dirs:
        dir_path = os.path.abspath(dir)
        for item in os.listdir(dir_path):
            if 'core' in item:
                file_paths.append(dir_path + '/' + item)

if len(file_paths) == 0:
    parser.print_help()
    raise Exception('No files provided to convert.')

for in_file_path in file_paths:
    in_file_name = os.path.basename(in_file_path)

    in_file_name_components = in_file_name.split('.')

    if 'core' not in in_file_name_components:
        raise Exception(f'Input file {in_file_name} is not marked as a core launch file '
                        '(with \'.core.\' in the file name).')

    out_file_name_components = in_file_name_components.copy()
    out_file_name_components.pop(out_file_name_components.index('core'))

    out_file_name = '.'.join(out_file_name_components)

    out_file_path = os.path.abspath(args.out_dir + '/' + out_file_name)

    # Generate and install flexible launch xml
    generate_flexible_launch_xml(args.package_name, in_file_path, out_file_path)

    # Install core launch xml
    out_core_file_path = os.path.abspath(args.out_dir + '/' + in_file_name)
    shutil.copy2(in_file_path, out_core_file_path)