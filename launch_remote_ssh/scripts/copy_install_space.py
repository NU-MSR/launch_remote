#!/usr/bin/env python3

import sys
from launch_remote_ssh import copy_install_space

help_message =  'usage:' \
                '\nros2 run launch_remote_ssh copy_install_space.py' \
                ' USER MACHINE LOCAL_INSTALL_SPACE REMOTE_INSTALL_SPACE' \
                '\n\tUSER\t\t\tUser that will be used to log into the remote machine' \
                '\n\tMACHINE\t\t\tRemote machine to which install space will be copied' \
                '\n\tLOCAL_INSTALL_SPACE\tPath to install space on local machine' \
                '\n\tREMOTE_INSTALL_SPACE\tPath to install space on remote machine' \

if '--help' in sys.argv:
    print(help_message)
    exit()

if len(sys.argv) < 5:
    raise Exception("Too few arguments provided.\n" + help_message)

copy_install_space(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])